#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <time.h>

#include "audio_player.h"
#include "secrets.h"

namespace {

constexpr uint16_t kPiPort = 4040;
constexpr uint16_t kSyncPort = 4041;
constexpr uint32_t kReconnectIntervalMs = 2000;
constexpr uint32_t kWifiStatusIntervalMs = 2000;
constexpr uint32_t kAckTimeoutMs = 2000;
constexpr uint8_t kButtonPin = 4;
constexpr uint8_t kVolumeDownButtonPin = 0;
constexpr uint8_t kVolumeUpButtonPin = 1;
constexpr uint8_t kStatusLedPin = 10;
constexpr uint32_t kButtonDebounceMs = 30;
constexpr uint32_t kSyncBlinkPeriodMs = 2000;
constexpr uint32_t kSyncBlinkOnMs = 150;
constexpr uint32_t kSuccessBlinkMs = 120;
constexpr bool kAudioTestOnly = false;
constexpr uint64_t kAudioPreparationLeadUs = 25000;

enum class RaceState {
  kIdle,
  kWaitingForClock,
  kReady,
  kWaitingForAck,
  kArmed,
  kCancelled,
};

enum class SyncLedMode {
  kOff,
  kSyncing,
  kSuccess,
};

WiFiClient piClient;
WiFiUDP syncUdp;
uint32_t lastConnectAttemptMs = 0;
uint32_t lastWifiStatusMs = 0;
uint32_t ackDeadlineMs = 0;
uint32_t lastButtonChangeMs = 0;
uint32_t lastVolumeDownChangeMs = 0;
uint32_t lastVolumeUpChangeMs = 0;
RaceState raceState = RaceState::kIdle;
volatile bool mainButtonPressPending = false;
bool lastVolumeDownReading = HIGH;
bool volumeDownState = HIGH;
bool lastVolumeUpReading = HIGH;
bool volumeUpState = HIGH;
bool wifiWasConnected = false;
bool syncUdpStarted = false;
int64_t utcOffsetNs = 0;
uint64_t t0UtcNs = 0;
uint64_t t0GunMonotonicUs = 0;
uint64_t setGunMonotonicUs = 0;
uint64_t marksGunMonotonicUs = 0;
uint8_t nextAudioEvent = 0;
SyncLedMode syncLedMode = SyncLedMode::kOff;
bool statusLedOn = false;
uint8_t successLedTransitionsRemaining = 0;
uint32_t nextStatusLedChangeMs = 0;

void setStatusLed(bool on) {
  statusLedOn = on;
  digitalWrite(kStatusLedPin, on ? HIGH : LOW);
}

void startSyncLed() {
  if (syncLedMode == SyncLedMode::kSyncing) {
    return;
  }
  syncLedMode = SyncLedMode::kSyncing;
  setStatusLed(true);
  nextStatusLedChangeMs = millis() + kSyncBlinkOnMs;
}

void showSyncSuccess() {
  syncLedMode = SyncLedMode::kSuccess;
  successLedTransitionsRemaining = 5;
  setStatusLed(true);
  nextStatusLedChangeMs = millis() + kSuccessBlinkMs;
}

void updateStatusLed() {
  if (syncLedMode == SyncLedMode::kOff ||
      static_cast<int32_t>(millis() - nextStatusLedChangeMs) < 0) {
    return;
  }

  if (syncLedMode == SyncLedMode::kSyncing) {
    setStatusLed(!statusLedOn);
    nextStatusLedChangeMs =
        millis() + (statusLedOn ? kSyncBlinkOnMs
                               : kSyncBlinkPeriodMs - kSyncBlinkOnMs);
    return;
  }

  setStatusLed(!statusLedOn);
  if (--successLedTransitionsRemaining == 0) {
    syncLedMode = SyncLedMode::kOff;
    setStatusLed(false);
    return;
  }
  nextStatusLedChangeMs = millis() + kSuccessBlinkMs;
}

void IRAM_ATTR captureMainButtonPress() {
  mainButtonPressPending = true;
}

int64_t adjustedUtcNowNs() {
  return esp_timer_get_time() * 1000LL + utcOffsetNs;
}

void formatUtc(int64_t timestampNs, char* output, size_t outputSize) {
  const time_t seconds = timestampNs / 1000000000LL;
  const long microseconds = (timestampNs % 1000000000LL) / 1000;
  struct tm utcTime;
  gmtime_r(&seconds, &utcTime);

  char dateTime[32];
  strftime(dateTime, sizeof(dateTime), "%Y-%m-%dT%H:%M:%S", &utcTime);
  snprintf(output, outputSize, "%s.%06ldZ", dateTime, microseconds);
}

void cancelRace(const char* reason) {
  raceState = RaceState::kCancelled;
  Serial.printf("Race cancelled: %s\n", reason);
}

void returnToReady(const char* reason) {
  raceState = RaceState::kReady;
  t0UtcNs = 0;
  t0GunMonotonicUs = 0;
  setGunMonotonicUs = 0;
  marksGunMonotonicUs = 0;
  nextAudioEvent = 0;
  Serial.printf("%s; ready for another T0\n", reason);
}

void updateWifi() {
  const bool connected = WiFi.status() == WL_CONNECTED;

  if (connected && !wifiWasConnected) {
    wifiWasConnected = true;
    Serial.printf("Wi-Fi connected: %s\n", WiFi.localIP().toString().c_str());

    if (!syncUdpStarted) {
      syncUdpStarted = syncUdp.begin(kSyncPort);
      Serial.printf("Clock sync listening on UDP port %u\n", kSyncPort);
    }
    return;
  }

  if (!connected && wifiWasConnected) {
    wifiWasConnected = false;
    piClient.stop();
    syncUdp.stop();
    syncUdpStarted = false;
    Serial.println("Wi-Fi disconnected; retrying in background");

    if (raceState == RaceState::kWaitingForClock ||
        raceState == RaceState::kWaitingForAck ||
        raceState == RaceState::kArmed) {
      cancelRace("Wi-Fi disconnected");
    }
  }

  const uint32_t now = millis();
  if (!connected && now - lastWifiStatusMs >= kWifiStatusIntervalMs) {
    lastWifiStatusMs = now;
    Serial.println("Waiting for Wi-Fi...");
  }
}

void scheduleT0() {
  const uint32_t t0DelaySeconds = 30 + (esp_random() % 11);
  const uint32_t setLeadMs = 1500 + (esp_random() % 2501);
  const uint32_t marksLeadSeconds = 15 + (esp_random() % 6);
  const uint64_t randomDelayUs = t0DelaySeconds * 1000000ULL;
  t0UtcNs = adjustedUtcNowNs() + randomDelayUs * 1000ULL;
  t0GunMonotonicUs = (t0UtcNs - utcOffsetNs + 999) / 1000;
  setGunMonotonicUs = t0GunMonotonicUs - setLeadMs * 1000ULL;
  marksGunMonotonicUs =
      setGunMonotonicUs - marksLeadSeconds * 1000000ULL;
  nextAudioEvent = 0;

  char formattedT0[48];
  formatUtc(t0UtcNs, formattedT0, sizeof(formattedT0));
  Serial.printf("T0 selected: %s (%llu ns)\n", formattedT0,
                static_cast<unsigned long long>(t0UtcNs));
  Serial.printf("Audio gaps: marks-to-set=%lu s, set-to-t0=%lu ms\n",
                static_cast<unsigned long>(marksLeadSeconds),
                static_cast<unsigned long>(setLeadMs));

  piClient.printf("T0 %llu\n", static_cast<unsigned long long>(t0UtcNs));
  ackDeadlineMs = millis() + kAckTimeoutMs;
  raceState = RaceState::kWaitingForAck;
}

void beginNewT0(bool replacingT0) {
  t0UtcNs = 0;
  t0GunMonotonicUs = 0;
  setGunMonotonicUs = 0;
  marksGunMonotonicUs = 0;
  nextAudioEvent = 0;
  if (replacingT0) {
    Serial.println("Replacing the scheduled T0");
  }
  scheduleT0();
}

void handlePiMessage(const char* message, int64_t receivedMonotonicNs) {
  Serial.printf("Pi: %s\n", message);

  if (strncmp(message, "CLOCK_SYNC ", 11) == 0 &&
      raceState == RaceState::kWaitingForClock) {
    unsigned long long estimatedUtcNs = 0;
    unsigned long long oneWayUs = 0;
    if (sscanf(message, "CLOCK_SYNC %llu %llu", &estimatedUtcNs, &oneWayUs) != 2) {
      cancelRace("invalid clock reference");
      return;
    }

    utcOffsetNs = static_cast<int64_t>(estimatedUtcNs) - receivedMonotonicNs;
    Serial.printf("Clock calibrated: one_way_us=%llu utc_offset_ns=%lld\n",
                  oneWayUs, static_cast<long long>(utcOffsetNs));
    raceState = RaceState::kReady;
    piClient.println("CLOCK_SYNCED");
    showSyncSuccess();
    Serial.println("Clock synchronized; press the button to set T0");
    return;
  }

  if (strcmp(message, "REQUEST_T0") == 0) {
    if (raceState != RaceState::kReady) {
      piClient.println("START_REJECT busy");
      return;
    }
    Serial.println("Pi requested a new T0");
    beginNewT0(false);
    return;
  }

  if (strncmp(message, "ACK_T0 ", 7) == 0 &&
      raceState == RaceState::kWaitingForAck) {
    unsigned long long acknowledgedT0 = 0;
    if (sscanf(message, "ACK_T0 %llu", &acknowledgedT0) != 1 ||
        acknowledgedT0 != t0UtcNs) {
      cancelRace("invalid T0 acknowledgement");
      return;
    }

    if (esp_timer_get_time() >= static_cast<int64_t>(t0GunMonotonicUs)) {
      cancelRace("T0 acknowledgement arrived too late");
      return;
    }

    raceState = RaceState::kArmed;
    showSyncSuccess();
    Serial.println("T0 acknowledged; gun armed");
    return;
  }

  if (strncmp(message, "REJECT_T0 ", 10) == 0 &&
      raceState == RaceState::kWaitingForAck) {
    returnToReady(message + 10);
    return;
  }

  if (strncmp(message, "CANCEL ", 7) == 0) {
    cancelRace(message + 7);
  }
}

void connectToPi() {
  if (WiFi.status() != WL_CONNECTED || piClient.connected()) {
    return;
  }

  startSyncLed();

  const uint32_t now = millis();
  if (now - lastConnectAttemptMs < kReconnectIntervalMs) {
    return;
  }
  lastConnectAttemptMs = now;

  const IPAddress piAddress = WiFi.gatewayIP();
  Serial.printf("Connecting to Pi at %s:%u...\n",
                piAddress.toString().c_str(), kPiPort);

  if (!piClient.connect(piAddress, kPiPort)) {
    Serial.println("Pi connection failed");
    return;
  }

  piClient.setNoDelay(true);
  raceState = RaceState::kWaitingForClock;
  piClient.printf("HELLO TZeroHeroGun %s\n", WiFi.macAddress().c_str());
  Serial.println("Connected to Pi; starting clock synchronization");
}

void readPiMessages() {
  static char message[128];
  static size_t length = 0;

  while (piClient.available()) {
    const char value = static_cast<char>(piClient.read());

    if (value == '\n') {
      message[length] = '\0';
      const int64_t receivedMonotonicNs = esp_timer_get_time() * 1000LL;
      handlePiMessage(message, receivedMonotonicNs);
      length = 0;
    } else if (value != '\r' && length < sizeof(message) - 1) {
      message[length++] = value;
    }
  }
}

uint64_t playScheduledAudio(const char* label, const char* path,
                            uint64_t targetUs);

void updateRaceState() {
  if (raceState == RaceState::kWaitingForAck &&
      static_cast<int32_t>(millis() - ackDeadlineMs) >= 0) {
    returnToReady("T0 acknowledgement timeout");
    return;
  }

  if (raceState != RaceState::kArmed) {
    return;
  }

  const char* labels[] = {"on your marks", "set", "t0 / gun"};
  const char* paths[] = {"/onYourMarks.wav", "/getSet.wav", "/gun.wav"};
  const uint64_t targets[] = {
      marksGunMonotonicUs, setGunMonotonicUs, t0GunMonotonicUs};

  if (nextAudioEvent >= 3 ||
      esp_timer_get_time() + kAudioPreparationLeadUs < targets[nextAudioEvent]) {
    return;
  }

  const uint8_t event = nextAudioEvent++;
  const uint64_t playbackStartUs =
      playScheduledAudio(labels[event], paths[event], targets[event]);
  if (playbackStartUs == 0) {
    cancelRace("audio playback failed");
    return;
  }
  if (event != 2) {
    return;
  }

  const int64_t actualUtcNs =
      static_cast<int64_t>(playbackStartUs) * 1000LL + utcOffsetNs;
  const int64_t errorUs = (actualUtcNs - static_cast<int64_t>(t0UtcNs)) / 1000;
  char actualUtc[48];
  formatUtc(actualUtcNs, actualUtc, sizeof(actualUtc));

  Serial.printf("GUN!! utc=%s (%lld ns) error_us=%lld\n", actualUtc,
                static_cast<long long>(actualUtcNs),
                static_cast<long long>(errorUs));
  returnToReady("Race fired");
}

uint32_t randomSeconds(uint32_t minimum, uint32_t maximum) {
  return minimum + (esp_random() % (maximum - minimum + 1));
}

void waitUntil(uint64_t targetUs) {
  while (true) {
    const uint64_t nowUs = esp_timer_get_time();
    if (nowUs >= targetUs) {
      return;
    }

    const uint64_t remainingUs = targetUs - nowUs;
    if (remainingUs > 2000) {
      delay((remainingUs - 1000) / 1000);
    } else {
      delayMicroseconds(remainingUs);
    }
  }
}

uint64_t playScheduledAudio(const char* label, const char* path,
                            uint64_t targetUs) {
  const uint64_t preparationUs = targetUs > kAudioPreparationLeadUs
                                     ? targetUs - kAudioPreparationLeadUs
                                     : 0;
  waitUntil(preparationUs);
  Serial.printf("Preparing audio: %s target=%llu us\n", label,
                static_cast<unsigned long long>(targetUs));

  const uint64_t playbackStartUs = playWav(path, targetUs);
  if (playbackStartUs != 0) {
    Serial.printf("Audio first write: %s scheduled=%llu us actual=%llu us error=%lld us\n",
                  label,
                  static_cast<unsigned long long>(targetUs),
                  static_cast<unsigned long long>(playbackStartUs),
                  static_cast<long long>(playbackStartUs - targetUs));
  }
  return playbackStartUs;
}

void runAudioSequence() {
  const uint32_t t0DelaySeconds = randomSeconds(30, 40);
  const uint32_t setLeadMs = 1500 + (esp_random() % 2501);
  const uint32_t marksLeadSeconds = randomSeconds(15, 20);
  const uint64_t nowUs = esp_timer_get_time();
  const uint64_t t0Us = nowUs + t0DelaySeconds * 1000000ULL;
  const uint64_t setUs = t0Us - setLeadMs * 1000ULL;
  const uint64_t marksUs = setUs - marksLeadSeconds * 1000000ULL;
  const uint64_t setDelayMs = (setUs - nowUs) / 1000ULL;
  const uint64_t marksDelayMs = (marksUs - nowUs) / 1000ULL;

  Serial.println("Schedule selected (microseconds since boot):");
  Serial.printf("  now:           %llu us\n",
                static_cast<unsigned long long>(nowUs));
  Serial.printf("  on your marks: %llu us (+%llu ms)\n",
                static_cast<unsigned long long>(marksUs),
                static_cast<unsigned long long>(marksDelayMs));
  Serial.printf("  set:           %llu us (+%llu ms)\n",
                static_cast<unsigned long long>(setUs),
                static_cast<unsigned long long>(setDelayMs));
  Serial.printf("  t0 / gun:      %llu us (+%lu s)\n",
                static_cast<unsigned long long>(t0Us),
                static_cast<unsigned long>(t0DelaySeconds));
  Serial.printf("  gaps: marks-to-set=%lu s, set-to-t0=%lu ms\n",
                static_cast<unsigned long>(marksLeadSeconds),
                static_cast<unsigned long>(setLeadMs));

  playScheduledAudio("on your marks", "/onYourMarks.wav", marksUs);
  playScheduledAudio("set", "/getSet.wav", setUs);
  playScheduledAudio("t0 / gun", "/gun.wav", t0Us);
}

bool wasButtonPressed(uint8_t pin, bool& lastReading, bool& stableState,
                      uint32_t& lastChangeMs) {
  const bool reading = digitalRead(pin);
  const uint32_t now = millis();

  if (reading != lastReading) {
    lastReading = reading;
    lastChangeMs = now;
  }

  if (now - lastChangeMs < kButtonDebounceMs || reading == stableState) {
    return false;
  }

  stableState = reading;
  return stableState == LOW;
}

void previewVolumeChange(int deltaPercent) {
  const int volumePercent = adjustAudioVolume(deltaPercent);
  Serial.printf("Volume: %d%%\n", volumePercent);
  playWav("/gun.wav", esp_timer_get_time() + kAudioPreparationLeadUs);
}

void updateVolumeButtons() {
  if (wasButtonPressed(kVolumeUpButtonPin, lastVolumeUpReading,
                       volumeUpState, lastVolumeUpChangeMs)) {
    previewVolumeChange(5);
  }

  if (wasButtonPressed(kVolumeDownButtonPin, lastVolumeDownReading,
                       volumeDownState, lastVolumeDownChangeMs)) {
    previewVolumeChange(-5);
  }
}

void updateButton() {
  noInterrupts();
  const bool pressed = mainButtonPressPending;
  mainButtonPressPending = false;
  interrupts();

  if (!pressed) {
    return;
  }

  const uint32_t now = millis();
  if (now - lastButtonChangeMs < kButtonDebounceMs) {
    return;
  }
  lastButtonChangeMs = now;

  Serial.println("Pressed!");

  if (kAudioTestOnly) {
    runAudioSequence();
    return;
  }

  if (!piClient.connected()) {
    Serial.println("Cannot start: Pi is not connected");
    return;
  }

  if (raceState != RaceState::kReady && raceState != RaceState::kArmed) {
    Serial.println("Cannot set T0: gun is not synchronized and ready");
    return;
  }

  const bool replacingT0 = raceState == RaceState::kArmed;

  beginNewT0(replacingT0);
}


void handleSyncRequests() {
  const int packetSize = syncUdp.parsePacket();
  if (packetSize <= 0) {
    return;
  }

  const uint64_t gunReceiveUs = esp_timer_get_time();
  const IPAddress remoteAddress = syncUdp.remoteIP();
  const uint16_t remotePort = syncUdp.remotePort();

  char request[96];
  const int length = syncUdp.read(request, sizeof(request) - 1);
  if (length <= 0) {
    return;
  }
  request[length] = '\0';

  unsigned long sequence = 0;
  unsigned long long piSendUs = 0;
  if (sscanf(request, "SYNC %lu %llu", &sequence, &piSendUs) != 2) {
    Serial.printf("Invalid sync request: %s\n", request);
    return;
  }

  syncUdp.beginPacket(remoteAddress, remotePort);
  syncUdp.printf("SYNC_REPLY %lu %llu %llu ", sequence, piSendUs,
                 static_cast<unsigned long long>(gunReceiveUs));
  const uint64_t gunSendUs = esp_timer_get_time();
  syncUdp.printf("%llu\n", static_cast<unsigned long long>(gunSendUs));
  syncUdp.endPacket();
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(kStatusLedPin, OUTPUT);
  setStatusLed(false);

  pinMode(kButtonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(kButtonPin), captureMainButtonPress,
                  FALLING);
  pinMode(kVolumeDownButtonPin, INPUT_PULLUP);
  pinMode(kVolumeUpButtonPin, INPUT_PULLUP);
  lastVolumeDownReading = digitalRead(kVolumeDownButtonPin);
  volumeDownState = lastVolumeDownReading;
  lastVolumeUpReading = digitalRead(kVolumeUpButtonPin);
  volumeUpState = lastVolumeUpReading;
  Serial.println("Button ready on GPIO4");

  if (setupAudio()) {
    Serial.println("Audio ready");
    Serial.printf("Volume controls ready: GPIO0 down, GPIO1 up (current: %d%%)\n",
                  getAudioVolumePercent());
  } else {
    Serial.println("Audio setup failed");
  }

  if (kAudioTestOnly) {
    Serial.println("Audio sequence ready; press the button to schedule T0");
    return;
  }

  startSyncLed();

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.printf("Connecting to Wi-Fi %s in background\n", WIFI_SSID);
}

void loop() {
  updateStatusLed();
  updateVolumeButtons();
  updateButton();

  if (kAudioTestOnly) {
    delay(1);
    return;
  }

  updateWifi();
  connectToPi();
  if (syncUdpStarted) {
    handleSyncRequests();
  }

  if (piClient.connected()) {
    readPiMessages();
  }

  updateRaceState();

  delay(1);
}
