#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <time.h>

#include "secrets.h"

namespace {

constexpr uint16_t kPiPort = 4040;
constexpr uint16_t kSyncPort = 4041;
constexpr uint32_t kReconnectIntervalMs = 2000;
constexpr uint32_t kHeartbeatIntervalMs = 1000;
constexpr uint32_t kWifiStatusIntervalMs = 2000;
constexpr uint32_t kAckTimeoutMs = 2000;
constexpr uint8_t kButtonPin = 4;
constexpr uint32_t kButtonDebounceMs = 30;

enum class RaceState {
  kIdle,
  kWaitingForClock,
  kWaitingForAck,
  kArmed,
  kFired,
  kCancelled,
};

WiFiClient piClient;
WiFiUDP syncUdp;
uint32_t lastConnectAttemptMs = 0;
uint32_t lastHeartbeatMs = 0;
uint32_t lastWifiStatusMs = 0;
uint32_t ackDeadlineMs = 0;
uint32_t lastButtonChangeMs = 0;
RaceState raceState = RaceState::kIdle;
bool lastButtonReading = HIGH;
bool buttonState = HIGH;
bool wifiWasConnected = false;
bool syncUdpStarted = false;
int64_t utcOffsetNs = 0;
uint64_t t0UtcNs = 0;
uint64_t t0GunMonotonicUs = 0;

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
  const uint64_t randomDelayUs = 7000000ULL + (esp_random() % 5000001ULL);
  t0UtcNs = adjustedUtcNowNs() + randomDelayUs * 1000ULL;
  t0GunMonotonicUs = (t0UtcNs - utcOffsetNs + 999) / 1000;

  char formattedT0[48];
  formatUtc(t0UtcNs, formattedT0, sizeof(formattedT0));
  Serial.printf("T0 selected: %s (%llu ns)\n", formattedT0,
                static_cast<unsigned long long>(t0UtcNs));

  piClient.printf("T0 %llu\n", static_cast<unsigned long long>(t0UtcNs));
  ackDeadlineMs = millis() + kAckTimeoutMs;
  raceState = RaceState::kWaitingForAck;
}

void handlePiMessage(const char* message, int64_t receivedMonotonicNs) {
  Serial.printf("Pi: %s\n", message);

  if (strcmp(message, "PI_READY") == 0 &&
      raceState == RaceState::kWaitingForClock) {
    piClient.println("GUN_READY");
    Serial.println("Gun ready; waiting for clock synchronization");
    return;
  }

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
    scheduleT0();
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
    Serial.println("T0 acknowledged; gun armed");
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
  Serial.println("Connected to Pi; press the button to begin");
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

void updateRaceState() {
  if (raceState == RaceState::kWaitingForAck &&
      static_cast<int32_t>(millis() - ackDeadlineMs) >= 0) {
    cancelRace("T0 acknowledgement timeout");
    return;
  }

  if (raceState != RaceState::kArmed ||
      esp_timer_get_time() < static_cast<int64_t>(t0GunMonotonicUs)) {
    return;
  }

  const int64_t actualUtcNs = adjustedUtcNowNs();
  const int64_t errorUs = (actualUtcNs - static_cast<int64_t>(t0UtcNs)) / 1000;
  char actualUtc[48];
  formatUtc(actualUtcNs, actualUtc, sizeof(actualUtc));

  Serial.printf("GUN!! utc=%s (%lld ns) error_us=%lld\n", actualUtc,
                static_cast<long long>(actualUtcNs),
                static_cast<long long>(errorUs));
  raceState = RaceState::kFired;
}

void updateButton() {
  const bool reading = digitalRead(kButtonPin);
  const uint32_t now = millis();

  if (reading != lastButtonReading) {
    lastButtonChangeMs = now;
    lastButtonReading = reading;
  }

  if (now - lastButtonChangeMs < kButtonDebounceMs || reading == buttonState) {
    return;
  }

  buttonState = reading;
  if (buttonState == LOW) {
    Serial.println("Pressed!");

    if (!piClient.connected()) {
      Serial.println("Cannot start: Pi is not connected");
      return;
    }

    if (raceState == RaceState::kWaitingForClock ||
        raceState == RaceState::kWaitingForAck ||
        raceState == RaceState::kArmed) {
      Serial.println("Cannot start: sprint sequence already running");
      return;
    }

    t0UtcNs = 0;
    t0GunMonotonicUs = 0;
    raceState = RaceState::kWaitingForClock;
    piClient.printf("HELLO TZeroHeroGun %s\n", WiFi.macAddress().c_str());
    Serial.println("Gun ready; starting communication with Pi");
  }
}

void sendHeartbeat() {
  if (!piClient.connected()) {
    return;
  }

  const uint32_t now = millis();
  if (now - lastHeartbeatMs < kHeartbeatIntervalMs) {
    return;
  }
  lastHeartbeatMs = now;

  piClient.printf("PING %llu\n",
                  static_cast<unsigned long long>(esp_timer_get_time()));
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

  pinMode(kButtonPin, INPUT_PULLUP);
  lastButtonReading = digitalRead(kButtonPin);
  buttonState = lastButtonReading;
  Serial.println("Button ready on GPIO4");

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.printf("Connecting to Wi-Fi %s in background\n", WIFI_SSID);
}

void loop() {
  updateWifi();
  updateButton();
  connectToPi();
  if (syncUdpStarted) {
    handleSyncRequests();
  }

  if (piClient.connected()) {
    readPiMessages();
    sendHeartbeat();
  }

  updateRaceState();

  delay(1);
}
