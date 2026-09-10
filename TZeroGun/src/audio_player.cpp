#include "audio_player.h"

#include <Arduino.h>
#include <LittleFS.h>
#include <driver/i2s.h>

namespace {

constexpr i2s_port_t kI2SPort = I2S_NUM_0;
constexpr int kI2SBclkPin = 5;
constexpr int kI2SLrcPin = 6;
constexpr int kI2SDataPin = 7;
constexpr int kAmplifierShutdownPin = 3;
constexpr uint32_t kAudioSampleRate = 22050;
constexpr float kVolume = 0.50f;

bool audioReady = false;

uint16_t readLittleEndian16(const uint8_t* value) {
  return static_cast<uint16_t>(value[0]) |
         (static_cast<uint16_t>(value[1]) << 8);
}

uint32_t readLittleEndian32(const uint8_t* value) {
  return static_cast<uint32_t>(value[0]) |
         (static_cast<uint32_t>(value[1]) << 8) |
         (static_cast<uint32_t>(value[2]) << 16) |
         (static_cast<uint32_t>(value[3]) << 24);
}

bool findWavData(File& file, uint32_t& dataSize) {
  uint8_t riffHeader[12];
  if (file.read(riffHeader, sizeof(riffHeader)) != sizeof(riffHeader) ||
      memcmp(riffHeader, "RIFF", 4) != 0 ||
      memcmp(riffHeader + 8, "WAVE", 4) != 0) {
    return false;
  }

  bool validFormat = false;
  while (file.available()) {
    uint8_t chunkHeader[8];
    if (file.read(chunkHeader, sizeof(chunkHeader)) != sizeof(chunkHeader)) {
      return false;
    }

    const uint32_t chunkSize = readLittleEndian32(chunkHeader + 4);
    if (memcmp(chunkHeader, "fmt ", 4) == 0) {
      uint8_t format[16];
      if (chunkSize < sizeof(format) ||
          file.read(format, sizeof(format)) != sizeof(format)) {
        return false;
      }

      validFormat = readLittleEndian16(format) == 1 &&
                    readLittleEndian16(format + 2) == 1 &&
                    readLittleEndian32(format + 4) == kAudioSampleRate &&
                    readLittleEndian16(format + 14) == 16;
      file.seek(file.position() + chunkSize - sizeof(format));
    } else if (memcmp(chunkHeader, "data", 4) == 0) {
      dataSize = chunkSize;
      return validFormat;
    } else {
      file.seek(file.position() + chunkSize);
    }

    if (chunkSize & 1) {
      file.seek(file.position() + 1);
    }
  }

  return false;
}

void waitUntilAudioStart(uint64_t targetUs) {
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

}  // namespace

bool setupAudio() {
  pinMode(kAmplifierShutdownPin, OUTPUT);
  digitalWrite(kAmplifierShutdownPin, LOW);

  if (!LittleFS.begin(false)) {
    Serial.println("LittleFS mount failed; upload the data filesystem");
    return false;
  }

  i2s_config_t config = {};
  config.mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_TX);
  config.sample_rate = kAudioSampleRate;
  config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  config.dma_buf_count = 8;
  config.dma_buf_len = 256;
  config.use_apll = false;
  config.tx_desc_auto_clear = true;
  config.fixed_mclk = 0;

  i2s_pin_config_t pins = {};
  pins.mck_io_num = I2S_PIN_NO_CHANGE;
  pins.bck_io_num = kI2SBclkPin;
  pins.ws_io_num = kI2SLrcPin;
  pins.data_out_num = kI2SDataPin;
  pins.data_in_num = I2S_PIN_NO_CHANGE;

  audioReady =
      i2s_driver_install(kI2SPort, &config, 0, nullptr) == ESP_OK &&
      i2s_set_pin(kI2SPort, &pins) == ESP_OK;
  return audioReady;
}

void playSilence(uint32_t durationMs) {
  if (!audioReady) {
    Serial.println("Audio is not ready");
    return;
  }

  int16_t silence[512] = {};
  i2s_zero_dma_buffer(kI2SPort);
  digitalWrite(kAmplifierShutdownPin, HIGH);
  delay(5);

  Serial.printf("Sending digital silence for %lu ms\n",
                static_cast<unsigned long>(durationMs));
  const uint32_t startedAt = millis();
  while (millis() - startedAt < durationMs) {
    size_t bytesWritten = 0;
    i2s_write(kI2SPort, silence, sizeof(silence),
              &bytesWritten, portMAX_DELAY);
  }

  delay(20);
  i2s_zero_dma_buffer(kI2SPort);
  delay(2);
  digitalWrite(kAmplifierShutdownPin, LOW);
  Serial.println("Silence test complete");
}

uint64_t playWav(const char* path, uint64_t targetUs) {
  if (!audioReady) {
    Serial.println("Audio is not ready");
    return 0;
  }

  File file = LittleFS.open(path, "r");
  uint32_t remaining = 0;
  if (!file || !findWavData(file, remaining)) {
    Serial.printf("Cannot play %s: invalid or missing WAV\n", path);
    return 0;
  }

  i2s_zero_dma_buffer(kI2SPort);
  digitalWrite(kAmplifierShutdownPin, HIGH);
  delay(5);

  Serial.printf("Prepared %s at %.0f%% volume\n", path, kVolume * 100.0f);
  int16_t mono[256];
  int16_t stereo[512];
  uint64_t firstWriteUs = 0;

  while (remaining > 0) {
    const size_t wanted = min(static_cast<size_t>(remaining), sizeof(mono));
    const size_t bytesRead =
        file.read(reinterpret_cast<uint8_t*>(mono), wanted);
    if (bytesRead == 0) {
      break;
    }

    const size_t samples = bytesRead / sizeof(int16_t);
    for (size_t index = 0; index < samples; ++index) {
      const int16_t scaled = static_cast<int16_t>(mono[index] * kVolume);
      stereo[index * 2] = scaled;
      stereo[index * 2 + 1] = scaled;
    }

    size_t bytesWritten = 0;
    if (firstWriteUs == 0) {
      waitUntilAudioStart(targetUs);
      firstWriteUs = esp_timer_get_time();
    }
    i2s_write(kI2SPort, stereo, samples * 2 * sizeof(int16_t),
              &bytesWritten, portMAX_DELAY);
    remaining -= bytesRead;
  }

  delay(20);
  i2s_zero_dma_buffer(kI2SPort);
  delay(2);
  digitalWrite(kAmplifierShutdownPin, LOW);
  file.close();
  Serial.println("Playback complete");
  return firstWriteUs;
}
