#pragma once
#include <stdint.h>

bool setupAudio();
int adjustAudioVolume(int deltaPercent);
int getAudioVolumePercent();
void playSilence(uint32_t durationMs);
uint64_t playWav(const char* path, uint64_t targetUs);
