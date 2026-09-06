#pragma once
#include <stdint.h>

bool setupAudio();
void playSilence(uint32_t durationMs);
void playWav(const char* path);
