#ifndef SPEECH_RECOGNIZER_H
#define SPEECH_RECOGNIZER_H

#include <string>
#include <vector>
#include <vosk_api.h>
#include <portaudio.h>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <espeak/speak_lib.h>
#include <fstream>
#include <unistd.h>


// 宏定义
#define SAMPLE_RATE 16000
#define FRAMES_PER_BUFFER 8000

// 全局变量声明
extern VoskModel *g_model;
extern VoskRecognizer *g_recognizer;
extern PaStream *g_stream;
extern std::vector<int16_t> g_buffer;

// 函数声明
bool speech_init(const std::string &model_path);
std::string speech_recognize();
bool speech_contains_phrase(const std::string &recognitionResult, const std::string &targetPhrase);
void speech_cleanup();

bool containsLiziSound(const std::string &input);

bool containsNaipiSound(const std::string &input);

void speech_flush_buffer();

bool isTextEmpty(const std::string& jsonResult);







#endif // SPEECH_RECOGNIZER_H