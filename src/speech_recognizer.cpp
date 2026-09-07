#include "speech_recognizer.h"

// 全局变量定义
VoskModel *g_model = nullptr;
VoskRecognizer *g_recognizer = nullptr;
PaStream *g_stream = nullptr;
std::vector<int16_t> g_buffer(FRAMES_PER_BUFFER);

bool speech_init(const std::string &model_path)
{
    // 初始化Vosk
    g_model = vosk_model_new(model_path.c_str());
    if (!g_model)
    {
        std::cerr << "Failed to load Vosk model" << std::endl;
        return false;
    }
    g_recognizer = vosk_recognizer_new(g_model, SAMPLE_RATE);

    // 初始化PortAudio
    if (Pa_Initialize() != paNoError)
    {
        std::cerr << "PortAudio initialization failed" << std::endl;
        return false;
    }

    if (Pa_OpenDefaultStream(&g_stream, 1, 0, paInt16, SAMPLE_RATE,
                             FRAMES_PER_BUFFER, NULL, NULL) != paNoError)
    {
        std::cerr << "Failed to open audio stream" << std::endl;
        return false;
    }

    if (Pa_StartStream(g_stream) != paNoError)
    {
        std::cerr << "Failed to start audio stream" << std::endl;
        return false;
    }

    return true;
}

void speech_flush_buffer()
{
    if (!g_stream)
        return;

    Pa_StopStream(g_stream);
    Pa_StartStream(g_stream);

    std::vector<int16_t> dummy_buffer(FRAMES_PER_BUFFER);
    
    // 只尝试读取有限次数（比如10次）
    for (int i = 0; i < 2; i++) {
        PaError result = Pa_ReadStream(g_stream, dummy_buffer.data(), FRAMES_PER_BUFFER);
        if (result != paNoError) {
            break; // 如果读取失败就退出
        }
        std::cout << "丢弃帧 " << i << std::endl;
    }
    
    std::cout << "清空完成" << std::endl;
}

std::string speech_recognize()
{

    // std::string final_result;
    // bool has_data = false;

    // // 持续读取直到缓冲区为空
    // while (true)
    // {
    //     std::vector<int16_t> chunk(FRAMES_PER_BUFFER);
    //     PaError status = Pa_ReadStream(g_stream, chunk.data(), FRAMES_PER_BUFFER);

    //     if (status != paNoError)
    //         break; // 无更多数据

    //     // 分段提交给Vosk
    //     if (vosk_recognizer_accept_waveform_s(g_recognizer, chunk.data(), chunk.size()))
    //     {
    //         const char *result = vosk_recognizer_result(g_recognizer);
    //         if (result && !isTextEmpty(result))
    //         {                          // 检查是否为空
    //             final_result = result; // 覆盖为最新结果
    //             has_data = true;
    //         }
    //     }
    // }

    // return has_data ? final_result : "";
    //----------------------------------------------------------------------------------------------------------



    // std::string final_result;
    // std::vector<int16_t> audio_data; // 存储所有待处理音频

    // // 第一步：读取所有积压的音频数据
    // while (true) {
    //     std::vector<int16_t> chunk(FRAMES_PER_BUFFER);
    //     PaError status = Pa_ReadStream(g_stream, chunk.data(), FRAMES_PER_BUFFER);
        
    //     if (status != paNoError) break; // 无更多数据或出错
        
    //     audio_data.insert(audio_data.end(), chunk.begin(), chunk.end());
    // }

    // // 第二步：一次性提交给Vosk识别
    // if (!audio_data.empty()) {
    //     if (vosk_recognizer_accept_waveform_s(g_recognizer, audio_data.data(), audio_data.size())) {
    //         const char* result = vosk_recognizer_result(g_recognizer);
    //         if (result) final_result = result;
    //     }
    // }

    // return final_result;


    //----------------------------------------------------------------------------------------------------------

    if (Pa_ReadStream(g_stream, g_buffer.data(), FRAMES_PER_BUFFER) != paNoError)
    {
        // std::cout << "no vosk" << std::endl;
        return "";
    }

    if (vosk_recognizer_accept_waveform_s(g_recognizer,
                                          g_buffer.data(), g_buffer.size()) != 0)
    {
        const char *result = vosk_recognizer_result(g_recognizer);
        return result;
    }
    std::cout << "no vosk" << std::endl;
    return "";
}

////1. 清空现有缓冲区
// speech_flush_buffer();

// // 2. 等待1秒（积累新语音）
// std::this_thread::sleep_for(std::chrono::milliseconds(500));

// // 3. 读取1秒的音频数据（16000采样点）
// std::vector<int16_t> buffer(SAMPLE_RATE); // 1秒缓冲区
// if (Pa_ReadStream(g_stream, buffer.data(), SAMPLE_RATE) != paNoError)
// {
//     return "";
// }

// // 4. 语音识别
// if (vosk_recognizer_accept_waveform_s(g_recognizer, buffer.data(), buffer.size()))
// {
//     return vosk_recognizer_result(g_recognizer);
// }
// return "";

void speech_cleanup()
{
    if (g_stream)
    {
        Pa_StopStream(g_stream);
        Pa_CloseStream(g_stream);
    }
    Pa_Terminate();
    if (g_recognizer)
        vosk_recognizer_free(g_recognizer);
    if (g_model)
        vosk_model_free(g_model);
}

bool speech_contains_phrase(const std::string &recognitionResult, const std::string &targetPhrase)
{
    // 简单的JSON解析（实际项目中应该使用JSON库）
    size_t textPos = recognitionResult.find("\"text\" : \"");
    if (textPos == std::string::npos)
        return false;

    size_t endPos = recognitionResult.find("\"", textPos + 10);
    if (endPos == std::string::npos)
        return false;

    std::string recognizedText = recognitionResult.substr(textPos + 10, endPos - (textPos + 10));

    // 移除所有空格和标点符号
    recognizedText.erase(std::remove_if(recognizedText.begin(), recognizedText.end(),
                                        [](char c)
                                        { return isspace(c) || ispunct(c); }),
                         recognizedText.end());

    std::string cleanTarget = targetPhrase;
    cleanTarget.erase(std::remove_if(cleanTarget.begin(), cleanTarget.end(),
                                     [](char c)
                                     { return isspace(c) || ispunct(c); }),
                      cleanTarget.end());

    // 检查目标短语是否按顺序出现在识别文本中
    size_t pos = 0;
    for (char c : cleanTarget)
    {
        pos = recognizedText.find(c, pos);
        if (pos == std::string::npos)
            return false;
        pos++;
    }
    return true;
}

bool containsNaipiSound(const std::string &input)
{
    // 全面覆盖"奶皮"发音及相似发音的所有常见中文词汇
    const std::vector<std::string> soundVariations = {
        // 标准同音词
        "奶皮", "耐皮", "乃皮", "奈皮", "氖皮", "萘皮", "柰皮",

        // 近音词/方言变体
        "奶啤", "奶脾", "奶疲", "奶匹", "奶劈", "奶癖", "奶屁",
        "耐啤", "耐脾", "耐疲", "耐匹", "耐劈", "耐癖", "耐屁",
        "奈啤", "奈脾", "奈疲", "奈匹", "奈劈", "奈癖", "奈屁",

        // 食品相关
        "奶皮子", "奶皮卷", "酥奶皮", "烤奶皮", "奶皮饼",

        // 拼音/网络用语
        "naipi", "Naipi", "NAIPI", "nǎipí", "nai pi",
        "奶p", "n皮", "nai皮",

        // 常见错误写法/谐音梗
        "乃啤", "氖啤", "萘啤", "柰啤",
        "奶批", "耐批", "奈批",
        "奶丕", "耐丕", "奈丕",

        // 两字分开但连续出现的情况
        "nai pi", "奶 pi", "nai 皮",

        // 包含在长词中
        "自制奶皮", "奶皮奶茶", "蒙古奶皮",
        "奶皮面膜", "奶皮起司"};

    // 转换为小写以不区分大小写
    std::string lowerInput = input;
    std::transform(lowerInput.begin(), lowerInput.end(), lowerInput.begin(), ::tolower);

    // 检查所有变体
    for (const auto &variant : soundVariations)
    {
        std::string lowerVariant = variant;
        std::transform(lowerVariant.begin(), lowerVariant.end(), lowerVariant.begin(), ::tolower);

        if (lowerInput.find(lowerVariant) != std::string::npos)
        {
            return true;
        }
    }

    return false;
}

bool containsLiziSound(const std::string &input)
{
    // 全面覆盖"lizi"发音及相似发音的所有常见中文词汇
    // 包括标准同音词、方言变体、常见误写、音译词等
    const std::vector<std::string> soundVariations = {
        // 标准同音词
        "粒子", "例子", "栗子", "李子", "离子", "利兹", "礼兹",
        "梨子", "狸子", "俚子", "厘子", "罹子", "骊子", "鲡子",

        // 近音词/方言变体
        "荔枝", "莉姿", "力子", "立子", "丽兹", "砺子", "詈子",
        "莅子", "吏子", "厉子", "疠子", "笠子", "粝子", "蛎子",

        // 特殊含义/网络用语
        "逆子", "妮子", "妮紫", // 某些方言/口音中的发音相似
        "哩子", "哩自",         // 口语化发音
        "lizi", "Lizi", "LIZI", // 拼音形式
        // 常见错误写法/谐音梗
        "粒紫", "栗紫", "李紫", "力紫",
        "栗渍", "梨渍", // 某些方言发音
        "里子",         // 某些情况下发音相似

        // 两字分开但连续出现的情况
        "li zi", "li 子", "李 z", "l 子",

        // 四字词中包含的情况
        "量子力学", "举个例子", "板栗子糖", // 只要包含"lizi"发音部分
        "离子烫发", "李子成熟"};

    // 转换为小写以不区分大小写
    std::string lowerInput = input;
    std::transform(lowerInput.begin(), lowerInput.end(), lowerInput.begin(), ::tolower);

    // 检查所有变体
    for (const auto &variant : soundVariations)
    {
        std::string lowerVariant = variant;
        std::transform(lowerVariant.begin(), lowerVariant.end(), lowerVariant.begin(), ::tolower);

        if (lowerInput.find(lowerVariant) != std::string::npos)
        {
            return true;
        }
    }

    return false;
}

bool isTextEmpty(const std::string &jsonResult)
{
    // 查找 "text" 字段的位置
    size_t textPos = jsonResult.find("\"text\"");
    if (textPos == std::string::npos)
    {
        return true; // 没有找到text字段
    }

    // 查找冒号后的第一个引号
    size_t quotePos = jsonResult.find('\"', textPos + 6); // 6是"text"的长度
    if (quotePos == std::string::npos || quotePos + 1 >= jsonResult.size())
    {
        return true;
    }

    // 检查下一个字符是否是闭合引号
    return jsonResult[quotePos + 1] == '\"';
}
