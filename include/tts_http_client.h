#ifndef TTS_HTTP_CLIENT_H
#define TTS_HTTP_CLIENT_H

#include <string>

namespace tts
{
    struct TtsResult
    {
        bool ok = false;
        int http_status = -1;
        std::string content_type;
        std::string output_path;
        std::string error_message;
        size_t audio_bytes = 0;
    };

    // 发送 POST {base_url}/tts，JSON body: {"text":"..."}，并将音频保存到 output_path。
    // 仅支持 http:// 协议（与原脚本默认一致）。
    TtsResult SynthesizeToFile(const std::string &base_url,
                               const std::string &text,
                               const std::string &output_path,
                               int timeout_sec = 30);

    // 便捷接口：传任意文本即可，自动保存到 /tmp 并尝试播放。
    // base_url 默认 http://127.0.0.1:4990
    TtsResult SpeakText(const std::string &text,
                        const std::string &base_url = "http://127.0.0.1:4990",
                        int timeout_sec = 30);

    // 同步：播报；仅打印成功/失败及原因（scene 用于日志前缀，如「欢迎语」）
    void SpeakTextWithLog(const std::string &text,
                          const std::string &base_url = "http://127.0.0.1:4990",
                          int timeout_sec = 30,
                          const char *scene = nullptr);

    // 异步：独立线程播报，不阻塞调用方
    void SpeakTextAsync(std::string text,
                        const std::string &base_url = "http://127.0.0.1:4990",
                        int timeout_sec = 30,
                        const char *scene = nullptr);

    // 售卖现场固定话术（实现与文案在 tts_http_client.cpp）
    void SpeakWelcomeSceneAsync();
    void SpeakAfterOrderAsync();
    void SpeakCupTakenAsync();
    void SpeakDeliveryDoneAsync();
}

#endif
