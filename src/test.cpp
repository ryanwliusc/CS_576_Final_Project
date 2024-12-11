#define _USE_MATH_DEFINES
#include <cmath>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <algorithm>
#include <thread>
#include <vector>
#define MINIAUDIO_IMPLEMENTATION
#include "miniaudio.h"

using namespace cv;
using namespace std;
namespace fs = std::filesystem;

// Global control flags
ma_bool32 g_IsPaused = MA_FALSE;
ma_bool32 g_Restart = MA_FALSE;
ma_bool32 g_Stop = MA_FALSE; // Added for stopping audio playback

// Audio position helper
int getAudioPos(int index, int fps) {
    return static_cast<int>((index / fps) * 1000);
}

// Miniaudio callback for audio playback
void data_callback(ma_device *pDevice, void *pOutput, const void *pInput, ma_uint32 frameCount) {
    ma_decoder *pDecoder = (ma_decoder *)pDevice->pUserData;
    if (pDecoder == nullptr || g_Stop) return;

    if (g_Restart) {
        ma_decoder_seek_to_pcm_frame(pDecoder, 0);
        g_Restart = MA_FALSE;
    }

    if (!g_IsPaused) {
        ma_data_source_read_pcm_frames(pDecoder, pOutput, frameCount, nullptr);
    }

    (void)pInput;
}

void createButtons(Mat &playerPanel, int width) {
    int centerX = width / 2;
    rectangle(playerPanel, Rect(centerX - 50, 560, 100, 30), Scalar(150, 150, 150), FILLED);
    putText(playerPanel, "Play/Pause", Point(centerX - 45, 580), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);

    rectangle(playerPanel, Rect(width - 150, 560, 100, 30), Scalar(150, 150, 150), FILLED);
    putText(playerPanel, "Step Forward", Point(width - 145, 580), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);

    rectangle(playerPanel, Rect(50, 560, 100, 30), Scalar(150, 150, 150), FILLED);
    putText(playerPanel, "Step Back", Point(55, 580), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
}

bool isButtonClicked(Point click, Rect button) {
    return button.contains(click);
}

struct ControlState {
    bool *paused;
    int *index;
    int totalFrames;
    int width;
    ma_decoder *decoder;
    int fps;
};

void onMouse(int event, int x, int y, int flags, void *userdata) {
    if (event != EVENT_LBUTTONDOWN) return;

    auto *state = static_cast<ControlState *>(userdata);
    Point click(x, y);

    int centerX = state->width / 2;
    Rect playPauseButton(centerX - 50, 560, 100, 30);
    Rect stepForwardButton(state->width - 150, 560, 100, 30);
    Rect stepBackButton(50, 560, 100, 30);

    if (isButtonClicked(click, playPauseButton)) {
        *(state->paused) = !*(state->paused);
        g_IsPaused = *(state->paused);
    } else if (isButtonClicked(click, stepForwardButton) && *(state->paused)) {
        *(state->index) = min(*(state->index) + 1, state->totalFrames - 1);
        ma_decoder_seek_to_pcm_frame(state->decoder, *(state->index) * state->decoder->outputSampleRate / state->fps);
    } else if (isButtonClicked(click, stepBackButton) && *(state->paused)) {
        *(state->index) = max(*(state->index) - 1, 0);
        ma_decoder_seek_to_pcm_frame(state->decoder, *(state->index) * state->decoder->outputSampleRate / state->fps);
    }
}

int main(int argc, char **argv) {
    string videoPath = "E:/576_final_project_test_files/SAL.rgb";
    string audioPath = "E:/576_final_project_test_files/SAL.wav";

    // Video dimensions
    const int width = 960;
    const int height = 540;
    const int fps = 30;
    const size_t frameSize = width * height * 3;

    // Load video file
    ifstream inputFile(videoPath, ios::binary);
    if (!inputFile.is_open()) {
        cerr << "Error: Unable to open video file." << endl;
        return -1;
    }

    vector<unsigned char> RGBStream;
    vector<unsigned char> rgbBuffer(frameSize);

    while (inputFile.read(reinterpret_cast<char *>(rgbBuffer.data()), frameSize)) {
        RGBStream.insert(RGBStream.end(), rgbBuffer.begin(), rgbBuffer.end());
    }

    inputFile.close();

    if (RGBStream.empty()) {
        cerr << "Error: Video file is empty or failed to load." << endl;
        return -1;
    }

    const int totalFrames = RGBStream.size() / frameSize;

    // Load audio file
    ma_result result;
    ma_decoder decoder;
    result = ma_decoder_init_file(audioPath.c_str(), nullptr, &decoder);
    if (result != MA_SUCCESS) {
        cerr << "Error: Unable to initialize audio decoder." << endl;
        return -2;
    }

    ma_data_source_set_looping(&decoder, MA_TRUE);

    ma_device_config deviceConfig = ma_device_config_init(ma_device_type_playback);
    deviceConfig.playback.format = decoder.outputFormat;
    deviceConfig.playback.channels = decoder.outputChannels;
    deviceConfig.sampleRate = decoder.outputSampleRate;
    deviceConfig.dataCallback = data_callback;
    deviceConfig.pUserData = &decoder;

    ma_device device;
    if (ma_device_init(nullptr, &deviceConfig, &device) != MA_SUCCESS) {
        cerr << "Error: Failed to initialize audio device." << endl;
        ma_decoder_uninit(&decoder);
        return -3;
    }

    if (ma_device_start(&device) != MA_SUCCESS) {
        cerr << "Error: Failed to start audio device." << endl;
        ma_device_uninit(&device);
        ma_decoder_uninit(&decoder);
        return -4;
    }

    // OpenCV setup
    namedWindow("Player", WINDOW_NORMAL);
    resizeWindow("Player", width, height + 80);
    setWindowProperty("Player", WND_PROP_TOPMOST, 1);

    Mat playerPanel(height + 80, width, CV_8UC3, Scalar(0, 0, 0));

    int index = 0;
    bool paused = false;

    const auto frameDuration = chrono::nanoseconds(static_cast<int>(1e9 / fps));
    auto nextFrameTime = chrono::high_resolution_clock::now() + frameDuration;

    ControlState state = {&paused, &index, totalFrames, width, &decoder, fps};
    setMouseCallback("Player", onMouse, &state);

    while (true) {
        Mat videoFrame(height, width, CV_8UC3);

        if (!paused && index < totalFrames) {
            const uint8_t *frameData = &RGBStream[index * frameSize];
            Mat frame(height, width, CV_8UC3, const_cast<uint8_t *>(frameData));
            if (frame.empty()) {
                cerr << "Error: Frame data is invalid." << endl;
                break;
            }

            // Convert RGB to BGR for OpenCV
            cvtColor(frame, videoFrame, COLOR_RGB2BGR);
            videoFrame.copyTo(playerPanel(Rect(0, 0, width, height)));
            index++;
        } else if (paused) {
            const uint8_t *frameData = &RGBStream[index * frameSize];
            Mat frame(height, width, CV_8UC3, const_cast<uint8_t *>(frameData));
            cvtColor(frame, videoFrame, COLOR_RGB2BGR);
            videoFrame.copyTo(playerPanel(Rect(0, 0, width, height)));
        }

        if (index >= totalFrames) {
            g_Stop = MA_TRUE; // Stop audio when video finishes
            break;
        }

        createButtons(playerPanel, width);
        imshow("Player", playerPanel);

        int key = waitKey(1);
        if (key == 27) {  // Esc key to exit
            break;
        }

        if (chrono::high_resolution_clock::now() < nextFrameTime) {
            this_thread::sleep_until(nextFrameTime);
        }
        nextFrameTime += frameDuration;
    }

    // Cleanup
    ma_device_uninit(&device);
    ma_decoder_uninit(&decoder);

    return 0;
}
