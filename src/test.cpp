#define _USE_MATH_DEFINES
#include <math.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <vector>
// Audio with miniaudio.h (instead) https://miniaud.io/
#define MINIAUDIO_IMPLEMENTATION
#include <stdio.h>

#include "miniaudio.h"
using namespace cv;
using namespace std;
namespace fs = std::filesystem;

ma_bool32 g_IsPaused = MA_FALSE;
ma_bool32 g_Restart = MA_FALSE;

int getAudioPos (int index, int fps){
    return static_cast<int>((index / fps) * 1000);
}

void data_callback(ma_device *pDevice, void *pOutput, const void *pInput, ma_uint32 frameCount) {
    ma_decoder *pDecoder = (ma_decoder *)pDevice->pUserData;
    if (pDecoder == NULL) {
        return;
    }
    //Not for looping
    //ma_decoder_read_pcm_frames(pDecoder, pOutput, frameCount, NULL);
    //Looping
    if (g_Restart) {
        ma_decoder_seek_to_pcm_frame(pDecoder, 0);
        g_Restart = MA_FALSE;
    }
    if (!g_IsPaused) {
        ma_data_source_read_pcm_frames(pDecoder, pOutput, frameCount, NULL);
    }

    (void)pInput;
}

int main(int argc, char **argv) {
    string videoPath = "../../../video/rgbs/WalkingStaticBackground.rgb";
    string audioPath = "../../../video/wavs/WalkingStaticBackground.wav";

    // Video Test
    int width = 960;
    int height = 540;

    // Audio Test
    ma_result result;
    ma_decoder decoder;
    ma_device_config deviceConfig;
    ma_device device;

    // Open the file in binary mode
    ifstream inputFile(videoPath, ios::binary);

    if (!inputFile.is_open()){
        cerr << "Error Opening File for Reading" << endl;
        exit(1);
    }

    result = ma_decoder_init_file(audioPath.c_str(), NULL, &decoder);
    if (result != MA_SUCCESS) {
        printf("Could not load file: %s\n", argv[1]);
        return -2;
    }

    //FOR LOOPING AUDIO
    ma_data_source_set_looping(&decoder, MA_TRUE);

    deviceConfig = ma_device_config_init(ma_device_type_playback);
    deviceConfig.playback.format = decoder.outputFormat;
    deviceConfig.playback.channels = decoder.outputChannels;
    deviceConfig.sampleRate = decoder.outputSampleRate;
    deviceConfig.dataCallback = data_callback;
    deviceConfig.pUserData = &decoder;
   //Initialize 
   if (ma_device_init(NULL, &deviceConfig, &device) != MA_SUCCESS) {
        printf("Failed to open playback device.\n");
        ma_decoder_uninit(&decoder);
        return -3;
    }
    vector<unsigned char> RGBStream;
    vector<unsigned char> rgbBuffer(width * height * 3);
    while (!inputFile.eof()){
        inputFile.read(reinterpret_cast<char *>(rgbBuffer.data()), width * height * 3);
        for (int i = 0; i < width*height*3; i++){
            RGBStream.push_back(rgbBuffer[i]);
        }
    }

    
    inputFile.close();
    // OpenCV is in BGR format
    int frameSize = width * height * 3;
    // Calculate total number of frames
    int totalFrames = RGBStream.size() / frameSize;
    int index = 0;
    bool paused = false;
    namedWindow("Video", WINDOW_NORMAL);
    resizeWindow("Video", width, height);
    setWindowProperty("Video", WND_PROP_TOPMOST, 1);
    int fps = 30;
    const size_t frame_length_ns = std::round(1000000000.0 / fps);
    const std::chrono::high_resolution_clock::duration duration =std::chrono::nanoseconds(frame_length_ns);
    const std::chrono::high_resolution_clock::time_point start_time =std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point next_frame_time_point =start_time + duration;

    // Audio Start (I think)
    if (ma_device_start(&device) != MA_SUCCESS) {
        printf("Failed to start playback device.\n");
        ma_device_uninit(&device);
        ma_decoder_uninit(&decoder);
        return -4;
    }

    // PlayBack Loop
    while (true) {
        const std::chrono::high_resolution_clock::time_point now =std::chrono::high_resolution_clock::now();
        const int milliseconds_to_wait =std::chrono::duration_cast<std::chrono::milliseconds>(next_frame_time_point - now).count();
        const uint8_t *frameData = &RGBStream[index * frameSize];
        Mat frame(height, width, CV_8UC3, const_cast<uint8_t *>(frameData));

        int audioPos = static_cast<int>((index / fps) * 1000);
        if (!paused) {
            // Video
            imshow("Video", frame);
            if (index < totalFrames) {
                index++;
            } else {
                index = 0;
            }
        }
        if (milliseconds_to_wait > 0) {
            int key = waitKey(milliseconds_to_wait);
            if (key == 27) {
                ma_decoder_uninit(&decoder);
                ma_device_uninit(&device);
                break;  // Press esc to exit
            } else if (key == 32) {
                paused = !paused;  // Press space to pause
                g_IsPaused = !g_IsPaused;
                cout << "Playback paused" << endl;
            } else if (paused && key == 'k') {
                // Step Forward
                index++;
                const uint8_t *frameData = &RGBStream[index * frameSize];
                Mat frame(height, width, CV_8UC3, const_cast<uint8_t *>(frameData));
                imshow("Video", frame);
            } else if (paused && key == 'j') {
                // Step Backwards
                if (index != 0) {
                    index--;
                }
                const uint8_t *frameData = &RGBStream[index * frameSize];
                Mat frame(height, width, CV_8UC3, const_cast<uint8_t *>(frameData));
                imshow("Video", frame);
            } else if (key == 'p') {
                // Play
                g_Restart = !g_Restart;
                index = 0;
                paused = false;
                g_IsPaused = false;
                const uint8_t *frameData = &RGBStream[index * frameSize];
                Mat frame(height, width, CV_8UC3,
                          const_cast<uint8_t *>(frameData));
                imshow("Video", frame);
            } else if (key == 's') {
                // Stop
                g_Restart = !g_Restart;
                g_IsPaused = !g_IsPaused;
                index = 0;
                paused = true;
                const uint8_t *frameData = &RGBStream[index * frameSize];
                Mat frame(height, width, CV_8UC3,
                          const_cast<uint8_t *>(frameData));
                imshow("Video", frame);
            }
            next_frame_time_point += duration;
            if (now > next_frame_time_point) {
                next_frame_time_point =
                    now + duration;  // we've fallen too far behind, reset the
                                     // time we need to show the next frame
            }
        }
    }

    // printf("Press Enter to quit...");
    // getchar();

    ma_device_uninit(&device);
    ma_decoder_uninit(&decoder);

    return 0;
}