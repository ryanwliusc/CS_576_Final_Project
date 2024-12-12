#define _USE_MATH_DEFINES
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <atomic>
#include <condition_variable>
#include <stdio.h>
//Audio only works on Windows
#include <windows.h>
#include <Mmsystem.h>
#include <mciapi.h>
#pragma comment(lib, "winmm.lib")
//Audio with miniaudio.h (instead) https://miniaud.io/
#define MINIAUDIO_IMPLEMENTATION
#include "miniaudio.h"

using namespace cv;
using namespace std;
namespace fs = std::filesystem;

/** Declarations*/ 
// Global control flags for audio
ma_bool32 g_IsPaused = MA_FALSE;
ma_bool32 g_Restart = MA_FALSE;
ma_bool32 g_Stop = MA_FALSE; // Added for stopping audio playback

struct ControlState {
    bool *paused;
    int *index;
    int totalFrames;
    int width;
    ma_decoder *decoder;
    int fps;
};

//Multithread section
deque<vector<vector<double>>> redQ;
deque<vector<vector<double>>> greenQ;
deque<vector<vector<double>>> blueQ;
mutex redMut, greenMut, blueMut;
condition_variable cvRed, cvGreen, cvBlue;
atomic<bool> dataExists(true);

vector<vector<double>> cosTableU;
vector<vector<double>> cosTableV;

void outputIDCTBlock(vector<vector<double>> &ogBlock, vector<vector<double>> cosTableU, vector<vector<double>> cosTableV, vector<vector<double>> &outBlock);
//Function for CosineTables
vector<vector<double>> outputCosineTableV(int sizeY, int sizeX);
vector<vector<double>> outputCosineTableU(int sizeY, int sizeX);
vector<unsigned char> to1D(vector<vector<double>> &output2D);
vector<unsigned char> transferInData(vector<unsigned char> red, vector<unsigned char> green, vector<unsigned char> blue);
void readDataThread(ifstream &inputFile, double n, double nn);
void redThread(vector<vector<double>> &red, int width);
void greenThread(vector<vector<double>> &green, int width);
void blueThread(vector<vector<double>> &blue, int width);

void data_callback(ma_device* pDevice, void* pOutput, const void* pInput, ma_uint32 frameCount);
void createButtons(Mat &playerPanel, int width);
bool isButtonClicked(Point click, Rect button);
void onMouse(int event, int x, int y, int flags, void *userdata);

int main(int argc, char **argv)
{
  if (argc != 3)
  {
    cerr << "The executable should be invoked with exactly two filepath "
            "arguments. Example ./MyDecoder 'input_video/cmp' input_audio.wav"
         << endl;
    exit(1);
  }
  int width = 960;
  int height = 536;
  string videoPath = argv[1];
  string audioPath = argv[2];
  // Open the file in binary mode
  ifstream inputFile(videoPath, ios::binary);

  if (!inputFile.is_open())
  {
    cerr << "Error Opening File for Reading" << endl;
    exit(1);
  }

  //Create Cosine Table (Having a cosine table might be faster, might not be, not sure) (What I did for assignment 3 -Colbert)
  cosTableU = outputCosineTableU(8,8);
  cosTableV = outputCosineTableV(8,8);
  cout << "Finished cosine table creation" << endl;

  //Parse first line for n1, n2
  int n1;
  int n2;
  inputFile >> n1 >> n2; //Parsed n1 and n2

  vector<vector<double>> redStream2D;
  vector<vector<double>> greenStream2D;
  vector<vector<double>> blueStream2D;
  double n = pow(2, n1);
  double nn = pow(2, n2);

  thread fileRead(readDataThread, ref(inputFile), n, nn);
  thread rThread(redThread, ref(redStream2D), width);
  thread gThread(greenThread, ref(greenStream2D), width);
  thread bThread(blueThread, ref(blueStream2D), width);
  

  if(rThread.joinable()){
    cout << "red good" << endl;
  }
  if(gThread.joinable()){
    cout << "green good" << endl;
  }
  if(bThread.joinable()){
    cout << "blue good" << endl;
  }

  fileRead.join();
  rThread.join();
  gThread.join();
  bThread.join();

  cout << "Parse done" << endl;
  inputFile.close();
  vector<unsigned char> redStream;
  vector<unsigned char> greenStream;
  vector<unsigned char> blueStream;
  redStream = to1D(redStream2D);
  greenStream = to1D(greenStream2D);
  blueStream = to1D(blueStream2D);
  cout<<"to 1D done" << endl;
  
  vector<unsigned char> RGBStream = transferInData(redStream, greenStream, blueStream);
  cout << "RGBStream size = " << RGBStream.size() << endl;
  
  //OpenCV is in BGR format
  const size_t frameSize = width * height * 3;
  // Calculate total number of frames
  const int totalFrames = RGBStream.size() / frameSize;
  // Check if the input data size is valid
  int added = 0;
  if (RGBStream.size() % frameSize != 0) {
      cout << "Error: RGB stream size is not divisible by frame size!" << endl;
      while (!(RGBStream.size() % frameSize == 0)){
        //RGBStream.push_back(1);
        RGBStream.pop_back();
        added++;
        for (int i = 0; i < 63; i++){
          RGBStream.pop_back();
          added++;
        }
      }
  }
  //cout << "added: " << added << " more values to fill out 960x540" << endl;
  cout << "removed: " << added << " values to get to 960x536" << endl;

  /*Audio Stuff*/
  ma_result result;
  ma_decoder decoder;
  ma_device_config deviceConfig;
  ma_device device;

  result = ma_decoder_init_file(audioPath.c_str(), NULL, &decoder);
  if (result != MA_SUCCESS) {
        cout << "Could not load file" << endl;
        //return -1;
  }

  deviceConfig = ma_device_config_init(ma_device_type_playback);
  deviceConfig.playback.format   = decoder.outputFormat;      // Set to ma_format_unknown to use the device's native format.
  deviceConfig.playback.channels = decoder.outputChannels;    // Set to 0 to use the device's native channel count.
  deviceConfig.sampleRate        = decoder.outputSampleRate;  // Set to 0 to use the device's native sample rate.
  deviceConfig.dataCallback      = data_callback;   // This function will be called when miniaudio needs more data.
  deviceConfig.pUserData         = &decoder;        // Can be accessed from the device object (device.pUserData).

  ma_data_source_set_looping(&decoder, MA_TRUE);

  if (ma_device_init(NULL, &deviceConfig, &device) != MA_SUCCESS) {
    cout << "Failed to open audio device.\n";
    ma_decoder_uninit(&decoder);
    //return -3;
  }
  //Start Audio
  if (ma_device_start(&device) != MA_SUCCESS) {
    cout << "Failed to start audio device.\n";
    ma_device_uninit(&device);
    ma_decoder_uninit(&decoder);
    //return -4;
  }

  //OpenCV Setup
  namedWindow("Player", WINDOW_NORMAL);
  resizeWindow("Player", width, height + 80);
  setWindowProperty("Player", WND_PROP_TOPMOST, 1);

  Mat playerPanel(height + 80, width, CV_8UC3, Scalar(0, 0, 0));

  int index = 0;
  bool paused = false;
  int fps = 30;

  const auto frameDuration = chrono::nanoseconds(static_cast<int>(1e9 / fps));
  auto nextFrameTime = chrono::high_resolution_clock::now() + frameDuration;

  ControlState state = {&paused, &index, totalFrames, width, &decoder, fps};
  setMouseCallback("Player", onMouse, &state);
  auto *stateP = static_cast<ControlState *>(&state);

  //PlayBack Loop
  while (true){
    Mat frame(height, width, CV_8UC3);
    if (!paused) {
      const uint8_t* frameData = &RGBStream[index * frameSize];
      Mat frame(height, width, CV_8UC3, const_cast<uint8_t*>(frameData));
      if (frame.empty()) {
        cerr << "Error: Frame data is invalid." << endl;
        break;
      }

      frame.copyTo(playerPanel(Rect(0, 0, width, height)));

      if (index < totalFrames) {
        index++;
      } else {
        //When video finishes, go back to start
        *(stateP->index) = 0;
        *(stateP->paused) = true;
        g_Restart = MA_TRUE;
        g_IsPaused = *(stateP->paused);
      }
    } else if (paused){
      const uint8_t* frameData = &RGBStream[index * frameSize];
      Mat frame(height, width, CV_8UC3, const_cast<uint8_t*>(frameData));
      frame.copyTo(playerPanel(Rect(0, 0, width, height)));
    }
    createButtons(playerPanel, width);
    imshow("Player", playerPanel);
      
    int key = waitKey(1);
    if (key == 27) {
      ma_decoder_uninit(&decoder);
      ma_device_uninit(&device);
      break;  // Press esc to exit
    } else if (key == 32){
      *(stateP->paused) = !*(stateP->paused);
      g_IsPaused = *(stateP->paused);
      cout << "Playback paused" << endl;
    } else if (paused && key == 'k'){
      //Step Forward
      *(stateP->index) = min(*(stateP->index) + 1, stateP->totalFrames - 1);
      ma_decoder_seek_to_pcm_frame(stateP->decoder, *(stateP->index) * stateP->decoder->outputSampleRate / stateP->fps);
    } else if (paused && key == 'j'){
      //Step Backwards
      *(stateP->index) = max(*(stateP->index) - 1, 0);
      ma_decoder_seek_to_pcm_frame(stateP->decoder, *(stateP->index) * stateP->decoder->outputSampleRate / stateP->fps);
    } else if(key == 'p'){
      //Play
      g_Restart = MA_TRUE;
      *(stateP->paused) = false;
      g_IsPaused = *(stateP->paused);
      *(stateP->index) = 0;
    } else if (key == 's'){
      //Stop
      *(stateP->index) = 0;
      *(stateP->paused) = true;
      g_Restart = MA_TRUE;
      g_IsPaused = *(stateP->paused);
    }
    if (chrono::high_resolution_clock::now() < nextFrameTime) {
      this_thread::sleep_until(nextFrameTime);
    }
    nextFrameTime += frameDuration;
  }

  //Clean up Audio and Video
  ma_device_uninit(&device);
  ma_decoder_uninit(&decoder);
  destroyAllWindows();
  return 0;
}


/**Function to output 8x8 IDCT block --Colbert**/ 
void outputIDCTBlock(vector<vector<double>> &ogBlock, vector<vector<double>> cosTableU, vector<vector<double>> cosTableV, vector<vector<double>> &outBlock) {
    //vector<vector<double>> block(8, vector<double>(8));
    // Do the equation
    double sum;
    double CU;
    double CV;
    for (int y = 0; y < 8; y++) {
        for (int x = 0; x < 8; x++) {
          sum = 0.0;
            for (int v = 0; v < 8; v++) {
                for (int u = 0; u < 8; u++) {
                    if (u == 0) {
                        CU = 1.0 / (sqrt(2.0));
                    } else {
                        CU = 1.0;
                    }
                    if (v == 0) {
                        CV = 1.0 / (sqrt(2.0));
                    } else {
                        CV = 1.0;
                    }
                    sum += CU * CV * ogBlock[v][u] * cosTableU[u][x] * cosTableV[v][y];
                }
            }
            outBlock[y][x] = static_cast<double> (clamp((sum * 0.25), 0.0, 255.0));
        }
    }
}
/**Cosine Table Function**/
vector<vector<double>> outputCosineTableV(int sizeY, int sizeX){
  vector<vector<double>> table(sizeY, vector<double>(sizeX));
  for (int v = 0; v < sizeY; v++) {
      for (int y = 0; y < sizeY; y++) {
          table[v][y] = cos((2.0 * y + 1.0) * v * M_PI / 16.0);
        }
      }
  return table;
}
vector<vector<double>> outputCosineTableU(int sizeY, int sizeX){
  vector<vector<double>> table(sizeY, vector<double>(sizeX));
    for (int u = 0; u < sizeX; u++) {
        for (int x = 0; x < sizeX; x++) {
          table[u][x] = cos((2.0 * x + 1.0) * u * M_PI / 16.0);
        }
      }
  return table;
}
vector<unsigned char> to1D(vector<vector<double>> &output2D){
  vector<unsigned char> buf;
  for (int i = 0; i < output2D.size(); i++){
    for (int j = 0; j < output2D[i].size(); j++){
      buf.push_back(static_cast<unsigned char>((output2D[i][j])));
    }
  }
  return buf;
}
vector<unsigned char> transferInData(vector<unsigned char> red, vector<unsigned char> green, vector<unsigned char> blue){
  vector<unsigned char> inData;
  for (int i = 0; i < red.size(); i++) {
    // We populate BGR values of each pixel in that order
    // BGR.BGR.BGR and so on for all pixels
    //OpenCV wants BGR
    inData.push_back(blue[i]);
    inData.push_back(green[i]);
    inData.push_back(red[i]);
  }
  return inData;
}
void readDataThread(ifstream &inputFile, double n, double nn){
  double bufVal;
  int color = 0; //r = 0 g = 1 b = 2
  int type = -1;
  vector<vector<double>> bufBlock(8, vector<double>(8));
  //vector<vector<double>> IDCTBlock(8, vector<double>(8));
  while(inputFile >> bufVal){
    //First value of each line is blocktype (foreground/background)
    type = static_cast<int> (bufVal);
    for (int i = 0; i < 8; i++){
      for (int j = 0; j < 8; j++){
        //Parse line (64 more values), Dequantize then add to block
        inputFile >> bufVal;
        if (type == 0){
          //iframe
          bufVal *= n;
        } else if(type == 1){
          //foreground
          bufVal *= n;
        } else {
          //background
          bufVal *= nn;
        }
        bufBlock[i][j] = bufVal;
      }
    }
    //outputIDCTBlock(bufBlock, cosTableU, cosTableV, IDCTBlock);
    if (color == 0){
      {
        unique_lock<mutex> lock(redMut);
        redQ.push_back(bufBlock);
        lock.unlock();
      }
      cvRed.notify_one();
    } else if (color == 1){
      {
       unique_lock<mutex> lock(greenMut);
        greenQ.push_back(bufBlock);
        lock.unlock();
      }
      cvGreen.notify_one();
    } else if (color == 2){
      {
        unique_lock<mutex> lock(blueMut);
        blueQ.push_back(bufBlock);
        lock.unlock();
      }
      cvBlue.notify_one();
    }
    if (color < 2){
      color++;
    } else {
      color = 0;
    }
  }
  dataExists = false;
  cvRed.notify_one();
  cvGreen.notify_one();
  cvBlue.notify_one();
}
void redThread(vector<vector<double>> &red, int width){
  this_thread::sleep_for(chrono::milliseconds(1000));
  int offsetY = 0;
  int offsetX = 0;
  int test = 0;
  bool nextRow = true;
  vector<vector<double>> IDCTBlock(8, vector<double>(8));
  while(true){
    {
    unique_lock<mutex> lock(redMut);
    cvRed.wait(lock, [&] { return !redQ.empty() || !dataExists;});
    if (redQ.empty() && !dataExists){
      break;
    } 
    vector<vector<double>> &bufBlock = redQ.front();
    outputIDCTBlock(bufBlock, cosTableU, cosTableV, IDCTBlock);
    for (int i = 0; i < 8; i++){
      if(nextRow){
        red.push_back(vector<double>());
      }
      for (int j = 0; j < 8; j++){
        red[i+offsetY].push_back(IDCTBlock[i][j]);
      }
    }
    redQ.pop_front();
    }
    offsetX += 8;
    //cout << "Consumed: Red" << " ---" << test << endl;
    if (offsetX >= width){
      offsetX = 0;
      offsetY += 7;
      nextRow = true;
    } else {
      nextRow = false;
    }
    if (test % 80000 == 0){
      cout << "Red Check: " << offsetY << endl;
    }
    test++;
  }
}
void greenThread(vector<vector<double>> &green, int width){
  this_thread::sleep_for(chrono::milliseconds(1000));
  int offsetY = 0;
  int offsetX = 0;
  int test = 0;
  bool nextRow = true;
  vector<vector<double>> IDCTBlock(8, vector<double>(8));
  while(true){
    {
    unique_lock<mutex> lock(greenMut);
    cvGreen.wait(lock, [&] { return !greenQ.empty() || !dataExists;});
    if (greenQ.empty() && !dataExists){
      break;
    } 
    vector<vector<double>> &bufBlock = greenQ.front();
    outputIDCTBlock(bufBlock, cosTableU, cosTableV, IDCTBlock);
    for (int i = 0; i < 8; i++){
      if(nextRow){
        green.push_back(vector<double>());
      }
      for (int j = 0; j < 8; j++){
        green[i+offsetY].push_back(IDCTBlock[i][j]);
      }
    }
    greenQ.pop_front();
    }
    offsetX += 8;
    //cout << "Consumed: Green" << " ---" << test << endl;
    if (offsetX >= width){
      offsetX = 0;
      offsetY += 7;
      nextRow = true;
    } else {
      nextRow = false;
    }
    if (test % 80000 == 0){
      cout << "Green Check: " << offsetY << endl;
    }
    test++;
  }
}
void blueThread(vector<vector<double>> &blue, int width){
  this_thread::sleep_for(chrono::milliseconds(1000));
  int offsetY = 0;
  int offsetX = 0;
  bool nextRow = true;
  int test = 0;
  vector<vector<double>> IDCTBlock(8, vector<double>(8));
  while(true){
    {
    unique_lock<mutex> lock(blueMut);
    cvBlue.wait(lock, [&] { return !blueQ.empty() || !dataExists;});
    if (blueQ.empty() && !dataExists){
      break;
    }
    vector<vector<double>>  &bufBlock = blueQ.front();
    outputIDCTBlock(bufBlock, cosTableU, cosTableV, IDCTBlock);
    for (int i = 0; i < 8; i++){
      if(nextRow){
        blue.push_back(vector<double>());
      }
      for (int j = 0; j < 8; j++){
        blue[i+offsetY].push_back(IDCTBlock[i][j]);
      }
    }
    blueQ.pop_front();
    }
    offsetX += 8;
    //cout << "Consumed: Blue" << " ---" << test << endl;
    if (offsetX >= width){
      offsetX = 0;
      offsetY += 7;
      nextRow = true;
    } else {
      nextRow = false;
    }
    if (test % 80000 == 0){
      cout << "Blue Check: " << offsetY << endl;
    }
    test++;
  }
}
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
    rectangle(playerPanel, Rect(centerX - 170, 560, 120, 30), Scalar(150, 150, 150), FILLED);
    putText(playerPanel, "Play/Pause", Point(centerX - 155, 580), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);

    rectangle(playerPanel, Rect(width - 150, 560, 120, 30), Scalar(150, 150, 150), FILLED);
    putText(playerPanel, "Step Forward", Point(width - 140, 580), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);

    rectangle(playerPanel, Rect(50, 560, 120, 30), Scalar(150, 150, 150), FILLED);
    putText(playerPanel, "Step Back", Point(70, 580), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);

    rectangle(playerPanel, Rect(centerX + 50, 560, 120, 30), Scalar(150, 150, 150), FILLED);
    putText(playerPanel, "Stop", Point(centerX + 95, 580), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
}
bool isButtonClicked(Point click, Rect button) {
    return button.contains(click);
}
void onMouse(int event, int x, int y, int flags, void *userdata) {
    if (event != EVENT_LBUTTONDOWN) return;

    auto *state = static_cast<ControlState *>(userdata);
    Point click(x, y);

    int centerX = state->width / 2;
    Rect playPauseButton(centerX - 170, 560, 120, 30);
    Rect stepForwardButton(state->width - 150, 560, 120, 30);
    Rect stepBackButton(50, 560, 120, 30);
    Rect stopButton(centerX + 50, 560, 120, 30);

    if (isButtonClicked(click, playPauseButton)) {
        *(state->paused) = !*(state->paused);
        g_IsPaused = *(state->paused);
    } else if (isButtonClicked(click, stepForwardButton) && *(state->paused)) {
        *(state->index) = min(*(state->index) + 1, state->totalFrames - 1);
        ma_decoder_seek_to_pcm_frame(state->decoder, *(state->index) * state->decoder->outputSampleRate / state->fps);
    } else if (isButtonClicked(click, stepBackButton) && *(state->paused)) {
        *(state->index) = max(*(state->index) - 1, 0);
        ma_decoder_seek_to_pcm_frame(state->decoder, *(state->index) * state->decoder->outputSampleRate / state->fps);
    } else if (isButtonClicked(click, stopButton)){
        *(state->index) = 0;
        *(state->paused) = true;
        g_Restart = MA_TRUE;
        g_IsPaused = *(state->paused);
    }
}