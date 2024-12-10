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
//Audio only works on Windows
#include <windows.h>
#include <Mmsystem.h>
#include <mciapi.h>
#pragma comment(lib, "winmm.lib")

using namespace cv;
using namespace std;
namespace fs = std::filesystem;

/** Declarations*/
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
/*
  //openCV testing
  //OpenCV is in BGR format
  Mat frame(height,width, CV_8UC3);
  Mat frameBGR;
  while(true) {
    inputFile.read(reinterpret_cast<char*>(frame.data), width * height * 3);
    cvtColor(frame, frameBGR, COLOR_RGB2BGR);
     if (inputFile.eof()) {
          break;
      }
      imshow("Video", frameBGR);
      if (waitKey(1000 / 30) == 27){
        break; //Press esc to close video
      }
  }
  */
  //Create Cosine Table (Having a cosine table might be faster, might not be, not sure) (What I did for assignment 3 -Colbert)
  cosTableU = outputCosineTableU(8,8);
  cosTableV = outputCosineTableV(8,8);
  cout << "Finished cosine table creation" << endl;

  //Parse first line for n1, n2
  int n1;
  int n2;
  inputFile >> n1 >> n2; //Parsed n1 and n2
  //cout << "n1: "<< n1 << " n2: " << n2 << endl;

  //4 is there for overflow --> 544 % 8 = 0;
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
  //cout << "IDCT Done" << endl;
  inputFile.close();
  /*
  for(int i = 0; i < redStream2D.size(); i++){
    for (int j = 0; j < redStream2D[0].size(); j++){
       cout << "redStream2D["<<i<<"]["<<j<<"]: " << redStream2D[i][j] << endl;
    }
  }
  for(int i = 0; i < greenStream2D.size(); i++){
    for (int j = 0; j < greenStream2D[i].size(); j++){
       cout << "greenStream2D["<<i<<"]["<<j<<"]: " << greenStream2D[i][j] << endl;
    }
  }
  for(int i = 0; i < blueStream2D.size(); i++){
    for (int j = 0; j < blueStream2D[i].size(); j++){
       cout << "blueStream2D["<<i<<"]["<<j<<"]: " << blueStream2D[i][j] << endl;
    }
  }*/
  vector<unsigned char> redStream;
  vector<unsigned char> greenStream;
  vector<unsigned char> blueStream;
  redStream = to1D(redStream2D);
  greenStream = to1D(greenStream2D);
  blueStream = to1D(blueStream2D);
  cout<<"to 1D done" << endl;
  
  vector<unsigned char> RGBStream = transferInData(redStream, greenStream, blueStream);
  //testing RGBStream
  /*
  for (int i = 0; i < RGBStream.size(); i++){
   cout << RGBStream[i] << endl;
  }*/
  cout << "No crash poggers" << endl;
  cout << "RGBStream size = " << RGBStream.size() << endl;
  
  //openCV testing
  //OpenCV is in BGR format
  int frameSize = width * height * 3;
  // Calculate total number of frames
  int totalFrames = RGBStream.size() / frameSize;
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

  /*Audio Stuff https://stackoverflow.com/questions/22253074/how-to-play-or-open-mp3-or-wav-sound-file-in-c-program*/
  string audioAlias = "open " + audioPath + " type waveaudio alias audio";
  MCIERROR audioCheck = mciSendString(audioAlias.c_str(), NULL, 0, NULL);
  if (audioCheck != 0){
    cerr << "Error opening audio file" << endl;
  }

  int index = 0;
  bool paused = false;
  namedWindow("Video", WINDOW_NORMAL);
  resizeWindow("Video", width, height);
  setWindowProperty("Video", WND_PROP_TOPMOST, 1);
  /* To fix fps:
   Found on https://github.com/stephanecharette/MoveDetect/blob/master/src-test/main.cpp#L127-L139
   */
  int fps = 30;
  const size_t frame_length_ns	= std::round(1000000000.0 / fps);
  const std::chrono::high_resolution_clock::duration		duration				= std::chrono::nanoseconds(frame_length_ns);
  const std::chrono::high_resolution_clock::time_point	start_time				= std::chrono::high_resolution_clock::now();
  std::chrono::high_resolution_clock::time_point			next_frame_time_point	= start_time + duration;

  //PlayBack Loop
  TCHAR tch[100];
  mciSendString("play audio notify", NULL, 0, NULL);
  while (true){
    const std::chrono::high_resolution_clock::time_point now =std::chrono::high_resolution_clock::now();
    const int milliseconds_to_wait =std::chrono::duration_cast<std::chrono::milliseconds>(next_frame_time_point - now).count();
    const uint8_t* frameData = &RGBStream[index * frameSize];
    Mat frame(height, width, CV_8UC3, const_cast<uint8_t*>(frameData));
    
    int audioPos = static_cast<int>((index / fps) * 1000);
    if (!paused) {
      //Video
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
        mciSendString("stop audio", NULL, 0, NULL);
        mciSendString("close audio", NULL, 0, NULL);
        break;  // Press esc to exit
      } else if (key == 32){
        paused = !paused; //Press space to pause
        if (paused){
          mciSendString("pause audio", NULL, 0, NULL);
          //mciSendString("stop audio", NULL, 0, NULL);
        } else {
          audioPos = static_cast<int>((index / fps) * 1000);
          string seek = "seek audio to " + to_string(audioPos);
          mciSendString(seek.c_str(), NULL, 100, NULL);
          //mciSendString("play audio notify", NULL, 0, NULL);
          mciSendString("resume audio", NULL, 0, NULL);
        }
        cout << "Playback paused" << endl;
      } else if (paused && key == 'k'){
        //Step Forward
        const uint8_t *frameData = &RGBStream[index * frameSize];
        Mat frame(height, width, CV_8UC3, const_cast<uint8_t *>(frameData));
        imshow("Video", frame);

        //audioPos = static_cast<int>((index / fps) * 1000);
        //string seek = "seek audio to " + to_string(audioPos) + " ms";
        index++;
      } else if (paused && key == 'j'){
        //Step Backwards
        if (index != 0){
          index--;
        }
        const uint8_t *frameData = &RGBStream[index * frameSize];
        Mat frame(height, width, CV_8UC3, const_cast<uint8_t *>(frameData));
        imshow("Video", frame);

        //audioPos = static_cast<int>((index / fps) * 1000);
        //string seek = "seek audio to " + to_string(audioPos) + " ms"
      } else if(key == 'p'){
        //Play
        index = 0;
        paused = false;
        const uint8_t *frameData = &RGBStream[index * frameSize];
        Mat frame(height, width, CV_8UC3, const_cast<uint8_t *>(frameData));
        imshow("Video", frame);
        mciSendString("stop audio", NULL, 0, NULL);
        mciSendString("seek audio to start", NULL, 100, NULL);
        //mciSendString("resume audio", NULL, 0, NULL);
        mciSendString("play audio notify", NULL, 0, NULL);
      } else if (key == 's'){
        //Stop
        //mciSendString("stop audio", NULL, 0, NULL);
        //mciSendString("seek audio to start", NULL, 0, NULL);
        mciSendString("play audio notify", NULL, 0, NULL);
        mciSendString("pause audio", NULL, 0, NULL);
        index = 0;
        paused = true;
        const uint8_t *frameData = &RGBStream[index * frameSize];
        Mat frame(height, width, CV_8UC3, const_cast<uint8_t *>(frameData));
        imshow("Video", frame);
      }
      next_frame_time_point += duration;
      if (now > next_frame_time_point) {
        next_frame_time_point = now + duration; // we've fallen too far behind, reset the time we need to show the next frame
      } 
    }
  }
  destroyAllWindows();
  //#TODO Create video player using OpenCV that plays frames with audio
  //30 FPS, audio: 44.1 KHz
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
  vector<vector<double>> IDCTBlock(8, vector<double>(8));
  while(inputFile >> bufVal){
    //First value of each line is blocktype (foreground/background)
    type = static_cast<int> (bufVal);
    for (int i = 0; i < 8; i++){
      for (int j = 0; j < 8; j++){
        //Parse line (64 more values), Dequantize then add to block
        inputFile >> bufVal;
        if (type == 0){
          //iframe, not sure what to do here
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
    outputIDCTBlock(bufBlock, cosTableU, cosTableV, IDCTBlock);
    if (color == 0){
      {
        unique_lock<mutex> lock(redMut);
        redQ.push_back(IDCTBlock);
        lock.unlock();
      }
      cvRed.notify_one();
    } else if (color == 1){
      {
       unique_lock<mutex> lock(greenMut);
        greenQ.push_back(IDCTBlock);
        lock.unlock();
      }
      cvGreen.notify_one();
    } else if (color == 2){
      {
        unique_lock<mutex> lock(blueMut);
        blueQ.push_back(IDCTBlock);
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
  while(true){
    {
    unique_lock<mutex> lock(redMut);
    cvRed.wait(lock, [&] { return !redQ.empty() || !dataExists;});
    if (redQ.empty() && !dataExists){
      break;
    } 
    vector<vector<double>> &bufBlock = redQ.front();
    for (int i = 0; i < 8; i++){
      if(nextRow){
        red.push_back(vector<double>());
      }
      for (int j = 0; j < 8; j++){
        red[i+offsetY].push_back(bufBlock[i][j]);
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
  while(true){
    {
    unique_lock<mutex> lock(greenMut);
    cvGreen.wait(lock, [&] { return !greenQ.empty() || !dataExists;});
    if (greenQ.empty() && !dataExists){
      break;
    } 
    vector<vector<double>> &bufBlock = greenQ.front();
    for (int i = 0; i < 8; i++){
      if(nextRow){
        green.push_back(vector<double>());
      }
      for (int j = 0; j < 8; j++){
        green[i+offsetY].push_back(bufBlock[i][j]);
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
  while(true){
    {
    unique_lock<mutex> lock(blueMut);
    cvBlue.wait(lock, [&] { return !blueQ.empty() || !dataExists;});
    if (blueQ.empty() && !dataExists){
      break;
    }
    vector<vector<double>>  &bufBlock = blueQ.front();
    for (int i = 0; i < 8; i++){
      if(nextRow){
        blue.push_back(vector<double>());
      }
      for (int j = 0; j < 8; j++){
        blue[i+offsetY].push_back(bufBlock[i][j]);
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
