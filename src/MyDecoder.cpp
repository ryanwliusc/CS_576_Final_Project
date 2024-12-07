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

using namespace cv;
using namespace std;
namespace fs = std::filesystem;

/** Declarations*/
//Multithread section
queue<vector<vector<double>>> redQ;
queue<vector<vector<double>>> greenQ;
queue<vector<vector<double>>> blueQ;
mutex mutexQ;
condition_variable cvRed, cvGreen, cvBlue;
atomic<bool> dataExists(true);

vector<vector<double>> cosTableU;
vector<vector<double>> cosTableV;

/** IDCT function */
void dctDecode(int width, int height, double ***rblocks, double ***gblocks, double ***bblocks, unsigned char *dctData);
vector<vector<int>> outputIDCTBlock(vector<vector<int>> ogBlock, vector<vector<double>> cosTableU, vector<vector<double>> cosTableV);
//Function for CosineTables
vector<vector<double>> outputCosineTableV(int sizeY, int sizeX);
vector<vector<double>> outputCosineTableU(int sizeY, int sizeX);
vector<unsigned char> to1D(vector<vector<double>> output2D);
vector<unsigned char> transferInData(vector<unsigned char> red, vector<unsigned char> green, vector<unsigned char> blue);
void readDataThread(ifstream &inputFile, double n, double nn);
void redThread(vector<vector<double>> &red, int width);
void greenThread(vector<vector<double>> &green, int width);
void blueThread(vector<vector<double>> &blue, int width);

int main(int argc, char **argv)
{
  if (argc != 3)
  {
    cerr << "The executable should be invoked with exactly one filepath "
            "argument. Example ./MyDecoder 'input_video/cmp' input_audio.wav"
         << endl;
    exit(1);
  }
  int width = 960;
  int height = 540;
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
  vector<vector<double>> redStream2D(1, vector<double>(width));
  vector<vector<double>> greenStream2D(1, vector<double>(width));
  vector<vector<double>> blueStream2D(1, vector<double>(width));
  double n = pow(2, n1);
  double nn = pow(2, n2);

  thread fileRead(readDataThread, ref(inputFile), n, nn);
  thread redThread(redThread, ref(redStream2D), width);
  thread greenThread(greenThread, ref(greenStream2D), width);
  thread blueThread(blueThread, ref(blueStream2D), width);

  if(redThread.joinable()){
    cout << "red good" << endl;
  }
  if(greenThread.joinable()){
    cout << "green good" << endl;
  }
  if(blueThread.joinable()){
    cout << "blue good" << endl;
  }

  fileRead.join();
  redThread.join();
  greenThread.join();
  blueThread.join();

  cout << "Parse done" << endl;
  //cout << "IDCT Done" << endl;
  inputFile.close();
  /*
  for(int i = 0; i < redStream2D.size(); i++){
    for (int j = 0; j < redStream2D[0].size(); j++){
       cout << "redStream2D[i][j]: " << redStream2D[i][j] << endl;
    }
  }*/
  vector<unsigned char> redStream;
  vector<unsigned char> greenStream;
  vector<unsigned char> blueStream;
  redStream = to1D(redStream2D);
  greenStream = to1D(greenStream2D);
  blueStream = to1D(blueStream2D);
  
  vector<unsigned char> RGBStream = transferInData(redStream, greenStream, blueStream);
  //testing RGBStream
  /*
  for (int i = 0; i < RGBStream.size(); i++){
   cout << RGBStream[i] << endl;
  }*/
  unsigned char* RGBp = &RGBStream[0];
  cout << "No crash poggers" << endl;
  
  //openCV testing
  //OpenCV is in BGR format
  Mat frame(height,width, CV_8UC3);
  Mat frameBGR;
  while(true) {
    //inputFile.read(reinterpret_cast<char*>(frame.data), width * height * 3);
    //frame = imdecode(RGBStream, IMREAD_COLOR);
    memcpy(frame.data, RGBStream.data(), width*height*3);
    cvtColor(frame, frameBGR, COLOR_RGB2BGR);
    imshow("Video", frameBGR);
    if (waitKey(1000 / 30) == 27){
      break; //Press esc to close video
    }
  }
  //#TODO (I think this is how to do it?)
  //Fill in RGB pixel data for each individual frame
  //Probably can process frames with openCV
  //openCV frames --> SDL maybe
  //Frame size --> 960 x 540 (Could be variable? Probably dont need to account for other resolutions)
  //Have an array of frames or something and read it one by one? (Not 100% how to do this)

  //#TODO Create video player using OpenCV that plays frames with audio
  //30 FPS, audio: 44.1 KHz
  //Stop Function
  //Pause Function
  //Play Function
  //Step Function (Step frame-by-frame)

  return 0;
}

void dctDecode(int width, int height, double ***rblocks, double ***gblocks, double ***bblocks, unsigned char *dctData)
{
  // DCT decoding, get 8x8 block of original data for each rgb channel, IDCT the block and put it back into the orig array
  int blockIndex = -1;
  for (int i = 0; i < height; i += 8)
  {
    for (int j = 0; j < width; j += 8)
    {
      blockIndex += 1;
      // Mapping to top Left pixel of 8x8 block
      int mapping = (i * width + j) * 3;

      double rblock[8][8] = {0};
      double gblock[8][8] = {0};
      double bblock[8][8] = {0};

      // apply IDCT using m coefficients
      for (int x = 0; x < 8; x++)
      {
        for (int y = 0; y < 8; y++)
        {
          for (int u = 0; u < 8; u++)
          {
            for (int v = 0; v < 8; v++)
            {
              double cu = 1, cv = 1;
              if (u == 0)
              {
                cu = 1 / sqrt(2);
              }
              if (v == 0)
              {
                cv = 1 / sqrt(2);
              }
              rblock[x][y] += cu * cv * rblocks[blockIndex][u][v] * cos(((2 * x + 1) * u * M_PI) / 16) * cos(((2 * y + 1) * v * M_PI) / 16);
              gblock[x][y] += cu * cv * gblocks[blockIndex][u][v] * cos(((2 * x + 1) * u * M_PI) / 16) * cos(((2 * y + 1) * v * M_PI) / 16);
              bblock[x][y] += cu * cv * bblocks[blockIndex][u][v] * cos(((2 * x + 1) * u * M_PI) / 16) * cos(((2 * y + 1) * v * M_PI) / 16);
            }
          }
          rblock[x][y] *= .25;
          gblock[x][y] *= .25;
          bblock[x][y] *= .25;
        }
      }
      // transfer decoded values back to dctData
      for (int u = 0; u < 8; u++)
      {
        for (int v = 0; v < 8; v++)
        {
          int pixel = mapping + (u * width * 3) + (v * 3);
          // accout for overflow and also we round to the nearest whole number based on decimal
          dctData[pixel] = min(max((int)(rblock[u][v] + 0.5), 0), 255);
          dctData[pixel + 1] = min(max((int)(gblock[u][v] + 0.5), 0), 255);
          dctData[pixel + 2] = min(max((int)(bblock[u][v] + 0.5), 0), 255);
        }
      }
    }
  }
}

/**Function to output 8x8 IDCT block --Colbert**/ 
vector<vector<int>> outputIDCTBlock(vector<vector<int>> ogBlock, vector<vector<double>> cosTableU, vector<vector<double>> cosTableV) {
    vector<vector<int>> block(8, vector<int>(8));
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
            block[y][x] = clamp((sum * 0.25), 0.0, 255.0);
        }
    }
    return block;
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
vector<unsigned char> to1D(vector<vector<double>> output2D){
  vector<unsigned char> buf;
  for (int i = 0; i < output2D.size(); i++){
    for (int j = 0; j < output2D[0].size(); j++){
      buf.push_back(static_cast<unsigned char>((output2D[i][j])));
    }
  }
  return buf;
}
vector<unsigned char> transferInData(vector<unsigned char> red, vector<unsigned char> green, vector<unsigned char> blue){
  vector<unsigned char> inData;
  for (int i = 0; i < red.size(); i++) {
    // We populate RGB values of each pixel in that order
    // RGB.RGB.RGB and so on for all pixels
    inData.push_back(red[i]);
    inData.push_back(green[i]);
    inData.push_back(blue[i]);
  }
  return inData;
}
void readDataThread(ifstream &inputFile, double n, double nn){
  int bufVal;
  int color = 0; //r = 0 g = 1 b = 2
  int type = -1;
  while(inputFile >> bufVal){
    //First value of each line is blocktype (foreground/background)
    vector<vector<double>> bufBlock(8, vector<double>(8));
    type = bufVal;
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
    //cout << "Produced: " << color << " ---" << endl;
    if (color == 0){
      {
        lock_guard<mutex> lock(mutexQ);
        redQ.push(bufBlock);
      }
      cvRed.notify_one();
    } else if (color == 1){
      {
        lock_guard<mutex> lock(mutexQ);
        greenQ.push(bufBlock);
      }
      cvGreen.notify_one();
    } else if (color == 2){
      {
        lock_guard<mutex> lock(mutexQ);
        blueQ.push(bufBlock);
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
}
void redThread(vector<vector<double>> &red, int width){
  this_thread::sleep_for(chrono::milliseconds(1000));
  int offsetY = 0;
  int offsetX = 0;
  vector<vector<double>> bufBlock(8, vector<double>(8));
  vector<double> newRow(width, 0);
  while(true){
    unique_lock<mutex> lock(mutexQ);
    cvRed.wait(lock, [&] { return !redQ.empty() || !dataExists;});
    if (redQ.empty() && !dataExists){
      break;
    }
    bufBlock = redQ.front();
    redQ.pop();
    //cout << "color: " << color << " ---" << endl;
    for (int i = 0; i < 8; i++){
      for (int j = 0; j < 8; j++){
        red[i + offsetY][j + offsetX] = bufBlock[i][j];
        //cout << red[i + offsetY][j + offsetX] << endl;
      }
      red.push_back(newRow);
    }
    offsetX += 8;
    //cout << "Consumed: Red" << " ---" << offsetY << endl;
    if (offsetX == width){
      //cout << "offSetX: " << offsetX << "  -------- offsetY:" << offsetY << endl;
      offsetX = 0;
      offsetY += 8;
    }
  }
}
void greenThread(vector<vector<double>> &green, int width){
  this_thread::sleep_for(chrono::milliseconds(1000));
  int offsetY = 0;
  int offsetX = 0;
  vector<vector<double>> bufBlock(8, vector<double>(8));
  vector<double> newRow(width, 0);
  while(true){
    unique_lock<mutex> lock(mutexQ);
    cvGreen.wait(lock, [&] { return !greenQ.empty() || !dataExists; });
    if (greenQ.empty() && !dataExists){
      break;
    }
    bufBlock = greenQ.front();
    greenQ.pop();
    //cout << "color: " << color << " ---" << endl;
    for (int i = 0; i < 8; i++){
      for (int j = 0; j < 8; j++){
        green[i + offsetY][j + offsetX] = bufBlock[i][j];
      }
      green.push_back(newRow);
    }
    offsetX += 8;
    //cout << "Consumed: Green" << " ---" << offsetY << endl;
    if (offsetX == width){
      //cout << "offSetX: " << offsetX << "  -------- offsetY:" << offsetY << endl;
      offsetX = 0;
      offsetY += 8;
    }
  }
}
void blueThread(vector<vector<double>> &blue, int width){
  this_thread::sleep_for(chrono::milliseconds(1000));
  int offsetY = 0;
  int offsetX = 0;
  vector<vector<double>> bufBlock(8, vector<double>(8));
  vector<double> newRow(width, 0);
  while(true){
    unique_lock<mutex> lock(mutexQ);
    cvBlue.wait(lock, [&] { return !blueQ.empty() || !dataExists;});
    if (blueQ.empty() && !dataExists){
      break;
    }
    bufBlock = blueQ.front();
    blueQ.pop();
    //cout << "color: " << color << " ---" << endl;
    for (int i = 0; i < 8; i++){
      for (int j = 0; j < 8; j++){
        blue[i + offsetY][j + offsetX] = bufBlock[i][j];
      }
      blue.push_back(newRow);
    }
    offsetX += 8;
    //cout << "Consumed: Blue" << " ---" << offsetY << endl;
    if (offsetX == width){
      //cout << "offSetX: " << offsetX << "  -------- offsetY:" << offsetY << endl;
      offsetX = 0;
      offsetY += 8;
    }
  }
}