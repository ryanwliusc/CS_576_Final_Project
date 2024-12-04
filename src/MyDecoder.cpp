#define _USE_MATH_DEFINES
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h>

using namespace cv;
using namespace std;
namespace fs = std::filesystem;

/** Declarations*/
vector<vector<double>> cosTableU;
vector<vector<double>> cosTableV;

/** IDCT function */
void dctDecode(int width, int height, double ***rblocks, double ***gblocks, double ***bblocks, unsigned char *dctData);
vector<vector<int>> outputIDCTBlock(vector<vector<int>> ogBlock, vector<vector<double>> cosTableU, vector<vector<double>> cosTableV);
//Function for CosineTables
vector<vector<double>> outputCosineTableV(int sizeY, int sizeX);
vector<vector<double>> outputCosineTableU(int sizeY, int sizeX);

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
        break;
      }
  }

  //Create Cosine Table (Having a cosine table might be faster, might not be, not sure) (What I did for assignment 3 -Colbert)
  cosTableU = outputCosineTableU(8,8);
  cosTableV = outputCosineTableV(8,8);
  cout << "Finished cosine table creation" << endl;

  //#TODO: Decode the .cmp file and recreate the video
  //Parse first line for n1, n2
  int n1;
  int n2;
  string firstLine;
  getline(inputFile, firstLine);
  stringstream ss(firstLine);
  ss >> n1 >> n2; //Parsed n1 and n2

  //Parse rest of lines:
  string line;
  vector<vector<int>> rBlock(8, vector<int>(8));
  vector<vector<int>> gBlock(8, vector<int>(8));
  vector<vector<int>> bBlock(8, vector<int>(8));
  vector<vector<int>> bufBlock(8, vector<int>(8));

  //IDCT //Kinda messy but I like separating R G B streams and doing things 3 times. //Feel free to change idk
  vector<vector<double>> IDCTRed(height, vector<double>(width));
  vector<vector<double>> IDCTGreen(height, vector<double>(width));
  vector<vector<double>> IDCTBlue(height, vector<double>(width));
  int bufVal;
  int color = 0; //r = 0 g = 1 b = 2
  int type = -1;
  int offsetY = 0;
  int offsetX = 0;

  int n = pow(2, n1);
  int nn = pow(2, n2);
  while(getline(inputFile, line)){
    stringstream ssline(line);
    //First value of each line is blocktype (foreground/background)
    //Every three values --> Check if they are equal, if not, something might be wrong (each rgb block triplet should be the same blocktype) but probs dont need to check
    /* probs dont need this actually
    if (!(type1 == type2 && type2 == type3)){
      cout << "probably something wrong with rgb block triplet type" << endl;
    }*/
    ssline >> bufVal;
    for (int i = 0; i < 8; i++){
      for (int j = 0; j < 8; j++){
        //Parse line (64 more values), Dequantize then add to block 
        ssline >> bufVal; 
        if (type == 0){
          //iframe, not sure what to do here
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
    switch(color){
      case 0:
        rBlock = outputIDCTBlock(bufBlock, cosTableU, cosTableV);
        for (int y = 0; y < 8; y++){
         for (int x = 0; x < 8; x++){
          IDCTRed[y + offsetY][x + offsetX] = rBlock[y][x];
          }
        }
        break;
      case 1:
        gBlock = outputIDCTBlock(bufBlock, cosTableU, cosTableV);
        for (int y = 0; y < 8; y++){
         for (int x = 0; x < 8; x++){
          IDCTGreen[y + offsetY][x + offsetX] = gBlock[y][x];
          }
        }
        break;
      case 2:
        bBlock = outputIDCTBlock(bufBlock, cosTableU, cosTableV);
        for (int y = 0; y < 8; y++){
         for (int x = 0; x < 8; x++){
          IDCTBlue[y + offsetY][x + offsetX] = bBlock[y][x];
          }
        }
        break;
      default:
        color = 0;
        offsetX += 8;
        offsetY += 8;
        if (offsetX >= width){
          offsetX = 0;
        }
    }
    color++;
  }
  inputFile.close();
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