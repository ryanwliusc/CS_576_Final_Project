#include <opencv2/opencv.hpp>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h>

using namespace std;
namespace fs = std::filesystem;

/** Declarations*/

/** IDCT function */
void dctDecode(int width, int height, double ***rblocks, double ***gblocks, double ***bblocks, unsigned char *dctData);

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

  //#TODO: Decode the .cmp file and recreate the video
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
