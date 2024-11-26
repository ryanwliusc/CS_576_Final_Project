
#include <filesystem>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h>

using namespace std;
namespace fs = std::filesystem;

/** Declarations*/
/** Utility function to read each frame of the video and output compressed information */
void readVideoData(string videoPath, int width, int height, ofstream &outputFile);

/** DCT encoding function */
void dctEncode(int width, int height, double ***rblocks, double ***gblocks, double ***bblocks, unsigned char *dctData);

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        cerr << "The executable should be invoked with exactly one filepath "
                "argument. Example ./MyEncoder '../video/rgbs/SAL.rgb' n1 n2"
             << endl;
        exit(1);
    }
    int width = 960;
    int height = 540;
    string videoPath = argv[1];
    int n1 = atoi(argv[2]);
    int n2 = atoi(argv[3]);
    string outputPath = "input_video.cmp";
    ofstream outputFile(outputPath, ios::binary);
    if (!outputFile.is_open())
    {
        cerr << "Error Opening File for Writing" << endl;
        exit(1);
    }
    outputFile << n1 << " " << n2 << "\n";
    readVideoData(videoPath, width, height, outputFile);
    cout << "terminated" << endl;
    return 0;
}

/** Utility function to read frames and output blocktype and DCT coefficients */
void readVideoData(string videoPath, int width, int height, ofstream &outputFile)
{

    // Open the file in binary mode
    ifstream inputFile(videoPath, ios::binary);

    if (!inputFile.is_open())
    {
        cerr << "Error Opening File for Reading" << endl;
        exit(1);
    }

    // Create and populate RGB buffers and data array
    vector<char> Rbuf(width * height);
    vector<char> Gbuf(width * height);
    vector<char> Bbuf(width * height);
    vector<unsigned char> prevFrame(width * height * 3);
    vector<unsigned char> currFrame(width * height * 3);


    // Loop and read 1 frame at a time until eof
    while (true)
    {
        Rbuf.clear();
        Gbuf.clear();
        Bbuf.clear();
        currFrame.clear();

        if (inputFile.eof()) {
            break;
        }
        inputFile.read(Rbuf.data(), width * height);
        inputFile.read(Gbuf.data(), width * height);
        inputFile.read(Bbuf.data(), width * height);


        for (int i = 0; i < height * width; i++)
        {
            // We populate RGB values of each pixel in that order
            // RGB.RGB.RGB and so on for all pixels
            currFrame[3 * i] = Rbuf[i];
            currFrame[3 * i + 1] = Gbuf[i];
            currFrame[3 * i + 2] = Bbuf[i];
        }
        //TODO: calculate motion vectors, determine if foreground or background, and apply DCT
        outputFile << "Blocktype " << "Coeffients" << "\n";
        prevFrame = currFrame;

    }
    inputFile.close();
    outputFile.close();
}

void dctEncode(int width, int height, double ***rblocks, double ***gblocks, double ***bblocks, unsigned char *dctData)
{
    int blockIndex = -1;
    // DCT encoding, get 8x8 block of original data for each rgb channel, DCT the block and put it back into the orig array
    for (int i = 0; i < height; i += 8)
    {
        for (int j = 0; j < width; j += 8)
        {
            blockIndex += 1;
            // Mapping to top Left pixel of 8x8 block
            int mapping = (i * width + j) * 3;
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
                    double c = .25 * cu * cv;
                    for (int x = 0; x < 8; x++)
                    {
                        for (int y = 0; y < 8; y++)
                        {
                            int index = mapping + (x * width * 3) + (y * 3);
                            rblocks[blockIndex][u][v] += (double)dctData[index] * cos(((2 * x + 1) * u * M_PI) / 16) * cos(((2 * y + 1) * v * M_PI) / 16);
                            gblocks[blockIndex][u][v] += (double)dctData[index + 1] * cos(((2 * x + 1) * u * M_PI) / 16) * cos(((2 * y + 1) * v * M_PI) / 16);
                            bblocks[blockIndex][u][v] += (double)dctData[index + 2] * cos(((2 * x + 1) * u * M_PI) / 16) * cos(((2 * y + 1) * v * M_PI) / 16);
                        }
                    }
                    rblocks[blockIndex][u][v] *= c;
                    gblocks[blockIndex][u][v] *= c;
                    bblocks[blockIndex][u][v] *= c;
                }
            }
        }
    }
}