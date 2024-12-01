#define _USE_MATH_DEFINES

#include <filesystem>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h>
#include <limits>
#include <cmath>
#include <map>
#include <queue>

using namespace std;
namespace fs = std::filesystem;

/** Declarations*/
/** Utility function to read each frame of the video and output compressed information */
void readVideoData(string videoPath, int width, int height, ofstream &outputFile);

/** DCT encoding function */
void dctEncode(int width, int height, double ***rblocks, double ***gblocks, double ***bblocks, unsigned char *dctData);

/** Function to compute motion vectors for macroblocks */
void computeMotionVectors(const vector<unsigned char>& currFrame, const vector<unsigned char>& prevFrame, int width, int height, vector<vector<pair<int, int>>>& motionVectors);

/** Function to group macroblocks into background and foreground */
void segmentForegroundBackground(const vector<vector<pair<int, int>>>& motionVectors, int widthBlocks, int heightBlocks, vector<vector<bool>>& isForeground);

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

    // Create buffer for interleaved RGB data
    vector<unsigned char> rgbBuffer(width * height * 3);
    vector<unsigned char> prevFrame(width * height * 3);
    vector<unsigned char> currFrame(width * height * 3);

    bool firstFrame = true;

    // Loop and read 1 frame at a time until eof
    while (!inputFile.eof())
    {
        // Read frame data directly in RGB interleaved format
        inputFile.read(reinterpret_cast<char*>(rgbBuffer.data()), width * height * 3);

        // Check if we actually read any data
        if (inputFile.gcount() == 0) {
            break;
        }

        // Copy RGB values to currFrame
        currFrame = rgbBuffer;

        if (firstFrame)
        {
            // For the first frame, we can assume it's an I-frame
            // Here, you might want to process the first frame differently
            // For simplicity, we'll just write a placeholder
            outputFile << "I-Frame data\n";
            firstFrame = false;
        }
        else
        {
            // Compute motion vectors based on the previous frame
            int macroblockSize = 16;
            int widthBlocks = width / macroblockSize;
            int heightBlocks = height / macroblockSize;

            // Vector to hold motion vectors for each macroblock
            vector<vector<pair<int, int>>> motionVectors(heightBlocks, vector<pair<int, int>>(widthBlocks));

            computeMotionVectors(currFrame, prevFrame, width, height, motionVectors);

            // Segment macroblocks into background and foreground
            vector<vector<bool>> isForeground(heightBlocks, vector<bool>(widthBlocks, false));
            segmentForegroundBackground(motionVectors, widthBlocks, heightBlocks, isForeground);

            // Here, you can process the macroblocks differently based on whether they're foreground or background
            // For simplicity, we'll write out the block types
            for (int i = 0; i < heightBlocks; ++i)
            {
                for (int j = 0; j < widthBlocks; ++j)
                {
                    if (isForeground[i][j])
                    {
                        outputFile << "Foreground Macroblock at (" << i << "," << j << ")\n";
                    }
                    else
                    {
                        outputFile << "Background Macroblock at (" << i << "," << j << ")\n";
                    }
                }
            }
        }

        // Update prevFrame
        prevFrame = currFrame;
    }

    inputFile.close();
    outputFile.close();
}

/** Function to compute motion vectors for macroblocks using Three-Step Search */
void computeMotionVectors(const vector<unsigned char>& currFrame, const vector<unsigned char>& prevFrame, int width, int height, vector<vector<pair<int, int>>>& motionVectors)
{
    int macroblockSize = 16;
    int initialStepSize = 4; // Adjust based on your needs
    int widthBlocks = width / macroblockSize;
    int heightBlocks = height / macroblockSize;

    for (int mbRow = 0; mbRow < heightBlocks; ++mbRow)
    {
        for (int mbCol = 0; mbCol < widthBlocks; ++mbCol)
        {
            int minMAD = std::numeric_limits<int>::max();
            int bestDx = 0;
            int bestDy = 0;

            // Coordinates of the current macroblock in the current frame
            int currX = mbCol * macroblockSize;
            int currY = mbRow * macroblockSize;

            int stepSize = initialStepSize;
            int centerX = currX;
            int centerY = currY;

            while (stepSize >= 1)
            {
                // Search points in the pattern
                vector<pair<int, int>> searchPoints = {
                    {centerX, centerY},
                    {centerX - stepSize, centerY},
                    {centerX + stepSize, centerY},
                    {centerX, centerY - stepSize},
                    {centerX, centerY + stepSize}
                };

                for (const auto& point : searchPoints)
                {
                    int x = point.first;
                    int y = point.second;

                    // Ensure the search point is within frame boundaries
                    if (x < 0 || x > width - macroblockSize || y < 0 || y > height - macroblockSize)
                        continue;

                    int mad = 0;

                    // Compute MAD between the current macroblock and the candidate block in the previous frame
                    for (int i = 0; i < macroblockSize; ++i)
                    {
                        for (int j = 0; j < macroblockSize; ++j)
                        {
                            int currIndex = ((currY + i) * width + (currX + j)) * 3;
                            int prevIndex = ((y + i) * width + (x + j)) * 3;

                            // Compute absolute difference for each color channel and sum
                            for (int c = 0; c < 3; ++c)
                            {
                                int diff = currFrame[currIndex + c] - prevFrame[prevIndex + c];
                                mad += abs(diff);
                            }
                        }
                    }

                    // Normalize MAD
                    mad = mad / (macroblockSize * macroblockSize * 3);

                    if (mad < minMAD)
                    {
                        minMAD = mad;
                        bestDx = x - currX;
                        bestDy = y - currY;
                        centerX = x;
                        centerY = y;
                    }
                }

                stepSize /= 2;
            }

            // Store the motion vector for the macroblock
            motionVectors[mbRow][mbCol] = {bestDx, bestDy};
        }
    }
}


/** Function to group macroblocks into background and foreground */
void segmentForegroundBackground(const vector<vector<pair<int, int>>>& motionVectors, int widthBlocks, int heightBlocks, vector<vector<bool>>& isForeground)
{
    // First, we need to cluster the motion vectors to find the dominant motion (background motion)

    // Map to store the frequency of each motion vector
    map<pair<int, int>, int> motionVectorFrequency;

    for (int i = 0; i < heightBlocks; ++i)
    {
        for (int j = 0; j < widthBlocks; ++j)
        {
            motionVectorFrequency[motionVectors[i][j]]++;
        }
    }

    // Find the most frequent motion vector (assumed to be background motion)
    pair<int, int> backgroundMotionVector;
    int maxFrequency = 0;
    for (const auto& entry : motionVectorFrequency)
    {
        if (entry.second > maxFrequency)
        {
            maxFrequency = entry.second;
            backgroundMotionVector = entry.first;
        }
    }

    // Threshold for considering motion vectors as similar
    int motionThreshold = 2; // You can adjust this value based on your needs

    // Label macroblocks as background or foreground
    for (int i = 0; i < heightBlocks; ++i)
    {
        for (int j = 0; j < widthBlocks; ++j)
        {
            int dx = motionVectors[i][j].first - backgroundMotionVector.first;
            int dy = motionVectors[i][j].second - backgroundMotionVector.second;
            int distance = sqrt(dx * dx + dy * dy);

            if (distance <= motionThreshold)
            {
                // Macroblock is background
                isForeground[i][j] = false;
            }
            else
            {
                // Macroblock is foreground
                isForeground[i][j] = true;
            }
        }
    }

    // Optional: Ensure contiguity by grouping connected foreground macroblocks
    // For simplicity, we can use a flood-fill algorithm to label connected regions
    vector<vector<int>> labels(heightBlocks, vector<int>(widthBlocks, -1));
    int label = 0;

    for (int i = 0; i < heightBlocks; ++i)
    {
        for (int j = 0; j < widthBlocks; ++j)
        {
            if (isForeground[i][j] && labels[i][j] == -1)
            {
                // Start a new region
                queue<pair<int, int>> q;
                q.push({i, j});
                labels[i][j] = label;

                while (!q.empty())
                {
                    auto [y, x] = q.front();
                    q.pop();

                    // Check 4-connected neighbors
                    const int dx[4] = { -1, 1, 0, 0 };
                    const int dy[4] = { 0, 0, -1, 1 };

                    for (int k = 0; k < 4; ++k)
                    {
                        int ny = y + dy[k];
                        int nx = x + dx[k];

                        if (ny >= 0 && ny < heightBlocks && nx >= 0 && nx < widthBlocks)
                        {
                            if (isForeground[ny][nx] && labels[ny][nx] == -1)
                            {
                                labels[ny][nx] = label;
                                q.push({ny, nx});
                            }
                        }
                    }
                }
                label++;
            }
        }
    }

    // Now, you can process each connected foreground region separately if needed
}

/** DCT encoding function */
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
