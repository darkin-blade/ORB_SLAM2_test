#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>

#include <opencv2/core/core.hpp>

#include <System.h>


#define LOG(format, ...) \
  printf("\033[1;32m" format "\33[0m\n", ## __VA_ARGS__)

using namespace std;
using namespace cv;

string settingsFile = "../MyTest/P20.yaml";
string vocFile      = "../Vocabulary/ORBvoc.txt";
string videoFile    = "../TestData/2.mp4";
const int FRAME_SIZE = 640 * 480;

int main(int argc, char **argv) {
  VideoCapture cap;
  cap.open(videoFile);
  if (!cap.isOpened()) {
    LOG("can't load video");
    return 1;
  }

  ORB_SLAM2::System SLAM(vocFile, settingsFile, ORB_SLAM2::System::MONOCULAR, true);
  auto start = chrono::system_clock::now();
  while (true) {
    Mat frame;
    cap >> frame;
    // imshow("frame", frame);
    if (frame.data == nullptr) {
      break;
    }
    double originSize = frame.cols * frame.rows;
    double scale = sqrt(FRAME_SIZE / originSize);
    resize(frame, frame, Size(), scale, scale);
    auto now = chrono::system_clock::now();
    auto timestamp = chrono::duration_cast<chrono::milliseconds>(now - start);
    SLAM.TrackMonocular(frame, double(timestamp.count()) / 1000.0);
    waitKey(30);
  }

  SLAM.Shutdown();
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
  return 0;
}