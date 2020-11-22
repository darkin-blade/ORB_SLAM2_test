#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>

#include <opencv2/core/core.hpp>

#include <System.h>

using namespace std;
using namespace cv;

string settingsFile = "../MyTest/P20.yaml";
string vocFile      = "../Vocabulary/ORBvoc.txt";
string videoFile    = "../TestData/1.mp4";

int main(int argc, char **argv) {
  ORB_SLAM2::System SLAM(vocFile, settingsFile, ORB_SLAM2::System::MONOCULAR, true);
  VideoCapture videoCapture(videoFile);

  auto start = chrono::system_clock::now();
  while (true) {
    Mat frame;
    videoCapture >> frame;
    if (frame.data == nullptr) {
      break;
    }
    auto now = chrono::system_clock::now();
    auto timestamp = chrono::duration_cast<chrono::milliseconds>(now - start);
    SLAM.TrackMonocular(frame, double(timestamp.count()) / 1000.0);
  }

  SLAM.Shutdown();
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
  return 0;
}