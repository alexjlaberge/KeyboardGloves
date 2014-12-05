#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <queue>
#include <stdio.h>
#include <math.h>

#include "constants.h"
#include "findEyeCenter.h"
#include "findEyeCorner.h"

#include <unistd.h>

#include <X11/Xlib.h>
#include <X11/Xutil.h>

#include <vector>

#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <iostream>

/** Function Headers */
std::vector<int> detectAndDisplay( cv::Mat frame );

/** Global variables */
cv::String face_cascade_name = "../res/haarcascade_frontalface_alt.xml";
cv::CascadeClassifier face_cascade;
std::string main_window_name = "Capture - Face detection";
std::string face_window_name = "Capture - Face";
cv::RNG rng(12345);
cv::Mat debugImage;
cv::Mat skinCrCbHist = cv::Mat::zeros(cv::Size(256, 256), CV_8UC1);

using namespace std;

int main( int argc, const char** argv ) {
  CvCapture* capture;
  cv::Mat frame;

  Display *display = XOpenDisplay(0);
  Window root = DefaultRootWindow(display);
    //for(int x = 0; x < 1000; x++){for(int y = 0; y < 1000; y++){XWarpPointer(display, None, root, 0, 0, 0, 0, x, y); usleep(100);}}
  XFlush(display);


  XCloseDisplay(display);

  int USB = open( "/dev/ttyUSB0", O_RDWR| O_NOCTTY );
  struct termios tty;
  struct termios tty_old;
  memset (&tty, 0, sizeof tty);

  /* Error Handling */
  if ( tcgetattr ( USB, &tty ) != 0 )
  {
      cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;
  }

  /* Save old tty parameters */
  tty_old = tty;

  /* Set Baud Rate */
  cfsetospeed (&tty, (speed_t)B9600);
  cfsetispeed (&tty, (speed_t)B9600);

  /* Setting other Port Stuff */
  tty.c_cflag     &=  ~PARENB;        // Make 8n1
  tty.c_cflag     &=  ~CSTOPB;
  tty.c_cflag     &=  ~CSIZE;
  tty.c_cflag     |=  CS8;

  tty.c_cflag     &=  ~CRTSCTS;       // no flow control
  tty.c_cc[VMIN]      =   1;                  // read doesn't block
  tty.c_cc[VTIME]     =   5;                  // 0.5 seconds read timeout
  tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
  cout << "test" << flush;
  /* Make raw */
  cfmakeraw(&tty);

  /* Flush Port, then applies attributes */
  tcflush( USB, TCIFLUSH );
  if ( tcsetattr ( USB, TCSANOW, &tty ) != 0)
  {
      cout << "Error " << errno << " from tcsetattr" << endl;
  }

    //XWarpPointer(display, None, root, 0, 0, 0, 0, x, y);
  // Load the cascades
  if( !face_cascade.load( face_cascade_name ) ){ printf("--(!)Error loading face cascade, please change face_cascade_name in source code.\n"); return -1; };



  createCornerKernels();
  ellipse(skinCrCbHist, cv::Point(113, 155.6), cv::Size(23.4, 15.2),
          43.0, 0.0, 360.0, cv::Scalar(255, 255, 255), -1);

  // Read the video stream
  capture = cvCaptureFromCAM( -1 );
  
  int i = 1;
  bool check = true;
  int x = 960, y = 540;
  
  if( capture ) 
  {
    while( check ) 
    {
      frame = cvQueryFrame( capture );
      // mirror it
      cv::flip(frame, frame, 1);
      frame.copyTo(debugImage);
      // Apply the class>ier to the frame
      vector<int> result;
      if( !frame.empty() ) {
        result = detectAndDisplay( frame );
      }
      else 
      {
        printf(" --(!) No captured frame -- Break!");
        break;
      }
      //120 ... 130 ... 140
      //0 ... 960 ... 1920 for x
      //45 ... 60 ... 75
      //0 ... 540 ... 1080
      int dX = result[0] - 130;
      int dY = result[1] - 60;
      dX = dX * (1920 / 20);
      x = x + dX; 
      dY = dY * (1080/30);
      y = y + dY;
      int c = cv::waitKey(10);
      if( (char)c == 'c' ) { break; }
      if( (char)c == 'f' ) { }
      i++;
      usleep(10000);
      //READ
      int n = 0;
      char buf = '\0';

      /* Whole response*/
      std::string response;

      do
      {
         n = read( USB, &buf, 1 );
         response.append( &buf );
      }while(buf != '\r' && n > 0);

      if (n < 0)
      {
         cout << "Error reading: " << strerror(errno) << endl;
      }
      else if (n == 0)
      {
          cout << "Read nothing!" << endl;
      }
      else
      {
          cout << "Response: " << response;
          response = response.substr(1,1);
          switch(*(response.c_str()))
          {
            case 'a':
              cout << 'a';
              break;
            case 'b':
              cout << 'b';
              break;
            case 'c':
              cout << 'c';
              break;
            case 'd':
              cout << 'd';
              break;
            case 'e':
              cout << 'e';
              break;
            case 'f':
              cout << 'f';
              break;
            case 'g':
              cout << 'g';
              break;
            case 'h':
              cout << 'h';
              break;
            case 'i':
              cout << 'i';
              break;
            case 'j':
              cout << 'j';
              break;
            case 'k':
              cout << 'k';
              break;
            case 'l':
              cout << 'l';
              break;
            case 'm':
              cout << 'm';
              break;
            case 'n':
              cout << 'n';
              break;
            case 'o':
              cout << 'o';
              break;
            case 'p':
              cout << 'p';
              break;
            case 'q':
              cout << 'q';
              break;
            case 'r':
              cout << 'r';
              break;
            case 's':
              cout << 's';
              break;
            case 't':
              cout << 't';
              break;
            case 'u':
              cout << 'u';
              break;
            case 'v':
              cout << 'v';
              break;
            case 'w':
              cout << 'w';
              break;
            case 'x':
              cout << 'x';
              break;
            case 'y':
              cout << 'y';
              break;
            case 'z':
              cout << 'z';
              break;
          }
      }
      //XWarpPointer(display, None, root, 0, 0, 0, 0, 960, 540);

    }
  }

  releaseCornerKernels();

  return 0;
}

vector<int> findEyes(cv::Mat frame_gray, cv::Rect face) {
  cv::Mat faceROI = frame_gray(face);
  cv::Mat debugFace = faceROI;

  if (kSmoothFaceImage) {
    double sigma = kSmoothFaceFactor * face.width;
    GaussianBlur( faceROI, faceROI, cv::Size( 0, 0 ), sigma);
  }
  //-- Find eye regions and draw them
  int eye_region_width = face.width * (kEyePercentWidth/100.0);
  int eye_region_height = face.width * (kEyePercentHeight/100.0);
  int eye_region_top = face.height * (kEyePercentTop/100.0);
  cv::Rect leftEyeRegion(face.width*(kEyePercentSide/100.0),
                         eye_region_top,eye_region_width,eye_region_height);
  cv::Rect rightEyeRegion(face.width - eye_region_width - face.width*(kEyePercentSide/100.0),
                          eye_region_top,eye_region_width,eye_region_height);

  //-- Find Eye Centers
  cv::Point leftPupil = findEyeCenter(faceROI,leftEyeRegion,"Left Eye");
  cv::Point rightPupil = findEyeCenter(faceROI,rightEyeRegion,"Right Eye");
  // get corner regions
  cv::Rect leftRightCornerRegion(leftEyeRegion);
  leftRightCornerRegion.width -= leftPupil.x;
  leftRightCornerRegion.x += leftPupil.x;
  leftRightCornerRegion.height /= 2;
  leftRightCornerRegion.y += leftRightCornerRegion.height / 2;
  cv::Rect leftLeftCornerRegion(leftEyeRegion);
  leftLeftCornerRegion.width = leftPupil.x;
  leftLeftCornerRegion.height /= 2;
  leftLeftCornerRegion.y += leftLeftCornerRegion.height / 2;
  cv::Rect rightLeftCornerRegion(rightEyeRegion);
  rightLeftCornerRegion.width = rightPupil.x;
  rightLeftCornerRegion.height /= 2;
  rightLeftCornerRegion.y += rightLeftCornerRegion.height / 2;
  cv::Rect rightRightCornerRegion(rightEyeRegion);
  rightRightCornerRegion.width -= rightPupil.x;
  rightRightCornerRegion.x += rightPupil.x;
  rightRightCornerRegion.height /= 2;
  rightRightCornerRegion.y += rightRightCornerRegion.height / 2;
  rectangle(debugFace,leftRightCornerRegion,200);
  rectangle(debugFace,leftLeftCornerRegion,200);
  rectangle(debugFace,rightLeftCornerRegion,200);
  rectangle(debugFace,rightRightCornerRegion,200);
  // change eye centers to face coordinates
  rightPupil.x += rightEyeRegion.x;
  rightPupil.y += rightEyeRegion.y;
  leftPupil.x += leftEyeRegion.x;
  leftPupil.y += leftEyeRegion.y;
  // draw eye centers
	//cout << "RIGHT PUPIL X,Y: " << rightPupil.x << " " << rightPupil.y << endl;
  circle(debugFace, rightPupil, 3, 1234);
  circle(debugFace, leftPupil, 3, 1234);

  //-- Find Eye Corners
  if (kEnableEyeCorner) {
    cv::Point2f leftRightCorner = findEyeCorner(faceROI(leftRightCornerRegion), true, false);
    leftRightCorner.x += leftRightCornerRegion.x;
    leftRightCorner.y += leftRightCornerRegion.y;
    cv::Point2f leftLeftCorner = findEyeCorner(faceROI(leftLeftCornerRegion), true, true);
    leftLeftCorner.x += leftLeftCornerRegion.x;
    leftLeftCorner.y += leftLeftCornerRegion.y;
    cv::Point2f rightLeftCorner = findEyeCorner(faceROI(rightLeftCornerRegion), false, true);
    rightLeftCorner.x += rightLeftCornerRegion.x;
    rightLeftCorner.y += rightLeftCornerRegion.y;
    cv::Point2f rightRightCorner = findEyeCorner(faceROI(rightRightCornerRegion), false, false);
    rightRightCorner.x += rightRightCornerRegion.x;
    rightRightCorner.y += rightRightCornerRegion.y;
    circle(faceROI, leftRightCorner, 3, 200);
    circle(faceROI, leftLeftCorner, 3, 200);
    circle(faceROI, rightLeftCorner, 3, 200);
    circle(faceROI, rightRightCorner, 3, 200);
  }

  //imshow(face_window_name, faceROI);
  vector<int> result;
  result.push_back(leftPupil.x);
  result.push_back(leftPupil.y);
  result.push_back(rightPupil.x);
  result.push_back(rightPupil.y);
  return result;
//  cv::Rect roi( cv::Point( 0, 0 ), faceROI.size());
//  cv::Mat destinationROI = debugImage( roi );
//  faceROI.copyTo( destinationROI );
}


cv::Mat findSkin (cv::Mat &frame) {
  cv::Mat input;
  cv::Mat output = cv::Mat(frame.rows,frame.cols, CV_8U);

  cvtColor(frame, input, CV_BGR2YCrCb);

  for (int y = 0; y < input.rows; ++y) {
    const cv::Vec3b *Mr = input.ptr<cv::Vec3b>(y);
//    uchar *Or = output.ptr<uchar>(y);
    cv::Vec3b *Or = frame.ptr<cv::Vec3b>(y);
    for (int x = 0; x < input.cols; ++x) {
      cv::Vec3b ycrcb = Mr[x];
//      Or[x] = (skinCrCbHist.at<uchar>(ycrcb[1], ycrcb[2]) > 0) ? 255 : 0;
      if(skinCrCbHist.at<uchar>(ycrcb[1], ycrcb[2]) == 0) {
        Or[x] = cv::Vec3b(0,0,0);
      }
    }
  }
  return output;
}

/**
 * @function detectAndDisplay
 */
vector<int> detectAndDisplay( cv::Mat frame ) {
  std::vector<cv::Rect> faces;
  //cv::Mat frame_gray;

  std::vector<cv::Mat> rgbChannels(3);
  cv::split(frame, rgbChannels);
  cv::Mat frame_gray = rgbChannels[2];

  //cvtColor( frame, frame_gray, CV_BGR2GRAY );
  //equalizeHist( frame_gray, frame_gray );
  //cv::pow(frame_gray, CV_64F, frame_gray);
  //-- Detect faces
  face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE|CV_HAAR_FIND_BIGGEST_OBJECT, cv::Size(150, 150) );
//  findSkin(debugImage);

  for( int i = 0; i < faces.size(); i++ )
  {
    rectangle(debugImage, faces[i], 1234);
  }
  vector<int> result;
  //-- Show what you got
  if (faces.size() > 0) {
    result = findEyes(frame_gray, faces[0]);
  }
  return result;
}
