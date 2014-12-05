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

#include <stdlib.h>
#include <string.h>     // string function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions

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
  //Initialize Imaging objects  
  CvCapture* capture;
  cv::Mat frame;

  cv::namedWindow(main_window_name,CV_WINDOW_NORMAL);
  cv::moveWindow(main_window_name, 400, 100);

  //Initialize Mouse objects
  Display *display = XOpenDisplay(0);
  Window root = DefaultRootWindow(display);

  //Open FTDI port
  int USB = open( "/dev/ttyUSB0", O_RDWR| O_NOCTTY );
  struct termios tty;
  struct termios tty_old;
  memset (&tty, 0, sizeof tty);

  if ( tcgetattr ( USB, &tty ) != 0 )
  {
      cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;
  }

  tty_old = tty;
 
  cfsetospeed (&tty, (speed_t)B9600);
  cfsetispeed (&tty, (speed_t)B9600);

  tty.c_cflag     &=  ~PARENB;    
  tty.c_cflag     &=  ~CSTOPB;
  tty.c_cflag     &=  ~CSIZE;
  tty.c_cflag     |=  CS8;

  tty.c_cflag     &=  ~CRTSCTS;    
  tty.c_cc[VMIN]      =   1;           
  tty.c_cc[VTIME]     =   5;               
  tty.c_cflag     |=  CREAD | CLOCAL;    
  cfmakeraw(&tty);
  tcflush( USB, TCIFLUSH );

  if ( tcsetattr ( USB, TCSANOW, &tty ) != 0)
  {
      cout << "Error " << errno << " from tcsetattr" << endl;
  }

  // Load the cascades
  if( !face_cascade.load( face_cascade_name ) ){ printf("--(!)Error loading face cascade, please change face_cascade_name in source code.\n"); return -1; };


  //TODO: Find out what this does
  createCornerKernels();
  ellipse(skinCrCbHist, cv::Point(113, 155.6), cv::Size(23.4, 15.2),
          43.0, 0.0, 360.0, cv::Scalar(255, 255, 255), -1);

  // Read the video stream
  capture = cvCaptureFromCAM( -1 );
  vector<int> result;
  if( !frame.empty() ) {
    result = detectAndDisplay( frame );
  }
  int rX=0, rY=0, lX=0, lY=0, tX=0, tY=0, bX=0, bY=0;
  int i = 1;
  bool check = true;
  int x = 960, y = 540;
  string c;

  //Calibrate the right edge of the screen
  cout << "look at the right part of the screen";
  cin >> c;
  frame = cvQueryFrame( capture );
  cv::flip(frame, frame, 1);
  frame.copyTo(debugImage);
  if( !frame.empty() ) 
  {
    result = detectAndDisplay( frame );
  }
  if(result.size() >= 2)      
  {
    rX = result[0];
    rY = result[1];
  }

  //Calibrate the left edge of the screen
  cout << "look at the left part of the screen";
  cin >> c;
  frame = cvQueryFrame( capture );
  cv::flip(frame, frame, 1);
  frame.copyTo(debugImage);
  if( !frame.empty() ) 
  {
    result = detectAndDisplay( frame );
  }
  if(result.size() >= 2)      
  {
    lX = result[0];
    lY = result[1];
  }

  //Calibrate the bottom part of the screen
  cout << "look at the bottom part of the screen";
  cin >> c;
  frame = cvQueryFrame( capture );
  cv::flip(frame, frame, 1);
  frame.copyTo(debugImage);
  if( !frame.empty() ) 
  {
    result = detectAndDisplay( frame );
  }
  if(result.size() >= 2)      
  {
    bX = result[0];
    bY = result[1];
  }

  //Calibrate the top part of the screen
  cout << "look at the top part of the screen";
  cin >> c;
  frame = cvQueryFrame( capture );
  cv::flip(frame, frame, 1);
  frame.copyTo(debugImage);
  if( !frame.empty() ) 
  {
    result = detectAndDisplay( frame );
  }
  if(result.size() >= 2)      
  {
    tX = result[0];
    tY = result[1];
  }

  cout << "TOP: " << tX << " " << tY << endl;
  cout << "BOTTOM: " << bX << " " << bY << endl;
  cout << "LEFT: " << lX << " " << lY << endl;
  cout << "RIGHT: " << rX << " " << rY << endl;
  //Mouse loop w/ reading serial
  if( capture ) 
  {
    while( check ) 
    {
      //Capture an image
      frame = cvQueryFrame( capture );
      
      //Mirror it
      cv::flip(frame, frame, 1);

      frame.copyTo(debugImage);
  
      // Apply the classfier to the frame
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

      i++;
      cout << i << endl;
      
      //Read serial data from Gloves
      int n = 0;
      char buf = '\0';
      std::string response;
    
      n = read( USB, &buf, 1 );
      response.append( &buf );

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
          cout << "Response: " << response << endl;
          
          //Figure out which key was pressed based on response
          switch(*(response.c_str()))
          {
            case 'a':
              cout << 'a' << endl;
              break;
            case 'b':
              cout << 'b' << endl;
              break;
            case 'c':
              cout << 'c' << endl;
              break;
            case 'd':
              cout << 'd' << endl;
              break;
            case 'e':
              cout << 'e' << endl;
              break;
            case 'f':
              cout << 'f' << endl;
              break;
            case 'g':
              cout << 'g' << endl;
              break;
            case 'h':
              cout << 'h' << endl;
              break;
            case 'i':
              cout << 'i' << endl;
              break;
            case 'j':
              cout << 'j' << endl;
              break;
            case 'k':
              cout << 'k' << endl;
              break;
            case 'l':
              cout << 'l' << endl;
              break;
            case 'm':
              cout << 'm' << endl;
              break;
            case 'n':
              cout << 'n' << endl;
              break;
            case 'o':
              cout << 'o' << endl;
              break;
            case 'p':
              cout << 'p' << endl;
              break;
            case 'q':
              cout << 'q' << endl;
              break;
            case 'r':
              cout << 'r' << endl;
              break;
            case 's':
              cout << 's' << endl;
              break;
            case 't':
              cout << 't' << endl;
              break;
            case 'u':
              cout << 'u' << endl;
              break;
            case 'v':
              cout << 'v' << endl;
              break;
            case 'w':
              cout << 'w' << endl;
              break;
            case 'x':
              cout << 'x' << endl;
              break;
            case 'y':
              cout << 'y' << endl;
              break;
            case 'z':
              cout << 'z' << endl;
              break;
          }
      }
      
      //Move mouse based on imaging
      XWarpPointer(display, None, root, 0, 0, 0, 0, 960 + i , 540);
      XFlush(display);

      //Reset the results vector
      result.clear();

    }
  }

  //Cleanup
  XCloseDisplay(display);
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
	cout << "RIGHT PUPIL X,Y: " << rightPupil.x << " " << rightPupil.y << endl;
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

  imshow(main_window_name, faceROI);
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
vector<int> detectAndDisplay( cv::Mat frame ) 
{
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
