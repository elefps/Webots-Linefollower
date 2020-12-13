#include "opencv2/core/core.hpp"
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "iostream"
#include <stdio.h>
#include <list>
#include <string>
#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/motor.h>
#include <pthread.h>
using namespace std;
using namespace cv;

#define TIME_STEP 64
#define MAX_SPEED 5


double kx=500,ky; //center

void getMidPoint(Mat& camImage);
void line_follow(double k);
void stop();


int main() { 
  // Initialize Webots 
  wb_robot_init();    
  // Initialize camera  
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera,TIME_STEP);  
  static int width = wb_camera_get_width(camera);
  static int height = wb_camera_get_height(camera);     
  // get the motor devices 
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  //set target position to infinity (speed control)
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);  
  //feedback loop: step simulation until an exit event is received
  while (wb_robot_step(TIME_STEP) != -1) {      
    //incoming frames become Mat objects
    Mat frame = Mat(Size(width, height), CV_8UC4);   
    frame.data = (uchar *)wb_camera_get_image(camera);    
    getMidPoint(frame);
    //deciding next move
    if (kx<150){
      printf("%s\n","LEFT");
      wb_motor_set_velocity(right_motor, 0.4 * MAX_SPEED);
      wb_motor_set_velocity(left_motor, -0.4 * MAX_SPEED);
    }
    else if (kx>273){   
      printf("%s\n","RIGHT");
      wb_motor_set_velocity(left_motor, 0.4 * MAX_SPEED);
      wb_motor_set_velocity(right_motor, -0.4 * MAX_SPEED);
    }
    else{   
      printf("%s\n","forward");
      wb_motor_set_velocity(left_motor, 0.5* MAX_SPEED);
    wb_motor_set_velocity(right_motor, 0.5 * MAX_SPEED);
    }
  }
  wb_robot_cleanup();  
  return 0;     
  }

void  getMidPoint(Mat& camImage){
  //convert to grayscale, gaussian blur filter, and threshold
  cvtColor(camImage, camImage,COLOR_BGR2GRAY);
  GaussianBlur(camImage, camImage,Size(9, 9),2,2);        
  threshold(camImage, camImage,100 , 255, THRESH_BINARY_INV);
  imwrite("../thresh.jpg",camImage);
  

  //erode for noise elimination,dilate to restore some eroded parts of image
  erode(camImage, camImage,Mat(),Point(-1,1),2);
  dilate(camImage, camImage,Mat(),Point(-1,1),3);

  vector<vector<Point> > contours;
  vector<Vec4i> h;    
  int largest_area=0;
  int largest_contour_index=0;

  
  findContours(camImage, contours, h,RETR_LIST, CHAIN_APPROX_SIMPLE );

  //find the biggest contour
  size_t i;
  for(  i = 0; i< contours.size(); i++ ){
    double a=contourArea( contours[i],false); 
      if(a>largest_area){
          largest_area=a;        
          largest_contour_index=i;          
      }
  }    
  //find the centroid of the biggest contour using image moments
  if (contours.size() > 0){
    vector<Moments> mu(contours.size());
    mu[largest_contour_index] = moments( contours[largest_contour_index] );
    if (mu[largest_contour_index].m00 > 0){
      kx=mu[largest_contour_index].m10/mu[largest_contour_index].m00;
      ky=mu[largest_contour_index].m01/mu[largest_contour_index].m00;
    }
  }
}


void stop(){
    //wb_motor_set_velocity(left_motor, 0 * MAX_SPEED);
  //wb_motor_set_velocity(right_motor, 0 * MAX_SPEED);
}