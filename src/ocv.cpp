//http://docs.opencv.org/3.1.0
#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <string>
#include <math.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int32.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/video/tracking.hpp>

using namespace std;
using namespace cv;

class Edge_Detector
{
   	ros::NodeHandle nh1_;
  	image_transport::ImageTransport it1_;
  	image_transport::Subscriber image_sub_;
  	image_transport::Publisher image_pub_;
  	
	ros::Publisher mx_pub;
  	ros::Publisher my_pub;
  	ros::Publisher rad_pub;
  	ros::Publisher det_pub;

	int H_MIN, H_MAX, S_MIN, S_MAX, V_MIN, V_MAX;
	int radius, cx, cy;

	int Thresh_Close;
	int Max_Thresh_C;			
	
	std::string windowName = "Original Image";
	std::string windowName1 = "HSV Image";

	
public:
	//Constructor por defecto de la clase con lista de constructores
  	Edge_Detector() : it1_(nh1_){

    		image_sub_ = it1_.subscribe("/cv_camera/image_raw", 1, &Edge_Detector::imageCb, this);
		//image_sub_ = it1_.subscribe("/bebop/image_raw", 1, &Edge_Detector::imageCb, this);

    		mx_pub = nh1_.advertise<std_msgs::Int32>("/MX",1);
    		my_pub = nh1_.advertise<std_msgs::Int32>("/MY",1);
    		rad_pub = nh1_.advertise<std_msgs::Int32>("/Radio",1);    
	
   		H_MIN = 0;
 		H_MAX = 255;
		S_MIN = 0;
		S_MAX = 255;
		V_MIN = 0;
		V_MAX = 255;
		radius=0; 
		cx=0; 
		cy=0; 
	
		Thresh_Close = 0;
		Max_Thresh_C = 20;  
  	}

	//Desctructor de la clase
  	~Edge_Detector(){
    		cv::destroyAllWindows();
	}

  	void imageCb(const sensor_msgs::ImageConstPtr& msg){

    		cv_bridge::CvImagePtr cv_ptr;
    		namespace enc = sensor_msgs::image_encodings;

   		 try{
      			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    		}
    		catch (cv_bridge::Exception& e){
      			ROS_ERROR("cv_bridge exception: %s", e.what());
      			return;
    		}	

		detect_edges(cv_ptr->image);	
  	}

	void createTrackbars(){

		cv::namedWindow(windowName1, CV_WINDOW_AUTOSIZE);
		      
		cv::createTrackbar("H_MIN", windowName1, &H_MIN, H_MAX);
		cv::createTrackbar("H_MAX", windowName1, &H_MAX, H_MAX);
		cv::createTrackbar("S_MIN", windowName1, &S_MIN, S_MAX);
		cv::createTrackbar("S_MAX", windowName1, &S_MAX, S_MAX);
		cv::createTrackbar("V_MIN", windowName1, &V_MIN, V_MAX);
		cv::createTrackbar("V_MAX", windowName1, &V_MAX, V_MAX);

		cv::createTrackbar("Thresh_Close", windowName1, &Thresh_Close, Max_Thresh_C);
	}

  	void detect_edges(cv::Mat img){
	
		cv::Mat src,HSV;
		cv::Mat threshold;
		cv::Mat threshold1;
		cv::Mat combine_image; 
		cv::Mat image_exit;
		int d = 0;
		img.copyTo(src);
		createTrackbars();

		cv::cvtColor(src, HSV, CV_RGB2HSV_FULL);
		cv::inRange(HSV, cv::Scalar(H_MIN, S_MIN, V_MIN), cv::Scalar(H_MAX, S_MAX, V_MAX), threshold);
				
		cv::Mat Element = getStructuringElement(cv::MORPH_ELLIPSE,  Size( 2*Thresh_Close + 1, 2*Thresh_Close+1 ), Point( Thresh_Close, Thresh_Close ));
		cv::morphologyEx(threshold, combine_image, cv::MORPH_CLOSE, Element);
		
		std::vector<cv::Vec3f> circles;

		cv::HoughCircles(combine_image, circles, CV_HOUGH_GRADIENT, 3, 100000, 200, 100, 0, 0);

			for (size_t i = 0; i < circles.size(); i++)
			{
				cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
				cx=cvRound(circles[i][0]);
				cy=cvRound(circles[i][1]);
				radius = cvRound(circles[i][2]);
				//Centro
				cv::circle(src, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
				// Borde
				cv::circle(src, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
				std::cout << "Radio " << radius <<" \n";
				std::cout << "Centro " << center << " \n";
				if (radius <= 10) {
				cv::putText(src, "Center", cv::Point(0, 50), 2, 1, cv::Scalar(200, 255, 0), 2);
				}
			}
			
			cv::imshow(windowName1, combine_image);
			cv::imshow(windowName, src);
			
			cout << "Width : " << src.cols << endl;
			cout << "Height: " << src.rows << endl;
			
			std_msgs::Int32 msgx; //Momento en X
			msgx.data = cx;
			std_msgs::Int32 msgy; //Momento en Y
			msgy.data = cy;
			std_msgs::Int32 msgrad; //Radio Circulo
			msgrad.data = radius;//circle detected
			
			mx_pub.publish(msgx);
			my_pub.publish(msgy);
			rad_pub.publish(msgrad);
			cv::waitKey(3);
			
  }	
 
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Edge_Detector");
  Edge_Detector ic;
  ros::spin();
  return 0;
}
