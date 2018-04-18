//http://docs.opencv.org/3.1.0
/*
 	23 de Febrero 20:42
	El programa funciona correctamente y se agregó la región de ínteres
*/
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
#include <geometry_msgs/Twist.h>

using namespace cv;
using namespace std;
	
class Edge_Detector
{
   	ros::NodeHandle nh1_;
  	image_transport::ImageTransport it1_;
  	image_transport::Subscriber image_sub_;
  	image_transport::Publisher image_pub_;
	geometry_msgs::Twist base_cmd;
  	
	ros::Publisher mx_pub;
  	ros::Publisher my_pub;
	ros::Publisher camera_pos_pub;

	int cx, cy;
	
	int threshold_value;
	int max_value;
	
	int Thresh_Close;
	int Max_Thresh_C;
	
	int cx_ROI;
	int cx_ROI_M;

	int ROI_W;
	int ROI_W_M;
			

	std::string windowName = "Original Image";
	std::string windowName1 = "GRAY Image";

	
public:
	//Constructor por defecto de la clase con lista de constructores
  	Edge_Detector() : it1_(nh1_){

    		image_sub_ = it1_.subscribe("/cv_camera/image_raw", 1, &Edge_Detector::imageCb, this);
		//image_sub_ = it1_.subscribe("/bebop/image_raw", 1, &Edge_Detector::imageCb, this);

    		mx_pub = nh1_.advertise<std_msgs::Int32>("/MX",1);
    		my_pub = nh1_.advertise<std_msgs::Int32>("/MY",1);
		camera_pos_pub = nh1_.advertise<geometry_msgs::Twist>("/bebop/camera_control", 1); 			
				
		cx=0; 
		cy=0; 
		
		threshold_value = 0; 		
		
		Thresh_Close = 0;
		Max_Thresh_C = 20;

		max_value = 255;
		
		cx_ROI = 240;
		cx_ROI_M = 280;
		
		ROI_W = 160;
		ROI_W_M = 160;
		
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
		      
		cv::createTrackbar("Threshold", windowName1, &threshold_value, max_value);
		cv::createTrackbar("Thresh_Close", windowName1, &Thresh_Close, Max_Thresh_C);
		cv::createTrackbar("CX_ROI", windowName1, &cx_ROI, cx_ROI_M);
		cv::createTrackbar("W_ROI", windowName1, &ROI_W, ROI_W_M);
	}

  	void detect_edges(cv::Mat img){
	
		cv::Mat src, src_gray;
		cv::Mat threshold;
		cv::Mat threshold1;
		cv::Mat canny; 
		cv::Mat image_exit;

		vector<vector<Point> > contours;
  		vector<Vec4i> hierarchy;
		
		vector<Vec4i> lines;
			
		base_cmd.angular.y = -45.0;
  		base_cmd.angular.z = 0.0;		
		camera_pos_pub.publish(base_cmd);
		
		Rect Rec(cx_ROI, 0, ROI_W, 480);
		//Rect Rec(240, 0, 160, 360);

		int d = 0;
		double area = 0, area_max = 0;
		
		char str[200];

		img.copyTo(src);
		createTrackbars();
		
		
		rectangle(src, Rec, cv::Scalar(0, 255, 0), 1, 8, 1);		  
		Mat Roi = src(Rec);		
		
		cv::cvtColor(Roi, src_gray, CV_BGR2GRAY);
		
	
		cv::threshold( src_gray, image_exit, threshold_value, 255, 0 );

		cv::Mat Element = getStructuringElement(cv::MORPH_RECT,  Size( 2*Thresh_Close + 1, 2*Thresh_Close+1 ), Point( Thresh_Close, Thresh_Close ));
		cv::morphologyEx(image_exit, image_exit, cv::MORPH_CLOSE, Element);
		
   		findContours( image_exit, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
		
		if(contours.size() == 0){
			sprintf(str,"NO HAY CONTORNOS");
			putText(src, str, Point2f(0,50), FONT_HERSHEY_PLAIN, 2,  Scalar(0,255,0), 3);
		
			cv::imshow(windowName1,image_exit );
			//cv::imshow(windowName, Roi);
			cv::imshow("O", src);
			cv::waitKey(3);
			
		}else{
			
			for( int i = 0; i< contours.size(); i++ ){
				area = contourArea(contours[i]);
				if (area > area_max){
					area_max = area;
					d = i;
				}
			
			}		
			// Momentos
			vector<Moments> mu(contours.size() );
 			mu[d] = moments( contours[d], false ); 

  			// Centros de Masa
  			vector<Point2f> mc( contours.size() );
			mc[d] = Point2f( mu[d].m10/mu[d].m00 , mu[d].m01/mu[d].m00 );
		
			cx = mu[d].m10/mu[d].m00;	
			cy = mu[d].m01/mu[d].m00;
		
			drawContours( Roi, contours, d, cv::Scalar(255, 0, 0), 2, 8, hierarchy, 0, Point() );
       			circle( Roi, mc[d], 7, cv::Scalar(0, 255, 0),-1);
		
			sprintf(str,"CX = %d CY = %d ",cx, cy);
			putText(src, str, Point2f(0,50), FONT_HERSHEY_PLAIN, 2,  Scalar(0,255,0), 3);
		
			cv::imshow(windowName1,image_exit );
			//cv::imshow(windowName, Roi);
			cv::imshow("O", src);
			
			std_msgs::Int32 msgx; //Momento en X
			msgx.data = cx;
			std_msgs::Int32 msgy; //Momento en Y
			msgy.data = cy;
			
			mx_pub.publish(msgx);
			my_pub.publish(msgy);
			cv::waitKey(3);
			
		}
		
		
			
  }	
 
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Edge_Detector");
  Edge_Detector ic;
  ros::spin();
  return 0;
}
