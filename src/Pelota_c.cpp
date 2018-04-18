//http://docs.opencv.org/3.1.0
/*
	28 Febrero 2018 22:23
	Se agregó Filtro Median Blur
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
	int M_Blur;
	int B_MAX;

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
		M_Blur=0;
		B_MAX=20;
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
		cv::createTrackbar("MedianB", windowName1, &M_Blur,B_MAX);
		cv::createTrackbar("Thresh_Close", windowName1, &Thresh_Close, Max_Thresh_C);
	}

  	void detect_edges(cv::Mat img){
	
		cv::Mat src,HSV;
		cv::Mat threshold;
		cv::Mat threshold1;
		cv::Mat combine_image; 
		cv::Mat image_exit;

		vector<vector<Point> > contours;
  		vector<Vec4i> hierarchy;
		
		vector<Vec4i> lines;
		
		int d = 0;
		double area = 0, area_max = 0;
		
		char str[200];
	
		img.copyTo(src);
		createTrackbars();

		cv::cvtColor(src, HSV, CV_RGB2HSV_FULL);
		cv::inRange(HSV, cv::Scalar(H_MIN, S_MIN, V_MIN), cv::Scalar(H_MAX, S_MAX, V_MAX), threshold);
		cv::medianBlur(threshold,threshold,2*M_Blur + 1);
				
		cv::Mat Element = getStructuringElement(cv::MORPH_ELLIPSE,  Size( 2*Thresh_Close + 1, 2*Thresh_Close+1 ), Point( Thresh_Close, Thresh_Close ));
		cv::morphologyEx(threshold, combine_image, cv::MORPH_CLOSE, Element);
		
		findContours( combine_image, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );

		if (contours.size() == 0 ){
			
			sprintf(str, "No hay contornos");
			putText(src, str, Point2f(0,50), FONT_HERSHEY_PLAIN, 2,  Scalar(0,0,255), 3);
			cv::imshow(windowName1, combine_image);
			cv::imshow(windowName, src);
			cv::waitKey(3);
			
		}else{

		
			for( int i = 0; i< contours.size(); i++ ){
				area = contourArea(contours[i]);
				if (area > area_max){
					area_max = area;
					radius=(int) ceil(sqrt(area_max/3.1416));
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
		
			//Dibuja contornos y centro de Masa
			//drawContours( src, contours, d, cv::Scalar(255, 0, 0), 2, 8, hierarchy, 0, Point() );
			circle( src, mc[d], 7, cv::Scalar(0, 255, 0),-1);
       			circle( src, mc[d], radius, cv::Scalar(255,0, 0),6);

			sprintf(str,"CX = %d CY = %d Radio = %d",cx,cy,radius);
			putText(src, str, Point2f(0,50), FONT_HERSHEY_PLAIN, 2,  Scalar(0,0,255), 3);
		
			
			cv::imshow(windowName1, combine_image);
			cv::imshow(windowName, src);

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
			
  }	
 
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Edge_Detector");
  Edge_Detector ic;
  ros::spin();
  return 0;
}