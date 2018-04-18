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

		int d = 0;
		double area = 0, area_max = 0;
		
		char str[200];

		img.copyTo(src);
		createTrackbars();
		
		cv::cvtColor(src, src_gray, CV_BGR2GRAY);
		
		 int hbins = 30, sbins = 32;
    		int histSize[] = {hbins, sbins};
    		// hue varies from 0 to 179, see cvtColor
    		float hranges[] = { 0, 180 };
    		// saturation varies from 0 (black-gray-white) to
    		// 255 (pure spectrum color)
    		float sranges[] = { 0, 256 };
    		const float* ranges[] = { hranges, sranges };
    		MatND hist;
    		// we compute the histogram from the 0-th and 1-st channels
    		int channels[] = {0};

    		calcHist( &src_gray, 1, channels, Mat(),hist, 2, histSize, ranges, true,false );
    		double maxVal=0;
    		minMaxLoc(hist, 0, &maxVal, 0, 0);

    		int scale = 10;
    		Mat histImg = Mat::zeros(sbins*scale, hbins*10, CV_8UC3);

    		for( int h = 0; h < hbins; h++ ){
        		for( int s = 0; s < sbins; s++ )
        {
            float binVal = hist.at<float>(h, s);
            int intensity = cvRound(binVal*255/maxVal);
            rectangle( histImg, Point(h*scale, s*scale),Point( (h+1)*scale - 1, (s+1)*scale - 1), Scalar::all(intensity),CV_FILLED );
        }}

		
		
			cv::imshow(windowName1,src_gray );
			//cv::imshow(windowName, Roi);
			cv::imshow("Histograma", histImg);
			
			std_msgs::Int32 msgx; //Momento en X
			msgx.data = cx;
			std_msgs::Int32 msgy; //Momento en Y
			msgy.data = cy;
			
			mx_pub.publish(msgx);
			my_pub.publish(msgy);
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
