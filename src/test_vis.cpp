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
#include <opencv2/calib3d/calib3d.hpp>  
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
	
	int thresh;
	int N;
	int N_MAX;
	int l;
	int l_MAX;
	
	vector<vector<Point> > squares;

	
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
		
		thresh = 50;
		N= 11;
		l=0;
		l_MAX=11;
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
		cv::createTrackbar("N", windowName1, &l, l_MAX);
	}

	static double angle( Point pt1, Point pt2, Point pt0 ){
		double dx1 = pt1.x - pt0.x;
		double dy1 = pt1.y - pt0.y;
		double dx2 = pt2.x - pt0.x;
		double dy2 = pt2.y - pt0.y;
		return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
	}

  	void detect_edges(cv::Mat img){
	
		cv::Mat src, src_gray,gray;
		cv::Mat threshold;
		cv::Mat threshold1;
		cv::Mat canny; 
		cv::Mat image_exit;

		base_cmd.angular.y = -45.0;
  		base_cmd.angular.z = 0.0;		
		camera_pos_pub.publish(base_cmd);
		
		
		int d = 0;
		double area = 0, area_max = 0;
		
		char str[200];
		
		vector<vector<Point> > contours;
		img.copyTo(src);
		createTrackbars();
		
		//squares.clear();
		
		cv::cvtColor(src, src_gray, CV_BGR2GRAY);
		
		//Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
		//cv::threshold( src_gray, image_exit, threshold_value, 255, 1 );
		// hack: use Canny instead of zero threshold level.
		//Canny helps to catch squares with gradient shading
		if( l == 0 ){
			// apply Canny. Take the upper threshold from slider
			// and set the lower to 0 (which forces edges merging)
			Canny(src_gray, gray, 0, thresh, 5);
			// dilate canny output to remove potential
			// holes between edge segments
			dilate(gray, gray, Mat(), Point(-1,-1));
		}
		else{
			// apply threshold if l!=0:
			//tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
			gray = src_gray >= (l+1)*255/N;
		}
		//find contours and store them all as a list
		findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
		
		vector<Point> approx;

		// test each contour
		for( size_t i = 0; i < contours.size(); i++ ){
                // approximate contour with accuracy proportional
                // to the contour perimeter
                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if( approx.size() == 4 && fabs(contourArea(Mat(approx))) > 1000 && isContourConvex(Mat(approx)) ){
                    double maxCosine = 0;

                    for( int j = 2; j < 5; j++ ){
                        // find the maximum cosine of the angle between joint edges
                        double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }

                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                    if( maxCosine < 0.3 )
                        squares.push_back(approx);
                }
            	}
		
		
		for( size_t i = 0; i < squares.size(); i++ ){
			const Point* p = &squares[i][0];
			int n = (int)squares[i].size();
			polylines(src, &p, &n, 1, true, Scalar(0,255,0), 3, LINE_AA);
		}

			
		
		cv::imshow(windowName1,src );
		//cv::imshow(windowName, Roi);
		cv::imshow("O", gray);
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