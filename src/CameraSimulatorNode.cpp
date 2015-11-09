#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>


cv::Mat convertToExcessGreen(cv::Mat input)
{
	cv::Mat planes[3];
	cv::split(input, planes);
	cv::Mat excessGreen = 255 - 2*(2*planes[1] - planes[0] - planes[2]);
	std::vector<cv::Mat> array_to_merge;
	
	array_to_merge.push_back(excessGreen);
	array_to_merge.push_back(excessGreen);
	array_to_merge.push_back(excessGreen);
	
	cv::Mat color;
	
	cv::merge(array_to_merge, color);
	
	return(color);
}

int main(int argc, char** argv)
{
	std::cout << "camera_simulator: " << std::endl;
	std::cout << "filename: " << argv[1] << std::endl;
	ros::init(argc, argv, "camera_simulator");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("usb_cam/image_raw", 1);
// 	cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
	cv::Mat image = cv::imread("/home/henrik/catkin_ws/src/camera_simulator/data/duck.jpg", CV_LOAD_IMAGE_COLOR);
	cv::waitKey(30);
	
	
	cv::Mat planes[3];
	
	ros::Rate loop_rate(2);
	while (nh.ok()) {
		cv::Mat scaledImage;
		cv::Size size(1280, 720);
		cv::Mat color = convertToExcessGreen(image);
		
		cv::resize(color, scaledImage, size);
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", scaledImage).toImageMsg();
		pub.publish(msg);
		cv::waitKey(1);
		ros::spinOnce();
		loop_rate.sleep();
	}
}