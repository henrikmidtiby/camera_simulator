#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <glob.h>
#include <vector>

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


std::vector<std::string> globVector(const std::string& pattern){
	glob_t glob_result;
	glob(pattern.c_str(),GLOB_TILDE,NULL,&glob_result);
	std::vector<std::string> files;
	for(unsigned int i=0;i<glob_result.gl_pathc;++i){
		files.push_back(std::string(glob_result.gl_pathv[i]));
	}
	globfree(&glob_result);
	return files;
}

int main(int argc, char** argv)
{
	std::cout << "camera_simulator: " << std::endl;
	std::cout << "filename: " << argv[1] << std::endl;
	ros::init(argc, argv, "camera_simulator");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("usb_cam/image_raw", 1);
	cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
	cv::waitKey(30);	
	
	std::string imageDirectory;
	if(!ros::param::get("/camera_simulator/imageDirectory", imageDirectory)) { ROS_ERROR("Failed to get param 'imageDirectory'"); }
	
	std::vector<std::string> files = globVector(imageDirectory + "/*.JPG");
	for(int k = 0; k < files.size(); k++)
	{
		std::cout << "k: " << k << "\tfilename: " << files.at(k) << std::endl;
	}
	
	ros::Rate loop_rate(10);
	int counter = 0;
	while (nh.ok()) {
		
		image = cv::imread(files.at(counter), CV_LOAD_IMAGE_COLOR);
		
		cv::Mat scaledImage;
		cv::Size size(1280, 720);
		cv::Mat color = convertToExcessGreen(image);
		
		cv::resize(color, scaledImage, size);
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", scaledImage).toImageMsg();
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		
		counter = (counter + 1) % files.size();
	}
}

