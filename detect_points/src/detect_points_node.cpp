
#include "detect_points/detect_points.h"

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "detect_points");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	// Clustering *cluster = new Clustering(nh, nh_private);
	// detect_points::Image_processing *img_pro = new detect_points::Image_processing(nh, nh_private);
	detect_points::Image_processing img_pro(nh, nh_private);
	ros::spin();
	// ros::shutdown();
	return 0;
}
