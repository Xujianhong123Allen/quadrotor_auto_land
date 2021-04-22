#ifndef _DETECT_POINTS_H_
#define _DETECT_POINTS_H_

#include <ros/ros.h>
#include <map>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Pose.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <sophus/se3.hpp>
#include <cmath>
#include <vector>
#include <string.h>

namespace detect_points
{
	class Image_processing
	{
	public:
		Image_processing(ros::NodeHandle &nh, ros::NodeHandle nh_private);
		~Image_processing();

	private:
		ros::NodeHandle nh_;
		ros::Subscriber Image_raw_sub_;
		// image_transport::Subscriber Image_raw_sub_;
		ros::Publisher Image_pub_;
		ros::Publisher Pose_pub_;

		std::vector<cv::Point3f> Points3D;
		std::vector<cv::Point2f> Points2D;

		void Image_processing_generateCb(const sensor_msgs::CompressedImage::ConstPtr &msg);
		// void Image_processing_generateCb(const sensor_msgs::Image::ConstPtr &msg);
	};

	const int thresh = 50, N = 5;
	const double PI = 3.141592653589;
	static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);
	double get_distance(cv::Point vec_1, cv::Point vec_2);
	static void findSquares(const cv::Mat &image, std::vector<cv::Point> &square, double &diagonal_max);
	static void draw_square_circle_cross(cv::Mat &image, const std::vector<cv::Point> &square, cv::Vec3f &circle, std::vector<cv::Vec6d> &lines_k_b, double &diagonal_max);

	/* 定义霍夫线性变换类 */
	class LineFinder
	{
	private:
		cv::Mat img;
		//包含被检测直线的端点的向量
		std::vector<cv::Vec4i> lines;

		//累加器分辨率参数
		double deltaRho;
		double deltaTheta;

		//确认直线之前必须受到的最小投票数
		int minVote;

		//直线的最小长度
		double minLength;
		//直线上允许的最大空隙
		double maxGap;

	public:
		LineFinder() : deltaRho(1), deltaTheta(PI / 180), minVote(10), minLength(0.0), maxGap(0.0) {}
		void setAccResolution(double dRho, double dTheta)
		{
			deltaRho = dRho;
			deltaTheta = dTheta;
		}
		void setminVote(int minv)
		{
			minVote = minv;
		}
		void setLengthAndGap(double length, double gap)
		{
			minLength = length;
			maxGap = gap;
		}
		std::vector<cv::Vec4i> findLines(cv::Mat &binary)
		{
			lines.clear();
			cv::HoughLinesP(binary, lines, deltaRho, deltaTheta, minVote, minLength, maxGap);
			return lines;
		}
	};

	bool cmp(cv::Vec6d a, cv::Vec6d b);
	static void find_cross(const cv::Mat &image, std::vector<cv::Vec6d> &lines_k_b);
	static void find_circle(const cv::Mat &image, cv::Vec3f &circle, std::vector<cv::Point> &square, double diagonal);
	static void find_feature_points(const cv::Mat &image, std::vector<cv::Point> &square, double &diagonal_max, std::vector<cv::Point2f> &feature_points_sort);
	static void draw_square_points(cv::Mat &image, const std::vector<cv::Point> &square, std::vector<cv::Point2f> &feature_points_sort);

	/* 四元数结构体 */
	struct Quaternion
	{
		double w;
		double x;
		double y;
		double z;
	};
	struct Quaternion rotMatrix2Quaternion(cv::Mat M);
	void set_world_points(std::vector<cv::Point3f> &Points3D);
	void set_image_points(std::vector<cv::Point2f> &Points2D, std::vector<cv::Point2f> &feature_points_sort);
	Eigen::MatrixXd bundleAdjustment(const std::vector<cv::Point3f> points_3d, const std::vector<cv::Point2f> points_2d, const cv::Mat &K, cv::Mat &R, cv::Mat &t);
} // namespace detect_points

#endif
