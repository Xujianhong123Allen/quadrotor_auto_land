/**********************************************************
 * Author        : Jianhong XU
 * Email         : jianhongxu1006@gmail.com
 * Last modified : 2021年04月22日 星期四 21时33分46秒 
 * Description   : 识别目标（正方形二维码），PnP估计位姿
**********************************************************/

#include "detect_points/detect_points.h"

namespace detect_points
{
	/* 初始化相机参数 */
	/* 内参矩阵K */
	double camD[9] = {
		544.880238, 0.000000, 320.466312,
		0.000000, 544.687423, 180.888460, 
		0.000000, 0.000000, 1.000000};

	cv::Mat camera_matrix = cv::Mat(3, 3, CV_64FC1, camD);
	/* 畸变参数D */
	double distCoeffD[5] = {0.000339, -0.001901, 0.000110, -0.000168, 0.000000};
	cv::Mat distortion_coefficients = cv::Mat(5, 1, CV_64FC1, distCoeffD);

	Image_processing::Image_processing(ros::NodeHandle &nh, ros::NodeHandle nh_private)
	{
		ROS_INFO("Detection feature points node is running...");

		Image_raw_sub_ = nh.subscribe("/camera/image_raw/compressed", 1, &Image_processing::Image_processing_generateCb, this);
		// Image_raw_sub_ = nh.subscribe("/iris/usb_cam/image_raw", 1, &Image_processing::Image_processing_generateCb, this);
		// Image_raw_sub_  = nh.subscribe( "/camera/image_raw",1,&Image_processing::Image_processing_generateCb,image_transport::TransportHints("compressed"))
		Image_pub_ = nh.advertise<sensor_msgs::Image>("/quadrotor/track_image", 1, true);
		Pose_pub_ = nh.advertise<geometry_msgs::Pose>("/quadrotor/pose", 1, true);

		set_world_points(Points3D);
	}

	Image_processing::~Image_processing()
	{
		ROS_INFO("Detection feature points node quits!");
	}

	void Image_processing::Image_processing_generateCb(const sensor_msgs::CompressedImage::ConstPtr &msg)
	// void Image_processing::Image_processing_generateCb(const sensor_msgs::Image::ConstPtr &msg)
	{
		cv_bridge::CvImagePtr cv_image_src;
		cv_image_src = cv_bridge::toCvCopy(msg, "bgr8");

		/* cv::Mat cv_image_des;
		cv::medianBlur(cv_image_src->image, cv_image_src->image, 5);
		// std::cout << cv_image_src->encoding << std::endl;
		cv::cvtColor(cv_image_src->image, cv_image_des, cv::COLOR_BGR2GRAY);
		cv::GaussianBlur(cv_image_des, cv_image_des, cv::Size(9, 9), 2, 2);
		cv::Canny(cv_image_des, cv_image_des, 10, 250, 5);
		std::vector<cv::Vec4f> circles; // 三维列向量表示一个圆，(x,y,r)

		// //霍夫圆变换函数
		cv::HoughCircles(cv_image_des, circles, cv::HOUGH_GRADIENT, 1, 30, 100, 33, int(cv_image_des.rows / 30), (cv_image_des.rows / 3));

		for (size_t i = 0; i < circles.size(); i++)
		{
			cv::Point circle_center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			int radius = cvRound(circles[i][2]);
			//绘制圆心
			cv::circle(cv_image_src->image, circle_center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
			//绘制圆轮廓
			cv::circle(cv_image_src->image, circle_center, radius, cv::Scalar(0, 255, 255), 3, 8, 0);
		}

		if (circles.size() == 4)
		{
			//判断四个检测点是否构成多边形
			//计算多边形的形心
			cv::Point2f polygon_center(0, 0);
			for (size_t i = 0; i < circles.size(); i++)
			{
				polygon_center.x += circles[i][0];
				polygon_center.y += circles[i][1];
			}
			polygon_center.x = polygon_center.x / circles.size();
			polygon_center.y = polygon_center.y / circles.size();

			//计算每一个特征点与形心的距离
			for (size_t i = 0; i < circles.size(); i++)
			{
				circles[i][3] = sqrt((circles[i][0] - polygon_center.x) * (circles[i][0] - polygon_center.x) +
									 (circles[i][1] - polygon_center.y) * (circles[i][1] - polygon_center.y));
			}
			//依据距离进行排序
			std::sort(circles.begin(), circles.end(), cmp);

			//检测其余三个特征点与形心是否符合要求
			int detect_polygon_num = 0;
			for (size_t i = 0; i < circles.size() - 2; i++)
			{
				if(fabs((circles[i][2] - circles[i+1][2]) < 0.2 * circles[i][2])
					&& sqrt( (circles[i][0]-circles[i+1][0])*(circles[i][0]-circles[i+1][0]) +
							(circles[i][1]-circles[i+1][1])*(circles[i][1]-circles[i+1][1])) < 3*circles[i][2])
				{
					detect_polygon_num++;
				}
			}
			if (detect_polygon_num > 1) //符合要求
			{
				//根据与3号点最短距离找出0号点
				double min_distance = cv_image_src->image.rows;
				double max_distance = 0.0;
				int dis_min_num = -1;
				int dis_max_num = -1;

				for (size_t i = 0; i < circles.size() - 1; i++)
				{
					double distance = sqrt((circles[i][0] - circles[3][0]) * (circles[i][0] - circles[3][0]) +
										   (circles[i][1] - circles[3][1]) * (circles[i][1] - circles[3][1]));
					if (distance < min_distance)
					{
						min_distance = distance;
						dis_min_num = i;
					}
					if (distance > max_distance)
					{
						max_distance = distance;
						dis_max_num = i;
					}
				}
				std::cout << "最小距离号：" << dis_min_num << "\t最大距离号：" << dis_max_num << std::endl;
				std::cout << "最小距离：" << min_distance << "\t最大距离：" << max_distance << std::endl;

				if (dis_min_num != 0)
				{
					swap(circles[dis_min_num], circles[0]);
				}
				if (dis_max_num != 1)
				{
					swap(circles[dis_max_num], circles[1]);
				}

				for (size_t i = 0; i < circles.size(); i++)
				{
					cv::Point circle_center(cvRound(circles[i][0]), cvRound(circles[i][1]));
					int radius = cvRound(circles[i][2]);
					switch (i)
					{
					case 0:
						circle(cv_image_src->image, circle_center, 3, cv::Scalar(255, 0, 0), -1, 8, 0);
						circle(cv_image_src->image, circle_center, radius, cv::Scalar(255, 0, 0), 3, 8, 0);
						break;
					case 1:
						circle(cv_image_src->image, circle_center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
						circle(cv_image_src->image, circle_center, radius, cv::Scalar(0, 255, 0), 3, 8, 0);
						break;
					case 2:
						circle(cv_image_src->image, circle_center, 3, cv::Scalar(0, 0, 255), -1, 8, 0);
						circle(cv_image_src->image, circle_center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
						break;
					case 3:
						circle(cv_image_src->image, circle_center, 3, cv::Scalar(255, 255, 0), -1, 8, 0);
						circle(cv_image_src->image, circle_center, radius, cv::Scalar(255, 255, 0), 3, 8, 0);
						break;
					}

					std::cout << i << ".x: " << circles[i][0] << "\ty: " << circles[i][1] << "\tr: " << circles[i][2]
							  << "\td: " << circles[i][3] << std::endl;
				}
				std::cout << std::endl;
			}
		} */

		// 检测目标正方形square
		std::vector<cv::Point> square;
		double diagonal_max = 0;
		findSquares(cv_image_src->image, square, diagonal_max);

		/* // 检测目标圆circle
		cv::Vec3f circle;
		find_circle(cv_image_src->image, circle, square, diagonal_max);

		// 检测交叉点cross
		std::vector<cv::Vec6d> lines_k_b;
		find_cross(cv_image_src->image, lines_k_b);

		// 绘制检测的正方形（中心）、圆（心）、交叉点
		draw_square_circle_cross(cv_image_src->image, square, circle, lines_k_b, diagonal_max); */

		//	检测目标特征点
		std::vector<cv::Point2f> feature_points_sort;
		find_feature_points(cv_image_src->image, square, diagonal_max, feature_points_sort);

		//	绘制检测的正方形（中心）、特征点
		draw_square_points(cv_image_src->image, square, feature_points_sort);

		// 估计位姿
		Points2D.clear();
		set_image_points(Points2D, feature_points_sort);
		if ((Points2D.size() == 4) && (Points3D.size() == 4))
		{
			// Quaternion q;
			cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
			// estimate_pose(Points3D, Points2D, camera_matrix, distortion_coefficients, q, tvec);
			cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
			double rm[9];
			cv::Mat rotM(3, 3, CV_64FC1, rm);
			solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, cv::SOLVEPNP_P3P);
			Rodrigues(rvec, rotM); //罗德里格斯公式，旋转向量转化为旋转矩阵
			// Quaternion quat;
			// quat = rotMatrix2Quaternion(rotM);
			// std::cout << "优化前的位姿：" << std::endl;
			// std::cout << "四元数:\nw:" << quat.w << "\tx:" << quat.x
			// 		  << "\ty:" << quat.y << "\tz:" << quat.z << std::endl;
			// std::cout << "平移向量：\n" << tvec << std::endl;

			/* //利用g2o求解位姿，并用bundle Adjustment优化
			VecVector3d pts_3d_eigen;
			VecVector2d pts_2d_eigen;
			for (size_t i = 0; i < Points3D.size(); ++i)
			{
				pts_3d_eigen.push_back(Eigen::Vector3d(Points3D[i].x, Points3D[i].y, Points3D[i].z));
				pts_2d_eigen.push_back(Eigen::Vector2d(Points2D[i].x, Points2D[i].y));
			}
			Sophus::SE3d pose_g2o;
			bundleAdjustmentG2O(pts_3d_eigen, pts_2d_eigen, camera_matrix, pose_g2o); */

			// 利用g2o进行bundle Adjustment优化
			Eigen::MatrixXd T_optimized = bundleAdjustment(Points3D, Points2D, camera_matrix, rotM, tvec);
			Eigen::Matrix3d R_optimized;
			Eigen::Vector3d t_optimized;
			R_optimized << T_optimized(0, 0), T_optimized(0, 1), T_optimized(0, 2), T_optimized(1, 0), T_optimized(1, 1), T_optimized(1, 2),
				T_optimized(2, 0), T_optimized(2, 1), T_optimized(2, 2);
			t_optimized << T_optimized(0, 3), T_optimized(1, 3), T_optimized(2, 3);
			Eigen::Quaterniond quat_optimized = Eigen::Quaterniond(R_optimized);

			geometry_msgs::Pose pose;
			pose.orientation.w = quat_optimized.w();
			pose.orientation.x = quat_optimized.x();
			pose.orientation.y = quat_optimized.y();
			pose.orientation.z = quat_optimized.z();

			pose.position.x = t_optimized(0);
			pose.position.y = t_optimized(1);
			pose.position.z = t_optimized(2);

			std::cout << "优化后的位姿：" << std::endl;
			std::cout << "四元数:\nw:" << pose.orientation.w << "\tx:" << pose.orientation.x
					  << "\ty:" << pose.orientation.y << "\tz:" << pose.orientation.z << std::endl;
			std::cout << "平移向量:\nx:" << pose.position.x << "\ty:" << pose.position.y << "\tz:"
					  << pose.position.z << std::endl
					  << std::endl;

			Pose_pub_.publish(pose);
		}

		Image_pub_.publish(cv_image_src->toImageMsg());
	}

	/* 求出向量夹角的余弦,从pt0->pt1到pt0->pt2 */
	static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
	{
		double dx1 = pt1.x - pt0.x;
		double dy1 = pt1.y - pt0.y;
		double dx2 = pt2.x - pt0.x;
		double dy2 = pt2.y - pt0.y;
		return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
	}

	/* 获取两个点之间的距离 */
	double get_distance(cv::Point vec_1, cv::Point vec_2)
	{
		double distance = sqrt((vec_1.x - vec_2.x) * (vec_1.x - vec_2.x) +
							   (vec_1.y - vec_2.y) * (vec_1.y - vec_2.y));
		return distance;
	}

	/* 返回在图像上检测到的最大正方形，序列存储在指定的内存中 */
	static void findSquares(const cv::Mat &image, std::vector<cv::Point> &square, double &diagonal_max)
	{
		cv::Mat image_ = image;

		std::vector<std::vector<cv::Point>> squares;
		squares.clear();

		//中值滤波将增强边缘检测
		cv::Mat timg(image_);
		cv::medianBlur(image_, timg, 9);
		cv::Mat gray0(timg.size(), CV_8U), gray;

		std::vector<std::vector<cv::Point>> contours;

		// find squares in every color plane of the image
		//在图像的每个颜色平面上找到正方形
		for (int c = 0; c < 3; c++)
		{
			int ch[] = {c, 0};
			cv::mixChannels(&timg, 1, &gray0, 1, ch, 1); //将输入数组的指定通道复制到输出数组的指定通道

			// try several threshold levels
			//尝试几个阈值级别
			for (int l = 0; l < N; l++)
			{
				// Canny帮助捕捉带有渐变阴影的正方形
				if (l == 0)
				{
					cv::Canny(gray0, gray, 5, thresh, 5);
					cv::dilate(gray, gray, cv::Mat(), cv::Point(-1, -1));
				}
				else
				{
					gray = gray0 >= (l + 1) * 255 / N;
				}

				cv::findContours(gray, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

				std::vector<cv::Point> approx;

				for (size_t i = 0; i < contours.size(); i++)
				{

					cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true) * 0.02, true);

					if (approx.size() == 4 &&
						fabs(contourArea(cv::Mat(approx))) > 1000 &&
						isContourConvex(cv::Mat(approx))) //凸性检测 检测一个曲线是不是凸的
					{
						double maxCosine = 0;

						for (int j = 2; j < 5; j++)
						{
							double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
							maxCosine = MAX(maxCosine, cosine);
						}

						if (maxCosine < 0.3)
							squares.push_back(approx);
					}
				}
			}
		}

		for (size_t i = 0; i < squares.size(); i++)
		{
			// std::cout << i << std::endl;
			cv::Point square_center(0, 0);
			for (size_t j = 0; j < squares[i].size(); j++)
			{
				// std::cout << "x: " << squares[i][j].x << "\ty: " << squares[i][j].y << std::endl;
				square_center.x += squares[i][j].x;
				square_center.y += squares[i][j].y;
			}
			square_center.x = square_center.x / squares[i].size();
			square_center.y = square_center.y / squares[i].size();
			// std::cout << "x: " << square_center.x << "\ty: " << square_center.y << std::endl;
			squares[i].push_back(square_center);
			// std::cout << squares[i].size() << std::endl
			// 		  << std::endl;
		}
		//使用基于密度的方法检测目标矩形中心和是否为正方形来剔除多余的多边形
		int detect_square_num = squares.size();
		for (std::vector<std::vector<cv::Point>>::iterator it = squares.begin(); it != squares.end();)
		{
			int center_num = 0;

			for (std::vector<std::vector<cv::Point>>::iterator jt = squares.begin(); jt != squares.end(); jt++)
			{
				if (get_distance((*it)[4], (*jt)[4]) < 10) //使用密度的方法检测目标矩形中心
				{
					center_num++;
				}
			}
			int length_error_num = 0;
			double length[3];
			length[0] = get_distance((*it)[0], (*it)[1]);
			length[1] = get_distance((*it)[1], (*it)[2]);
			length[2] = get_distance((*it)[2], (*it)[3]);

			for (size_t k = 0; k < 2; k++)
			{
				if (fabs(length[k] - length[k + 1]) > 10) //判断是否为正方形
				{
					length_error_num++;
				}
			}

			if ((center_num < detect_square_num / 3) || (length_error_num > 0))
			{
				it = squares.erase(it);
			}
			else
			{
				it++;
			}
		}
		// std::cout << "去除多余正方形后的数量： " << squares.size() << std::endl;
		if (squares.size() != 0)
		{
			cv::Point center_average(0, 0);
			for (size_t i = 0; i < squares.size(); i++)
			{
				// for (size_t j = 0; j < squares[i].size(); j++)
				// {
				// 	std::cout << "x: " << squares[i][j].x << "\ty: " << squares[i][j].y << std::endl;
				// }
				// std::cout << std::endl;
				center_average.x += squares[i][4].x;
				center_average.y += squares[i][4].y;
			}
			center_average.x = center_average.x / squares.size();
			center_average.y = center_average.y / squares.size();
			// std::cout << "平均中心点： x: " << center_average.x << "\ty: " << center_average.y << std::endl;

			diagonal_max = 0;
			int square_max_num = -1;
			for (size_t i = 0; i < squares.size(); i++)
			{
				double diagonal = get_distance(squares[i][0], squares[i][4]);
				// double diagonal = sqrt((squares[i][0].x - squares[i][4].x) * (squares[i][0].x - squares[i][4].x) +
				//    (squares[i][0].y - squares[i][4].y) * (squares[i][0].y - squares[i][4].y));
				if (diagonal > diagonal_max)
				{
					diagonal_max = diagonal;
					square_max_num = i;
				}
				// std::cout << i << std::endl;
			}

			square.clear();
			square = squares[square_max_num];
			// std::cout << "squares 个数: " << squares.size() << std::endl;
			// std::cout << "square_max_num:" << square_max_num << std::endl;
			// drawSquares(image, squares);
			// draw_center(image,  center_average);
			// std::cout << "检测目标正方形：" << std::endl;
			// for (size_t i = 0; i < square.size() - 1; i++)
			// {
			// 	std::cout << "x: " << square[i].x << "\ty: " << square[i].y << std::endl;
			// }
			// std::cout << "中心：\nx: " << square[4].x << "\ty: " << square[4].y << std::endl
			// 		  << std::endl;
		}
		else //检测不到目标正方形
		{
			square.clear();
			diagonal_max = 0;
		}
	}

	/* 寻找交叉点 */
	static void find_cross(const cv::Mat &image, std::vector<cv::Vec6d> &lines_k_b)
	{
		// std::cout << "找出交叉点" << std::endl;
		LineFinder cFinder;
		cFinder.setLengthAndGap(80, 20);
		cFinder.setminVote(60);
		cv::Mat contours;
		cv::Canny(image, contours, 350, 400);
		std::vector<cv::Vec4i> lines = cFinder.findLines(contours);
		for (size_t i = 0; i < lines.size(); i++)
		{
			// std::cout << "Lines[0]: " << lines[i][0] << "\tLines[1]: " << lines[i][1] << "\tLines[2]: " << lines[i][2] << "\tLines[3]: " << lines[i][3] << std::endl;
			double k, b;
			k = 1.0 * (lines[i][3] - lines[i][1]) / (lines[i][2] - lines[i][0]);
			b = double(lines[i][1]) - k * lines[i][0];

			cv::Vec6d l_k_b = cv::Vec6d(lines[i][0], lines[i][1], lines[i][2], lines[i][3], k, b);
			lines_k_b.push_back(l_k_b);
		}
		// std::cout << std::endl;
		//依据角度对直线段进行排序
		std::sort(lines_k_b.begin(), lines_k_b.end(), cmp);
		//去除多余直线
		// for (size_t i = 0; i < lines_k_b.size(); i++)
		// {
		// 	std::cout << "Lines[0]: " << lines_k_b[i][0] << "\tLines[1]: " << lines_k_b[i][1] << "\tLines[2]: " << lines_k_b[i][2]
		// 			  << "\tLines[3]: " << lines_k_b[i][3] << "\tk: " << lines_k_b[i][4] << "\tb: " << lines_k_b[i][5] << std::endl;
		// }
		// 检测两条直线斜率k1,k2
		// double k1 = k2 = 0;
		for (std::vector<cv::Vec6d>::iterator it = lines_k_b.begin(); it != lines_k_b.end();)
		{
			int lines_detect_num = 0;
			int lines_error_num = 0;
			for (std::vector<cv::Vec6d>::iterator jt = lines_k_b.begin(); jt != lines_k_b.end(); jt++)
			{
				if (fabs((*it)[4] - (*jt)[4]) < 0.1)
				{
					lines_detect_num++;
				}
				if ((fabs((*it)[4] - (*jt)[4]) < 0.1) && (fabs((*it)[5] - (*jt)[5]) < 3))
				{
					lines_error_num++;
				}
			}
			if ((lines_detect_num < 2) || (lines_error_num > 1))
			{
				it = lines_k_b.erase(it);
			}
			else
			{
				it++;
			}
		}
	}

	/* 寻找圆 */
	static void find_circle(const cv::Mat &image, cv::Vec3f &circle, std::vector<cv::Point> &square, double diagonal)
	{
		if (square.begin() != square.end())
		{
			cv::Mat cimg;
			cv::medianBlur(image, image, 5);
			cv::cvtColor(image, cimg, cv::COLOR_BGR2GRAY);
			cv::GaussianBlur(cimg, cimg, cv::Size(9, 9), 2, 2);
			//   medianBlur(cimg, cimg, 5);
			cv::Canny(cimg, cimg, 10, 250, 5);
			// std::vector<Vec4f> circles_sample;
			// HoughCircles(cimg, circles, HOUGH_GRADIENT, 1, 30, 100, 30, 10, 120);
			std::vector<cv::Vec3f> circles;
			cv::HoughCircles(cimg, circles, cv::HOUGH_GRADIENT, 1, 30, 100, 30, int(diagonal * 0.2), diagonal * 0.6);

			for (std::vector<cv::Vec3f>::iterator it = circles.begin(); it != circles.end();)
			{
				if (sqrt(((*it)[0] - square[4].x) * ((*it)[0] - square[4].x) +
						 ((*it)[1] - square[4].y) * ((*it)[1] - square[4].y)) > 5)
				{
					it = circles.erase(it);
				}
				else
				{
					it++;
				}
			}
			if (circles.size() == 1)
			{
				circle = circles[0];
				// std::cout << "检测目标圆：" << std::endl;
				// std::cout << "x: " << circle[0] << "\ty: " << circle[1] << "\tr: " << circles[2] << std::endl
				// 		  << std::endl;
			}
		}
	}

	/* 绘制检测的正方形（中心）、圆（心）、交叉点 */
	static void draw_square_circle_cross(cv::Mat &image, const std::vector<cv::Point> &square, cv::Vec3f &circle, std::vector<cv::Vec6d> &lines_k_b, double &diagonal_max)
	{
		if (square.begin() != square.end())
		{
			const cv::Point *p = &square[0];
			int n = 4;
			//dont detect the border
			if (p->x > 3 && p->y > 3)
				// polylines(image, &p, 4, 1, true, Scalar(0, 0, 255), 2, LINE_AA);
				polylines(image, &p, &n, 1, true, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
		}

		cv::Point center(cvRound(circle[0]), cvRound(circle[1]));
		int radius = cvRound(circle[2]);

		cv::circle(image, center, radius, cv::Scalar(255, 0, 0), 3, 8, 0);

		// std::cout << "去除直线后" << std::endl;
		// for (size_t i = 0; i < lines_k_b.size(); i++)
		// {
		// 	std::cout << "Lines[0]: " << lines_k_b[i][0] << "\tLines[1]: " << lines_k_b[i][1] << "\tLines[2]: " << lines_k_b[i][2]
		// 			  << "\tLines[3]: " << lines_k_b[i][3] << "\tk: " << lines_k_b[i][4] << "\tb: " << lines_k_b[i][5] << std::endl;
		// }
		if (lines_k_b.size() == 4) //如果检测出4条直线，则计算交叉中心cross,并绘制
		{
			double k_average[2], b_average[2];
			for (size_t i = 0; i < (lines_k_b.size() / 2); i++)
			{
				k_average[i] = 0;
				b_average[i] = 0;

				k_average[i] += lines_k_b[2 * i][4] + lines_k_b[2 * i + 1][4];
				b_average[i] += lines_k_b[2 * i][5] + lines_k_b[2 * i + 1][5];

				k_average[i] /= (lines_k_b.size() / 2);
				b_average[i] /= (lines_k_b.size() / 2);

				// std::cout << "k: " << k_average[i] << "\tb: " << b_average[i] << std::endl;
			}
			cv::Point2d cross_center;
			cross_center.x = (b_average[1] - b_average[0]) / (k_average[0] - k_average[1]);
			cross_center.y = k_average[0] * cross_center.x + b_average[0];
			// std::cout << "交叉点： x: " << cross_center.x << "\ty: " << cross_center.y << std::endl;

			int line_length = diagonal_max * 5.0 / 14.4;
			cv::circle(image, cross_center, 7, cv::Scalar(0, 255, 0), -1, 8, 0);

			for (size_t i = 0; i < (lines_k_b.size() / 2); i++)
			{
				cv::Point2d p1((cross_center.x + line_length * (1.0 / sqrt(1 + k_average[i] * k_average[i]))), (cross_center.x + line_length * (1.0 / sqrt(1 + k_average[i] * k_average[i]))) * k_average[i] + b_average[i]);
				cv::Point2d p2((cross_center.x - line_length * (1.0 / sqrt(1 + k_average[i] * k_average[i]))), (cross_center.x - line_length * (1.0 / sqrt(1 + k_average[i] * k_average[i]))) * k_average[i] + b_average[i]);
				cv::line(image, p1, p2, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
			}
		}

		// 绘制正方形中心
		if (square.begin() != square.end())
		{
			cv::circle(image, square[4], 3, cv::Scalar(0, 0, 255), -1, 8, 0);
		}
		// 绘制圆心
		cv::circle(image, center, 3, cv::Scalar(255, 0, 0), -1, 8, 0);
		// imshow(wndname, image);
	}

	/* stl vector排序比较函数，根据斜率k值大小排序 */
	bool cmp(cv::Vec6d a, cv::Vec6d b)
	{
		return a[4] < b[4];
	}

	static void find_feature_points(const cv::Mat &image, std::vector<cv::Point> &square, double &diagonal_max, std::vector<cv::Point2f> &feature_points_sort)
	{
		if (square.begin() != square.end())
		{

			cv::Mat img;
			cv::GaussianBlur(image, img, cv::Size(3, 3), 0);
			cv::Canny(image, img, 100, 250);

			std::vector<std::vector<cv::Point>> contours;
			std::vector<cv::Vec4i> hierarchy;

			findContours(img, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE, cv::Point());
			cv::Mat imageContours = cv::Mat::zeros(img.size(), CV_8UC1);
			cv::Mat Contours = cv::Mat::zeros(img.size(), CV_8UC1); //绘制

			std::vector<cv::Point2f> feature_points;
			for (int i = 0; i < contours.size(); i++)
			{
				//contours[i]代表的是第i个轮廓，contours[i].size()代表的是第i个轮廓上所有的像素点数
				// std::cout << i << std::endl;
				cv::Point2f contour_center = cv::Point2f(0, 0);
				if (contours[i].size() >= 8)
				{
					for (int j = 0; j < contours[i].size(); j++)
					{
						//绘制出contours向量所有的像素点
						cv::Point P = cv::Point(contours[i][j].x, contours[i][j].y);
						Contours.at<uchar>(P) = 255;
						// std::cout << "x: " << contours[i][j].x << " y: " << contours[i][j].y << std::endl;
						contour_center.x += contours[i][j].x;
						contour_center.y += contours[i][j].y;
					}

					contour_center.x = contour_center.x / contours[i].size();
					contour_center.y = contour_center.y / contours[i].size();
					// std::cout << "中心： x: " << contour_center.x << " y: " << contour_center.y << std::endl
					// 		  << std::endl;
				}

				double distance = get_distance(contour_center, square[4]);
				if ((distance < (diagonal_max / sqrt(2))) && (distance > 8))
				{
					feature_points.push_back(contour_center);
				}
				// //输出hierarchy向量内容
				// char ch[256];
				// sprintf(ch, "%d", i);
				// string str = ch;
				// // std::cout << "向量hierarchy的第" << str << "个元素内容为：" << hierarchy[i] << std::endl
				// //      << std::endl;

				//绘制轮廓
				// cv::drawContours(imageContours, contours, i, Scalar(255), 1, 8, hierarchy);
			}
			// cv::imshow("Contours Image CV_TRET_TREE", imageContours);		//轮廓
			// cv::imshow("Point of Contours CV_CHAIN_APPROX_NONE", Contours); //向量contours内保存的所有轮廓点集

			// std::cout << "feature_points.size: " << feature_points.size() << std::endl;
			for (std::vector<cv::Point2f>::iterator it = feature_points.begin(); it != feature_points.end();)
			{
				int center_num = 0;

				for (std::vector<cv::Point2f>::iterator jt = feature_points.begin(); jt != feature_points.end(); jt++)
				{
					if (get_distance((*it), (*jt)) < 5)
					{
						center_num++;
					}
				}

				if (center_num > 1)
				{
					it = feature_points.erase(it);
				}
				else
				{
					it++;
				}
			}
			// std::cout << "feature_points.size: " << feature_points.size() << std::endl;

			if (feature_points.size() == 4)
			{
				/* 对四个特征点进行排序 */
				feature_points_sort.clear();
				// 计算四个围成多边形的形心
				cv::Point2f polygon_center = cv::Point2f(0, 0);
				for (size_t i = 0; i < feature_points.size(); i++)
				{
					polygon_center.x += feature_points[i].x;
					polygon_center.y += feature_points[i].y;
				}
				polygon_center.x /= feature_points.size();
				polygon_center.y /= feature_points.size();
				// std::cout << "Polygon_center:" << std::endl;
				// std::cout << "x: " << polygon_center.x << " y: " << polygon_center.y << std::endl;

				//根据距离特征点与形心寻找特征点0
				double distance_max = 0;
				int distance_max_num = -1;
				for (size_t i = 0; i < feature_points.size(); i++)
				{
					if (distance_max < get_distance(feature_points[i], polygon_center))
					{
						distance_max = get_distance(feature_points[i], polygon_center);
						distance_max_num = i;
					}
				}
				feature_points_sort.push_back(feature_points[distance_max_num]);

				//根据第特征点0、形心、其余特征点的夹角余弦来寻找特征点1
				for (size_t i = 0; i < feature_points.size(); i++)
				{
					if (i != distance_max_num)
					{
						if (angle(feature_points[i], polygon_center, feature_points[distance_max_num]) > 0.97)
						{
							feature_points_sort.push_back(feature_points[i]);
							break;
						}
					}
				}

				//根据向量叉乘寻找特征点2、3
				int feature_point2_num = -1;
				int feature_point3_num = -1;
				cv::Vec3f detect_vec((feature_points_sort[0].x - feature_points_sort[1].x), (feature_points_sort[0].y - feature_points_sort[1].y), 0);
				// std::cout << "detect_vec: " << detect_vec << std::endl;
				for (size_t i = 0; i < feature_points.size(); i++)
				{
					if ((feature_points[i].x != feature_points_sort[0].x) && (feature_points[i].x != feature_points_sort[1].x))
					{
						cv::Vec3f vec((feature_points[i].x - feature_points_sort[1].x), (feature_points[i].y - feature_points_sort[1].y), 0);
						cv::Vec3f vn = detect_vec.cross(vec);
						// std::cout << "vn: " << vn << std::endl;
						if (vn[2] > 0)
						{
							feature_point2_num = i;
						}
						else
						{
							feature_point3_num = i;
						}
					}
				}
				if ((feature_point3_num != -1) && (feature_point2_num != -1))
				{
					feature_points_sort.push_back(feature_points[feature_point2_num]);
					feature_points_sort.push_back(feature_points[feature_point3_num]);
				}

				// std::cout << "Feature Points 排序后： " << std::endl;
				// for (size_t i = 0; i < feature_points_sort.size(); i++)
				// {
				// 	std::cout << "x: " << feature_points_sort[i].x << " y: " << feature_points_sort[i].y << std::endl;
				// }
			}
		}
	}

	static void draw_square_points(cv::Mat &image, const std::vector<cv::Point> &square, std::vector<cv::Point2f> &feature_points_sort)
	{
		if (square.begin() != square.end())
		{
			const cv::Point *p = &square[0];

			int n = 4;
			//dont detect the border
			if (p->x > 3 && p->y > 3)
				// polylines(image, &p, 4, 1, true, Scalar(0, 0, 255), 2, LINE_AA);
				polylines(image, &p, &n, 1, true, cv::Scalar(255, 255, 0), 3, cv::LINE_AA);

			cv::circle(image, square[4], 5, cv::Scalar(255, 255, 0), -1, 8, 0);
		}
		//绘制特征点
		if (feature_points_sort.size() == 4)
		{
			for (size_t i = 0; i < feature_points_sort.size(); i++)
			{
				switch (i)
				{
				case 0:
					cv::circle(image, feature_points_sort[i], 5, cv::Scalar(255, 0, 0), -1, 8, 0);
					break;
				case 1:
					cv::circle(image, feature_points_sort[i], 5, cv::Scalar(0, 255, 0), -1, 8, 0);
					break;
				case 2:
					cv::circle(image, feature_points_sort[i], 5, cv::Scalar(0, 0, 255), -1, 8, 0);
					break;
				case 3:
					cv::circle(image, feature_points_sort[i], 5, cv::Scalar(0, 255, 255), -1, 8, 0);
					break;
				}
			}
		}
	}

	/* 从旋转矩阵变换到四元数 */
	struct Quaternion rotMatrix2Quaternion(cv::Mat M)
	{
		double t;

		int i = 0, j = 1, k = 2;

		if (M.at<double>(1, 1) > M.at<double>(0, 0))
			i = 1, j = 2, k = 0;

		if (M.at<double>(2, 2) > M.at<double>(i, i))
			i = 2;
		j = 0;
		k = 1;

		t = M.at<double>(i, i) - M.at<double>(j, j) + M.at<double>(k, k) + 1;

		double q[4];
		q[0] = M.at<double>(k, j) - M.at<double>(j, k);
		q[i + 1] = t;
		q[j + 1] = M.at<double>(i, j) + M.at<double>(i, k);
		q[k + 1] = M.at<double>(k, i) + M.at<double>(i, k);

		static struct Quaternion q_;

		q_.w = q[0] * 0.5 / sqrt(t);
		q_.x = q[1] * 0.5 / sqrt(t);
		q_.y = q[2] * 0.5 / sqrt(t);
		q_.z = q[3] * 0.5 / sqrt(t);

		return q_; //q = [qw qx qy qz]
	}

	/* 设置世界坐标点 */
	void set_world_points(std::vector<cv::Point3f> &Points3D)
	{
		// Points3D.push_back(cv::Point3f(205, 314.33, 0));	  //P1 三维坐标的单位是毫米
		// Points3D.push_back(cv::Point3f(205, 95.67, 0));	  //P2
		// Points3D.push_back(cv::Point3f(109.33, 177.67, 0)); //P3
		// Points3D.push_back(cv::Point3f(300.67, 177.67, 0));	  //P4
		// Points3D.push_back(cv::Point3f(314.33, 205, 0));	//P1 三维坐标的单位是毫米
		// Points3D.push_back(cv::Point3f(95.67, 205, 0));		//P2
		// Points3D.push_back(cv::Point3f(177.67, 109.33, 0)); //P3
		// Points3D.push_back(cv::Point3f(177.67, 300.67, 0)); //P4
		Points3D.push_back(cv::Point3f(145, 67.67, 0));	  //P1 三维坐标的单位是毫米   真实降落板尺寸
		Points3D.push_back(cv::Point3f(145, 222.33, 0));	  //P2
		Points3D.push_back(cv::Point3f(87, 125, 0)); //P3
		Points3D.push_back(cv::Point3f(203, 125, 0));	  //P4
	}

	/* 设置图像座标点 */
	void set_image_points(std::vector<cv::Point2f> &Points2D, std::vector<cv::Point2f> &feature_points_sort)
	{
		if (feature_points_sort.size() == 4)
		{
			for (size_t i = 0; i < feature_points_sort.size(); i++)
			{
				Points2D.push_back(cv::Point2f(feature_points_sort[i].x, feature_points_sort[i].y));
			}
		}
	}

	Eigen::MatrixXd bundleAdjustment(const std::vector<cv::Point3f> points_3d, const std::vector<cv::Point2f> points_2d, const cv::Mat &K, cv::Mat &R, cv::Mat &t)
	{
		// 初始化g2o
		typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Block;												  // pose 维度为 6, landmark 维度为 3
		std::unique_ptr<Block::LinearSolverType> linearSolver(new g2o::LinearSolverCSparse<Block::PoseMatrixType>()); // 线性方程求解器
		std::unique_ptr<Block> solver_ptr(new Block(std::move(linearSolver)));										  // 矩阵块求解器
		g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
		g2o::SparseOptimizer optimizer;
		optimizer.setAlgorithm(solver);

		// vertex
		g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap(); // camera pose
		Eigen::Matrix3d R_mat;
		R_mat << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
			R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
		pose->setId(0);
		pose->setEstimate(g2o::SE3Quat(R_mat, Eigen::Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0))));
		optimizer.addVertex(pose);

		int index = 1;
		for (const cv::Point3f p : points_3d) // landmarks
		{
			g2o::VertexSBAPointXYZ *point = new g2o::VertexSBAPointXYZ();
			point->setId(index++);
			point->setEstimate(Eigen::Vector3d(p.x, p.y, p.z));
			point->setMarginalized(true); // g2o 中必须设置 marg 参见第十讲内容
			optimizer.addVertex(point);
		}

		// parameter: camera intrinsics
		g2o::CameraParameters *camera = new g2o::CameraParameters(K.at<double>(0, 0), Eigen::Vector2d(K.at<double>(0, 2), K.at<double>(1, 2)), 0);
		camera->setId(0);
		optimizer.addParameter(camera);

		// edges
		index = 1;
		for (const cv::Point2f p : points_2d)
		{
			g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();
			edge->setId(index);
			edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(index)));
			edge->setVertex(1, pose);
			edge->setMeasurement(Eigen::Vector2d(p.x, p.y));
			edge->setParameterId(0, 0);
			edge->setInformation(Eigen::Matrix2d::Identity());
			optimizer.addEdge(edge);
			index++;
		}

		// chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
		optimizer.setVerbose(true);
		optimizer.initializeOptimization();
		optimizer.optimize(100);
		// chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
		// chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
		// std::cout<<"optimization costs time: "<<time_used.count() <<" seconds."<<endl;

		// std::cout << "after optimization:" << std::endl;
		// std::cout << "T=\n"
		// 		  << Eigen::Isometry3d(pose->estimate()).matrix() << std::endl;

		return Eigen::Isometry3d(pose->estimate()).matrix();
	}

} // namespace detect_points
