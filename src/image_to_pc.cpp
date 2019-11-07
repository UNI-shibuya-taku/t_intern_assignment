#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
/* #include <opencv2/opencv_lib.hpp> */
/* #include <opencv2/highgui/highgui.hpp> */

class ImageToPC{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		/*publish*/
		ros::Publisher pub_pc;
		/*pcl*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZ>};
		/*objects*/
		cv::Mat img_depth;
		/*parameters*/
		double publish_rate;
		std::string frame_id_name;
		std::string file_path;
		double vertical_upper_range_min;	//[deg]
		double vertical_upper_range_max;	//[deg]
		double vertical_lower_range_min;	//[deg]
		double vertical_lower_range_max;	//[deg]
		double horizontal_range_min;	//[deg]
		double horizontal_range_max;	//[deg]
		double vertical_upper_resolution;	//[deg]
		double vertical_lower_resolution;	//[deg]
		double depth_max;
		int depth_step;
	public:
		ImageToPC();
		void LoopExecution(void);
		void LoadImage(int number);
		void InputPC(void);
		void GetXYZ(int row, int col, double depth, double& x, double& y, double& z);
		void Publication(void);
		double DegToRad(double deg);
};

ImageToPC::ImageToPC()
	:nhPrivate("~")
{
	pub_pc = nh.advertise<sensor_msgs::PointCloud2>("/cloud", 1);

	nhPrivate.param("publish_rate", publish_rate, 10.0);
	std::cout << "publish_rate = " << publish_rate << std::endl;
	nhPrivate.param("frame_id_name", frame_id_name, std::string("/lidar"));
	std::cout << "frame_id_name = " << frame_id_name << std::endl;
	nhPrivate.param("file_path", file_path, std::string("/path"));
	std::cout << "file_path = " << file_path << std::endl;
	nhPrivate.param("vertical_upper_range_min", vertical_upper_range_min, -8.33);
	std::cout << "vertical_upper_range_min = " << vertical_upper_range_min << std::endl;
	nhPrivate.param("vertical_upper_range_max", vertical_upper_range_max, 2.0);
	std::cout << "vertical_upper_range_max = " << vertical_upper_range_max << std::endl;
	nhPrivate.param("vertical_lower_range_min", vertical_lower_range_min, -24.33);
	std::cout << "vertical_lower_range_min = " << vertical_lower_range_min << std::endl;
	nhPrivate.param("vertical_lower_range_max", vertical_lower_range_max, -8.53);
	std::cout << "vertical_lower_range_max = " << vertical_lower_range_max << std::endl;
	nhPrivate.param("horizontal_range_min", horizontal_range_min, -180.0);
	std::cout << "horizontal_range_min = " << horizontal_range_min << std::endl;
	nhPrivate.param("horizontal_range_max", horizontal_range_max, 180.0);
	std::cout << "horizontal_range_max = " << horizontal_range_max << std::endl;
	nhPrivate.param("vertical_upper_resolution", vertical_upper_resolution, {1.0/3.0});
	std::cout << "vertical_upper_resolution = " << vertical_upper_resolution << std::endl;
	nhPrivate.param("vertical_lower_resolution", vertical_lower_resolution, {1.0/2.0});
	std::cout << "vertical_lower_resolution = " << vertical_lower_resolution << std::endl;
	nhPrivate.param("depth_max", depth_max, 100.0);
	std::cout << "depth_max = " << depth_max << std::endl;
	nhPrivate.param("depth_step", depth_step, 65536);
	std::cout << "depth_step = " << depth_step << std::endl;
}

void ImageToPC::LoopExecution(void)
{
	ros::Rate loop_rate(publish_rate);
	int counter = 0;
	while(ros::ok()){
		LoadImage(counter);
		InputPC();
		Publication();

		counter++;

		ros::spinOnce();
		loop_rate.sleep();
	}
}

void ImageToPC::LoadImage(int number)
{
	std::string file_name_depth = file_path + "/depth/" + std::to_string(number) + "_depth.png";
	img_depth = cv::imread(file_name_depth);
	/* std::cout << "img_depth = " << img_depth << std::endl; */
	/* std::cout << "img_depth.rows = " << img_depth.rows << std::endl; */
	/* std::cout << "img_depth.cols = " << img_depth.cols << std::endl; */
	if(img_depth.empty()){
		std::cout << file_name_depth  << " cannot be opened" << std::endl;
		exit(1);
	}
}

void ImageToPC::InputPC(void)
{
	cloud->points.clear();
	for(size_t i=0;i<img_depth.rows;++i){
		for(size_t j=0;j<img_depth.cols;++j){
			double x, y, z;
			double depth = img_depth.at<unsigned short>(i, j)*depth_max/(double)depth_step;
			/* std::cout << "depth = "  << depth << std::endl; */
			/* std::cout << "img_depth.at<unsigned short>(i, j) = "  << img_depth.at<unsigned short>(i, j) << std::endl; */
			GetXYZ(i, j, depth, x, y, z);
			pcl::PointXYZ tmp;
			tmp.x = x;
			tmp.y = y;
			tmp.z = z;
			cloud->points.push_back(tmp);
		}
	}
}

void ImageToPC::GetXYZ(int row, int col, double depth, double& x, double& y, double& z)
{
	double angle_h = (img_depth.cols/2.0 - col)*DegToRad(horizontal_range_max - horizontal_range_min)/(double)img_depth.cols;
	double angle_v;
	int vertical_delimit_index = (vertical_upper_range_max - vertical_upper_range_min)/vertical_upper_resolution + 2;
	/* std::cout << "vertical_delimit_index = "  << vertical_delimit_index << std::endl; */
	if(row < vertical_delimit_index)	angle_v = DegToRad(vertical_upper_range_max - row*vertical_upper_resolution);	//upper
	else	angle_v = DegToRad(vertical_lower_range_max - (row - vertical_delimit_index)*vertical_lower_resolution);	//lower
	if(fabs(angle_h) > M_PI){
		std::cout << fabs(angle_h) << " > M_PI" << std::endl;
		exit(1);
	}

	x = depth*cos(angle_v)*cos(angle_h);
	y = depth*cos(angle_v)*sin(angle_h);
	z = depth*sin(angle_v);
}

void ImageToPC::Publication(void)
{
	/*pc*/
	sensor_msgs::PointCloud2 pc_ros;
	pcl::toROSMsg(*cloud, pc_ros);
	pc_ros.header.frame_id = frame_id_name;
	pc_ros.header.stamp = ros::Time::now();
	pub_pc.publish(pc_ros);	
}

double ImageToPC::DegToRad(double deg)
{
	return deg/180.0*M_PI;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_to_pc");
	
	ImageToPC image_to_pc;
	image_to_pc.LoopExecution();
}
