#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <pcl/visualization/cloud_viewer.h>

class ImageToPC{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		/*publish*/
		ros::Publisher pub_pc;
		/*pcl*/
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZINormal>};
		pcl::visualization::PCLVisualizer viewer {"image_to_pc"};
		/*objects*/
		cv::Mat img_depth;
		cv::Mat img_intensity;
		cv::Mat img_normal;
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
		void GetXYZNormal(int row, int col, double depth, double& x, double& y, double& z, double& nx, double& ny, double& nz);
		void Publication(void);
		void Visualization(void);
		double DegToRad(double deg);
		double PiToPi(double angle);
};

ImageToPC::ImageToPC()
	:nhPrivate("~")
{
	pub_pc = nh.advertise<sensor_msgs::PointCloud2>("/cloud", 1);

	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(1.0, "axis");
	viewer.setCameraPosition(0.0, 0.0, 100.0, 0.0, 0.0, 0.0);

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
		Visualization();

		counter++;

		ros::spinOnce();
		loop_rate.sleep();
	}
}

void ImageToPC::LoadImage(int number)
{
	/*depth*/
	std::string file_name_depth = file_path + "/depth/" + std::to_string(number) + "_depth.png";
	img_depth = cv::imread(file_name_depth, -1);
	if(img_depth.empty()){
		std::cout << file_name_depth  << " cannot be opened" << std::endl;
		exit(1);
	}
	/*intensity*/
	std::string file_name_intensity = file_path + "/intensity/" + std::to_string(number) + "_intensity.png";
	img_intensity = cv::imread(file_name_intensity, 0);
	if(img_intensity.empty()){
		std::cout << file_name_intensity  << " cannot be opened" << std::endl;
		exit(1);
	}
	/*normal*/
	std::string file_name_normal = file_path + "/normal/" + std::to_string(number) + "_normal.png";
	img_normal = cv::imread(file_name_normal, 1);
	if(img_normal.empty()){
		std::cout << file_name_normal  << " cannot be opened" << std::endl;
		exit(1);
	}
}

void ImageToPC::InputPC(void)
{
	cloud->points.clear();
	for(int i=0;i<img_depth.rows;++i){
		for(int j=0;j<img_depth.cols;++j){
			double x, y, z, nx, ny, nz;
			double depth = img_depth.at<unsigned short>(i, j)*depth_max/(double)depth_step;
			/* std::cout << "depth = "  << depth << std::endl; */
			/* std::cout << "img_depth.at<unsigned short>(i, j) = "  << img_depth.at<unsigned short>(i, j) << std::endl; */
			GetXYZNormal(i, j, depth, x, y, z, nx, ny, nz);
			pcl::PointXYZINormal tmp;
			tmp.x = x;
			tmp.y = y;
			tmp.z = z;
			tmp.normal_x = nx;
			tmp.normal_y = ny;
			tmp.normal_z = nz;
			tmp.intensity = img_intensity.at<unsigned char>(i, j);
			cloud->points.push_back(tmp);
		}
	}
}

void ImageToPC::GetXYZNormal(int row, int col, double depth, double& x, double& y, double& z, double& nx, double& ny, double& nz)
{
	/*xyz*/
	double angle_h = DegToRad((img_depth.cols/2 - col)*(horizontal_range_max - horizontal_range_min)/(double)img_depth.cols);
	double angle_v;
	int vertical_delimit_index = img_depth.rows/2;
	if(row < vertical_delimit_index)	angle_v = DegToRad(vertical_upper_range_max - row*vertical_upper_resolution);	//upper
	else	angle_v = DegToRad(vertical_lower_range_max - (row - vertical_delimit_index)*vertical_lower_resolution);	//lower
	if(fabs(angle_h) > M_PI){
		std::cout << fabs(angle_h) << " > M_PI" << std::endl;
		exit(1);
	}
	x = depth*cos(angle_v)*cos(angle_h);
	y = depth*cos(angle_v)*sin(angle_h);
	z = depth*sin(angle_v);
	
	/*normal*/
	Eigen::Vector3d bgr(
		(double)img_normal.at<cv::Vec3b>(row, col)[0],	//b
		(double)img_normal.at<cv::Vec3b>(row, col)[1],	//g
		(double)img_normal.at<cv::Vec3b>(row, col)[2]	//r
	);
	if(bgr(0)==0 && bgr(1)==0 && bgr(2)==0){
		nx = 0.0;
		ny = 0.0;
		nz = 0.0;
	}
	else{
		std::vector<Eigen::Vector3d> axes(3);
		axes[0] = {0, 0, 1};	//b
		/* axes[2] = {-x, -y, -z};	//r */
		axes[2] = {-x, -y, 0};	//r
		axes[1] = (axes[0]).cross(axes[2]);	//g
		Eigen::Matrix3d Axes;
		for(size_t i=0;i<axes.size();++i)	Axes.block(0, i, 3, 1) = axes[i].normalized();
		Eigen::Vector3d normal = Axes.transpose()*bgr;
		normal.normalize();
		nx = normal(0);
		ny = normal(1);
		nz = normal(2);
	}
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

void ImageToPC::Visualization(void)
{
	viewer.removeAllPointClouds();

	viewer.addPointCloudNormals<pcl::PointXYZINormal>(cloud, 1, 0.5, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 0.5, "cloud");

	viewer.spinOnce();
}

double ImageToPC::DegToRad(double deg)
{
	return deg/180.0*M_PI;
}

double ImageToPC::PiToPi(double angle)
{
	return atan2(sin(angle), cos(angle)); 
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_to_pc");
	
	ImageToPC image_to_pc;
	image_to_pc.LoopExecution();
}
