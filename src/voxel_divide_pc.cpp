#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>

class VoxelDividePC{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		/*subscribe*/
		ros::Subscriber sub_pc;
		/*pcl objects*/
		pcl::visualization::PCLVisualizer viewer {"voxel_divide_pc"};
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZ>};
		std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> voxels;
		/*parameters*/
		double voxel_size;
		int voxel_per_line;
	public:
		VoxelDividePC();
		void CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg);
		void Divide(void);
		void PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_out, std::vector<double> range);
		void Visualization(void);
};

VoxelDividePC::VoxelDividePC()
	:nhPrivate("~")
{
	sub_pc = nh.subscribe("/cloud", 1, &VoxelDividePC::CallbackPC, this);
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(1.0, "axis");
	viewer.setCameraPosition(0.0, 0.0, 80.0, 0.0, 0.0, 0.0);

	nhPrivate.param("voxel_size", voxel_size, 1.0);
	std::cout << "voxel_size = " << voxel_size << std::endl;
	nhPrivate.param("voxel_per_line", voxel_per_line, 10);
	std::cout << "voxel_per_line = " << voxel_per_line << std::endl;
}

void VoxelDividePC::CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	/* std::cout << "CALLBACK PC" << std::endl; */

	pcl::fromROSMsg(*msg, *cloud);
	std::cout << "==========" << std::endl;
	std::cout << "cloud->points.size() = " << cloud->points.size() << std::endl;

	voxels = std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>>(voxel_per_line, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>(voxel_per_line));
	Divide();
	Visualization();
}

void VoxelDividePC::Divide(void)
{
	double time_start = ros::Time::now().toSec();

	for(int i=0;i<voxel_per_line;++i){
		for(int j=0;j<voxel_per_line;++j){
			std::vector<double> range{
				(i - voxel_per_line/2)*voxel_size,
				(i - voxel_per_line/2)*voxel_size + voxel_size,
				(j - voxel_per_line/2)*voxel_size,
				(j - voxel_per_line/2)*voxel_size + voxel_size
			};
			pcl::PointCloud<pcl::PointXYZ>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZ>);
			PassThroughFilter(cloud, tmp, range);
			voxels[i][j] = tmp;
		}
	}

	std::cout << "dividing time [s] = " << ros::Time::now().toSec() - time_start << std::endl;
}

void VoxelDividePC::PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_out, std::vector<double> range)
{
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(pc_in);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(range[0], range[1]);
	pass.filter(*pc_out);
	pass.setInputCloud(pc_out);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(range[2], range[3]);
	pass.filter(*pc_out);
}

void VoxelDividePC::Visualization(void)
{
	viewer.removeAllPointClouds();

	/*cloud*/
	viewer.addPointCloud(cloud, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
	/*voxels*/
	double rgb[3] = {};
	const int channel = 3;
	const double step = ceil(pow(voxel_per_line*voxel_per_line+2, 1.0/(double)channel));	//exept (000),(111)
	const double max = 1.0;
	for(int i=0;i<voxel_per_line;i++){
		for(int j=0;j<voxel_per_line;j++){
			int index = i*voxel_per_line + j;
			std::string name = "voxel_" + std::to_string(index);
			rgb[0] += 1/step;
			for(int c=0;c<channel-1;c++){
				if(rgb[c]>max){
					rgb[c] -= max + 1/step;
					rgb[c+1] += 1/step;
				}
			}
			viewer.addPointCloud(voxels[i][j], name);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, rgb[0], rgb[1], rgb[2], name);
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
		}
	}
	
	viewer.spinOnce();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "voxel_divide_pc");
	
	VoxelDividePC voxel_divide_pc;

	ros::spin();
}
