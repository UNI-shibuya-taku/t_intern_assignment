#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

class RoadPCExtraction{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		/*subscribe*/
		ros::Subscriber sub_pc;
		/*publish*/
		ros::Publisher pub_pc;
		/*pcl*/
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZINormal>};
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_road {new pcl::PointCloud<pcl::PointXYZINormal>};
		/*objects*/
		/*parameters*/
		double threshold_curvature;
	public:
		RoadPCExtraction();
		void CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg);
		void Extraction(void);
		void Publication(void);
};

RoadPCExtraction::RoadPCExtraction()
	:nhPrivate("~")
{
	sub_pc = nh.subscribe("/cloud", 1, &RoadPCExtraction::CallbackPC, this);
	pub_pc = nh.advertise<sensor_msgs::PointCloud2>("/cloud/road", 1);

	nhPrivate.param("threshold_curvature", threshold_curvature, 0.5);
	std::cout << "threshold_curvature = " << threshold_curvature << std::endl;
}

void RoadPCExtraction::CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	/* std::cout << "CALLBACK PC" << std::endl; */

	pcl::fromROSMsg(*msg, *cloud);
	Extraction();
	Publication();
}

void RoadPCExtraction::Extraction(void)
{
	// *cloud_road = *cloud;
	pcl::PassThrough<pcl::PointXYZINormal> pt;
	pt.setInputCloud(cloud);
	pt.setFilterFieldName("curvature");
	pt.setFilterLimits(0.0, threshold_curvature);
	pt.filter(*cloud_road);
}

void RoadPCExtraction::Publication(void)
{
	/*pc*/
	sensor_msgs::PointCloud2 pc_ros;
	pcl::toROSMsg(*cloud_road, pc_ros);
	pub_pc.publish(pc_ros);	
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "road_pc_extraction");
	
	RoadPCExtraction road_pc_extraction;

	ros::spin();
}
