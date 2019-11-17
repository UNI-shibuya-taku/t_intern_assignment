#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/octree/octree.h>
#include <pcl/visualization/cloud_viewer.h>

class DynamicPCExtraction{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		/*subscribe*/
		ros::Subscriber sub_pc;
		tf::TransformListener tf_listener;
		/*publish*/
		ros::Publisher pub_pc;
		/*pcl objects*/
		pcl::visualization::PCLVisualizer viewer {"dynamic_pc_extraction"};
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_current {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_last {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_dynamic {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_static {new pcl::PointCloud<pcl::PointXYZ>};
		/*parameters*/
		std::string child_frame_name;
		std::string parent_frame_name;
		double voxel_size;
	public:
		DynamicPCExtraction();
		void CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg);
		void PCTransform(void);
		void Extraction(void);
		void PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_out, std::vector<double> range);
		bool JudgeDynamic(pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_current, pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_last);
		void Publication(void);
		void Visualization(void);
};

DynamicPCExtraction::DynamicPCExtraction()
	:nhPrivate("~")
{
	sub_pc = nh.subscribe("/cloud", 1, &DynamicPCExtraction::CallbackPC, this);
	pub_pc = nh.advertise<sensor_msgs::PointCloud2>("/cloud/dynamic", 1);
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(1.0, "axis");
	viewer.setCameraPosition(0.0, 0.0, 100.0, 0.0, 0.0, 0.0);

	nhPrivate.param("child_frame_name", child_frame_name, std::string("/lidar"));
	std::cout << "child_frame_name = " << child_frame_name << std::endl;
	nhPrivate.param("parent_frame_name", parent_frame_name, std::string("/odom"));
	std::cout << "parent_frame_name = " << parent_frame_name << std::endl;
	nhPrivate.param("voxel_size", voxel_size, 1.0);
	std::cout << "voxel_size = " << voxel_size << std::endl;
}

void DynamicPCExtraction::CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	/* std::cout << "CALLBACK PC" << std::endl; */

	pcl::fromROSMsg(*msg, *pc_current);
	std::cout << "==========" << std::endl;
	std::cout << "pc_current->points.size() = " << pc_current->points.size() << std::endl;

	if(!pc_last->points.empty()){
		PCTransform();
		Extraction();
		Publication();
	}
	Visualization();

	pc_last->header = pc_current->header;
	*pc_last += *pc_current;
}

void DynamicPCExtraction::PCTransform(void)
{
	/*current*/
	tf::StampedTransform tf_trans;
	ros::Time time_current;
	pcl_conversions::fromPCL(pc_current->header.stamp, time_current);
	try{
		tf_listener.lookupTransform(
			parent_frame_name,
			pc_current->header.frame_id,
			time_current,
			tf_trans
		);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}
	/*last*/
	tf::StampedTransform tf_trans_last;
	ros::Time time_last;
	pcl_conversions::fromPCL(pc_last->header.stamp, time_last);
	try{
		tf_listener.lookupTransform(
			parent_frame_name,
			pc_last->header.frame_id,
			time_last,
			tf_trans_last
		);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}
	/*transformation*/
	tf::Quaternion relative_rotation = tf_trans_last.getRotation()*tf_trans.getRotation().inverse();
	relative_rotation.normalize();	
	Eigen::Quaternionf rotation(relative_rotation.w(), relative_rotation.x(), relative_rotation.y(), relative_rotation.z());
	tf::Quaternion q_global_move(
		tf_trans_last.getOrigin().x() - tf_trans.getOrigin().x(),
		tf_trans_last.getOrigin().y() - tf_trans.getOrigin().y(),
		tf_trans_last.getOrigin().z() - tf_trans.getOrigin().z(),
		0.0
	);
	tf::Quaternion q_local_move = tf_trans_last.getRotation().inverse()*q_global_move*tf_trans_last.getRotation();
	Eigen::Vector3f offset(q_local_move.x(), q_local_move.y(), q_local_move.z());
	pcl::transformPointCloud(*pc_last, *pc_last, offset, rotation);
}

void DynamicPCExtraction::Extraction(void)
{
	double time_start = ros::Time::now().toSec();

	/*initialize*/
	pc_dynamic->points.clear();
	pc_static->points.clear();
	std::vector<int> newPointIdxVector;
	pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(voxel_size);
	/*last*/
	octree.setInputCloud(pc_last);
	octree.addPointsFromInputCloud();
	/*switch*/
	octree.switchBuffers();	
	/*current*/
	octree.setInputCloud(pc_current);
	octree.addPointsFromInputCloud();
	/*get diff*/
	// octree.getPointIndicesFromNewVoxels(newPointIdxVector);
	octree.getPointIndicesFromNewVoxels(newPointIdxVector, 10);

	/*extract*/
	/* pc_dynamic->points.resize(newPointIdxVector.size()); */
	/* for(size_t i=0;i<newPointIdxVector.size();++i){ */
	/* 	pc_dynamic->points[i] = pc_current->points[newPointIdxVector[i]]; */
	/* } */
	int counter_static = 0;
	int counter_dynamic = 0;
	pc_dynamic->points.resize(newPointIdxVector.size());
	pc_static->points.resize(pc_current->points.size() - newPointIdxVector.size());
	for(size_t i=0;i<pc_current->points.size();++i){
		if(i == newPointIdxVector[counter_dynamic]){
			pc_dynamic->points[counter_dynamic] = pc_current->points[i];
			counter_dynamic++;
		}
		else{
			pc_static->points[counter_static] = pc_current->points[i];
			counter_static++;
		}
	}

	std::cout << "extraction time [s] = " << ros::Time::now().toSec() - time_start << std::endl;
}

void DynamicPCExtraction::Publication(void)
{
	/*pc*/
	pc_dynamic->header.frame_id = pc_current->header.frame_id;
	pc_dynamic->header.stamp = pc_current->header.stamp;
	sensor_msgs::PointCloud2 pc_ros;
	pcl::toROSMsg(*pc_dynamic, pc_ros);
	pub_pc.publish(pc_ros);	
}

void DynamicPCExtraction::Visualization(void)
{
	viewer.removeAllPointClouds();

	/*pc_current*/
	viewer.addPointCloud(pc_current, "pc_current");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "pc_current");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "pc_current");

	/*pc_last*/
	viewer.addPointCloud(pc_last, "pc_last");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "pc_last");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "pc_last");

	/*pc_dynamic*/
	viewer.addPointCloud(pc_dynamic, "pc_dynamic");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "pc_dynamic");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "pc_dynamic");
	
	viewer.spinOnce();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dynamic_pc_extraction");
	
	DynamicPCExtraction dynamic_pc_extraction;

	ros::spin();
}
