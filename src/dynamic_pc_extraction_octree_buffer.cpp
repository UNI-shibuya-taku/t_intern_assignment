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
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_dynamic {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_static {new pcl::PointCloud<pcl::PointXYZ>};
		// pcl::PointCloud<pcl::PointXYZ>::Ptr pc_test {new pcl::PointCloud<pcl::PointXYZ>};
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> buffer_pc;
		/*parameters*/
		std::string child_frame_name;
		std::string parent_frame_name;
		double voxel_size;
		int min_points_per_leaf;
		int buffer_size;
		double threshhold_dynamic_likelihood;
	public:
		DynamicPCExtraction();
		void CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg);
		bool PCTransform(void);
		void Extraction(void);
		std::vector<int> GetDynamicIndices(const pcl::PointCloud<pcl::PointXYZ>::Ptr pc_reference, const pcl::PointCloud<pcl::PointXYZ>::Ptr pc_target);
		void Publication(void);
		void Visualization(void);
};

DynamicPCExtraction::DynamicPCExtraction()
	:nhPrivate("~")
{
	sub_pc = nh.subscribe("/cloud", 1, &DynamicPCExtraction::CallbackPC, this);
	pub_pc = nh.advertise<sensor_msgs::PointCloud2>("/cloud/dynamic", 1);
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(3.0, "axis");
	viewer.setCameraPosition(0.0, 0.0, 150.0, 0.0, 0.0, 0.0);

	nhPrivate.param("child_frame_name", child_frame_name, std::string("/lidar"));
	std::cout << "child_frame_name = " << child_frame_name << std::endl;
	nhPrivate.param("parent_frame_name", parent_frame_name, std::string("/odom"));
	std::cout << "parent_frame_name = " << parent_frame_name << std::endl;
	nhPrivate.param("voxel_size", voxel_size, 1.0);
	std::cout << "voxel_size = " << voxel_size << std::endl;
	nhPrivate.param("min_points_per_leaf", min_points_per_leaf, 0);
	std::cout << "min_points_per_leaf = " << min_points_per_leaf << std::endl;
	nhPrivate.param("buffer_size", buffer_size, 10);
	std::cout << "buffer_size = " << buffer_size << std::endl;
	nhPrivate.param("threshhold_dynamic_likelihood", threshhold_dynamic_likelihood, 0.5);
	std::cout << "threshhold_dynamic_likelihood = " << threshhold_dynamic_likelihood << std::endl;
}

void DynamicPCExtraction::CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	/* std::cout << "CALLBACK PC" << std::endl; */

	/*subscribe*/
	pcl::fromROSMsg(*msg, *pc_current);
	std::cout << "==========" << std::endl;
	std::cout << "pc_current->points.size() = " << pc_current->points.size() << std::endl;

	/*extract*/
	if(!buffer_pc.empty()){
		if(PCTransform()){
			Extraction();
			Publication();
		}
		else	buffer_pc.erase(buffer_pc.end());
	}
	Visualization();

	/*buffer*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_pc {new pcl::PointCloud<pcl::PointXYZ>};
	*tmp_pc = *pc_current;
	buffer_pc.push_back(tmp_pc);
	if(buffer_pc.size() > (size_t)buffer_size)	buffer_pc.erase(buffer_pc.begin());
}

bool DynamicPCExtraction::PCTransform(void)
{
	/*current*/
	tf::StampedTransform tf_trans_current;
	ros::Time time_current;
	pcl_conversions::fromPCL(pc_current->header.stamp, time_current);
	try{
		tf_listener.lookupTransform(
			parent_frame_name,
			pc_current->header.frame_id,
			time_current,
			tf_trans_current
		);
	}
	catch(tf::TransformException ex){
		std::cout << "Error: current tf listen" << std::endl;
		ROS_ERROR("%s", ex.what());
		ros::Duration(1.0).sleep();
		return false;
	}
	/*last*/
	tf::StampedTransform tf_trans_last;
	ros::Time time_last;
	pcl_conversions::fromPCL(buffer_pc[buffer_pc.size() -1]->header.stamp, time_last);
	try{
		tf_listener.lookupTransform(
			parent_frame_name,
			buffer_pc[buffer_pc.size() -1]->header.frame_id,
			time_last,
			tf_trans_last
		);
	}
	catch(tf::TransformException ex){
		std::cout << "Error: last tf listen" << std::endl;
		ROS_ERROR("%s", ex.what());
		ros::Duration(1.0).sleep();
		return false;
	}
	/*transformation*/
	tf::Quaternion relative_rotation = tf_trans_last.getRotation()*tf_trans_current.getRotation().inverse();
	relative_rotation.normalize();	
	Eigen::Quaternionf rotation(relative_rotation.w(), relative_rotation.x(), relative_rotation.y(), relative_rotation.z());
	tf::Quaternion q_global_move(
		tf_trans_last.getOrigin().x() - tf_trans_current.getOrigin().x(),
		tf_trans_last.getOrigin().y() - tf_trans_current.getOrigin().y(),
		tf_trans_last.getOrigin().z() - tf_trans_current.getOrigin().z(),
		0.0
	);
	tf::Quaternion q_local_move = tf_trans_last.getRotation().inverse()*q_global_move*tf_trans_last.getRotation();
	Eigen::Vector3f offset(q_local_move.x(), q_local_move.y(), q_local_move.z());
	for(size_t i=0;i<buffer_pc.size();++i)	pcl::transformPointCloud(*buffer_pc[i], *buffer_pc[i], offset, rotation);

	return true;
}

void DynamicPCExtraction::Extraction(void)
{
	double time_start = ros::Time::now().toSec();

	/*initialize*/
	pc_dynamic->points.clear();
	/*octree*/
	std::vector<int> dynamic_counters(pc_current->points.size(), 0);
	for(size_t i=0;i<buffer_pc.size();++i){
		std::vector<int> tmp_indices = GetDynamicIndices(buffer_pc[i], pc_current);
		for(size_t j=0;j<tmp_indices.size();++j)	dynamic_counters[tmp_indices[j]] += 1;
	}
	/*judge*/
	for(size_t i=0;i<dynamic_counters.size();++i){
		/* std::cout << "dynamic_counters[i] = " << dynamic_counters[i] << std::endl; */
		double dynamic_likelihood = dynamic_counters[i]/(double)buffer_pc.size();
		if(dynamic_likelihood > threshhold_dynamic_likelihood)	pc_dynamic->points.push_back(pc_current->points[i]);
	}

	std::cout << "extraction time [s] = " << ros::Time::now().toSec() - time_start << std::endl;
}

std::vector<int> DynamicPCExtraction::GetDynamicIndices(const pcl::PointCloud<pcl::PointXYZ>::Ptr pc_reference, const pcl::PointCloud<pcl::PointXYZ>::Ptr pc_target)
{
	/*initialize*/
	std::vector<int> newPointIdxVector;
	pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(voxel_size);
	/*last*/
	octree.setInputCloud(pc_reference);
	octree.addPointsFromInputCloud();
	/*switch*/
	octree.switchBuffers();	
	/*current*/
	octree.setInputCloud(pc_target);
	octree.addPointsFromInputCloud();
	/*get diff*/
	octree.getPointIndicesFromNewVoxels(newPointIdxVector, min_points_per_leaf);

	return newPointIdxVector;
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

	/*pc_store*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc_store {new pcl::PointCloud<pcl::PointXYZ>};
	for(size_t i=0;i<buffer_pc.size();++i)	*pc_store += *buffer_pc[i];
	viewer.addPointCloud(pc_store, "pc_store");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "pc_store");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "pc_store");

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
