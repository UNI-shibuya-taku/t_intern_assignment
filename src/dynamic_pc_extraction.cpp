#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>

class DynamicPCExtraction{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		/*subscribe*/
		ros::Subscriber sub_pc;
		tf::TransformListener tf_listener;
		/*pcl objects*/
		pcl::visualization::PCLVisualizer viewer {"dynamic_pc_extraction"};
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_last {new pcl::PointCloud<pcl::PointXYZ>};
		std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> voxels;
		/*parameters*/
		std::string child_frame_name;
		std::string parent_frame_name;
		double voxel_size;
		int voxel_per_line;
	public:
		DynamicPCExtraction();
		void CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg);
		void PCTransform(void);
		void Divide(void);
		void PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_out, std::vector<double> range);
		void Visualization(void);
};

DynamicPCExtraction::DynamicPCExtraction()
	:nhPrivate("~")
{
	sub_pc = nh.subscribe("/cloud", 1, &DynamicPCExtraction::CallbackPC, this);
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(1.0, "axis");
	viewer.setCameraPosition(0.0, 0.0, 80.0, 0.0, 0.0, 0.0);

	nhPrivate.param("child_frame_name", child_frame_name, std::string("/lidar"));
	std::cout << "child_frame_name = " << child_frame_name << std::endl;
	nhPrivate.param("parent_frame_name", parent_frame_name, std::string("/odom"));
	std::cout << "parent_frame_name = " << parent_frame_name << std::endl;
	nhPrivate.param("voxel_size", voxel_size, 1.0);
	std::cout << "voxel_size = " << voxel_size << std::endl;
	nhPrivate.param("voxel_per_line", voxel_per_line, 10);
	std::cout << "voxel_per_line = " << voxel_per_line << std::endl;
}

void DynamicPCExtraction::CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	/* std::cout << "CALLBACK PC" << std::endl; */

	pcl::fromROSMsg(*msg, *pc);
	std::cout << "==========" << std::endl;
	std::cout << "pc->points.size() = " << pc->points.size() << std::endl;

	if(!pc_last->points.empty()){
		PCTransform();
		/* #<{(|current|)}># */
		/* tf::StampedTransform tf_trans; */
		/* try{ */
		/* 	tf_listener.lookupTransform( */
		/* 		parent_frame_name, */
		/* 		msg->header.frame_id, */
		/* 		msg->header.stamp, */
		/* 		tf_trans */
		/* 	); */
		/* } */
		/* catch (tf::TransformException ex){ */
		/* 	ROS_ERROR("%s",ex.what()); */
		/* 	ros::Duration(1.0).sleep(); */
		/* } */
		/* #<{(|last|)}># */
		/* tf::StampedTransform tf_trans_last; */
		/* ros::Time time_last; */
		/* pcl_conversions::fromPCL(pc_last->header.stamp, time_last); */
		/* try{ */
		/* 	tf_listener.lookupTransform( */
		/* 		parent_frame_name, */
		/* 		pc_last->header.frame_id, */
		/* 		time_last, */
		/* 		tf_trans_last */
		/* 	); */
		/* } */
		/* catch (tf::TransformException ex){ */
		/* 	ROS_ERROR("%s",ex.what()); */
		/* 	ros::Duration(1.0).sleep(); */
		/* } */
        /*  */
		/* tf::Quaternion relative_rotation = tf_trans_last.getRotation()*tf_trans.getRotation().inverse(); */
		/* relative_rotation.normalize();	 */
		/* Eigen::Quaternionf rotation(relative_rotation.w(), relative_rotation.x(), relative_rotation.y(), relative_rotation.z()); */
		/* tf::Quaternion q_global_move( */
		/* 	tf_trans_last.getOrigin().x() - tf_trans.getOrigin().x(), */
		/* 	tf_trans_last.getOrigin().y() - tf_trans.getOrigin().y(), */
		/* 	tf_trans_last.getOrigin().z() - tf_trans.getOrigin().z(), */
		/* 	0.0 */
		/* ); */
		/* tf::Quaternion q_local_move = tf_trans_last.getRotation().inverse()*q_global_move*tf_trans_last.getRotation(); */
		/* Eigen::Vector3f offset(q_local_move.x(), q_local_move.y(), q_local_move.z()); */
		/* pcl::transformPointCloud(*pc_last, *pc_last, offset, rotation); */
	}
	Visualization();

	*pc_last = *pc;
}

void DynamicPCExtraction::PCTransform(void)
{
	/*current*/
	tf::StampedTransform tf_trans;
	ros::Time time_current;
	pcl_conversions::fromPCL(pc->header.stamp, time_current);
	try{
		tf_listener.lookupTransform(
			parent_frame_name,
			pc->header.frame_id,
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

void DynamicPCExtraction::Divide(void)
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
			PassThroughFilter(pc, tmp, range);
			voxels[i][j] = tmp;
		}
	}

	std::cout << "dividing time [s] = " << ros::Time::now().toSec() - time_start << std::endl;
}

void DynamicPCExtraction::PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_out, std::vector<double> range)
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

void DynamicPCExtraction::Visualization(void)
{
	viewer.removeAllPointClouds();

	/*pc*/
	viewer.addPointCloud(pc, "pc");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "pc");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pc");
	/*pc_last*/
	viewer.addPointCloud(pc_last, "pc_last");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "pc_last");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pc_last");
	/*voxels*/
	if(!voxels.empty()){
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
	}
	
	viewer.spinOnce();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dynamic_pc_extraction");
	
	DynamicPCExtraction dynamic_pc_extraction;

	ros::spin();
}
