#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

class PCCorrection{
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
		pcl::visualization::PCLVisualizer viewer {"pc_correction"};
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_current {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_last {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_corrected {new pcl::PointCloud<pcl::PointXYZ>};
		/*objects*/
		Eigen::Vector3d local_move_xyz;
		Eigen::Vector3d local_move_rpy;
		tf::Quaternion q_relative_rotation;
		/*parameters*/
		std::string parent_frame_name;
	public:
		PCCorrection();
		void CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg);
		bool GetLocalMove(void);
		void Correction(void);
		double ComputeAngle(pcl::PointXYZ p);
		double ZeroTo2Pi(double angle);
		void Publication(void);
		void Visualization(void);
};

PCCorrection::PCCorrection()
	:nhPrivate("~")
{
	sub_pc = nh.subscribe("/cloud", 1, &PCCorrection::CallbackPC, this);
	pub_pc = nh.advertise<sensor_msgs::PointCloud2>("/cloud/corrected", 1);
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(3.0, "axis");
	viewer.setCameraPosition(0.0, 0.0, 180.0, 0.0, 0.0, 0.0);

	nhPrivate.param("parent_frame_name", parent_frame_name, std::string("/odom"));
	std::cout << "parent_frame_name = " << parent_frame_name << std::endl;
}

void PCCorrection::CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	/* std::cout << "CALLBACK PC" << std::endl; */

	/*subscribe*/
	pcl::fromROSMsg(*msg, *pc_current);
	std::cout << "==========" << std::endl;
	std::cout << "pc_current->points.size() = " << pc_current->points.size() << std::endl;

	/*extract*/
	if(!pc_last->points.empty()){
		if(GetLocalMove()){
			Correction();
			Publication();
		}
	}
	Visualization();

	/*buffer*/
	*pc_last = *pc_current;
}

bool PCCorrection::GetLocalMove(void)
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
	pcl_conversions::fromPCL(pc_last->header.stamp, time_last);
	try{
		tf_listener.lookupTransform(
			parent_frame_name,
			pc_last->header.frame_id,
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
	q_relative_rotation = tf_trans_current.getRotation()*tf_trans_last.getRotation().inverse();
	tf::Matrix3x3(q_relative_rotation).getRPY(local_move_rpy(0), local_move_rpy(1), local_move_rpy(2));
	tf::Quaternion q_global_move(
		tf_trans_current.getOrigin().x() - tf_trans_last.getOrigin().x(),
		tf_trans_current.getOrigin().y() - tf_trans_last.getOrigin().y(),
		tf_trans_current.getOrigin().z() - tf_trans_last.getOrigin().z(),
		0.0
	);
	tf::Quaternion q_local_move = tf_trans_current.getRotation().inverse()*q_global_move*tf_trans_current.getRotation();
	local_move_xyz =  {
		q_local_move.x(), 
		q_local_move.y(), 
		q_local_move.z()
	};

	std::cout << "local_move_xyz = " << std::endl << local_move_xyz << std::endl;
	std::cout << "local_move_rpy/M_PI*360.0 = " << std::endl << local_move_rpy/M_PI*360.0 << std::endl;

	return true;
}

void PCCorrection::Correction(void)
{
	pc_corrected->points.clear();
	for(size_t i=0;i<pc_current->points.size();++i){
		double angle = ComputeAngle(pc_current->points[i]);
		// angle = ZeroTo2Pi(angle);
		/* angle = M_PI - angle; */
		if(angle < 0)	angle *= -1;
		else	angle = 2*M_PI - angle;
		/* if(std::isnan(angle))	std::cout << "pc_current->points[i] = " << pc_current->points[i] << std::endl; */
		double ratio = 1.0 - angle/(2*M_PI);
		Eigen::Vector3d correction = -ratio*local_move_xyz;
		pcl::PointXYZ tmp_p;
		/* tmp_p.x = pc_current->points[i].x + correction[0]; */
		/* tmp_p.y = pc_current->points[i].y + correction[1]; */
		/* tmp_p.z = pc_current->points[i].z + correction[2]; */
		tf::Quaternion q_zero(0.0, 0.0, 0.0, 1.0);
		tf::Quaternion q_point(
			pc_current->points[i].x + correction[0],
			pc_current->points[i].y + correction[1],
			pc_current->points[i].z + correction[2],
			0.0
		);
		tf::Quaternion q_rotation = q_zero.slerp(q_relative_rotation.inverse(), ratio);
		q_point = q_rotation*q_point*q_rotation.inverse();
		tmp_p.x = q_point.x();
		tmp_p.y = q_point.y();
		tmp_p.z = q_point.z();
		pc_corrected->points.push_back(tmp_p);
		
		std::cout << "pc_current->points[i] = " << pc_current->points[i] << std::endl;
		std::cout << "pc_corrected->points[i] = " << pc_corrected->points[i] << std::endl;
	}
}

double PCCorrection::ComputeAngle(pcl::PointXYZ p)
{
	return atan2(p.y, p.x);
}

double PCCorrection::ZeroTo2Pi(double angle)
{
	if(angle > 0)	return angle;
	else	return angle + 2*M_PI;
}

void PCCorrection::Publication(void)
{
	/*pc*/
	pc_corrected->header.frame_id = pc_current->header.frame_id;
	pc_corrected->header.stamp = pc_current->header.stamp;
	sensor_msgs::PointCloud2 pc_ros;
	pcl::toROSMsg(*pc_corrected, pc_ros);
	pub_pc.publish(pc_ros);	
}

void PCCorrection::Visualization(void)
{
	viewer.removeAllPointClouds();

	/*pc_current*/
	viewer.addPointCloud(pc_current, "pc_current");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "pc_current");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "pc_current");

	/*pc_corrected*/
	viewer.addPointCloud(pc_corrected, "pc_corrected");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "pc_corrected");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "pc_corrected");
	
	viewer.spinOnce();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_correction");
	
	PCCorrection pc_correction;

	ros::spin();
}
