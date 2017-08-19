#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <sstream>
#include "std_msgs/String.h"
#include <sstream>
#include "std_srvs/Empty.h"
#include <cstdlib>



void reset_sim()
{
	//ros::NodeHandle n;
	//ros::ServiceClient client = n.serviceClient<>();
	std_srvs::Empty resetWorldSrv;
	ros::service::call("/gazebo/reset_world",resetWorldSrv);

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "restart_sim_node");
	 
	ros::NodeHandle node;
	tf::TransformListener listener;	
	double roll, pitch, yaw;
	ros::Rate rate(10.0);

	while(node.ok())
	{
		tf::StampedTransform transform;	//lookupTransform needs stampedTransform param
		try{
			listener.lookupTransform("/odom","/base_link",ros::Time(0), transform);
		}
		catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}
		
		transform.getBasis().getRPY(roll, pitch, yaw);
				
		if (std::abs(pitch) > 1.4)
			{
				ROS_INFO("RESTART SIMULATION");
				reset_sim();
				
			}
		ROS_INFO("%f",pitch);
		std::cout<<pitch<<std::endl;
		rate.sleep();

	}//end of while	

	return 0;
}

