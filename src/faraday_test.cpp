#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

void setCurrentTrajectory(const trajectory_msgs::JointTrajectoryConstPtr& traj)
{
	/*std::vector<double> block_position;
	double dist_points[6];
	double dist_time;
	dist_time=(traj->points[2].time_from_start.toSec())-(traj->points[1].time_from_start.toSec());

	long motor_position;
	short motor_velocity;
	// for (int i = 1; i < traj->points.size(); ++i)
	// {
	// 	block_position[0]=traj->points[i].positions[0];
	// 	motor_position=static_cast<long>(block_position[0]*16000);
	// 	dist_points[0]=traj->points[i].positions[0]-traj->points[i-1].positions[0];
	// 	motor_velocity=static_cast<short>((dist_points[0]/dist_time)*480);
	// 	printf("motor_position is %ld\n",motor_position);
	// 	printf("motor_velocity is %d\n",motor_velocity);

	// 	usbcan0.MotorVelPos_Mode(MDriver_Group,MDriver_Number,2500,motor_velocity,motor_position);
	// 	usleep(1000);
	// }

	block_position[0]=traj->points[1].positions[0];
	// motor_position=static_cast<long>(block_position[0]*16);
	printf("block_position is %f\n",block_position[0]);
	// motor_velocity=static_cast<short>((dist_points[0]/dist_time)*480);
	// usbcan0.MotorVelPos_Mode(MDriver_Group,MDriver_Number,2500,500,motor_position);
	// usleep(1000);*/
	printf("%f\n",traj->points[0].time_from_start.toSec());
	printf("%f\n",traj->points[0].positions[0]);
	printf("%f\n",traj->points[1].time_from_start.toSec());
	printf("%f\n",traj->points[1].positions[0]);
	printf("two points interval time is %f\n",traj->points[1].time_from_start.toSec()-traj->points[0].time_from_start.toSec());

	std::vector<double> block_position;
	block_position.reserve(6);
	block_position[0]=traj->points[0].positions[0];
	printf("%f\n",block_position[0]);
	printf("hahahahahahahaha\n");
}

int main(int argc, char *argv[])
{
	printf("Welcome to CANopen_ROS_node!\n");
	ros::init(argc,argv,"faraday_test_node");
	ros::NodeHandle nh;

	ros::Subscriber traj_sub=nh.subscribe("joint_path_command",16,&setCurrentTrajectory);

	ros::spin();

	return 0;
}