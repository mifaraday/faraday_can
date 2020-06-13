#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <cstring>

#include "faraday_can/Can_Frame.h"
#include "faraday_can/MDrive_Feedback.h"
#include "sensor_msgs/JointState.h"

//motor grear and encoder ... 
#define REDUCTION_RATIO 16000.0 

using namespace std;

class JointState
{
public:
	JointState();
	void registerNodeHandle(ros::NodeHandle& _nh);
	void registerPubSub();
	void init();
	void PosFeedCallback(const faraday_can::MDrive_Feedback::ConstPtr& motor_msg);
	~JointState();

private:
	// double init_structure_motor_pos[7]={140.8,140.8,140.8,140.8,9.2,9.27,0};//电机的初始状态，也就是滑块在模组上的初始位置
	double init_structure_motor_pos[7]={227.139, 225.652, 225.652, 227.139, 0.0, 0.0, 284.0};
	double current_pos[7];
	double current_vel[7];
	sensor_msgs::JointState  jointFeedbackMsg;
	ros::Publisher jointFeedbackPub;
	ros::Subscriber MotorFeedback_sub;
	ros::NodeHandle nh;	
};

int main(int argc, char *argv[])
{
	printf("Welcome to this node\n");
	ros::init(argc,argv,"faraday_joint_states_publisher");
	ros::NodeHandle nh;

	JointState JointState1;
	JointState1.init();
	JointState1.registerNodeHandle(nh);
	JointState1.registerPubSub();
	

	ros::spin();

	return 0;
}

JointState::JointState()
{
	//构造函数中初始化滑块的初始位置
	// init_structure_motor_pos[7]={140.8,140.8,140.8,140.8,9.2,9.27,0};
	current_pos[7]={0};
	current_vel[7]={0};
}

JointState::~JointState() {}

void JointState::registerNodeHandle(ros::NodeHandle& _nh)
{
	nh=_nh;
}

void JointState::registerPubSub()
{
	jointFeedbackPub=nh.advertise<sensor_msgs::JointState>("/slide_block/joint_states",1000);
	//回调函数PosFeedCallback是非静态的，所以下面要加一个this指针,如果是静态成员函数就不用
	MotorFeedback_sub=nh.subscribe("MotorFeedback",1000,&JointState::PosFeedCallback,this);
}

//初始化jointFeedbackMsg中的某些项。
void JointState::init()
{
	for(int i=0;i<7;i++)
	{
		jointFeedbackMsg.position.push_back(0);
		jointFeedbackMsg.velocity.push_back(0);
		jointFeedbackMsg.effort.push_back(0);
		jointFeedbackMsg.name.push_back("joint"+std::to_string(i+1));
	}
}

//订阅MotorFeedback这个话题，将电机的位置速度信息转化为滑块的位置速度信息
void JointState::PosFeedCallback(const faraday_can::MDrive_Feedback::ConstPtr& motor_msg)
{
	for(int i=0;i<7;++i)
	{
		//通过电机反馈的位置，计算滑块的位置
		long int absolute_pos=motor_msg->Position[i];
		ROS_INFO("joint%d current position of encoder is %ld qc",i+1,absolute_pos);
		//编码器的线数为500线，减速比为16，丝杠的导程为２
		// current_pos[i]=init_structure_motor_pos[i]+(double(absolute_pos)/(2000*16))*2;
		current_pos[i]=init_structure_motor_pos[i]+(static_cast<double>(absolute_pos)/REDUCTION_RATIO);
		jointFeedbackMsg.position[i]=current_pos[i];
		ROS_INFO("current position of joint%d is %fmm",i+1,current_pos[i]);

		//通过电机反馈的速度，计算滑块的速度
		//减速比16，丝杠导程2,单位为mm/min,最后除以60单位转化为mm/s
		current_vel[i]=double(motor_msg->Velocity[i])/16*2/60;
		ROS_INFO("current velocity of joint%d is %fmm/s",i+1,current_vel[i]);
	}
	jointFeedbackMsg.header.stamp=ros::Time::now();
	//发布每个模组中滑块的位置速度等信息
	jointFeedbackPub.publish(jointFeedbackMsg);
}