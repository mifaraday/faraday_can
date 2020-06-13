#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <sstream>
#include <unistd.h>
#include <cstdio>
#include <cmath>

#include "usbcan.h"
#include "faraday_can/Can_Frame.h"
#include "faraday_can/MDrive_Feedback.h"
#include "faraday_can/Joint_Space.h"



//motor grear and encoder ... 
#define REDUCTION_RATIO 16000.0 

//为了在ROS_CAN类中调用电机速度指令，将usbcan0声明为全局变量
USBCAN usbcan0;
unsigned char MDriver_Group=0,MDriver_Number=0;//0　0　则代表对0组进行广播发送

class ROS_CAN
{
public:
	ROS_CAN();
	~ROS_CAN();
	void registerNodeHandle(ros::NodeHandle& _nh);
	void registerPubSub();
	void publisher_pub();//发布者发布消息函数
	static void cansendCallback(const faraday_can::Can_Frame::ConstPtr& canmsg);
	//接受IP_pos topic并转换为位置控制PDO发送
    static void PosCmdCallback(const faraday_can::Joint_Space::ConstPtr& cmd);
    //将接受到的路径点信息转化成电机的位置和速度信息 //receive command from ros_controller to control motor
    // static void setCurrentTrajectory(const trajectory_msgs::JointTrajectoryConstPtr& traj);
	void setCurrentTrajectory(const sensor_msgs::JointStateConstPtr& traj);
	
private:
	ros::NodeHandle nh;
	ros::Publisher canrecieve_pub;
	ros::Publisher MotorFeedback_pub;
	ros::Subscriber cansend_sub;
	ros::Subscriber motor_pos_sub;	
	//订阅经过轨迹规划后的路径点的topic
	ros::Subscriber traj_sub;
	//电机的初始状态，也就是滑块在模组上的初始位置
	// double init_structure_motor_pos[7]={140.8,140.8,140.8,140.8,9.2,9.27,0};
//	double init_structure_motor_pos[7]={227.139, 225.652, 225.652, 227.139, 0.0, 0.0, 284.0};
    double init_structure_motor_pos[7]={147.139, 145.652, 145.652, 147.139, 0.0, 0.0, 234.0};

};

ROS_CAN::ROS_CAN()
{
	//构造函数中初始化滑块的初始位置
	// init_structure_motor_pos[7]={140.8,140.8,140.8,140.8,9.2,9.27,0};
}

ROS_CAN::~ROS_CAN(){}

void ROS_CAN::registerNodeHandle(ros::NodeHandle& _nh)
{
	nh=_nh;
}

void ROS_CAN::registerPubSub()
{
	canrecieve_pub=nh.advertise<faraday_can::Can_Frame>("can_recieve",16);
	MotorFeedback_pub=nh.advertise<faraday_can::MDrive_Feedback>("MotorFeedback",16);
    cansend_sub = nh.subscribe("can_send", 1, &ROS_CAN::cansendCallback);
    motor_pos_sub = nh.subscribe("motor_pos",16,&ROS_CAN::PosCmdCallback);	
    traj_sub=nh.subscribe("zzz/rrr/joint_states",1000,&ROS_CAN::setCurrentTrajectory,this);

}

/*利用在usbcan.cpp中留的_can_receive接口，将驱动器反馈回来的位置速度限位信号等信息
发送出去.*/
void ROS_CAN::publisher_pub()
{
	faraday_can::Can_Frame can_msg;
	can_msg.id=USBCAN::_can_receive.ID;
	can_msg.is_error=USBCAN::_can_receive.SendType;
	can_msg.is_rtr=USBCAN::_can_receive.RemoteFlag;
	can_msg.is_extended=USBCAN::_can_receive.ExternFlag;
	can_msg.dlc=USBCAN::_can_receive.DataLen;
	can_msg.stamp=ros::Time::now();
	for(int i = 0; i < can_msg.dlc; i++)
	{
		can_msg.data[i]=USBCAN::_can_receive.Data[i];
	}
	// ROS_INFO("CAN_ID:0x%03X",can_msg.id);
	//将驱动器返回的can消息，发送出去
	canrecieve_pub.publish(can_msg);

	faraday_can::MDrive_Feedback motor_msg;
	for(int i=0;i<7;++i)
	{
		motor_msg.Current[i]=USBCAN::Real_Current_Value[i];
		motor_msg.Velocity[i]=USBCAN::Real_Velocity_Value[i];
		motor_msg.Position[i]=USBCAN::Real_Position_Value[i];
		motor_msg.Ctl1_Value[i]=USBCAN::Real_Ctl1_Value[i];
		motor_msg.Ctl2_Value[i]=USBCAN::Real_Ctl2_Value[i];
	}	
	motor_msg.stamp=ros::Time::now();
	// ROS_INFO("motor velocity: %dr/min",motor_msg.Velocity[0]);
	// 将电机的速度位置等等信息发送出去
	MotorFeedback_pub.publish(motor_msg);
}

//某一节点订阅can_send这个话题，就可以实现主机向驱动器发送can消息。
void ROS_CAN::cansendCallback(const faraday_can::Can_Frame::ConstPtr& canmsg)
{
    VCI_CAN_OBJ send[1];
	send[0].ID = canmsg->id;
	send[0].SendType=canmsg->is_error;
	send[0].RemoteFlag=canmsg->is_rtr;
	send[0].ExternFlag=canmsg->is_extended;
	send[0].DataLen=canmsg->dlc;
	
	int i=0;
	for(i = 0; i < send[0].DataLen; i++)
	{
		send[0].Data[i] = canmsg->data[i];
	}
    if(VCI_Transmit(VCI_USBCAN2, 0, CAN1, send, 1) == 1)
		{
			printf("CAN1 TX ID:0x%08X",send[0].ID);
			if(send[0].ExternFlag==0) printf(" Standard ");
			if(send[0].ExternFlag==1) printf(" Extend   ");
			if(send[0].RemoteFlag==0) printf(" Data   ");
			if(send[0].RemoteFlag==1) printf(" Remote ");
			printf("DLC:0x%02X",send[0].DataLen);
			printf(" data:0x");

			for(i=0;i<send[0].DataLen;i++)
			{
				printf(" %02X",send[0].Data[i]);
			}

			printf("\n");
		}
}

//某一节点订阅motor_pose这个话题，就可以实现对电机的控制。
void ROS_CAN::PosCmdCallback(const faraday_can::Joint_Space::ConstPtr& cmd)
{
	unsigned char motor_num_cmd;
	double motor_position_cmd[6];
	double motor_velocity_cmd[6];

	motor_num_cmd=cmd->motor_num;
	for(int i=0;i<motor_num_cmd;++i)
	{
		motor_position_cmd[i]=cmd->position[i];
		motor_velocity_cmd[i]=cmd->velocity[i];
	}

	//测试一下电机的速度模式 此处还没有考虑减速比等等因素
	// usbcan0.MotorVel_Mode(MDriver_Group,MDriver_Number,5000,short(motor_velocity_cmd[0]));

	usbcan0.MotorVelPos_Mode(MDriver_Group,MDriver_Number,2500,
							 short(motor_velocity_cmd[0]),
							 long(motor_position_cmd[0]));
}

void ROS_CAN::setCurrentTrajectory(const sensor_msgs::JointStateConstPtr& traj)
{
	std::vector<double> block_position;
	block_position.reserve(7);

	block_position=traj->position;

	std::vector<long> motor_position;
	motor_position.reserve(7);

	//set the max velocity of motor 6000r/min
	short motor_velocity=6000;

	//set the max current of motor 2400mA
	short motor_current=2400;

	for(int i=0;i<7;i++)
	{
		motor_position[i]=static_cast<long>((block_position[i]-init_structure_motor_pos[i])*REDUCTION_RATIO);
		// use motor drive current velocity position mode
        usbcan0.MotorCurVelPos_Mode(0,i+1,motor_current,motor_velocity,motor_position[i]);
	}

//    ROS_INFO_STREAM("motor_position_command: "<<motor_position[0]<<" "<<motor_position[1]<<" "
//					<<motor_position[2]<<" "<<motor_position[3]<<" "<<motor_position[4]<<" "
//                    <<motor_position[5]<<" "<<motor_position[6]);
	
}



int main(int argc, char *argv[])
{
	printf("Welcome to CANopen_ROS_node!\n");
	ros::init(argc,argv,"CANopen_ROS_node");
	ros::NodeHandle nh;

	ROS_CAN ROS_CAN1;
	ROS_CAN1.registerNodeHandle(nh);
	ROS_CAN1.registerPubSub();

	usbcan0.CanDev_Init();
	usbcan0.startRecieveCanThread();
	usbcan0.MotorDriver_Reset(MDriver_Group,MDriver_Number);
	usleep(500000);
	usbcan0.MotorMode_Choice(MDriver_Group,MDriver_Number,CurVelPos_Mode);
	//配置驱动器10ｍｓ返回一次数据
	usbcan0.MotorDriver_Config(MDriver_Group,MDriver_Number,10,10);
	usleep(500000);
	// usbcan0.MotorVel_Mode(MDriver_Group,MDriver_Number,5000,3000);

	ros::Rate loop_rate(100);
	while(ros::ok())
	{
		ROS_CAN1.publisher_pub();

		ros::spinOnce();
		loop_rate.sleep();
	}

	// usbcan0.MotorVelPos_Mode(MDriver_Group,MDriver_Number,2500,1000);
	usbcan0.MotorDriver_Reset(MDriver_Group,MDriver_Number);
	usleep(3000000);
	// usbcan0.MotorVelPos_Mode(MDriver_Group,MDriver_Number,5000,0);

	return 0;
}

