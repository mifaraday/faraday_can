#include <iostream>
#include "unistd.h"
#include "usbcan.h"
#include "controlcan.h"
#include "pthread.h"
#include "stdio.h"

using namespace std;

int main(int argc, char* argv[])
{   
    unsigned char MDriver_Group=0,MDriver_Number=0;
    int key=0,Dmode=0;
    USBCAN usbcan0;//定义一个USBCAN类
    usbcan0.CanDev_Init();//USBCAN设备初始化  
    usbcan0.startRecieveCanThread();

    while (1)
    {
        cin>>key;
        if(key== 1)//数字1:切换成速度模式
        {
            usbcan0.MotorDriver_Reset(0,8);
            usleep(500000);
            // usbcan0.MotorMode_Choice(0,8,Velocity_Mode);
            usleep(500000);
            usbcan0.MotorDriver_Config(0,0,10,10);
            Dmode=1;
        }
        else if(key== 2)//数字2:切换成位置模式
        {
            usbcan0.MotorMode_Choice(0,8,Velocity_Mode);
            // usbcan0.MotorDriver_Reset(0,8);
            // usleep(500000);
            // usbcan0.MotorMode_Choice(0,8,Position_Mode); 
            // usleep(500000);        
            // usbcan0.MotorDriver_Config(0,8,10,10);        
            // Dmode=0;   
        }
        else if((key== 3)&&(Dmode==1))//数字3：速度模式下运动
        {
            usbcan0.MotorVel_Mode(MDriver_Group,8,2000,4000);

            // usbcan0.MotorVel_Mode(MDriver_Group,2,2000,1000);
            // usbcan0.MotorVel_Mode(MDriver_Group,3,2000,1000);
            // usbcan0.MotorVel_Mode(MDriver_Group,4,2000,1000);
            // usbcan0.MotorVel_Mode(MDriver_Group,5,2000,1000);
            // usbcan0.MotorVel_Mode(MDriver_Group,6,2000,1000);
            // usbcan0.MotorVel_Mode(MDriver_Group,MDriver_Number,5000,0); 
            usleep(1000);
        }
        else if((key== 4)&&(Dmode==1))//数字4：速度模式下运动
        {
            usbcan0.MotorOpenLoop_Mode(0,8,0);
            // usbcan0.MotorVel_Mode(MDriver_Group,8,2000,2000);
            // usbcan0.MotorVel_Mode(MDriver_Group,2,2000,-1000);
            // usbcan0.MotorVel_Mode(MDriver_Group,3,2000,-1000);
            // usbcan0.MotorVel_Mode(MDriver_Group,4,2000,-1000);
            // usbcan0.MotorVel_Mode(MDriver_Group,5,2000,-1000);
            // usbcan0.MotorVel_Mode(MDriver_Group,6,2000,-1000);
            // usbcan0.MotorVel_Mode(MDriver_Group,MDriver_Number,5000,0); 
            usleep(1000);
        }
        else if((key== 5)&&(Dmode==0))//数字5：位置模式下运动
        {
            // usbcan0.MotorVelPos_Mode(MDriver_Group,MDriver_Number,5000,4000,50000); 
            usbcan0.MotorPos_Mode(MDriver_Group,3,500,50000);
            usleep(1000);
        }
        else if((key== 6)&&(Dmode==0))//数字6：位置模式下运动
        {
            // usbcan0.MotorVelPos_Mode(MDriver_Group,MDriver_Number,5000,4000,50000); 
            usbcan0.MotorPos_Mode(MDriver_Group,3,500,-50000);
            usleep(1000);
        }
        else if(key==7)//数字7：退出循环
        {
            break;
        }
        else if(key==8)
        {
            printf("电机当前的速度：%d\n",USBCAN::Real_Velocity_Value[7]);
            printf("电机当前的位置：%ld\n",USBCAN::Real_Position_Value[0]);
            printf("电机当前的电流：%d\n",USBCAN::Real_Current_Value[0]);
            printf("Ctl1: %d,Ctl2: %d\n",USBCAN::Real_Ctl1_Value[0],USBCAN::Real_Ctl2_Value[0]);
        }

        
        key=0;
    }
        
    usbcan0.CanDev_Close();//USBCAN设备关闭
    
    return 0;
}


/*电主轴相关功能设置  
    电主轴在0组8号
1.复位指令   MotorDriver_Reset(0,8)

2.主轴开始旋转 MotorMode_Choice(0,8,Velocity_Mode)

3.主轴停止 MotorOpenLoop_Mode(0,8,0)

4.主轴转速设置 MotorVel_Mode(0,8,0,2000) 速度设定为2000r/min  直接发完速度指令电主轴也会动

*/

    