#ifndef USBCAN_H
#define USBCAN_H
#include "pthread.h"
#include "controlcan.h"

#define return_false 0
#define return_true 1

#define DeviceInd 0

#define OpenLoop_Mode   0x01
#define Current_Mode    0x02
#define Velocity_Mode   0x03
#define Position_Mode   0x04
#define VelPos_Mode     0x05
#define CurVel_Mode     0x06
#define CurPos_Mode     0x07
#define CurVelPos_Mode  0x08   

class USBCAN{
public:
    USBCAN();
    virtual ~USBCAN();
    int startRecieveCanThread();
    static void *receive_func(void* param);
    short CanDev_Init();
    void CanDev_Close();

    /*****************************************************
    *发送can消息发送
    *输入:    id------CAN ID
    *         CANx----CAN端口
    *         pdata[8]------发送数据字符数组,
    *         frame_num-----一次发送的帧数
    ********************************************************/
    static int send(unsigned int id,unsigned int CANx, const unsigned char *pdata,unsigned int frame_num);

    short MotorDriver_Reset(unsigned char Temp_Group,unsigned char Temp_Number);
    short MotorDriver_Config(unsigned char Temp_Group,unsigned char Temp_Number,unsigned char Temp_Time,unsigned char Ctl1_Ctl2);
    short MotorDriver_OnlineCheck(unsigned char Temp_Group,unsigned char Temp_Number);

    short MotorMode_Choice(unsigned char Temp_Group,unsigned char Temp_Number,unsigned char Temp_Mode);
    short MotorOpenLoop_Mode(unsigned char Temp_Group,unsigned char Temp_Number,short Temp_PWM);
    short MotorCurrent_Mode(unsigned char Temp_Group,unsigned char Temp_Number,short Temp_PWM,short Temp_Current);
    short MotorVel_Mode(unsigned char Temp_Group,unsigned char Temp_Number,short Temp_PWM,short Temp_Velocity);
    short MotorPos_Mode(unsigned char Temp_Group,unsigned char Temp_Number,short Temp_PWM,long Temp_Position);
    short MotorVelPos_Mode(unsigned char Temp_Group,unsigned char Temp_Number,short Temp_PWM,short Temp_Velocity,long Temp_Position); 
    short MotorCurVel_Mode(unsigned char Temp_Group,unsigned char Temp_Number,short Temp_Current,short Temp_Velocity);
    short MotorCurPos_Mode(unsigned char Temp_Group,unsigned char Temp_Number,short Temp_Current,long Temp_Position);
    short MotorCurVelPos_Mode(unsigned char Temp_Group,unsigned char Temp_Number,short Temp_Current,short Temp_Velocity,long Temp_Position);

    //接收驱动器返回来的数据,假设有４个驱动器，都挂在０组，编号为１,2,3,4
    static short Real_Current_Value[7];
    static short Real_Velocity_Value[8];    //最后一个为电主轴的速度
    static long Real_Position_Value[7];
    static char Real_Online[7];
    static char Real_Ctl1_Value[7];
    static char Real_Ctl2_Value[7];

    //接受驱动器返回的消息,留有一个外界访问的接口
    static VCI_CAN_OBJ _can_receive;

private:
    pthread_t threadid;
    int m_run0;
    int CanRecieveThread_Flag;
    VCI_BOARD_INFO pInfo;
};

short USBCAN::Real_Current_Value[7]={0};
short USBCAN::Real_Velocity_Value[8]={0};
long USBCAN::Real_Position_Value[7]={0};
char USBCAN::Real_Online[7]={0};
char USBCAN::Real_Ctl1_Value[7]={0};
char USBCAN::Real_Ctl2_Value[7]={0};
VCI_CAN_OBJ USBCAN::_can_receive={0};

typedef enum
{
    CAN1=0,
    CAN2=1
} CANx;

#endif