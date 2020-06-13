#include <iostream>
#include "unistd.h"
#include "stdio.h"
#include "stdlib.h"
#include "usbcan.h"
#include "controlcan.h"
#include <cmath>

USBCAN::USBCAN(){
}

USBCAN::~USBCAN(){
    printf(" USBCAN release\n");
    m_run0=0;//线程关闭指令。
    pthread_join(threadid,NULL);//等待线程关闭   
    VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
    usleep(100000);//延时100ms。
    VCI_ResetCAN(VCI_USBCAN2, 0, 1);//复位CAN2通道。
    usleep(100000);//延时100ms。
    VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。
   // 除收发函数外，其它的函数调用前后，最好加个毫秒级的延时，即不影响程序的运行，又可以让USBCAN设备有充分的时间处理指令。
}


int USBCAN::startRecieveCanThread()
{
    /*pthread_t threadid;
    int m_run0=1;*/
    m_run0=1;
    CanRecieveThread_Flag = pthread_create(&threadid,NULL, USBCAN::receive_func,&m_run0);

    return CanRecieveThread_Flag;//创建线程成功时为0，非零则创建失败。
}

int count=0;//数据列表中，用来存储列表序号。

void *USBCAN::receive_func(void* param)  //接收线程。
{
	int reclen=0;
	VCI_CAN_OBJ rec[3000];//接收缓存，设为3000为佳。
	int i,j;
	
	int *run=(int*)param;//线程启动，退出控制。
    int ind=0;
    if((*run)&0x0f)
	    printf("thread work on!!!\n");
	while((*run)&0x0f)
	{
		if((reclen=VCI_Receive(VCI_USBCAN2,DeviceInd,CAN1,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
		{
			for(j=0;j<reclen;j++)
			{
                //接受驱动器返回的消息
                USBCAN::_can_receive=rec[j];                

				// printf("Index:%04d  ",count);count++;//序号递增
				// printf("CAN%d RX ID:0x%03X", ind+1, rec[j].ID);//ID
				// if(rec[j].ExternFlag==0) printf(" Standard ");//帧格式：标准帧
				// if(rec[j].ExternFlag==1) printf(" Extend   ");//帧格式：扩展帧
				// if(rec[j].RemoteFlag==0) printf(" Data   ");//帧类型：数据帧
				// if(rec[j].RemoteFlag==1) printf(" Remote ");//帧类型：远程帧
				// printf("DLC:0x%02X",rec[j].DataLen);//帧长度
				// printf(" data:0x");	//数据
				// for(i = 0; i < rec[j].DataLen; i++)
				// {
				// 	printf(" %02X", rec[j].Data[i]);
				// }
				// printf(" TimeStamp:0x%08X",rec[j].TimeStamp);//时间标识。
				// printf("\n");

                if((0==rec[j].RemoteFlag)&&(0==rec[j].ExternFlag))//标准帧、数据帧
                {
                    if(0x1B==rec[j].ID)
                    {
                        USBCAN::Real_Current_Value[0]=(rec[j].Data[0]<<8)|(rec[j].Data[1]);
                        USBCAN::Real_Velocity_Value[0]=(rec[j].Data[2]<<8)|(rec[j].Data[3]);
                        USBCAN::Real_Position_Value[0]=((rec[j].Data[4]<<24)|(rec[j].Data[5]<<16)|(rec[j].Data[6]<<8)|(rec[j].Data[7]));
                    }
                    else if(0x2B==rec[j].ID)
                    {
                        USBCAN::Real_Current_Value[1]=(rec[j].Data[0]<<8)|(rec[j].Data[1]);
                        USBCAN::Real_Velocity_Value[1]=(rec[j].Data[2]<<8)|(rec[j].Data[3]);
                        USBCAN::Real_Position_Value[1]=((rec[j].Data[4]<<24)|(rec[j].Data[5]<<16)|(rec[j].Data[6]<<8)|(rec[j].Data[7]));
                    }
                    else if(0x3B==rec[j].ID)
                    {
                        USBCAN::Real_Current_Value[2]=(rec[j].Data[0]<<8)|(rec[j].Data[1]);
                        USBCAN::Real_Velocity_Value[2]=(rec[j].Data[2]<<8)|(rec[j].Data[3]);
                        USBCAN::Real_Position_Value[2]=((rec[j].Data[4]<<24)|(rec[j].Data[5]<<16)|(rec[j].Data[6]<<8)|(rec[j].Data[7]));
                    }
                    else if(0x4B==rec[j].ID)
                    {
                        USBCAN::Real_Current_Value[3]=(rec[j].Data[0]<<8)|(rec[j].Data[1]);
                        USBCAN::Real_Velocity_Value[3]=(rec[j].Data[2]<<8)|(rec[j].Data[3]);
                        USBCAN::Real_Position_Value[3]=((rec[j].Data[4]<<24)|(rec[j].Data[5]<<16)|(rec[j].Data[6]<<8)|(rec[j].Data[7]));
                    }
                    else if(0x5B==rec[j].ID)
                    {
                        USBCAN::Real_Current_Value[4]=(rec[j].Data[0]<<8)|(rec[j].Data[1]);
                        USBCAN::Real_Velocity_Value[4]=(rec[j].Data[2]<<8)|(rec[j].Data[3]);
                        USBCAN::Real_Position_Value[4]=((rec[j].Data[4]<<24)|(rec[j].Data[5]<<16)|(rec[j].Data[6]<<8)|(rec[j].Data[7]));
                    }
                    else if(0x6B==rec[j].ID)
                    {
                        USBCAN::Real_Current_Value[5]=(rec[j].Data[0]<<8)|(rec[j].Data[1]);
                        USBCAN::Real_Velocity_Value[5]=(rec[j].Data[2]<<8)|(rec[j].Data[3]);
                        USBCAN::Real_Position_Value[5]=((rec[j].Data[4]<<24)|(rec[j].Data[5]<<16)|(rec[j].Data[6]<<8)|(rec[j].Data[7]));
                    }
                    else if(0x7B==rec[j].ID)
                    {
                        USBCAN::Real_Current_Value[6]=(rec[j].Data[0]<<8)|(rec[j].Data[1]);
                        USBCAN::Real_Velocity_Value[6]=(rec[j].Data[2]<<8)|(rec[j].Data[3]);
                        USBCAN::Real_Position_Value[6]=((rec[j].Data[4]<<24)|(rec[j].Data[5]<<16)|(rec[j].Data[6]<<8)|(rec[j].Data[7]));
                    }
                    else if(0x8B==rec[j].ID)
                    {
                        USBCAN::Real_Velocity_Value[7]=(rec[j].Data[2]<<8)|(rec[j].Data[3]);  //电主轴只反馈速度
                    }
                    else if(0x1F==rec[j].ID)
                    {
                        USBCAN::Real_Online[0]=1;
                    }
                    else if(0x2F==rec[j].ID)
                    {
                        USBCAN::Real_Online[1]=1;
                    }
                    else if(0x3F==rec[j].ID)
                    {
                        USBCAN::Real_Online[2]=1;
                    }
                    else if(0x4F==rec[j].ID)
                    {
                        USBCAN::Real_Online[3]=1;
                    }
                    else if(0x5F==rec[j].ID)
                    {
                        USBCAN::Real_Online[4]=1;
                    }
                    else if(0x6F==rec[j].ID)
                    {
                        USBCAN::Real_Online[5]=1;
                    }
                    else if(0x7F==rec[j].ID)
                    {
                        USBCAN::Real_Online[6]=1;
                    }
                    else if(0x1C==rec[j].ID)
                    {
                        USBCAN::Real_Ctl1_Value[0]=rec[j].Data[0];
                        USBCAN::Real_Ctl2_Value[0]=rec[j].Data[1];
                    }
                    else if(0x2C==rec[j].ID)
                    {
                        USBCAN::Real_Ctl1_Value[1]=rec[j].Data[0];
                        USBCAN::Real_Ctl2_Value[1]=rec[j].Data[1];
                    }
                    else if(0x3C==rec[j].ID)
                    {
                        USBCAN::Real_Ctl1_Value[2]=rec[j].Data[0];
                        USBCAN::Real_Ctl2_Value[2]=rec[j].Data[1];
                    }
                    else if(0x4C==rec[j].ID)
                    {
                        USBCAN::Real_Ctl1_Value[3]=rec[j].Data[0];
                        USBCAN::Real_Ctl2_Value[3]=rec[j].Data[1];
                    }
                    else if(0x5C==rec[j].ID)
                    {
                        USBCAN::Real_Ctl1_Value[4]=rec[j].Data[0];
                        USBCAN::Real_Ctl2_Value[4]=rec[j].Data[1];
                    }
                    else if(0x6C==rec[j].ID)
                    {
                        USBCAN::Real_Ctl1_Value[5]=rec[j].Data[0];
                        USBCAN::Real_Ctl2_Value[5]=rec[j].Data[1];
                    }
                    else if(0x7C==rec[j].ID)
                    {
                        USBCAN::Real_Ctl1_Value[6]=rec[j].Data[0];
                        USBCAN::Real_Ctl2_Value[6]=rec[j].Data[1];
                    }
                }
			}
		}
	}
	printf("run thread exit\n");//退出接收线程	
	pthread_exit(0);
}

short USBCAN::CanDev_Init()
{
    printf(">>this is hello !\r\n");//指示程序已运行
    if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)//打开设备
	{
		printf(">>open deivce success!\n");//打开设备成功
	}else
	{
		printf(">>open deivce error!\n");
		exit(1);		
	}
    	if(VCI_ReadBoardInfo(VCI_USBCAN2,0,&pInfo)==1)//读取设备序列号、版本等信息。
	{
                printf(">>Get VCI_ReadBoardInfo success!\n");
		
		//printf(" %08X", pInfo.hw_Version);printf("\n");
		//printf(" %08X", pInfo.fw_Version);printf("\n");
		//printf(" %08X", pInfo.dr_Version);printf("\n");
		//printf(" %08X", pInfo.in_Version);printf("\n");
		//printf(" %08X", pInfo.irq_Num);printf("\n");
		//printf(" %08X", pInfo.can_Num);printf("\n");
		printf(">>Serial_Num:%c", pInfo.str_Serial_Num[0]);
		printf("%c", pInfo.str_Serial_Num[1]);
		printf("%c", pInfo.str_Serial_Num[2]);
		printf("%c", pInfo.str_Serial_Num[3]);
		printf("%c", pInfo.str_Serial_Num[4]);
		printf("%c", pInfo.str_Serial_Num[5]);
		printf("%c", pInfo.str_Serial_Num[6]);
		printf("%c", pInfo.str_Serial_Num[7]);
		printf("%c", pInfo.str_Serial_Num[8]);
		printf("%c", pInfo.str_Serial_Num[9]);
		printf("%c", pInfo.str_Serial_Num[10]);
		printf("%c", pInfo.str_Serial_Num[11]);
		printf("%c", pInfo.str_Serial_Num[12]);
		printf("%c", pInfo.str_Serial_Num[13]);
		printf("%c", pInfo.str_Serial_Num[14]);
		printf("%c", pInfo.str_Serial_Num[15]);
		printf("%c", pInfo.str_Serial_Num[16]);
		printf("%c", pInfo.str_Serial_Num[17]);
		printf("%c", pInfo.str_Serial_Num[18]);
		printf("%c", pInfo.str_Serial_Num[19]);printf("\n");

		printf(">>hw_Type:%c", pInfo.str_hw_Type[0]);
		printf("%c", pInfo.str_hw_Type[1]);
		printf("%c", pInfo.str_hw_Type[2]);
		printf("%c", pInfo.str_hw_Type[3]);
		printf("%c", pInfo.str_hw_Type[4]);
		printf("%c", pInfo.str_hw_Type[5]);
		printf("%c", pInfo.str_hw_Type[6]);
		printf("%c", pInfo.str_hw_Type[7]);
		printf("%c", pInfo.str_hw_Type[8]);
		printf("%c", pInfo.str_hw_Type[9]);printf("\n");	
	}else
	{
		printf(">>Get VCI_ReadBoardInfo error!\n");
		exit(1);
	}
    	//初始化参数，严格参数二次开发函数库说明书。
	VCI_INIT_CONFIG config;
	config.AccCode=0;
	config.AccMask=0xFFFFFFFF;
	config.Filter=1;//接收所有帧
	config.Timing0=0x00;/*波特率500 Kbps  0x03  0x1C*/
	config.Timing1=0x14;
	config.Mode=0;//正常模式		
	
	if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
	{
		printf(">>Init CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);
	}
	else printf(">>Init CAN1 success\n");

	if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
	{
		printf(">>Start CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);

	}
	else printf(">>Start CAN1 success\n");

	if(VCI_InitCAN(VCI_USBCAN2,0,1,&config)!=1)
	{
		printf(">>Init can2 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);

	}
	else printf(">>Init CAN2 success\n");

	if(VCI_StartCAN(VCI_USBCAN2,0,1)!=1)
	{
		printf(">>Start can2 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);

	}
	else printf(">>Start CAN2 success\n");
    return 0;
}

void USBCAN::CanDev_Close()//设备关闭
{
    usleep(10000000);//延时单位us，这里设置 10 000 000=10s    10s后关闭接收线程，并退出主程序。
    m_run0=0;//线程关闭指令。
    pthread_join(threadid,NULL);//等待线程关闭。
    usleep(100000);//延时100ms。
    VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
    usleep(100000);//延时100ms。
    VCI_ResetCAN(VCI_USBCAN2, 0, 1);//复位CAN2通道。
    usleep(100000);//延时100ms。
    if(1!=VCI_CloseDevice(VCI_USBCAN2,0))
    {
        printf(">>Close CAN1 error !\n");
        //return return_false ;
    }
    else
    {
        printf(">>Close CAN1 success !\n");
    }
    
    //return return_true ;
}

int USBCAN::send(unsigned int id,unsigned int CANx, const unsigned char *pdata, unsigned int frame_num)
{
    unsigned int packed_num=0;
    // 假设最大发送６个帧，６×８＝４８
    VCI_CAN_OBJ psend[48];

    if(packed_num<frame_num){
        psend[packed_num].ID = id;
        psend[packed_num].SendType = 0;
        psend[packed_num].RemoteFlag = 0;
        psend[packed_num].ExternFlag = 0;
        psend[packed_num].DataLen= 8 ;
    
        int i=0;
        for(i = 0; i < psend[packed_num].DataLen; i++)
        {
            psend[packed_num].Data[i] = pdata[i];
        }
        packed_num++;
    }
    // printf("send once, packed num = %d, frame num = %d\n",packed_num, frame_num);
    if(packed_num==frame_num){
        packed_num = 0;
        // printf("send once, packed num = %d\n",packed_num);
        if(VCI_Transmit(VCI_USBCAN2, DeviceInd, CANx, psend, frame_num) >= 1)
        {
            printf("Index:%04d  ",count);
            count++;
            printf("CAN%d TX ID:0x%08X",CANx+1, psend[0].ID);
            if(psend[0].ExternFlag==0) printf(" Standard ");
            if(psend[0].ExternFlag==1) printf(" Extend   ");
            if(psend[0].RemoteFlag==0) printf(" Data   ");
            if(psend[0].RemoteFlag==1) printf(" Remote ");
            printf("DLC:0x%02X",psend[0].DataLen);
            printf(" data:0x");

            for(int i=0;i<psend[0].DataLen;i++)
            {
                printf(" %02X",psend[0].Data[i]);
            }

            printf("\n");
            psend[0].ID+=1;

            return return_true;
        }
    }

    return return_false;
}

/*********************************************************************
*                               复位指令
*Group   取值范围０－７
*Number  取值范围０－１５，其中Number==0时，为广播发送
*********************************************************************/

short USBCAN::MotorDriver_Reset(unsigned char Temp_Group,unsigned char Temp_Number){
    unsigned short can_id = 0x0000;
    VCI_CAN_OBJ TxMessage;

    TxMessage.SendType=0;
    TxMessage.RemoteFlag=0;
    TxMessage.ExternFlag=0; 
    TxMessage.DataLen=8;

    if((Temp_Group<=7)&&(Temp_Number<=15)){
        can_id |= Temp_Group <<8;
        can_id |= Temp_Number <<4;
    }
    else
    {
        return return_false;
    }

    TxMessage.ID=can_id;

    TxMessage.Data[0]=0x55;
    TxMessage.Data[1]=0x55;
    TxMessage.Data[2]=0x55;
    TxMessage.Data[3]=0x55;
    TxMessage.Data[4]=0x55;
    TxMessage.Data[5]=0x55;
    TxMessage.Data[6]=0x55;
    TxMessage.Data[7]=0x55;

    if(0 == VCI_Transmit(VCI_USBCAN2, DeviceInd, CAN1, &TxMessage, 1)){
        printf("CanSend error\n");
        return return_false;
    }
    printf("CanSend success! All MDriver Reset\n");
    printf("CAN TX ID:0x%03X", TxMessage.ID);//ID
    for(int i = 0; i < TxMessage.DataLen; i++)
	{
		printf(" %02X", TxMessage.Data[i]);
	}
    printf("\n");
    return return_true;

}


/**************************************************************************
*                                  模式选择指令
*Group   取值范围０－７
*Number  取值范围０－１５，其中Number==0时，为广播发送
*
*mode 取值范围
*
*OpenLoop_Mode                        0x01
*Current_Mode                         0x02
*Velocity_Mode                        0x03
*Position_Mode                        0x04
*Velocity_Position_Mode               0x05
*Current_Velocity_Mode                0x06
*Current_Position_Mode                0x07
*Current_Velocity_Position_Mode       0x08
****************************************************************************/
short USBCAN::MotorMode_Choice(unsigned char Temp_Group,unsigned char Temp_Number,unsigned char Temp_Mode){
    unsigned short can_id = 0x0001;
    VCI_CAN_OBJ TxMessage;

    TxMessage.SendType=0;
    TxMessage.RemoteFlag=0;
    TxMessage.ExternFlag=0; 
    TxMessage.DataLen=8;

    if((Temp_Group<=7)&&(Temp_Number<=15)){
        can_id |= Temp_Group <<8;
        can_id |= Temp_Number <<4;
    }
    else
    {
        return return_false;
    }

    TxMessage.ID=can_id;

    TxMessage.Data[0]=Temp_Mode;
    TxMessage.Data[1]=0x55;
    TxMessage.Data[2]=0x55;
    TxMessage.Data[3]=0x55;
    TxMessage.Data[4]=0x55;
    TxMessage.Data[5]=0x55;
    TxMessage.Data[6]=0x55;
    TxMessage.Data[7]=0x55;

    if(0 == VCI_Transmit(VCI_USBCAN2, DeviceInd, CAN1, &TxMessage, 1)){
        printf("CanSend error\n");
        return return_false;
    }
    if(Temp_Mode==Velocity_Mode) printf("CanSend success! Velocity Mode\n");
    else if(Temp_Mode==Position_Mode)   printf("CanSend success! Position Mode\n");

    printf("CAN TX ID:0x%03X", TxMessage.ID);//ID
    for(int i = 0; i < TxMessage.DataLen; i++)
	{
		printf(" %02X", TxMessage.Data[i]);
	}
    printf("\n");
    return return_true;    
}


/********************************************************************************************
                                开环模式下的数据指令
Group   取值范围０－７
Number  取值范围０－１５，其中Number==0时，为广播发送

Temp_pwm的取值范围如下：
-5000~+5000,满值5000,其中Temp_pwm=正负5000时，最大输出电压为电源电压

**********************************************************************************************/
short USBCAN::MotorOpenLoop_Mode(unsigned char Temp_Group,unsigned char Temp_Number,short Temp_PWM)
{
    unsigned short can_id = 0x0002;
    VCI_CAN_OBJ TxMessage;

    TxMessage.SendType=0;
    TxMessage.RemoteFlag=0;
    TxMessage.ExternFlag=0; 
    TxMessage.DataLen=8;

    if((Temp_Group<=7)&&(Temp_Number<=15)){
        can_id |= Temp_Group <<8;
        can_id |= Temp_Number <<4;
    }
    else
    {
        return return_false;
    }


    if(Temp_PWM>5000)
    {
        Temp_PWM=5000;
    }
    else if(Temp_PWM<-5000)
    {
        Temp_PWM=-5000;
    }

    TxMessage.ID=can_id;

    TxMessage.Data[0]=(unsigned char)((Temp_PWM>>8)&0xff);
    TxMessage.Data[1]=(unsigned char)(Temp_PWM&0xff);
    TxMessage.Data[2]=0x55;
    TxMessage.Data[3]=0x55;
    TxMessage.Data[4]=0x55;
    TxMessage.Data[5]=0x55;
    TxMessage.Data[6]=0x55;
    TxMessage.Data[7]=0x55;  

    if(0 == VCI_Transmit(VCI_USBCAN2, DeviceInd, CAN1, &TxMessage, 1)){
        printf("CanSend error\n");
        return return_false;
    }
    printf("CanSend success!Send PWM:%d\n",Temp_PWM);

    printf("CAN TX ID:0x%03X", TxMessage.ID);//ID
    for(int i = 0; i < TxMessage.DataLen; i++)
    {
        printf(" %02X", TxMessage.Data[i]);
    }
    printf("\n");
    return return_true;
}


/********************************************************************************************
                                电流模式下的数据指令
Group   取值范围０－７
Number  取值范围０－１５，其中Number==0时，为广播发送

Temp_pwm的取值范围如下：
0~+5000,满值5000,其中Temp_pwm=正负5000时，最大输出电压为电源电压

Temp_Current的取值范围如下：
-32768~+32767,单位mA

**********************************************************************************************/
short USBCAN::MotorCurrent_Mode(unsigned char Temp_Group,unsigned char Temp_Number,short Temp_PWM,short Temp_Current){
    unsigned short can_id = 0x0003;
    VCI_CAN_OBJ TxMessage;

    TxMessage.SendType=0;
    TxMessage.RemoteFlag=0;
    TxMessage.ExternFlag=0; 
    TxMessage.DataLen=8;

    if((Temp_Group<=7)&&(Temp_Number<=15)){
        can_id |= Temp_Group <<8;
        can_id |= Temp_Number <<4;
    }
    else
    {
        return return_false;
    }

    if(Temp_PWM>5000)    Temp_PWM=5000;
    else if(Temp_PWM<-5000)  Temp_PWM=-5000;
    else    Temp_PWM=abs(Temp_PWM);
    

    TxMessage.ID=can_id;

    TxMessage.Data[0]=(unsigned char)((Temp_PWM>>8)&0xff);
    TxMessage.Data[1]=(unsigned char)(Temp_PWM&0xff);
    TxMessage.Data[2]=(unsigned char)((Temp_Current>>8)&0xff);
    TxMessage.Data[3]=(unsigned char)(Temp_Current&0xff);
    TxMessage.Data[4]=0x55;
    TxMessage.Data[5]=0x55;
    TxMessage.Data[6]=0x55;
    TxMessage.Data[7]=0x55;

    if(0 == VCI_Transmit(VCI_USBCAN2, DeviceInd, CAN1, &TxMessage, 1)){
        printf("CanSend error\n");
        return return_false;
    }
    printf("CanSend success!Send Current:%d\n",Temp_Current);

    printf("CAN TX ID:0x%03X", TxMessage.ID);//ID
    for(int i = 0; i < TxMessage.DataLen; i++)
	{
		printf(" %02X", TxMessage.Data[i]);
	}
    printf("\n");
    return return_true;        
}


/********************************************************************************************
                                速度模式下的数据指令
Group   取值范围０－７
Number  取值范围０－１５，其中Number==0时，为广播发送

Temp_pwm的取值范围如下：
0~+5000,满值5000,其中Temp_pwm=正负5000时，最大输出电压为电源电压

Temp_Velocity的取值范围如下：
-32768~+32767,单位RPM

**********************************************************************************************/
short USBCAN::MotorVel_Mode(unsigned char Temp_Group,unsigned char Temp_Number,short Temp_PWM,short Temp_Velocity){
    unsigned short can_id = 0x0004;
    VCI_CAN_OBJ TxMessage;

    TxMessage.SendType=0;
    TxMessage.RemoteFlag=0;
    TxMessage.ExternFlag=0; 
    TxMessage.DataLen=8;

    if((Temp_Group<=7)&&(Temp_Number<=15)){
        can_id |= Temp_Group <<8;
        can_id |= Temp_Number <<4;
    }
    else
    {
        return return_false;
    }

    if(Temp_PWM>5000)    Temp_PWM=5000;
    else if(Temp_PWM<-5000)  Temp_PWM=5000;
    else    Temp_PWM=abs(Temp_PWM);

    TxMessage.ID=can_id;

    TxMessage.Data[0]=(unsigned char)((Temp_PWM>>8)&0xff);
    TxMessage.Data[1]=(unsigned char)(Temp_PWM&0xff);
    TxMessage.Data[2]=(unsigned char)((Temp_Velocity>>8)&0xff);
    TxMessage.Data[3]=(unsigned char)(Temp_Velocity&0xff);
    TxMessage.Data[4]=0x55;
    TxMessage.Data[5]=0x55;
    TxMessage.Data[6]=0x55;
    TxMessage.Data[7]=0x55;

    if(0 == VCI_Transmit(VCI_USBCAN2, DeviceInd, CAN1, &TxMessage, 1)){
        printf("CanSend error\n");
        return return_false;
    }
    printf("CanSend success!Send Velocity:%d\n",Temp_Velocity);

    printf("CAN TX ID:0x%03X", TxMessage.ID);//ID
    for(int i = 0; i < TxMessage.DataLen; i++)
	{
		printf(" %02X", TxMessage.Data[i]);
	}
    printf("\n"); 
    return return_true;        
}


/********************************************************************************************
                                位置模式下的数据指令
Group   取值范围０－７
Number  取值范围０－１５，其中Number==0时，为广播发送

Temp_pwm的取值范围如下：
0~+5000,满值5000,其中Temp_pwm=正负5000时，最大输出电压为电源电压

Temp_Position的取值范围如下：
-2147483648~+2147483647,单位qc

**********************************************************************************************/
short USBCAN::MotorPos_Mode(unsigned char Temp_Group,unsigned char Temp_Number,short Temp_PWM,long Temp_Position){
    unsigned short can_id = 0x0005;
    VCI_CAN_OBJ TxMessage;

    TxMessage.SendType=0;
    TxMessage.RemoteFlag=0;
    TxMessage.ExternFlag=0; 
    TxMessage.DataLen=8;

    if((Temp_Group<=7)&&(Temp_Number<=15)){
        can_id |= Temp_Group <<8;
        can_id |= Temp_Number <<4;
    }
    else
    {
        return return_false;
    }

    if(Temp_PWM>5000)    Temp_PWM=5000;
    else if(Temp_PWM<-5000)  Temp_PWM=5000;
    else    Temp_PWM=abs(Temp_PWM);

    TxMessage.ID=can_id;

    TxMessage.Data[0]=(unsigned char)((Temp_PWM>>8)&0xff);
    TxMessage.Data[1]=(unsigned char)(Temp_PWM&0xff);
    TxMessage.Data[2]=0x55;
    TxMessage.Data[3]=0x55;
    TxMessage.Data[4]=(unsigned char)((Temp_Position>>24)&0xff);
    TxMessage.Data[5]=(unsigned char)((Temp_Position>>16)&0xff);
    TxMessage.Data[6]=(unsigned char)((Temp_Position>>8)&0xff);
    TxMessage.Data[7]=(unsigned char)(Temp_Position&0xff);

    if(0 == VCI_Transmit(VCI_USBCAN2, DeviceInd, CAN1, &TxMessage, 1)){
        printf("CanSend error\n");
        return return_false;
    }
    printf("CanSend success!Send Position:%ld\n",Temp_Position);

    printf("CAN TX ID:0x%03X", TxMessage.ID);//ID
    for(int i = 0; i < TxMessage.DataLen; i++)
	{
		printf(" %02X", TxMessage.Data[i]);
	}
    printf("\n");
    return return_true;        
}    


/********************************************************************************************
                                速度位置模式下的数据指令
Group   取值范围０－７
Number  取值范围０－１５，其中Number==0时，为广播发送

Temp_pwm的取值范围如下：
0~+5000,满值5000,其中Temp_pwm=正负5000时，最大输出电压为电源电压

Temp_Velocity的取值范围如下：
0~+32767,单位RPM

Temp_Position的取值范围如下：
-2147483648~+2147483647,单位qc

**********************************************************************************************/
short USBCAN::MotorVelPos_Mode(unsigned char Temp_Group,unsigned char Temp_Number,short Temp_PWM,short Temp_Velocity,long Temp_Position){
    unsigned short can_id = 0x0006;
    VCI_CAN_OBJ TxMessage;

    TxMessage.SendType=0;
    TxMessage.RemoteFlag=0;
    TxMessage.ExternFlag=0; 
    TxMessage.DataLen=8;

    if((Temp_Group<=7)&&(Temp_Number<=15)){
        can_id |= Temp_Group <<8;
        can_id |= Temp_Number <<4;
    }
    else
    {
        return return_false;
    }

    if(Temp_PWM>5000)    Temp_PWM=5000;
    else if(Temp_PWM<-5000)  Temp_PWM=5000;
    else    Temp_PWM=abs(Temp_PWM);
    if(Temp_Velocity<0)  Temp_Velocity=abs(Temp_Velocity);

    TxMessage.ID=can_id;

    TxMessage.Data[0]=(unsigned char)((Temp_PWM>>8)&0xff);
    TxMessage.Data[1]=(unsigned char)(Temp_PWM&0xff);
    TxMessage.Data[2]=(unsigned char)((Temp_Velocity>>8)&0xff);
    TxMessage.Data[3]=(unsigned char)(Temp_Velocity&0xff);
    TxMessage.Data[4]=(unsigned char)((Temp_Position>>24)&0xff);
    TxMessage.Data[5]=(unsigned char)((Temp_Position>>16)&0xff);
    TxMessage.Data[6]=(unsigned char)((Temp_Position>>8)&0xff);
    TxMessage.Data[7]=(unsigned char)(Temp_Position&0xff);

    if(0 == VCI_Transmit(VCI_USBCAN2, DeviceInd, CAN1, &TxMessage, 1)){
        printf("CanSend error\n");
        return return_false;
    }
    printf("CanSend success!Send Velocity:%d\n",Temp_Velocity);
    printf("CanSend success!Send Position:%ld\n",Temp_Position);

    printf("CAN TX ID:0x%03X", TxMessage.ID);//ID
    for(int i = 0; i < TxMessage.DataLen; i++)
	{
		printf(" %02X", TxMessage.Data[i]);
	}
    printf("\n");
    return return_true;            
}


/********************************************************************************************
                                电流速度模式下的数据指令
Group   取值范围０－７
Number  取值范围０－１５，其中Number==0时，为广播发送

Temp_Current的取值范围如下：
0~+32767,单位mA

Temp_Velocity的取值范围如下：
-32768~+32767,单位RPM

**********************************************************************************************/
short USBCAN::MotorCurVel_Mode(unsigned char Temp_Group,unsigned char Temp_Number,short Temp_Current,short Temp_Velocity){
    unsigned short can_id = 0x0007;
    VCI_CAN_OBJ TxMessage;

    TxMessage.SendType=0;
    TxMessage.RemoteFlag=0;
    TxMessage.ExternFlag=0; 
    TxMessage.DataLen=8;

    if((Temp_Group<=7)&&(Temp_Number<=15)){
        can_id |= Temp_Group <<8;
        can_id |= Temp_Number <<4;
    }
    else
    {
        return return_false;
    }

    if(Temp_Current<0)  Temp_Current=abs(Temp_Current);

    TxMessage.ID=can_id;

    TxMessage.Data[0]=(unsigned char)((Temp_Current>>8)&0xff);
    TxMessage.Data[1]=(unsigned char)(Temp_Current&0xff);
    TxMessage.Data[2]=(unsigned char)((Temp_Velocity>>8)&0xff);
    TxMessage.Data[3]=(unsigned char)(Temp_Velocity&0xff);
    TxMessage.Data[4]=0x55;
    TxMessage.Data[5]=0x55;
    TxMessage.Data[6]=0x55;
    TxMessage.Data[7]=0x55;

    if(0 == VCI_Transmit(VCI_USBCAN2, DeviceInd, CAN1, &TxMessage, 1)){
        printf("CanSend error\n");
        return return_false;
    }
    printf("CanSend success!Send Current:%d\n",Temp_Current);
    printf("CanSend success!Send Velocity:%d\n",Temp_Velocity);

    printf("CAN TX ID:0x%03X", TxMessage.ID);//ID
    for(int i = 0; i < TxMessage.DataLen; i++)
	{
		printf(" %02X", TxMessage.Data[i]);
	}
    printf("\n");
    return return_true;                
}


/********************************************************************************************
                                电流位置模式下的数据指令
Group   取值范围０－７
Number  取值范围０－１５，其中Number==0时，为广播发送

Temp_Current的取值范围如下：
0~+32767,单位mA

Temp_Position的取值范围如下：
-2147483648~+2147483647,单位qc

**********************************************************************************************/
short USBCAN::MotorCurPos_Mode(unsigned char Temp_Group,unsigned char Temp_Number,short Temp_Current,long Temp_Position)
{
    unsigned short can_id = 0x0008;
    VCI_CAN_OBJ TxMessage;

    TxMessage.SendType=0;
    TxMessage.RemoteFlag=0;
    TxMessage.ExternFlag=0; 
    TxMessage.DataLen=8;

    if((Temp_Group<=7)&&(Temp_Number<=15)){
        can_id |= Temp_Group <<8;
        can_id |= Temp_Number <<4;
    }
    else
    {
        return return_false;
    }

    if(Temp_Current<0)  abs(Temp_Current);

    TxMessage.ID=can_id;

    TxMessage.Data[0]=(unsigned char)((Temp_Current>>8)&0xff);
    TxMessage.Data[1]=(unsigned char)(Temp_Current&0xff);
    TxMessage.Data[2]=0x55;
    TxMessage.Data[3]=0x55;
    TxMessage.Data[4]=(unsigned char)((Temp_Position>>24)&0xff);
    TxMessage.Data[5]=(unsigned char)((Temp_Position>>16)&0xff);
    TxMessage.Data[6]=(unsigned char)((Temp_Position>>8)&0xff);
    TxMessage.Data[7]=(unsigned char)(Temp_Position&0xff);

    if(0 == VCI_Transmit(VCI_USBCAN2, DeviceInd, CAN1, &TxMessage, 1)){
        printf("CanSend error\n");
        return return_false;
    }
    printf("CanSend success!Send Current:%d\n",Temp_Current);
    printf("CanSend success!Send Position:%ld\n",Temp_Position);

    printf("CAN TX ID:0x%03X", TxMessage.ID);//ID
    for(int i = 0; i < TxMessage.DataLen; i++)
    {
        printf(" %02X", TxMessage.Data[i]);
    }
    printf("\n");
    return return_true; 
}


/********************************************************************************************
                                电流速度位置模式下的数据指令
Group   取值范围０－７
Number  取值范围０－１５，其中Number==0时，为广播发送

Temp_Current的取值范围如下：
0~+32767,单位mA

Temp_Velocity的取值范围如下：
０~+32767,单位RPM

Temp_Position的取值范围如下：
-2147483648~+2147483647,单位qc

**********************************************************************************************/
short USBCAN::MotorCurVelPos_Mode(unsigned char Temp_Group,unsigned char Temp_Number,short Temp_Current,short Temp_Velocity,long Temp_Position)
{
    unsigned short can_id = 0x0009;
    VCI_CAN_OBJ TxMessage;

    TxMessage.SendType=0;
    TxMessage.RemoteFlag=0;
    TxMessage.ExternFlag=0; 
    TxMessage.DataLen=8;

    if((Temp_Group<=7)&&(Temp_Number<=15)){
        can_id |= Temp_Group <<8;
        can_id |= Temp_Number <<4;
    }
    else
    {
        return return_false;
    }

    if(Temp_Current<0)  abs(Temp_Current);
    if(Temp_Velocity<0) abs(Temp_Velocity);

    TxMessage.ID=can_id;

    TxMessage.Data[0]=(unsigned char)((Temp_Current>>8)&0xff);
    TxMessage.Data[1]=(unsigned char)(Temp_Current&0xff);
    TxMessage.Data[2]=(unsigned char)((Temp_Velocity>>8)&0xff);
    TxMessage.Data[3]=(unsigned char)(Temp_Velocity&0xff);
    TxMessage.Data[4]=(unsigned char)((Temp_Position>>24)&0xff);
    TxMessage.Data[5]=(unsigned char)((Temp_Position>>16)&0xff);
    TxMessage.Data[6]=(unsigned char)((Temp_Position>>8)&0xff);
    TxMessage.Data[7]=(unsigned char)(Temp_Position&0xff);

    if(0 == VCI_Transmit(VCI_USBCAN2, DeviceInd, CAN1, &TxMessage, 1)){
        printf("CanSend error\n");
        return return_false;
    }
    printf("CanSend success!Send Current:%d\n",Temp_Current);
    printf("CanSend success!Send Velocity:%d\n",Temp_Velocity);
    printf("CanSend success!Send Position:%ld\n",Temp_Position);

    printf("CAN TX ID:0x%03X", TxMessage.ID);//ID
    for(int i = 0; i < TxMessage.DataLen; i++)
    {
        printf(" %02X", TxMessage.Data[i]);
    }
    printf("\n");
    return return_true;    
}


/****************************************************************
                        配置指令
Temp_Time的取值范围:0~255,为0的时候，关闭电流速度位置反馈功能，单位：毫秒
Ctl1_Ctl2的取值范围:0~255,为0的时候，关闭限位信号反馈功能，单位：毫秒


****************************************************************/

short USBCAN::MotorDriver_Config(unsigned char Temp_Group,unsigned char Temp_Number,unsigned char Temp_Time,unsigned char Ctl1_Ctl2)
{
    unsigned short can_id = 0x000A;
    VCI_CAN_OBJ TxMessage;

    TxMessage.SendType=0;
    TxMessage.RemoteFlag=0;
    TxMessage.ExternFlag=0; 
    TxMessage.DataLen=8;

    if((Temp_Group<=7)&&(Temp_Number<=15)){
        can_id |= Temp_Group <<8;
        can_id |= Temp_Number <<4;
    }
    else
    {
        return return_false;
    }

    TxMessage.ID=can_id;

    TxMessage.Data[0]=Temp_Time;
    TxMessage.Data[1]=Ctl1_Ctl2;
    TxMessage.Data[2]=0x55;
    TxMessage.Data[3]=0x55;
    TxMessage.Data[4]=0x55;
    TxMessage.Data[5]=0x55;
    TxMessage.Data[6]=0x55;
    TxMessage.Data[7]=0x55;

    if(0 == VCI_Transmit(VCI_USBCAN2, DeviceInd, CAN1, &TxMessage, 1)){
        printf("CanSend error\n");
        return return_false;
    }
    printf("CanSend success!Send Temp_Time:%d\n",Temp_Time);
    printf("CanSend success!Send Ctl1_Ctl2:%d\n",Ctl1_Ctl2);

    printf("CAN TX ID:0x%03X", TxMessage.ID);//ID
    for(int i = 0; i < TxMessage.DataLen; i++)
    {
        printf(" %02X", TxMessage.Data[i]);
    }
    printf("\n");
    return return_true;  
}

/************************************************************
                        在线检测
*************************************************************/
short USBCAN::MotorDriver_OnlineCheck(unsigned char Temp_Group,unsigned char Temp_Number)
{
    unsigned short can_id = 0x000F;
    VCI_CAN_OBJ TxMessage;

    TxMessage.SendType=0;
    TxMessage.RemoteFlag=0;
    TxMessage.ExternFlag=0; 
    TxMessage.DataLen=8;

    if((Temp_Group<=7)&&(Temp_Number<=15)){
        can_id |= Temp_Group <<8;
        can_id |= Temp_Number <<4;
    }
    else
    {
        return return_false;
    }

    TxMessage.ID=can_id;

    TxMessage.Data[0]=0x55;
    TxMessage.Data[1]=0x55;
    TxMessage.Data[2]=0x55;
    TxMessage.Data[3]=0x55;
    TxMessage.Data[4]=0x55;
    TxMessage.Data[5]=0x55;
    TxMessage.Data[6]=0x55;
    TxMessage.Data[7]=0x55;

    if(0 == VCI_Transmit(VCI_USBCAN2, DeviceInd, CAN1, &TxMessage, 1)){
        printf("CanSend error\n");
        return return_false;
    }

    printf("CAN TX ID:0x%03X", TxMessage.ID);//ID
    for(int i = 0; i < TxMessage.DataLen; i++)
    {
        printf(" %02X", TxMessage.Data[i]);
    }
    printf("\n");
    return return_true;  
}
