// =============================================================================
//	CFS_Sample 本体部
//
//					Filename: main.c
//
// =============================================================================
//		Ver 1.0.0		2012/11/01
// =============================================================================

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
//ROS
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

typedef struct{
	double com[3];
	double rfpos[3];
	double lfpos[3];
	double rhpos[3];
	double lhpos[3];
	double rfwrench[6];
	double lfwrench[6];
	double zmp[3];
	bool startsync;
}humanpose_t;
humanpose_t humanpose;


#include "pCommon.h"
#include "rs_comm.h"
#include "pComResInternal.h"

// =============================================================================
//	マクロ定義
// =============================================================================
#define PRG_VER	"Ver 1.0.0"

// =============================================================================
//	構造体定義
// =============================================================================
typedef struct ST_SystemInfo {
	int com_ok;
} SystemInfo;

// =============================================================================
//	プロトタイプ宣言
// =============================================================================
void App_Init(const char *devpath);
void App_Close(void);
int GetRcv_to_Cmd( char *rcv, char *prm);
ULONG SendData(UCHAR *pucInput, USHORT usSize);
void GetProductInfo(void);
void SerialStart(void);
void SerialStop(void);

// =============================================================================
//	モジュール変数定義
// =============================================================================
SystemInfo gSys;
UCHAR CommRcvBuff[256];
UCHAR CommSendBuff[1024];
UCHAR SendBuff[512];

// ----------------------------------------------------------------------------------
//	メイン関数
// ----------------------------------------------------------------------------------
//	引数	: non
//	戻り値	: non
// ----------------------------------------------------------------------------------
int main(int argc, char *argv[]){
	if(argc<2){
		ROS_ERROR("[leptrino_reader][usage] rosrun leptrino_reader leptrino_reader [device path]");
		exit(1);
	}



	// ROS initialization
	ros::init(argc, argv, "leptrino_reader");
	ros::NodeHandle n;
	ros::Rate loop_rate(1000);
	ros::Publisher pub = n.advertise<geometry_msgs::WrenchStamped>("output", 1000);
//	std::string devpath(argv[1]);
    std::string devpath,frame_id;
	ros::param::get("~device_path", devpath);
	ros::param::get("~foot_frame_id", frame_id);
//	tfbr = new tf::TransformBroadcaster();
	ROS_INFO("[leptrino_reader] ROS init OK [%s]",devpath.c_str());

	int /*i,*/ l = 0, rt = 0;
	int mode_step = 0;
	int AdFlg = 0, EndF = 0;
	long cnt = 0;
	UCHAR strprm[256];
	ST_RES_HEAD *stCmdHead;
	ST_R_DATA_GET_F *stForce;
	ST_R_GET_INF *stGetInfo;


	const double realfactor[6] = {1000.0/10000.0, 1000.0/10000.0, 2000.0/10000.0, 300.0/10000.0, 300.0/10000.0, 300.0/10000.0};//±定格＝±10000

	App_Init(devpath.c_str());
	
	if (gSys.com_ok == NG) {
		ROS_ERROR("ComPort Open Fail [%s] re-check device existence and permittion",devpath.c_str());
		exit(0);
	}
	
	// 製品情報取得
	GetProductInfo();
	while(1) {
		Comm_Rcv();
		if ( Comm_CheckRcv() != 0 ) {		//受信データ有
			CommRcvBuff[0]=0; 
			
			rt = Comm_GetRcvData( CommRcvBuff );
			if ( rt>0 ) {
				stGetInfo = (ST_R_GET_INF *)CommRcvBuff;
				stGetInfo->scFVer[F_VER_SIZE] = 0;
				printf("Version:%s\n", stGetInfo->scFVer);
				stGetInfo->scSerial[SERIAL_SIZE] = 0;
				printf("SerialNo:%s\n", stGetInfo->scSerial);
				stGetInfo->scPName[P_NAME_SIZE] = 0;
				printf("Type:%s\n", stGetInfo->scPName);
				printf("\n");
				EndF = 1;
			}
			
		}
		if ( EndF==1 ) break;
	}
	
	usleep(10000);
	//初期化
	memset(&humanpose,0,sizeof(humanpose_t));
	double offset[6];
	for(int i=0;i<6;i++)offset[i] = 0;
	// 連続送信開始
	SerialStart();
	EndF = 0;


	geometry_msgs::WrenchStamped pubdata;
	unsigned int pubdataseq = 0;


	while(ros::ok()) {
		Comm_Rcv();
		if ( Comm_CheckRcv() != 0 ) {		//受信データ有
			memset(CommRcvBuff,0,sizeof(CommRcvBuff)); 
			
			rt = Comm_GetRcvData( CommRcvBuff );
			if ( rt>0 ) {
				if(cnt<1000){//オフセットキャリブ
					ROS_INFO_ONCE("[leptrino_reader] Calib for 1 sec (Don't step onto the device) [%s]",devpath.c_str());
					stForce = (ST_R_DATA_GET_F *)CommRcvBuff;
					for(int i=0;i<6;i++){
						offset[i] += (double)stForce->ssForce[i]/1000.0;
					}
				}else{
					ROS_INFO_ONCE("[leptrino_reader] Start publishing [%s]",devpath.c_str());
					stForce = (ST_R_DATA_GET_F *)CommRcvBuff;

					const double F_H_OFFSET = 0.03;
					const double rfpos[3] = {0,-0.1,-0.9},lfpos[3] = {0,0.1,-0.9};//高さ適当
					double rfzmp[2],lfzmp[2],testzmp[2];

					if( humanpose.rfwrench[2] > 1.0e-6 ){
						rfzmp[0] = ( -humanpose.rfwrench[4] - humanpose.rfwrench[0] * F_H_OFFSET + humanpose.rfwrench[2] * 0 ) / humanpose.rfwrench[2];
						rfzmp[1] = ( humanpose.rfwrench[3] - humanpose.rfwrench[1] * F_H_OFFSET + humanpose.rfwrench[2] * 0 ) / humanpose.rfwrench[2];
						if(rfzmp[0] > 0.160)rfzmp[0] = 0.160;
						if(rfzmp[0] < -0.100)rfzmp[0] = -0.100;
						if(rfzmp[1] > 0.065)rfzmp[1] = 0.065;
						if(rfzmp[1] < -0.065)rfzmp[1] = -0.065;
					}
					if( humanpose.lfwrench[2] > 1.0e-6 ){
						lfzmp[0] = ( -humanpose.lfwrench[4] - humanpose.lfwrench[0] * F_H_OFFSET + humanpose.lfwrench[2] * 0 ) / humanpose.lfwrench[2];
						lfzmp[1] = ( humanpose.lfwrench[3] - humanpose.lfwrench[1] * F_H_OFFSET + humanpose.lfwrench[2] * 0 ) / humanpose.lfwrench[2];
						if(lfzmp[0] > 0.160)lfzmp[0] = 0.160;
						if(lfzmp[0] < -0.100)lfzmp[0] = -0.100;
						if(lfzmp[1] > 0.065)lfzmp[1] = 0.065;
						if(lfzmp[1] < -0.065)lfzmp[1] = -0.065;
					}
					humanpose.zmp[0] = ( (rfzmp[0]+rfpos[0])*humanpose.rfwrench[2] + (lfzmp[0]+lfpos[0])*humanpose.lfwrench[2] ) / ( humanpose.rfwrench[2] + humanpose.lfwrench[2] );
					humanpose.zmp[1] = ( (rfzmp[1]+rfpos[1])*humanpose.rfwrench[2] + (lfzmp[1]+lfpos[1])*humanpose.lfwrench[2] ) / ( humanpose.rfwrench[2] + humanpose.lfwrench[2] );
					humanpose.zmp[2] = (rfpos[2] + lfpos[2])/2;//体幹から見たZMP高さ？
					if( humanpose.rfwrench[2] < 10.0 && humanpose.lfwrench[2] < 10.0 ){humanpose.zmp[0]=0.0;humanpose.zmp[1]=0.0;humanpose.zmp[2]=0.0;}//両足10N以下はZMP計算しない

//					humanpose.lfwrench[0] = -(stForce->ssForce[1]-offset[1]) * realfactor[1];//レプトリノは右X前Y上Z，普通のロボットは前X左Y上Z
//					humanpose.lfwrench[1] = +(stForce->ssForce[0]-offset[0]) * realfactor[0];
//					humanpose.lfwrench[2] = -(stForce->ssForce[2]-offset[2]) * realfactor[2];
//					humanpose.lfwrench[3] = -(stForce->ssForce[4]-offset[4]) * realfactor[4];
//					humanpose.lfwrench[4] = +(stForce->ssForce[3]-offset[3]) * realfactor[3];
//					humanpose.lfwrench[5] = -(stForce->ssForce[5]-offset[5]) * realfactor[5];
//					if (cnt%100 == 0) {
//					printf("%ld:[L] %+06.1f,%+06.1f,%+06.1f,%+06.1f,%+06.1f,%+06.1f [R] %+06.1f,%+06.1f,%+06.1f,%+06.1f,%+06.1f,%+06.1f [ZMP] %+06.3f,%+06.3f\n",cnt,
//							humanpose.lfwrench[0],humanpose.lfwrench[1],humanpose.lfwrench[2],humanpose.lfwrench[3],humanpose.lfwrench[4],humanpose.lfwrench[5],
//							humanpose.rfwrench[0],humanpose.rfwrench[1],humanpose.rfwrench[2],humanpose.rfwrench[3],humanpose.rfwrench[4],humanpose.rfwrench[5],
//							humanpose.zmp[0],humanpose.zmp[1]);
//					}
					pubdata.header.seq = pubdataseq++;
					pubdata.header.stamp = ros::Time::now();
					pubdata.header.frame_id =  frame_id;
					pubdata.wrench.force.x  = -(stForce->ssForce[1]-offset[1]) * realfactor[1];
					pubdata.wrench.force.y  = +(stForce->ssForce[0]-offset[0]) * realfactor[0];
					pubdata.wrench.force.z  = -(stForce->ssForce[2]-offset[2]) * realfactor[2];
					pubdata.wrench.torque.x = -(stForce->ssForce[4]-offset[4]) * realfactor[4];
					pubdata.wrench.torque.y = +(stForce->ssForce[3]-offset[3]) * realfactor[3];
					pubdata.wrench.torque.z = -(stForce->ssForce[5]-offset[5]) * realfactor[5];
					pub.publish(pubdata);

				}
				ros::spinOnce();
				loop_rate.sleep();

				// 連続送信停止
//				if (cnt == 100000) { SerialStop();}
				stCmdHead = (ST_RES_HEAD *)CommRcvBuff;
				if (stCmdHead->ucCmd == CMD_DATA_STOP) {
					printf("Receive Stop Response:");
					l = stCmdHead->ucLen;
					for (int ii=0; ii<l; ii++) {
						printf("%02x ", CommRcvBuff[ii]);
					}
					printf("\n");
					EndF = 1;
				}
				cnt++;
			}
			
		}
		if ( EndF==1 ) break;
	}
	App_Close();
	return 0;
}

// ----------------------------------------------------------------------------------
//	アプリケーション初期化
// ----------------------------------------------------------------------------------
//	引数	: non
//	戻り値	: non
// ----------------------------------------------------------------------------------
void App_Init(const char *devpath)
{
	int rt;
	
	//Commポート初期化
	gSys.com_ok = NG;
/*	rt = Comm_Open("/dev/ttyACM0");*/
/*	rt = Comm_Open("/dev/ttyUSB0");*/
	rt = Comm_Open(devpath);
	if ( rt==OK ) {
		Comm_Setup( 460800, PAR_NON, BIT_LEN_8, 0, 0, CHR_ETX);
		gSys.com_ok = OK;
	}

}

// ----------------------------------------------------------------------------------
//	アプリケーション終了処理
// ----------------------------------------------------------------------------------
//	引数	: non
//	戻り値	: non
// ----------------------------------------------------------------------------------
void App_Close(void)
{
	printf("Application Close\n");
	
	if ( gSys.com_ok == OK) {
		Comm_Close();
	}
}

/*********************************************************************************
* Function Name  : HST_SendResp
* Description    : データを整形して送信する
* Input          : pucInput 送信データ
*                : 送信データサイズ
* Output         : 
* Return         : 
*********************************************************************************/
ULONG SendData(UCHAR *pucInput, USHORT usSize)
{
	USHORT usCnt;
	UCHAR ucWork;
	UCHAR ucBCC = 0;
	UCHAR *pucWrite = &CommSendBuff[0];
	USHORT usRealSize;
	
	// データ整形 
	*pucWrite = CHR_DLE;					// DLE 
	pucWrite++;
	*pucWrite = CHR_STX;					// STX 
	pucWrite++;
	usRealSize =2;
	
	for (usCnt = 0; usCnt < usSize; usCnt++) {
		ucWork = pucInput[usCnt];
		if (ucWork == CHR_DLE) {			// データが0x10ならば0x10を付加 
			*pucWrite = CHR_DLE;			// DLE付加 
			pucWrite++;						// 書き込み先 
			usRealSize++;					// 実サイズ
			// BCCは計算しない!
		}
		*pucWrite = ucWork;					// データ 
		ucBCC ^= ucWork;					// BCC 
		pucWrite++;							// 書き込み先 
		usRealSize++;						// 実サイズ 
	}
	
	*pucWrite = CHR_DLE;					// DLE 
	pucWrite++;
	*pucWrite = CHR_ETX;					// ETX 
	ucBCC ^= CHR_ETX;						// BCC計算 
	pucWrite++;
	*pucWrite = ucBCC;						// BCC付加 
	usRealSize += 3;
	
	Comm_SendData(&CommSendBuff[0], usRealSize);
	
	return OK;
}

void GetProductInfo(void)
{
	USHORT len;
	
	printf("Get SensorInfo\n");
	len = 0x04;								// データ長
	SendBuff[0] = len;						// レングス
	SendBuff[1] = 0xFF;						// センサNo.
	SendBuff[2] = CMD_GET_INF;				// コマンド種別
	SendBuff[3] = 0;						// 予備
	
	SendData(SendBuff, len);
}

void SerialStart(void)
{
	USHORT len;
	
//	printf("Start\n");
	len = 0x04;								// データ長
	SendBuff[0] = len;						// レングス
	SendBuff[1] = 0xFF;						// センサNo.
	SendBuff[2] = CMD_DATA_START;			// コマンド種別
	SendBuff[3] = 0;						// 予備
	
	SendData(SendBuff, len);
}

void SerialStop(void)
{
	USHORT len;
	
	printf("Stop\n");
	len = 0x04;								// データ長
	SendBuff[0] = len;						// レングス
	SendBuff[1] = 0xFF;						// センサNo.
	SendBuff[2] = CMD_DATA_STOP;			// コマンド種別
	SendBuff[3] = 0;						// 予備
	
	SendData(SendBuff, len);
}

