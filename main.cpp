//#define SIM (1)
#include <predef.h>
#include <init.h>
#include <stdio.h>
#include <ctype.h>
#include <startnet.h>
#include <dhcpclient.h>
#include <system.h>
#include <string.h>

#include <nbrtos.h>
#include <nbstring.h>
#include <config_obj.h>


#include <fdprintf.h>
#include <pins.h>
#include <serial.h>
#include <serinternal.h>
#include <sim.h>
#include <pitr_sem.h>
#include <pin_irq.h>
#include <intcdefs.h>
#include <cfinter.h>
#include <multipartpost.h>
#include <math.h>

#include "SinLookup.h"
#include "servodrive.h"
#include "introspec.h"
#include "lidar.h"
#include "dsm2.h"
#include "mpu9250.h"
#include "nav.h"
#include "path.h"
#include "SimpleAD.h"
#include "runprop.h"
LOGFILEINFO;



RunProperties RunProps;

OS_SEM MainTaskSem;

static uint32_t tPedStop;
static bool DoLidarWarnings;

const char * AppName="Avc2017";


#define STEER_SERVO (0)
#define MOTOR_SERVO (1)



volatile int nActiveMode;// 340 1025 1780 Ch 5
volatile int nFileMode;	 //340 1025 1780 Ch 6
bool bLidarStop; 

extern int32_t GZero[3];
extern int32_t ZeroCount;


volatile uint32_t OdoCount;
volatile uint32_t DtOdoCount;
volatile uint32_t LastOdoTime;

float calc_speed;
float last_motor;
const float SPEED_CONST = (125000000*4.58*3600)/(5280*12); //(Clock * In_per_odo * sec_perhour)/(ft_per_mile*in_per_feet)
int CurPathIndex;
float SegSpeed;
bool bStopHere;
float SegRotv;

class LidarCornerParams:public config_obj
{
public:
LidarCornerParams():config_obj(appdata,"LidarCorner") {};
    config_int det_diff{400000,"diff"}; //Lidar counts are 1250/cm so ~125in
	ConfigEndMarker;
};
LidarCornerParams lidar_corner;




void OdoIrq(void)
{
uint32_t t=sim2.timer[3].tcn;
OdoCount++;

uint32_t dt;
if(t<LastOdoTime)
{//Roll over every 34 sec or so...
 dt=(t-0x80000000)-(LastOdoTime-0x80000000);
}
else
{
dt=(t-LastOdoTime);
}
DtOdoCount=dt;
LastOdoTime=t;

MainTaskSem.Post();
}


volatile static uint8_t bLIDAR_MODE;
volatile static uint32_t LIDAR_LAST_START;
volatile uint32_t LIDAR_VALUE;
volatile uint32_t LIDAR_COUNT;
volatile uint32_t LIDAR_MIN;


INTERRUPT(LIDAR_ISR,0x2700)
{
	sim2.timer[3].ter=3;
	if(bLIDAR_MODE==1)
		{
		sim2.timer[3].tmr=0x0083;   // 0000 0000 01 0 0 0 0 1 1 //Rising edge
		LIDAR_LAST_START=sim2.timer[3].tcr;		
		bLIDAR_MODE=2;
		}
	else
		if(bLIDAR_MODE==2)
		{
		
			sim2.timer[3].tmr=0x0043;   // 0000 0000 01 0 0 0 0 1 1 //Rising edge
			bLIDAR_MODE=1;
			uint32_t v=sim2.timer[3].tcr;
#ifndef SIM
			LIDAR_VALUE=(v-LIDAR_LAST_START);
			if (LIDAR_VALUE<LIDAR_MIN)
			{
			 LIDAR_MIN=LIDAR_VALUE;
			}
#endif
            LIDAR_COUNT++;
		}
}



void InitTimer3()
{
	SETUP_DMATIMER3_ISR(&LIDAR_ISR,2);
	sim2.timer[3].txmr=0;
	sim2.timer[3].ter=3;
	sim2.timer[3].trr=0;
	sim2.timer[3].tcr=0;
	sim2.timer[3].tcn=0;
	sim2.timer[3].ter=3;
	
sim2.timer[3].tmr=0x0043;   // 0000 0000 01 0 0 0 0 1 1 //Rising edge
bLIDAR_MODE=1;

}



void LCD_X_Y(int sp, int x, int y)
{
if((x>15) || (y>1)) return;
uint8_t b[2];
b[0]=254;
b[1]=128+x+64*y;
write(sp,(char *)b,2);

}

void LCD_CLS(int sp)
{
 LCD_X_Y(sp,0,0);
 write(sp,"                    ",16);
 write(sp,"                    ",16);
}

extern volatile uint32_t LastIerror;
extern volatile uint32_t nCor;
extern volatile double lferror;




void ProcessNewRCPos(uint32_t ch, uint32_t v)
{
static uint32_t LastCount;
	   switch(ch)
	   {
	   case 0: break; //Alieron
	
	   case 1: break; //Elevator
	
	   case 2: break; //Throttle

	   case 3: break; //Rudder

	   case 4: //Gear Switch
		   if(v<1024) bLog=false;
		   else
			          bLog=true;
		   break;
	
	   case 5: //nActiveMode
		     break;

	   case 6://File Mode
		   if(v<700) nFileMode=0;
		   else
		    if(v<1400) nFileMode=1;
			else
			nFileMode=2;
		break;
   }


if(RCFrameCnt!=LastCount)
  {
	MainTaskSem.Post();
	LastCount=RCFrameCnt;
  }
}



START_INTRO_OBJ(RCStateObj,"RCCh")
uint16_element c0{"c0"};
uint16_element c1{"c1"};
uint16_element c2{"c2"};
uint16_element c3{"c3"};
uint16_element c4{"c4"};
uint16_element c5{"c5"};
uint16_element c6{"c6"};
uint16_element c7{"c7"};
END_INTRO_OBJ;


RCStateObj RcLog;

void LogRC()
{
USER_ENTER_CRITICAL()

RcLog.c0=rc_ch[0];
RcLog.c1=rc_ch[1];
RcLog.c2=rc_ch[2];
RcLog.c3=rc_ch[3];
RcLog.c4=rc_ch[4];
RcLog.c5=rc_ch[5];
RcLog.c6=rc_ch[6];
RcLog.c7=rc_ch[7];

USER_EXIT_CRITICAL()
RcLog.Log();
}


float TargetHeading FAST_USER_VAR;

bool bIsMoving()
{
uint32_t t=sim2.timer[3].tcn;
uint32_t lot=LastOdoTime;
uint32_t dt;
if(t<lot)
{//Roll over every 34 sec or so...
 dt=(t-0x80000000)-(lot-0x80000000);
}
else
{
dt=(t-lot);
}
if(dt<125000000) return true;
return false;
}


START_INTRO_OBJ(ModeChangeObj,"ModeChange")
uint16_element m{"mode"};
END_INTRO_OBJ;

START_INTRO_OBJ(BootSecObj,"BootSecs")
uint32_element s{"Secs"};
uint32_element nRc{"nRc"};
uint32_element nOdo{"nOdo"};
uint32_element nSf{"nSf"};

END_INTRO_OBJ;



START_INTRO_OBJ(SteerLoopObj,"SteerLoop")
float_element t{"targ"};
float_element e{"err"};
//float_element i{"ierr"};
//float_element rve{"rverr"};
float_element steer{"steer"};
float_element motor{"motor"};
//uint32_element dt{"sldt"};
//uint8_element m{"move"};


END_INTRO_OBJ;


START_INTRO_OBJ(CurPosObj,"POS")
float_element x{"x"};
float_element y{"y"};
float_element px{"px"};
float_element py{"py"};
uint32_element pc{"pointcnt"};
uint32_element dt{"dt"};
uint32_element sn{"sn"};
uint32_element tc{"tc"};
uint32_element nproj{"nproj"};
//uint32_element dt1{"dt1"};
//uint32_element dt2{"dt2"};
//uint32_element dt3{"dt3"};
//uint32_element dto{"dto"};
int32_element b{"b"};
END_INTRO_OBJ;


START_INTRO_OBJ(CornerRec,"CornerRc")
int32_element s{"s"};
uint32_element lc{"lc"};
float_element  x{"x"};
float_element  y{"y"};
int32_element  h{"h"};
END_INTRO_OBJ;


START_INTRO_OBJ(NextPointRec,"NextPt")
int32_element rp{"rp"};
int32_element cp{"cp"};
END_INTRO_OBJ;

START_INTRO_OBJ(LidarCorrectObj,"LidarCorr")
float_element dx{"dx"};
float_element dy{"dy"};
float_element hCor{"HCor"};
float_element b4{"b4"};
float_element aft{"aft"};
float_element th{"th"};
END_INTRO_OBJ;



START_INTRO_OBJ(NavCalcRec,"NavCalc")
float_element xtk{"xtk"};
float_element hd{"hd"};
float_element th{"th"};
float_element xh{"xh"};
float_element ts{"tspeed"};
float_element x{"x"};
float_element y{"y"};
float_element da{"da"};
float_element ra{"ra"};
int32_element w{"w"};
END_INTRO_OBJ;


static NextPointRec NextPointObj FAST_USER_VAR;
static NavCalcRec NavCalcObj FAST_USER_VAR;
static ModeChangeObj mco FAST_USER_VAR;
static SteerLoopObj slo FAST_USER_VAR;
static BootSecObj   bso FAST_USER_VAR;
static CurPosObj    cpo FAST_USER_VAR;
static LidarCorrectObj lco FAST_USER_VAR;
static CornerRec corn FAST_USER_VAR;


static fPoint  CurPos FAST_USER_VAR;


const float max_steer =0.75;
const float min_steer =-0.75;

//Throttle 0.33= 18 mph
//		   0.2=	 11 mph
//		   0.125 =4.5
//y=mx+b
//


float ManageSpeed(float target_speed)
{
float sv=(target_speed*(float)RunProps.Speed_M)+(float)RunProps.Speed_B;
float err=target_speed-calc_speed;
//Lets try 0.1 per 5mph  0.1/5 = 0.02

return sv+((float)RunProps.Speed_G*err);

}

   //zot
void Steer()
{
// static float ierr FAST_USER_VAR;
 //static float last_heading FAST_USER_VAR;
// uint32_t ts=sim2.timer[3].tcn;

 float head;
 if(RunProps.UseMag)
	head=IntegratedHeading;
	else
	head=RawHeading;

 //float cur_rot_dps=turn_angle(last_heading,head)*50; //Magicv 50 is RC frame rate 

 //float rotv_error=(SegRotv-cur_rot_dps); //left is minus
float err=(TargetHeading-head);


if(RawPaths[CurPathIndex].m_mode==eBar) err*=2; 


 if(err>180) err-=360;
 if(err<-180) err+=360;


 float sv=(float)RunProps.SteerP*err+ 
	      //(float)RunProps.SteerI*ierr+
          ((float)RunProps.SteerD*(float)RotVel[2]/1000.0)
		  //+(float)RunProps.SteerVE*rotv_error
	      ;

	      



 if(sv>max_steer) sv=max_steer;
 if(sv<min_steer) sv=min_steer;
// slo.rve=rotv_error;
 slo.t=TargetHeading;
 slo.e=err;

 sv-=(float)RunProps.SteerZero;
 SetServoPos(STEER_SERVO ,-sv);
/*
if(bIsMoving())
{
 slo.m=1;
// ierr+=err;
}
else
{
 slo.m=0;
}
*/
//slo.i=ierr;
slo.steer=-sv;
slo.motor=last_motor;
//slo.dt=sim2.timer[3].tcn-ts;
slo.Log();
//last_heading=head;
}





void ProcessNewImuData()
{


}





fPoint AvgPoint(fPoint & p1, fPoint& p2)
{
	fPoint pr;
	pr.x=(p1.x+p2.x)/2;
	pr.y=(p1.y+p2.y)/2;
	return pr;
}

int32_t AvgHIndex(int32_t h1,int32_t h2)
{
  int32_t v=(h1+h2)/2;
  if (((h1-h2)>131072)||((h2-h1)>131072))
	   {v-=131072;}
  v=v&0x3FFFF;
  return v;
}



float avg_angle(float a1, float a2)
{
float v;
	v=(a1+a2)/2;
	if(fabs(a1-a2) > 180.0)
	{
	  v=v-180;
	}
 if(v>180.0) v-=360.;
 if(v<-180) v+=360.0;
 return v;
}




void NextPoint()
{
	if(RawPaths[CurPathIndex].end_speed!=0)	CurPathIndex++;
    else
	{
	 bStopHere=true;
	 if(RunProps.StopOnRcLoss==false) bLog=false;
	 return;
	}

	DoLidarWarnings=((CurPathIndex>1) && (RawPaths[CurPathIndex].m_mode==eNormal) &&(RawPaths[CurPathIndex].end_speed!=0));
	   
	   


	 RawPaths[CurPathIndex].CornerSatisfied=false;
    NextPointObj.rp=CurPathIndex;
    NextPointObj.cp=RawPaths[CurPathIndex].ref_path_num;
	NextPointObj.Log();

}


void ProcessCorner(int sign, uint32_t counts,fPoint p_at_det,int32_t head_index)
{
   corn.s=sign;
   corn.lc=counts;
   corn.x=p_at_det.x;
   corn.y=p_at_det.y;
   corn.h=head_index;
   corn.Log();
}
/* Made with these rules
      int s=0;
	  if(wb==0) s=0;
	  else
	  if(wb==0xff) s=0;
	  else
	  if((wb&0xF)==0xF) s=1;
	  else
	  if((wb&0xF0)==0xF0) s=-1;
	  else
	  if((wb&0x18)==0x18) s=0;
	  else
	  if((wb&0x10)==0x10) s=-1;
	  else
	  if((wb&0x08)==0x08) s=1;
	  else
	  if((wb&0x20)==0x20) s=-1;
	  else
	  if((wb&0x04)==0x04) s=1;
	  else
	  if((wb&0x40)==0x40) s=-2;
	  else
	  if((wb&0x02)==0x02) s=2;
	  else
	  if((wb&0x80)==0x80) s=-2;
	  else
	  if((wb&0x01)==0x01) s=2;
	  else
		  s=0; //Should not be able to get here
*/


const int ActionTable[256]={
	+0,+2,+2,+2,+1,+1,+1,+1,+1,+1,+1,+1,+1,+1,+1,+1,-1,-1,-1,-1,-1,-1,-1,-1,+0,+0,+0,+0,+0,+0,+0,+1,
-1,-1,-1,-1,-1,-1,-1,-1,+1,+1,+1,+1,+1,+1,+1,+1,-1,-1,-1,-1,-1,-1,-1,-1,+0,+0,+0,+0,+0,+0,+0,+1,
-2,-2,-2,-2,+1,+1,+1,+1,+1,+1,+1,+1,+1,+1,+1,+1,-1,-1,-1,-1,-1,-1,-1,-1,+0,+0,+0,+0,+0,+0,+0,+1,
-1,-1,-1,-1,-1,-1,-1,-1,+1,+1,+1,+1,+1,+1,+1,+1,-1,-1,-1,-1,-1,-1,-1,-1,+0,+0,+0,+0,+0,+0,+0,+1,
-2,-2,+2,+2,+1,+1,+1,+1,+1,+1,+1,+1,+1,+1,+1,+1,-1,-1,-1,-1,-1,-1,-1,-1,+0,+0,+0,+0,+0,+0,+0,+1,
-1,-1,-1,-1,-1,-1,-1,-1,+1,+1,+1,+1,+1,+1,+1,+1,-1,-1,-1,-1,-1,-1,-1,-1,+0,+0,+0,+0,+0,+0,+0,+1,
-2,-2,-2,-2,+1,+1,+1,+1,+1,+1,+1,+1,+1,+1,+1,+1,-1,-1,-1,-1,-1,-1,-1,-1,+0,+0,+0,+0,+0,+0,+0,+1,
-1,-1,-1,-1,-1,-1,-1,-1,+1,+1,+1,+1,+1,+1,+1,+1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,+0
};




  static uint32_t LastLidarScanCount; 

float DoNewNavCalcs(const fPoint &proj_pt, float proj_head)
{

	float th;
	float xtk;
	float ts;
	float t_rotv;
	float head=proj_head;
   // uint32_t tims=sim2.timer[3].tcn; 

 
  //  float head;
 //   if(RunProps.UseMag)
 //      head=IntegratedHeading;
 //      else
 //      head=RawHeading;
   
	if(bStopHere) return 0;

	if(RawPaths[CurPathIndex].NavCalc(proj_pt,proj_head,th,xtk,ts,t_rotv)) 
	{
     NextPoint();
	 if(bStopHere) return 0;
	 RawPaths[CurPathIndex].NavCalc(proj_pt,proj_head,th,xtk,ts,t_rotv);
	}

    SegSpeed=ts;
	if((RawPaths[CurPathIndex].m_mode==ePed) && (tPedStop!=0))
		SegSpeed=6;
	SegRotv=t_rotv;

   // NavCalcObj.dt1=sim2.timer[3].tcn-tims; 
 
 float xh=0;
  xh=-xtk*(float)RunProps.CrossAngleScale;
  if(xh<-(float)RunProps.CrossMaxCorrect) xh=-(float)RunProps.CrossMaxCorrect;
  if(xh> (float)RunProps.CrossMaxCorrect) xh=(float)RunProps.CrossMaxCorrect;
 
  //iprintf("xtk=%g r=%g th=%g\r\n",xtk,RawHeading,th);;

  //th is the direction we are headed for....
  //When TH is WAAAAAY off we have to make sure that 
  //The cross track does not push us beyond the 180 deg point...

  
  xh+=th;
  if(xh>180) xh-=360;
  if(xh<-180) xh+=360;

  float rerr=(th-head);
  if(rerr>180) rerr-=360;
  if(rerr<-180) rerr+=360;

  float err=(xh-head);
  if(err>180) err-=360;
  if(err<-180) err+=360;

  //If both errors are same sign after scaling then were good
  if((rerr<0) &&(err<0)) TargetHeading=xh;
  else
  if((rerr>0) &&(err>0)) TargetHeading=xh;
  else
  if(fabs(rerr)<(float)RunProps.CrossMaxCorrect)
	  TargetHeading=xh; 
  else
  TargetHeading=th;

  if((RawPaths[CurPathIndex].m_mode==ePed) && (tPedStop==0) && DoPedStop())
  {
   tPedStop=TimeTick+60;
   SegSpeed=6;
  }


  if(RawPaths[CurPathIndex].m_mode==eBar)
  {
    if(LidarScanCount==LastLidarScanCount) bLidarStop=true;
   else
   {
   LastLidarScanCount=LidarScanCount;
   int da=(int)turn_angle(head,TargetHeading);
   NavCalcObj.da=da;
   int ra=GetLidarHeading(da);
   if(ra!=-999)
   {
   TargetHeading+=(ra-da);
   if(TargetHeading>180) TargetHeading-=360;
   if(TargetHeading<(-180))TargetHeading+=360;
   }
   bLidarStop=false;
   NavCalcObj.ra=ra;
   }

  }
  else
  {NavCalcObj.da=0;
   NavCalcObj.ra=0; 
  }
  NavCalcObj.w=0; 

  if((DoLidarWarnings ) && (warning_byte!=0xFF))
  {
	  NavCalcObj.w=warning_byte;
	  TargetHeading+=ActionTable[warning_byte]*RunProps.WarnTurnAngle;
  }
  else
	  NavCalcObj.w=0;

   if(TargetHeading>180) TargetHeading-=360;
   if(TargetHeading<(-180))TargetHeading+=360;

  
  NavCalcObj.xtk=xtk;
  NavCalcObj.hd=TargetHeading;
  NavCalcObj.ts=SegSpeed;
  NavCalcObj.xh=xh;
  NavCalcObj.th=th;
  NavCalcObj.x=CurPos.x;
  NavCalcObj.y=CurPos.y;
 // NavCalcObj.dt=sim2.timer[3].tcn-tims;
  NavCalcObj.Log();
  return xtk;
}







void ModeChange(int new_mode, int prev_mode)
{
 mco.m=(int16_t)new_mode;
 mco.Log();
 if((prev_mode==0) && (new_mode!=0))
 {
	 CurPos=RawPaths[0].start_point;
	 //CurPos.y-=70;

     CurPathIndex=0;
	 bStopHere=false;
	 tPedStop=0;

  if(!RunProps.UseMag)
  {
   SetRawHeading(RawPaths[0].start_head);
   DoNewNavCalcs(CurPos,RawHeading);
  }
  else
  {
   NextPoint();
   DoNewNavCalcs(CurPos,IntegratedHeading);

  }


 }


}



extern volatile uint32_t LidarRxc;
extern void RegisterPost();

volatile bool bRCFrameError;

int LCD_SER;


void LCD_DisplayTask(void * pd)
{
	uint32_t LastLidar=LidarScanCount;
	uint32_t LastRCFrame=RCFrameCnt;

	while (1)
   {//LCD processing loop

		OSTimeDly(20);

	{
	    LCD_X_Y(LCD_SER,0,0);
		if(Secs &1)
			fdprintf(LCD_SER,"X%ld,%3.0f,%3.0f ",OdoCount,IntegratedHeading,RawHeading);
		else
			fdprintf(LCD_SER,"+%ld,%3.0f,%3.0f ",OdoCount,IntegratedHeading,RawHeading);

	if((bCompassCalDirty) && (Secs>20) && ((Secs %30)==0))
	 {
	   SaveConfigToStorage();
	   bCompassCalDirty=false;
	   iprintf("Compass cal saved\r\n");
	 }


	  LCD_X_Y(LCD_SER,0,1);
	  fdprintf(LCD_SER,"[%08b] L:%3ld",warning_byte,GetLogPercent());
//	  fdprintf(LCD_SER,"L:%ld:R:%2ld %3ld ",(LidarScanCount-LastLidar)/1000,RCFrameCnt-LastRCFrame,GetLogPercent());

	  if(RCFrameCnt-LastRCFrame<10) 
		  bRCFrameError=true;
	  else
		  bRCFrameError=false;
//
//	  printf("L:%ld:R:%2ld %3ld ",(LidarScanCount-LastLidar)/1000,RCFrameCnt-LastRCFrame,GetLogPercent());
//      printf("OdoCount %ld LIDAR_VALUE=%ld\r\n",OdoCount,LIDAR_VALUE);
//      printf("AD [%04X,%04X,%04X,%04X]\r\n",GetADResult(0),GetADResult(1),GetADResult(2),GetADResult(3));

	  LastLidar=LidarScanCount;
	  LastRCFrame=RCFrameCnt;
	}
  }
}







bool newRC()
{
static uint32_t LastRCFrameCnt FAST_USER_VAR;
uint32_t RC_In=RCFrameCnt;
if(RC_In!=LastRCFrameCnt)
{
  LastRCFrameCnt=RC_In;
  return true;
}
return false;
}

uint32_t newODO()
{
static uint32_t myLastODOCnt FAST_USER_VAR;
uint32_t ODO_In=OdoCount;
if(ODO_In!=myLastODOCnt)
{
  uint32_t rv=ODO_In-myLastODOCnt;
  myLastODOCnt=ODO_In;
  return rv;
}
return 0;
}


bool newServoFrame()
{
static uint32_t lastServoFrameCnt FAST_USER_VAR;
uint32_t SF_In=ServoFrameCnt;
if(SF_In!=lastServoFrameCnt)
{
lastServoFrameCnt=SF_In;
return true;
}
return false;
}








uint32_t ProcessLidarLines(int32_t &b,uint32_t &tc);

#ifdef SIM

static int Sim_FD;
volatile bool bNeedToAck;

void SimDo(int32_t new_raw_head);
void ProcessSimChar(char * buffer,int cnt)
{
static int state;
static unsigned int tv[5];
  if(cnt<=0) return;
for(int i=0; i< cnt; i++)
{
if(buffer[i]=='I')
  {
	tv[0]=0;
	tv[1]=0;  //Raw heading
	tv[2]=0;  //Odo count
	tv[3]=0;  //DT odo
	tv[4]=0;  //Side Lidar
	state=1;
  }
if(buffer[i]=='\r')
  {
   if(state==4)
   {bool bSem=(OdoCount!=tv[2]);
	OdoCount=tv[2];
	DtOdoCount=tv[3]; 
	LIDAR_VALUE=tv[4];
	SimDo((int)tv[1]);
	bNeedToAck=true;
	MainTaskSem.Post();
	state=0;
   }
   else
	   iprintf("?");

  }
if(buffer[i]==',')
 {
	state++;
 }
if((buffer[i]>='0') && (buffer[i]<='9')) tv[state]+=buffer[i]-'0';
if((buffer[i]>='a') && (buffer[i]<='f')) tv[state]+=10+(buffer[i]-'a');
if((buffer[i]>='A') && (buffer[i]<='F')) tv[state]+=10+(buffer[i]-'A');
}

}



#include <tcp.h>
void TCP_Server(void * pd)
{
 char buffer[256];

  int fdl=listen(INADDR_ANY,9999,1);
  while(1)
  {
	Sim_FD=accept(fdl,NULL,NULL,0);
	if (Sim_FD>0)
	   {
	
		  int rv=read(Sim_FD,buffer,256);
			ProcessSimChar(buffer,rv);
		  while(rv>0)
		  {
		   //iprintf("Got %d bytes\r\n",rv);
		   if(rv<0) break;

		   rv=read(Sim_FD,buffer,256); 
		   ProcessSimChar(buffer,rv);
		  }
		  close(Sim_FD);
		  Sim_FD=0;
	   }
	}
}

#endif

/*
void TestSet(fPoint p1, fPoint p2)
{
 float h=p2.HeadToHereDeg(p1);
 float d=p2.Dist(p1);
 printf("[%g,%g] to [%g,%g] H:%g D=%g\r\n",p1.x,p1.y,p2.x,p2.y,h,d);
}

void Testnav()
{
fPoint p1(0,0);
fPoint p2(0,10);
TestSet(p1,p2);
p2.x=10;
p2.y=10;
TestSet(p1,p2);
p2.y=0;
TestSet(p1,p2);
p2.y=-10;
TestSet(p1,p2);
p2.x=0;
TestSet(p1,p2);
p2.x=-10;
TestSet(p1,p2);
p2.y=0;
TestSet(p1,p2);
p2.y=10;
TestSet(p1,p2);
p2.x=8.6;
p2.y=5;
TestSet(p1,p2);
p2.y=-5;
TestSet(p1,p2);
p2.x=-8.6;
TestSet(p1,p2);
p2.y=5;
TestSet(p1,p2);

}
  */
void UserMain(void * pd)
{
	initWithWeb();
	if ( !EnableMultiPartForms( 500000 ) )
	{
		iprintf( "EnableMultiPartForms() initialization failed\r\n" );
	}
	RegisterPost();
	LCD_SER=SimpleOpenSerial(5,9600);
	LCD_CLS(LCD_SER);


	iprintf("AVC  at  %s on %s\r\n",__TIME__,__DATE__);
	fdprintf(LCD_SER,"Setting up Trig");
    uint32_t ttn=TimeTick;
    SetUpTables((float)RunProps.ODODist);
    iprintf("Table init Took %ld ticks\r\n",TimeTick-ttn);

	//Testnav();


   Pins[27].function(PIN_27_I2C0_SCL    );//I2C for IMU
   Pins[29].function(PIN_29_I2C0_SDA    );//I2C For IMU
   Pins[50].function(PIN_50_IRQ2  ); //IRQ for IMU

   Pins[13].function(PIN_13_UART2_RXD);	//LIDAR RX
   Pins[16].function(PIN_16_UART2_TXD);	//LIDAR TX
   Pins[19].function(PIN_19_T0OUT);	   //Steer servo
   Pins[21].function(PIN_21_T1OUT);	   //Throttle Servo

   Pins[25].function(PIN_25_T3IN); //LIDAR pulse
   Pins[14].function(PIN_14_UART6_RXD);	//RC RX
   Pins[38].function(PIN_38_UART5_TXD); //TX to Serial LCD

   iprintf("*************************pop path **********************\r\n");
   PopulatePath();
   iprintf("*************************end path **********************\r\n");
   iprintf("Size of a single path_element is :%ld\r\n",sizeof(path_element));


   LCD_CLS(LCD_SER);

   SetPinIrq(49,-1,OdoIrq);




   iprintf("Application started\n");


   LogFileVersions();
   LogAppRecords();

   InitTimer3();
#ifdef SIM
   OSSimpleTaskCreatewName(TCP_Server,MAIN_PRIO-3,"TCP Server");
   bIMU_Id=true;
   ZeroCount=100;
#else
   Mpu9250setup(MAIN_PRIO-2);
#endif

   InitLidar(2,MAIN_PRIO-1,MAIN_PRIO+2);
   InitLogFtp(MAIN_PRIO+1);

   ServoDriveInit();
   InitDSM2Rx(6);
   RCCallBack=ProcessNewRCPos;

    InitSingleEndAD();
	StartAD();
	while(!ADDone()) OSTimeDly(1);

	if(
	   (GetADResult(1)>25000) &&
	   (GetADResult(2)>25000) &&
	   (GetADResult(3)>25000)
	  ) 
		RunProps.StopOnRcLoss=false;




    ImuMode=eCalibrating;
    OSTimeDly(TICKS_PER_SECOND);
   uint32_t Lsec=Secs;

   while(Lsec==Secs) asm(" nop");

   uint32_t LastImu=IMUSample;

   while(!bIMU_Id)
   {
	   LCD_X_Y(LCD_SER,0,0);
	   fdprintf(LCD_SER,"NO IMU");
  }

   LCD_X_Y(LCD_SER,0,0);
   fdprintf(LCD_SER,"%s",__DATE__);
   LCD_X_Y(LCD_SER,0,1);
   fdprintf(LCD_SER,"%s",__TIME__);
   OSTimeDly(40);


#ifndef SIM

   while(Secs<=(Lsec+RunProps.CalTime ))
   {
      OSTimeDly(TICKS_PER_SECOND);
      LCD_X_Y(LCD_SER,0,0);
        fdprintf(LCD_SER,"Cal: %ld of %ld ",(Lsec-Secs),(int)RunProps.CalTime );
      printf("Cal: %ld of %d \r\n",(Secs-Lsec),(int)RunProps.CalTime );
   }

#endif
   LCD_CLS(LCD_SER);
   iprintf("Samples=%ld DT=%ld\r\n",IMUSample-LastImu, Secs-(Lsec+1));

   iprintf("GZ[2]=%ld n=%ld off=%ld\r\n",GZero[2], ZeroCount,GZero[2]/ZeroCount);
   ImuMode=eRunning;
   OSTimeDly(10);
   SetInitalCompass();

   while(Lsec==Secs) asm("nop");



   pNotifyNextFrameSem=&MainTaskSem;

   uint32_t nRc=0;
   uint32_t nOdo=0;
   uint32_t nSf=0;
   
   if(RunProps.StopOnRcLoss==false)
   {
	 //ZOT

   	 LCD_X_Y(LCD_SER,0,0);
	 fdprintf(LCD_SER,"Release Button");
	 int cnt=0;
	 int adr=500;  
	   while(cnt<5)
	   {
		   StartAD();
		   while(!ADDone()) MainTaskSem.Pend(1);
		   adr=GetADResult(0);
		   if(adr>500) cnt++;
		   else cnt=0;
	   }
	   LCD_CLS(LCD_SER);
	   LCD_X_Y(LCD_SER,0,0);
       fdprintf(LCD_SER,"Armed");

	   cnt=0;
	   while(cnt<5)
	   {
		   StartAD();
		   while(!ADDone()) MainTaskSem.Pend(1); 
		   adr=GetADResult(0);
		   if(adr<500) cnt++;
		   else cnt=0;
	   }
	   LCD_CLS(LCD_SER);
       fdprintf(LCD_SER,"Go");

	  iprintf("Starting!\r\n");
	  bLog=true;
	  nActiveMode=2;
	  ModeChange(2,0);


   }
   else
   OSSimpleTaskCreatewName(LCD_DisplayTask,MAIN_PRIO+3,"LCD State");


/*************************While the world turns ****************************/
  while(1)
  {
    MainTaskSem.Pend(2);
#ifdef SIM
   if(bNeedToAck)
   {
	bNeedToAck=false;
	write(Sim_FD,"A",1);
   }
#endif
	
   if(Lsec!=Secs)
	   { Lsec=Secs;
		bso.s=Secs;
		bso.nRc=nRc;
		bso.nOdo=nOdo;
		bso.nSf=nSf;
		bso.Log();
		nRc=0;
		nOdo=0;
		nSf=0; 
       }

/*************************Process new RC info *******************/
	if(newRC())
   {
	nRc++;
     static int LastActiveMode FAST_USER_VAR;
	
	if(rc_ch[5]<700) nActiveMode=0;
     else if(rc_ch[5]<1200) nActiveMode=1;
		               else    nActiveMode=2;


	if(LastActiveMode!=nActiveMode)
		{
		 ModeChange(nActiveMode,LastActiveMode);
		
		}
		LastActiveMode =nActiveMode;
   // LogRC();
   }
   


/*************************If stopped for Lidar recover *******************/

	if((bLidarStop==true) && (LidarScanCount!=LastLidarScanCount))
		{
		 bLidarStop=false;
		 DoNewNavCalcs(CurPos,RawHeading);
	    }


/*******************Process new ODO count ******************************/
uint32_t delta_odo=newODO();
   if(delta_odo)
   {
       nOdo++;
	//uint32_t ts=sim2.timer[3].tcn; 
	   if(DtOdoCount==0)
		calc_speed=0;
		   else
	    calc_speed=SPEED_CONST/(float)DtOdoCount;


	 float head;
	 static float last_head FAST_USER_VAR;
	 static int Lidar_Skip FAST_USER_VAR; 
	 static int try_lidar_again;
     if(RunProps.UseMag)
		 head=IntegratedHeading;
		 else
		 head=RawHeading;

		// cpo.dt1=(sim2.timer[3].tcn-ts);

         //Can't average headings...s
		 
		 float usehead=avg_angle(head,last_head);
		 unsigned long usehead_index=ConvertDegToIndex(usehead);  
		 float dx;
		 float dy;
		 LookUpSinCosDistIndex(usehead_index,dx,dy);
		 if(delta_odo!=1)
		 {
		  dy*=delta_odo;
		  dx*=delta_odo;
		 }
		 
		 last_head=head;
		 CurPos.x+=dx;
     	 CurPos.y+=dy;
		 cpo.x=CurPos.x;
		 cpo.y=CurPos.y;
		 
		 uint32_t tc=0;
		 int32_t b=0;
		 uint32_t pc=0;


		{ static  uint32_t PLidarArray[16] FAST_USER_VAR; ;
	      static  fPoint pre_pos[16];
		  static  int head_index[16];
	      static  int index;

	      PLidarArray[index&0x0f]=LIDAR_VALUE;
	      pre_pos[index&0x0f]=CurPos;
		  head_index[index&0x0f]=usehead_index;


	      uint32_t a1=(PLidarArray[(index-3)&0x0f]+PLidarArray[(index-2)&0x0f])/2;
	      uint32_t a0=(PLidarArray[(index-1)&0x0F]+PLidarArray[index&0x0f])/2;

			 if (a1>(a0+lidar_corner.det_diff))
			 {
			  if((PLidarArray[(index-3)&0x0f]>a0) && (PLidarArray[(index-2)&0x0f]>a0))
				  {
				  fPoint fp= AvgPoint(pre_pos[(index-2)&0x0f],pre_pos[(index-1)&0x0f]);
				  int32_t ah=AvgHIndex(head_index[(index-2)&0x0f],head_index[(index-2)&0x0f]);

				  ProcessCorner(-1,a0,fp,ah);
				  }//From Open Corner to toward me
			 }
			else
			 if(a0>(a1+lidar_corner.det_diff))
			   {
				if((PLidarArray[(index-3)&0x0f]<a0) && (PLidarArray[(index-3)&0x0f]<a0))
				{ 
				  fPoint fp=AvgPoint(pre_pos[(index-2)&0x0f],pre_pos[(index-1)&0x0f]);
				  int32_t ah=AvgHIndex(head_index[(index-2)&0x0f],head_index[(index-2)&0x0f]);
					ProcessCorner(+1,a1,fp,ah);
				}
			  }
				index++;
		}


		 fPoint ProjectPos=CurPos;
		 unsigned long was_head_index=usehead_index;
		  if(DtOdoCount!=0)
		  {
          //We want to project 200msec ahead...
		 int ProjectDist=(25000000/DtOdoCount);
		 unsigned long deltahead_index=ConvertDegToIndex(turn_angle(last_head,head));
		 
		 //cpo.dt2=(sim2.timer[3].tcn-ts);



		 for(int i=0; i<ProjectDist; i++)
		 {
		  usehead_index+=deltahead_index;
		  LookUpSinCosDistIndex(usehead_index,dx,dy); 
          ProjectPos.x+=dx;
          ProjectPos.y+=dy;
		 }
		 usehead=ConvertIndexToFloat(usehead_index);
		 cpo.nproj=ProjectDist;

		}
		  else
		  {
			  cpo.nproj=0;
			  ProjectPos=CurPos;
			  usehead=head;
		  }
		 cpo.px=ProjectPos.x;
		 cpo.py=ProjectPos.y;
		 
		 //cpo.dt3=(sim2.timer[3].tcn-ts);

		 if(nActiveMode!=0)
			 { 
			 //Save XTK for Laser corrections...
			  float xtk=DoNewNavCalcs(ProjectPos,usehead);

			  
			 if(!LidarBusy()) 
			 {
			  int SlopeNum=GetLidarResult(b,tc,pc);
			  cpo.sn=SlopeNum;
			  LidarSampleStart(RawPaths[CurPathIndex].m_eWall);
			  cpo.dt=Lidar_Skip;
			  Lidar_Skip=0;
			  
			  cpo.pc=pc;
		      cpo.tc=tc;
		      cpo.b=b;

			  //Right LIDAR
			  //<128 is car pointed to wall
			  //128 is straight
			  //>128 car pointed away

			  //Left Lidar
			  //<128 Car is pointed away from wall
			  //>128 Car is pointed at wall
			  
			  //So to fix heading 
			  //<128 actual heading is more than correct so cor-
			  //>128 actual heading is less than correct so cor+

			  if(
				 (tc>5) && 
				 (pc>19) && 
				 (SlopeNum!=0) && 
				 (SlopeNum!=256)&&
				 (RawPaths[CurPathIndex].m_eWall!=eOff)&&
				 (try_lidar_again==0)
			    )
			  {//Do Lidar NAV Correction...
			      if(RawPaths[CurPathIndex].m_bAdjust_Heading)
				  {
					   float DegCor=SlopeNumToDeg(SlopeNum);
						//Segment heading
						//Measured Wall heading
						float WallHead=RawPaths[CurPathIndex].start_head;
						
						WallHead-=DegCor;
						if(WallHead<(-180)) WallHead+=360;
						else
						if(WallHead>180) WallHead-=360;
		
						lco.hCor=DegCor;
						lco.b4=RawHeading;
						 SetRawHeading(WallHead);
				  }
				  else
			      {
					  lco.b4=RawHeading;
				  }

				lco.aft=RawHeading;
				lco.th=RawPaths[CurPathIndex].start_head;

				//Now for B...dist
				//Both numbers + for to right - for to left
				b=b-RawPaths[CurPathIndex].wall_dist;
				//b=-100 dist -98  ->-2 we are 2" to far from left 	move left
				//b=-98  dist -100 -> +2 we are 2" to close to left	move right
               
				//b= 100 dist 98  -> +2 we are 2" to far from right	   ->move right
				//b= 98  dist 100 -> -2 we are 2" to close to the right ->move left

				//so b is positive if we need to move right to get on track
				//So coordiantes should be adjusted so that afterwards we are father left.
				
				//xtk is positive if we are too far right  neg too left
                
				//If b says we are too far left  b+2  xtk=+2 actuall error 0
				b-=xtk;

				//Now B has actual Lidar left/right error...
				//+ if we are left of track so actual coordinates 
				//are too far right


                //We have was_head_index from above
				LookUpSinCosIndex(was_head_index,dx,dy);

				//Course dy=1 dx=0 N  b+  x-=(dy*b)  
				//course dy=0 dx=1 E  b+  y+=(dx*b)
                //course dy=1 dx=1 NE     x-=(dy*b) y+=(dx*b)

				lco.dx=-b*dy;
                lco.dy=b*dx;
				lco.Log();
				CurPos.x+=-b*dy;
                CurPos.y+=b*dx;
				try_lidar_again=20;
			  }

			 }
		    else
			{
			 Lidar_Skip++;
			 cpo.dt=Lidar_Skip;
			}

		    }
		 else
		 {
			 if(!LidarBusy()) 
			  {
			  cpo.sn=GetLidarResult(b,tc,pc);
			   LidarSampleStart(eLeft);
			   cpo.dt=Lidar_Skip;
			   Lidar_Skip=0;
			   cpo.pc=pc;
			   cpo.b=b;
			  }
			  else
			  {
				  cpo.dt=0;
				  cpo.pc=0;
				  cpo.tc=0;
				  cpo.b=0;
			  }
		 }
        
	   // cpo.dto=(sim2.timer[3].tcn-ts);
		cpo.Log();
	   if(try_lidar_again) try_lidar_again--;

   }





/********************************Time to do new servo frame ***********************/

   if(newServoFrame())
   {
#ifdef SIM
	  if(Sim_FD)
	  {
		 fdprintf(Sim_FD,"S:%d,M:%d\r\n",GetServoCount(STEER_SERVO),GetServoCount(MOTOR_SERVO));
	  }
#endif
	  nSf++;
	   switch (nActiveMode)
	   {
	   case 0:
		    {
			 SetServoPos(STEER_SERVO,HiCon(rc_ch[1])); //Steer
             SetServoRaw(MOTOR_SERVO,rc_ch[2]); //Throttle
			}
	   break;
	    case 1:
		   {
			   if((bStopHere) || (bLidarStop)||(tPedStop>TimeTick))
			   { last_motor=-(fabs((float)RunProps.Brake));
			     SetServoPos(MOTOR_SERVO,last_motor);
			   }
			   else
			   SetServoRaw(MOTOR_SERVO,rc_ch[2]); //Throttle
			   Steer();
           }
		break;
	   case 2:
		  {
			  if((bStopHere) || (bLidarStop)|| (tPedStop>TimeTick))
			  { last_motor=-(fabs((float)RunProps.Brake));
			  }
			  else
			  {
				  last_motor=ManageSpeed(SegSpeed);
			  }
			  SetServoPos(MOTOR_SERVO,last_motor);
			  Steer();
		  }
	   break;
	   }

	    if(bRCFrameError && RunProps.StopOnRcLoss)
	     {
	   	   SetServoPos(MOTOR_SERVO,0); 
	   	   last_motor=0;
	    }
   }
 
  }
/***********************************End of monster while **********************/


}





