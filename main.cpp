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


const char * AppName="Avc2017";


#define STEER_SERVO (0)
#define MOTOR_SERVO (1)



volatile int nActiveMode;// 340 1025 1780 Ch 5
volatile int nFileMode;	 //340 1025 1780 Ch 6


extern int32_t GZero[3];
extern int32_t ZeroCount;


volatile uint32_t OdoCount;
volatile uint32_t DtOdoCount;
volatile uint32_t LastOdoTime;

float calc_speed;

const float SPEED_CONST = (125000000*4.58*3600)/(5280*12); //(Clock * In_per_odo * sec_perhour)/(ft_per_mile*in_per_feet)




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
			LIDAR_VALUE=(v-LIDAR_LAST_START);
			if (LIDAR_VALUE<LIDAR_MIN)
			{
			 LIDAR_MIN=LIDAR_VALUE;
			}
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

START_INTRO_OBJ(RCDriveObj,"RCDrive")
uint32_element steer{"steer"};
uint32_element motor{"motor"};
END_INTRO_OBJ;

RCDriveObj SFObj;



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


float TargetHeading;

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
END_INTRO_OBJ;



START_INTRO_OBJ(SteerLoopObj,"SteerLoop")
float_element t{"targ"};
float_element e{"err"};
float_element i{"ierr"};
uint8_element m{"move"};
END_INTRO_OBJ;


START_INTRO_OBJ(CurPosObj,"POS")
float_element x{"x"};
float_element y{"y"};
uint32_element pc{"pointcnt"};
uint32_element dt{"dt"};
uint32_element sn{"sn"};
uint32_element tc{"tc"};
int32_element b{"b"};
END_INTRO_OBJ;



START_INTRO_OBJ(NextPointRec,"NextPt")
int32_element rp{"rp"};
int32_element cp{"cp"};
END_INTRO_OBJ;




START_INTRO_OBJ(NavCalcRec,"NavCalc")
float_element xtk{"xtk"};
float_element hd{"hd"};
float_element th{"th"};
float_element xh{"xh"};
float_element x{"x"};
float_element y{"y"};
END_INTRO_OBJ;


static NextPointRec NextPointObj;
static NavCalcRec NavCalcObj;
static ModeChangeObj mco;
static SteerLoopObj slo;
static BootSecObj   bso;
static CurPosObj    cpo;

static fPoint  CurPos;


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
 static float ierr;

 float head;
 if(RunProps.UseMag)
	head=IntegratedHeading;
	else
	head=RawHeading;


 float err=(TargetHeading-head);
 if(err>180) err-=360;
 if(err<-180) err+=360;

 float sv=(float)RunProps.SteerP*err+ (float)RunProps.SteerI*ierr+(float)RunProps.SteerD*(float)RotVel[2]/1000.0;

 if(sv>max_steer) sv=max_steer;
 if(sv<min_steer) sv=min_steer;

 slo.t=TargetHeading;
 slo.e=err;

 SetServoPos(0,-sv+(float)RunProps.SteerZero);

if(bIsMoving())
{
 slo.m=1;
 ierr+=err;
}
else
{
 slo.m=0;
}
slo.i=ierr;
slo.Log();
}





void ProcessNewImuData()
{


}

static int CurPathIndex;
static float SegSpeed;
static bool bStopHere;







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

void DoNewNavCalcs();




void NextPoint()
{

	if(RawPaths[CurPathIndex].end_speed!=0)	CurPathIndex++;
    else
	{bStopHere=true;
	 return;
	}

    NextPointObj.rp=CurPathIndex;
    NextPointObj.cp=RawPaths[CurPathIndex].ref_path_num;
	NextPointObj.Log();
	DoNewNavCalcs();

}

void DoNewNavCalcs()
{
//	zot      

	float th;
	float xtk;
	float ts;

	float head;
	if(RunProps.UseMag)
	   head=IntegratedHeading;
	   else
	   head=RawHeading;
	
	if(RawPaths[CurPathIndex].NavCalc(CurPos,head,th,xtk,ts)) 
	{
     NextPoint();
	 return;
	}
    SegSpeed=ts;

 
 float xh=0;
  xh=-xtk*(float)RunProps.CrossAngleScale;
  if(xh<-(float)RunProps.CrossMaxCorrect) xh=-(float)RunProps.CrossMaxCorrect;
  if(xh> (float)RunProps.CrossMaxCorrect) xh=(float)RunProps.CrossMaxCorrect;
 

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

  NavCalcObj.xtk=xtk;
  NavCalcObj.hd=TargetHeading;
  NavCalcObj.xh=xh;
  NavCalcObj.th=th;
  NavCalcObj.x=CurPos.x;
  NavCalcObj.y=CurPos.y;
  NavCalcObj.Log();
}







void ModeChange(int new_mode, int prev_mode)
{
 mco.m=(int16_t)new_mode;
 mco.Log();
 if((prev_mode==0) && (new_mode!=0))
 {
	 CurPos=RawPaths[0].start_point;
     CurPathIndex=0;

  if(!RunProps.UseMag)
  {
   SetRawHeading(RawPaths[0].start_head);
  }
  NextPoint();

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
	uint32_t Lsec=Secs;

	while (1)
   {//Main processing loop


	if(Lsec!=Secs)
	{
		Lsec=Secs;
		bso.s=Secs;
		bso.Log();

		LCD_X_Y(LCD_SER,0,0);
		if(bIMU_Id)
		{if(Secs &1)
			fdprintf(LCD_SER,"X%ld,%3.0f,%3.0f ",OdoCount,IntegratedHeading,MagHeading);
		else
			fdprintf(LCD_SER,"+%ld,%3.0f,%3.0f ",OdoCount,IntegratedHeading,MagHeading);
		}
		else
			fdprintf(LCD_SER,"NO IMU NO IMU");

	if((bCompassCalDirty) && (Secs>20) && ((Secs %30)==0))
	 {
	   SaveConfigToStorage();
	   bCompassCalDirty=false;
	   iprintf("Compass cal saved\r\n");
	 }


	  LCD_X_Y(LCD_SER,0,1);
	  fdprintf(LCD_SER,"L:%ld:R:%2ld %3ld ",(LidarScanCount-LastLidar)/1000,RCFrameCnt-LastRCFrame,GetLogPercent());

	  if(RCFrameCnt-LastRCFrame<10) 
		  bRCFrameError=true;
	  else
		  bRCFrameError=false;
//
//	  printf("L:%ld:R:%2ld %3ld ",(LidarScanCount-LastLidar)/1000,RCFrameCnt-LastRCFrame,GetLogPercent());
//      printf("OdoCount %ld LIDAR_VALUE=%ld\r\n",OdoCount,LIDAR_VALUE);
//      printf("AD [%04X,%04X,%04X,%04X]\r\n",GetADResult(0),GetADResult(1),GetADResult(2),GetADResult(3));
      StartAD();

	  Lsec=Secs;
	  LastLidar=LidarScanCount;
	  LastRCFrame=RCFrameCnt;
	}
	OSTimeDly(1);
  }
}







bool newRC()
{
static uint32_t LastRCFrameCnt;
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
static uint32_t myLastODOCnt;
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
static uint32_t lastServoFrameCnt;
uint32_t SF_In=ServoFrameCnt;
if(SF_In!=lastServoFrameCnt)
{
lastServoFrameCnt=SF_In;
return true;
}
return false;
}








uint32_t ProcessLidarLines(int32_t &b,uint32_t &tc);


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

	iprintf("AVC  at  %s on %s\r\n",__TIME__,__DATE__);

    uint32_t ttn=TimeTick;
    SetUpTables();
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


   LCD_SER=SimpleOpenSerial(5,9600);
   LCD_CLS(LCD_SER);


   SetPinIrq(49,-1,OdoIrq);




   iprintf("Application started\n");


   LogFileVersions();
   LogAppRecords();

   InitTimer3();
   Mpu9250setup(MAIN_PRIO-2);
   InitLidar(2,MAIN_PRIO-1);
   InitLogFtp(MAIN_PRIO+1);

   ServoDriveInit();
   InitDSM2Rx(6);
   RCCallBack=ProcessNewRCPos;

    InitSingleEndAD();

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




   while(Secs<=(Lsec+RunProps.CalTime ))
   {
      OSTimeDly(TICKS_PER_SECOND);
      LCD_X_Y(LCD_SER,0,0);
        fdprintf(LCD_SER,"Cal: %ld of %ld ",(Lsec-Secs),(int)RunProps.CalTime );
      printf("Cal: %ld of %d \r\n",(Secs-Lsec),(int)RunProps.CalTime );
   }


   LCD_CLS(LCD_SER);
   iprintf("Samples=%ld DT=%ld\r\n",IMUSample-LastImu, Secs-(Lsec+1));

   iprintf("GZ[2]=%ld n=%ld off=%ld\r\n",GZero[2], ZeroCount,GZero[2]/ZeroCount);
   ImuMode=eRunning;
   OSTimeDly(10);
   SetInitalCompass();

   while(Lsec==Secs) asm("nop");


   OSSimpleTaskCreatewName(LCD_DisplayTask,MAIN_PRIO+2,"LCD State");

   pNotifyNextFrameSem=&MainTaskSem;


  while(1)
  {
	  MainTaskSem.Pend(2);

	if(newRC())
   {
     static int LastActiveMode;
	
	if(rc_ch[5]<700) nActiveMode=0;
     else if(rc_ch[5]<1200) nActiveMode=1;
		               else    nActiveMode=2;


	if(LastActiveMode!=nActiveMode)
		{
		 ModeChange(nActiveMode,LastActiveMode);
		
		}
		LastActiveMode =nActiveMode;
    LogRC();
   }
   uint32_t delta_odo=newODO();
   if(delta_odo)
   {
	   if(DtOdoCount==0)
		calc_speed=0;
		   else
	    calc_speed=SPEED_CONST/(float)DtOdoCount;


	 float head;
	 static float last_head;

     if(RunProps.UseMag)
		 head=IntegratedHeading;
		 else
		 head=RawHeading;

		 float dist=(float)RunProps.ODODist;
		 if(delta_odo!=1)
		 {
		  dist=(float)RunProps.ODODist*(float)delta_odo;
		 }


         //Can't average headings...s
		 float usehead=avg_angle(head,last_head);
		 float dx=dist*LookUpSinDeg(usehead);
		 float dy=dist*LookUpCosDeg(usehead);
		 last_head=head;
		 CurPos.x+=dx;
     	 CurPos.y+=dy;
		 cpo.x=CurPos.x;
		 cpo.y=CurPos.y;
		 uint32_t t1=sim2.timer[3].tcn;
		 uint32_t tc=0;
		 int32_t b=0;
		 bLidarRight=false;
         cpo.pc=LidarPointCount;
		 //cpo.sn=ProcessLidarLines(b,tc);
		 cpo.tc=tc;
		 cpo.b=b;
		 uint32_t t2=sim2.timer[3].tcn;
		 LidarPointCount=0;
		 cpo.dt=(t2-t1);
		 bLidarRight=true;

		 if(nActiveMode!=0)
			 { 
			  DoNewNavCalcs();
			 };

        cpo.Log();
   }

   if(newServoFrame())
   {
	   if(nActiveMode==0)
		   {
		    SetServoRaw(STEER_SERVO,rc_ch[1]); //Steer
            SetServoRaw(MOTOR_SERVO,rc_ch[2]); //Throttle
	       }
	   else
		 {
		 Steer();
		 if(bStopHere) SetServoPos(MOTOR_SERVO,-(fabs((float)RunProps.Brake)));
		  else
		 if(nActiveMode==1)
		   SetServoRaw(MOTOR_SERVO,rc_ch[2]); //Throttle
		 else
		   if(SegSpeed>-1)
		   {if(SegSpeed<1)
            SetServoPos(MOTOR_SERVO,SegSpeed);
		    else
			 SetServoPos(MOTOR_SERVO,ManageSpeed(SegSpeed));
		   }
			   else
		   SetServoPos(MOTOR_SERVO,ManageSpeed((float)RunProps.DefSpeed));
		}
		   if(bRCFrameError && RunProps.StopOnRcLoss)
		   {
			   SetServoPos(MOTOR_SERVO,0); 
		   }
	   SFObj.steer=GetServoCount(STEER_SERVO);
	   SFObj.motor=GetServoCount(MOTOR_SERVO);
	   SFObj.Log();

   }
 }



}

uint32_t LineSlopeCount[256];
long long  BSum[256];


uint32_t ProcessLidarLines(int32_t & b,uint32_t &tc)
{
uint32_t n=LidarPointCount;
if(n>256) n=256;
uint32_t total_counted=0;

if(n<10) //Need at least 10 points
{
 tc=0;
 return 0;
}

bzero(LineSlopeCount,256*sizeof(uint32_t));
bzero(BSum,256*sizeof(BSum[0]));
for (uint32_t i=0; i<(n-1); i++)
	for( uint32_t j=i+1; j<n; j++)
	{
	 int dx=LidarPointSet[i].x-LidarPointSet[j].x;
	 int dy=LidarPointSet[i].y-LidarPointSet[j].y;
	 if(dy!=0)
	 { //dx is left right
	   //dy fore aft
       //dx/dy = slope only care about -1 to +1
	   //Or +/- 45 degrees    scaled to 0 to 256
	   //  x=my+b
	   //X1=my1+b
	   //m=dx/dy
	   //X1=dxy1/dy +b
	   //B=X1-(DX*Y1/Dy)
	
	  int slope=(dx*128)/dy;  //+1==128 -1==-128
	
	  if((slope>=-128) && (slope<=127))
	   {
         LineSlopeCount[slope+128]++;
		 //B=LidarPointSet[i].x-((dx*LidarPointSet[i].y)/dy);
		 BSum[slope+128]+=LidarPointSet[i].x-((dx*LidarPointSet[i].y)/dy);
		 total_counted++;
	   }
	 }
	}
if(total_counted>5)
{
 tc=total_counted;
//Now find Median value

 n=0;
 total_counted/=2;

for(int i=0; i<256; i++)
{
 n+=LineSlopeCount[i];

 if(n>=(total_counted))
  {
	if(i>1)
	{
	 //Find the greatest i-1,i,i+1
	 if(LineSlopeCount[i-1]>LineSlopeCount[i+1])
	 {
		 if(LineSlopeCount[i-1]>LineSlopeCount[i])
		 i--;
	 }
	 else
	 {
		 if(LineSlopeCount[i+1]>LineSlopeCount[i])
			 i++;
	 }
	}
	if(LineSlopeCount[i]!=0)
	{
     b=BSum[i]/LineSlopeCount[i];
    return i;
	}
	return 256;
  }
}
}
return 256;
}



