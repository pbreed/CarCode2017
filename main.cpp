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

LOGFILEINFO;

class RunProperties:public config_obj
{
public:
RunProperties():config_obj(appdata,"RunProp") {};
    config_int    CalTime{20,"CalTime"};
    config_bool   UseMag{true,"UseMag"};
	config_double ODODist{4.58,"OdoDist"};
	config_double CrossAngleScale{1,"XandScale"};
	config_double CrossMaxCorrect{45,"MaxCorrect"};
	ConfigEndMarker;
};



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
int32_element cp{"cp"}; 
int32_element np{"np"};
float_element nx{"nx"};
float_element ny{"ny"};
float_element ty{"ty"};
float_element tx{"tx"};
float_element bh{"bh"};
float_element sa{"sa"};
uint32_element stop{"stop"};
END_INTRO_OBJ;




START_INTRO_OBJ(NavCalcRec,"NavCalc")
float_element xtk{"xtk"}; 
float_element hd{"hd"}; 
float_element x{"x"}; 
float_element y{"y"}; 
float_element htd{"htd"}; 
float_element propt{"propt"}; 
END_INTRO_OBJ;


static NextPointRec NextPointObj;
static NavCalcRec NavCalcObj;
static ModeChangeObj mco;
static SteerLoopObj slo;
static BootSecObj   bso;
static CurPosObj    cpo;

static fPoint  CurPos;



const float steer_p =0.01;
const float steer_i =0.0; //001;
const float max_steer =0.6;
const float min_steer =-0.6;


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

 float sv=steer_p*err + steer_i*ierr;

 if(sv>max_steer) sv=max_steer;
 if(sv<min_steer) sv=min_steer;

 slo.t=TargetHeading;
 slo.e=err;

 SetServoPos(0,-sv);

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
static fPoint FromPt;
static fPoint ToPt;
static float base_head;
static float next_head;
static float split_angle;
static bool bStopHere;
static float line_dx;
static float line_dy;
static float cross_a;
static float cross_b;
static float inverse_caroot;




//- is left of cource, + is right of course
float CalcCrossTrack()
{
//We know line dx and line dy

if(line_dx==0) 
{//Veritcal line.

 if(line_dy>0)
	 {//headed north  
	 return (CurPos.x-ToPt.x);
     }
	else 
    {//headed south
	 return (ToPt.x-CurPos.x); 
	}
 }
//Now the hard cases we are not DXor Dy=0
float  dist = (cross_a * CurPos.x + cross_b - CurPos.y)*inverse_caroot;
if(line_dx<0) return -dist;
return dist;
}



float turn_angle(float cur_a, float next_a)
{
float a=(next_a-cur_a);
if(a<-180) a+=360;
if(a>180) a-=360;
return a;
}

float add_angle(float cur_a, float change)
{
 float a=cur_a+change;
 if(a<-180) a+=360;
 if(a>180) a-=360;
 return a;
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

void DoNewNavCalcs();




void NextPoint()
{
	FromPt=PathArray[CurPathIndex].pt;
	int np=PathArray[CurPathIndex].next_seq;
	if((np<0) || (PathArray[np].m_bValid==false))
	{
	   bStopHere=true;

	   NextPointObj.cp=CurPathIndex;
	   NextPointObj.np=np;
	   NextPointObj.nx=ToPt.x;
	   NextPointObj.ny=ToPt.y;
	   NextPointObj.tx=FromPt.x;
	   NextPointObj.ty=FromPt.y;
	   NextPointObj.bh=base_head;
	   NextPointObj.sa=base_head;
	   NextPointObj.stop=true;
	   NextPointObj.Log();


	   return;
	}
	CurPathIndex=np;
	bStopHere=false;
	
	
	ToPt=PathArray[CurPathIndex].pt; 
    base_head=ToPt.HeadToHereDeg(FromPt);
	if((PathArray[CurPathIndex].next_seq<0) || (PathArray[PathArray[CurPathIndex].next_seq].m_bValid==false))
	{
	next_head=base_head;
	}
	else
	{
	 next_head=PathArray[PathArray[CurPathIndex].next_seq].HeadToHereDeg(ToPt);
	}

	split_angle=avg_angle(base_head,next_head);
	line_dx=ToPt.x-FromPt.x;
	line_dy=ToPt.y-FromPt.y;
	
	if(line_dx!=0)
	{
 	cross_a = (ToPt.y - FromPt.y) / (ToPt.x - FromPt.x); 
	cross_b = ToPt.y - cross_a * ToPt.x;                 
	inverse_caroot= inv_sqrt(cross_a * cross_a + 1);      
	}
	NextPointObj.cp=CurPathIndex;
	NextPointObj.np=PathArray[CurPathIndex].next_seq;
	NextPointObj.nx=ToPt.x;
	NextPointObj.ny=ToPt.y;
	NextPointObj.tx=FromPt.x;
	NextPointObj.ty=FromPt.y;
	NextPointObj.bh=base_head;
	NextPointObj.sa=split_angle;
	NextPointObj.stop=true;
	NextPointObj.Log();
	DoNewNavCalcs();

}

void DoNewNavCalcs()
{


    float head_to_dest=ToPt.HeadToHereDeg(CurPos);
    float prop_turn=turn_angle(head_to_dest,split_angle);

	//head_to_Dest is direction directly to the current wp
	//split angle is the bisector fo the two current headings 
	//So prop turn is how many degree change from head to and split if the turn is mroe than 90
	//we shoudl go to next wp
    if((prop_turn>90) || (prop_turn<-90)) 
		{
		NextPoint();
		}


  float xtk=CalcCrossTrack();
  
  float xh=-xtk*(float)RunProps.CrossAngleScale;
  if(xh<-(float)RunProps.CrossMaxCorrect) xh=-(float)RunProps.CrossMaxCorrect; 
  if(xh> (float)RunProps.CrossMaxCorrect) xh=(float)RunProps.CrossMaxCorrect; 
  
  xh+=base_head;
  if(xh>180) xh-=360;
  if(xh<-180) xh+=360;
  TargetHeading=xh;

  NavCalcObj.xtk=xtk;
  NavCalcObj.hd=TargetHeading;
  NavCalcObj.x=CurPos.x;
  NavCalcObj.y=CurPos.y;
  NavCalcObj.htd=head_to_dest;
  NavCalcObj.propt=prop_turn;
  NavCalcObj.Log();
}




 


void ModeChange(int new_mode, int prev_mode)
{
 mco.m=(int16_t)new_mode;
 mco.Log();
 if(new_mode==1)
 {
	 CurPos=PathArray[0].pt;
     CurPathIndex=0;
  
  if(!RunProps.UseMag)
  {
   float head=PathArray[1].HeadToHereDeg(PathArray[0]);
   SetRawHeading(head);
  }
  NextPoint();

 }


}



extern volatile uint32_t LidarRxc;
extern void RegisterPost();

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
if(RCFrameCnt!=LastRCFrameCnt)
{
  LastRCFrameCnt=RCFrameCnt;
  return true;
}
return false;
}

bool newODO()
{
static uint32_t myLastODOCnt;
if(OdoCount!=myLastODOCnt)
{
  myLastODOCnt=OdoCount;
  return true;
}
return false;
}


bool newServoFrame()
{
static uint32_t lastServoFrameCnt;
if(ServoFrameCnt!=lastServoFrameCnt)
{
lastServoFrameCnt=ServoFrameCnt;
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

   if(newODO())
   {
	 float head;
	 static float last_head;
     if(RunProps.UseMag)
		 head=IntegratedHeading;
		 else
		 head=RawHeading;

		 float dist=(float)RunProps.ODODist;

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
		 int32_t b;
		 bLidarRight=false;
         cpo.pc=LidarPointCount;
		 cpo.sn=ProcessLidarLines(b,tc);
		 cpo.tc=tc;
		 cpo.b=b;
		 uint32_t t2=sim2.timer[3].tcn;
		 LidarPointCount=0;
		 cpo.dt=(t2-t1);
		 bLidarRight=true;

		 if(nActiveMode==1) DoNewNavCalcs();

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
		   if(nActiveMode==1) 
		 {
		 Steer();
		 if(bStopHere) SetServoPos(MOTOR_SERVO,0);
			 else
			 SetServoRaw(MOTOR_SERVO,rc_ch[2]); //Throttle
		}


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



