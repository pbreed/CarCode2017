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

#include "servodrive.h"
#include "introspec.h"
#include "lidar.h"
#include "dsm2.h"
#include "mpu9250.h"


LOGFILEINFO;

class RunProperties:public config_obj
{
public:
RunProperties():config_obj(appdata,"RunProp") {};
    config_int CalTime{20,"CalTime"};
    config_bool  UseMag{true,"UseMag"};
	ConfigEndMarker;
};

RunProperties RunProps;



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


OS_SEM RCFrameSem;


void ProcessNewRCPos(uint32_t ch, uint32_t v)
{
static uint32_t LastCount;
	   switch(ch)
	   {
	   case 0: break; //Throttle
	
	   case 1:
		   if(nActiveMode==0) SetServoRaw(0,v);
		   break;
	
	   case 2:
		   //if(nActiveMode==0) 
		   SetServoRaw(1,v);
           break;

	   case 3: //Rudder
		    break;

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
	RCFrameSem.Post();
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

static ModeChangeObj mco;
static SteerLoopObj slo;
static BootSecObj   bso;


const float steer_p =0.01;
const float steer_i =0.0; //001;
const float max_steer =0.6;
const float min_steer =-0.6;


void Steer()
{
 static float ierr;
  
 float err=(TargetHeading-IntegratedHeading); 
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

static int SeqMode;
static uint32_t LastSOdo;

//Heading 30deg for 100 count
//Heading -60   for 40 count
//Heading -150  for 100 count
//Heading 120   for 40 count
void DoSequence()
{
if(nActiveMode==2)
{
switch (SeqMode) 
{
case 0:
	 TargetHeading=145;
	 if((OdoCount-LastSOdo)>=130) 
		{SeqMode++;
		 LastSOdo=OdoCount;
        }
	break;
case 1:
	 TargetHeading=145-90;
	 if((OdoCount-LastSOdo)>=100) 
		{SeqMode++;
		 LastSOdo=OdoCount;
        }
	break;
case 2:
	 TargetHeading=145-180;
	 if((OdoCount-LastSOdo)>=130) 
		{SeqMode++;
		 LastSOdo=OdoCount;
        }
	break;

case 3:
	 TargetHeading=145-270;
	 if((OdoCount-LastSOdo)>=100) 
		{SeqMode=0;
		 LastSOdo=OdoCount;
        }
	break;
}
}

Steer();
}


void ProcessNewTick()
{
 if(nActiveMode!=0)DoSequence();




}

void ModeChange(int new_mode, int prev_mode)
{
 if(prev_mode==0) TargetHeading=IntegratedHeading;

 if ((prev_mode==1) && (new_mode==2))
 {
  LastSOdo=OdoCount;
  SeqMode=0;
 }
 mco.m=(int16_t)new_mode;
 mco.Log();
}



extern volatile uint32_t LidarRxc;
void UserMain(void * pd)
{
	initWithWeb();
	iprintf("AVC  at  %s on %s\r\n",__TIME__,__DATE__);


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



   int LCD_SER=SimpleOpenSerial(5,9600);
   LCD_CLS(LCD_SER);




   SetPinIrq(49,-1,OdoIrq);


   iprintf("Application started\n");


   LogFileVersions();
   LogAppRecords();

   InitTimer3();   
   Mpu9250setup(MAIN_PRIO-2);
   InitLidar(2,MAIN_PRIO-1);
   InitLogFtp(MAIN_PRIO+1);
   //InitFileTask(MAIN_PRIO+2);
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
      printf("Cal: %ld of %ld \r\n",(Secs-Lsec),(int)RunProps.CalTime );
   }

   while(Secs==(Lsec+11)) asm(" nop");

   LCD_CLS(LCD_SER); 
   iprintf("Samples=%ld DT=%ld\r\n",IMUSample-LastImu, Secs-(Lsec+1));

   iprintf("GZ[2]=%ld n=%ld off=%ld\r\n",GZero[2], ZeroCount,GZero[2]/ZeroCount);
   ImuMode=eRunning;
   OSTimeDly(10);
   SetInitalCompass();



   uint32_t LastLidar=LidarScanCount;
   uint32_t LastRCFrame=RCFrameCnt;


   Lsec=Secs;
   while(Lsec==Secs) asm("nop");
   LastImu=IMUSample;
   Lsec=Secs;




   while (1)
  {



   if(charavail())
   {switch(getchar())
      {
      }
   }
   

   if(Lsec!=Secs)
   {
   Lsec=Secs;
   bso.s=Secs;
   bso.Log();

   LCD_X_Y(LCD_SER,0,0);
   /*if(nFileMode==2)
   {
	   writestring(LCD_SER,FileStateMsg);
   }
   else
   */
   if(bIMU_Id)
	{if(Secs &1)
	   fdprintf(LCD_SER,"X%ld,%3.0f,%3.0f  ",OdoCount,IntegratedHeading,MagHeading);
   else
	   fdprintf(LCD_SER,"+%ld,%3.0f  ",OdoCount,IntegratedHeading);
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
   fdprintf(LCD_SER,"L:%ld:%3.0fR:%2ld %3ld ",(LidarScanCount-LastLidar)/1000,IntegratedHeading,RCFrameCnt-LastRCFrame,GetLogPercent());

   Lsec=Secs;
   LastLidar=LidarScanCount;
   LastImu=IMUSample;       Lsec=Secs;
   LastRCFrame=RCFrameCnt;
  }
  if(RCFrameSem.Pend(1)==OS_NO_ERR)
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
 }
}
