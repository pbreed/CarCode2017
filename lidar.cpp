#include <predef.h>
#include <init.h>
#include <constants.h>
#include <buffers.h>
#include <iosys.h>
#include <serial.h>
#include <serinternal.h>
#include <nbrtos.h>
#include <stdlib.h>
#include <string.h>
#include "introspec.h"
#include "Lidar.h"
#include "SinLookup.h"

LOGFILEINFO;





volatile IntPoint LidarPointSet[MAX_LIDAR_POINTS];
volatile uint32_t LidarPointCount;
volatile LidarWallMode LidarMode;
volatile LidarWallMode DoneMode;


OS_SEM LidarCalcSem;




unsigned int state;
unsigned int tv;
unsigned char tb[5];
int nb=0;

static int Lidar_port;

struct OneResult
{
uint8_t qf;
uint8_t a_lsb;
uint8_t a_msb;
uint8_t d_lsb;
uint8_t d_msb;
};

uint8_t a_lsb;


volatile uint32_t Result[360];
volatile uint32_t ssResult[360];
volatile uint32_t UResult[360];
volatile uint32_t ssUResult[360];
volatile uint32_t LidarScanCount;



START_INTRO_OBJ(LidarReadingObj,"LidarHit")
uint16_element a{"ax128"};
uint16_element d{"dx40"};
uint8_element qf{"qf"};
END_INTRO_OBJ;


START_INTRO_OBJ(LidarStateObj,"LidarState")
int32_element  a{"action"};
END_INTRO_OBJ;

static LidarStateObj LidarState;


static LidarReadingObj LidarHit;


inline void LIDAR_ProcessChar(uint8_t c)
{
	{
		   if(state<2)
		   {

		   tv=(tv<<8)+c;
		   if(tv==0xA55A0500)
		   {
			  state=1;
		   }

		   if((tv==0x00004081) && (state==1))
		   {
			  state=2;
			  nb=0;
		   }
		   }
		   else
		   {
			 tb[nb++]=c;
			 if(nb==5)
			 {
				OneResult * po;
				po=(OneResult *) tb;

				if(tb[0]& 0x01)
				{
					for(int i=0; i<360; i++)
					{
					UResult[i]=Result[i];
					ssUResult[i]=ssResult[i];
					Result[i]=0;
					ssResult[i]=0;
					LidarScanCount++;					
				   }
				}
				uint32_t d;
				d=po->d_msb;
				d=(d<<8)|po->d_lsb;

				uint32_t a;

				a=po->a_msb;
				 a=(a<<8)|po->a_lsb;


			   uint32_t ss=po->qf&0xFC;
			   int index=((a/128)%360);
			   if(ss>0)
			   {
				  bool bSample=false;

				   switch(LidarMode)
				   {
					case eOff: break;
					case eCalculating: break;
					case eRight:
					case eDoneRight:
					   bSample=((index >45) && (index<135));
					   break;
					case eLeft:
					case eDoneLeft:
						bSample=((index >135) && (index<315));
					  break;
				   }

				  if(bSample)
				   {
					   int x=(int)(AX128ToSinCosInIn[a][0]*(float)d);
					   int y=(int)(AX128ToSinCosInIn[a][1]*(float)d);
					   uint32_t n=(LidarPointCount % MAX_LIDAR_POINTS);
					   LidarPointCount++;
					   LidarPointSet[n].x=x;
					   LidarPointSet[n].y=y;
					   if((LidarPointCount>20) && ((LidarMode==eRight) || (LidarMode==eLeft)))
					   {
						   LidarMode=eCalculating;
						   LidarCalcSem.Post();
						   		 LidarState.a=100;
								 LidarState.Log();
 
					   }

				   }


				  if (ssResult[index]<ss)
				  {
					  ssResult[index]=ss;
					  Result[index]=(d/40);
					  
					  LidarHit.a=a;
					  LidarHit.d=d;
					  LidarHit.qf=po->qf;
					  LidarHit.Log();

				  }
			   }
			  nb=0;
			 }

		   }



		}

}

void LidarTask(void *p)
{
int fds=SimpleOpenSerial(Lidar_port,115200);
    OSTimeDly(2);

    char lidar_buffer[256];
    lidar_buffer[0]=0xA5;
	lidar_buffer[1]=0x20;
	lidar_buffer[2]=0;
    write(fds,(char *)lidar_buffer,2);

	while(1)
	{
		int rv=read(fds,lidar_buffer,256);
		if(rv>0)
		{

	      for(int i=0; i<rv; i++)
		  {
			LIDAR_ProcessChar(lidar_buffer[i]);
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


volatile int32_t iResult;
volatile int32_t bResult;
volatile uint32_t tcResult;
volatile uint32_t pcResult;

void LIDAR_CALC_Task(void * pd)
{
while(1)
 {
  LidarCalcSem.Pend(0);
  		 LidarState.a=6;
		 LidarState.Log();
  pcResult=LidarPointCount;
  int32_t b;
  uint32_t tc;
  LidarState.a=7;
  LidarState.Log();

  iResult=ProcessLidarLines(b,tc); 
  bResult=b;
  tcResult=tc;
  LidarPointCount=0;
  LidarMode=DoneMode;
  LidarState.a=8;
  LidarState.Log();

 }
}


bool LidarBusy()
{
	switch(LidarMode)
		{
		 case eCalculating: 
		 case eRight:
	     case eLeft:
		 LidarState.a=3;
		 LidarState.Log();
			  return true;
	     case eOff:
		 case eDoneRight:
		 case eDoneLeft:
			 LidarState.a=4;
			 LidarState.Log();
			 return false;
		}
 LidarState.a=5;
 LidarState.Log();

 return false;
}


int GetLidarResult(int32_t & b,uint32_t &tc, uint32_t &pc)
{
if( (LidarMode==eDoneRight) || (LidarMode==eDoneLeft)) 
{
   b=bResult;
  tc=tcResult;
  pc=pcResult;
  return iResult;
}

return 0;

}

void LidarSampleStart(LidarWallMode new_mode)
{
    switch(new_mode)
		{
		 case eDoneRight:
		 case eDoneLeft:
		 case eCalculating:  //Should not get this one
		 case eOff: 
          LidarMode=eOff;
		  LidarPointCount=0;
		 break;

		 case eRight:
			 DoneMode=eDoneRight;
			 LidarMode=new_mode;
			 LidarState.a=1;
			 LidarState.Log();
		     break;
		 case eLeft:
			 DoneMode=eDoneLeft;
			 LidarMode=new_mode;
			 LidarState.a=2;
			 LidarState.Log();

			 break;
		}
}



void InitLidar(int port, int SampleTaskPrio,int CalcTaskPrio)
{
	Lidar_port=port;
	LidarMode=eOff;
	OSSimpleTaskCreatewName(LidarTask,SampleTaskPrio,"LIDARTask");
	OSSimpleTaskCreatewName(LIDAR_CALC_Task,CalcTaskPrio,"LidarCalc");
}
