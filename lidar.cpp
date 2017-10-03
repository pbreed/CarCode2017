#include <predef.h>
#include <init.h>
#include <constants.h>
#include <buffers.h>
#include <iosys.h>
#include <serial.h>
#include <serinternal.h>
#include <nbrtos.h>
#include "introspec.h"
#include "Lidar.h"
#include "SinLookup.h"

LOGFILEINFO;





volatile IntPoint LidarPointSet[MAX_LIDAR_POINTS];
volatile uint32_t LidarPointCount;
volatile bool bLidarRight;
volatile bool bLidarLeft;







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
				   if(
					  ((bLidarRight) && (index >45) && (index<135))
					  ||
					  ((bLidarLeft) && (index >135) && (index<315))
					  )
				   {
					   int x=(int)(AX128ToSinCosInIn[a][0]*(float)d);
					   int y=(int)(AX128ToSinCosInIn[a][1]*(float)d);
					   uint32_t n=(LidarPointCount& 0xFF);
					   LidarPointCount++;
					   LidarPointSet[n].x=x;
					   LidarPointSet[n].y=y;
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


void InitLidar(int port, int TaskPrio)
{
	Lidar_port=port;
	OSSimpleTaskCreatewName(LidarTask,TaskPrio,"LIDARTask");


}





