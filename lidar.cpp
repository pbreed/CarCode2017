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

const unsigned int WarnLimit[360]={121,121,121,121,121,121,121,121,121,121,121,121,121,121,121,121,121,121,121,121,121,121,121,121,121,121,
	121,121,121,121,121,118,115,111,109,106,103,101,99,96,94,92,91,89,87,86,84,83,82,80,79,78,77,76,75,74,73,72,71,71,70,69,69,
	68,67,67,66,66,65,65,64,64,64,63,63,63,62,62,62,62,61,61,61,61,61,61,61,61,60,60,60,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,60,60,60,61,61,61,61,61,61,61,61,62,62,62,62,
	63,63,63,64,64,64,65,65,66,66,67,67,68,69,69,70,71,71,72,73,74,75,76,77,78,79,80,82,83,84,86,87,89,91,92,94,96,99,101,103
	,106,109,111,115,118,121,121,121,121,121,121,121,121,121,121,121,121,121,121,121,121,121,121,121,121,121,121,121,121,121,
	121,121,121,121,121};

	const uint8_t warn_mask[360]={231,247,247,247,247,247,247,247,247,247,247,247,247,247,247,247,247,247,247,247,247,247,247,
251,251,251,251,251,251,251,251,251,251,251,251,251,251,251,251,251,251,251,251,251,251,251,253,253,253,253,253,253,253,253,
253,253,253,253,253,253,253,253,253,253,253,253,253,253,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,
254,254,254,254,254,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,191,191,191,191,191,191,191,191,191,
191,191,191,191,191,191,191,191,191,191,191,191,191,191,223,223,223,223,223,223,223,223,223,223,223,223,223,223,223,223,223,
223,223,223,223,239,239,239,239,239,239,239,239,239,239,239,239,239,239,239,239,239,239,239,239,239,239,239};








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

uint8_t calc_warn_byte;
volatile uint8_t warning_byte;
 



volatile bool OpenLeftSector[6];
volatile bool OpenRightSector[6];





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

					warning_byte=calc_warn_byte;
					calc_warn_byte=0xff;

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
 
					   }

				   }


				  if (ssResult[index]<ss)
				  {
					  ssResult[index]=ss;
					  Result[index]=(d/40);
					  if(Result[index]<WarnLimit[index])
					  {
						calc_warn_byte&=warn_mask[index];
                        
					  }
					  
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
		 if(LineSlopeCount[i-1]>LineSlopeCount[i]) i--;
	 }
	 else
	 {
		 if(LineSlopeCount[i+1]>LineSlopeCount[i]) i++;
	 }
	}
	if(LineSlopeCount[i]!=0)
	{
     b=BSum[i]/LineSlopeCount[i];
    return i; //Slope number 128 is ZERO 
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
  		 //LidarState.a=6;
		 //LidarState.Log();
  pcResult=LidarPointCount;
  int32_t b;
  uint32_t tc;
  iResult=ProcessLidarLines(b,tc); 
  bResult=b;
  tcResult=tc;
  LidarPointCount=0;
  LidarMode=DoneMode;
 // LidarState.a=8;
 // LidarState.Log();

 }
}


bool LidarBusy()
{
	switch(LidarMode)
		{
		 case eCalculating: 
		 case eRight:
	     case eLeft:
			  return true;
	     case eOff:
		 case eDoneRight:
		 case eDoneLeft:
			 return false;
		}

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

pc=0;
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
		     break;
		 case eLeft:
			 DoneMode=eDoneLeft;
			 LidarMode=new_mode;

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



const char * GetLidarStateName(LidarWallMode lm)
{
   switch(lm)
   {
   case eOff:        return "eOff";        
   case eRight:      return "eRight";      
   case eLeft:       return "eLeft";       
   case eCalculating:return "eCalculating";
   case eDoneLeft:   return "eDoneLeft";   
   case eDoneRight:  return "eDoneRight";  
   default: return "Corrupt";
   }
  return "Corrupt";
}


const float SlopeTable[256]={
-45,
-44.77531182,
-44.54886145,
-44.32063524,
-44.09061955,
-43.85880074,
-43.62516522,
-43.3896994	,
-43.15238973,
-42.91322272,
-42.67218491,
-42.42926289,
-42.18444332,
-41.93771291,
-41.68905849,
-41.43846691,
-41.18592517,
-40.93142032,
-40.67493957,
-40.41647019,
-40.15599962,
-39.89351543,
-39.6290053	,
-39.36245712,
-39.09385889,
-38.82319882,
-38.5504653	,
-38.27564691,
-37.99873244,
-37.71971092,
-37.43857157,
-37.1553039	,
-36.86989765,
-36.58234282,
-36.29262973,
-36.00074895,
-35.7066914	,
-35.41044829,
-35.11201118,
-34.81137199,
-34.50852299,
-34.20345683,
-33.89616656,
-33.58664565,
-33.27488798,
-32.96088788,
-32.64464013,
-32.32613999,
-32.00538321,
-31.68236603,
-31.35708522,
-31.02953811,
-30.69972255,
-30.36763698,
-30.03328044,
-29.69665254,
-29.35775354,
-29.01658434,
-28.67314649,
-28.3274422	,
-27.97947439,
-27.62924667,
-27.27676338,
-26.92202961,
-26.56505118,
-26.2058347	,
-25.84438755,
-25.48071795,
-25.11483489,
-24.74674821,
-24.37646862,
-24.00400765,
-23.62937773,
-23.25259217,
-22.87366519,
-22.4926119	,
-22.10944834,
-21.7241915	,
-21.33685929,
-20.94747059,
-20.55604522,
-20.16260399,
-19.76716868,
-19.36976203,
-18.97040781,
-18.56913073,
-18.16595653,
-17.76091192,
-17.35402464,
-16.94532338,
-16.53483786,
-16.12259878,
-15.70863783,
-15.29298769,
-14.875682	,
-14.4567554	,
-14.03624347,
-13.61418274,
-13.19061071,
-12.76556578,
-12.33908728,
-11.91121543,
-11.48199135,
-11.05145703,
-10.61965528,
-10.18662976,
-9.752424942,
-9.317086069,
-8.880659151,
-8.443190929,
-8.004728857,
-7.565321068,
-7.125016349,
-6.683864108,
-6.241914347,
-5.799217629,
-5.355825043,
-4.911788173,
-4.467159061,
-4.021990177,
-3.576334375,
-3.130244862,
-2.683775159,
-2.236979063,
-1.789910608,
-1.342624027,
-0.89517371	,
-0.447614171,
0			,
0.447614171	,
0.89517371	,
1.342624027	,
1.789910608	,
2.236979063	,
2.683775159	,
3.130244862	,
3.576334375	,
4.021990177	,
4.467159061	,
4.911788173	,
5.355825043	,
5.799217629	,
6.241914347	,
6.683864108	,
7.125016349	,
7.565321068	,
8.004728857	,
8.443190929	,
8.880659151	,
9.317086069	,
9.752424942	,
10.18662976	,
10.61965528	,
11.05145703	,
11.48199135	,
11.91121543	,
12.33908728	,
12.76556578	,
13.19061071	,
13.61418274	,
14.03624347	,
14.4567554	,
14.875682	,
15.29298769	,
15.70863783	,
16.12259878	,
16.53483786	,
16.94532338	,
17.35402464	,
17.76091192	,
18.16595653	,
18.56913073	,
18.97040781	,
19.36976203	,
19.76716868	,
20.16260399	,
20.55604522	,
20.94747059	,
21.33685929	,
21.7241915	,
22.10944834	,
22.4926119	,
22.87366519	,
23.25259217	,
23.62937773	,
24.00400765	,
24.37646862	,
24.74674821	,
25.11483489	,
25.48071795	,
25.84438755	,
26.2058347	,
26.56505118	,
26.92202961	,
27.27676338	,
27.62924667	,
27.97947439	,
28.3274422	,
28.67314649	,
29.01658434	,
29.35775354	,
29.69665254	,
30.03328044	,
30.36763698	,
30.69972255	,
31.02953811	,
31.35708522	,
31.68236603	,
32.00538321	,
32.32613999	,
32.64464013	,
32.96088788	,
33.27488798	,
33.58664565	,
33.89616656	,
34.20345683	,
34.50852299	,
34.81137199	,
35.11201118	,
35.41044829	,
35.7066914	,
36.00074895	,
36.29262973	,
36.58234282	,
36.86989765	,
37.1553039	,
37.43857157	,
37.71971092	,
37.99873244	,
38.27564691	,
38.5504653	,
38.82319882	,
39.09385889	,
39.36245712	,
39.6290053	,
39.89351543	,
40.15599962	,
40.41647019	,
40.67493957	,
40.93142032	,
41.18592517	,
41.43846691	,
41.68905849	,
41.93771291	,
42.18444332	,
42.42926289	,
42.67218491	,
42.91322272	,
43.15238973	,
43.3896994	,
43.62516522	,
43.85880074	,
44.09061955	,
44.32063524	,
44.54886145	,
44.77531182	};

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

 
float SlopeNumToDeg(int SlopeNum)
{
 if((SlopeNum>255) ||(SlopeNum<0)) return 0.0;

 return SlopeTable[SlopeNum];
}

/*
const int LidarAngleLimit[46] 
={100,100,100,100,100,100,100,100,100,100,100,100,99,91,85,79,74,70,67,63,60,
   57,55,53,51,49,47,46,44,43,41,40,39,38,37,36,35,35,34,33,32,32,31,31,30,30};

int a_range[360];

bool NotClear(int i,int limit)
{
if(i<0) i+=360;
return (a_range[i]<limit);
}


bool IsClear(int h,int span)
{
for(int i=0; i<span; i++)
{
    if(NotClear(h+i,LidarAngleLimit[i])) return false;
    if(NotClear(h-i,LidarAngleLimit[i])) return false;
}

 return true;
}

//Head is realtive to Car straight is 0...positive to right
int GetLidarHeading(int DesiredHead)
{
   
   for(int i=0; i<360;  i++)
   {
	 if(ssUResult[i]>2) a_range[i]=UResult[i];
	 else a_range[i]=999;
   }


   if (IsClear(DesiredHead,44))
	   {
	    return DesiredHead;		
       }
 
   for(int i=1; i<73; i++)
       {
	    if(IsClear(DesiredHead-i,45)) return DesiredHead-i;
	    if(IsClear(DesiredHead+i,45)) return DesiredHead+i;
       }

   for(int i=1; i<73; i++)
	  {
	   if(IsClear(DesiredHead-i,16)) return DesiredHead-i;
	   if(IsClear(DesiredHead+i,16)) return DesiredHead+i;
	  }
  
  // iprintf("No clear\r\n");
	return -999; //We failed bull through
}

  */

bool is_clear[360];

bool NotClear(int i)
{
if(i<0) i+=360;
return !is_clear[i];
}


bool IsClear(int h)
{for(int i=(h-16); i<(h+16); i++)
{
    if(NotClear(i)) return false;
}

 return true;
}

//Head is realtive to Car straight is 0...positive to right
int GetLidarHeading(int DesiredHead)
{
   uint32_t limit=100; //About 40 in
   
   for(int i=0; i<360;  i++)
   {
	 //iprintf("%3d:%6d,%6d\r\n",i,ssUResult[i],UResult[i]);
	 if((ssUResult[i]>2) && (UResult[i]<limit))is_clear[i]=false;
	 else is_clear[i]=true;
	// if(is_clear[i]) iprintf(".");
	// else iprintf("X,");
	// if((i==90)||(i==180)||(i==270)) printf("\r\n");

   }


   if (IsClear(DesiredHead))
	   {
	//    iprintf("DH Clear\r\n");
	    return DesiredHead;		
       }
 
   for(int i=1; i<73; i++)
       {
		iprintf("%d",i);
	    if(IsClear(DesiredHead-i)) return DesiredHead-i;
	    if(IsClear(DesiredHead+i)) return DesiredHead+i;
       }
  
  // iprintf("No clear\r\n");
	return -999; //We failed bull through
}






bool DoPedStop()
{
 int ns=0;
 for(int i=0; i<20; i++)
	 if((ssUResult[i]>2) && (UResult[i]<150)) ns++;

  for(int i=339; i<360; i++)
		 if((ssUResult[i]>2) && (UResult[i]<150)) ns++;
LidarState.a=ns;
LidarState.Log();
if (ns>=8  ) return true;
return false;

}




