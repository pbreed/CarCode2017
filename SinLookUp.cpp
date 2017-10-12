#include <basictypes.h>
#include <math.h>
#include <stdio.h>
#include "introspec.h"
LOGFILEINFO;


const float pi=3.14159265358979323846;

float DistSinQuadTable[65536];
float SinQuadTable[65536];
float AX128ToSinCosInIn[46080][2];


void SetUpTables(float dist)
{
iprintf("Starting to setup tables\n");
 
for(int i=0; i<65536; i++)
 {
    double d=i;
    d*=pi;
    d/=131072.0;
    SinQuadTable[i]=sin(d);
	DistSinQuadTable[i]=dist*sin(d);
 }

 iprintf("Workign on table 2\r\n");

  AX128ToSinCosInIn[0][0]=0; //sin 0 is 0
  AX128ToSinCosInIn[11520][0]=1/(40.0*2.54); //sin 90 is 1
  AX128ToSinCosInIn[23040][0]=0;//sin 180 is 0
  AX128ToSinCosInIn[(46080-11520)][0]=-1/(40.0*2.54); //sin 270  is -1

 for(int i=1; i<11520; i++)
 {
  double d=i;
  d*=pi;
  d/=23040.0;
  float s=sin(d)/(40.0*2.54);
  AX128ToSinCosInIn[i][0]=s;       //0 to 90
  AX128ToSinCosInIn[23040-i][0]=s; //90 to 180
  AX128ToSinCosInIn[46080-i][0]=-s;//270 to 360
  AX128ToSinCosInIn[23040+i][0]=-s;//180 to 270
 }

 for(int i=0; i<46080; i++)
 {
     AX128ToSinCosInIn[i][1]=AX128ToSinCosInIn[(i+11520)%46080][0];
 }
 iprintf("Table2 setup complete\n");

}



//Looking for sin fomr 0 to 2pi represented by uint32_ going from 0 to 0x40000==2pi
float CoreSinLookup(unsigned long index)
{
index &=(0X3FFFF); //tRUNCATE TO 0 TO 2PI, DOES THE RIGHT THING WITH RESPECT TO WRAP AROUND
	switch(index & 0x30000)
	{
	case 0x00000: return SinQuadTable[index];  //0 to 90 degrees...
	case 0x10000: return SinQuadTable[0x1FFFF-index];//90 to 180 degrees...
	case 0x20000: return -SinQuadTable[index & 0xFFFF];//180 to 270 degrees...
	case 0x30000: return -SinQuadTable[0x1FFFF-(index &0x1FFFF)];//270 to 360 degrees...
	}
return 0; //We should never get here...
}


float CoreDistSinLookup(unsigned long index)
{
index &=(0X3FFFF); //tRUNCATE TO 0 TO 2PI, DOES THE RIGHT THING WITH RESPECT TO WRAP AROUND
	switch(index & 0x30000)
	{
	case 0x00000: return DistSinQuadTable[index];  //0 to 90 degrees...
	case 0x10000: return DistSinQuadTable[0x1FFFF-index];//90 to 180 degrees...
	case 0x20000: return -DistSinQuadTable[index & 0xFFFF];//180 to 270 degrees...
	case 0x30000: return -DistSinQuadTable[0x1FFFF-(index &0x1FFFF)];//270 to 360 degrees...
	}
return 0; //We should never get here...
}




float LookUpSinRad(float rads)
{
unsigned long index=rads*131072.0/pi;
//0 to 2pi now represented by 0 to 0x3FFFF
return CoreSinLookup(index);
}

//Now round off for values bigger than 2pi



float LookUpCosRad(float rads)
{
unsigned long index=rads*131072.0/pi;
//0 to 2pi now represented by 0 to 0x3FFFF
//Now round off for values bigger than 2pi
index+=0x10000;//Cos is just sin(x+90)
return CoreSinLookup(index);
}


float LookUpSinDeg(float deg)
{
unsigned long index=(deg*131072.0/180.0);
//0 to 2pi now represented by 0 to 0x3FFFF
return CoreSinLookup(index);
}

float LookUpCosDeg(float deg)
{
unsigned long index=(deg*131072.0/180.0);
//0 to 2pi now represented by 0 to 0x3FFFF
//Now round off for values bigger than 2pi
index+=0x10000;//Cos is just sin(x+90)
return CoreSinLookup(index);
}


void LookUpSinCosDistIndex(unsigned long index ,float & s ,float & c)
{
s=CoreDistSinLookup(index);
c=CoreDistSinLookup(index+0x10000);
}

void LookUpSinCosIndex(unsigned long index ,float & s ,float & c)
{
s=CoreSinLookup(index);
c=CoreSinLookup(index+0x10000);
}

unsigned long ConvertDegToIndex(float deg)
{
return (deg*131072.0/180.0); 
}




float inv_sqrt( float number )
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;

	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                       // evil floating point bit level hacking
	i  = 0x5f3759df - ( i >> 1 );               // wtf?
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
	return y;
}

void Normalize(float &x, float &y)
{
  float mag=(x*x)+(y*y);
  if(mag!=0)
  {
   mag=inv_sqrt(mag);
   x*=mag;
   y*=mag;
  }
}


#define PI_FLOAT     3.14159265f
#define PIBY2_FLOAT  1.5707963f
// |error| < 0.005
float Fast_atan2( float y, float x )
{
	if ( x == 0.0f )
	{
		if ( y > 0.0f ) return PIBY2_FLOAT;
		if ( y == 0.0f ) return 0.0f;
		return -PIBY2_FLOAT;
	}
	float atan;
	float z = y/x;
	if ( fabs( z ) < 1.0f )
	{
		atan = z/(1.0f + 0.28f*z*z);
		if ( x < 0.0f )
		{
			if ( y < 0.0f ) return atan - PI_FLOAT;
			return atan + PI_FLOAT;
		}
	}
	else
	{
		atan = PIBY2_FLOAT - z/(z*z + 0.28f);
		if ( y < 0.0f ) return atan - PI_FLOAT;
	}
	return atan;
}



/*
// Test Code

#include <stdio.h>
#include <math.h>


int main()
{
double max_err=-100.0;
double min_err=100.0;
double mev=9999;
double minev=9999;


for (double dt=0; dt<360.0; dt+=0.0001)
 {
   double sinv=sin(dt*pi/180.0);
   double lsinv=LookUpSin(dt*pi/180.0);
   double cosv=cos(dt*pi/180.0);
   double lcosv=LookUpCos(dt*pi/180.0);
   double err=lsinv-sinv;

   if(err>max_err) {max_err=err; mev=dt; }
   if(err<min_err) {min_err=err; minev=dt; }
   err=lcosv-cosv;
   if(err>max_err) {max_err=err; mev=1000+dt; }
   if(err<min_err) {min_err=err; minev=1000+dt; }
 }
printf("Max error = %.20f at %f  Min Error=%.20f at %f\n",max_err,mev,min_err,minev);
return 0;
}

*/
