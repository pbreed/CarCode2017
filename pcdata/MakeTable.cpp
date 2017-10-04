#include <math.h>
#include <stdio.h>


#define M_PI	3.14159265358979323846

int main()
{
printf(" const float AX128ToSinCosInIn[%d][2] = {\n",128*360);	
double d;

for( int i=0; i<((360*128)-1); i++)
	{
	d=i;
	d/=128;
	d*=(M_PI/180.0);
	printf("{%g,%g},\n",sin(d)/(40.0*2.54),cos(d)/(40.0*2.54));
	}
	d=360.0-(1.0/128);
	d*=(M_PI/180.0);
 	printf("{%g,%g}};\n",sin(d)/(40.0*2.54),cos(d)/(40.0*2.54));
return 0;
}
