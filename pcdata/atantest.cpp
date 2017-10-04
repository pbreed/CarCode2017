#include <stdio.h>
#include <stdlib.h>
#include <math.h>

const float M_PI=3.14159265358979323846;
 
float atan2_approximation1(float y, float x);
float atan2_approximation2(float y, float x);


float rd(float a)
{
return a*180.0/M_PI;
}


int main()
{
    float eo=0;
    float e1=0;
	float e2=0;

	for(float f=0; f<(2*M_PI); f+=0.0001)
	{
	float s=sin(f);
	float c=cos(f);
	float ato=atan2(s,c);
	float at1=atan2_approximation1(s, c);
	float at2=atan2_approximation2(s, c);
	float err=fabs((f-ato));
	if(err>=(M_PI)) err-=2*M_PI;
	if(err>eo) eo=err;

	err=fabs((f-at1));
	if(err>=(M_PI)) err-=2*M_PI;
	if(err>e1) e1=err;
	err=fabs((f-at2));
	if(err>=(M_PI)) err-=2*M_PI;
	if(err>e2) e2=err;


	 printf("for %5.3g = %5.3g %5.3g %5.3g\n",rd(f),rd(ato),rd(at1),rd(at2));
    }

	printf("Eo Mag = %g E1 Max=%g, E2 Max=%g\n",rd(eo),(e1),rd(e2));


    return 0;
}

float atan2_approximation1(float y, float x)
{
    //http://pubs.opengroup.org/onlinepubs/009695399/functions/atan2.html
    //Volkan SALMA

    const float ONEQTR_PI = M_PI / 4.0;
	const float THRQTR_PI = 3.0 * M_PI / 4.0;
	float r, angle;
	float abs_y = fabs(y) + 1e-10f;      // kludge to prevent 0/0 condition
	if ( x < 0.0f )
	{
		r = (x + abs_y) / (abs_y - x);
		angle = THRQTR_PI;
	}
	else
	{
		r = (x - abs_y) / (x + abs_y);
		angle = ONEQTR_PI;
	}
	angle += (0.1963f * r * r - 0.9817f) * r;
	if ( y < 0.0f )
		return( -angle );     // negate if in quad III or IV
	else
		return( angle );


}

#define PI_FLOAT     3.14159265f
#define PIBY2_FLOAT  1.5707963f
// |error| < 0.005
float atan2_approximation2( float y, float x )
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
