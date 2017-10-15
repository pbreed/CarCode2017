#include <stdio.h>

int main()
{
	for(int wb=0; wb<256; wb++)
	{  int s=0;
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
	  printf("%+d,",s);
	  if(wb %32==31) printf("\n");
	}
return 0;
}
