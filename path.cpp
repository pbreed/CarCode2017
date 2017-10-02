#include <http.h>
#include <stdio.h>
#include <utils.h>
#include <iosys.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include "path.h"

char * pPostPath;
int postDataLen;
							   
void ProcessPathPost( int sock, char *url, char *pData, char *rxBuffer)
{
SendHTMLHeader( sock );
writestring( sock, "<HTML><BODY>Got Post </BODY></HTML>" );
while((*pData) && (*pData!='{')) pData++;
int len=strlen(pData);
if(pPostPath) free(pPostPath);
pPostPath=(char *)malloc(len+1);

memcpy(pPostPath,pData,len);
pPostPath[len+1]=0;
postDataLen=len;

iprintf("Got post of %d bytes\r\n",len);
}



int MyDoPost( int sock, char *url, char *pData, char *rxBuffer )
{
   iprintf( "Processing post for %s\r\n", url );
   if ( httpstricmp( url, "PATHPOST" ) == 0 )
   {
	  ProcessPathPost(sock,url,pData,rxBuffer );
   }

   return 0;
}



void RegisterPost()
{
   SetNewPostHandler( MyDoPost );
}


extern const unsigned long PathDataLen; 
extern const unsigned char PathData[];

path_element PathArray[1000];


class Point
{ public:
	 float x;
     float y;
 Point(float xi=0, float yi=0) {x=xi; y=yi; };
};

float Calc_Distance(Point p1, Point p2)
{
float s=((p1.x-p2.x)*(p1.x-p2.x))+((p1.y-p2.y)*(p1.y-p2.y));
return sqrtf(s);
}

#define PI (3.141592653589793)

//Assumes X is west /east
//Assumes y is norht south
float Calc_HeadDeg(Point pfrom, Point pto)
{
float dx=(pto.x-pfrom.x);
float dy=(pto.y-pfrom.y);

if(dx==0) //North or South
{
  if(dy>0) return 0;
  else
	  return 180.0;
}
else
if(dy==0)
{
   if(dx>0) return 90.0;
   else return -90.0;
}

float d=atan2(dy,dx);
return (float)d*180.0/PI;
}



void ParseAndPopulatePath(const char * cp)
{
const char * cpo;


cp=strstr(cp,"Path");
if(cp)
{
while ((*cp!='[') && (*cp)) cp++;
if (*cp) 
 { cp++; //Go past [
	int n=0; 
    cpo=cp;
	while((*cp) && (*cp!=']') && (PathArray[n].Parse(cp,n+1)))
		{
	
		if(PathArray[n].m_bDoEdge)
		{
		 Point pnext(PathArray[n].x,PathArray[n].y);
		 Point pprev(PathArray[n-1].x,PathArray[-1].y);
		 Point wall(PathArray[n].m_edge_x,PathArray[n].m_edge_y);
		 Point pcenter((pnext.x+pprev.x)/2,(pnext.y+pprev.y)/2);
		 float dist=Calc_Distance(pcenter,wall);
		 float head=Calc_HeadDeg(pprev,pnext);
		 float whead=Calc_HeadDeg(pcenter,wall);
		 float diff=whead-head;
		 while (diff>180) diff-=360;
		 while (diff<-180) diff+=360;
		 if(diff<0) 
			 PathArray[n].m_edge_dist=dist; //+ to right, - to left 
		 else
			 PathArray[n].m_edge_dist=-dist; //+ to right, - to left 
		}
		
		if(PathArray[n].m_bDoCorner)
		{
		 Point pnext(PathArray[n].x,PathArray[n].y);
		 Point pprev(PathArray[n-1].x,PathArray[-1].y);
		 Point corner(PathArray[n].m_CornerAct_X,PathArray[n].m_CornerAct_Y);
		 Point onpath(PathArray[n].m_CornerDet_Path_X,PathArray[n].m_CornerDet_Path_Y);
		 float dist=Calc_Distance(onpath,corner);
		 float head=Calc_HeadDeg(pprev,pnext);
		 float chead=Calc_HeadDeg(onpath,corner);

		 float diff=chead-head;
		 while (diff>180) diff-=360;
		 while (diff<-180) diff+=360;
		 if(diff<0) 
			 PathArray[n].m_Corner_dist=-dist;
			 else
			 PathArray[n].m_Corner_dist=dist;
		}


		n++;
		
		/*	iprintf("Parsed[%d][",n);
		while(cpo!=cp)
		{
			iprintf("%c",*cpo++);
		}
		iprintf("]\r\n");
		PathArray[n-1].Show();
		*/
		}; 
    //iprintf("Parsed %d path points\r\n",n);
	PathArray[n].m_bValid=false;
 }
}
}

void PopulatePath()
{
const char * cp;
if(pPostPath) 
	{cp=pPostPath;
    }
else
{ cp=(const char *)PathData;
}

ParseAndPopulatePath(cp);
}







void trimlead(const char * &cp)
{
    while(*cp)
	{
	 char c=*cp;
	 if (!((isspace(c)) || (c==','))) return; 
	 cp++;
	}
}


bool strid(const char* & cp,const char * id)
{
const char * cpy=cp;
char cpd='"';

trimlead(cpy);
if((*cpy=='"')|| (*cpy=='\''))
{
cpd=*cpy;
cpy++;
if(strstr(cpy,id)==cpy)
 {
   cpy+=strlen(id);
	if (*cpy==cpd)
	{
		cpy++;
		trimlead(cpy);
		if(*cpy==':')
		{
		cpy++;
		cp=cpy;
		trimlead(cp);
	  //  iprintf("Matched [%s]\r\n",id);
		return true;
		}
	}


 }

}
return false;
}

void ReadString(const char* &cp, char * dest, int maxlen )
{
	trimlead(cp);
	if((*cp=='"')|| (*cp=='\''))
	{
		char cpd=*cp;
		cp++;
	   while((*cp) && (*cp!=cpd))
	   {
		if(*cp=='\\')
		{
		  cp++;
		}
		
		if(maxlen) {*dest++=*cp; maxlen--; };
		cp++;
	   }
	   if(*cp==cpd)
		   cp++;
	}
	if(maxlen) *dest=0;

	trimlead(cp);
}


float ReadFNumber(const char* & cp)
{	double d=0;
	trimlead(cp);
	if((*cp=='\"') || (*cp=='\''))
	{char tbuf[40];
	 ReadString(cp,tbuf,40);
	 d=strtod(tbuf,0);
    }
	else
	{
    char * cpe;
	d=strtod (cp,&cpe);
	cp=cpe;
	}
	trimlead(cp);
	return (float)d;
}


int ReadINumber(const char* & cp)
{
return (int)ReadFNumber(cp);
}


bool ReadBool(const char* & cp)
{
trimlead(cp);
if(strstr(cp,"true")==cp)
{
 cp+=4;
 trimlead(cp);
 return true;
}
if(strstr(cp,"false")==cp)
{
cp+=5;
trimlead(cp);
}
return false;
}


/*void eatbrackets(const char * &cp)
{
 trimlead(cp);
 int l=0;
 if(*cp=='{')
 {	cp++;
	while(*cp)
	{
	 if(*cp=='{') l++;
	 if(*cp=='}') l--;
	 cp++;
	 if(l<0) break;;
	}
 }
 trimlead(cp);
}
*/
bool nextnull(const char* &cp)
{

 if(strstr(cp,"null") ==cp)
 {
  cp+=4;
  trimlead(cp);
  return true;
 }
 return false;
}

void ReadXY(const char* &cp, float & x, float & y)
{
	trimlead(cp);
    if(*cp!='{') return;
	cp++;
    while((*cp) && (*cp!='}'))
	 {

		 if(strid(cp,"x")){x=ReadFNumber(cp);}
		 if(strid(cp,"y")){y=ReadFNumber(cp);}
	 }
   if(*cp=='}') cp++;
}


void path_element::Show()
{
printf("[x:%4.2f, y:%4.2f] Edge %d Corner %d \r\n",x,y,m_bDoEdge,m_bDoCorner);
}


bool path_element::Parse(const char * & cp,int def_next_seq)
{
 
//  iprintf("\nAbout to parse starting at :[%c]\n",*cp);
  
  trimlead(cp);

 PathInitalValue();
 next_seq=def_next_seq;
 
 if(*cp!='{') return false;
 cp++;
 trimlead(cp);
 
 
 while((*cp) && (*cp!='}'))
 {
 if(strid(cp,"pt"))
  {
   ReadXY(cp,x,y);
  }
 
 if(strid(cp,"Edgev"))
	 {
	   if(!nextnull(cp))
	   {
        /*"Edgev": {
			"inter": true,
			"adj_dist": false,
			"adj_head": null,
			"pt": {
				"x": 947.3976887471016,
				"y": 587.6540727294537
			}
		},*/

		   trimlead(cp);
		  if(*cp!='{') return false;
		  cp++;
		  trimlead(cp);
		  
		  m_bDoEdge=true;
          while((*cp) && (*cp!='}'))
	       {
			  if(strid(cp,"inter")) {m_edge_intercept=ReadBool(cp);}
			  if(strid(cp,"adj_dist")) {m_edge_adj_dist=ReadBool(cp);}
			  if(strid(cp,"adj_head")) 
				  {
				  if (!nextnull(cp))
					{ 
					  ReadFNumber(cp);
					  m_adj_head=true;
				    } else 
					{
						m_adj_head=false;
					}

				  }
			  if(strid(cp,"pt"))  {ReadXY(cp,m_edge_x,m_edge_y);}
		  }
		  if(*cp=='}') cp++;
		  else 
			  {
			  //iprintf("Bailing Early at %c,%c,%c\r\n",*cp,cp[1],cp[2]);
			  return false;
			   }


	   }
	   //init value handled the null
	 }

 if(strid(cp,"corner_d"))
	 {
	 if(!nextnull(cp))
		  {
		  trimlead(cp);
		  m_bDoCorner=true;
		  if(*cp!='{') return false;
		  cp++;
		  trimlead(cp);
          while((*cp) && (*cp!='}'))
	       {
			  if(strid(cp,"adj_lr")) {m_bCornerAdj_LR=ReadBool(cp);}
			  if(strid(cp,"adj_dist")){m_bCornerAdj_FA=ReadBool(cp);}
			  if(strid(cp,"corner_pt")) 
				  {
				  ReadXY(cp,m_CornerAct_X,m_CornerAct_Y);
				  }

			  if(strid(cp,"path_pt")) 
				 {
				  ReadXY(cp,m_CornerDet_Path_X,m_CornerDet_Path_Y);
				 }
			  if(strid(cp,"indent")) {m_bCorner_Indent=ReadBool(cp);}
 		   }
		  if(*cp=='}') cp++;
		  else return false;
		 }
		  //init value handled the null
     }
 
 if(strid(cp,"next_seq"))
	 {
	 if(!nextnull(cp))
		  {
		   next_seq=ReadINumber(cp);
		  }
		  //init value handled the null
	 }
 if(strid(cp,"Options")){if(!nextnull(cp))ReadString(cp,m_option,20); }
 if(strid(cp,"bStop")){if(ReadBool(cp)) next_seq=-1;}
 if(strid(cp,"Speed"))
	 {
	 trimlead(cp); 
	 if (!nextnull(cp))
	 { 
		 speed=ReadFNumber(cp);
	 } else 
		 speed=-1;

     }
// iprintf("afta Speed *cp=[%c]\r\n",*cp);
// ShowData((uint8_t *)cp,20);
 }
 trimlead(cp);
 if(*cp=='}') 
	{ cp++;
       m_bValid=true;
    }
 return true;
}




