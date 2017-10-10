#include <http.h>
#include <stdio.h>
#include <utils.h>
#include <iosys.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <system.h>
#include <buffers.h>
#include <fdprintf.h>
#include <config_obj.h>
#include "SinLookup.h"
#include "nav.h"
#include "path.h"
#include "introspec.h"
LOGFILEINFO;


extern "C"
{void GetPathJson(int sock, const char * url);
}
							
void ParseAndPopulatePath(const char * cp);


void GetPathJson(int sock, const char * url)
{
const char * cp=(const char *) GetUserParameters();
if(cp[0]=='{') writestring(sock,cp);
}

void ProcessPathPost( int sock, char *url, char *pData, char *rxBuffer)
{
while((*pData) && (*pData!='{')) pData++;
int len=strlen(pData);
if(len < 8191) SaveUserParameters(pData,len+1);
iprintf("Got post of %d bytes\r\n",len);
ParseAndPopulatePath(pData);
RedirectResponse(sock,"PathUp.html"); 

}

void ProcessRunPropsPost( int sock, char *url, char *pData, char *rxBuffer)
{
SendHTMLHeader( sock );
writestring( sock, "<HTML><BODY>Got Run Props Post </BODY></HTML>" );
}

void ProcessPathPostFile( int sock, char *url, char *pData, char *rxBuffer)
{
while((*pData) && (*pData!='{')) pData++;
int len=strlen(pData);
if(len < 8191) SaveUserParameters(pData,len+1);
iprintf("Got post of %d bytes\r\n",len);
ParseAndPopulatePath(pData);
RedirectResponse(sock,"PathUp.html"); 
}


static http_gethandler *oldhand;
int MyDoGet( int sock, char *  url, char *  rxBuffer )
{
 iprintf("Doing Get for %s\r\n",url);
	if ( strlen( url ) )
	{
		if (httpstricmp(url,"CONFIG")) 
		  {
		   char * cp=url;
		   while((*cp!='c') && (*cp!='C')) cp++;
		   cp+=6;
		   if(strstr(cp,".json"))
		   {
			   *strstr(cp,".json")=0;
		   }
		   iprintf("Searching [%s]\r\n",cp);

			 config_leaf * pl=config_leaf::FindConfigLeaf((unsigned char *)cp);
			// config_leaf * pl=config_leaf::FindConfigLeaf((unsigned char *)"APPDATA/RunProp");

			 if (pl) {	iprintf("Found config\r\n");
						 writestring(sock,"HTTP/1.0 200 OK\r\nPragma: no-cache\r\nContent-Type: application/json\r\n\r\n" );
						 pl->RenderToFd(sock,true);
						 return 1;
					 }
			 else
				iprintf("Did not find config leaf");

	   }
	}
	return ( *oldhand ) ( sock, url, rxBuffer );
}


int ShowCurPath(int sock, const char* pUrl)
{
  fdprintf(sock,"<TABLE border=\"1\"><TR><TH>#</TH><TH>X</TH><TH>Y</TH><TH>Head</TH><TH>Speed</TH><TH>Next</TH><TH>Arc</TH><TH>ArcR</TH><TH>Chord</TH><TH>Right/Left</TH></TR>\r\n");
  int i=0;
  while(PathArray[i].m_bValid)
  {
	  fdprintf(sock,"<TR><TD>%d</TD><TD>%g</TD><TD>%g</TD><TD>%g</TD><TD>%g</TD>",i,PathArray[i].pt.x,PathArray[i].pt.y,PathArray[i].m_base_head,PathArray[i].speed);

	  if(PathArray[i].next_seq!=i+1)
	  {
		  if((PathArray[i].next_seq==-1)||(PathArray[i+1].m_bValid==false))
		  {
			  fdprintf(sock,"<TD>STOP</TD>");

		  }
		  else
		  {
			fdprintf(sock,"<TD>%d</TD>",PathArray[i].next_seq);
		  }

	  }
	  else
	  {
		  fdprintf(sock,"<TD></TD>");
	  }

	  if(PathArray[i].m_bArc)
	  {
	    fdprintf(sock,"<TD>true</TD><TD>%g</TD><TD>%g</TD>",PathArray[i].m_arc_r,PathArray[i].m_ChordAngle);
		if(PathArray[i].m_ChordAngle<0)
			fdprintf(sock,"<TD>left</TD></TR>\r\n");
			else
    		fdprintf(sock,"<TD>right</TD></TR>\r\n");
	  }
	  else
	  fdprintf(sock,"<TD>false</TD></TR>\r\n");
	i++;
  }
  fdprintf(sock,"</TABLE><BR>\r\n");
 return 1;
}

int MyDoPost( int sock, char *url, char *pData, char *rxBuffer )
{
   iprintf( "Processing post for %s\r\n", url );

   if(url[0]=='/') url++;
   if(url[0]=='\\') url++;
   if ( httpstricmp( url,"PATH" ))
   {
	   iprintf("Processing post[%s]\r\n",url);
	  ProcessPathPost(sock,url,pData,rxBuffer );
   }
   else
   if ( httpstricmp( url,"PATHPOST.HTML"))
	{
	ProcessPathPostFile(sock,url,pData,rxBuffer);
	}
   if ( httpstricmp( url,"CONFIG" ))
   {
	iprintf("Processing port 80 config post\r\n");

   }
   else
   {
	iprintf("Failed to do post [%s]\r\n",url);
   }

   return 0;
}



void RegisterPost()
{
   oldhand = SetNewGetHandler( MyDoGet );
   SetNewPostHandler( MyDoPost );
}


extern const unsigned long PathDataLen;
extern const unsigned char PathData[];

path_element PathArray[1000];



float Calc_Distance(const fPoint &p1,const  fPoint &p2)
{
float s=((p1.x-p2.x)*(p1.x-p2.x))+((p1.y-p2.y)*(p1.y-p2.y));
return sqrtf(s);
}

#define PI (3.141592653589793)

//Assumes X is west /east
//Assumes y is norht south
float Calc_HeadDeg(const fPoint &pfrom,const  fPoint &pto)                                                                                                                                                                                     
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

// So realize that atan 2 works on unit circle where 0 deg is east 
// and 90 is north
// and 180 is west
// and 270/-90 is south

float d=Fast_atan2(dx,dy);
//So we need to reverse direction....
d*=(180.0/PI);
//and rotate 90
//d-=90;
if(d<-180) d+=360.0;
if(d> 180) d-=360.0;
return d;
}

float fPoint::Dist(const fPoint &p1) {return Calc_Distance(*this,p1); };
float fPoint::HeadToHereDeg(const fPoint &from) {return Calc_HeadDeg(from,*this);};





static float xoffset;
static float yoffset;
static bool bOffset_Set;

void ParseAndPopulatePath(const char * cp)
{
/*iprintf("Aborting early....First bit of post data...");
for(int i=0; i<256; i++)
{
if(cp[i]!=0) iprintf("%c",cp[i]);
else break;
}

return ;
*/
cp=strstr(cp,"Path");
if(cp)
{
while ((*cp!='[') && (*cp)) cp++;
if (*cp)
 { cp++; //Go past [
	int n=0;
	xoffset=0;
	yoffset=0;
	bOffset_Set=false;
	while((*cp) && (*cp!=']') && (PathArray[n].Parse(cp,n+1)))
		{
        if((PathArray[n].m_bDoEdge) && (n>0))
		{
		 fPoint pnext=PathArray[n].pt;
		 fPoint pprev=PathArray[n-1].pt;
		 fPoint wall(PathArray[n].m_edge_x,PathArray[n].m_edge_y);
		 fPoint pcenter((pnext.x+pprev.x)/2,(pnext.y+pprev.y)/2);
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
		
		if((PathArray[n].m_bDoCorner)&& (n>0))
		{
		 fPoint pnext=PathArray[n].pt;
         fPoint pprev=PathArray[n-1].pt;
		 fPoint corner(PathArray[n].m_CornerAct_X,PathArray[n].m_CornerAct_Y);
		 fPoint onpath(PathArray[n].m_CornerDet_Path_X,PathArray[n].m_CornerDet_Path_Y);
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
const char * cp=(const char *) GetUserParameters();
if(cp[0]=='{') ParseAndPopulatePath(cp);
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


void eatbrackets(const char * &cp)
{
 trimlead(cp);
 int l=0;
 if(*cp=='{')
 { iprintf("{");
    cp++;
	while(*cp)
	{
	  iprintf("%c",*cp);

	 if(*cp=='{') l++;
	 if(*cp=='}') l--;
	 cp++;
	 if(l<0) break;;
	}
 }
 trimlead(cp);
 iprintf("\r\n Eat brackets done\r\n");
}


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

START_INTRO_OBJ(PathOrigin,"PathOrigin")
float_element x{"x"};
float_element y{"y"};
END_INTRO_OBJ;

static PathOrigin porg;


void ReadXY(const char* &cp, float & x, float & y)
{
	trimlead(cp);
    if(*cp!='{') return;
	cp++;
    while((*cp) && (*cp!='}'))
	 {

		 if(strid(cp,"x")){x=ReadFNumber(cp);}
		 if(strid(cp,"y")){y=-ReadFNumber(cp);}  /*Y Signs are reversed! */
	 }
     if(!bOffset_Set)
	 {
      xoffset=x;
      yoffset=y;   
	  bOffset_Set=true;
	  porg.x=xoffset;
	  porg.y=yoffset;
	  porg.Log(true);
	 }
     x-=xoffset;
	 y-=yoffset;

   if(*cp=='}') cp++;
}


void ReadXY(const char* &cp, fPoint & p)
{
ReadXY(cp,p.x,p.y);
}


void path_element::Show()
{
printf("[x:%4.2f, y:%4.2f] Edge %d Corner %d \r\n",pt.x,pt.y,m_bDoEdge,m_bDoCorner);
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
// iprintf("At [%c%c%c%c%c]\r\n",cp[0],cp[1],cp[2],cp[3],cp[4]);
 if(strid(cp,"pt")){ReadXY(cp,pt);}
 else
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
	else
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
			  if(strid(cp,"corner_pt")){ReadXY(cp,m_CornerAct_X,m_CornerAct_Y);}

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
  else
 if(strid(cp,"next_seq")){if(!nextnull(cp)){next_seq=ReadINumber(cp);}}
 else
	 if(strid(cp,"ix")){if(!nextnull(cp)){ReadFNumber(cp);}}
	else
	 if(strid(cp,"iy")){if(!nextnull(cp)){ReadFNumber(cp);}}
	 else
	  if(strid(cp,"arc_r")){if(!nextnull(cp)){m_arc_r=ReadFNumber(cp);}}
  else
  if(strid(cp,"head")){if(!nextnull(cp)){m_base_head=ReadFNumber(cp);}}
  else
  if(strid(cp,"ChordAngle")){if(!nextnull(cp)){m_ChordAngle=ReadFNumber(cp);}}
  else
  if(strid(cp,"center_arc")){if(!nextnull(cp)){ReadXY(cp,m_ArcCenter);}}
  else
  if(strid(cp,"p1")){if(!nextnull(cp)){ReadXY(cp,m_p1);}}
  else
  if(strid(cp,"p2")){if(!nextnull(cp)){ReadXY(cp,m_p2);}}
  else
 if(strid(cp,"Options")){if(!nextnull(cp))ReadString(cp,m_option,20); }
 else
 if(strid(cp,"bStop")){if(ReadBool(cp)) next_seq=-1;}
 else
 if(strid(cp,"Speed"))
	 {
	 trimlead(cp);
	 if (!nextnull(cp))
	 {
		 speed=ReadFNumber(cp);
	 } else
		 speed=-1;

     }
   else
   if(strid(cp,"Arc")){m_bArc=ReadBool(cp);}
   else
   {
	iprintf("Unknown ID:\r\n");
	while(*cp!=':')
	{
	 iprintf("%c",*cp++);
	}
	iprintf(":");
	cp++;

	while((*cp) && (*cp!=',') && (*cp!='}'))
	{
	  iprintf("%c",*cp);

     if(*cp=='{')
	 {
		 eatbrackets(cp);
     }
	 else cp++;
    }
	iprintf("%c",*cp);
	iprintf("\r\n*******END Unknown \r\n");

	if(*cp==',') 
		{
		 cp++;
	    }

   }
 
 trimlead(cp);
 if(*cp=='}')
	{ cp++;
       m_bValid=true;
	   return true;
    }
 }
 return true;
}




