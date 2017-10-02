#include <ctype.h>
#include <string>
#include <math.h>
#include <stdio.h>

using namespace std;


/*
+  r0 <- r1+r0 pop one
-  r0 <- r1-r0 pop one
/  r0 <- r1/r0 pop one
*  r0 <- r1*r0 pop one
^  r0 <- r1^r0 pop one
<  r0 <- if(r0<r1) r0->1 else 0 pop one
>  r0 <- if(r0>r1) r0->1 else 0 pop one
=  r0 <- if(r0==r1) r0->1 else 0 pop one
push push r1->r0
pop  r0-<r1 pop one
swap swap ro,r1
neg r0<- -r0
sin	  r0 <-  sin(r0)
cos   r0 <-  cos(r0)
tan   r0 <-  tan(r0)
asin  r0 <-  asin(r0)
acos  r0 <-  acos(r0)
atan  r0 <-  atan(r0)
atan2 r0 <-  atan2(r0)
log   r0 <-  log(r0)
ln    r0 <-  ln(r0)
pi	  ro-<pi
#number ->  push r0 <- atof number
*/

double stack[16];

enum Token {eadd,esub,ediv,emul,epow,elt,egt,eeq,epush,epop,eswp,eneg,esin,ecos,etan,easin,eacos,eatan,eatan2,elog,eln,epi,eval,eeof,enop, eor, eand, enot};


Token NextToken(string & s,double & val)
{
	while((!s.empty()) && (isspace(s[0]))) s=s.substr(1,string::npos);
	if(s.empty()) return eeof;

	Token rv=enop;

if(isdigit(s[0]) || ((s[0]=='-')&& isdigit(s[1])))
{
const char * cp=s.c_str();
rv=eval;
val=atof(cp);
}
else
{
switch (s[0])
{
case '+': rv=eadd; break;
case '-': rv=esub; break;
case '/': rv=ediv; break;
case '*': rv=emul; break;
case '^': rv=epow; break;
case '<': rv=elt; break;
case '>': rv=egt; break;
case '=': rv=eeq; break;
case 'c': rv=ecos;break;
case '#': rv=eval; break;
case 'a':
	      if(s[1]=='n') {rv=eand; break; }
	      if(s[1]=='s') {rv=easin; break; }
	      if(s[1]=='c') {rv=eacos; break; }
	      if(s[1]=='t')
			  {
			   if(s[4]=='2') rv=eatan2; else rv=eatan; break;
			  }
		  break;
case 'l':
	if(s[1]=='n') {rv=eln; break; }
	if(s[1]=='o') {rv=elog; break; }
	break;
case 'n':
	if(s[1]=='o') {rv=enot; break; }
	if(s[1]=='e') {rv=eneg; break; }
	break;
case 'o': rv=eor; break;
case 'p':
	if(s[1]=='u') {rv=epush; break; }
	if(s[1]=='o') {rv=epop; break; }
	if(s[1]=='i') {rv=epi; break; }
	break;
case 's':
	if(s[1]=='i') {rv=esin; break; }
	if(s[1]=='w') {rv=eswp; break; }
	break;

case 't': rv=etan; break;
}
if(rv==eval)
{
while((!s.empty()) &&  (isspace(s[0]))) s=s.substr(1,string::npos);
val=atof(s.c_str());
}

}

s=s.substr(1,string::npos);

while ((!s.empty()) && (s[0]!=' ')) s=s.substr(1,string::npos);

return rv;
}


void popat1()
{
for(int i=1; i<15; i++) stack[i]=stack[i+1];
}


const char * TokName(Token t)
{
switch(t)
{
case eadd: return "add";
case esub: return "sub";
case ediv: return "div";
case emul: return "mul";
case epow: return "pow";
case elt:  return "lt";
case egt:  return "gt";
case eeq:  return "eq";
case epush: return "pus";
case epop: return "pop";
case eswp: return "swp";
case eneg: return "neg";
case esin: return "sin";
case ecos: return "cos";
case etan: return "tan";
case easin: return "asin";
case eacos: return "acos";
case eatan: return "atan";
case eatan2: return "atan2";
case elog: return "log";
case eln:  return "ln";
case epi:  return "pi";
case eval: return "val";
case eeof: return "eof";
case enop: return "nop";
case enot: return "not";
case eand: return "and";
case eor: return "or";


}
return "??";
}

string Eval(const string & s, const string & rpn)
{
double d=atof(s.c_str());
int i;
for(i=0; i<16; i++) stack[i]=0;
stack[0]=d;

//printf("starting v=%g for %s\n",d,s.c_str());
string rem=rpn;

Token t;
double val;
do
{
t=NextToken(rem,val);
switch(t)
{
case eadd: stack[0]=(stack[0]+stack[1]); popat1(); break;
case esub: stack[0]=(stack[1]+stack[0]); popat1(); break;
case ediv: stack[0]=(stack[1]/stack[0]); popat1(); break;
case emul: stack[0]=(stack[1]*stack[0]); popat1(); break;
case epow: stack[0]=pow(stack[1],stack[0]); popat1(); break;
case elt: if(stack[0]<stack[1]) stack[0]=1; else stack[0]=0; popat1(); break;
case egt: if(stack[0]>stack[1]) stack[0]=1; else stack[0]=0; popat1(); break;
case eeq: if(stack[0]==stack[1]) stack[0]=1; else stack[0]=0; popat1(); break;
case epush: for(i=15; i>0; i--) stack[i]=stack[i-1]; break;
case epop: stack[0]=stack[1]; popat1(); break;

case eswp: val=stack[0]; stack[0]=stack[1]; stack[1]=val; break;
case eneg: stack[0]=-stack[0]; break;
case esin: stack[0]=sin(stack[0]);break;
case ecos: stack[0]=cos(stack[0]);break;
case etan: stack[0]=tan(stack[0]);break;
case easin:stack[0]=asin(stack[0]); break;
case eacos:stack[0]=acos(stack[0]); break;
case eatan:stack[0]=atan(stack[0]); break;
case eatan2:stack[0]=atan2(stack[0],stack[1]); popat1(); break;
case elog: stack[0]=log10(stack[0]); break;
case eln: stack[0]=log(stack[0]); break;
case epi:  for(i=15; i>0; i--) stack[i]=stack[i-1]; stack[0]=3.141592654; break;
case eval: for(i=15; i>0; i--) stack[i]=stack[i-1]; stack[0]=val; break;
case eor: if((stack[0]!=0) || (stack[1]!=0)) stack[0]=1; else stack[0]=0; popat1(); break;
case eand: if((stack[0]!=0) && (stack[1]!=0)) stack[0]=1; else stack[0]=0; popat1(); break;
case enot: if((stack[0]!=0)) stack[0]=0; else stack[0]=1; popat1(); break;
case eeof: break;
case enop: break;
}//switch

//printf("Just did %s %g,%g,%g  [%s]\n",TokName(t),stack[0],stack[1],stack[2],rem.c_str());

}
while(t!=eeof);
char buffer[80];
sprintf(buffer,"%g",stack[0]);
string rs=buffer;
return rs;
}



/*
int main(int argc, const char ** argv)
{
string s2=argv[1];
if(argc>2)
{
	for(int i=2; i<argc; i++)
		{
		 s2=s2+" ";
		 s2+=argv[i];
	    }

}

printf("Evaling %s\n",s2.c_str());
string s3=Eval("0",s2);

printf("Result = %s\n",s3.c_str());
return 0;
}
*/
