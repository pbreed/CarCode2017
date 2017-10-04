class fPoint
{ public:
	 float x;
     float y;
 fPoint(float xi=0, float yi=0) {x=xi; y=yi; };
 fPoint & operator  =(const fPoint d) {x=d.x; y=d.y; return *this; }; 
 float Dist(const fPoint &p1);
 float HeadToHereDeg(const fPoint &from);
};

float Calc_Distance(const fPoint &p1, const fPoint &p2);
float Calc_HeadDeg(const fPoint &pfrom,const  fPoint &pto);                                                                                                                                                                                     

float inv_sqrt( float number );





