
/*
pt The X,Y location

Edgev either null 
or 
	  inter true if doing intercept rather than follow
	  adj_dist should we adjust the calculated l/r distance;
	  null or difference between path and wall heading
	  midpoitn of path to wall
 

corner_d either null 
	  adj_lr true/false adjust left right based on corner detect
	  adj_dist true/fals adjsut the distance along path
	  corner_pt=corner_pt where in cordiante the corner is
	  path_pt=path_pt where the path should reset to when detected
	  indent true if corner goes in

next_seq either null or number of next seq
bStop true/false    
Options=null;	//null ped bar ramp
Speed=null or mph
*/



struct path_element
{
float x;
float y;
float speed;
float m_CornerDet_Path_X; 
float m_CornerDet_Path_Y; 
float m_CornerAct_X;      
float m_CornerAct_Y;      
float m_edge_x;
float m_edge_y;
float m_edge_dist; //+ to right, - to left
float m_Corner_dist; //+ to right - to left;
char m_option[20];
int next_seq; //Either next numeber or -1 to stop

bool m_bDoEdge;
bool m_edge_intercept;
bool m_edge_adj_dist;
bool m_adj_head;
bool m_bDoCorner;
bool m_bCornerAdj_LR;
bool m_bCornerAdj_FA;
bool m_bCorner_Indent;
bool m_bValid;

path_element()
{
x=0;
y=0;
speed=0;
m_CornerDet_Path_X=0;
m_CornerDet_Path_Y=0;
m_CornerAct_X=0;     
m_CornerAct_Y=0;     
m_edge_x=0;          
m_edge_y=0;          
m_edge_dist=0; //+ to
m_Corner_dist =0;
m_option[0]=0;
next_seq=-1;
m_bDoEdge=false;       
m_edge_intercept=false;
m_edge_adj_dist=false; 
m_adj_head=false;      
m_bDoCorner=false;     
m_bCornerAdj_LR=false; 
m_bCornerAdj_FA=false; 
m_bCorner_Indent=false;
m_bValid=false;         
};

void PathInitalValue()
{
x=0;
y=0;
speed=0;
m_CornerDet_Path_X=0;
m_CornerDet_Path_Y=0;
m_CornerAct_X=0;     
m_CornerAct_Y=0;     
m_edge_x=0;          
m_edge_y=0;          
m_edge_dist=0; //+ to
m_Corner_dist=0; 
m_option[0]=0;
next_seq=-1;
m_bDoEdge=false;       
m_edge_intercept=false;
m_edge_adj_dist=false; 
m_adj_head=false;      
m_bDoCorner=false;     
m_bCornerAdj_LR=false; 
m_bCornerAdj_FA=false; 
m_bCorner_Indent=false;
m_bValid=false;         
};

bool Parse(const char * & cp,int def_next_seq);
void Show();
};



extern path_element PathArray[];
void PopulatePath();


