#include <stdio.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <time.h>
#include <sys/time.h>

using namespace std;

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD
#define dsDrawSphere   dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule  dsDrawCapsuleD
#endif

#define VIEW 1
//#define VIEW 0

#define N_Box 0
#define N_Sphere 1
#define N_Cylinder 2

#define  NUM_l 9       // link number
#define  NUM_p 27      // parameter number
#define  NUM_c 2       // chambers number in one cylinder

#define XYZ 3
//#define Num_t 1000
#define Num_t 1000
#define PI 3.14159

//#define TIMESTEP 0.001
#define TIMESTEP 1e-4

dWorldID world;             // dynamic simulation world
dSpaceID space;             // contact dection space
dGeomID  ground;            // ground
dJointGroupID contactgroup; // contact group
dsFunctions fn;             // draw function of drawn stuff

typedef struct { // MyObject structure
  dBodyID body;  // ID number of (rigid) body (for dynamical simulations)
  dGeomID geom;  // ID number of geometry (for collision detection)
  double  l,r,m; // length [m], radius [m], weight [kg]
} MyObject;

typedef struct { 
  int num_link; 
  int num_mother; 
  int num_shape;
  dReal m;
  dReal l[XYZ];
  dReal a[XYZ];
  dReal p_j[XYZ]; 
  dReal p_c[XYZ]; 
  double  R[XYZ][XYZ];
  double RoM[2];
  double D;
  double Q;
  double dot_Q;
  double Q_old;
  double L_stk;
  double L_MA;
} RobotLink;

typedef struct { 
  double V;
  double dot_V;
  double prs;
  double prsTar;
} CylinderChamber;

static int STEPS = 0; // simulation step number

MyObject rlink[NUM_l]; // number
dJointID joint[NUM_l]; // joint ID number
RobotLink uLINK[NUM_l];
CylinderChamber Chamber[NUM_l][NUM_c];

double Pos_link_data [Num_t][NUM_l][XYZ];
double Pos_joint_data[Num_t][NUM_l][XYZ];
double Angle_data[Num_t][NUM_l];
double Prs_data [Num_t][NUM_l][NUM_c];

char filename_o[999];
char filename_m[999];
char filename_p[999];

dReal jointTorque[NUM_l];
unsigned int DirName;

dReal radius = 0.02;
dReal height = 0.1;
//dReal height = 0.0;

#define k_air 1.4
#define R_air 8.3
#define T_air 300

#define S_ori 5.6e-5
#define L_mgn 2e-2
#define Prs_room 1e+5

#define MAX_str 1024
#define NUM_OF_PHASE 10

double Value_valves_phase[NUM_OF_PHASE][NUM_l] = {};
double Time_phase[NUM_OF_PHASE] = {};
double Time_switch[NUM_OF_PHASE] = {};
double Value_valves[NUM_l] = {};

void load_command(void){
  FILE *fp_cmd;
  char str[MAX_str];
  //unsigned int num_line = 0;
  //unsigned int i, j;
  //double sum_time;
  //unsigned int phase;
  double phase_time[NUM_OF_PHASE];
  char s[NUM_l + 2][MAX_str]; 

  fp_cmd = fopen( "../data/cmd.dat", "r");
  
  if (fp_cmd == NULL){
    printf("File open error\n");
    return;
  }

  fgets( str, MAX_str, fp_cmd);
  for (int i = 0; i < NUM_OF_PHASE; i++){
    fgets( str, MAX_str, fp_cmd);
    sscanf( str, "%s %s  %s %s %s %s  %s %s %s %s  %s %s %s %s  %s %s %s %s", s[0], s[1],  
	    s[ 2], s[ 3], s[ 4], s[ 5],  s[ 6], s[ 7], s[ 8], s[9], 
	    s[10], s[11], s[12], s[13],  s[14], s[15], s[16], s[17]);
    phase_time[i] = atof( s[1]);
    for (int j = 0; j < NUM_l; j++)
      Value_valves_phase[i][j] = atof( s[j+2]);
  }
  
  for (int i = 0; i < NUM_OF_PHASE; i++){
    double sum_time = 0.0;
    for (int j = 0; j <= i; j++)
      sum_time = sum_time + phase_time[j];
    Time_switch[i] = sum_time;
  }

  //for (i = 0; i < NUM_OF_PHASE; i++)
  //printf( "%3.2f ", phase_time[i]);
  //printf("\n"); 
  //for (i = 0; i < NUM_OF_PHASE; i++)
  //printf( "%3.2f ", Time_switch[i]);
  //printf("\n"); 

  //for (i = 0; i < NUM_OF_PHASE; i++){
  //for (j = 0; j < NUM_OF_CHANNELS; j++)
  //printf( "%3.2f ", Value_valves_phase[i][j]);
  //printf("\n"); 
  //}

  fclose(fp_cmd);
}

int get_phase_number( double elasped_t_){
  unsigned int phase_num = 0;
  //unsigned int i;

  if ( elasped_t_ > Time_switch[NUM_OF_PHASE - 1])
    phase_num = NUM_OF_PHASE - 1;
  else 
    for ( int i = 1; i < NUM_OF_PHASE; i++)
      if ( elasped_t_ < Time_switch[i] && elasped_t_ > Time_switch[i - 1])
	phase_num = i;

  return phase_num;
}

void get_valve_value( double old_tv_s_, double old_tv_us_){
  //double elasped_t = get_elasped_ms_time( old_tv_s_, old_tv_us_);
  double elasped_t = double( STEPS);
  int num_phase    = get_phase_number( elasped_t);

  for ( int num_ch = 0; num_ch < NUM_l; num_ch++)
    Value_valves[num_ch] = Value_valves_phase[num_phase][num_ch];
}

double getMassFlowRate( double prsU, double prsD){
  double dot_m;

  if ( prsU/ prsD > pow( 2.0/ (k_air + 1), k_air/( k_air - 1.0)))
    dot_m = ( S_ori/ sqrt( T_air))* sqrt(( 2.0* k_air)/(( k_air - 1.0)* R_air))* prsU* sqrt( pow( prsU/ prsD, 2.0/k_air) - pow( prsU/ prsD, (k_air - 1.0)/ k_air));
  else 
    dot_m = ( S_ori/ sqrt( T_air))* sqrt(( 2.0* k_air)/(( k_air + 1.0)* R_air))* prsU;
  return dot_m; 
}

double getDotPressure( double prsNow, double dot_m, double V, double dot_V)
{
  return k_air* R_air* T_air* dot_m / V - k_air* prsNow* dot_V/ V;
}

void updateChamberAll()
{
  double L_cmb[NUM_c], dot_L_cmb[NUM_c];
  double L_tmp, dot_L_tmp;

  for (int i = 1; i < NUM_l; i++){
    L_tmp    = uLINK[i].L_MA*( uLINK[i].Q - uLINK[i].RoM[0]);

    L_cmb[0] = L_tmp + L_mgn;
    L_cmb[1] = (uLINK[i].L_stk - L_tmp) + L_mgn;

    Chamber[i][0].V = L_cmb[0]* PI* pow( uLINK[i].D, 2)/ 4;
    Chamber[i][1].V = L_cmb[1]* PI* pow( uLINK[i].D, 2)/ 4;
  }
  for (int i = 1; i < NUM_l; i++){
    dot_L_tmp           = uLINK[i].L_MA* uLINK[i].dot_Q;
    dot_L_cmb[0]        = + dot_L_tmp;
    dot_L_cmb[1]        = - dot_L_tmp;
    Chamber[i][0].dot_V = dot_L_cmb[0]* PI* pow( uLINK[i].D, 2)/ 4;
    Chamber[i][1].dot_V = dot_L_cmb[1]* PI* pow( uLINK[i].D, 2)/ 4;
  }
}

void updateAngularVelocityAll()
{
  for (int i = 1; i < NUM_l; i++)
    if (STEPS == 0)
      uLINK[i].dot_Q = 0;
    else
      uLINK[i].dot_Q = (1.0/ TIMESTEP)*( uLINK[i].Q - uLINK[i].Q_old);
}

void updateAngleAll()
{
  for (int i = 1; i < NUM_l; i++){
    uLINK[i].Q_old = uLINK[i].Q;
    uLINK[i].Q     = dJointGetHingeAngle( joint[i]);
  }
}

void updatePressureAll()
{
  double prsU, prsD;

  updateAngleAll();
  updateAngularVelocityAll();
  updateChamberAll();

  for (int i = 1; i < NUM_l; i++){
    for (int s = 0; s < 2; s++){
      if ( Chamber[i][s].prsTar > Chamber[i][s].prs){
	prsU = Chamber[i][s].prsTar;
	prsD = Chamber[i][s].prs;
      }else{
	prsU = Chamber[i][s].prs;
	prsD = Chamber[i][s].prsTar;
      }
      double dot_m      = getMassFlowRate( prsU, prsD); 
      double dot_Prs    = getDotPressure( Chamber[i][s].prs, dot_m, Chamber[i][s].V, Chamber[i][s].dot_V);
      Chamber[i][s].prs = Chamber[i][s].prs + TIMESTEP* dot_Prs;
    
      if ( Chamber[i][s].prs < Prs_room)
	Chamber[i][s].prs = Prs_room;
      if ( Chamber[i][s].prs > Chamber[i][s].prsTar)
	Chamber[i][s].prs = Chamber[i][s].prsTar;
    }
  }
}

void readRobot()
{
  ifstream ifs("/home/isi/tanaka/MATLAB/Data/jumpHitODE/InitialPosture.dat");
  //  string str, str2;
  string str;

  if ( ifs.fail()){
    cerr << "data import failed." << endl;
  }else{
    for (int i = 0; i < NUM_l; i++) {
      getline( ifs, str);
      sscanf( str.data(), "%d %d %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
	      &(uLINK[i].num_link), &(uLINK[i].num_mother), &(uLINK[i].num_shape), &(uLINK[i].m),  
	      &(uLINK[i].a[0]),    &(uLINK[i].a[1]),    &(uLINK[i].a[2]), 
	      &(uLINK[i].l[0]),    &(uLINK[i].l[1]),    &(uLINK[i].l[2]), 
	      &(uLINK[i].p_j[0]),  &(uLINK[i].p_j[1]),  &(uLINK[i].p_j[2]), 
	      &(uLINK[i].p_c[0]),  &(uLINK[i].p_c[1]),  &(uLINK[i].p_c[2]), 
	      &(uLINK[i].R[0][0]), &(uLINK[i].R[1][0]), &(uLINK[i].R[2][0]),  
	      &(uLINK[i].R[0][1]), &(uLINK[i].R[1][1]), &(uLINK[i].R[2][1]), 
	      &(uLINK[i].R[0][2]), &(uLINK[i].R[1][2]), &(uLINK[i].R[2][2]),
	      &(uLINK[i].RoM[0]),  &(uLINK[i].RoM[1]));
      //cout << "[" << str << "]" << endl;
    }
    // out to terminal 
    if (VIEW > 0)    
      for (int i = 0; i < NUM_l; i++) {
	cout << "#" << uLINK[i].num_link << ", mother: " << uLINK[i].num_mother 
	     << ", #shape: " << uLINK[i].num_shape << endl;
	cout << "joint position:( " 
	     << uLINK[i].p_j[0] << ", "<< uLINK[i].p_j[1] << ", "<< uLINK[i].p_j[2] << ")" << endl;
	cout << "CoM position:( " 
	     << uLINK[i].p_c[0] << ", "<< uLINK[i].p_c[1] << ", "<< uLINK[i].p_c[2] << ")" << endl;
	cout << "Axis vector:( " 
	     << uLINK[i].a[0] << ", "<< uLINK[i].a[1] << ", "<< uLINK[i].a[2] << ")" << endl;
	cout << "R: " << endl;
	cout << "(" << uLINK[i].R[0][0] << ", " << uLINK[i].R[0][1] << ", "<< uLINK[i].R[0][2] << ")" << endl;   
	cout << "(" << uLINK[i].R[1][0] << ", " << uLINK[i].R[1][1] << ", "<< uLINK[i].R[1][2] << ")" << endl;   
	cout << "(" << uLINK[i].R[2][0] << ", " << uLINK[i].R[2][1] << ", "<< uLINK[i].R[2][2] << ")" << endl;
	cout << endl;
      }
    
    // set diameter
    for (int i = 0; i < NUM_l; i++)    
      uLINK[i].D = 40e-3;
    uLINK[7].D = 32e-3;
    uLINK[8].D = 25e-3;
    
    // set moment arm
    for (int i = 0; i < NUM_l; i++)    
      uLINK[i].L_MA = 50e-3;
    uLINK[7].L_MA = 20e-3;
    uLINK[8].L_MA = 20e-3;
    
    // set piston stroke
    for (int i = 0; i < NUM_l; i++)    
      uLINK[i].L_stk = uLINK[i].L_MA*( uLINK[i].RoM[1] - uLINK[i].RoM[0]);

    // target pressure (temporary)
    for (int i = 0; i < NUM_l; i++)    
      for (int s = 0; s < 2; s++)    
	Chamber[i][s].prs = Prs_room;

    for (int i = 0; i < NUM_l; i++)    
      for (int s = 0; s < 2; s++)    
	Chamber[i][s].prsTar = Prs_room;

    Chamber[8][0].prsTar = 5.0* Prs_room;
    
  }
}

void  makeRobot() // make the robot
{
  dMass mass; // mass parameter
  dMatrix3 R;

  for (int i = 0; i < NUM_l; i++) {
    rlink[i].body  = dBodyCreate( world);

    // position, posture
    dBodySetPosition( rlink[i].body, uLINK[i].p_c[0], uLINK[i].p_c[1], uLINK[i].p_c[2] + height);

    for (int n = 0; n < XYZ; n++) 
      for (int m = 0; m < XYZ; m++) 
	R[ (XYZ + 1)*n + m] = dReal( uLINK[i].R[n][m]);
    dBodySetRotation( rlink[i].body, R);

    // mass
    dMassSetZero( &mass);
    if ( uLINK[i].num_shape == N_Box)
      dMassSetBox ( &mass, uLINK[i].m, uLINK[i].l[0], uLINK[i].l[1], uLINK[i].l[2]);
    else if ( uLINK[i].num_shape == N_Sphere)
      dMassSetSphereTotal(  &mass, uLINK[i].m, radius);
    else
      dMassSetCapsuleTotal( &mass, uLINK[i].m, 3, radius, uLINK[i].l[2]);
    dBodySetMass( rlink[i].body, &mass);

    // geometory
    if ( uLINK[i].num_shape == N_Box)
      rlink[i].geom  = dCreateBox( space, uLINK[i].l[0], uLINK[i].l[1], uLINK[i].l[2]);
    else if ( uLINK[i].num_shape == N_Sphere)
      rlink[i].geom  = dCreateSphere( space, uLINK[i].l[0]);
    else
      rlink[i].geom  = dCreateCapsule( space, radius, uLINK[i].l[2]);
    dGeomSetBody( rlink[i].geom, rlink[i].body);
  }

  // joint
  joint[0] = dJointCreateFixed( world, 0); // fixed base trunk
  dJointAttach( joint[0], rlink[0].body, 0);
  dJointSetFixed( joint[0]);

  //for (int j = 0; j < NUM_l; j++) {
  for (int j = 1; j < NUM_l; j++) {
    int m = uLINK[j].num_mother;
    
    joint[j] = dJointCreateHinge( world, 0); // hinge
    
    dJointAttach( joint[j], rlink[j].body, rlink[m].body);
    //dJointSetHingeAnchor( joint[j], uLINK[j].p_j[0], uLINK[j].p_j[1], uLINK[j].p_j[2]);
    dJointSetHingeAnchor( joint[j], uLINK[j].p_j[0], uLINK[j].p_j[1], uLINK[j].p_j[2] + height);
    dJointSetHingeAxis( joint[j], uLINK[j].a[0], uLINK[j].a[1], uLINK[j].a[2]);
    dJointSetHingeParam( joint[j], dParamLoStop, uLINK[j].RoM[0]);
    dJointSetHingeParam( joint[j], dParamHiStop, uLINK[j].RoM[1]);
  }
}

void drawRobot() // draw the robot
{
  for (int i = 0; i < NUM_l; i++ ) // draw capsule
    if ( uLINK[i].num_shape == N_Box)
      dsDrawBox( dBodyGetPosition( rlink[i].body), dBodyGetRotation(rlink[i].body), uLINK[i].l);
    else if ( uLINK[i].num_shape == N_Sphere)
      dsDrawSphere( dBodyGetPosition( rlink[i].body), dBodyGetRotation(rlink[i].body), radius);
    else
      dsDrawCapsule( dBodyGetPosition( rlink[i].body), dBodyGetRotation(rlink[i].body), uLINK[i].l[2], radius);
}


static void nearCallback(void *data, dGeomID o1, dGeomID o2) // collison detection calculation
{
  static const int N = 7; // collision point number
  dContact contact[N];

  int isGround = ((ground == o1) || (ground == o2));

  // no collision detection in case of two body connected by a joint 
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact)) return;

  int n =  dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));
  if (isGround)  {
    for (int i = 0; i < n; i++) {
      contact[i].surface.mode   = dContactBounce | dContactSoftERP |
                                  dContactSoftCFM;
      contact[i].surface.soft_erp   = 0.2;   // ERP of contact point
      contact[i].surface.soft_cfm   = 0.001; // CFM of contact point
      contact[i].surface.mu     = dInfinity; // friction coefficient: infinity
      dJointID c = dJointCreateContact(world,
                                       contactgroup,&contact[i]);
      dJointAttach (c,dGeomGetBody(contact[i].geom.g1),
                      dGeomGetBody(contact[i].geom.g2));
    }
  }
}

void destroyRobot() // destroy the robot
{
  dBodyDestroy( rlink[0].body); // destroy body
  dGeomDestroy( rlink[0].geom); // destroy geometory

  for (int i = 1; i < NUM_l; i++) {
    dJointDestroy( joint[i]);     // destroy joint 
    dBodyDestroy( rlink[i].body); // destroy body
    dGeomDestroy( rlink[i].geom); // destroy geometory
  }
}

void AddTorque()
{
  for (int i = 1; i < NUM_l; i++)
    jointTorque[i] = uLINK[i].L_MA* PI* pow( uLINK[i].D, 2)/ 4 *( Chamber[i][1].prs - Chamber[i][0].prs);
 
  //  if (STEPS < 50)
  for (int i = 1; i < NUM_l; i++)
    dJointAddHingeTorque( joint[i], jointTorque[i]);
}

void getState(){
  //double q[NUM];
  dVector3 res;

  for (int i = 0; i < NUM_l; i++){
    const dReal *p = dBodyGetPosition( rlink[i].body);
    for (int d = 0; d < XYZ; d++)
      Pos_link_data[STEPS][i][d] = p[d];
  }
  for (int i = 1; i < NUM_l; i++){
    dJointGetHingeAnchor( joint[i], res);
    for (int d = 0; d < XYZ; d++)
      Pos_joint_data[STEPS][i][d] = res[d];
      //Pos_joint_data[STEPS][i][d] = dJointGetHingeAngle( joint[i]);
  }
  for (int i = 0; i < NUM_l; i++)
    for (int d = 0; d < NUM_c; d++)
      Prs_data[STEPS][i][d] = Chamber[i][d].prs;
}

//static void restart() // simulation restart
//{
//STEPS    = 0;                        // initialize step number
//destroyRobot();                        // destroy the robot
//dJointGroupDestroy(contactgroup);    // destroy joint group
//contactgroup = dJointGroupCreate(0); // create joint group
//makeRobot();                           // make the robot
//}

static void simLoop(int pause) // simulation loop
{
  if (!pause) {
    //if (VIEW != 1)
    getState();
    
    updatePressureAll();
    AddTorque();
    dSpaceCollide(space,0,&nearCallback);
    dWorldStep( world, TIMESTEP);
    dJointGroupEmpty(contactgroup);
    STEPS++;

    if (VIEW == 1)
      drawRobot(); // draw the robot
  }
}

static void start()
{
  static float xyz[3] = {  0.0, 1.5, 0.5};
  static float hpr[3] = {-90.0, 0.0, 0.0};
  //static float xyz[3] = { 1.0, 1.0, 0.7};
  //static float hpr[3] = {-135, 0.0, 0.0};

  dsSetViewpoint( xyz, hpr); // viewpoint, direction setting
  dsSetSphereQuality(3);     // sphere quality setting
}

void setDrawStuff()        // setup of draw functions
{
  fn.version = DS_VERSION; // version of draw stuff
  fn.start   = &start;     // preprocess: pointer of start function 
  fn.step    = &simLoop;   // pointer of function simLoop
  fn.path_to_textures = "../../drawstuff/textures"; // texture path
}

void getFileName(){
  struct timeval now;
  gettimeofday(&now, NULL);
  struct tm *pnow = localtime(&now.tv_sec);

  int year   = pnow->tm_year + 1900;
  int month  = pnow->tm_mon + 1;
  int day    = pnow->tm_mday;

  int hour   = pnow->tm_hour;
  int minute = pnow->tm_min;
  int second = pnow->tm_sec;
  int usec   = now.tv_usec;
  
  sprintf( filename_o, "../data/%04d%02d%02d/%06d/jump_o_%02d%02d_%02d%06d.dat", 
	   year, month, day, DirName, hour, minute, second, usec);
  sprintf( filename_m, "../data/%04d%02d%02d/%06d/jump_m_%02d%02d_%02d%06d.dat", 
	   year, month, day, DirName, hour, minute, second, usec);
  sprintf( filename_p, "../data/%04d%02d%02d/%06d/jump_p_%02d%02d_%02d%06d.dat", 
	   year, month, day, DirName, hour, minute, second, usec);
  //cout << filename_o << endl;
}

void saveData(){
  ofstream fout_m( filename_m, ios::out);	
  ofstream fout_o( filename_o, ios::out);	
  ofstream fout_p( filename_p, ios::out);	
  
  for(int t=0; t < Num_t; t++){
    fout_m << t << "\t";
    for(int i=0; i < NUM_l; i++)
      for(int d=0; d < XYZ; d++)
	fout_m << Pos_link_data[t][i][d] << "\t";
    for(int i=0; i < NUM_l; i++)  
      for(int d=0; d < XYZ; d++)        
	fout_m << Pos_joint_data[t][i][d] << "\t";
    fout_m << endl;
  }
  for(int i=0; i < NUM_l; i++)
    fout_o << jointTorque[i] << "\t";
  fout_o << endl;

  for(int t=0; t < Num_t; t++){
    fout_p << t << "\t";
    for(int i=0; i < NUM_l; i++)
      for(int d=0; d < NUM_c; d++)
	fout_p << Prs_data[t][i][d] << "\t";
    fout_p << endl;
  }

  fout_m.close();
  fout_o.close();
  fout_p.close();
}

int main (int argc, char *argv[])
{
  
  // variables for filaneme
  if ( argc != (NUM_l + 2)){
    printf("error: input ten values!: nine joint torque and directory name\n");
    return 0;
  }
  for(int i=0; i < NUM_l; i++)
    jointTorque[i] = atof(argv[i + 1]);
  DirName = atoi( argv[ NUM_l + 1]);
  //cout << DirName << endl;

  // initiation
  dInitODE();
  setDrawStuff();

  world        = dWorldCreate();
  space        = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);

  dWorldSetGravity( world, 0,0, -9.8);      // set gravity
  dWorldSetERP( world, 0.9);                // set ERP
  dWorldSetCFM( world, 1e-4);               // set CFM
  ground = dCreatePlane(space, 0, 0, 1, 0); // set ground
  readRobot();                              // set the robot
  makeRobot();                              // set the robot

  // loop
  if ( VIEW == 1)
    dsSimulationLoop ( argc, argv, 640, 480, &fn);
  else
    for (int i = 0; i < Num_t; i++) 
      simLoop(0);

  // termination
  dWorldDestroy (world);
  dCloseODE();

  getFileName();
  //if ( VIEW != 1)
  saveData();

  return 0;
}
