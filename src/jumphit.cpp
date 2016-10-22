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

#define N_Box 0
#define N_Sphere 1
#define N_Cylinder 2

#define  NUM_l 9       // link number
#define  NUM_p 25      // parameter number

#define XYZ 3
#define Num_t 1000

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
  dReal l[3];
  dReal a[3];
  dReal p_j[3]; 
  dReal p_c[3]; 
  double  R[3][3];
} RobotLink;

static int STEPS = 0; // simulation step number

MyObject rlink[NUM_l]; // number
dJointID joint[NUM_l]; // joint ID number
RobotLink uLINK[NUM_l];

dReal Pi = 3.14159;

double Pos_link_data [Num_t][NUM_l][XYZ];
double Pos_joint_data[Num_t][NUM_l][XYZ];
double Angle_data[Num_t][NUM_l];

char filename_o[999];
char filename_m[999];

dReal jointTorque[NUM_l];
unsigned int DirName;

dReal radius = 0.02;

int readRobot()
{
  ifstream ifs("/home/isi/tanaka/MATLAB/Data/jumpHitODE/InitialPosture.dat");
  //  string str, str2;
  string str;

  if ( ifs.fail()){
    cerr << "data import failed." << endl;
    return -1;
  }

  for (int i = 0; i < NUM_l; i++) {
    getline( ifs, str);
    sscanf( str.data(), 
	    "%d %d %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
	    &(uLINK[i].num_link), &(uLINK[i].num_mother), &(uLINK[i].num_shape), &(uLINK[i].m),  
	    &(uLINK[i].a[0]),    &(uLINK[i].a[1]),    &(uLINK[i].a[2]), 
	    &(uLINK[i].l[0]),    &(uLINK[i].l[1]),    &(uLINK[i].l[2]), 
	    &(uLINK[i].p_j[0]),  &(uLINK[i].p_j[1]),  &(uLINK[i].p_j[2]), 
	    &(uLINK[i].p_c[0]),  &(uLINK[i].p_c[1]),  &(uLINK[i].p_c[2]), 
	    &(uLINK[i].R[0][0]), &(uLINK[i].R[1][0]), &(uLINK[i].R[2][0]),  
	    &(uLINK[i].R[0][1]), &(uLINK[i].R[1][1]), &(uLINK[i].R[2][1]), 
	    &(uLINK[i].R[0][2]), &(uLINK[i].R[1][2]), &(uLINK[i].R[2][2]));
    //cout << "[" << str << "]" << endl;
  }

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


  return 1;
}

void  makeRobot() // make the robot
{
  dMass mass; // mass parameter
  dMatrix3 R;

  for (int i = 0; i < NUM_l; i++) {
    rlink[i].body  = dBodyCreate( world);

    // position, posture
    dBodySetPosition( rlink[i].body, uLINK[i].p_c[0], uLINK[i].p_c[1], uLINK[i].p_c[2]);

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
  for (int j = 0; j < NUM_l; j++) {
    int m = uLINK[j].num_mother;
    
    joint[j] = dJointCreateHinge( world, 0); // hinge
    
    dJointAttach( joint[j], rlink[j].body, rlink[m].body);
    dJointSetHingeAnchor( joint[j], uLINK[j].p_j[0], uLINK[j].p_j[1], uLINK[j].p_j[2]);
    dJointSetHingeAxis( joint[j], uLINK[j].a[0], uLINK[j].a[1], uLINK[j].a[2]);
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
  for (int i = 0; i < NUM_l; i++) {
    dJointDestroy(joint[i]);     // destroy joint 
    dBodyDestroy(rlink[i].body); // destroy body
    dGeomDestroy(rlink[i].geom); // destroy geometory
  }
}

void AddTorque()
{
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
  }
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
    if (VIEW != 1)
      getState();
    
    AddTorque();
    dSpaceCollide(space,0,&nearCallback);
    dWorldStep(world,0.001);
    dJointGroupEmpty(contactgroup);
    STEPS++;

    if (VIEW == 1)
      drawRobot(); // draw the robot
  }
}

static void start()
{
  //static float xyz[3] = {  0.0, 1.5, 0.5};
  //static float hpr[3] = {-90.0, 0.0, 0.0};
  static float xyz[3] = { 1.0, 1.0, 0.7};
  static float hpr[3] = {-135, 0.0, 0.0};

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
  
  sprintf( filename_o, "../data/%04d%02d%02d/%06d/jump_o_%02d_%02d_%02d_%06d_jump.dat", 
	   year, month, day, DirName, hour, minute, second, usec);
  sprintf( filename_m, "../data/%04d%02d%02d/%06d/jump_m_%02d_%02d_%02d_%06d_jump.dat", 
	   year, month, day, DirName, hour, minute, second, usec);
  //cout << filename_o << endl;
}

void saveData(){
  ofstream fout_m( filename_m, ios::out);	
  ofstream fout_o( filename_o, ios::out);	
  
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

  fout_m.close();
  fout_o.close();
}

int main (int argc, char *argv[])
{
  // variables for filaneme
  if ( argc != (NUM_l + 1)){
    printf("error: input 9 values!: eight joint torque and directory name\n");
    return 0;
  }
  for(int i=0; i < NUM_l; i++)
    jointTorque[i] = atof(argv[i + 1]);
  DirName = atoi( argv[ NUM_l]);

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

  if ( VIEW != 1){
    getFileName();
    saveData();
  }

  return 0;
}
