#include <stdio.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"
#include <iostream>
#include <fstream>
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

dWorldID world;             // dynamic simulation world
dSpaceID space;             // contact dection space
dGeomID  ground;            // ground
dJointGroupID contactgroup; // contact group
dReal r = 0.2, m  = 1.0;
dsFunctions fn;             // draw function of drawn stuff

typedef struct { // MyObject structure
  dBodyID body;  // ID number of (rigid) body (for dynamical simulations)
  dGeomID geom;  // ID number of geometry (for collision detection)
  double  l,r,m; // length [m], radius [m], weight [kg]
} MyObject;

static int STEPS = 0; // simulation step number

#define  NUM_l 9       // link number
#define  NUM_j 8       // joint number
MyObject rlink[NUM_l]; // number
dJointID joint[NUM_j]; // joint ID number

dReal Pi = 3.14159;

#define XYZ 3
#define Num_t 1000
//double Angle_data[Num_t][NUM];
//double Position_data[Num_t][NUM][XYZ];
double Pos_link_data [Num_t][NUM_l][XYZ];
double Pos_joint_data[Num_t][NUM_j][XYZ];
double Angle_data[Num_t][NUM_j];

char filename_o[999];
char filename_m[999];

//dReal jointTorque[NUM];
dReal jointTorque[NUM_j];
unsigned int DirName;

double theta[NUM_l] = { Pi, Pi/6.0, 5.0*Pi/6.0, Pi/6.0}; 
//double phi[NUM_l];

void  makeRobot() // make the robot
{
  dMass mass; // mass parameter
  dMatrix3 R;
  dReal length[NUM_l] = {0.20, 0.30, 0.30, 0.50}; 
  dReal weight[NUM_l] = {0.40, 0.80, 0.80, 3.60};
  dReal r[NUM_l]      = {0.02, 0.02, 0.02, 0.02}; // radius
  dReal axis_x = 0; // joint axis x 
  dReal axis_y = 1; // joint axis y 
  dReal axis_z = 0; // joint axis z 

  dReal x[NUM_l], y[NUM_l], z[NUM_l];  
  dReal c_x[NUM_l], c_y[NUM_l], c_z[NUM_l];   

  c_x[0] = 0; c_y[0] = 0; c_z[0] = 1.2* r[0] + 0.5* length[0]* sin(theta[0]);

  for (int i = 1; i < NUM_l; i++) {
    c_x[i] = c_x[i-1] + 1.0* length[i-1]* cos(theta[i-1]); 
    c_y[i] = c_y[i-1];
    c_z[i] = c_z[i-1] + 1.0* length[i-1]* sin(theta[i-1]); 
  }
  for (int i = 0; i < NUM_l; i++) {
    x[i]   = c_x[i] + 0.5* length[i]* cos(theta[i]); 
    y[i]   = c_y[i];
    z[i]   = c_z[i] + 0.5* length[i]* sin(theta[i]); 
  }

  for (int i = 0; i < NUM_l; i++) {
    rlink[i].body  = dBodyCreate( world);
    dBodySetPosition( rlink[i].body,  x[i], y[i], z[i]);
    dRFromAxisAndAngle( R, axis_x, axis_y, axis_z, - theta[i] - Pi/2.0);
    dBodySetRotation( rlink[i].body, R);
    dMassSetZero( &mass);
    dMassSetCapsuleTotal( &mass, weight[i], 3, r[i], length[i]);
    dBodySetMass( rlink[i].body, &mass);
    rlink[i].geom  = dCreateCapsule( space, r[i], length[i]);
    dGeomSetBody( rlink[i].geom, rlink[i].body);
  }

  for (int j = 0; j < NUM_j; j++) {
    joint[j] = dJointCreateHinge( world, 0); // hinge
    dJointAttach( joint[j], rlink[j + 1].body, rlink[j].body);
    //dJointSetHingeAnchor( joint[j], c_x[j], c_y[j], c_z[j]);
    dJointSetHingeAnchor( joint[j], c_x[j+1], c_y[j+1], c_z[j+1]);
    dJointSetHingeAxis( joint[j], axis_x, axis_y, axis_z);
  }

  // define initial angle
  //phi[0] = 0;
  //phi[1] = theta[1] - theta[0] + 2.0*Pi;
  //phi[2] = theta[2] - theta[1];
  //phi[3] = theta[3] - theta[2] + 2.0*Pi;

}


void drawRobot() // draw the robot
{
  dReal r,length;
  for (int i = 0; i < NUM_l; i++ ) { // draw capsule
    dGeomCapsuleGetParams( rlink[i].geom, &r, &length);
    dsDrawCapsule(dBodyGetPosition( rlink[i].body), dBodyGetRotation(rlink[i].body), length, r);
  }
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
    //dJointDestroy(joint[i]);     // destroy joint 
    dBodyDestroy(rlink[i].body); // destroy body
    dGeomDestroy(rlink[i].geom); // destroy geometory
  }
  for (int i = 0; i < NUM_j; i++)
    dJointDestroy(joint[i]);     // destroy joint 
}

void AddTorque()
{
  dJointAddHingeTorque( joint[0], 0);
  for (int i = 0; i < NUM_j; i++)
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
  for (int i = 0; i < NUM_j; i++){
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
    //STEPS++;
    getState();
    AddTorque();
    dSpaceCollide(space,0,&nearCallback);
    dWorldStep(world,0.001);
    dJointGroupEmpty(contactgroup);
    STEPS++;
    //drawRobot(); // draw the robot
  }
}

static void start()
{
  static float xyz[3] = {  0.0, 1.5, 0.5};
  static float hpr[3] = {-90.0, 0.0, 0.0};
  
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
    for(int i=0; i < NUM_j; i++)  
      for(int d=0; d < XYZ; d++)        
	fout_m << Pos_joint_data[t][i][d] << "\t";
    fout_m << endl;
  }
  for(int i=0; i < NUM_j; i++)
    fout_o << jointTorque[i] << "\t";
  fout_o << endl;

  fout_m.close();
  fout_o.close();
}

int main (int argc, char *argv[])
{
  // variables for filaneme
  if ( argc != (NUM_l + 1)){
    printf("error: input 4 values!: three joint torque and directory name\n");
    return 0;
  }
  for(int i=0; i < NUM_j; i++)
    jointTorque[i] = atof(argv[i + 1]);
  DirName = atoi(argv[NUM_j + 1]);

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
  makeRobot();                                // set the robot

  // loop
  dsSimulationLoop (argc, argv, 640, 480, &fn);
  //for (int i = 0; i < Num_t; i++) 
  //simLoop(0);

  // termination
  dWorldDestroy (world);
  dCloseODE();

  getFileName();
  saveData();

  return 0;
}
