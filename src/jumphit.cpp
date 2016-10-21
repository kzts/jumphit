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

#define VIEW 1

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

static int STEPS = 0; // simulation step number

#define  NUM_l 9       // link number
MyObject rlink[NUM_l]; // number
dJointID joint[NUM_l]; // joint ID number

dReal Pi = 3.14159;

#define XYZ 3
#define Num_t 1000
double Pos_link_data [Num_t][NUM_l][XYZ];
double Pos_joint_data[Num_t][NUM_l][XYZ];
double Angle_data[Num_t][NUM_l];

char filename_o[999];
char filename_m[999];

dReal jointTorque[NUM_l];
unsigned int DirName;

dReal L_Trunk[XYZ]  = { 0.15, 0.2, 0.426};
dReal radius[NUM_l] = { 0.10,  0.01, 0.02, 0.02,  0.01, 0.02, 0.02,  0.01, 0.02};
dReal length[NUM_l] = { 0.30,  0.01, 0.28, 0.30,  0.01, 0.28, 0.30,  0.01, 0.28};
dReal weight[NUM_l] = { 5.00,  0.05, 0.60, 0.60,  0.05, 0.60, 0.60,  0.05, 0.40};

// axis position in the world coordination
dReal a[NUM_l][XYZ] = { 
  { + 0.000, + 0.000, + 0.000},
  { + 0.000, + 0.090, + 0.000},
  { - 0.028, + 0.144, + 0.000},
  { - 0.054, + 0.144, - 0.288},  
  { + 0.000, - 0.090, + 0.000},
  { - 0.028, - 0.144, + 0.000},
  { - 0.054, - 0.144, - 0.288},
  { + 0.021, + 0.138, + 0.426},
  { + 0.021, + 0.184, + 0.426}};  

// link CoG position in the world coordination
dReal c[NUM_l][XYZ] = {
  { + 0.000, + 0.000, + 0.218},  
  { - 0.028, + 0.144, + 0.000},  
  { - 0.082, + 0.172, - 0.144},  
  { - 0.054, + 0.144, - 0.444},
  { - 0.028, - 0.144, + 0.000},  
  { - 0.082, - 0.172, - 0.144},  
  { - 0.054, - 0.144, - 0.444},
  { + 0.021, + 0.138, + 0.426},
  { + 0.021, + 0.184, + 0.284}};

dReal axis_pitch[XYZ] = {0,1,0};
dReal axis_roll[XYZ]  = {1,0,0};
dReal height_hip = 1.0;

void  makeRobot() // make the robot
{
  dMass mass; // mass parameter
  dMatrix3 R;

  for (int i = 0; i < NUM_l; i++) {
    rlink[i].body  = dBodyCreate( world);

    // position, posture
    dBodySetPosition( rlink[i].body, c[i][0], c[i][1], c[i][2] + height_hip);

    //dRFromAxisAndAngle( R, axis_x, axis_y, axis_z, - theta[i] - Pi/2.0)
    if ( i == 0)
      dRFromAxisAndAngle( R, axis_pitch[0], axis_pitch[1], axis_pitch[2], - Pi/3.0);
    else if ( i == 1 || i == 4 || i == 7)
      dRFromAxisAndAngle( R, axis_roll[0], axis_roll[1], axis_roll[2], 0);
    else
      dRFromAxisAndAngle( R, axis_pitch[0], axis_pitch[1], axis_pitch[2], 0);
    dBodySetRotation( rlink[i].body, R);

    // mass
    dMassSetZero( &mass);
    if ( i == 0)
      dMassSetBox ( &mass, weight[i], L_Trunk[0], L_Trunk[1], L_Trunk[2]);
    else if ( i == 1 || i == 4 || i == 7)
      dMassSetSphereTotal(  &mass, weight[i], radius[i]);
    else
      dMassSetCapsuleTotal( &mass, weight[i], 3, radius[i], length[i]);
    dBodySetMass( rlink[i].body, &mass);

    // geometory
    if ( i == 0)
      rlink[i].geom  = dCreateBox( space, L_Trunk[0], L_Trunk[1], L_Trunk[2]);
    else if ( i == 1 || i == 4 || i == 7)
      rlink[i].geom  = dCreateSphere( space, radius[i]);
    else
      rlink[i].geom  = dCreateCapsule( space, radius[i], length[i]);
    dGeomSetBody( rlink[i].geom, rlink[i].body);
  }

  // joint
  joint[0] = dJointCreateFixed( world, 0); // fixed base trunke
  dJointAttach( joint[0], rlink[0].body, 0);
  dJointSetFixed( joint[0]);
  for (int j = 1; j < NUM_l; j++) {
    joint[j] = dJointCreateHinge( world, 0); // hinge

    if ( j == 4 || j == 7)
      dJointAttach( joint[j], rlink[j].body, rlink[0].body);
    else
      dJointAttach( joint[j], rlink[j].body, rlink[j - 1].body);

    dJointSetHingeAnchor( joint[j], a[j][0], a[j][1], a[j][2] + height_hip);
    
    if ( j == 1 || j == 4)
      dJointSetHingeAxis( joint[j], axis_roll[0], axis_roll[1], axis_roll[2]);
    else
      dJointSetHingeAxis( joint[j], axis_pitch[0], axis_pitch[1], axis_pitch[2]);
  }

}


void drawRobot() // draw the robot
{
  for (int i = 0; i < NUM_l; i++ ) // draw capsule
    if ( i == 0)
      dsDrawBox( dBodyGetPosition( rlink[i].body), dBodyGetRotation(rlink[i].body), L_Trunk);
    else if ( i == 1 || i == 4 || i == 7)
      dsDrawSphere( dBodyGetPosition( rlink[i].body), dBodyGetRotation(rlink[i].body), radius[i]);
    else
      dsDrawCapsule( dBodyGetPosition( rlink[i].body), dBodyGetRotation(rlink[i].body), length[i], radius[i]);
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
  for (int i = 0; i < NUM_l; i++)
    dJointDestroy(joint[i]);     // destroy joint 
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
    //STEPS++;
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
