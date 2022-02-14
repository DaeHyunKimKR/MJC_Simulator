#include "calculator.h"
#include "trajectory.h"
#include <control.h>
#include "controller.h"
#include "stdio.h"
#include "string.h"
#include <thread>
#include <mutex>
#include <chrono>
#include "../include/mjxmacro.h"
#include "../include/uitools.h"
#include <control.odl>

#define PI 3.141592
#define JDOF 10

mjModel* m = NULL;
mjData* d = NULL;
char filename[1000] = "";


// abstract visualization
mjvScene scn;
mjvCamera cam;
mjvOption vopt;
mjvPerturb pert;
mjvFigure figconstraint;
mjvFigure figcost;
mjvFigure figtimer;
mjvFigure figsize;
mjvFigure figsensor;


// OpenGL rendering and UI
GLFWvidmode vmode;
int windowpos[2];
int windowsize[2];
mjrContext con;
GLFWwindow* window = NULL;
mjuiState uistate;
mjUI ui0, ui1;

double pos_init[4];
double pos_goal[4];
double vel_goal[4];
double pos_des[4];
double vel_des[4];
double q_init[4];
double q_prev[4];

// cpu-sim syncronization point
double cpusync = 0;
double t0 = 0;
double dt = 0;
/*
int Box_Number = getValue();
int Table_Number = getValue2();
*/
mjtNum simsync = 0;
CController Control(4, d->qpos, d->qvel);
CTrajectory Trajectory[2 * JDOF]; //size = joint dof
double q_goal[JDOF];
double qdot_goal[JDOF];
double q_des[JDOF];
double qdot_des[JDOF];

void calculator::start()
{
}