/*
	Lander Control simulation.

	Updated by F. Estrada for CSC C85, Oct. 2013
	Updated by Per Parker, Sep. 2015

	Learning goals:

	- To explore the implementation of control software
	  that is robust to malfunctions/failures.

	The exercise:

	- The program loads a terrain map from a .ppm file.
	  the map shows a red platform which is the location
	  a landing module should arrive at.
	- The control software has to navigate the lander
	  to this location and deposit the lander on the
	  ground considering:

	  * Maximum vertical speed should be less than 10 m/s at touchdown
	  * Maximum landing angle should be less than 15 degrees w.r.t vertical

	- Of course, touching any part of the terrain except
	  for the landing platform will result in destruction
	  of the lander

	This has been made into many videogames. The oldest one
	I know of being a C64 game called 1985 The Day After.
        There are older ones! (for bonus credit, find the oldest
        one and send me a description/picture plus info about the
        platform it ran on!)

	Your task:

	- These are the 'sensors' you have available to control
          the lander.

	  Velocity_X();  - Gives you the lander's horizontal velocity
	  Velocity_Y();	 - Gives you the lander's vertical velocity
	  Position_X();  - Gives you the lander's horizontal position (0 to 1024)
	  Position Y();  - Gives you the lander's vertical position (0 to 1024)

          Angle();	 - Gives the lander's angle w.r.t. vertical in DEGREES (upside-down = 180 degrees)

	  SONAR_DIST[];  - Array with distances obtained by sonar. Index corresponds
                           to angle w.r.t. vertical direction measured clockwise, so that
                           SONAR_DIST[0] is distance at 0 degrees (pointing upward)
                           SONAR_DIST[1] is distance at 10 degrees from vertical
                           SONAR_DIST[2] is distance at 20 degrees from vertical
                           .
                           .
                           .
                           SONAR_DIST[35] is distance at 350 degrees from vertical

                           if distance is '-1' there is no valid reading. Note that updating
                           the sonar readings takes time! Readings remain constant between
                           sonar updates.

          RangeDist();   - Uses a laser range-finder to accurately measure the distance to ground
                           in the direction of the lander's main thruster.
                           The laser range finder never fails (probably was designed and
                           built by PacoNetics Inc.)

          Note: All sensors are NOISY. This makes your life more interesting.

	- Variables accessible to your 'in flight' computer

	  MT_OK		- Boolean, if 1 indicates the main thruster is working properly
	  RT_OK		- Boolean, if 1 indicates the right thruster is working properly
	  LT_OK		- Boolean, if 1 indicates thr left thruster is working properly
          PLAT_X	- X position of the landing platform
          PLAY_Y        - Y position of the landing platform

	- Control of the lander is via the following functions
          (which are noisy!)

	  Main_Thruster(double power);   - Sets main thurster power in [0 1], 0 is off
	  Left_Thruster(double power);	 - Sets left thruster power in [0 1]
	  Right_Thruster(double power);  - Sets right thruster power in [0 1]
	  Rotate(double angle);	 	 - Rotates module 'angle' degrees clockwise
					   (ccw if angle is negative) from current
                                           orientation (i.e. rotation is not w.r.t.
                                           a fixed reference direction).

 					   Note that rotation takes time!


	- Important constants

	  G_ACCEL = 8.87	- Gravitational acceleration on Venus
	  MT_ACCEL = 35.0	- Max acceleration provided by the main thruster
	  RT_ACCEL = 25.0	- Max acceleration provided by right thruster
	  LT_ACCEL = 25.0	- Max acceleration provided by left thruster
          MAX_ROT_RATE = .075    - Maximum rate of rotation (in radians) per unit time

	- Functions you need to analyze and possibly change

	  * The Lander_Control(); function, which determines where the lander should
	    go next and calls control functions
          * The Safety_Override(); function, which determines whether the lander is
            in danger of crashing, and calls control functions to prevent this.

	- You *can* add your own helper functions (e.g. write a robust thruster
	  handler, or your own robust sensor functions - of course, these must
	  use the noisy and possibly faulty ones!).

	- The rest is a black box... life sometimes is like that.

        - Program usage: The program is designed to simulate different failure
                         scenarios. Mode '1' allows for failures in the
                         controls. Mode '2' allows for failures of both
                         controls and sensors. There is also a 'custom' mode
                         that allows you to test your code against specific
                         component failures.

			 Initial lander position, orientation, and velocity are
                         randomized.

	  * The code I am providing will land the module assuming nothing goes wrong
          with the sensors and/or controls, both for the 'easy.ppm' and 'hard.ppm'
          maps.

	  * Failure modes: 0 - Nothing ever fails, life is simple
			   1 - Controls can fail, sensors are always reliable
			   2 - Both controls and sensors can fail (and do!)
			   3 - Selectable failure mode, remaining arguments determine
                               failing component(s):
                               1 - Main thruster
                               2 - Left Thruster
                               3 - Right Thruster
                               4 - Horizontal velocity sensor
                               5 - Vertical velocity sensor
                               6 - Horizontal position sensor
                               7 - Vertical position sensor
                               8 - Angle sensor
                               9 - Sonar

        e.g.

             Lander_Control easy.ppm 3 1 5 8

             Launches the program on the 'easy.ppm' map, and disables the main thruster,
             vertical velocity sensor, and angle sensor.

		* Note - while running. Pressing 'q' on the keyboard terminates the 
			program.

        * Be sure to complete the attached REPORT.TXT and submit the report as well as
          your code by email. Subject should be 'C85 Safe Landings, name_of_your_team'

	Have fun! try not to crash too many landers, they are expensive!

  	Credits: Lander image and rocky texture provided by NASA
		 Per Parker spent some time making sure you will have fun! thanks Per!
*/

/*
  Standard C libraries
*/
#include <math.h>
#include <stdio.h>

#include "Lander_Control.h"


// Declare constants and functions.
const int OK_ALL = 0;
const int LT_FAIL_ONLY = 1;
const int RT_FAIL_ONLY = 2;
const int LT_OK_ONLY = 3;
const int RT_OK_ONLY = 4;
static int Control_Fail_Mode = OK_ALL;
const int SensorSampleCount = 100;
const int ErrorSampleCount = 20;
const double PosNoiseThreshold = 40;
const double VelNoiseThreshold = 1;
typedef double (*SampleFunc)();
double GetMeanAndError(SampleFunc sampelFunc, double *stdErr, const char *name);
double Position_X_robust(void);
double Position_Y_robust(void);
double Velocity_X_robust(void);
double Velocity_Y_robust(void);
double Angle_robust(void);
bool NearLanding(double pos_x, double pos_y);
bool Rotate_robust(double angle, double pos_x, double pos_y);
void Thruster_robust(double xPower, double yPower, const char *name);


double GetMeanAndError(SampleFunc sampelFunc, double *stdErr, const char *name)
{
 double sum = 0;
 for (int i = 0; i < SensorSampleCount; i++)
 {
  double val = sampelFunc();
  sum += val;
 }
 double mean = sum / SensorSampleCount;
 double sumerr = 0;
 for (int i = 0; i < ErrorSampleCount; i++)
 {
  double val = sampelFunc();
  sumerr += (val - mean) * (val - mean);
 }
 *stdErr = sqrt(sumerr / (SensorSampleCount - 1));
 printf("%s: mean=%.2f, stdErr=%.2f\n", name, mean, *stdErr);
 return mean;
}

double Position_X_robust(void)
{
 double stdErr;
 double mean = GetMeanAndError(Position_X, &stdErr, "Position_X");
 return mean;
}

double Position_Y_robust(void)
{
 double stdErr;
 double mean = GetMeanAndError(Position_Y, &stdErr, "Position_Y");
 return mean;
}

double Velocity_X_robust(void)
{
 double stdErr;
 double mean = GetMeanAndError(Velocity_X, &stdErr, "Velocity_X");
 return mean;
}

double Velocity_Y_robust(void)
{
 double stdErr;
 double mean = GetMeanAndError(Velocity_Y, &stdErr, "Velocity_Y");
 return mean;
}

double Angle_robust(void)
{
 double sumsin = 0;
 double sumcos = 0;
 for (int i=0;i<SensorSampleCount;i++)
 {
  double angle = Angle();
  sumsin += sin(angle*M_PI/180);
  sumcos += cos(angle*M_PI/180);
 }
 double meansin = sumsin / SensorSampleCount;
 double meancos = sumcos / SensorSampleCount;
 double sumsinerr = 0;
 double sumcoserr = 0;
 for (int i=0;i<ErrorSampleCount;i++)
 {
  double angle = Angle();
  sumsinerr += (sin(angle*M_PI/180) - meansin) * (sin(angle*M_PI/180) - meansin);
  sumcoserr += (cos(angle*M_PI/180) - meancos) * (cos(angle*M_PI/180) - meancos);
 }
 double result = atan2(sumsin, sumcos)*180/M_PI;
 if (result < 0) result += 360;
 if (result >= 360) result -= 360;
 return result;
}


bool NearLanding(double pos_x, double pos_y)
{
 printf("PLAT_X=%.2f,PLAT_Y=%.2f,pos_x=%.2f,pos_y=%.2f\n", PLAT_X, PLAT_Y, pos_x, pos_y);
 return fabs(PLAT_X - pos_x) < 50 && fabs(PLAT_Y - pos_y) < 50;
}

bool Rotate_robust(double angle, double pos_x, double pos_y)
{
 bool prepareLanding = NearLanding(pos_x, pos_y);
 if ((MT_OK && RT_OK && LT_OK) || prepareLanding)
 {
  Control_Fail_Mode = OK_ALL;
  if (angle > 1 && angle < 359)
  {
   if (angle >= 180) Rotate(360 - angle);
   else Rotate(-angle);
   return true;
  }
  return false;
 }
 else if (MT_OK && !LT_OK && RT_OK)
 {
  Control_Fail_Mode = LT_FAIL_ONLY;
  if (angle < 44 || (angle > 46 && angle <= 225)) Rotate(45-angle);
  else if (angle > 225) Rotate(405-angle);
  else return false;
  return true;
 }
 else if (MT_OK && LT_OK && !RT_OK)
 {
  Control_Fail_Mode = RT_FAIL_ONLY;
  if (angle > 316 || (angle > 135 && angle < 314)) Rotate(315-angle);
  else if (angle <= 135) Rotate(-45-angle);
  else return false;
  return true;
 }
 return false;
}

void Thruster_robust(double xPower, double yPower, const char *name)
{
 printf("%s: xPower=%.2f,yPower=%.2f\n", name, xPower, yPower);
 double ratioM = 1.0;
 double ratioL = 1.0;
 double ratioR = 1.0;
 if (yPower <= 0)
 {
  ratioM = G_ACCEL / MT_ACCEL / sqrt(0.5);
  ratioL = G_ACCEL / LT_ACCEL / sqrt(0.5);
  ratioR = G_ACCEL / RT_ACCEL / sqrt(0.5);
 }
 switch (Control_Fail_Mode)
 {
  case LT_FAIL_ONLY:
  {
   if (yPower > 0) {
    ratioM = 1.0 * RT_ACCEL / MT_ACCEL;
   }
   if (xPower > 0) {
    Main_Thruster(xPower * ratioM);
    Right_Thruster(0);
   } else if (xPower < 0) {
    Right_Thruster(-xPower * ratioR);
    Main_Thruster(0);
   } else if (yPower > 0) {
    Main_Thruster(yPower / (MT_ACCEL+RT_ACCEL) * RT_ACCEL);
    Right_Thruster(yPower / (MT_ACCEL+RT_ACCEL) * MT_ACCEL);
   } else {
    Main_Thruster(0);
    Right_Thruster(0);
   }
   return;
  }
  case RT_FAIL_ONLY:
  {
   if (yPower > 0) {
    ratioM = 1.0 * LT_ACCEL / MT_ACCEL;
   }
   if (xPower > 0) {
    Main_Thruster(0);
    Left_Thruster(xPower * ratioL);
   } else if (xPower < 0) {
    Left_Thruster(0);
    Main_Thruster(-xPower * ratioM);
   } else if (yPower > 0) {
    Main_Thruster(yPower / (MT_ACCEL+LT_ACCEL) * LT_ACCEL);
    Left_Thruster(yPower / (MT_ACCEL+LT_ACCEL) * MT_ACCEL);
   } else {
    Main_Thruster(0);
    Left_Thruster(0);
   }
   return;
  }
  default:
  {
   if (xPower > 0) {
    Left_Thruster(xPower);
    Right_Thruster(0);
   } else {
    Left_Thruster(0);
    Right_Thruster(-xPower);
   }
   if (yPower > 0) Main_Thruster(yPower);
   else Main_Thruster(0);
   return;
  }
 }
}

void Lander_Control(void)
{
 /*
   This is the main control function for the lander. It attempts
   to bring the ship to the location of the landing platform
   keeping landing parameters within the acceptable limits.

   How it works:

   - First, if the lander is rotated away from zero-degree angle,
     rotate lander back onto zero degrees.
   - Determine the horizontal distance between the lander and
     the platform, fire horizontal thrusters appropriately
     to change the horizontal velocity so as to decrease this
     distance
   - Determine the vertical distance to landing platform, and
     allow the lander to descend while keeping the vertical
     speed within acceptable bounds. Make sure that the lander
     will not hit the ground before it is over the platform!

   As noted above, this function assumes everything is working
   fine.
*/

/*************************************************
 TO DO: Modify this function so that the ship safely
        reaches the platform even if components and
        sensors fail!

        Note that sensors are noisy, even when
        working properly.

        Finally, YOU SHOULD provide your own
        functions to provide sensor readings,
        these functions should work even when the
        sensors are faulty.

        For example: Write a function Velocity_X_robust()
        which returns the module's horizontal velocity.
        It should determine whether the velocity
        sensor readings are accurate, and if not,
        use some alternate method to determine the
        horizontal velocity of the lander.

        NOTE: Your robust sensor functions can only
        use the available sensor functions and control
        functions!
	DO NOT WRITE SENSOR FUNCTIONS THAT DIRECTLY
        ACCESS THE SIMULATION STATE. That's cheating,
        I'll give you zero.
**************************************************/

 double VXlim;
 double VYlim;

 // Set velocity limits depending on distance to platform.
 // If the module is far from the platform allow it to
 // move faster, decrease speed limits as the module
 // approaches landing. You may need to be more conservative
 // with velocity limits when things fail.
 printf("X=%.2f,Y=%.2f,Vx=%.2f,Vy=%.2f,Angle=%.2f\n",
  Position_X(), Position_Y(), Velocity_X(), Velocity_Y(), Angle());

 double pos_x = Position_X_robust();
 double pos_y = Position_Y_robust();
 double vel_x = Velocity_X_robust();
 double vel_y = Velocity_Y_robust();
 double angle = Angle_robust();

 printf("Robust: X=%.2f,Y=%.2f,Vx=%.2f,Vy=%.2f,Angle=%.2f\n",
  pos_x, pos_y, vel_x, vel_y, angle);

 if (fabs(pos_x-PLAT_X)>200) VXlim=25;
 else if (fabs(pos_x-PLAT_X)>100) VXlim=15;
 else VXlim=5;

 if (PLAT_Y-pos_y>200) VYlim=-20;
 else if (PLAT_Y-pos_y>100) VYlim=-10;  // These are negative because they
 else VYlim=-4;				       // limit descent velocity

 // Ensure we will be OVER the platform when we land
 if (fabs(PLAT_X-pos_x)/fabs(vel_x)>1.25*fabs(PLAT_Y-pos_y)/fabs(vel_y)) VYlim=0;

 // IMPORTANT NOTE: The code below assumes all components working
 // properly. IT MAY OR MAY NOT BE USEFUL TO YOU when components
 // fail. More likely, you will need a set of case-based code
 // chunks, each of which works under particular failure conditions.

 // Check for rotation away from zero degrees - Rotate first,
 // use thrusters only when not rotating to avoid adding
 // velocity components along the rotation directions
 // Note that only the latest Rotate() command has any
 // effect, i.e. the rotation angle does not accumulate
 // for successive calls.

 if (Rotate_robust(angle, pos_x, pos_y))
 {
  return;
 }

 // Module is oriented properly, check for horizontal position
 // and set thrusters appropriately.
 double xPower = 0;
 double yPower = 0;
 if (pos_x>PLAT_X)
 {
  // Lander is to the LEFT of the landing platform, use Right thrusters to move
  // lander to the left.
  if (vel_x>(-VXlim)) xPower = -((VXlim+fmin(0,vel_x))/VXlim);
  else
  {
   // Exceeded velocity limit, brake
   // xPower = (fabs(VXlim-vel_x));
   xPower = 1.0;
  }
 }
 else
 {
  // Lander is to the RIGHT of the landing platform, opposite from above
  if (vel_x<VXlim) xPower = ((VXlim-fmax(0,vel_x))/VXlim);
  else
  {
   // xPower = -(fabs(VXlim-vel_x));
   xPower = -1.0;
  }
 }

 // Vertical adjustments. Basically, keep the module below the limit for
 // vertical velocity and allow for continuous descent. We trust
 // Safety_Override() to save us from crashing with the ground.
 if (vel_y<VYlim) yPower = (1.0);
 else yPower = (0);
 Thruster_robust(xPower, yPower, "Lander_Control");
}

void Safety_Override(void)
{
 /*
   This function is intended to keep the lander from
   crashing. It checks the sonar distance array,
   if the distance to nearby solid surfaces and
   uses thrusters to maintain a safe distance from
   the ground unless the ground happens to be the
   landing platform.

   Additionally, it enforces a maximum speed limit
   which when breached triggers an emergency brake
   operation.
 */

/**************************************************
 TO DO: Modify this function so that it can do its
        work even if components or sensors
        fail
**************************************************/

/**************************************************
  How this works:
  Check the sonar readings, for each sonar
  reading that is below a minimum safety threshold
  AND in the general direction of motion AND
  not corresponding to the landing platform,
  carry out speed corrections using the thrusters
**************************************************/

 double DistLimit;
 double Vmag;
 double dmin;

 double pos_x = Position_X_robust();
 double pos_y = Position_Y_robust();
 double vel_x = Velocity_X_robust();
 double vel_y = Velocity_Y_robust();
 double angle = Angle_robust();
 double xPower = 0;
 double yPower = 0;

 // Establish distance threshold based on lander
 // speed (we need more time to rectify direction
 // at high speed)
 Vmag=vel_x*vel_x;
 Vmag+=vel_y*vel_y;

 DistLimit=fmax(75,Vmag);

 // If we're close to the landing platform, disable
 // safety override (close to the landing platform
 // the Control_Policy() should be trusted to
 // safely land the craft)
 if (fabs(PLAT_X-pos_x)<150&&fabs(PLAT_Y-pos_y)<150) return;

 // Determine the closest surfaces in the direction
 // of motion. This is done by checking the sonar
 // array in the quadrant corresponding to the
 // ship's motion direction to find the entry
 // with the smallest registered distance
 printf("SONAR_DIST: ");
 for (int i = 0; i < 36; i++)
 {
  printf("%.2f ", SONAR_DIST[i]);
 }
 printf("\n");
 
 // Horizontal direction.
 dmin=1000000;
 if (vel_x>0)
 {
  for (int i=5;i<14;i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
 }
 else
 {
  for (int i=22;i<32;i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
 }
 // Determine whether we're too close for comfort. There is a reason
 // to have this distance limit modulated by horizontal speed...
 // what is it?
 double threshold = DistLimit*fmax(.25,fmin(fabs(vel_x)/5.0,1));
 printf("DistLimit=%.2f,dmin=%.2f,vel_x=%.2f,threshold=%.2f\n",
  DistLimit, dmin, vel_x, threshold);
 if (dmin<DistLimit*fmax(.25,fmin(fabs(vel_x)/5.0,1)))
 { // Too close to a surface in the horizontal direction
  if (Rotate_robust(angle, pos_x, pos_y))
  {
   return;
  }

  if (vel_x>0){
   xPower = -1.0;
  }
  else
  {
   xPower = 1.0;
  }
 }

 // Vertical direction
 dmin=1000000;
 if (vel_y>5)      // Mind this! there is a reason for it...
 {
  for (int i=0; i<5; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
  for (int i=32; i<36; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
 }
 else
 {
  for (int i=14; i<22; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
 }
 if (dmin<DistLimit)   // Too close to a surface in the horizontal direction
 {
  if (Rotate_robust(angle, pos_x, pos_y))
  {
   return;
  }
  if (vel_y>2.0){
   yPower = 0;
  }
  else
  {
   yPower = 1.0;
  }
 }

 if (xPower != 0 || yPower != 0)
 {
  Thruster_robust(xPower, yPower, "Safety_Override");
 }
}
