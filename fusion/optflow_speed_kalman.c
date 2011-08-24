/*
 * optflow_speed_kalman.c
 *
 *  Created on: 23.08.2010
 *      Author: Petri Tanskanen
 *      		Gim Hee Lee
 *      		Laurens Mackay
 */
#include "optflow_speed_kalman.h"
#include "kalman.h"

#include "debug.h"
#include "sensors.h"
#include "math.h"
#include "transformation.h"

//#define VELOCITY_HOLD 0.999f
//#define ACCELERATION_HOLD 0.99f
#define VELOCITY_HOLD 0.99f
#define ACCELERATION_HOLD 1.0f

#define VEL_KF_TIME_STEP_X (1.0f / 50.0f)
#define VEL_KF_TIME_STEP_Y (1.0f / 50.0f)

#define PI  3.1415926535897932384626433832795029

//kalman_t optflow_speed_kalman_x;
//kalman_t optflow_speed_kalman_y;

double ax, bx, cx;
double ay, by, cy;
double vx, vy, x, y, pvx, pvy;
double Qx, Rx;
double Qy, Ry;
double scale;


void optflow_speed_kalman_init(void)
{
   // kalman filter parameters
   ax = PARAM_KAL_VEL_AX;
   bx = PARAM_KAL_VEL_BX;
   cx = 1.0;

   ay = PARAM_KAL_VEL_AY;
   by = PARAM_KAL_VEL_BY;
   cy = 1.0;

   // assumes initial state is 0
   vx = 0.0;
   vy = 0.0;
   x = 0.0;
   y = 0.0;

   // assumes initial error covariance is 0
   pvx = 0.0;
   pvy = 0.0;

   // noise parameters (hard-coded)
   Qx = 0.002;
   Rx = 0.1;

   Qy = 0.005;
   Ry = 0.2;

   // set the optical flow scale (assuming linear relation)
	//   scale = -0.0714;
	//   scale = 0.04;
	scale = 0.0008 / VEL_KF_TIME_STEP_X;
}

void optflow_speed_kalman(void)
{

	static double viconPre = 0.0;
	static bool flag=true;
	float_vect3 debug;
//    double vx_ = 0.0;


	//Transform accelerometer used in all directions
	float_vect3 acc_nav;
	body2navi(&global_data.accel_si, &global_data.attitude, &acc_nav);

	// transform optical flow into global frame
	float_vect3 flow, flowQuad, flowWorld;//, flowQuadUncorr, flowWorldUncorr;
	flow.x = global_data.optflow.x;
	flow.y = global_data.optflow.y;
	flow.z = 0.0;

	turn_xy_plane(&flow, PI, &flowQuad);
	flowQuad.x = flowQuad.x*scale - global_data.gyros_si.y;
	flowQuad.y = flowQuad.y*scale + global_data.gyros_si.x;

	body2navi(&flowQuad, &global_data.attitude, &flowWorld);

//	turn_xy_plane(&flow, PI, &flowQuadUncorr);
//	body2navi(&flowQuadUncorr, &global_data.attitude, &flowWorldUncorr);

	if(global_data.state.fly == FLY_FLYING)
	{
	   // initializes x and y to global position
	   if(flag)
	   {
		   x = global_data.position.x;
		   y = global_data.position.y;
		   flag = false;
	   }

	   //---------------------------------------------------
	   // Vx Kalman Filter
	   // prediction
	   double vx_ = ax*vx + bx*global_data.attitude.y;
	   double pvx_ = ax*pvx + Qx;

	   // do an update only if optical flow is good
	   if(global_data.optflow.z > 10.0)
	   {
		   // kalman gain
		   double Kx = pvx_*ax/(ax*pvx_*ax + Rx);

		   // update step
		   double xflow = global_data.optflow.x*global_data.position.z*scale;
		   vx = vx_ + Kx*(xflow - cx*vx_);
		   pvx = (1.0 - Kx*cx)*pvx_;
	   }
	   // otherwise take only the prediction
	   else
	   {
		   vx = vx_;
		   pvx = pvx_;
	   }

	   // assign readings from Kalman Filter
	   //global_data.velocity.x = vx;
	   //global_data.position.x += vx*VEL_KF_TIME_STEP_X;

	   //---------------------------------------------------
	   // Vy Kalman Filter
	   // prediction
	   double vy_ = ay*vy + by*global_data.attitude.x;
	   double pvy_ = ay*pvy + Qy;

	   // do an update only if optical flow is good
	   if(global_data.optflow.z > 10.0)
	   {
		   // kalman gain
		   double Ky = pvy_*ay/(ay*pvy_*ay + Ry);

		   // update step
		   double yflow = global_data.optflow.y*global_data.position.z*scale;
		   vy = vy_ + Ky*(yflow - cy*vy_);
		   pvy = (1.0 - Ky*cy)*pvy_;
	   }
	   // otherwise take only the prediction
	   else
	   {
		   vy = vy_;
		   pvy = pvy_;
	   }

	   // assign readings from Kalman Filter
	   //global_data.velocity.y = vy;
	   //global_data.position.y += vy*VEL_KF_TIME_STEP_Y;
	}



	double xvel = (global_data.position.x - viconPre)/VEL_KF_TIME_STEP_X;
	viconPre = global_data.position.x;
	debug.x = -global_data.position.z*flowWorld.x;
	debug.y = -global_data.position.z*flowWorld.y;
	debug.z = xvel;
	debug_vect("KALMAN", debug);
}
