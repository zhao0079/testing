/*=====================================================================

 PIXHAWK Micro Air Vehicle Flying Robotics Toolkit

 (c) 2009, 2010 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

 This file is part of the PIXHAWK project

 PIXHAWK is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 PIXHAWK is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with PIXHAWK. If not, see <http://www.gnu.org/licenses/>.

 ======================================================================*/

/**
* @file
* @brief Main program execution
*   @author Martin Rutschmann
*   @author Laurens MacKay
*   @author Lorenz Meier
*   @author Christian Dobler
*
*  Please refer to
*
*  http://pixhawk.ethz.ch/software/imu/start
*  and
*  http://pixhawk.ethz.ch/api/imu_autopilot/
*
*  For the full documentation of this code.
*
*/

#include "conf.h" /* Defines the system type in conf/user_conf.h */
#include "mavlink_types.h"
mavlink_system_t mavlink_system;

// All mainloops are in folder main/mainloop_xx.h
// Please note that through the default -O2 optimization
// only executed code is compiled into the binary.
// It is therefore sufficient to have the mainloop executions
// guarded by the define switch, the C-files can be safely
// compiled along.

#include "mainloop_quadrotor.h"

/**
* @brief Main function
*
* The main function first calls the main_init() function and then loops through
* the main_loop() function until shutdown.
*/
int main(void)
{
	main_loop_quadrotor();
	return 0;
}
