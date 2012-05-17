/********************************************************************
 *                    COPYRIGHT PARROT 2010
 ********************************************************************
 *       PARROT - A.R.Drone SDK Windows Client Example
 *-----------------------------------------------------------------*/
/**
 * @file navdata.c 
 * @brief Navdata handling code
 *
 * @author sylvain.gaeremynck@parrot.com
 * @date 2009/07/01
 *
 * @author Stephane Piskorski <stephane.piskorski.ext@parrot.fr>
 * @date   Sept, 8. 2010
 *
 *******************************************************************/


/* Includes required to handle navigation data */
	#include <ardrone_tool/Navdata/ardrone_navdata_client.h>
	#include <Navdata/navdata.h>

#include <custom_code.h>
#include <config_keys.h>

/*
#include <ardrone_tool/ardrone_tool.h>
#include <ardrone_tool/ardrone_tool_configuration.h>
*/


//const int MAX_DATA = 100;

// these arrays hold the past 100 values of the distance to the tag, the x and y coordinate of the tag on the camera,
// and the velocity along the y axis (side to side movement)
float distances[100] = {0};
float xs[100] = {0};
float ys[100] = {0};
float vys[100] = {0};
float vxs[100] = {0};
int index = 0;

#define VEL_SIZE 1000

/*---------------------------------------------------------------------------------------------------------------------
Function taking the drone control state stored as an integer, and prints in a string 
the names of the set control bits.

The drone control state is an integer sent in the navdata which says if the drone is landed, flying,
hovering, taking-off, crashed, etc ...
---------------------------------------------------------------------------------------------------------------------*/

	#define CTRL_STATES_STRING
	#include "control_states.h"

const char* ctrl_state_str(uint32_t ctrl_state)
{
  #define MAX_STR_CTRL_STATE 256
  static char str_ctrl_state[MAX_STR_CTRL_STATE];

  ctrl_string_t* ctrl_string;
  uint32_t major = ctrl_state >> 16;
  uint32_t minor = ctrl_state & 0xFFFF;

  if( strlen(ctrl_states[major]) < MAX_STR_CTRL_STATE )
  {
    vp_os_memset(str_ctrl_state, 0, sizeof(str_ctrl_state));

    strcat_s(str_ctrl_state, sizeof(str_ctrl_state),ctrl_states[major]);
    ctrl_string = control_states_link[major];

    if( ctrl_string != NULL && (strlen(ctrl_states[major]) + strlen(ctrl_string[minor]) < MAX_STR_CTRL_STATE) )
    {
      strcat_s( str_ctrl_state,sizeof(str_ctrl_state), " | " );
      strcat_s( str_ctrl_state, sizeof(str_ctrl_state),ctrl_string[minor] );
    }
  }
  else
  {
    vp_os_memset( str_ctrl_state, '#', sizeof(str_ctrl_state) );
  }

  return str_ctrl_state;
}



/*---------------------------------------------------------------------------------------------------------------------
Initialization local variables before event loop  
---------------------------------------------------------------------------------------------------------------------*/
inline C_RESULT demo_navdata_client_init( void* data )
{
	/**	======= INSERT USER CODE HERE ========== **/
	/* Initialize your navdata handler */
	

	/*
	int enemyColors = ARDRONE_DETECTION_COLOR_ORANGE_BLUE;
	ARDRONE_TOOL_CONFIGURATION_ADDEVENT (enemy_colors, &enemyColors, NULL);
	*/

	/*
	ARDRONE_TOOL_CONFIGURATION_ADDEVENT ("detect_type", "3", NULL);
	ARDRONE_TOOL_CONFIGURATION_ADDEVENT ("detections_select_h", TAG_TYPE_MASK (TAG_TYPE_SHELL_TAG), NULL);
	*/

	
	
	//CAD_TYPE detect_type = CAD_TYPE_MULTIPLE_DETECTION_MODE;

	/*
	ardrone_at_set_toy_configuration("detect:detect_type", "2");
	//ardrone_at_set_toy_configuration("detect:detections_select_h", "1");
	ardrone_at_set_toy_configuration("detect:enemy_colors", "3");
	*/

	/*
	ardrone_at_set_toy_configuration("detect:detections_select_h", "1");
	ardrone_at_set_toy_configuration("detect:enemy_colors", "3");
	*/


	//ardrone_at_set_toy_configuration("detect:enemy_colors", (char*) (ARDRONE_DETECTION_COLOR_ORANGE_BLUE));
	ardrone_at_set_toy_configuration("detect:enemy_without_shell", "1");
	ardrone_at_set_toy_configuration("detect:enemy_colors", "3");
	//ardrone_at_set_toy_configuration("detect:detect_type", "2");
	//ardrone_at_set_toy_configuration("detect:detect_type", "10");


	//ARDRONE_TOOL_CONFIGURATION_ADDEVENT("detect_type", "2", NULL);
	//ARDRONE_TOOL_CONFIGURATION_ADDEVENT (enemy_colors, (int32_t*) (ARDRONE_DETECTION_COLOR_ORANGE_BLUE), NULL);

	// This works, it sets the video feed to the vertical camera.
	//ardrone_at_set_toy_configuration("video:video_channel", "1");

	/**	======= INSERT USER CODE HERE ========== **/

  return C_OK;
}





/*---------------------------------------------------------------------------------------------------------------------
Navdata handling function, which is called every time navdata are received
---------------------------------------------------------------------------------------------------------------------*/
inline C_RESULT demo_navdata_client_process( const navdata_unpacked_t* const navdata )
{
	static int cpt=0;

    const navdata_demo_t* const nd = &navdata->navdata_demo;
	const navdata_vision_detect_t* const vd = &navdata->navdata_vision_detect;

	//FILE *fp;
	//static int index = 0;
	//static float vx_array[VEL_SIZE];
	//static float the_times[VEL_SIZE];

	//static int64 timeStart_sp_timer;
	//static double timeDiff_sp_timer;
	//static double timeTally_sp_timer = 0;
	//static int countTally_sp_timer = 0;
	//static int started = 0;

	//int i = 0;

	//IplImage* graph = cvCreateImage(cvSize(1200, 400), IPL_DEPTH_8U, 3);

	//cvSet(graph, cvScalarAll(255), NULL);
	//setGraphColor(0);

	//if (index == 0) {
	//	START_TIMING(sp_timer);
	//	
	//}

	//if (index < VEL_SIZE) {
	//	vx_array[index] = nd->vy;
	//	STOP_TIMING(sp_timer);
	//	the_times[index] = (float) GET_TIMING(sp_timer)/1000;
	//	
	//	drawFloatGraph(vx_array, VEL_SIZE, graph, -200, 800, graph->width, graph->height, "Vx", 1);
	//	cvNamedWindow("Vx Graph", CV_WINDOW_AUTOSIZE);
	//	cvShowImage("Vx Graph", graph);
	//	cvWaitKey(1);

	//	index++;
	//}

	//if (index == VEL_SIZE) {
	//	if((fopen_s(&fp, "velocity_list.txt", "w"))) {
	//		exit(-1);
	//	}
	//
	//	for (i = 0; i<VEL_SIZE; i++) {
	//		fprintf(fp, "%f %f\n", the_times[i], vx_array[i]);
	//	}
	//	fclose(fp);
	//	index++;
	//}

	//cvReleaseImage(&graph);

	
		/**	======= INSERT USER CODE HERE ========== **/

				// Drone_Tag_Tracking uses the onboard tag detection system to determine an appropiate
				// pitch, roll, yaw, and gaz.
				//Drone_Tag_Tracking(navdata);
				fd->vx = nd->vx;
				fd->vy = nd->vy;
				fd->vz = nd->vz;

				ARWin32Demo_AcquireConsole();
				if ((cpt++)==90) { system("cls"); cpt=0; }
				
				ARWin32Demo_SetConsoleCursor(0,0);  // Print at the top of the console
				printf("=================================\n");
				printf("Navdata for flight demonstrations\n");
				printf("=================================\n");

				printf("Control state : %s                                      \n",ctrl_state_str(nd->ctrl_state));
				printf("Battery level : %i/100          \n",nd->vbat_flying_percentage);
				printf("Orientation   : [Theta] %4.3f  [Phi] %4.3f  [Psi] %4.3f          \n",nd->theta,nd->phi,nd->psi);
				printf("Altitude      : %i                                          \n",nd->altitude);
				printf("Speed         : [vX] %4.3f  [vY] %4.3f  [vZ] %4.3f          \n",nd->vx,nd->vy,nd->vz);
				//printf("Tag           : %i                                          \n", vd->nb_detected);
				//printf("xc            : %i                                          \n", vd->xc[0]);
				//printf("yc            : %i                                          \n", vd->yc[0]);
				//printf("Distance      : %i                                          \n", vd->dist[0]);
				printf("Automatic     : [Pitch %f] [Roll %f]                        \n", fd->pitch, fd->roll);
				printf("              : [Yaw %f] [Gaz %f]%                          \n", fd->yaw, fd->gaz);
				printf("bg_progress   : %i                                          \n", bg_progress);
				printf("HSV           : H: %f  S: %f  V:  %f                        \n", BGR.val[0], BGR.val[1], BGR.val[2]);
				ARWin32Demo_ReleaseConsole();

		/** ======= INSERT USER CODE HERE ========== **/
		
		return C_OK;
}






/*---------------------------------------------------------------------------------------------------------------------
 Relinquish the local resources after the event loop exit 
---------------------------------------------------------------------------------------------------------------------*/
inline C_RESULT demo_navdata_client_release( void )
{
	/**	======= INSERT USER CODE HERE ========== **/
	/* Clean up your navdata handler */
	/**	======= INSERT USER CODE HERE ========== **/
  return C_OK;
}





/* 
Registering the navdata handling function to 'navdata client' which is part 
of the ARDroneTool.
You can add as many navdata handlers as you want.
Terminate the table with a NULL pointer.
*/
BEGIN_NAVDATA_HANDLER_TABLE
  NAVDATA_HANDLER_TABLE_ENTRY(demo_navdata_client_init, demo_navdata_client_process, demo_navdata_client_release, NULL)
END_NAVDATA_HANDLER_TABLE

