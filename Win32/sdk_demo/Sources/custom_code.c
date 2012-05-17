/********************************************************************
 *                    COPYRIGHT PARROT 2010
 ********************************************************************
 *       PARROT - A.R.Drone SDK Windows Client Example
 *-----------------------------------------------------------------*/
/**
 * @file custom_code.c 
 * @brief User added code
 *
 * @author sylvain.gaeremynck@parrot.com
 * @date 2009/07/01
 *
 * @author Stephane Piskorski <stephane.piskorski.ext@parrot.fr>
 * @date   Sept, 8. 2010
 *
 *******************************************************************/



#include <custom_code.h>

//ARDroneLib
	#include <ardrone_tool/ardrone_time.h>
	#include <ardrone_tool/Navdata/ardrone_navdata_client.h>
	#include <ardrone_tool/Control/ardrone_control.h>
	#include <ardrone_tool/UI/ardrone_input.h>

//Common
	#include <config.h>
	#include <ardrone_api.h>

//VP_SDK
	#include <ATcodec/ATcodec_api.h>
	#include <VP_Os/vp_os_print.h>
	#include <VP_Api/vp_api_thread_helper.h>
	#include <VP_Os/vp_os_signal.h>

//Local project
	#include <UI/gamepad.h>
	#include <Video/video_stage.h>
	#include <UI/directx_rendering.h>

//Custom Headers
	#include <pthread.h>
	//#include <time.h>
 //   #include <cv.h>
 //   #include <cvaux.h>
 //   #include <highgui.h>
	//#include <cxcore.h>
//Global variables
	int32_t exit_ihm_program = 1;
	vp_os_mutex_t consoleMutex;

	long color_check;
	long num_hits;
	const int THRESHOLD[3] = {40, 40, 40};
	const int MAX_DATA = 100;
	int bg_progress = 0;
	bg_model new_model;


	Flight_Data Flight_Data_t;
	Flight_Data* const fd = &Flight_Data_t;
	struct Custom_Navdata Custom_Navdata_t = { {0}, {0}, {0}, {0}, {0}, 0 };
	struct Custom_Navdata* const CN = &Custom_Navdata_t;
	object_info new_obj;
	object_info* obj_p = &new_obj;
	dir_vector new_vec = { {0}, {0}, {0}, {0}, {0}, {0}, 0, 900 };
	dir_vector* vec_p = &new_vec;
	pthread_mutex_t fd_mutex;
	CvScalar BGR;
	double mytime = 0;
	double time2 = 0;
	double time3 = 0;
	double xyz[3] = {1, 0};

	#define NUMROWS 240
	#define NUMCOLS 320
	#define NUMTHRD 10
	pthread_t callRow[NUMROWS];
	pthread_t callCol[NUMCOLS];
	pthread_t RowThreads[NUMTHRD];
	pthread_mutex_t mutexrow;
	pthread_mutex_t mutexcol;


	typedef struct _vector_info
	{
		int *hitmap;
		double row_sum;
		double col_sum;
	} vector_info;

	vector_info hitmap_info;

	typedef struct _rgb_map
	{
		int colors;
		uint8_t* pix_buf;
		long *hits;
		int **hit_map;
		double **Target_RGB;
	} rgb_map;

	rgb_map rgb_map_info;
	

/* Implementing Custom methods for the main function of an ARDrone application */



/*--------------------------------------------------------------------
The delegate object calls this method during initialization of an 
ARDrone application 
--------------------------------------------------------------------*/
C_RESULT ardrone_tool_init_custom(int argc, char **argv)
{
	/* Change the console title */
		vp_os_mutex_init(&consoleMutex);
		system("cls");
		SetConsoleTitle(TEXT("Parrot A.R. Drone SDK Demo for Windows"));
		//CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)&HmiStart,NULL,0,0);
		//CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)&HmiStart2,NULL,0,0);


	/* Registering for a new device of game controller */
		ardrone_tool_input_add( &dx_keyboard );
		ardrone_tool_input_add( &dx_gamepad );
	
	/* Start all threads of your application */
		START_THREAD( directx_renderer_thread , NULL);
		START_THREAD( video_stage, NULL );

		
  
  return C_OK;
}



/*--------------------------------------------------------------------
The delegate object calls this method when the event loop exit     
--------------------------------------------------------------------*/
C_RESULT ardrone_tool_shutdown_custom()
{
  /* Relinquish all threads of your application */
  JOIN_THREAD( video_stage );

  /* Unregistering for the current device */
  ardrone_tool_input_remove( &dx_gamepad );
  ardrone_tool_input_remove( &dx_keyboard );

  /* Mutex needs to be destroyed but may still be used by navdata thread */
  /* todo : synchronize threads shutdown */
  //vp_os_mutex_destroy(&consoleMutex);

  return C_OK;
}

/*--------------------------------------------------------------------
The event loop calls this method for the exit condition            
--------------------------------------------------------------------*/
bool_t ardrone_tool_exit()
{
  return exit_ihm_program == 0;
}

C_RESULT signal_exit()
{
  exit_ihm_program = 0;

  return C_OK;
}

int custom_main(int argc,char**argv) { return 0;};


/* Implementing thread table in which you add routines of your application and those provided by the SDK */
BEGIN_THREAD_TABLE
  THREAD_TABLE_ENTRY( ardrone_control, 20 )
  THREAD_TABLE_ENTRY( navdata_update, 20 )
  THREAD_TABLE_ENTRY( video_stage, 20 )
  THREAD_TABLE_ENTRY( directx_renderer_thread, 20 )
END_THREAD_TABLE



/* Function to change the cursor place in the console window */

	HANDLE hStdout =  NULL;  /* Handle to the output console */
	CONSOLE_SCREEN_BUFFER_INFO csbiInfo;				/* Information about the output console */



void ARWin32Demo_SetConsoleCursor(int x,int y)
{
	if (hStdout==NULL) hStdout=GetStdHandle(STD_OUTPUT_HANDLE);

	if (hStdout != INVALID_HANDLE_VALUE){
			GetConsoleScreenBufferInfo(hStdout, &csbiInfo);
			csbiInfo.dwCursorPosition.X=x;
			csbiInfo.dwCursorPosition.Y=y;
			SetConsoleCursorPosition(hStdout,csbiInfo.dwCursorPosition);
	}
}

void ARWin32Demo_AcquireConsole(int x,int y)
{
	vp_os_mutex_lock(&consoleMutex);
}
void ARWin32Demo_ReleaseConsole(int x,int y)
{
	vp_os_mutex_unlock(&consoleMutex);
}


// This function will take the weighted average of a given sequence of numbers. It does this by taking three averages then averaging
// those together. The small average covers about a tenth of the data from the current point, the medium average covers half the data,
// and the full average takes the average of all the data. Then the function returns the weighted average of those three numbers, with
// the averages closer in time to the present being weighted more heavily.
float weighted_avg(float seq[], int ind, int size)
{
	float full_avg, med_avg, sm_avg;
	float sum = 0;
	int j = 0;
	int k = ind;
	for (j = 0; j<size; j++)
	{
		sum = sum + seq[k];
		k--;
		if (k < 0)
		{
			k = size-1;
		}

		if (j == (size/10-1))
		{
			sm_avg = 10*sum/size;
		}
		if (j == (size/2-1))
		{
			med_avg = 2*sum/size;
		}
		if (j == (size-1))
		{
			full_avg = sum/size;
		}
	}

	return (float) (0.5*sm_avg + 0.3*med_avg + 0.2*full_avg);
}

// This function is meant to calculate an appropiate percentage to pass onto the drone command function. It takes the value to
// be converted and an appropiate end point. For example, the pixels on the camera measure 1000x1000, so, measuring from the center,
// the farthest pixel distance is 500. This function creates a curve similar to a logarithmic function, but it have no negative values,
// and the function approaches 1 as the value approaches infinity. 
float throttle_val(float value, float end)
{
	float percent;
	float coef;
	
	coef = (float) ((-0.0001) * end + 0.06);
	percent = 1 - (float) exp(-(coef)*fabs(value));

	if (value<0)
	{
		percent = -percent;
	}

	return percent;

}


// The old drone tag tracking program has been moved here. It takes points to the navdata, flight data, and custom navdata as inputs.
// It's purpose is to set the flight data to allow the drone to be controlled according to the information in navdata. 
int Drone_Tag_Tracking(const navdata_unpacked_t* const navdata)
{
    const navdata_demo_t* const nd = &navdata->navdata_demo;
	const navdata_vision_detect_t* const vd = &navdata->navdata_vision_detect;

	float dist = 0;
	float xc = 0;
	float yc = 0;
	float vy = 0;
	float vx = 0;

	
	//struct Flight_Data* fd = &Flight_Data_t;

	// Along some turns, it seemed that the drone was drifting to the side a lot. This code basically
	// tells the drone to keep its y-axis velocity close to zero.
	(CN->vys)[CN->index] = nd->vy;
	vy = weighted_avg(CN->vys, CN->index, MAX_DATA);
	if (fabs(vy) < 40)
	{
		vy = 0;
	}
	fd->roll = (float) 0.25*throttle_val(((float) (-vy)), 100);

	if (vd->nb_detected >= 1)
	{	
		CN->distances[CN->index] = (float) (vd->dist[0]);
		CN->xs[CN->index] = (float) (vd->xc[0]);
		CN->ys[CN->index] = (float) (vd->yc[0]);

		dist = weighted_avg(CN->distances, CN->index, MAX_DATA);
		xc = weighted_avg(CN->xs, CN->index, MAX_DATA);
		yc = weighted_avg(CN->ys, CN->index, MAX_DATA);

		fd->pitch = throttle_val(((float) (200-dist)), 100);
		fd->yaw = throttle_val(((float)(xc)-500), 500);
		fd->gaz = throttle_val((500-(float)(yc)), 500);
	}

	else
	{
			// When the tag is not detected, values are assigned to the arrays as if the drone were
			// perfectly aligned with the tag. This way, the drone will continue what it was doing for a while,
			// but it will eventually stop movement.
			CN->distances[CN->index] = 200;
			CN->xs[CN->index] = 500;
			CN->ys[CN->index] = 500;

			CN->vxs[CN->index] = nd->vx;
			vx = weighted_avg(CN->vxs, CN->index, MAX_DATA);
			if (fabs(vx) < 40)
			{
				vx = 0;
			}
			

			dist = weighted_avg(CN->distances, CN->index, MAX_DATA);
			xc = weighted_avg(CN->xs, CN->index, MAX_DATA);
			yc = weighted_avg(CN->ys, CN->index, MAX_DATA);


			// If the distance weighted difference away is still significant, then continue what the drone was doing.
			if (fabs(dist-200) > 20)
			{
				fd->pitch = (float) 0.35*throttle_val(((float) (200-dist)), 100);
			}
			// Otherwise, try and stop moving in along the x-axis.
			else
			{
				fd->pitch = (float) 0.25*throttle_val(((float) (vx)), 100);
			}

			// Also, only a fraction of the pitch, yaw, and gaz values are used.
			
			fd->yaw = (float) 0.5*throttle_val(((float)(xc)-500), 500);
			fd->gaz = (float) 0.5*throttle_val((500-(float)(yc)), 500);
	}

	(CN->index)++;

	if (CN->index == MAX_DATA)
	{
		CN->index = 0;
	}
	return 0;
}

// Color_Control
// this function is supposed to determine appropiate values of pitch, roll, yaw, and gaz based on what colors can be seen.
// inputs: Target_RGBs[] - an array of pointers to the RGB arrays of desired color
//         colors - the number of colors to be detected
//         tolerance - currently not used, it's supposed to be like some kind of threshold so if enough pixels are found
//                     to match the description, then do something.
//         pixbuf_data - the picture buffer
//         pic_size - the size of the picture buffer (currently it takes a value of 230400, which is the actual image size. The
//                    actual buffer size is larger)
//         hits - a pointer to an array containing ints, this array will hold the number of matching pixels for each desired color
//         hit_map - a pointer to various arrays that will contain either 1s or 0s, representing where each matching pixel is on the frame
int Color_Control(double *Target_RGBs[], int colors, double tolerance, uint8_t* pixbuf_data, long pic_size, long *hits, int *hit_map[])
{
	
	fd->pitch = 0;
	fd->roll = 0;
	fd->yaw = 0;
	fd->gaz = 0;
	

	find_color(pixbuf_data, pic_size, Target_RGBs, colors, hits, hit_map);

	
	if (hits[0] > tolerance*pic_size/3)
	{
		fd->roll = 1;
	}
	if (hits[1] > tolerance*pic_size/3)
	{
		fd->roll = -1;
	}
	


	return 0;
}

// find_color function
// purpose: to sift through the picture buffer data and return 
//          the number of pixels that match a certain description
// inputs: pixbuf_data - a pointer to an array containing RGB information about the picture
//         size - the size of the buffer
//         *Target_RGB[] - an array containing pointers to arrays containing 
//                         the three RGB values being searched for each color
//         colors - # of colors being searched for
//         *hits - pointer to an array containing # of hits for each color
//         hit_map - pointer to an array of int's that mark the position of the matching pixels
int find_color(uint8_t* pixbuf_data, long size, double *Target_RGB[], int colors, long *hits, int* hit_map[])
{
	/*
	long i = 0;
	int j = 0;
	int k = 0;
	int height = 240;
	int width = 320;
	int max_width = 640;
	double RGB_value[3] = {0, 0, 0};

	for (k = 0; k<height; k++)
	{
		for (i = 0; i<(3*width); i+=3)
		{
			RGB_value[0] = (double) pixbuf_data[i+3*k*max_width];
			RGB_value[1] = (double) pixbuf_data[i+1+3*k*max_width];
			RGB_value[2] = (double) pixbuf_data[i+2+3*k*max_width];
			
			for (j = 0; j<colors; j++)
			{
				if (RGB_in_Range(RGB_value, Target_RGB[j]) == 1)
				{
					(hits[j])++;
					hit_map[j][((i+3*k*width)/3)] = 1;
					if (j == 0)
					{
						pixbuf_data[i+3*k*max_width] = (char) 255;
						pixbuf_data[i+1+3*k*max_width] = (char) 255;
						pixbuf_data[i+2+3*k*max_width] = (char) 255;
					}
					else
					{
						pixbuf_data[i+3*k*max_width] = (char) 34;
						pixbuf_data[i+1+3*k*max_width] = (char) 139;
						pixbuf_data[i+2+3*k*max_width] = (char) 34;
					}
				}
			}
			
		}
	}
	return 0;
	*/
	

	
	int rc;
	int k = 0;
	double RGB_value[3] = {0, 0, 0};
	void *status;
	pthread_attr_t attr;
	size_t stacksize;

	pthread_mutex_init(&mutexrow, NULL);

	rgb_map_info.colors = colors;
	rgb_map_info.pix_buf = pixbuf_data;
	rgb_map_info.hits = hits;
	rgb_map_info.hit_map = hit_map;
	rgb_map_info.Target_RGB = Target_RGB;

	pthread_attr_init(&attr);
	stacksize = sizeof(double)*3*100000;
	pthread_attr_setstacksize (&attr, stacksize);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

	
	for (k = 0; k<NUMTHRD; k++)
	{
		rc = pthread_create(&RowThreads[k], &attr, search_row, (void *) k);
		if (rc)
		{
			printf("ERROR; return code from pthread_create() is %d\n", rc);
			exit(-1);
		}
	}
	pthread_attr_destroy(&attr);
	k = 0;
	for (k = 0; k<NUMTHRD; k++)
	{
		rc = pthread_join(RowThreads[k], &status);
	}
	k = 0;
	for (k = 0; k<colors; k++)
	{
		hits[k] = rgb_map_info.hits[k];
	}

	pthread_mutex_destroy(&mutexrow);
	return 0;
	

}

// RGB_in_Range function
// purpose: given a set of RGB values, determine whether or not they lie within
//          a certain threshold, return 1 if yes, 0 if not
// inputs: RGB_value[] - an array holding the RGB values to be compared
//         threshold - a number to decide how wide the range is
//         Target_RGB[] - an array holding the desired RGB values
float RGB_in_Range(double RGB_value[], double Target_RGB[])
{
	/*
	int i = 0;
	
	for (i = 0; i<3; i++)
	{
		if ( !(((Target_RGB[i] - THRESHOLD[i]) < RGB_value[i]) && (RGB_value[i] < (Target_RGB[i] + THRESHOLD[i]))) )
		{
			return 0;
		}
	}
	return 1;
	*/

	int range = 25;
	double D;

	//normalize_RGB(Target_RGB);
	normalize_RGB(RGB_value);

	D = sqrt( pow((Target_RGB[0]-RGB_value[0]), 2) + pow((Target_RGB[1]-RGB_value[1]), 2) + pow((Target_RGB[2]-RGB_value[2]), 2));


	
	if (D>range)
	{
		return 0;
	}
	return 1;
	

}

/*
double find_hue(double RGB[])
{
	double M, m;
	double C;
	double H;

	M = max((max(RGB[0], RGB[1])), RGB[2]);
	m = min((min(RGB[0], RGB[1])), RGB[2]);
	C = M - m;

	if (C == 0)
	{
		H = 0;
	} 
	else if (M == RGB[0])
	{
		H = ((RGB[1] - RGB[2])/C)%6;
	}
	else if (M == RGB[1])
	{
		H = ((RGB[2] - RGB[0])/C + 2);
	}
	else if (M == RGB[2])
	{
		H = ((RGB[0] - RGB[1])/C + 4);
	}

	return (60*H);
}
*/

int normalize_RGB(double RGB[])
{
	// approximation to the normalized value
	int i = 0;
	double sum = 0;

	for (i = 0; i<3; i++)
	{
		sum += RGB[i];
	}

	if (sum != 0)
	{
		i = 0;
		for (i = 0; i<3; i++)
		{
			RGB[i] = 255*RGB[i]/sum;
		}
	}
	else
	{
		RGB[0] = 147;
		RGB[1] = 147;
		RGB[2] = 147;
	}
	return 0;
	
	/* 
	// Exact normalization
	double scale = sqrt( pow((RGB[0]), 2) + pow((RGB[1]), 2) + pow((RGB[2]), 2));
	if (scale == 0)
	{
		RGB[0] = 147;
		RGB[1] = 147;
		RGB[2] = 147;
	}
	else
	{
		RGB[0] = 255*RGB[0]/scale;
		RGB[1] = 255*RGB[1]/scale;
		RGB[2] = 255*RGB[2]/scale;
	}

	return 0;
	*/
}


// find_xy_centroid
// purpose: given the map of pixels with a correct description, determine the approximate x and y centroids
// inputs:  hit_map - an array containing the locations of the pixels
//          width - the width of the picture
//          height - the height of the picture
//          hits - the total number of matching pixels
//          xy - an array that will contain the xy coordinate
int find_xy_centroid(int hit_map[], int width, int height, int hits, int xy[])
{
	
	int c_sum = 0;
	int r_sum = 0;
	int x_centroid = 0;
	int i = 0;
	int j = 0;

	for (i = 0; i<width; i++)
	{
		c_sum = 0;

		for (j = 0; j<height; j++)
		{
			c_sum = c_sum + hit_map[i+j*width];
			r_sum = r_sum + j*hit_map[i+j*width];

		}
		x_centroid = x_centroid + c_sum*i;
	}

	if (hits != 0)
	{
		xy[0] = x_centroid/hits;
		xy[1] = r_sum/hits;
	}
	else
	{
		return 0;
	}

	return 1;
	

	/*
	void *status;
	pthread_attr_t attr;
	int i = 0;
	int j = 0;
	size_t stacksize;

	pthread_mutex_init(&mutexcol, NULL);
	pthread_mutex_init(&mutexrow, NULL);

	hitmap_info.col_sum = 0;
	hitmap_info.row_sum = 0;
	hitmap_info.hitmap = hit_map;

	pthread_attr_init(&attr);

	stacksize = sizeof(double)*3*4000000;
	pthread_attr_setstacksize (&attr, stacksize);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	
	for (i = 0; i<(NUMROWS); i++)
	{
		pthread_create(&callRow[i], &attr, row_sum, (void *) i);
	}
	i = 0;
	for (i = 0; i<(NUMCOLS); i++)
	{
		pthread_create(&callCol[i], &attr, col_sum, (void *) i);
	}
	
	pthread_attr_destroy(&attr);
	for (i = 0; i<(NUMROWS); i++)
	{
		pthread_join(callRow[i], &status);
	}
	for (i = 0; i<(NUMCOLS); i++)
	{
		pthread_join(callCol[i], &status);
	}
	pthread_mutex_destroy(&mutexcol);
	pthread_mutex_destroy(&mutexrow);

	xy[0] = (int) hitmap_info.col_sum/hits;
	xy[1] = (int) hitmap_info.row_sum/hits;
	
	return 0;
	*/
	
}

void *col_sum(void *arg)
{
	int i = 0;
	int my_sum = 0;
	long tid = (long) arg;

	for (i = 0; i<NUMROWS; i++)
	{
		my_sum += (tid)*(hitmap_info.hitmap[tid + i*NUMCOLS]);
	}
	
	
	pthread_mutex_lock(&mutexcol);
	hitmap_info.col_sum += my_sum;
	pthread_mutex_unlock(&mutexcol);
	
	pthread_exit(NULL);
	return 0;
}

void *row_sum(void *arg)
{
	int i = 0;
	int my_sum = 0;
	long tid = (long) arg;

	for (i = 0; i<NUMCOLS; i++)
	{
		my_sum += (tid)*(hitmap_info.hitmap[i + tid*NUMCOLS]);
	}

	
	pthread_mutex_lock(&mutexrow);
	hitmap_info.row_sum += my_sum;
	pthread_mutex_unlock(&mutexrow);
	
	

	pthread_exit((void*) 0);
	return 0;
}

void *PrintHello(void *threadid)
{
	long tid;
	tid = (long)threadid;

	pthread_mutex_lock(&mutexrow);
	hitmap_info.hitmap[0];
	pthread_mutex_unlock(&mutexrow);
   
	pthread_exit(NULL);
   
	return 0;
}

void *search_row(void *arg)
{
	int i = 0;
	int j = 0;
	int k = 0;
	long tid;
	double RGB_value[3] = {0, 0, 0};
	int offset = 0;

	tid = (long) arg;

	for (k = 0; k<24; k++)
	{
		offset = 3*(NUMCOLS*2)*((24*tid)+k);

		for (i = 0; i<(3*NUMCOLS); i+=3)
		{
			RGB_value[0] = (double) rgb_map_info.pix_buf[i+offset];
			RGB_value[1] = (double) rgb_map_info.pix_buf[i+1+offset];
			RGB_value[2] = (double) rgb_map_info.pix_buf[i+2+offset];
			
			for (j = 0; j<rgb_map_info.colors; j++)
			{
				
				if (RGB_in_Range(RGB_value, rgb_map_info.Target_RGB[j]) == 1)
				{
					pthread_mutex_lock(&mutexrow);
					(rgb_map_info.hits[j])++;
					rgb_map_info.hit_map[j][((i+offset/2)/3)] = 1;
					if (j == 0)
					{
						rgb_map_info.pix_buf[i+offset] = (char) 255;
						rgb_map_info.pix_buf[i+1+offset] = (char) 255;
						rgb_map_info.pix_buf[i+2+offset] = (char) 255;
					}
					else
					{
						rgb_map_info.pix_buf[i+offset] = (char) 34;
						rgb_map_info.pix_buf[i+1+offset] = (char) 139;
						rgb_map_info.pix_buf[i+2+offset] = (char) 34;
					}
					pthread_mutex_unlock(&mutexrow);
				}
				
			}
		}
		
		
	}
	pthread_exit(NULL);
	return 0;
}

IplImage* convert_to_CVimg(uint8_t* pixbuf_data)
{
	// a simple conversion function
	IplImage* img = cvCreateImage(cvSize(320, 240), IPL_DEPTH_8U, 3);
	int i = 0;
	int j = 0;
	int k = 0;
	uchar* data;

	// set a pointer to the image data of the newly created image
	data = img->imageData;

	for (i = 0; i<240; i++)
	{
		for (j = 0; j<(320); j++)
		{
			for (k = 0; k<3; k++)
			{
				// I'm assuming a 320x240 image is going to be created, and
				// I already know that pixbuf_data is the top left corner of 
				// a 640x480 image, so the numbers are pretty specific here
				data[3*j+3*i*320+k] = pixbuf_data[3*j+3*i*640+(2-k)];
			}
		}
	}
	return img;
}

IplImage* threshold_colors(IplImage* img, int color_hue, int thresh)
{
	IplImage* temp_img = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3);
	//IplImage* temp_img = cvCloneImage(img);
	IplImage* imgHSV = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3);
	IplImage* imgThreshed = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
	IplImage* img2 = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
	
	//IplImage* saved_img = cvCloneImage(img);
	//IplImage* canny_img = cvCloneImage(imgThreshed);

	// note: this assumes you got the hue values from MS Paint
	int cv_hue = (int) (color_hue);
	int lower = cv_hue - thresh;
	int upper = cv_hue + thresh;
	int low2 = cv_hue - thresh/2;
	int up2 = cv_hue + thresh/2;

	//CvSize sz = cvSize( img->width & -2, img->height & -2 );
	//IplImage* pyr = cvCreateImage( cvSize(sz.width/2, sz.height/2), 8, 3 );


	//cvSaveImage("SampleImg.bmp", img, 0);

	cvSmooth(img, temp_img, CV_GAUSSIAN, 11, 0, 11, 11);
	//cvDilate(imgThreshed, imgThreshed, NULL, 3);
	//cvPyrDown( temp_img, pyr, 7 );
 //   cvPyrUp( pyr, temp_img, 7 );

	//cvSaveImage("SampleGaussian.bmp", temp_img, 0);

	// convert image to HSV
	cvCvtColor(temp_img, imgHSV, CV_BGR2HSV);

	//cvCvtColor(img, imgHSV, CV_BGR2HSV);
	//histeq(imgHSV, imgHSV);

	// threshold the image
	// if the threshold takes the value above or below the max, then two separate thresholds are needed.
	// After aquiring those, they need to be added together.
	// For example, the hue of red is roughly (in MS Paint units) 0-10 and 230-240. So if I set the threshold
	// to 3 and look for hues of 0, I'll get -3 and +3, so I need to search the thresholds 0-3 and 237-240.
	if (lower<0) {
		lower += 180;
		cvInRangeS(imgHSV, cvScalar(lower, SV_LOWER, 0, 0), cvScalar(180, 256, 256, 0), img2);
		cvInRangeS(imgHSV, cvScalar(0, SV_LOWER, 0, 0), cvScalar(upper, 256, 256, 0), imgThreshed);
		cvAdd(imgThreshed, img2, imgThreshed, NULL);
	} else if (upper > 180) {
		upper -= 180;
		cvInRangeS(imgHSV, cvScalar(lower, SV_LOWER, -1, 0), cvScalar(180, 256, 256, 0), img2);
		cvInRangeS(imgHSV, cvScalar(0, SV_LOWER, -1, 0), cvScalar(upper, 256, 256, 0), imgThreshed);
		cvAdd(imgThreshed, img2, imgThreshed, NULL);
	} else {
		cvInRangeS(imgHSV, cvScalar(lower, SV_LOWER, -1, 0), cvScalar(upper, 256, 256, 0), imgThreshed);
		cvInRangeS(imgHSV, cvScalar(low2, 0.8*SV_LOWER, -1, 0), cvScalar(up2, 256, 256, 0), img2);
		cvAdd(imgThreshed, img2, imgThreshed, NULL);
	}

	//cvSaveImage("SampleThreshed.bmp", imgThreshed, 0);
	//cvCanny(imgThreshed, canny_img, 50, 150, 3);
	//cvSaveImage("SampleCanny.bmp", canny_img, 0);

	//cvErode(imgThreshed, imgThreshed, NULL, 2);
	//cvDilate(imgThreshed, imgThreshed, NULL, 3);
	cvSmooth(imgThreshed, imgThreshed, CV_GAUSSIAN, 7, 0, 11, 11);
	cvDilate(imgThreshed, imgThreshed, NULL, 1);
	cvThreshold(imgThreshed, imgThreshed, 200, 255, CV_THRESH_BINARY);

	cvReleaseImage(&imgHSV);
	cvReleaseImage(&temp_img);
	//cvReleaseImage(&pyr);
	cvReleaseImage(&img2);

	//cvReleaseImage(&saved_img);
	//cvReleaseImage(&canny_img);
	//exit(-1);


	return imgThreshed;
}
/*
IplImage* dist_thresh(IplImage* img, int color_hue)
{
	IplImage* img32 = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_32F, 3);
	IplImage* HSV32 = cvCloneImage(img32);
	CvScalar tar_HSV;
	float hue = (float) (color_hue/(180.0));
	
	tar_HSV = cvScalar(hue, 1, 0.5, 0);

	cvConvertScale(img, img32, 1.0/255.0, 0);
	cvCvtColor(img32, HSV32, CV_BGR2HSV);

	

	return 0;
}

CvMat* dist_mat(IplImage* HSV, CvScalar tar_HSV)
{
	CvMat* mat1 = cvCreateMat(HSV->height, HSV->width, CV_32FC1);
	IplImage* tar_img = cvCreateImage(cvGetSize(HSV), IPL_DEPTH_32F, 3);

	float h = (float) tar_HSV.val[0];
	float s = (float) tar_HSV.val[1];
	float v = (float) tar_HSV.val[2];

	double x = v*s*sin(2*PI*h);
	double y = v*s*cos(2*PI*h);

	CvScalar comp = cvScalar(x, y, v, 0);

	cvSet(tar_img, comp, 0);


	

	return 0;
}

CvMat* fill_dist_mat(IplImage* HSV, CvMat* dist_mat)
{
	int mat_step = dist_mat->step/sizeof(float);
	float *mat_data = dist_mat->data.fl;

	int img_step = HSV->widthStep/sizeof(float);
	float *img_data = HSV->imageData;

	int i = 0;
	int j = 0;
	int k = 0;

	float h;
	float s;
	float v;

	double x;
	double y;

	for (i = 0; i<HSV->height; i++) {
		for (j = 0; j<HSV->width; j++) {
			h = img_data[i*img_step + j*HSV->nChannels];
			s = img_data[i*img_step + j*HSV->nChannels + 1];
			v = img_data[i*img_step + j*HSV->nChannels + 2];

			x = v*s*sin(2*PI*h);
			y = v*s*cos(2*PI*h);

			(mat_data+i*mat_step)[j] = 0;
		}
	}

	return 0;
}

*/
void histeq(IplImage* src, IplImage* dst)
{
  IplImage* hsv, * h, * s, * v;

  if (src->nChannels == 3)
  {
    hsv = cvCreateImage(cvGetSize(src), 8, 3);
    h   = cvCreateImage(cvGetSize(src), 8, 1);
    s   = cvCreateImage(cvGetSize(src), 8, 1);
    v   = cvCreateImage(cvGetSize(src), 8, 1);

    cvCvtColor(src, hsv, CV_BGR2HSV);
    cvSplit(hsv, h, s, v, NULL);

    cvEqualizeHist(v, v);

    cvMerge(h, s, v, NULL, hsv);
    //cvCvtColor(hsv, dst, CV_HSV2BGR);
	dst = src;
  }
  else if (src->nChannels == 1)
    cvEqualizeHist(src, dst);

  if (hsv) cvReleaseImage(&hsv);
  if (h) cvReleaseImage(&h);
  if (s) cvReleaseImage(&s);
  if (v) cvReleaseImage(&v);
}


void find_moments(IplImage* imgThreshed, IplImage* imgScribble, int color_hue, int color_num, CvScalar my_BGR)
{
	CvMoments *moments = (CvMoments*)vp_os_malloc(sizeof(CvMoments));

    double moment10;
    double moment01;

	int lastX = obj_p->positions[color_num].lastXY[0];
	int lastY = obj_p->positions[color_num].lastXY[1];
	int posX = obj_p->positions[color_num].posXY[0];
	int posY = obj_p->positions[color_num].posXY[1];

	// the last position becomes the old current position
	obj_p->positions[color_num].lastXY[0] = obj_p->positions[color_num].posXY[0];
	obj_p->positions[color_num].lastXY[1] = obj_p->positions[color_num].posXY[1];

	// find the x,y positions
	cvMoments(imgThreshed, moments, 1);
    moment10 = cvGetSpatialMoment(moments, 1, 0);
    moment01 = cvGetSpatialMoment(moments, 0, 1);
    obj_p->area[color_num] = cvGetCentralMoment(moments, 0, 0);

	// a little thresholding to make sure the positions are only updated for significant color detection
	if (obj_p->area[color_num] >= 100) {
		// the current postion is updated
		obj_p->positions[color_num].posXY[0] = (int) (moment10/obj_p->area[color_num]);
		obj_p->positions[color_num].posXY[1] = (int) (moment01/obj_p->area[color_num]);

		if(lastX>0 && lastY>0 && posX>0 && posY>0)
			{
				// Draw a line from the previous point to the current point
				cvLine(imgScribble, cvPoint(posX, posY), cvPoint(lastX, lastY), my_BGR, 3, 8, 0);
			}
	}

	vp_os_free(moments);
	return;
}

void find_contours(IplImage* imgThreshed, IplImage* imgScribble, int color_hue, int color_num, CvScalar my_BGR)
{
	CvSeq *contours = 0;
	CvRect B_Box;
	CvMemStorage *mem = cvCreateMemStorage(0);
	CvMemStorage *poly = cvCreateMemStorage(0);
	CvMoments *moments = (CvMoments*)vp_os_malloc(sizeof(CvMoments));

	//IplImage* imgTemp = cvCreateImage(cvGetSize(imgThreshed), 32, 1);
	//IplImage* imgEigen = cvCreateImage(cvGetSize(imgThreshed), 32, 1);

	IplImage* img_copy = cvCreateImage(cvGetSize(imgThreshed), IPL_DEPTH_8U, 1);

	//int count = 100;
	int i = 0;

	CvPoint left_side[2];
	CvPoint right_side[2];

	obj_p->left_side[color_num] = 0;
	obj_p->right_side[color_num] = 0;

	cvMoments(imgThreshed, moments, 1);
	obj_p->area[color_num] = cvGetCentralMoment(moments, 0, 0);

	//CvPoint2D32f* corners = vp_os_malloc(sizeof(CvPoint2D32f)*count);

	// a little thresholding to make sure boxes are only drawn for significant colors
	if (obj_p->area[color_num] >= 100) {
		// cvFindContours seems to distort the image you pass to it, so I send it a copy instead.
		cvCopy(imgThreshed, img_copy, NULL);
		cvFindContours(img_copy, mem, &contours, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));

		if(contours)
		{
			contours = find_largest_contour(contours);
			// find a box around the contour and create a rectangle there on imgScribble
			B_Box = cvBoundingRect(contours, 1);
			// clears the thresholded image for everything but the bounding box of the
			// largest detected contour
			focus_ROI(B_Box, imgThreshed);
			// creates a bounding box
			//cvRectangle(imgScribble, cvPoint(B_Box.x, B_Box.y), cvPoint(B_Box.x + B_Box.width, B_Box.y + B_Box.height), my_BGR, 2, 8, 0);
			obj_p->left_area[color_num] = (obj_p->positions[color_num].posXY[0] - B_Box.x)*B_Box.height;
			obj_p->right_area[color_num] = (B_Box.x + B_Box.width - obj_p->positions[color_num].posXY[0])*B_Box.height;

			// approximate the contour as a polygon. At first, I was getting too many verticies leading to a crazy polygon.
			// Somehow, changing the 5th argument (which represents the approximation accuracy) lead to exactly what I wanted,
			// a contour containing only the corners. I found code using cvApproxPoly in the OpenCV examples, square.c.
			contours = cvApproxPoly(contours, sizeof(CvContour), poly, CV_POLY_APPROX_DP, cvContourPerimeter(contours)*0.03, 0);
			cvDrawContours(imgScribble, contours, my_BGR, my_BGR, 0, 2, 8, cvPoint(0,0));

			// I found info on cvGoodFeaturesToTrack() online, but it wasn't really working well. It did seem to
			// detect "strong features" I suppose, but I was looking for the corners specifically.
			// distance = MIN(B_Box.width, B_Box.height);
			// Find corners
			//cvGoodFeaturesToTrack(imgThreshed, imgEigen, imgTemp, corners, &count, 0.1, distance, NULL, 3, 0, 0.04);

			// Mark these corners on the original image
			//for(i=0;i<count;i++)
			//{
			//	//cvLine(imgThreshed, cvPoint(corners[i].x, corners[i].y), cvPoint(corners[i].x, corners[i].y), CV_RGB(255,0,0), 5, 8, 0);
			//	cvDrawCircle( imgScribble, cvPoint((int) corners[i].x, (int) corners[i].y) , 3, my_BGR, 1, 8, 0);
			//}

			if (contours->total == 4) {
				if (fill_pt_array(left_side, right_side, contours, imgScribble, my_BGR)) {
					obj_p->left_side[color_num] = find_pt_dist(left_side);
					obj_p->right_side[color_num] = find_pt_dist(right_side);
				}
			}
		}
	}

	//vp_os_free(corners);
	cvReleaseMemStorage(&mem);
	cvReleaseMemStorage(&poly);
	cvReleaseImage(&img_copy);
	//cvReleaseImage(&imgTemp);
	//cvReleaseImage(&imgEigen);
	vp_os_free(moments);

	return;
}

void focus_ROI(CvRect B_Box, IplImage* imgThreshed)
{
	IplImage* temp_thresh = cvCreateImage(cvGetSize(imgThreshed), IPL_DEPTH_8U, 1);

	cvSetZero(temp_thresh);
	cvSetImageROI(imgThreshed, B_Box);
	cvSetImageROI(temp_thresh, B_Box);
	cvCopy(imgThreshed, temp_thresh, 0);
	cvResetImageROI(temp_thresh);
	cvResetImageROI(imgThreshed);
	cvCopy(temp_thresh, imgThreshed, 0);

	cvReleaseImage(&temp_thresh);

	return;
}

CvSeq* find_largest_contour(CvSeq* contours)
{
	int max = 0;
	CvSeq *ptr;

	while (contours != NULL) {
		if (contours->total > max) {
			ptr = contours;
			max = ptr->total;
		}
		contours = contours->h_next;
	}

	return ptr;
}

// Code borrowed from http://www.bukisa.com/articles/267697_filtering-connected-components
// note: for my purposes, it would probably be easier just to create a function to look for
// the largest contour, but this way is more general it seems. It allows us to filter the
// contours for any criteria we desire. For now, I'll leave this here if we need it and
// implement the other function.
CvSeq* filter_contours(CvSeq* contours)
{
	CvSeq *ptr = contours;

	while (ptr != NULL){	
		if (criteria(ptr) == 0) {
			if (ptr == contours) {
				contours = ptr->h_next;
				ptr = contours;
				ptr->h_prev = NULL;
			} else {
				ptr->h_prev->h_next = ptr->h_next;
				if (ptr->h_next != NULL)
					ptr->h_next->h_prev = ptr->h_prev;
				ptr = ptr->h_next;
			}
		} else {
			ptr = ptr->h_next;
		}
	}
	return contours;
}

int criteria(CvSeq* contour)
{
	static int max = 0;

	if (contour->total > max) {
		max = contour->total;
		return 1;
	}

	return 0;
}

int fill_pt_array(CvPoint left[], CvPoint right[], CvSeq* contours, IplImage* imgScribble, CvScalar my_BGR)
{
	int left_count = 0;
	int right_count = 0;
	int i = 0;

	CvPoint p;

	for (i=0; i<4; i++) {
		p = *(CvPoint*) cvGetSeqElem(contours, i);
		cvDrawCircle( imgScribble, p , 6, my_BGR, 1, 8, 0);

		if (check_left(contours, i)) {
			if (left_count >= 2) {
				return 0;
			}
			left[left_count] = p;
			left_count++;
		} else {
			if (right_count >= 2) {
				return 0;
			}
			right[right_count] = p;
			right_count++;
		}

	}
	return 1;
}

int check_left(CvSeq* contours, int elem)
{
	int matches = 0;
	int i = 0;
	CvPoint target_p;
	CvPoint curr_p = *(CvPoint*) cvGetSeqElem(contours, elem);

	for (i = 0; i<4; i++) {
		if (i != elem) {
			target_p = *(CvPoint*) cvGetSeqElem(contours, i);
			//if (target_p.x == curr_p.x) {
			//	if (target_p.y > curr_p.y) {
			//		return 1;
			//	} else {
			//		return 0;
			//	}
			//} else {
			if (curr_p.x < target_p.x) {
				matches++;
			}
			//}
		}
		if (matches>=2) {
			return 1;
		}
	}

	return 0;
}
double find_pt_dist(CvPoint points[])
{
	double distance = 0;
	int x1 = points[0].x;
	int y1 = points[0].y;
	int x2 = points[1].x;
	int y2 = points[1].y;

	distance = (x1-x2)*(x1-x2)+(y1-y2)*(y1-y2);

	return distance;
}

IplImage* create_overlay(IplImage* imgThreshed, int color_hue, int color_num)
{
	//static IplImage* imgScribble;
	IplImage* imgScribble = cvCreateImage(cvGetSize(imgThreshed), IPL_DEPTH_8U, 3);
	IplImage* test_img = cvCreateImage(cvSize(1,1), IPL_DEPTH_8U, 3);
	CvScalar my_BGR = cvScalar(0, 0, 0, 0);

	// Assumes color_hue value came from MS Paint
	int cv_hue = (int) (color_hue);
	static int create_check = 0;

	/*
	// this part is also related to leaving behind a trail of the marks
	// basically, it makes sure imgScribble is only initialized once
	if (!create_check) {		
		imgScribble = cvCreateImage(cvGetSize(imgThreshed), IPL_DEPTH_8U, 3);
		cvSetZero(imgScribble);
		create_check = 1;
	}
	*/

	// comment this out if you want to leave traces of the marks
	cvSetZero(imgScribble);

	// This next part's sole purpose is to create a BGR color based on the given hue.
	// It's highly convoluted, but it currently works.
	// create a HSV image of the desired color
	test_img->imageData[0] = cv_hue;
	test_img->imageData[1] = 255;
	test_img->imageData[2] = 120;

	// convert that HSV color to BGR
	cvCvtColor(test_img, test_img, CV_HSV2BGR);

	// send that info to my_BGR
	my_BGR.val[0] = test_img->imageData[0];
	my_BGR.val[1] = test_img->imageData[1];
	my_BGR.val[2] = test_img->imageData[2];

	// This part actually finds the xy positions and the contours
	find_contours(imgThreshed, imgScribble, color_hue, color_num, my_BGR);
	if (obj_p->area[color_num] > 100) {
		find_moments(imgThreshed, imgScribble, color_hue, color_num, my_BGR);
	}


	cvReleaseImage(&test_img);
	return imgScribble;
}

IplImage *color_detection(IplImage* img, int color_hue[], int num_colors, int thresh)
{
	IplImage* imgThreshed;
	IplImage* imgScribble;
	IplImage* img_copy = cvCloneImage(img);
	IplImage* graphimg;

	int i = 0;

	//IplImage* imgThreshed_copy;

	// Saving a video stuff
	static CvVideoWriter *writer = 0;
	int isColor = 1;
	int fps     = 15;  // or 30
	int frameW  = 320; // 744 for firewire cameras
	int frameH  = 240; // 480 for firewire cameras
	static int video_on = 0;
	static int frames = 0;

	/*FILE *fp;*/

	static int64 timeStart_com_timer;
	static double timeDiff_com_timer;
	static double timeTally_com_timer = 0;
	static int countTally_com_timer = 0;
	static int started = 0;

	if (!video_on) {
		//writer = cvCreateVideoWriter("ColorDemo.avi", CV_FOURCC('F', 'L', 'V', '1'), fps, cvSize(frameW,frameH), isColor);
		writer = cvCreateVideoWriter("FollowingDemo.avi", 0, fps, cvSize(frameW,frameH), isColor);
		if (writer == NULL) {
			exit(-1);
		}
		video_on = 1;
	}


	for (i = 0; i<num_colors; i++)
	{
		// check the image for matching pixels for the i-th color
		imgThreshed = threshold_colors(img_copy, color_hue[i], thresh);

		// creates visual markers of the centroid and boxed location
		imgScribble = create_overlay(imgThreshed, color_hue[i], i);
		if (i == 0)
		{
			// show the thresholded result
			cvNamedWindow("threshed", CV_WINDOW_AUTOSIZE);
			cvShowImage("threshed", imgThreshed);

			/* // save images of the threshed image
			imgThreshed_copy = cvCloneImage(img);
			cvCvtColor(imgThreshed, imgThreshed_copy, CV_GRAY2BGR);
			cvSaveImage("ThreshedImage.bmp", imgThreshed_copy, 0);
			cvSaveImage("Image.bmp", img, 0);
			cvReleaseImage(&imgThreshed_copy);
			*/
		}

		if (i == 1)
		{
			// show the thresholded result
			cvNamedWindow("threshed2", CV_WINDOW_AUTOSIZE);
			cvShowImage("threshed2", imgThreshed);
		}

		cvDrawCircle(imgScribble, cvPoint(159, 119), 1, cvScalar(0, 0, 255, 0), 2, 8, 0);
		// adds the visual markers to the img
		cvAdd(img, imgScribble, img, NULL);

		cvReleaseImage(&imgThreshed);
		cvReleaseImage(&imgScribble);
	}

	//cvSaveImage("RedError.bmp", img, 0);

	if (!started) {
		START_TIMING(com_timer);
		started = 1;
	}

	if (vec_p->index < vec_p->size) {
		pthread_mutex_lock(&fd_mutex);

		vec_p->roll[vec_p->index] = fd->roll;
		STOP_TIMING(com_timer);
		time2 = GET_TIMING(com_timer)/1000.0;
		vec_p->time[vec_p->index] = (float) GET_TIMING(com_timer)/1000;
		vec_p->pos[vec_p->index] = (float) obj_p->positions[1].posXY[0];
		//vec_p->gaz[vec_p->index] = fd->gaz;
		//vec_p->pitch[vec_p->index] = fd->pitch;
		//vec_p->yaw[vec_p->index] = fd->yaw;
		(vec_p->index)++;

		pthread_mutex_unlock(&fd_mutex);
	}
	setGraphColor(0);

	if (frames < 900) {
		drawFloatGraph(vec_p->roll, vec_p->size, img, -1, 1, img->width, img->height, "Roll Function", 1);
		//drawFloatGraph(vec_p->time, vec_p->size, img, -1, 70, img->width, img->height, "Time Function", 1);
		//drawFloatGraph(vec_p->pos, vec_p->size, img, -1, 330, img->width, img->height, "Position Function", 1);
		cvWriteFrame(writer,img);
		frames++;
	} else if (frames == 900) {
		cvReleaseVideoWriter(&writer);
		graphimg = cvCreateImage(cvSize(640, 320), IPL_DEPTH_8U, 3);
		cvSet(graphimg, cvScalarAll(255), NULL);
		drawFloatGraph(vec_p->roll, vec_p->size, graphimg, -1, 1, graphimg->width, graphimg->height, "Roll Function", 1);
		//drawFloatGraph(vec_p->time, vec_p->size, graphimg, -1, 70, graphimg->width, graphimg->height, "Time Function", 1);
		//drawFloatGraph(vec_p->pos, vec_p->size, graphimg, -1, 330, graphimg->width, graphimg->height, "Position Function", 1);
		cvSaveImage("Roll.jpg", graphimg, 0);
		cvReleaseImage(&graphimg);

		//if((fp=fopen("array_test.txt", "w"))==NULL) {
		//	exit(-1);
		//}

		//for (i = 0; i<vec_p->size; i++) {
		//	fprintf(fp, "%f %f\n", vec_p->pos[i], vec_p->time[i]);
		//}

		//fclose(fp);

		frames++;
	}

	cvReleaseImage(&img_copy);
	return img;
}

IplImage* motion_detection(IplImage* img)
{
	// Fuction to detect the difference between two frames. I'm not really
	// using it right now, since it's kind of useless, but it could be a
	// starting point for some kind of optical flow algorithm.
	static IplImage* last_img;
	static img_created = 0;

	IplImage* img_diff = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
	IplImage* img_gray = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);

	// I need the grayscale image here.
	cvCvtColor(img, img_gray, CV_BGR2GRAY);

	// I only want the last_img initialized once, to prevent memory leaks.
	// Actually, I never free this memory, so this is probably an example
	// of a memory leak.
	if (!img_created) {
		last_img = cvCloneImage(img_gray);
		img_created = 1;
	}

	// Subtract the two images and threshold a little.
	cvSub(img_gray, last_img, img_diff, NULL);
	cvThreshold(img_diff, img_diff, 50, 255, CV_THRESH_TOZERO);

	find_motion_vector(img_diff);

	cvNamedWindow("Image Difference", CV_WINDOW_AUTOSIZE);
	cvShowImage("Image Difference", img_diff);

	// I release the last image since I don't need it anymore,
	// but the I reassign it to the newest grayscale image.
	cvReleaseImage(&last_img);
	last_img = img_gray;
	cvReleaseImage(&img_diff);
	
	return 0;
}

IplImage* find_motion_vector(IplImage* img_diff)
{
	// This is basically the exact same thing I was doing in the color
	// detection section. Basically, I'm just drawing a line from the
	// last centroid to the new centroid.
	CvMoments *moments = (CvMoments*)vp_os_malloc(sizeof(CvMoments));

    double moment10;
    double moment01;
	double area;

	static int posX = 0;
	static int posY = 0;

	int lastX = posX;
	int lastY = posY;

	cvMoments(img_diff, moments, 1);
    moment10 = cvGetSpatialMoment(moments, 1, 0);
    moment01 = cvGetSpatialMoment(moments, 0, 1);
    area = cvGetCentralMoment(moments, 0, 0);

	if (area >= 0) {
		posX = (int) (moment10/area);
		posY = (int) (moment01/area);

		if(lastX>0 && lastY>0 && posX>0 && posY>0)
		{
			// Draw a line from the previous point to the current point
			cvLine(img_diff, cvPoint(posX, posY), cvPoint(lastX, lastY), cvScalar(128, 128, 128, 0), 3, 8, 0);
		}
	}


	vp_os_free(moments);
	return 0;
}

int create_bg_model(IplImage* current_img)
{
	// background is a pointer to integer pointers
	// basically, it holds an array of integer pointers, which point to an array of
	// one channel's values
	static int** background;
	static int created = 0;
	static int frame_num = 0;
	int i = 0;
	uchar* data;
	bg_model* bg_p = &new_model;

	IplImage *img_copy = cvCloneImage(current_img);

	// I'm being pretty specific here, too. I'm assuming that the image
	// is a 320x240 image with 3 channels for a total of 230400 values.
	// The number of samples can be changed pretty easily, but not the
	// image size, at least not yet.
	if (bg_progress == 0) {
		background = (int**) vp_os_malloc(sizeof(int)*230400);
		for (i = 0; i<230400; i++) {
			background[i] = (int*) vp_os_malloc(sizeof(int)*NUM_SAMPLES);
		}
		bg_progress = 1;
	}

	// bg_progress is just a way of marking my way through the "learning"
	// process. As the function starts filling the background arrays with data,
	// bg_progress is set to 1. I wanted to have some sort of global variable
	// that could be seen across files so that I could reset with value and start
	// recording another background, but currently I can't send commands through
	// gamepad.cpp. It just can't recognize the variable for some reason.
	if (bg_progress == 1) {

		//cvSmooth(img_copy, img_copy, CV_GAUSSIAN, 3, 0, 11, 11);
		// set a pointer to the image data of the newly created image
		data = img_copy->imageData;

		// this loop is pretty simple, just set the new data points to the correct
		// background pixel for the given frame number
		for(i = 0; i<230400; i++) {
			background[i][frame_num] = data[i];
		}

		frame_num++;
		
		// if the number of frames has reached the number of samples, then I need 
		// to do a few things. First, calculate the mean image from the data. Second,
		// calculate the variance in the background. 

		// A few quick words on the variance. Variance is just the standard deviation squared.
		// Taking the square root of values seems to take a long time so I opted to leave it as
		// the variance instead. This means that later when I'm comparing the actual image with
		// the background, I need to square the actual image's values when making the comparison.
		// Also, here, I'm multiplying the variance by 16 to make sure in the end, I get pixels
		// which are more than 4 (sqrt(16)) standard deviations away, if that makes any sense.
		// The values for the pictures have to be ints, so I'm probably introducing a lot of error.
		if (frame_num == NUM_SAMPLES) {
			for (i = 0; i<230400; i++) {
				bg_p->mean_img->imageData[i] = (int) find_mean(background[i], NUM_SAMPLES);
				bg_p->var_img->imageData[i] = (int) find_variance(background[i], NUM_SAMPLES, bg_p->mean_img->imageData[i])*16;
			}

			// After setting the two pictures to the corresponding mean and variance values,
			// I no longer need the background info so I release it.
			for (i = 0; i<NUM_SAMPLES; i++)
			{
			  vp_os_free(background[i]);
			}
			vp_os_free(background);

			frame_num = 0;
			bg_progress = 2;
		}
	}

	cvReleaseImage(&img_copy);

	if (bg_progress == 2) {
		return 1;
	}

	return 0;
}

double find_mean(int pix_samp[], double size)
{
	// A simple function to find the mean of an array, given its size.
	// It will segfault if you pass a size greater than the array size.
	// Also note that sum is used for both the sum and the final mean size.
	int i = 0;
	double sum = 0;

	for (i = 0; i<size; i++) {
		sum += pix_samp[i];
	}
	sum = (sum/size);

	return sum;
}

double find_variance(int pix_samp[], int size, double mean)
{
	// Function to find the variance.
	// I'm not sure about all the type casting. I just hope I don't
	// lose too much information
	double variance = 0;
	int i = 0;

	for (i = 0; i<size; i++) {
		variance += ((double)pix_samp[i]-mean)*((double)pix_samp[i]-mean);
	}

	variance = (variance/size);

	return variance;
}

IplImage* remove_bg(IplImage* current_img)
{
	// After the background model is completed, we can start subtracting that
	// from our video feed
	bg_model* bg_p = &new_model;
	IplImage* foreground = cvCreateImage(cvGetSize(current_img), IPL_DEPTH_8U, 1);
	IplImage* img_check = cvCloneImage(current_img);

	//IplImage* color_img = cvCloneImage(current_img);
	//IplImage* vid_frame = cvCreateImage(cvSize(640, 240), IPL_DEPTH_8U, 3);
	//// Saving a video stuff
	//static CvVideoWriter *writer = 0;
	//int isColor = 1;
	//int fps     = 15;  // or 30
	//int frameW  = 640; // 744 for firewire cameras
	//int frameH  = 240; // 480 for firewire cameras
	//static int video_on = 0;
	//static int frames = 0;
	//int i = 0;
	//int j = 0;
	//int k = 0;

	//if (!video_on) {
	//	writer = cvCreateVideoWriter("Bg_Sub.avi", 0, fps, cvSize(frameW,frameH), isColor);
	//	if (writer == NULL) {
	//		exit(-1);
	//	}
	//	video_on = 1;
	//}

	cvSetZero(foreground);

	//cvSmooth(img_check, img_check, CV_GAUSSIAN, 3, 0, 11, 11);

	// Take the absolute difference between the current image
	// and the mean image.
	cvAbsDiff(img_check, bg_p->mean_img, img_check);

	// Square the difference (I guess this part makes the absolute
	// part of the last line useless). Subtract the variance. If 
	// the difference squared is greater than the variance*16, then
	// the pixel probably isn't part of the background. Because of that, I
	// find all values greater than 0 and put the values in a grayscale
	// map called foreground. 
	cvMul(img_check, img_check, img_check, 1);
	cvSub(img_check, bg_p->var_img, img_check, NULL);
	cvInRangeS(img_check, cvScalar(1, 1, 1, 0), cvScalar(256, 256, 256, 0), foreground);

	// A little smoothing of the final image.
	cvSmooth(foreground, foreground, CV_GAUSSIAN, 3, 0, 11, 11);
	cvThreshold(foreground, foreground, 150, 255, CV_THRESH_TOZERO);


	cvNamedWindow("Foreground", CV_WINDOW_AUTOSIZE);
	cvShowImage("Foreground", foreground);

	//cvCvtColor(foreground, color_img, CV_GRAY2BGR);

	//for (i = 0; i<240; i++) {
	//	for (j = 0; j<320; j++) {
	//		for (k = 0; k<3; k++) {
	//			vid_frame->imageData[i*3*640 + j*3 + k] = color_img->imageData[i*3*320 + j*3 + k];
	//		}
	//	}
	//	for (j = 320; j<640; j++) {
	//		for (k = 0; k<3; k++) {
	//			vid_frame->imageData[i*3*640 + j*3 + k] = current_img->imageData[i*3*320 + (j-320)*3 + k];
	//		}
	//	}
	//}

	//if (frames < 450) {
	//	cvWriteFrame(writer, vid_frame);
	//	frames++;
	//} else if (frames == 450) {
	//	cvReleaseVideoWriter(&writer);
	//	frames++;
	//	exit(-1);
	//}

	//cvReleaseImage(&color_img);
	//cvReleaseImage(&vid_frame);

	cvReleaseImage(&foreground);
	cvReleaseImage(&img_check);
	return 0;
}

void *drone_c_thread(void *arg)
{
	int color_num = (int) arg;
	double area = obj_p->area[color_num];
	double scaled = sqrt(area) - 27;
	double min_area = 100;
	int x = obj_p->positions[color_num].posXY[0] - 160;
	int y = 120 - obj_p->positions[color_num].posXY[1];
	double left = obj_p->left_side[color_num];
	double right = obj_p->right_side[color_num];
	//double left = obj_p->left_area[color_num];
	//double right = obj_p->right_area[color_num];
	double ratio = find_side_ratio(left, right);
	int i = 0;
	int j = 0;

	static int64 timeStart_my_timer;
	static double timeDiff_my_timer;
	static double timeTally_my_timer = 0;
	static int countTally_my_timer = 0;
	static int64 timeStart_timer2;
	static double timeDiff_timer2;
	static double timeTally_timer2 = 0;
	static int countTally_timer2 = 0;
	static int started = 0;

	static Flight_Data last_cmd = {0, 0, 0, 0, 0, 0, 0};
	Flight_Data* last_p = &last_cmd;
	static Flight_Data cmd = {0, 0, 0, 0, 0, 0, 0};
	Flight_Data* curr_c = &cmd;


	int reset[4] = {0};
	float alpha = (float) 0.2;

	pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
	pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);

	pthread_mutex_lock(&fd_mutex);

	if (fabs(fd->roll) <= 0.05) {
		reset[0] = 1;
	}
	if (fabs(fd->gaz) <= 0.05) {
		reset[1] = 1;
	}
	if (fabs(fd->pitch) <= 0.05) {
		reset[2] = 1;
	}
	//if (fabs(fd->yaw) <= 0.05) {
	//	reset[3] = 1;
	//}

	fd->gaz = last_p->gaz;
	fd->pitch = last_p->pitch;
	fd->roll = last_p->roll;
	fd->yaw = last_p->yaw;

	pthread_mutex_unlock(&fd_mutex);

	if (area < min_area) {
		started = 0;

		for (j = 0; j<3; j++) {
			for (i = 0; i<20; i++) {
				ardrone_at_set_progress_cmd(1 , last_p->roll, last_p->pitch, last_p->gaz, last_p->yaw);
			}
			ardrone_at_send();
			cvWaitKey(1);
		}

		last_p->gaz = (float) 0.94*last_p->gaz;
		last_p->pitch = (float) 0.94*last_p->pitch;
		last_p->roll = (float) 0.94*last_p->roll;
		last_p->yaw = (float) 0.94*last_p->yaw;

		pthread_exit(NULL);
	}

	if (!started) {
		START_TIMING(my_timer);
		started = 1;
	}
	STOP_TIMING(my_timer);
	//mytime = GET_AVERAGE_TIMING(my_timer);
	mytime = GET_TIMING(my_timer)/1000.0;


	//curr_c->roll = dir_PID(x, mytime, reset[0], 0);
	//fd->gaz = dir_PID(y, mytime, reset[1], 1);
	//fd->pitch = dir_PID(scaled, mytime, reset[2], 2);
	curr_c->roll = (float) (alpha*last_p->roll + (1-alpha)*dir_PID(x, mytime, reset[0], 0));
	curr_c->gaz = (float) (alpha*last_p->gaz + (1-alpha)*dir_PID(y, mytime, reset[1], 1));
	curr_c->pitch = (float) (alpha*last_p->pitch + (1-alpha)*dir_PID(scaled, mytime, reset[2], 2));

	last_p->gaz = curr_c->gaz;
	last_p->pitch = curr_c->pitch;
	last_p->roll = curr_c->roll;
	last_p->yaw = curr_c->yaw;

	//if (ratio != 0) {
	//	fd->yaw = dir_PID(ratio, mytime, reset[3], 3);
	//}

	//fd->yaw = compare_sides(left, right, 3);
	
	//fd->pitch = (float) (0.4*weighted_response(40, scaled));

	START_TIMING(my_timer);
	pthread_mutex_lock(&fd_mutex);

	fd->gaz = curr_c->gaz;
	fd->pitch = curr_c->pitch;
	fd->roll = curr_c->roll;
	fd->yaw = curr_c->yaw;

	pthread_mutex_unlock(&fd_mutex);

	//if ( (fabs(fd->roll) + fabs(fd->pitch) + fabs(fd->yaw) + fabs(fd->gaz)) != 0 )
	//{	
	//	//22, sleep of 3
	//	//88, sleep of 1, once
	//	//40, sleep of 3, 3x
	//	//18, sleep of 3, 3x
	//	for (i = 0; i<26; i++)
	//	{
	//		ardrone_at_set_progress_cmd(1 , fd->roll, fd->pitch, fd->gaz, fd->yaw);
	//		ardrone_at_set_progress_cmd(1 , fd->roll, fd->pitch, fd->gaz, fd->yaw);
	//		ardrone_at_set_progress_cmd(1 , fd->roll, fd->pitch, fd->gaz, fd->yaw);
	//		ardrone_at_set_progress_cmd(1 , fd->roll, fd->pitch, fd->gaz, fd->yaw);
	//		Sleep(2);
	//		//bg_progress++;
	//	}
	//}
	pthread_testcancel();
	if ( (fabs(curr_c->roll) + fabs(curr_c->pitch) + fabs(curr_c->yaw) + fabs(curr_c->gaz)) != 0 ) {
		while (1) {
			START_TIMING(timer2);
			
			for (i = 0; i<30; i++) {
				ardrone_at_set_progress_cmd(1 , curr_c->roll, curr_c->pitch, curr_c->gaz, curr_c->yaw);
			}
			ardrone_at_send();
			//ardrone_at_set_progress_cmd(0 , curr_c->roll, curr_c->pitch, curr_c->gaz, curr_c->yaw);
			//Sleep(1);
			cvWaitKey(1);
			pthread_testcancel();
			STOP_TIMING(timer2);
			time3 = GET_TIMING(timer2)/1000.0;
		}
	}

	pthread_exit(NULL);
	return 0;
}

float dir_PID(double value, double dt, int reset, int dir)
{
	static double prev_err[4] = {0, 0, 0, 0};
	static double integral[4] = {0, 0, 0, 0};
	double derivative;
	double output;
	// yaw based on sides
	//double Kp[4] = {6.23/1600.0, 10.5/1200.0, 4.5/270.0, 1.1};
	//double Ki[4] = {0.235/640000.0, 0.20/640000.0, 0.2/640000, 2.0/262144.0};
	//// yaw based on area
	////double Kp[4] = {5.85/1600.0, 10.0/1200.0, 4.0/270.0, 4.0};
	////double Ki[4] = {0.245/640000.0, 0.06/640000.0, 0.072/640000, 4.0/262144.0};
	//double Kd[4] = {3.65, 4.57, 3.45, 0};

	//double Kp[4] = {5.23/160.0, 10.5/1200.0, 4.5/270.0, 1.1};
	//double Ki[4] = {2.35/64000000.0, 0.20/640000.0, 0.2/640000, 2.0/262144.0};
	//double Kd[4] = {30.65, 4.57, 3.45, 0};

	double Kp[4] = {0.0112910927646499/4.5, 10.5/1200.0, 4.5/270.0, 1.1};
	double Ki[4] = {0.000266660681223664, 0.20/640000.0*1000, 0.2/640000*1000, 2.0/262144.0};
	double Kd[4] = {0.00511547774396304/2.0, 4.57/1000, 3.45/1000, 0};
	double N = 4.669963878216;
	double scale = 1.0/(N*dt+1);
	static double last_d[4] = {0, 0, 0, 0};

	double max_i[4] = {1.0/Ki[0], 1.0/Ki[1], 1.0/Ki[2], 1.0/Ki[3]};
	double min_i = -max_i[dir];
	double error = value;
	int left = 0;

	// old settings p = 6.23, d = 3.65, i = 0.235/640000
	// ^ (no weighting with previous average)
	// experimental settings p = 8.25, d = 4.65, i = 0.235
	// ^ (weighted with previous average)

	if (reset) {
		integral[dir] = 0;
	}

	if (dir == 3) {
		if (value<0) {
			left = 1;
			error += 1;
		} if (value>0) {
			error -= 1;
		}
	}

	integral[dir] = integral[dir] + (error*dt);
	if (integral[dir] > max_i[dir]) {
		integral[dir] = max_i[dir];
	} else if (integral[dir] < min_i) {
		integral[dir] = min_i;
	}
	//derivative = (error - prev_err[dir])/dt;
	derivative = scale*last_d[dir] + Kd[dir]*N*scale*(error - prev_err[dir]);
	last_d[dir] = derivative;
	output = ((Kp[dir]*error) + (Ki[dir]*integral[dir]) + derivative);
	prev_err[dir] = error;

	if (output>1) {
		output = 1;
	} else if (output<-1) {
		output = -1;
	}

	return (float) output;
}

float total_response(float speed, int max, double value)
{
	float result = 0;

	if ( fabs(speed) > 100 ) {
		if ( (fabs(value) - 0.1*max) < 0) {
			result = 0.5;
			if (speed > 0) {
				result *= -1;
			}
		}
	} else {
		result = weighted_response(max, value);
	}

	return result;
}

float weighted_response(int max, double value)
{
	float result;
	float middle = 0.5;
	double temp_val = fabs(value);

	result = (float) (temp_val/max);

	if (result>1) {
		result = 1;
	}

	if (value<0) {
		result *= -1;
	}

	return result;
}

double find_side_ratio(double left, double right)
{
	double top = MAX(left, right);
	double bot = MIN(left, right);
	double ratio;

	if (top == bot) {
		return 0;
	}

	ratio = top/bot;

	if (top == left) {
		ratio *= -1;
	}

	return ratio;
}

float compare_sides(double left, double right, double max)
{
	double top = MAX(left, right);
	double bot = MIN(left, right);
	double ratio;

	float result;

	if (top == bot) {
		return 0;
	}

	ratio = top/bot;

	if (ratio < 1.1) {
		return 0;
	}

	//result = (float) ( (ratio-0.5)/max);
	result = (float) (1.1*(ratio)/max);
	
	if (result>1) {
		result = 1;
	}

	if (top == left) {
		result *= -1;
	}

	//fd->roll = (float) -0.3*result;

	return result;
}

void MouseHandler(int event, int x, int y, int flags, void *param)
{
	IplImage* HSV = cvCreateImage(cvSize(1,1), IPL_DEPTH_8U, 3);
	IplImage* img = (IplImage*) param;
	IplImage* color = cvCreateImage(cvSize(150, 150), IPL_DEPTH_8U, 3);

	if (event == CV_EVENT_LBUTTONDOWN) {
		BGR = cvGet2D(img, y, x);
		HSV->imageData[0] = (int) BGR.val[0];
		HSV->imageData[1] = (int) BGR.val[1];
		HSV->imageData[2] = (int) BGR.val[2];
		cvCvtColor(HSV, HSV, CV_BGR2HSV);

		cvSet(color, BGR, 0);

		BGR.val[0] = (double) HSV->imageData[0];
		BGR.val[1] = (double) HSV->imageData[1];
		BGR.val[2] = (double) HSV->imageData[2];

		cvNamedWindow("Color", CV_WINDOW_AUTOSIZE);
		cvShowImage("Color", color);

		cvReleaseImage(&color);
		cvReleaseImage(&HSV);
	}


	return;
}