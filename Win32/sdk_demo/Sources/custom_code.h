/********************************************************************
 *                    COPYRIGHT PARROT 2010
 ********************************************************************
 *       PARROT - A.R.Drone SDK Windows Client Example
 *-----------------------------------------------------------------*/
/**
 * @file custom_code.h 
 * @brief Header file for user-added code.
 *
 * @author Stephane Piskorski <stephane.piskorski.ext@parrot.fr>
 * @date   Sept, 8. 2010
 *
 *******************************************************************/


#ifndef _MYKONOS_TESTING_TOOL_H_



#include <cv.h>
#include <cvaux.h>
#include <highgui.h>
#include <cxcore.h>
//#include <GL/gl.h>
//#include <GL/glut.h>
//#include <AR/gsub.h>
//#include <AR/video.h>
//#include <AR/param.h>
//#include <AR/ar.h>

struct Custom_Navdata
{
	// these arrays hold the past 100 values of the distance to the tag, the x and y coordinate of the tag on the camera,
	// and the velocity along the y axis (side to side movement)
	float distances[100];
	float xs[100];
	float ys[100];
	float vys[100];
	float vxs[100];
	int index;
};
extern struct Custom_Navdata Custom_Navdata_t;
extern struct Custom_Navdata* const CN;


typedef struct _Flight_Data
{
	float pitch;
	float roll;
	float yaw;
	float gaz;
	float vx;
	float vy;
	float vz;
} Flight_Data;
extern Flight_Data Flight_Data_t;
extern Flight_Data* const fd;


#define _MYKONOS_TESTING_TOOL_H_






#ifdef __cplusplus
extern "C" {
#endif

#define DECLARE_TIMING(s)  int64 timeStart_##s; double timeDiff_##s; double timeTally_##s = 0; int countTally_##s = 0
#define START_TIMING(s)    timeStart_##s = cvGetTickCount()
#define STOP_TIMING(s) 	   timeDiff_##s = (double)(cvGetTickCount() - timeStart_##s); timeTally_##s += timeDiff_##s; countTally_##s++
#define GET_TIMING(s) 	   (double)(timeDiff_##s / (cvGetTickFrequency()*1000.0))
#define GET_AVERAGE_TIMING(s)   (double)(countTally_##s ? timeTally_##s/ ((double)countTally_##s * cvGetTickFrequency()*1000.0) : 0)
#define CLEAR_AVERAGE_TIMING(s) timeTally_##s = 0; countTally_##s = 0

#include <stdio.h>
#include <VP_Os/vp_os_types.h>

#include <ardrone_tool/Navdata/ardrone_navdata_client.h>
#include <Navdata/navdata.h>

#include <pthread.h>
#include <GraphUtils.h>


//#include <cv.h>
//#include <cvaux.h>
//#include <highgui.h>
//#include <cxcore.h>


C_RESULT signal_exit();

void ARWin32Demo_SetConsoleCursor(int x,int y);
void ARWin32Demo_AcquireConsole();
void ARWin32Demo_ReleaseConsole();



#ifdef __cplusplus
}
#endif

// This struct will hold the background information
#define NUM_SAMPLES 150

typedef struct _bg_model
{
	IplImage* mean_img;
	IplImage* var_img;
} bg_model;

extern bg_model new_model;


// These structs contain information about the centroids of the detected colors
#define MAX_COLORS 2
#define SV_LOWER 80

typedef struct _obj_pos
{
	int posXY[2];
	int lastXY[2];
} obj_pos;

typedef struct _object_info
{
	double area[MAX_COLORS];
	obj_pos positions[MAX_COLORS];
	double left_side[MAX_COLORS];
	double right_side[MAX_COLORS];
	double left_area[MAX_COLORS];
	double right_area[MAX_COLORS];
} object_info;
extern object_info new_obj;
extern object_info* obj_p;

typedef struct _dir_vector
{
	float roll[900];
	float gaz[900];
	float pitch[900];
	float yaw[900];
	float time[900];
	float pos[900];
	int index;
	int size;
} dir_vector;
extern dir_vector new_vec;
extern dir_vector* vec_p;

// Global Variables
extern long color_check;
extern long num_hits;
extern const int THRESHOLD[3];
extern const int MAX_DATA;
extern int bg_progress;
extern pthread_mutex_t fd_mutex;
extern CvScalar BGR;
extern double mytime;
extern double time2;
extern double time3;
extern double xyz[3];



float weighted_avg(float seq[], int ind, int size);
float throttle_val(float value, float end);

// Tag tracking using the Drone's own tag system
int Drone_Tag_Tracking(const navdata_unpacked_t* const navdata);

// Custom code that uses the raw pixel input and searches for color
int Color_Control(double *Target_RGBs[], int colors, double tolerance, uint8_t* pixbuf_data, long pic_size, long hits[], int* hit_map[]);

// functions called by Color_Control, takes raw buffer and searchs for the target RGBs.
// Also populates a map of matching pixels
int find_color(uint8_t* pixbuf_data, long size, double *Target_RGB[], int colors, long hits[], int* hit_map[]);
float RGB_in_Range(double RGB_value[], double Target_RGB[]);
//double find_hue(double RGB[]);
int normalize_RGB(double RGB[]);
// finds the xy centroid of a map of matching pixels
int find_xy_centroid(int hit_map[], int width, int height, int hits, int xy[]);

// thread functions used by Color_Control when searching through the buffer
void *col_sum(void *arg);
void *row_sum(void *arg);
void *PrintHello(void *threadid);
void *search_row(void *arg);

// Color detection functions using OpenCV library
// The following functions convert raw buffer data to OpenCV image, threshold
// the image for certain colors, find the moments and contours, and creates an
// overlay to place over the image.
IplImage* convert_to_CVimg(uint8_t* pixbuf_data);
IplImage* threshold_colors(IplImage* img, int color_hue, int thresh);

//IplImage* dist_thresh(IplImage* img, int color_hue);
//CvMat* dist_mat(IplImage* HSV, CvScalar tar_HSV);
//CvMat* fill_dist_mat(IplImage* HSV, CvMat* dist_mat);

void histeq(IplImage* src, IplImage* dst);
void find_moments(IplImage* imgThreshed, IplImage* imgScribble, int color_hue, int color_num, CvScalar my_BGR);

void find_contours(IplImage* imgThreshed, IplImage* imgScribble, int color_hue, int color_num, CvScalar my_BGR);
void focus_ROI(CvRect B_Box, IplImage* imgThreshed);
CvSeq* find_largest_contour(CvSeq* contours);
CvSeq* filter_contours(CvSeq* contours);
int criteria(CvSeq* contour);
int fill_pt_array(CvPoint left[], CvPoint right[], CvSeq* contours, IplImage* imgScribble, CvScalar my_BGR);
int check_left(CvSeq* contours, int elem);
double find_pt_dist(CvPoint points[]);

IplImage* create_overlay(IplImage* imgThreshed, int color_hue, int color_num);
IplImage* color_detection(IplImage* img, int color_hue[], int num_colors, int thresh);

// Motion detection functions using OpenCV library
// simply subtracts the previous frame and displays grayscale result
IplImage* motion_detection(IplImage* img);
IplImage* find_motion_vector(IplImage* img_diff);

// Motion detection by subtracting the background
// takes 10 seconds to form the averages for the background.
// Then subtracts the result from subsequent frames and displays
// the result.
int create_bg_model(IplImage* current_img);
double find_mean(int pix_samp[], double size);
double find_variance(int pix_samp[], int size, double mean);
IplImage* remove_bg(IplImage* current_img);

// side thread called by video_stage.c. Takes information from
// obj_p pointer and sets the drone to move accordingly. Argument
// is the number of which color being used.
void *drone_c_thread(void *arg);
float dir_PID(double value, double dt, int reset, int dir);
float total_response(float speed, int max, double value);
float weighted_response(int max, double value);
double find_side_ratio(double left, double right);
float compare_sides(double left, double right, double max);

void MouseHandler(int event, int x, int y, int flags, void *param);


#endif // _MYKONOS_TESTING_TOOL_H_
