/********************************************************************
 *                    COPYRIGHT PARROT 2010
 ********************************************************************
 *       PARROT - A.R.Drone SDK Windows Client Example
 *-----------------------------------------------------------------*/
/**
 * @file video_stage.c 
 * @brief Video stream reception code
 *
 * @author marc-olivier.dzeukou@parrot.com
 * @date 2007/07/27
 *
 * @author Stephane Piskorski <stephane.piskorski.ext@parrot.fr>
 * @date   Sept, 8. 2010
 *
 *******************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <custom_code.h>

#include <time.h>

/* A.R.Drone OS dependant includes */
	#include <config.h>
	#include <VP_Os/vp_os_print.h>
	#include <VP_Os/vp_os_malloc.h>
	#include <VP_Os/vp_os_delay.h>

/* A.R.Drone Video API includes */
	#include <VP_Api/vp_api.h>
	#include <VP_Api/vp_api_error.h>
	#include <VP_Api/vp_api_stage.h>
	#include <VP_Api/vp_api_picture.h>
	#include <VP_Stages/vp_stages_io_file.h>
	#include <VP_Stages/vp_stages_i_camif.h>
	#include <VLIB/Stages/vlib_stage_decode.h>
	#include <VP_Stages/vp_stages_yuv2rgb.h>
	#include <VP_Stages/vp_stages_buffer_to_picture.h>

/* A.R.Drone Tool includes */
	#include <ardrone_tool/ardrone_tool.h>
	#include <ardrone_tool/Com/config_com.h>
	#include <ardrone_tool/UI/ardrone_input.h>
	#include <ardrone_tool/Video/video_com_stage.h>

/* Configuration file */
	#include <win32_custom.h>

/* Our local pipeline */
	#include "Video/video_stage.h"
	
#include <UI/directx_rendering.h>


 //   #include <cv.h>
 //   #include <cvaux.h>
 //   #include <highgui.h>
	//#include <cxcore.h>

	//#include "vidnav.h"

/* Global variables to build our video pipeline*/
	#define NB_STAGES 10
	PIPELINE_HANDLE pipeline_handle;
	static uint8_t*  pixbuf_data       = NULL;
	static vp_os_mutex_t  video_update_lock;
	pthread_t control_thread;








/*****************************************************************************/


BYTE* ConvertRGBToBMPBuffer ( BYTE* Buffer, int width, int height, long* newsize )
{
	int padding = 0;
	int scanlinebytes = width * 3;
	long bufpos = 0;   
	long newpos = 0;
	int x, y;
	int psw;
	BYTE* newbuf;

	// first make sure the parameters are valid
	if ( ( NULL == Buffer ) || ( width == 0 ) || ( height == 0 ) )
		return NULL;

	// now we have to find with how many bytes
	// we have to pad for the next DWORD boundary	


	while ( ( scanlinebytes + padding ) % 4 != 0 )     // DWORD = 4 bytes
		padding++;
	// get the padded scanline width
	psw = scanlinebytes + padding;
	
	// we can already store the size of the new padded buffer
	*newsize = height * psw;

	// and create new buffer
	newbuf = (BYTE*)vp_os_malloc(*newsize);
	
	// fill the buffer with zero bytes then we dont have to add
	// extra padding zero bytes later on
	vp_os_memset ( newbuf, 0, *newsize );

	// now we loop trough all bytes of the original buffer, 
	// swap the R and B bytes and the scanlines

	for (y = 0; y < height; y++ )
		for (x = 0; x < 3 * width; x+=3 )
		{
			bufpos = y * 3 * width + x;     // position in original buffer
			newpos = ( height - y - 1 ) * psw + x;           // position in padded buffer

			newbuf[newpos] = Buffer[bufpos+2];       // swap r and b
			newbuf[newpos + 1] = Buffer[bufpos + 1]; // g stays
			newbuf[newpos + 2] = Buffer[bufpos];     // swap b and r
		}

	return newbuf;
}

int SaveBMP ( BYTE* Buffer, int width, int height, long paddedsize, LPCTSTR bmpfile )
{
	// declare bmp structures 
	BITMAPFILEHEADER bmfh;
	BITMAPINFOHEADER info;
	HANDLE file;
	unsigned long bwritten;
	
	// andinitialize them to zero
	vp_os_memset ( &bmfh, 0, sizeof (BITMAPFILEHEADER ) );
	vp_os_memset ( &info, 0, sizeof (BITMAPINFOHEADER ) );
	
	// fill the fileheader with data
	bmfh.bfType = 0x4d42;       // 0x4d42 = 'BM'
	bmfh.bfReserved1 = 0;
	bmfh.bfReserved2 = 0;
	bmfh.bfSize = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER) + paddedsize;
	bmfh.bfOffBits = 0x36;		// number of bytes to start of bitmap bits
	
	// fill the infoheader

	info.biSize = sizeof(BITMAPINFOHEADER);
	info.biWidth = width;
	info.biHeight = height;
	info.biPlanes = 1;			// we only have one bitplane
	info.biBitCount = 24;		// RGB mode is 24 bits
	info.biCompression = BI_RGB;	
	info.biSizeImage = 0;		// can be 0 for 24 bit images
	info.biXPelsPerMeter = 0x0ec4;     // paint and PSP use this values
	info.biYPelsPerMeter = 0x0ec4;     
	info.biClrUsed = 0;			// we are in RGB mode and have no palette
	info.biClrImportant = 0;    // all colors are important

	// now we open the file to write to
	file = CreateFile ( bmpfile , GENERIC_WRITE, FILE_SHARE_READ,
		 NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL );
	if ( file == NULL )
	{
		CloseHandle ( file );
		return 0;
	}
	
	// write file header
	if ( WriteFile ( file, &bmfh, sizeof ( BITMAPFILEHEADER ), &bwritten, NULL ) == 0 )
	{	
		CloseHandle ( file );
		return 0;
	}
	// write infoheader
	if ( WriteFile ( file, &info, sizeof ( BITMAPINFOHEADER ), &bwritten, NULL ) == 0 )
	{	
		CloseHandle ( file );
		return 0;
	}
	// write image data
	if ( WriteFile ( file, Buffer, paddedsize, &bwritten, NULL ) == 0 )
	{	
		CloseHandle ( file );
		return 0;
	}
	
	// and close file
	CloseHandle ( file );

	return 1;
}
/*
\brief Initialization of the video rendering stage.
*/
C_RESULT output_rendering_device_stage_open( void *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
	vp_os_mutex_init(&video_update_lock);
	//return start_visual_navigation();
	return (VP_SUCCESS);
}



extern uint8_t * FrameBuffer;



/*****************************************************************************/
/*
\brief Video rendering function (called for each received frame from the drone).
*/

 
C_RESULT output_rendering_device_stage_transform( void *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
	vlib_stage_decoding_config_t* vec = (vlib_stage_decoding_config_t*)cfg;
	

	// these variables are used to save images of the file
	/*
	long newsize;
	BYTE* buffer;
	char *true_buffer = (char*) vp_os_malloc(sizeof(char)*307200);

	*/

	int v = 0;
	int w = 0;
	int u = 0;
	long num_pix = 320*240*3;
	int i = 0;

	
	// some color definitions
	/*
	double Black[3] = {0, 0, 0};
	double White[3] = {255, 255, 255};
	double Dark_Turquoise[3] = {0, 206, 209};
	double Gainsboro[3] = {220, 220, 220};
	double Gray[3] = {128, 128, 128};
	double Dark_Orange[3] = {255, 165, 0};
	double Dodger_Blue[3] = {30, 144, 255};
	double Red[3] = {255, 0, 0};
	double Slate_Gray[3] = {112, 135, 144};
	double Olive_Drab[3] = {107, 142, 35};
	double Aqua[3] = {0, 255, 255};
	double Orange_Red[3] = {255, 69, 0};
	double Deep_Sky_Blue[3] = {0, 191, 255};
	double my_Orange[3] = {235, 105, 95};
	double Green[3] = {0, 255, 0};
	double Yellow[3] = {255, 255, 0};
	double Blue[3] = {0, 0, 255};
	double my_Blue[3] = {20, 92, 189};
	double my_Wall[3] = {255, 153, 153};
	double Gold[3] = {245, 210, 60};

	double *Target_RGBs[2];
	long hits[2] = {0};
	int *hit_map[2];
	int xy[2] = {0};
	int xy2[2] = {0};

	double test_RGB[3] = {255, 255, 255};
	double H;
	*/


	// Image declarations
	IplImage* img;
	IplImage* img_out = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
	IplImage* graphimg = cvCreateImage(cvSize(640, 320), IPL_DEPTH_8U, 3);

	static IplImage* mean_bg;
	static IplImage* var_bg;
	static int created = 0;

	// this object holds the present and last positions of the centroid of the color (it holds up to
	// 2 colors right now, but that can be changed. It also has roughly how many pixels match the color.
	// It's static because I only want it declared once as well as being persistent across all calls to
	// this function.
	bg_model* bg_p = &new_model;
	// Some color definitions
	int Dark_Turquoise = 90;
	int Green = 79;
	int Purple = 122;
	int Purple2 = 120;
	

	// This array holds which colors we're looking for in HSV values taken from MS Paint
	int color_hue[2] = {Green, Purple2};
	//int color_hue[2] = {Purple, Green};

	//DECLARE_TIMING(my_timer);
	//static int64 timeStart_my_timer;
	//static double timeDiff_my_timer;
	//static double timeTally_my_timer = 0;
	//static int countTally_my_timer = 0;

	/*
	for (i = 0; i<2; i++)
	{
		hit_map[i] = (int*) vp_os_malloc(sizeof(int)*76800);
		vp_os_memset (hit_map[i], 0, sizeof(int)*76800);
	}
	*/

	pthread_attr_t attr;
	size_t stacksize;

	
	pthread_attr_init(&attr);
	stacksize = sizeof(double)*3*2000000;
	pthread_attr_setstacksize (&attr, stacksize);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);


	if (!created) {
		created = 1;
		mean_bg = cvCreateImage(cvSize(320, 240), IPL_DEPTH_8U, 3);
		var_bg = cvCloneImage(mean_bg);
		pthread_mutex_init(&fd_mutex, NULL);
	}
	//START_TIMING(my_timer);

	bg_p->mean_img = mean_bg;
	bg_p->var_img = var_bg;

	vp_os_mutex_lock(&video_update_lock);

	/* Get a reference to the last decoded picture */
	pixbuf_data      = (uint8_t*)in->buffers[0];


	// Take the pixbuf_data and convert it into an image that OpenCV understands
	img = convert_to_CVimg(pixbuf_data);

	// This part handles the motion tracking aspect. Two simple methods are image subtraction,
	// which just subtracts the last frame in this case, and background subtraction, which
	// creates a distribution for each pixel and then compares all future frames to the distribution.
	//if (create_bg_model(img) == 1) {
	//	remove_bg(img);
	//}
	//motion_detection(img);

	// Everything related to color detection occurs here. The last two numbers are the number
	// of colors being detected and the threshold for how close the color has to be.
	img = color_detection(img, color_hue, 2, 5);

	pthread_cancel(control_thread);
	pthread_create(&control_thread, &attr, drone_c_thread, (void *) 0);
	pthread_attr_destroy(&attr);

	//cvInitFont(&font1, CV_FONT_HERSHEY_DUPLEX, hscale, vscale, shear, thickness2, line_type);
	//cvPutText(img_out, text, cvPoint(30,40), &font1, CV_RGB(0, 255, 255));

	// Creates a larger image for viewing purposes.
	cvResize(img, img_out, 1);



	//H = find_hue(test_RGB);
	//H = find_hue(Dark_Turquoise);
	//normalize_RGB(Gold);
	//normalize_RGB(Dark_Turquoise);

	//Target_RGBs[0] = Dark_Turquoise;
	//Target_RGBs[1] = Gold;

	//v = 0;
	//w = 0;
	//u = 0;

	//for (v = 0; v<240; v++)
	//{
	//	for (w = 0; w<(320); w++)
	//	{
	//		for (u = 0; u<3; u++)
	//		{
	//			pixbuf_data[3*w+3*v*640+(u)] = img_out->imageData[3*w+3*v*320+(2-u)];
	//		}
	//	}
	//}

	/*
	for (v = 0; v<240; v++)
	{
		for (w = 0; w<(3*320); w++)
		{
			true_buffer[w+3*v*320] = pixbuf_data[w+3*v*640];
		}
	}
	*/

	//buffer = ConvertRGBToBMPBuffer (true_buffer, 320, 240, &newsize);
	//SaveBMP (buffer, 320, 240, newsize, L"Drone_Pic_real.bmp");

	/*
	i = 0;

	for (i = 0; i<num_pix; i+=3)
	{
		test_RGB[0] = (double) true_buffer[i];
		test_RGB[1] = (double) true_buffer[i+1];
		test_RGB[2] = (double) true_buffer[i+2];

		normalize_RGB(test_RGB);

		true_buffer[i] = (char) ( (int) test_RGB[0]);
		true_buffer[i+1] = (char) ( (int) test_RGB[1]);
		true_buffer[i+2] = (char) ( (int) test_RGB[2]);
	}
	*/
	

	// These two lines of code make use of two functions defined at the top of this file. Their purpose is 
	// to save a bitmap file of the current picture frame. Currently, the old frame is overwritten and there
	// are large pink areas in the image. The picture can be found here:
	// \Desktop\ARDrone1.7\ARDrone_SDK_Version_1_7_20110525\Examples\Win32\VCProjects\ARDrone\Win32Client
	// Found the error. The image is really 320x240, but it uses the space of 640x480. Additionally, the image
	// data is not continuous, meaning that there are 960 (3*320) data values, then 960 useless values. Then,
	// another actual 960 values, and so on until the image is complete. To get a better idea of how this works,
	// look at the faulty drone picture. The pink areas are no data zones, and their purpose is to allow a complete
	// image to form in the top left.
	
	//buffer = ConvertRGBToBMPBuffer (pixbuf_data, DRONE_VIDEO_MAX_WIDTH, DRONE_VIDEO_MAX_HEIGHT, &newsize);
	//SaveBMP (buffer, DRONE_VIDEO_MAX_WIDTH, DRONE_VIDEO_MAX_HEIGHT, newsize, L"Faulty_Drone_Pic.bmp");
	//buffer = ConvertRGBToBMPBuffer (true_buffer, 320, 240, &newsize);
	//SaveBMP (buffer, 320, 240, newsize, L"Drone_Pic.bmp");
	

	//Color_Control(Target_RGBs, 2, 0.013, pixbuf_data, num_pix, hits, hit_map);
	//find_xy_centroid(hit_map[0], 320, 240, hits[0], xy);
	//find_xy_centroid(hit_map[1], 320, 240, hits[1], xy2);


	/*if ( (fabs(fd->roll) + fabs(fd->pitch) + fabs(fd->yaw) + fabs(fd->gaz)) != 0 )
	{
		
		for (i = 0; i<30; i++)
		{
			ardrone_at_set_progress_cmd(1 , fd->roll, fd->pitch, fd->gaz, fd->yaw);
		}
	}
	else
	{
		ardrone_at_set_progress_cmd(0, 0, 0, 0, 0);
	}*/
	
	//pixbuf_data[3*(159+119*640)] = (char) 255;
	//pixbuf_data[3*(159+119*640) + 1] = (char) 255;
	//pixbuf_data[3*(159+119*640) + 2] = (char) 255;

	//STOP_TIMING(my_timer);
	pthread_mutex_lock(&fd_mutex);

	//if (vec_p->index < vec_p->size) {
	//	vec_p->roll[vec_p->index] = fd->roll;
	//	vec_p->gaz[vec_p->index] = fd->gaz;
	//	(vec_p->index)++;
	//}

	//cvSet(graphimg, cvScalar(255, 255, 255, 0), NULL);
	//setGraphColor(0);
	//drawFloatGraph(vec_p->roll, vec_p->size, graphimg, -1, 1, graphimg->width, graphimg->height, "Roll Function", 1);
	//cvNamedWindow("Roll", CV_WINDOW_AUTOSIZE);
	//cvShowImage("Roll", graphimg);

	//if (vec_p->index == vec_p->size) {
	//	cvSaveImage("Roll.jpg", graphimg, 0);
	//}

	//cvSet(graphimg, cvScalar(255, 255, 255, 0), NULL);
	//drawFloatGraph(vec_p->gaz, vec_p->size, graphimg, -1, 1, graphimg->width, graphimg->height, "Gaz Function", 1);
	//cvNamedWindow("Gaz", CV_WINDOW_AUTOSIZE);
	//cvShowImage("Gaz", graphimg);

	//if (vec_p->index == vec_p->size) {
	//	cvSaveImage("Gaz.jpg", graphimg, 0);
	//	vec_p->index++;
	//}

	ARWin32Demo_AcquireConsole();
	ARWin32Demo_SetConsoleCursor(0,14);
	printf("Automatic     : [Pitch %f] [Roll %f]                        \n", fd->pitch, fd->roll);
	printf("              : [Yaw %f] [Gaz %f]                           \n", fd->yaw, fd->gaz);
	printf("Color1 (x,y)  : %i/320      %i/240                          \n", obj_p->positions[0].posXY[0], obj_p->positions[0].posXY[1]);
	printf("Color2 (x,y)  : %i/320      %i/240                          \n", obj_p->positions[1].posXY[0], obj_p->positions[1].posXY[1]);
	printf("Areas         : 1: %f     2: %f                             \n", obj_p->area[0], obj_p->area[1]);
	//printf("Mean          : %f                                          \n", find_mean(rnd_array, 8));
	//printf("Variance      : %f                                          \n", find_variance(rnd_array, 8, 5));
	//printf("Hits          : [Color1] %i       [Color2] %i               \n", hits[0], hits[1]);
	//printf("Test RGB      : [R] %f    [G] %f    [B] %f                  \n", test_RGB[0], test_RGB[1], test_RGB[2]);
	//printf("Corner RGB    : [R] %i    [G] %i    [B] %i                  \n", pixbuf_data[3*(159+119*640)], pixbuf_data[3*(159+119*640) + 1], pixbuf_data[3*(159+119*640) + 2]);
	//printf("Num_hits      : %i                                          \n", num_hits);
	//printf("Hue           : %f                                          \n", H);
	//printf("Left/Right H  : %f/%f                                       \n", obj_p->left_side[0], obj_p->right_side[0]);
	printf("Ratio         : %f/%f                                        \n", obj_p->left_area[0], obj_p->right_area[0]);
	printf("Execution Time: %f                                          \n", mytime);
	printf("Time2         : %f                                          \n", time2);
	printf("Time3         : %f                                          \n", time3);
	//printf("XYZ           : x:%f   y:%f   z:%f                          \n", xyz[0], xyz[1], xyz[1]);
	//printf("Average Time    : %f                                        \n", GET_AVERAGE_TIMING(my_timer));
	ARWin32Demo_ReleaseConsole();

	pthread_mutex_unlock(&fd_mutex);

	// Create a window and show.
	//BGR = cvScalarAll(0);
	cvNamedWindow("mainWin", CV_WINDOW_AUTOSIZE);
	cvShowImage("mainWin", img_out);
	cvSetMouseCallback("mainWin", MouseHandler, (void*) img_out);
	cvWaitKey(1);
	//ar_function(img_out);
  
			/** ======= INSERT USER CODE HERE ========== **/
		

				// Send the decoded video frame to the DirectX renderer.
				// This is an example; do here whatever you want to do
				//  with the decoded frame.
  
				/* Send the actual video resolution to the rendering module */
				D3DChangeTextureSize(vec->controller.width,vec->controller.height);
				/* Send video picture to the rendering module */
				D3DChangeTexture(pixbuf_data);


			/** ======= INSERT USER CODE HERE ========== **/
		

/*
  i = 0;
  for (i = 0; i<2; i++)
  {
	  vp_os_free(hit_map[i]);
  }
*/

  cvReleaseImage(&img);
  cvReleaseImage(&img_out);
  cvReleaseImage(&graphimg);
  //vp_os_free(buffer);
  //vp_os_free(true_buffer);
  vp_os_mutex_unlock(&video_update_lock);
  //pthread_mutex_destroy(&fd_mutex);
  return (VP_SUCCESS);
}




/*****************************************************************************/
/*
\brief Video rendering function (called for each received frame from the drone).
*/
C_RESULT output_rendering_device_stage_close( void *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
	//return end_visual_navigation();
  return (VP_SUCCESS);
}


/*****************************************************************************/
/*
	List of the functions that define the rendering stage.
*/
const vp_api_stage_funcs_t vp_stages_output_rendering_device_funcs =
{
  NULL,
  (vp_api_stage_open_t)output_rendering_device_stage_open,
  (vp_api_stage_transform_t)output_rendering_device_stage_transform,
  (vp_api_stage_close_t)output_rendering_device_stage_close
};




/*****************************************************************************/
/*
	The video processing thread.
	This function can be kept as it is by most users.
	It automatically receives the video stream in a loop, decode it, and then 
		call the 'output_rendering_device_stage_transform' function for each decoded frame.
*/
DEFINE_THREAD_ROUTINE(video_stage, data)
{
  C_RESULT res;

  vp_api_io_pipeline_t    pipeline;
  vp_api_io_data_t        out;
  vp_api_io_stage_t       stages[NB_STAGES];

  vp_api_picture_t picture;

  video_com_config_t              icc;
  vlib_stage_decoding_config_t    vec;
  vp_stages_yuv2rgb_config_t      yuv2rgbconf;
  

  /* Picture configuration */
	  picture.format        = PIX_FMT_YUV420P;

	  picture.width         = DRONE_VIDEO_MAX_WIDTH;
	  picture.height        = DRONE_VIDEO_MAX_HEIGHT;
	  picture.framerate     = 15;

	  picture.y_buf   = vp_os_malloc( DRONE_VIDEO_MAX_WIDTH * DRONE_VIDEO_MAX_HEIGHT     );
	  picture.cr_buf  = vp_os_malloc( DRONE_VIDEO_MAX_WIDTH * DRONE_VIDEO_MAX_HEIGHT / 4 );
	  picture.cb_buf  = vp_os_malloc( DRONE_VIDEO_MAX_WIDTH * DRONE_VIDEO_MAX_HEIGHT / 4 );

	  picture.y_line_size   = DRONE_VIDEO_MAX_WIDTH;
	  picture.cb_line_size  = DRONE_VIDEO_MAX_WIDTH / 2;
	  picture.cr_line_size  = DRONE_VIDEO_MAX_WIDTH / 2;

	  vp_os_memset(&icc,          0, sizeof( icc ));
	  vp_os_memset(&vec,          0, sizeof( vec ));
	  vp_os_memset(&yuv2rgbconf,  0, sizeof( yuv2rgbconf ));

   /* Video socket configuration */
	  icc.com                 = COM_VIDEO();
	  icc.buffer_size         = 100000;
	  icc.protocol            = VP_COM_UDP;
  
	  COM_CONFIG_SOCKET_VIDEO(&icc.socket, VP_COM_CLIENT, VIDEO_PORT, wifi_ardrone_ip);

  /* Video decoder configuration */
	  /* Size of the buffers used for decoding 
         This must be set to the maximum possible video resolution used by the drone 
         The actual video resolution will be stored by the decoder in vec.controller 
		 (see vlib_stage_decode.h) */
	  vec.width               = DRONE_VIDEO_MAX_WIDTH;
	  vec.height              = DRONE_VIDEO_MAX_HEIGHT;
	  vec.picture             = &picture;
	  vec.block_mode_enable   = TRUE;
	  vec.luma_only           = FALSE;

  yuv2rgbconf.rgb_format = VP_STAGES_RGB_FORMAT_RGB24;

   /* Video pipeline building */
  
		 pipeline.nb_stages = 0;

		/* Video stream reception */
		stages[pipeline.nb_stages].type    = VP_API_INPUT_SOCKET;
		stages[pipeline.nb_stages].cfg     = (void *)&icc;
		stages[pipeline.nb_stages].funcs   = video_com_funcs;

		pipeline.nb_stages++;

		/* Video stream decoding */
		stages[pipeline.nb_stages].type    = VP_API_FILTER_DECODER;
		stages[pipeline.nb_stages].cfg     = (void*)&vec;
		stages[pipeline.nb_stages].funcs   = vlib_decoding_funcs;

		pipeline.nb_stages++;

		/* YUV to RGB conversion 
		YUV format is used by the video stream protocol
		Remove this stage if your rendering device can handle 
		YUV data directly
		*/
		stages[pipeline.nb_stages].type    = VP_API_FILTER_YUV2RGB;
		stages[pipeline.nb_stages].cfg     = (void*)&yuv2rgbconf;
		stages[pipeline.nb_stages].funcs   = vp_stages_yuv2rgb_funcs;

		pipeline.nb_stages++;

		/* User code */  
		stages[pipeline.nb_stages].type    = VP_API_OUTPUT_SDL;  /* Set to VP_API_OUTPUT_SDL even if SDL is not used */
		stages[pipeline.nb_stages].cfg     = (void*)&vec;   /* give the decoder information to the renderer */
		stages[pipeline.nb_stages].funcs   = vp_stages_output_rendering_device_funcs;

		  pipeline.nb_stages++;
		  pipeline.stages = &stages[0];
 
 
		  /* Processing of a pipeline */
			  if( !ardrone_tool_exit() )
			  {
				PRINT("\n   Video stage thread initialisation\n\n");

				res = vp_api_open(&pipeline, &pipeline_handle);

				if( VP_SUCCEEDED(res) )
				{
				  int loop = VP_SUCCESS;
				  out.status = VP_API_STATUS_PROCESSING;

				  while( !ardrone_tool_exit() && (loop == VP_SUCCESS) )
				  {
					  if( VP_SUCCEEDED(vp_api_run(&pipeline, &out)) ) {
						if( (out.status == VP_API_STATUS_PROCESSING || out.status == VP_API_STATUS_STILL_RUNNING) ) {
						  loop = VP_SUCCESS;
						}
					  }
					  else loop = -1; // Finish this thread
				  }

				  vp_api_close(&pipeline, &pipeline_handle);
				}
			}

  PRINT("   Video stage thread ended\n\n");

  return (THREAD_RET)0;
}

