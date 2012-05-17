//Video based navigation library -- custom code
//J.Weeks Tufts University 2011/07/06

//local directory: sdk_demo/Sources/

//To be implemented from video_stages.c with feedback support for navdata.c

//Need to update Makefile to include vidnav.c as a generic binary source file.

#ifndef _VIDEO_BASED_NAVIGATION_
#define _VIDEO_BASED_NAVIGATION_

#include <VP_Os/vp_os_types.h>

//Image data array handler struct
typedef struct image_data_ 
{
  int          h;     //number of rows in the image
  int          w;     //number of columns in the image
  uint8_t***   data;  //RGB multi-dimensional pointer based data array
} 
image_data_t;

/*
 * initializes data structures
 * call in thread start sector
 */
C_RESULT start_visual_navigation(void);

/*
* pseudo main function
* function to be called from video_stages.c
* picbuff points to start of RGB buffer array
* length is length of buffer array
*/
C_RESULT enact_visual_navigation(uint8_t* picbuff, int length);

/*
 * clean up data structures
 * call in thread shutdown sector
 */
C_RESULT end_visual_navigation(void);

/*
 * detection flag for target detection
 * 1-target detected
 * 0-otherwise
 */
int target_detected(void);

/*
 * provides access to the first level filtered copy of image
 */
image_data_t* get_stage1_image(void);

/*
 * downsizing algorithm - averaging filter
 * returns downsized image data or NULL if factors do not evenly divide original dimensions.
 * xfact is horizontal compression
 * yfact is vertical compression
 * (ie xfact=8 would reduce image width to 1/8th its original value)
 */
image_data_t* downsize(image_data_t* prior, int xfact, int yfact);

/*
 * converts image array back to buffer stream
 */
uint8_t* restream(image_data_t* image);

/*
 * garbage collect for image_data_t
 */
void free_image_data(image_data_t* image);

#endif
/*  LocalWords:  navdata RGB picbuff xfact yfact sdk Makefile vidnav multi
 */
