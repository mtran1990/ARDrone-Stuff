//Video based navigation -- custom code
//J.Weeks Tufts University 2011/07/06

//local directory: sdk_demo/Sources/

#include "vidnav.h"

#include <ardrone_api.h>
#include <VP_Os/vp_os_types.h>

//detection tolerances
const char RTOL=5;   //red tolerance
const char GTOL=5;   //green tolerance
const char BTOL=5;   //blue tolerance

//origional and stage 1 size parameters
const int H=240; //origional image dimensions
const int W=320;
const int SG1X=4; //stage 1 compression factors
const int SG1Y=4;

//data variables
image_data_t* origional=NULL;
image_data_t* stage1=NULL;
int           detected=0;
uint8_t*      buffer=NULL;

//counting variables
int i, j, k, l;


C_RESULT start_visual_navigation(void)
{
  origional=(image_data_t*)vp_os_malloc(sizeof(image_data_t));
  stage1=(image_data_t*)vp_os_malloc(sizeof(image_data_t));

  //build origional image 3D array structure
  origional->h=H;
  origional->w=W;

  origional->data=(uint8_t***)vp_os_malloc(sizeof(uint8_t**)*H);
  if(origional->data==NULL) return C_FAIL;
  for(i=0; i<H; i++) {
    origional->data[i]=(uint8_t**)vp_os_malloc(sizeof(uint8_t*)*W);
    if(origional->data[i]==NULL) return C_FAIL;
    for(j=0; j<W; j++) {
      origional->data[i][j]=(uint8_t*)vp_os_malloc(sizeof(uint8_t)*3);
      if(origional->data[i][j]==NULL) return C_FAIL;
    }
  }

  //build stage 1 array for efficiency

  const int H1=H/SG1Y;
  const int W1=W/SG1X;

  //build 3D array structure
  stage1->h=H1;
  stage1->w=W1;

  stage1->data=(uint8_t***)vp_os_malloc(sizeof(uint8_t**)*H1);
  if(stage1->data==NULL) return C_FAIL;
  for(i=0; i<H1; i++) {
    stage1->data[i]=(uint8_t**)vp_os_malloc(sizeof(uint8_t*)*W1);
    if(stage1->data[i]==NULL) return C_FAIL;
    for(j=0; j<W1; j++) {
      stage1->data[i][j]=(uint8_t*)vp_os_malloc(sizeof(uint8_t)*3);
      if(stage1->data[i][j]==NULL) return C_FAIL;
    }
  }
  
  return C_OK;
}

C_RESULT enact_visual_navigation(uint8_t* picbuff, int length)
{
  if (length!=230400) return C_FAIL; //check buffer is for 320x240 RGB image

  //transcribe picture buffer to 3D array

  l=0;
  for (i=0; i<origional->h; i++) {
    for (j=0; j<origional->w; j++) {
      for (k=0; k<3; k++) {
	origional->data[i][j][k]=picbuff[l++];
      }
    }
  }

  stage1=downsize(origional, SG1X, SG1Y);
  if(stage1==NULL) return C_FAIL;

  return C_OK;
}

C_RESULT end_visual_navigation(void)
{
  free_image_data(origional);
  free_image_data(stage1);
  if(buffer!=NULL) vp_os_free(buffer);

  return C_OK;
}

int target_detected(void)
{
  return detected;
}

image_data_t* get_stage1_image(void)
{
  return stage1;
}

image_data_t* downsize(image_data_t* prior, int xfact, int yfact)
{
  uint8_t sumR, sumB, sumG;
  uint8_t divisor=xfact*yfact;
  image_data_t* downs;

  if(prior==origional) downs=stage1; //optimization step for fixed stage 1 downsize
  else  
    {
      downs=(image_data_t*)vp_os_malloc(sizeof(image_data_t));

      //get scaled down array size
      if(prior->w % xfact==0) downs->w=prior->w/xfact;
      else return NULL; //odd divisor failure
      if(prior->h % yfact==0) downs->h=prior->h/yfact;
      else return NULL;

      //create new array
      const int HD=downs->h;
      const int WD=downs->w;

      downs->data=(uint8_t***)vp_os_malloc(sizeof(uint8_t**)*HD);
      if(downs->data==NULL) return NULL;
      for(i=0; i<HD; i++) {
	downs->data[i]=(uint8_t**)vp_os_malloc(sizeof(uint8_t*)*WD);
	if(downs->data[i]==NULL) return NULL;
	for(j=0; j<WD; j++) {
	  downs->data[i][j]=(uint8_t*)vp_os_malloc(sizeof(uint8_t)*3);
	  if(downs->data[i][j]==NULL) return NULL;
	}
      }
    }

  //commence downsizing
  for (i=0; i<downs->h; i++) {
    for (j=0; j<downs->w; j++) {
      //i,j are destination coordinates

      //initialize sums
      sumR=0;
      sumB=0;
      sumG=0;
      
      //parse sector summing R,G,B values
      for (k=0; k<yfact; k++) {
	for (l=0; l<xfact; l++) {
	  //i*yfact+k,j*xfact+l are target coordinates
	 
	  sumR+=prior->data[i*yfact+k][j*xfact+l][0];
	  sumB+=prior->data[i*yfact+k][j*xfact+l][1];
	  sumG+=prior->data[i*yfact+k][j*xfact+l][2];
	}
      }
      //average sums and transfer averages to downsized array
      downs->data[i][j][0]=sumR/divisor;
      downs->data[i][j][1]=sumB/divisor;
      downs->data[i][j][2]=sumG/divisor;
    }
  }

  return downs;
}

uint8_t* restream(image_data_t* image)
{
  if(buffer!=NULL) vp_os_free(buffer);
  buffer=(uint8_t*)vp_os_malloc(sizeof(uint8_t)*image->h*image->w*3);
  
  k=0; //location in buffer

  for (i=0; i<image->h; i++) {
    for (j=0; j<image->w; j++) {
      buffer[k++]=image->data[i][j][0];
      buffer[k++]=image->data[i][j][1];
      buffer[k++]=image->data[i][j][2];
    }
  }

  return buffer;
}

void free_image_data(image_data_t* image)
{
  if(image!=NULL) //image_data_t exists
    {
      if(image->data!=NULL) //top data array tier exists
	{
	  for (i=0; i<image->h; i++)
	    {
	      if(image->data[i]!=NULL) //next tier exists
		{
		  for (j=0; j<image->w; j++)
		    {
		      if(image->data[i][j]!=NULL) //bottom tier exists
			vp_os_free(image->data[i][j]); //free bottom tier
		    }
		  vp_os_free(image->data[i]); //free column tier
		}
	    }
	  vp_os_free(image->data); //free row tier
	}
      vp_os_free(image); //free image_data_t space
    }
}

/*  LocalWords:  vidnav RGB sdk xfact yfact
 */
