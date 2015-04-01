/*
 * Copyright (C) 2014 Hann Woei Ho
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/computer_vision/opticflow/visual_estimator.c
 * @brief Estimate velocity from optic flow.
 *
 * Using sensors from vertical camera and IMU of Parrot AR.Drone 2.0.
 *
 * Warning: all this code is called form the Vision-Thread: do not access any autopilot data in here.
 */

/////////////////////////////////////////////////////////////////////////////////
/// Includes
/////////////////////////////////////////////////////////////////////////////////
#include "std.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// PeakFinder function
#include "../peakfinder.h"
#include "../avoid_nav.h"

// Debug defines & macros
#include "../debug_nav.h"

// Image related includes
#include "encoding/rtp.h"
#include "resize.h"
#include "image.h"

// Own Header
#include "visual_estimator.h"

// Computer Vision
#include "opticflow/optic_flow_int.h"

// for FPS
#include "../cv/framerate.h"

// Avoidance headers
#include "../avoid_nav_transportFcns.h"

/////////////////////////////////////////////////////////////////////////////////
/// Module defines
/////////////////////////////////////////////////////////////////////////////////

#define AVOID_BASED_ON_COLOR
#define WITH_NAVIGATION
#define WITH_BINARY_IMAGE

//#define USE_HARRIS_DETECTOR

// Corner Detection
#define MAX_FEATURE_COUNT 200

// Peakdetector Threshold
#define PEAKDETECTOR_THRESHOLD  0.2
#define MINIMUM_FLOW_SCALE_VAL  20.0f
#define COLOR_MIN_SUM           50

// ARDrone Vertical Camera Parameters

// Bottom Camera
//#define FOV_H 0.67020643276
//#define FOV_W 0.89360857702

// Front Camera
#define FOV_W 1.2183


#ifdef AVOID_BASED_ON_COLOR
   #pragma message("Avoiding based on color!")
#endif

#ifndef AVOID_BASED_ON_COLOR
  #ifdef USE_HARRIS_DETECTOR
    #error This implementation of Harris is very slow and it just doesn't work. Use FAST!
    #include "opticflow/harris.h"
    #pragma message("Using the Harris detector")
  #else
    #include "opticflow/fast9/fastRosten.h"
    #pragma message("Using the FAST detector")
  #endif
#endif

// This will downscale the front camera image from (1280x720) to (320x180)
//#ifdef AVOID_BASED_ON_COLOR
//    #define IMAGE_DOWNSIZE_FACTOR 1
//#else
  #ifdef DOWNLINK_VIDEO
    #define IMAGE_DOWNSIZE_FACTOR 8
  #else
    #define IMAGE_DOWNSIZE_FACTOR 2
  #endif
//#endif


/////////////////////////////////////////////////////////////////////////////////
/// Variable definitions
/////////////////////////////////////////////////////////////////////////////////

#ifdef DOWNLINK_FLOWSUM
  #pragma message("Horizontal Flow & arena map downlink enabled")
  struct UdpSocket *flowSock;
  struct UdpSocket *mapSock;
#endif

#ifdef DOWNLINK_VIDEO
  #pragma message("Video Downlink Enabled")
  #include "encoding/jpeg.h"
  struct UdpSocket *vsock;

  uint8_t *downlinkBuffer;
  uint8_t *downlinkBufferGray;
  uint8_t *jpegbuf;
#endif

// Local variables
#ifdef USE_HARRIS_DETECTOR
  harris_data harrisData;
#endif

/////////////////////////////////////////////////////////////////////////////////
/// Init Function
/////////////////////////////////////////////////////////////////////////////////

// Called by plugin
void opticflow_plugin_init(unsigned int w, unsigned int h, struct CVresults *results)
{
  // Initialize variables
  visual_estimator.inFrameWidth   = w;
  visual_estimator.inFrameHeight  = h;

// Only the incoming frame has the complete size.
  w = w / IMAGE_DOWNSIZE_FACTOR;
  h = h / IMAGE_DOWNSIZE_FACTOR;
//#ifndef AVOID_BASED_ON_COLOR <= TODO: Sigfaults if not commented!
  visual_estimator.current_frame    = (uint8_t *)malloc(w * h * 2);
  visual_estimator.prev_frame       = (uint8_t *)malloc(w * h * 2);
  visual_estimator.gray_frame       = (unsigned char *) calloc(w * h, sizeof(uint8_t));
  visual_estimator.prev_gray_frame  = (unsigned char *) calloc(w * h, sizeof(uint8_t));
//#else
//  visual_estimator.gray_frame       = (unsigned char *) calloc(w * h, sizeof(uint8_t));
//#endif
  visual_estimator.old_img_init = 1;
  visual_estimator.prev_pitch = 0.0;
  visual_estimator.prev_roll = 0.0;

  visual_estimator.imgWidth = w;
  visual_estimator.imgHeight = h;

  results->cnt          = 0;
  results->flow_count   = 0;
  results->count        = 0;
  results->FPS          = 0;
  
  results->WP_pos_X     = 0;
  results->WP_pos_Y     = 0;
  results->head_cmd     = 0;
  
  framerate_init();

#ifdef USE_HARRIS_DETECTOR
  // Initialize the Harris detector
  harris_init(&harrisData, 3, 2, 0.5, 0.04);
#endif

#ifdef WITH_NAVIGATION
// Initialize obstacle map   <= It's done from the flight plan via avoid_module_post_ahrs_init()
//  init_map();
//  vehicle_cache_init();
#else
  #warning Navigation Disabled!
#endif

  flowPeaks.angles      = 0;
  flowPeaks.nAngles     = 0;

#ifdef DOWNLINK_VIDEO
  // Video Compression
  downlinkBufferGray = (uint8_t *)malloc(w * h);
  downlinkBuffer     = (uint8_t *)malloc(w * h * 2);
  jpegbuf            = (uint8_t *)malloc(w * h * 2);

  // Network Transmit
  vsock = udp_socket("192.168.1.255", 5000, 5001, FMS_BROADCAST);
#endif
#ifdef DOWNLINK_FLOWSUM
  flowSock = udp_socket("192.168.1.255", 9000, 9001, FMS_BROADCAST);;
  mapSock  = udp_socket("192.168.1.255", 10000, 10001, FMS_BROADCAST);;
#endif
}
void opticflow_plugin_free(void)
{
#ifdef USE_HARRIS_DETECTOR
  // Initialize the Harris detector
  harris_free(&harrisData);
#endif
}

/////////////////////////////////////////////////////
//// Debug stuff
/////////////////////////////////////////////////////
//#ifdef DEBUG_CONSOLE_PRINT
int printfCounter = 0;
//#endif


void opticflow_plugin_run(unsigned char *frame, struct PPRZinfo* info, struct CVresults *results)
{
/*
//#ifdef DEBUG_CONSOLE_PRINT
if(printfCounter > 10)
{  nav_print_full_report(); printfCounter = 0; }
else
  ++printfCounter;
//#endif
*/
  // Corner Tracking
  // Working Variables
/*
if(printfCounter>10)
{  printfCounter = 0; }
else
  ++printfCounter;
test_angles(); */


if(printfCounter>10)
{  
  printfCounter = 0; 
  saveMap();
}
else
  ++printfCounter;

#ifndef AVOID_BASED_ON_COLOR
  int borderx = 24, bordery = 24;
  int x[MAX_FEATURE_COUNT], y[MAX_FEATURE_COUNT];
  int new_x[MAX_FEATURE_COUNT], new_y[MAX_FEATURE_COUNT];
  int status[MAX_FEATURE_COUNT];
  int dx[MAX_FEATURE_COUNT];//, dy[MAX_FEATURE_COUNT]; <- we don't use the vertical flow
#endif

  int w = visual_estimator.imgWidth;
  int h = visual_estimator.imgHeight;

  // Framerate Measuring
  results->FPS = framerate_run();

  /*printf("visual_estimator.c: ds: %d Current FPS: %.2f ",IMAGE_DOWNSIZE_FACTOR, results->FPS);*/
  /*V_LOG("visual_estimator.c: ds: %d Current FPS: %.2f ",IMAGE_DOWNSIZE_FACTOR, results->FPS);*/

  // Downsize the image for processing
  ImResizeUYVU(visual_estimator.current_frame, visual_estimator.imgWidth, visual_estimator.imgHeight, 
                frame, visual_estimator.inFrameWidth, visual_estimator.inFrameHeight, 
              IMAGE_DOWNSIZE_FACTOR);

  // Get Grayscale image
#ifndef AVOID_BASED_ON_COLOR
  CvtYUYV2Gray(visual_estimator.gray_frame, visual_estimator.current_frame, w, h);
#else
  ImUYVU2Gray(visual_estimator.gray_frame, visual_estimator.current_frame, w, h); // <= not really grayscale image though.
#endif
// Convert the image to binary
#ifdef WITH_BINARY_IMAGE
  double pxValSum = 0;
  for (int i=0; i < w*h; ++i)
    pxValSum += visual_estimator.gray_frame[i];
  
  pxValSum /= (w*h);

  unsigned char pxTh = pxValSum;
  /*printf("pxTh: %d ",pxTh);*/
  for (int i=0; i < w*h; ++i)
  {
    if (visual_estimator.gray_frame[i] > pxTh)
      visual_estimator.gray_frame[i] = 255;
    else
      visual_estimator.gray_frame[i] = 0;
  }
#endif

  // Initialize prev_frame & prev_gray_frame to the current frames
  if (visual_estimator.old_img_init == 1) {
    memcpy(visual_estimator.prev_frame, frame, w * h * 2);
    memcpy(visual_estimator.prev_gray_frame, visual_estimator.gray_frame, w * h);
    visual_estimator.old_img_init = 0;
  }
#ifdef DOWNLINK_VIDEO
 // Make a copy of the current frame, so we can add stuff to it for debuging
 memcpy(downlinkBufferGray, visual_estimator.prev_gray_frame, w*h);
#endif

#ifndef AVOID_BASED_ON_COLOR
  // *************************************************************************************
  // Corner detection
  // *************************************************************************************

#ifdef USE_HARRIS_DETECTOR
  xyPos *featPts = harris(&harrisData, visual_estimator.prev_gray_frame, w, h, &results->count, HARRIS_LOOKFOR_EDGES);
#else
  // FAST corner detection
  int fast_threshold = 20;
  xyFAST *featPts;
  featPts = fast9_detect((const byte *)visual_estimator.prev_gray_frame, w, h, w,
                           fast_threshold, &results->count);
#endif

/*printf("fP: %d ", results->count);*/
  // Remove neighboring corners
#ifdef DOWNLINK_VIDEO
  int min_distance = 5;
#else
  int min_distance = 30;
#endif
  int min_distance2 = min_distance * min_distance;

  unsigned char labelRemove[results->count];
  //Ensure that the labelRemove values are initialized to zero
  for (int i = 0; i < results->count; i++)
    labelRemove[i] = 0;

  for (int i = 0; i < results->count; i++) 
  {
    if (labelRemove[i]) //The point is already marked to be removed *flies away*
        continue;

    for (int j = i+1; j < results->count; j++) 
    {
      if (labelRemove[j]) //The point is already marked to be removed *flies away*
        continue;

      // distance squared:
      int p_dx = featPts[i].x - featPts[j].x;
      int p_dy = featPts[i].y - featPts[j].y;
      int distance2 = p_dx*p_dx + p_dy*p_dy;;
      if (distance2 < min_distance2)
        labelRemove[j] = 1;
    }
  }

  // Get the first at most MAX_FEATURE_COUNT sparsely spead points (as per labelRemove!)
  int srcPos  = 0;
  int destPos = 0;
  while ( (srcPos < results->count) && (destPos < MAX_FEATURE_COUNT) )
  {
    if (!labelRemove[srcPos])
    {
      x[destPos] = featPts[srcPos].x;
      y[destPos] = featPts[srcPos].y;

#ifdef DOWNLINK_VIDEO
  #ifdef DEBUG_VIDEO
    #ifdef DEBUG_MARK_FEATUREPOINTS

 // Mark the feature points on the downlinked image as white (255)! (note: 2x2 pixels should be marked, otherwise they disappear after jpeg compression)
  downlinkBufferGray[w*y[destPos] + x[destPos]]   = DEBUG_OVERLAY_COLOR;
  downlinkBufferGray[w*y[destPos] + x[destPos]+1] = DEBUG_OVERLAY_COLOR;

  if (x[destPos] < (w-2) )
  {
    downlinkBufferGray[w*y[destPos] + x[destPos] + w]    = DEBUG_OVERLAY_COLOR;
    downlinkBufferGray[w*y[destPos] + x[destPos] + w +1] = DEBUG_OVERLAY_COLOR;
  }
  else
  {
    downlinkBufferGray[w*y[destPos] + x[destPos] + w]    = DEBUG_OVERLAY_COLOR;
    downlinkBufferGray[w*y[destPos] + x[destPos] + w +1] = DEBUG_OVERLAY_COLOR;
  }
    #endif
  #endif
#endif

      ++destPos;
    }
    ++srcPos;
  }
  results->count      = destPos;
  results->flow_count = results->count;

  free(featPts);
  /*printf("nP: %d ", results->count);*/

  // *************************************************************************************
  // Corner Tracking
  // *************************************************************************************
  opticFlowLK(visual_estimator.gray_frame, visual_estimator.prev_gray_frame, x, y,
              results->flow_count, w, h, new_x, new_y, status, 5, 100);

  // Remove the invalid points
  srcPos  = 0;
  destPos = 0;
  for (srcPos = 0; srcPos < results->flow_count; ++srcPos)
  {
    if (  status[srcPos] && 
        !(new_x[srcPos] < borderx || new_x[srcPos] > (w - 1 - borderx) ||
                 new_y[srcPos] < bordery || new_y[srcPos] > (h - 1 - bordery)) &&
          (x[srcPos] != new_x[srcPos])
       )
    {
      if (srcPos != destPos) // No point in copying to the same place... is there?
      {
        x[destPos]      = x[srcPos];
        y[destPos]      = y[srcPos];
        new_x[destPos]  = new_x[srcPos];
        new_y[destPos]  = new_y[srcPos];
      }
      ++destPos;
    }
  }
  results->flow_count = destPos;

  // Optical Flow Computation
  for (int i = 0; i < results->flow_count; i++) 
  {
//    dy[i] = new_y[i] - y[i]; // <- we don't use the vertical flow
    if (new_x[i] > x[i]) // So we get the magnitude only
      dx[i] = new_x[i] - x[i];
    else
      dx[i] = x[i] - new_x[i];
  }

#else // #ifndef AVOID_BASED_ON_COLOR
// There is nothing to calculate here really...
#endif

  // *************************************************************************************
  // This is where the avoidance magic happen!
  // *************************************************************************************
  navTransportData.stateEnuPosX     = info->enuPosX;
  navTransportData.stateEnuPosY     = info->enuPosY;
  navTransportData.stateEnuHeading  = info->psi;
  
  navTransportData.stateWpStatus    = (info->targetDist < 0.1f); // Threshold on reaching the target is set to 0.1 meters!

  float peakAngles[100];
  int   numPeaks = 100;  
  float flowSum[w];
#ifndef AVOID_BASED_ON_COLOR
  V_LOG("nF: %d ", results->flow_count);


  cv_flowSum((int*)&x, (int*)&dx, results->flow_count, w, (float*)&flowSum);
  int smootherSize = 128;
#else
  for (int i=0; i < w; ++i)
  {
    double colSum = 0;
    for (int j=0; j < h; ++j) // Sum the pixel values over the height of the image
      colSum +=visual_estimator.gray_frame[i + j*w];
    
    if (colSum/(h*255))
    colSum /= h; // Normalize by height;

    flowSum[i] = (float)colSum;
  }
#pragma message("@Anton: flowSum is the vertical sum of the intensity. PEAKDETECTOR_THRESHOLD is the threshold used. ")
  
#ifdef DEBUG_DEBUG
  printf("\n");
  printf("Biary Image Sum:\n");
  for (int i=0; i < w; ++i)
  {
    if ( flowSum[i] > PEAKDETECTOR_THRESHOLD )
      printf("1 ");
    else
      printf("0 ");
  }
  printf("\n\n");
#endif

#endif
#pragma message("@Anton: This is where the peak detection is called")
//  cv_peakFinder((float*)&flowSum, w, PEAKDETECTOR_THRESHOLD, &numPeaks, (float*)&peakAngles, FOV_W);
  cv_peakFinder((float*)&flowSum, w, 0, &numPeaks, (float*)&peakAngles, FOV_W);

  // Variables to the navigation module
  flowPeaks.angles = (float*)&peakAngles;
  flowPeaks.nAngles = numPeaks;

  /*V_LOG("\nAngles: \n");*/
  for (int i=1; i < flowPeaks.nAngles; ++i)
  /*V_LOG("%.2f ", peakAngles[i]);*/
  /*V_LOG("\n");*/
#ifdef DOWNLINK_FLOWSUM
  { 
    int nS = w*2;
#ifdef AVOID_NAV_DEBUG
    unsigned char txBuf[nS+AVOID_NAV_DEBUG_DOWNLINK_SIZE*sizeof(float)];
#else
    unsigned char txBuf[nS];
#endif
    for (int l=0; l < w; ++l)
    {
      float V = (100.0f*flowSum[l]);
      int16_t Vi = ((int16_t)V);
      txBuf[l] = ((unsigned char)Vi);


      if (flowSum[l] > PEAKDETECTOR_THRESHOLD)
        txBuf[l+w] = 100;
      else
        txBuf[l+w] = 0;
    }
#ifdef AVOID_NAV_DEBUG
    memcpy(&txBuf+nS, &nav_debug_downlink, AVOID_NAV_DEBUG_DOWNLINK_SIZE*sizeof(float));
    if (udp_write(flowSock, (unsigned char*)&txBuf, (nS + (AVOID_NAV_DEBUG_DOWNLINK_SIZE*sizeof(float)))) != (nS + (AVOID_NAV_DEBUG_DOWNLINK_SIZE*sizeof(float))))
      V_LOG("UDP write error! ");
#else
    if (udp_write(flowSock, (unsigned char*)&txBuf, nS) != nS)
      V_LOG("UDP write error! ");
#endif
  }
  {
    unsigned char obsMapFile[GRID_RES*GRID_RES*2];

    for (int i=0; i < GRID_RES*GRID_RES; ++i)
      obsMapFile[i] = (unsigned char)(arena.grid_weights_obs[i]*2);

    for (int i=0; i < GRID_RES*GRID_RES; ++i)
      obsMapFile[i+(GRID_RES*GRID_RES)] = (unsigned char)(arena.grid_weights_exp[i]*10);


    if (udp_write(flowSock, (unsigned char*)&obsMapFile, GRID_RES*GRID_RES*2) != GRID_RES*GRID_RES*2)
      V_LOG("UDP map write error! ");
  }
#endif

#ifdef DOWNLINK_VIDEO
  #ifdef DEBUG_VIDEO
    #ifdef DEBUG_MARK_FLOWSUM
  for (int i=0; i < w; ++i)
  {
    int mH = (flowSum[w])*(h-2)+1;
    downlinkBufferGray[i + mH*w]      = DEBUG_OVERLAY_COLOR;
    downlinkBufferGray[i + (mH-1)*w]  = DEBUG_OVERLAY_COLOR;
    if (i == 0)
    {
      downlinkBufferGray[i + mH*w + 1]      = DEBUG_OVERLAY_COLOR;
      downlinkBufferGray[i + (mH-1)*w + 1]  = DEBUG_OVERLAY_COLOR;
    }
    else
    {
      downlinkBufferGray[i + mH*w - 1]      = DEBUG_OVERLAY_COLOR;
      downlinkBufferGray[i + (mH-1)*w - 1]  = DEBUG_OVERLAY_COLOR;
    }
  }
    #endif
  #endif
#endif
//  #warning !!!!!!!!!!!!!!!!!Navigation Disabled in visual_estimator.c!!!!!!!!!!!!!!!!!!!
#ifdef WITH_NAVIGATION
  if (numPeaks > 0)
    navigate();
#endif

  results->WP_pos_X     = navTransportData.currentWpLocationX;
  results->WP_pos_Y     = navTransportData.currentWpLocationY;
  results->head_cmd     = navTransportData.currentHeadingSetpoint;

#ifdef DOWNLINK_VIDEO
    // JPEG encode the image:
//    uint32_t quality_factor = 5; //20 if no resize,
    uint32_t quality_factor = 20; //20 if no resize,
    uint8_t dri_header = 0;
    uint32_t image_format = FOUR_TWO_TWO;  // format (in jpeg.h)

    // Convert back the grayscale image to yuyv as the jpeg encoder expects this format! The color codes are all set to zero (grayscale)
    ImGray2UYVU(downlinkBuffer, downlinkBufferGray, w, h);

    // Send the small image, not the full-sized one!
    uint8_t *end = encode_image(downlinkBuffer, jpegbuf, quality_factor, image_format, w, h, dri_header);
    
    uint32_t size = end - (jpegbuf);

    //V_LOG("Sending an image ...%u\n", size);
    send_rtp_frame(vsock, jpegbuf, size, w, h, 0, quality_factor, dri_header, 0);
#endif


  // *************************************************************************************
  // Next Loop Preparation
  // *************************************************************************************
#ifndef AVOID_BASED_ON_COLOR
  memcpy(visual_estimator.prev_frame, frame, w * h * 2);
  memcpy(visual_estimator.prev_gray_frame, visual_estimator.gray_frame, w * h);
#endif
  visual_estimator.prev_pitch = info->theta;
  visual_estimator.prev_roll  = info->phi;

  /*printf("\n");*/
  V_LOG("\n");
}
void ImGray2UYVU(unsigned char *frame, unsigned char *grayFrame, int imW, int imH)
{
  ++grayFrame;
  for (int i=0; i < imW*imH; ++i)
  {
    *frame++ = 127; 		      // U
    *frame++ = *grayFrame++; 	// Y
    ++grayFrame; 		          // <= NO idea why this is necessary!
  }
}
void ImUYVU2Gray(unsigned char *grayFrame, unsigned char *frame, int imW, int imH)
{
// If avoiding based on color information than the image intensity is not relevant, but the U/V components, especially the V for the orange obstacles
#ifndef AVOID_BASED_ON_COLOR 
  for (int i=0; i < imW*imH; ++i)
  {
    *grayFrame++ = *frame++;
    ++frame;
  }
#else
  unsigned char *dest = grayFrame;
  unsigned char *src  = frame;
// Take the V values of the image
  src+=2; // This is the first V
  while ((dest - grayFrame) < imW*imH)
  {
    if ( (*src > 155) && (*(src-2) > 100) )
    { *dest++ = 1; *dest++ = 1;}
    else
    { *dest++ = 0; *dest++ = 0;}
//    *dest++ = *src; // Use it for both pixel values
//    *dest++ = *src;
    src += 4;       // Go to the next V value
  }
#endif
}
void ImResizeUYVU(unsigned char *output, int imWOut, int imHOut, unsigned char *input, int imWIn, int imHIn, int downsample)
{ // Adapted from resize.h for using the raw image buffers
  uint8_t *source = input;
  uint8_t *dest = output;

  int pixelskip = (downsample - 1) * 2;
  for (int y = 0; y < imHOut; y++) {
    for (int x = 0; x < imWOut; x += 2) {
      // YUYV
      *dest++ = *source++; // U
      *dest++ = *source++; // Y
      *dest++ = *source++; // V
      source += pixelskip;
      *dest++ = *source++; // Y
      source += pixelskip;
    }
    // read 1 in every 'downsample' rows, so skip (downsample-1) rows after reading the first
    source += (downsample-1) * imWIn * 2;
  }
}

unsigned char saveEnabled = 0;;
FILE  *obsMapFile;
FILE  *expMapFile;

struct timeval map_time1;
struct timeval map_time2;
struct timeval *mapLastTime;
struct timeval *mapCurrentTime;

#define OBS_FILE_PATH ""
#define EXP_FILE_PATH ""

void   initMapSaveFile(void)
{
  obsMapFile = fopen(OBS_FILE_PATH, "w");
  if (obsMapFile == NULL)
  {
    printf("Error opening obsMap!\n");
    exit(1);
  }
  expMapFile = fopen(EXP_FILE_PATH, "w");
  if (expMapFile == NULL)
  {
    printf("Error opening expMap!\n");
    exit(1);
  }

  fprintf(obsMapFile, "%d %d\n",GRID_RES,GRID_RES);
  fprintf(expMapFile, "%d %d\n",GRID_RES,GRID_RES);

  gettimeofday(mapLastTime, NULL);
  saveEnabled = 1;
}
void   saveMap(void)
{
  if (!saveEnabled)
    return;

    gettimeofday(mapCurrentTime, NULL);
    float dt = ( ((float)time_elapsed(mapLastTime, mapCurrentTime))/1000000); // time_elapsed is (long) in microseconds

    fprintf(obsMapFile, "%.3f ", dt);
    for (int i=0; i < GRID_RES*GRID_RES; ++i)
      fprintf(obsMapFile, "%.2f ", arena.grid_weights_obs[i]);

    fprintf(obsMapFile, "\n");



    fprintf(expMapFile, "%.3f ", dt);
    for (int i=0; i < GRID_RES*GRID_RES; ++i)
      fprintf(expMapFile, "%.2f ", arena.grid_weights_exp[i]);

    fprintf(expMapFile, "\n");

//      obsMapFile[i+(GRID_RES*GRID_RES)] = (unsigned char)(*10);

  struct timeval *tmp = mapLastTime;
  mapLastTime         = mapCurrentTime;
  mapCurrentTime      = tmp;
}
void closeMap(void)
{
  saveEnabled = 0;
  fclose(obsMapFile);
  fclose(expMapFile);
}
