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
 * @file modules/computer_vision/opticflow/visual_estimator.h
 * @brief Estimate velocity from optic flow.
 *
 * Using sensors from vertical camera and IMU of Parrot AR.Drone 2.0
 */

#ifndef VISUAL_ESTIMATOR_H
#define VISUAL_ESTIMATOR_H

#include "inter_thread_data.h"

struct visual_estimator_struct
{
  // Image size
  unsigned int imgWidth;
  unsigned int imgHeight;

  unsigned int inFrameWidth;
  unsigned int inFrameHeight;

  // Images
  uint8_t *current_frame;
  uint8_t *prev_frame;

  uint8_t *gray_frame;
  uint8_t *prev_gray_frame;

  // Initialization
  int old_img_init;

  // Store previous
  float prev_pitch;
  float prev_roll;
  float prev_yaw;
} visual_estimator;

/**
 * Initialize visual estimator.
 * @param w  image width
 * @param h  image height
 */


extern unsigned char saveEnabled;
extern void   initMapSaveFile(void);
extern void   saveMap(void);
extern void   closeMap(void);

extern void opticflow_plugin_init(unsigned int w, unsigned int h, struct CVresults *results);
extern void opticflow_plugin_free(void);



extern void opticflow_plugin_run(unsigned char *frame, struct PPRZinfo* info, struct CVresults* results);

extern void ImGray2UYVU(unsigned char *frame, unsigned char *grayFrame, int imW, int imH);
extern void ImUYVU2Gray(unsigned char *grayFrame, unsigned char *frame, int imW, int imH);

extern void ImResizeUYVU(unsigned char *output, int imWOut, int imHOut, unsigned char *input, int imWIn, int imHIn, int downsample);
#endif /* VISUAL_ESTIMATOR_H */
