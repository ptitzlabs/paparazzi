#ifndef DEBUG_NAV_H
#define DEBUG_NAV_H

// Downlink Video
/*#define DOWNLINK_VIDEO 	          */
#define DEBUG_VIDEO 	            1
#define DEBUG_MARK_FEATUREPOINTS  1
//#define DEBUG_MARK_FLOWSUM        1

#define DEBUG_OVERLAY_COLOR       255

// Send the summed horizontal flow
/*#define DOWNLINK_FLOWSUM 1*/

// Console print debug
#define DEBUG_CONSOLE_PRINT
#define DEBUG_NAV_PRINT
/*#define DEBUG_VISION_PRINT*/
//#define DEBUG_DEBUG


// Print macros
#ifdef DEBUG_CONSOLE_PRINT
  #define C_LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
    #define C_LOG(fmt, ...)
#endif

#ifdef DEBUG_NAV_PRINT
  #define N_LOG(fmt, ...) C_LOG(fmt, ##__VA_ARGS__)
#else
    #define N_LOG(fmt, ...)
#endif

#ifdef DEBUG_VISION_PRINT
  #define V_LOG(fmt, ...) C_LOG(fmt, ##__VA_ARGS__)
#else
    #define V_LOG(fmt, ...)
#endif

#endif
