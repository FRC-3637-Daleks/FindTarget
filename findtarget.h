#ifndef findtarget_h
#define findtarget_h

// === Features ===
// Uncomment this when this works properly
//#define FEATURE_RANGE
#define FEATURE_SAVE_IMAGES

// === Server configuration ===
#define SERVER_ADDR     "10.36.37.2"
#define SERVER_PORT     1130

// === Camera configuration ===
#define CAMERA_FOV_DEG          30
#define CAMERA_OFFSET_DEG       10
#define CAMERA_HEIGHT_IN        7
#define TARGET_TOP_HEIGHT_IN    68

// === Image configuration ===
#define IMAGE_FORMAT    CV_8UC3
#define IMAGE_WIDTH     1024
#define IMAGE_HEIGHT    768

// === Filter configuration ===
#define HUE_MIN_FILTER  30
#define HUE_MAX_FILTER  180
#define SAT_MIN_FILTER  80
#define SAT_MAX_FILTER  256
#define VAL_MIN_FILTER  50
#define VAL_MAX_FILTER  256

// === Threshold values for targets (px) ===
#define SMALL_TARGET_WIDTH          10
#define LARGE_TARGET_WIDTH          SMALL_TARGET_WIDTH * 4
#define NUM_CONTINOUS_OVER_THRESH   5

// === Misc Other ===
#define RANGE_INVALID   -1

#ifdef __arm__
#define RASPI_COMPILE
#endif

#endif
