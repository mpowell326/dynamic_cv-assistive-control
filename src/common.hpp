#ifndef COMMON_H
#define COMMON_H
/* -------------------------------------------------------------
    src/common.hpp        

    Purpose:    Common parameters needed for various modules
    
    Created:    Jan 2016
    Author:     Morgan
   ------------------------------------------------------------- */


#define VOXEL_LEAFSIZE  0.02f
#define MAX_RANGE       4.0

#define MAP_WIDTH       400
#define MAP_DEPTH       int(MAX_RANGE*100)

#define FLOOR_ANGLE_EPS 30.0

#define CAMERA_OFFSET   30
#define CAMERA_HEIGHT   60 // Camera mounting offset from floor (m)
#define CAMERA_ANGLE    20.0  // Camera mounting angle (deg). Positive is rotated torwards ground.
#define CHAIR_WIDTH     100

#define K1              3
#define K2              (30)*2*M_PI/360
#define FOS             100



#endif  // COMMON_H
