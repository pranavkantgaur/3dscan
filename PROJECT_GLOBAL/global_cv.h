#include"opencv/cv.h"
#include"opencv/highgui.h"
#include<fcntl.h>
#include<errno.h>
#include <unistd.h>
#include<sys/ioctl.h>
#include <fenv.h>// to check for floating-point exception
#include<linux/videodev2.h>
#include<gphoto2/gphoto2-camera.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include<pcl/io/ply_io.h>

#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include<pcl/visualization/interactor_style.h>
#include<pcl/visualization/point_picking_event.h>



#include <jpeglib.h> // to decompress the JPEG image returned by gphoto2 as captured image by the camera.
#include <jerror.h>


#include <string.h>

#include<stdlib.h>


#include<stdio.h>
#include<math.h>

#include <assert.h>
#include <getopt.h>             /* getopt_long() */

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>

#define Camera_imagewidth 1600//3648
#define Camera_imageheight 1200//2736

#define Projector_imagewidth 1280//1024
#define Projector_imageheight 720//768



#define total_camera_pixels (Camera_imagewidth*Camera_imageheight)

#define total_projector_pixels (Projector_imagewidth*Projector_imageheight)


#define Pi 22.0/7.0



#define lmax 10 //this is the limit for the outer loop in phase unwrapping algorithm.

