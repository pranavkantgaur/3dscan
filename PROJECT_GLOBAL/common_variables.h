#include "/home/pranav/Desktop/PROJECT_GLOBAL/global_cv.h"


char filename[150];
char tagname[50];
int number_of_codes_vertical=40;
int number_of_codes_horizontal=23;
int number_of_patterns_binary_vertical=6;
int number_of_patterns_binary_horizontal=5;
int number_of_patterns_fringe=3;

int (*code_vertical)[Camera_imageheight];
int (*code_horizontal)[Camera_imageheight];

long int (*c_p_map)[2];


int (*selected_region)[Camera_imageheight];//'1' means that pixel is selcted for 3D reconstruction
int (*valid_map_vertical)[Camera_imageheight];//for shadow & background removal.
int (*valid_map_horizontal)[Camera_imageheight];
int (*valid_map)[Camera_imageheight];//will contain the intersection of both above valid maps.

int fringe_width_pixels_vertical=32;
int fringe_width_pixels_horizontal=32;

int Monitor_width_pixels=1280;

int cam_gain=40;//Used for setting camera & projector brighness.
int proj_gain=40;
int cam_exp=150;//Used for setting camera exposure time(currently 1/1000000th of a second or 1us)
CvCapture*camera;
IplImage*cap;
//Camera & projector matrices.
CvMat*cam_intrinsic_mat;
CvMat*cam_dist_vect;
CvMat*cam_world_rot_vect;//relative rotation & translation vectors.
CvMat*cam_world_trans_vect;
CvMat*cam_world_rot_mat;



CvMat*proj_intrinsic_mat;
CvMat*proj_dist_vect;
CvMat*proj_world_rot_vect;
CvMat*proj_world_trans_vect;
CvMat*proj_world_rot_mat;

CvMat*proj_cam_rot_mat;
CvMat*proj_cam_trans_vect;


CvMat*proj_world_coordinates;



float (*wrapped_phi_vertical)[Camera_imageheight];
float (*wrapped_phi_horizontal)[Camera_imageheight];

float (*unwrapped_phi_vertical)[Camera_imageheight];
float (*unwrapped_phi_horizontal)[Camera_imageheight];

double (*intersection_points)[Camera_imageheight][3];//it will contain the coordinates of point of intersection for each camera pixel(origin will be Camera-coordinate-system origin)
int fd;


//int fd_webcam;
//struct v4l2_control ctrl;

//int (*valid_camera_pixel)[Projector_imageheight][2];
//bool (*assigned)[Projector_imageheight];

float gamma_value;

/// Camera access functions.
int cam_shutter_speed=22;
int cam_aperture=0;

GPContext *context;
//Camera *camera;
CameraWidget *widget=NULL; //used for setting camera constrols
CameraFile* img_file=NULL;
int ret=-1; //represents return value for gphoto2 library functions.
CameraWidget *child=NULL;
unsigned long location;
JSAMPROW row_pointer[1];
char * raw_image= NULL;
struct jpeg_decompress_struct cinfo;
struct jpeg_error_mgr jerr;
unsigned long size;
const unsigned char *data;

// V4L2 device name
char*dev_name;
int exposure=0;
int exposure_range=10;


char* powershot_g7_aperture_map[]=
{
    "2.8",
    "3.2",
    "3.5",
    "4",
    "5.0",
    "5.6",
    "6.3",
    "7.1",
    "8"
};

char *powershot_g7_shutterspeed_map[]=
{
    "15",
    "13",
    "10",
    "8",
    "6",
    "5",
    "4",
    "3.2",
    "2.5",
    "2",
    "1.6",
    "1.3",
    "1",
    "0.8",
    "0.6",
    "0.5",
    "0.4",
    "0.3",
    "1/4",
    "1/5",
    "1/6",
    "1/8",
    "1/10",
    "1/13",
    "1/15",
    "1/20",
    "1/25",
    "1/30",
    "1/40",
    "1/50",
    "1/60",
    "1/80",
    "1/100",
    "1/125",
    "1/160",
    "1/200",
    "1/250",
    "1/320",
    "1/400",
    "1/500",
    "1/640",
    "1/800",
    "1/1000",
    "1/1250",
    "1/1600",
    "1/2000",
    "1/2500"
};

