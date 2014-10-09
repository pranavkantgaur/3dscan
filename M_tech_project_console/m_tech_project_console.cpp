//TARGET:-This file acts like a console(control) application for 3D scanner application.
/*STEPS:-
1.Give a panel to user for selecting an option:-
  a).Generate pattern.
  b).Project pattern.
  c).Capture pattern.
  d).Compute wrapped phase.
  e).Compute unwrapped(relative) phase.
  f).Compute absolute phase.
  g).Perform system calibration.
  h).Perform triangulation.
  i).Save point-cloud.
//ADD MORE OPTIONS AFTER STEPS UPTO (i) are completely working.
  j).Exit.

*/

/////////////////////////////////////////////////////////////////////////////////////////Include files.

#include "../PROJECT_GLOBAL/global_cv.h"
#include"../PROJECT_GLOBAL/intermodule_dependencies.h"

int Z=0;

/////////////////////////////////////////////////////////////////////////////////////////Global variables.
extern CvCapture*camera;
extern int fd;
extern int exposure;
extern int exposure_range;
extern IplImage*cap;//Note:-This will be a common variable used by the modules to access frames from the camera.
extern int number_of_patterns_fringe;
extern int fringe_width_pixels;
extern int fringe_width_pixels;
extern int Monitor_width_pixels;

//extern Camera *camera;
extern GPContext *context;
extern CameraWidget *widget; //used for setting camera constrols
extern CameraFile* img_file;
extern int ret;
extern int cam_shutter_speed;
extern int cam_aperture;

extern float (*wrapped_phi_vertical)[Camera_imageheight];
extern float (*wrapped_phi_horizontal)[Camera_imageheight];

extern float (*unwrapped_phi_vertical)[Camera_imageheight];
extern float (*unwrapped_phi_horizontal)[Camera_imageheight];

extern long int (*c_p_map)[2];//THIS is the map containing camera to projector correspondance.
//0 component:-X
//1 component:-Y

extern int cam_gain;
extern int proj_gain;
extern int cam_exp;

//extern int fd_webcam;
//extern v4l2_control ctrl;


extern char filename[150];
extern int number_of_patterns_binary_vertical;

extern int (*code_vertical)[Camera_imageheight];
extern int (*code_horizontal)[Camera_imageheight];


extern int (*selected_region)[Camera_imageheight];//'1' means that pixel is selcted for 3D reconstruction
extern int (*valid_map_vertical)[Camera_imageheight];//for shadow & background removal.
extern int (*valid_map_horizontal)[Camera_imageheight];
extern int (*valid_map)[Camera_imageheight];//will contain the intersection of both above valid maps.

extern CvMat*cam_intrinsic_mat;
extern CvMat*cam_dist_vect;
extern CvMat*cam_world_rot_vect;//relative rotation & translation vectors.
extern CvMat*cam_world_trans_vect;
extern CvMat*cam_world_rot_mat;
extern CvMat*proj_intrinsic_mat;
extern CvMat*proj_dist_vect;
extern CvMat*proj_world_rot_vect;
extern CvMat*proj_world_trans_vect;
extern CvMat*proj_world_rot_mat;

extern CvMat*proj_cam_rot_mat;
extern CvMat*proj_cam_trans_vect;

extern double (*intersection_points)[Camera_imageheight][3];//it will contain the coordinates of point of intersection for each camera pixel(origin will be Camera-coordinate-system origin)

extern char* dev_name;

float rot_step;

float tx;
float ty;
float tz;

IplImage*selection_image;
IplImage*internal_image;
bool region_selected=false;
bool start_selection=false;
CvPoint previous_point;


void select_region_callback(int event,int x,int y,int flag,void* param)
{
    if(Z!=0)
        return;

    switch(event)
    {
    case CV_EVENT_LBUTTONDOWN:
        //cvCircle(selection_image,cvPoint(x,y),1,cvScalar(0,255,0),-1);
        //cvShowImage("Select region:",selection_image);
        //internal_image->imageData[y*internal_image->widthStep+x]=255;
        start_selection=true;
        previous_point.x=x;
        previous_point.y=y;
        break;

    case CV_EVENT_MOUSEMOVE:
        if(start_selection)
        {
            cvLine(selection_image,previous_point,cvPoint(x,y),cvScalar(0,255,0));
            cvLine(internal_image,previous_point,cvPoint(x,y),cvScalar(255));
            //printf("X1=%d\tY1=%d\tX2=%d\tY2=%d\n",previous_point.x,previous_point.y,x,y);
            previous_point.x=x;
            previous_point.y=y;
            cvShowImage("Select region:",selection_image);

            fflush(stdout);
        }
        break;

    case CV_EVENT_LBUTTONUP:
        region_selected=true;
        start_selection=false;
    }

    return;
}



//This function will provide interactive scissor for selecting the desired region out of colplete scanned region.
void image_scissor()
{
    /*
    Step:
    1.User interactively selects the region in wrapped phase image.
    2.All points of the boundary of region are recorded & drawn on image for reference of user.
    3.Create a blank image internally,with white boundary selected by user.
    4.In internal image,search pixel with min.y & then min.x component.
    5.Validate all pixels in wrapped phase within 2 successive white pixels of border in a single row.
    6.After 2nd white pixel is passed,look for another valid region in the same row in case selected region is concave.
    7.Perform same operation for all following rows.
    */
    cvNamedWindow("Select region:",0);
    cvSetMouseCallback("Select region:",select_region_callback,(void*)NULL);

    sprintf(filename,"Captured_patterns/Coded_patterns/Gray_coded/Vertical/Undistorted/Captured_image_%d.bmp",number_of_patterns_binary_vertical-1);
    selection_image=cvLoadImage(filename);
    internal_image=cvCreateImage(cvSize(Camera_imagewidth,Camera_imageheight),IPL_DEPTH_8U,1);
    cvSet(internal_image,cvScalar(0));

    region_selected=false;

    while(1)
    {
        cvShowImage("Select region:",selection_image);

        if(cvWaitKey(30)==27 || (region_selected==true))
        {
            break;

        }
    }

    //cvSetMouseCallback("Select region:",NULL,NULL);

    int p1=-1;

    selected_region=new int[Camera_imagewidth][Camera_imageheight];

    //Region selection done or user voluantarily aborted the selection
    if(region_selected==true)
    {
        for(int i=0; i<Camera_imagewidth; i++)
            for(int j=0; j<Camera_imageheight; j++)
            {
                selected_region[i][j]=0;

            }
        //fill the region & validate the wrapped phase

        for(int j=0; j<Camera_imageheight; j++)
        {
            p1=-1;

            for(int i=0; i<Camera_imagewidth; i++)
            {
                if((internal_image->imageData[j*internal_image->widthStep+i]!=0) && (p1==-1)) //start point
                {
                    p1=i;
                    //Search for its end:

                    for(i=p1+1; i<Camera_imagewidth; i++)
                    {
                        if(internal_image->imageData[j*internal_image->widthStep+i]!=0)
                        {
                            //lets fill the region
                            for(int h=p1+1; h<i; h++)
                            {
                                internal_image->imageData[j*internal_image->widthStep+h]=(unsigned char)255;
                                selected_region[h][j]=1;
                            }

                            break;
                        }

                    }

                    p1=-1;
                    i--;

                }

            }
        }
        cvSaveImage("i1.jpg",internal_image);

    }


    Z=1;

    return;
}



///////////////////////////////////////////////////////////////////////////////////////Console function.
extern int number_of_patterns_binary_horizontal;
int main()
{
    /*
    //lets intialize the camera.
        if((camera=cvCreateCameraCapture(0))==NULL)
        {
            printf("No Camera found,exiting...");
            return 0;
        }

        //lets set its properties as well.

        cvSetCaptureProperty(camera,CV_CAP_PROP_FRAME_WIDTH,Camera_imagewidth);
        cvSetCaptureProperty(camera,CV_CAP_PROP_FRAME_HEIGHT,Camera_imageheight);

        cvSetCaptureProperty(camera,CV_CAP_PROP_AUTO_EXPOSURE,1); //setting exposure manipulation to Manual mode.

        cvSetCaptureProperty(camera,CV_CAP_PRO,200);

        printf("Exp. value:%d",(int)cvGetCaptureProperty(camera,CV_CAP_PROP_EXPOSURE));
    */


    fd = open("/dev/video0", O_RDWR);
    if (fd == -1)
    {
        perror("Opening video device");
        return 1;
    }

    if(print_caps())
        return 1;

    if(init_mmap())
        return 1;

    //Lets create the respective windows for camera & projector images.
    cvNamedWindow("Projector_pattern",0);
    cvNamedWindow("Camera_view",0);
    cvNamedWindow("Projector_brightness_control",0);

    cvMoveWindow("Projector_pattern",Monitor_width_pixels,0);//That is,projector is assumed to be present on right side of user panel so it will be shifted by camera width & height(currently it is in the same window as user panel).
    cvSetWindowProperty("Projector_pattern",CV_WND_PROP_FULLSCREEN,CV_WINDOW_FULLSCREEN);


    //Lets attach trackbars to camera & projector window to allow to change exposure...

    cvCreateTrackbar2("Projector_gain","Projector_brightness_control",&proj_gain,100,NULL);


    ///STEP-1

    printf("\nCalibrating system...\n");
    system_calibration();//that is to compute the internal geometry of camera.
    printf("\nSystem calibrated!!!\n");

    ///STEP-2
    printf("\nGenerating the fringe & coded patterns...\n");
    generate_pattern();
    printf("\nPatterns generated!!!\n");



    printf("Would you like to do full revolution scanning?");
    fflush(stdout);

    unsigned n_scans;

    if(cvWaitKey(0)=='y') // do 360 degree scanning
    {

        printf("\nEnter rotation step:");
        scanf("%f",&rot_step);

        printf("\nEnter translation tx,ty,tz");
        scanf("%f%f%f",&tx,&ty,&tz);

        n_scans=ceil(360/rot_step);

        printf("\n%d scans will be required to cover the object",n_scans);
    }

    else
        n_scans=1; // single view scanning

    unsigned t=0;

   while(t<n_scans)
   {

        printf("Start projecting & capturing?\n");
        printf("%d\t%d\n",number_of_patterns_binary_vertical,number_of_patterns_binary_horizontal);


        cvWaitKey(0); //any key will do!!

        ///STEP-3
        IplImage*white_image=cvCreateImage(cvSize(Projector_imagewidth,Projector_imageheight),IPL_DEPTH_8U,1);
        cvSet(white_image,cvScalar(255));

        cvShowImage("Projector_pattern",white_image);

        printf("Press 'c' for capturing texture image");

        fflush(stdout);

        preview_and_capture();

        cvSaveImage("Point_cloud/texture.bmp",cap);

        printf("\nTexture image saved!!");

        printf("\nProjecting & capturing the pattern...\n");
        project_and_capture_pattern(0);
        project_and_capture_pattern(1);
        printf("\nPatterns captured!!!\n");

        /// Lets switch-off the camera:
        //close(fd);

        ///STEP-4 & 5
        //Now we will select a region that will be reconstructed
        image_scissor();

        printf("\nComputing wrapped phase...\n");

        //wrapping for vertical pattern.
        printf("\nComputing vertical wrapped phase...\n");
        compute_wrapped_phase(0);

        //wrapping for horizontal pattern.
        printf("\nComputing horizontal wrapped phase...\n");
        compute_wrapped_phase(1);


        printf("\nComputing vertical unwrapped phase...\n");
        unwrap_phase(0);
        //Vertical phase unwrapped.

        printf("\nComputing horizontal unwrapped phase...\n");
        unwrap_phase(1);
        //Horizontal phase unwrapped.

        ///STEP-6
        printf("\nComputing correspondence...\n");
        compute_c_p_map();
        printf("\nCorrespondance computed!!!\n");


        ///STEP-7
        printf("\nTriangulating...\n");
        triangulate(); //3D reconstruction...
        printf("\nTriangulation done!!!\n");

        ///STEP-8
        //Lets do point cloud visualization.
        //save_point_cloud(t);
        save_point_cloud(t);
        t++;

    }

    if(n_scans>1)
        {
          printf("\nMerging view %d...",t);
          register_point_clouds(n_scans,tx,ty,tz,rot_step);
        }

    printf("\nRendering 3D view...");
    fflush(stdout);
    visualize_point_cloud();

    printf("\n3D scanner exiting...\n");

     close(fd);




    //close(fd_webcam);
/*
unsigned n_scans=3;
float tx=54.0,ty=0.0,tz=0.0,rot_step=120.0; // inverse rotate the object
register_point_clouds(n_scans,tx,ty,tz,rot_step);
*/

//Done!!
    return 0;

}

