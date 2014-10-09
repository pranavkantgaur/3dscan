/*Steps:-
1.CAMERA CALIBRATION.
2.PROJECTOR CALIBRATION.
3.EXTRINSIC CALIBRATION.
///////////////////////////////////////////////////////////////////////Step-1.
Camera calibration:-
Use already developed code.
///////////////////////////////////////////////////////////////////////Step-2.
Projector calibration:-
1.Project virtual checkerboard over the physical checkerboard.
2.Capture it using camera.
3.Detect corners for both physical checkerboard in that orientation and that of virtual checkerboard.
4.Undistort the detected corners.
5.Project the detected virtual checkerboard corners back to the screen.
6.Give this correspondance between screen points and projector image corners for projector calibration.
7.Done.
////////////////////////////////////////////////////////////////////////Step-3.
Extrinsic calibration(to determine the rotation & translation matrices for projector and camera wrt. world coordinate system):-
1.Position the physical checkerboard which be used as the reference plane for world coordinate system.
2.Capture image and detect the corners.
3.Compute extrinsic parameters for camera & projector(for projector i have to transfer the detected points into projectors coordinate space).
4.Done.
*/

///UPDATE 1: 28 June 2012
/*
METHOD:1
==>Allowing camera to be static & only projector to move since camera-world homography needs to be computed only once.
*/

/// UPDATE 2: 3 July 2012
/*
METHOD 2:
==>Calibration without virtual checkerboard:
Use only physical checkerboard to get projector image & using it to get image points for projector,object points are same for both(camera & projector) anyway!!
*/

/// UPDATE 3: 6 July 2012
/*
Making extrinsic calibration also based on 4 points for camera extrinsic & camera-wall homography calculation.
*/

/// UPDATE 4: 24 Aug 2012
/*
Making provision for VPCLib direct calibration methods in 'calibrate_projector()'.
*/


#include "/home/pranav/Desktop/PROJECT_GLOBAL/global_cv.h"
#include "../PROJECT_GLOBAL/intermodule_dependencies.h"
//ASSUMPTION:USER PERFORMS CALIBRATION WITH SAME NUMBER OF BOARD POSITION AS HE/SHE ENTERS.

#include<iostream>

//Module specific constants.
#define physical_checkerboard_square_mm 10.5f//26.5f //due to printer error:25mm->26.5mm

#define physical_checkerboard_X 5//10//9
#define physical_checkerboard_Y 4//8//6
#define virtual_checkerboard_X 14
#define virtual_checkerboard_Y 8
#define virtual_checkerboard_square_pixels 75


#define total_physical_checkerboard_corners (physical_checkerboard_X*physical_checkerboard_Y)
#define total_virtual_checkerboard_corners (virtual_checkerboard_X*virtual_checkerboard_Y)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Global variables.
extern IplImage*cap;
extern CvCapture*camera;

extern CvMat*cam_intrinsic_mat;
extern CvMat*cam_dist_vect;
extern CvMat*cam_world_rot_vect;
extern CvMat*cam_world_trans_vect;

extern CvMat*proj_intrinsic_mat;
extern CvMat*proj_dist_vect;
extern CvMat*proj_world_rot_vect;
extern CvMat*proj_world_trans_vect;



extern CvMat*cam_world_rot_mat;
extern CvMat*proj_world_rot_mat;

extern CvMat*proj_cam_rot_mat;
extern CvMat*proj_cam_trans_vect;



extern char filename[150];
extern int Monitor_width_pixels;
//The virtual checkerboard pattern.
IplImage*virtual_checkerboard_image=cvLoadImage("Projector_calibration/Virtual_calibration_rig/pico_checkerboard.bmp",CV_LOAD_IMAGE_GRAYSCALE);

IplImage*preview_image;//used for drawing circles of selected points.
IplImage*gray_preview_image;

bool corners_detected=false;
CvPoint2D32f*detected_points=new CvPoint2D32f[4];
static int point_number=0;


double proj_border_X_pixels=50.0f;
double proj_border_Y_pixels=20.0f;

extern CvMat*proj_world_coordinates;

extern long int (*c_p_map)[2];

extern int cam_gain;
extern int proj_gain;
extern int cam_exp;


extern int fd_webcam;
extern v4l2_control ctrl;

extern  int cam_shutter_speed;
extern  int cam_aperture;
extern char* powershot_g7_aperture_map;
extern char* powershot_g7_shutterspeed_map;

using namespace cv;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Camera calibration.
void calibrate_camera()
{


    int no_board;



    printf("\nEnter the number of checkerboard poses to be used for calibration:\n");
    scanf("%d",&no_board);


    CvMat*cam_rot_vect=cvCreateMat(no_board,3,CV_64FC1);//Local since they are only used for plotting and not by other external functions.
    CvMat*cam_trans_vect=cvCreateMat(no_board,3,CV_64FC1);


    CvMat*image_points=cvCreateMat(total_physical_checkerboard_corners*no_board,2,CV_64FC1);//Image points of detected checkerboard corners.
    CvMat*object_points=cvCreateMat(total_physical_checkerboard_corners*no_board,3,CV_64FC1);//Object points of detected chekerboard corners.
    CvPoint2D32f*corners;
    int*corner_count=new int;
    CvMat*corners_count=cvCreateMat(no_board,1,CV_32SC1);
    int found=0;
    int i=0;
    int w;
    int successes=0;
    corners=new CvPoint2D32f[total_physical_checkerboard_corners];


    if(no_board<=1)
    {

        printf("Atleast 2 boards are required,exiting....");
        exit(0);
    }


    //if number of boards are greater than 1 then calibration can be done.
    //create the capture device.
    //cap=cvQueryFrame(camera);
    preview_and_capture();

    //cap=camera_capture("Camera_calibration/junk.jpg");

    CvSize frame_size=cvGetSize(cap);
    IplImage*gray_image=cvCreateImage(cvGetSize(cap),cap->depth,1);  //this image will be used to compute subpixel accurate corners.
    IplImage*temp_image=cvCreateImage(cvGetSize(cap),cap->depth,3);
    //Start capturing and detecting the corners now!!


    while(i<no_board)
    {
        printf("Put your calibration rig in front and press 'c' in the \"Camera_View\" window(%d more views remaining)\n",no_board-i);
        fflush(stdout);

        //Allowing user to align the rig correctly.
        //while(cvWaitKey(30)!='c')
        //{
        //cap=cvQueryFrame(camera);
        //preview_and_capture();
        //cap=camera_preview("Camera_calibration/junk.jpg");
        //cvScale(cap,cap,2.*(cam_gain/100.),0);
        // set_camera_control("shutterspeed",(powershot_g7_shutterspeed_map+cam_shutter_speed));
        // set_camera_control("aperture",(powershot_g7_aperture_map+cam_aperture));

        //cvShowImage("Camera_view",cap);
        //}



        while(!found)
        {
            //cap=cvQueryFrame(camera);
            //cvScale(cap,cap,2.*(cam_gain/100.),0);
            preview_and_capture();
            /*
                        cap=camera_capture("Camera_calibration/junk1.jpg");


                        set_camera_control("shutterspeed",(powershot_g7_shutterspeed_map+cam_shutter_speed));
                        set_camera_control("aperture",(powershot_g7_aperture_map+cam_aperture));
            */

            cvCvtColor(cap,gray_image,CV_RGB2GRAY);

            found=cvFindChessboardCorners(gray_image,cvSize(physical_checkerboard_X,physical_checkerboard_Y),corners,corner_count);
            sprintf(filename,"Camera_calibration/Captured_images/View_%d.bmp",i);

            cvWaitKey(100);
            cvCopyImage(cap,temp_image);

            cvDrawChessboardCorners(cap,cvSize(physical_checkerboard_X,physical_checkerboard_Y),corners,*corner_count,found);

            cvShowImage("Camera_view",cap);


        }

        //cvSaveImage(filename,temp_image);
        cvSaveImage(filename,cap);

        found=0;//i.e.,for next iteration(next pose).


        //Lets do subpixel accurate corner computation.
        //cvFindCornerSubPix(gray_image,corners,*corner_count,cvSize(11,11),cvSize(-1,-1),cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,30,0.01));



        //Now 'corners' contains the sub pixel accurate corners.

        //Now if the checkerboard corners are detected,then we must store its detected corners to 'image_point' matrix & its corresponding object points to 'object_points' matrix.
        //First,store the image points.
        w=0;
        for(int j=i*total_physical_checkerboard_corners; w<total_physical_checkerboard_corners; w++,j++)
        {
            CV_MAT_ELEM(*image_points,double,j,0)=(double)corners[w].x;
            CV_MAT_ELEM(*image_points,double,j,1)=(double)corners[w].y;

        }



        //Now storing the no. of detected corners.

        CV_MAT_ELEM(*corners_count,int,i,0)=*corner_count;

        i++;

        if(cvWaitKey(30)==27)
        {
            if(i<2)
            {
                printf("At least 2 poses are required,repeat again,but now exiting....");
                exit(0);
            }
        }

    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////WE ARE OUTSIDE THE 'while' LOOP

    successes=i;


    //Now we have to allocate matrices to store image and corresponding object points to be given to cvCameraCalibrate2().
    //We are reallocating point matrices for the cases when the user gets fed up and exits in the middle then he/she might have given lesser number of calibaration rig poses,hence matrix must be of that size only.


    CvMat*image_points2=cvCreateMat(successes*total_physical_checkerboard_corners,2,CV_64FC1);
    CvMat*object_points2=cvCreateMat(successes*total_physical_checkerboard_corners,3,CV_64FC1);
    CvMat*corner_count2=cvCreateMat(successes,1,CV_32SC1);

    //Now simply copying the actually given image points into image_points2 from image_points matrix.
    //Similary,for object points as well.
    for(int g=0; g<successes*total_physical_checkerboard_corners; g++)
    {
        CV_MAT_ELEM(*image_points2,double,g,0)=CV_MAT_ELEM(*image_points,double,g,0);
        CV_MAT_ELEM(*image_points2,double,g,1)=CV_MAT_ELEM(*image_points,double,g,1);

    }


    int y=0;
    for(int h=0; h<successes; h++) //i.e, for every view.
    {
        for(int row_no=1; row_no<=physical_checkerboard_Y; row_no++)
            for(int col_no=1; col_no<=physical_checkerboard_X; col_no++)
            {
                CV_MAT_ELEM(*object_points2,double,y,0)=(double)(col_no*physical_checkerboard_square_mm);
                CV_MAT_ELEM(*object_points2,double,y,1)=(double)(row_no*physical_checkerboard_square_mm);
                CV_MAT_ELEM(*object_points2,double,y,2)=(double)0.0;
                y++;

            }

    }
    //Similarly,for number of detected corners in each calibration rig pose.

    for(int q=0; q<successes; q++)
        CV_MAT_ELEM(*corner_count2,int,q,0)=CV_MAT_ELEM(*corners_count,int,q,0);

    //Now allocating space for the upcoming rotation,translation,intrinsic & distortion matrices.


    cam_intrinsic_mat=cvCreateMat(3,3,CV_64FC1);
    cam_dist_vect=cvCreateMat(5,1,CV_64FC1);

    //Now setting the 'calibration flags' that are used by calibration function.

    int calib_flags=0;
    calib_flags=CV_CALIB_FIX_K3|CV_CALIB_ZERO_TANGENT_DIST;

    //lets calibrate the camera now!!


    printf("\nCalibrating the camera...\n");


    double reprojection_error=0.0;

    reprojection_error=cvCalibrateCamera2(object_points2,image_points2,corner_count2,frame_size,cam_intrinsic_mat,cam_dist_vect,cam_rot_vect,cam_trans_vect,calib_flags);

    //Now save all the computed parameters.


    printf("\nCamera calibrated,saving the parameters...\n");

    cvSave("Camera_calibration/Matrices/cam_intrinsic_mat.xml",cam_intrinsic_mat);
    cvSave("Camera_calibration/Matrices/cam_distortion_vect.xml",cam_dist_vect);

    //Saving reprojection error
    FILE*fd_rep_error=fopen("Camera_calibration/Matrices/reprojection_error.txt","w");
    fprintf(fd_rep_error,"%lf",reprojection_error);
    fclose(fd_rep_error);


    CvMat*mat_3_3=cvCreateMat(1,3,CV_64FC1);

    for(int y=0; y<successes; y++)
    {
        //Copying rotation vector.
        for(int i=0; i<3; i++)
            CV_MAT_ELEM(*mat_3_3,double,0,i)=CV_MAT_ELEM(*cam_rot_vect,double,y,i);

        sprintf(filename,"Camera_calibration/Matrices/world_to_cam_rot_vect_%d.xml",y);
        //cvSave(filename,mat_3_3);

        //Copying translation vector.
        for(int i=0; i<3; i++)
            CV_MAT_ELEM(*mat_3_3,double,0,i)=CV_MAT_ELEM(*cam_trans_vect,double,y,i);

        sprintf(filename,"Camera_calibration/Matrices/world_to_cam_trans_vect_%d.xml",y);
        //cvSave(filename,mat_3_3);
    }


    //Throwing off the rotation and translation parameters as they are not of use in the current context.
    //Now releasing all the resources.

    cvReleaseMat(&object_points2);
    cvReleaseMat(&image_points2);
    cvReleaseMat(&image_points);
    cvReleaseMat(&object_points);
    cvReleaseMat(&corner_count2);
    cvReleaseMat(&corners_count);
    delete corner_count;
    delete [] corners;


    cvReleaseMat(&cam_rot_vect);
    cvReleaseMat(&cam_trans_vect);

    cvReleaseMat(&corners_count);

    cvReleaseImage(&gray_image);

    fclose(fd_rep_error);

    cvReleaseMat(&mat_3_3);


//Done!!
    return;

}






//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////FOLLOWING ARE THE SUPPORTING FUNCTIONS FOR PROJECTOR CALIBRATION.
//Following function will return the object points corresponding to the virtual checkerboard corners.
void compute_proj_obj_points(CvMat*cwhomo,CvMat* proj_detected_corners,CvMat*world_proj_points_result)
{
//I have to undistort the captured virtual checkerboard points.
    CvMat*dist_proj_points=cvCreateMat(total_virtual_checkerboard_corners,1,CV_64FC2);
    CvMat*undist_proj_points=cvCreateMat(total_virtual_checkerboard_corners,1,CV_64FC2);
    CvMat*undist_proj_points_one_channel=cvCreateMat(3,total_virtual_checkerboard_corners,CV_64FC1);//Compatible with multiplication.
    CvMat*world_proj_points=cvCreateMat(3,total_virtual_checkerboard_corners,CV_64FC1);//This will contain the world coordinates.


//Lets do some copying stuff.
    for(int i=0; i<total_virtual_checkerboard_corners; i++)
        cvSet1D(dist_proj_points,i,cvScalar(CV_MAT_ELEM(*proj_detected_corners,double,i,0),CV_MAT_ELEM(*proj_detected_corners,double,i,1)));


    cvUndistortPoints(dist_proj_points,undist_proj_points,cam_intrinsic_mat,cam_dist_vect);


//Lets convert the 2-channel undistoted array to 1-channel undistoted array.
    CvScalar pd;

    for(int j=0; j<total_virtual_checkerboard_corners; j++)
    {
        pd=cvGet1D(undist_proj_points,j);

        CV_MAT_ELEM(*undist_proj_points_one_channel,double,0,j)=(double)pd.val[0];
        CV_MAT_ELEM(*undist_proj_points_one_channel,double,1,j)=(double)pd.val[1];
        CV_MAT_ELEM(*undist_proj_points_one_channel,double,2,j)=1.0f;
    }

//I have to convert these points back to camera pixel coordinates before using.
    cvMatMul(cam_intrinsic_mat,undist_proj_points_one_channel,undist_proj_points_one_channel);

//lets homogenize them..
    for(int q=0; q<total_virtual_checkerboard_corners; q++)
        for(int a=0; a<3; a++)
            CV_MAT_ELEM(*undist_proj_points_one_channel,double,a,q)/=CV_MAT_ELEM(*undist_proj_points_one_channel,double,2,q);





//Lets multipy these points with cwhomo to get the points in world coordinate system.
    cvMatMul(cwhomo,undist_proj_points_one_channel,world_proj_points);



//homogenize it.
    for(int k=0; k<total_virtual_checkerboard_corners; k++)
        for(int p=0; p<3; p++)
        {
            CV_MAT_ELEM(*world_proj_points,double,p,k)/=CV_MAT_ELEM(*world_proj_points,double,2,k);

        }
//homogenized!!
//Lets convert in the form usable for projector calibration.
    for(int k=0; k<total_virtual_checkerboard_corners; k++)
        CV_MAT_ELEM(*world_proj_points,double,2,k)=0.0f;

    cvTranspose(world_proj_points,world_proj_points_result);




//Lets copy back the undistorted projector's detected corner for further use.
    for(int n=0; n<total_virtual_checkerboard_corners; n++)
        for(int p=0; p<2; p++)
            CV_MAT_ELEM(*proj_detected_corners,double,n,p)=CV_MAT_ELEM(*undist_proj_points_one_channel,double,p,n);


//Deallocation...
    cvReleaseMat(&dist_proj_points);
    cvReleaseMat(&undist_proj_points);
    cvReleaseMat(&undist_proj_points_one_channel);
    cvReleaseMat(&world_proj_points);



//Done.
    return ;


}










//Following function will return the difference image of its argument images.(Similar to background removal program).
void subtract_images(IplImage*src_1,IplImage*src_2,IplImage*result)
{
// A-B=C
    /*Steps:-
    1.Convert the images in double arrays.
    2.Perform the difference A-B.
    3.Normalize the difference array.
    4.Convert it into the image(gray image) by 127.0+128.0*normalized_value.
    5.Return the image.
    6.Done!!
    */


//Lets do it!!
//1.Convert the images in double arrays.

    double (*arr_2)[Camera_imageheight]=new double[Camera_imagewidth][Camera_imageheight];//Intensities of image 2.('arr_2' is pointer to 1-D array of size 'Camera_imageheight'.)
    double(*arr_result)[Camera_imageheight]=new double[Camera_imagewidth][Camera_imageheight];//resulting array(difference array)
//    double (*arr_result_normalize)[Camera_imageheight]=new double[Camera_imagewidth][Camera_imageheight];//Normalized difference.
    double (*arr_1)[Camera_imageheight]=new double[Camera_imagewidth][Camera_imageheight];//Intensities of image 1.




    for(int h=0; h<Camera_imageheight; h++)
        for(int c=0; c<Camera_imagewidth; c++)
        {
            arr_1[c][h]=(double)(unsigned char)(src_1->imageData[c+h*src_1->widthStep]);
            arr_2[c][h]=(double)(unsigned char)(src_2->imageData[c+h*src_2->widthStep]);
        }

//2.Perform the difference A-B.
    for(int a=0; a<Camera_imageheight; a++)
        for(int b=0; b<Camera_imagewidth; b++)
        {
            if(arr_1[b][a]>=arr_2[b][a])
                arr_result[b][a]=arr_1[b][a]-arr_2[b][a];
            else
                arr_result[b][a]=0.0;
        }

    for(int g=0; g<Camera_imageheight; g++)
        for(int v=0; v<Camera_imagewidth; v++)
            result->imageData[v+g*result->widthStep]=(unsigned char)(arr_result[v][g]);





    /* An improvement:-
       Subtract(A,B)=A-B;A>B
                    =A;A<B

    */
    delete[] arr_1;
    delete[] arr_2;
    delete[] arr_result;
//double (*arr_result_normalize)[Camera_imageheight]=new double[Camera_imagewidth][Camera_imageheight];//Normalized difference.





//5.Done!!
    return ;

}







//Following function can compute the homography between given distorted image points & object points.
void findhomography(CvMat* dist_cam_image_points,CvMat*cam_obj_points,int total_points,CvMat*cwhomo)
{
    //Undistort points.
    //find homography & return it.
    //Done.

    CvMat*undist_points_two_channel=cvCreateMat(total_points,1,CV_64FC2);
    CvMat*dist_points_two_channel=cvCreateMat(total_points,1,CV_64FC2);
    CvMat*undist_points_one_channel=cvCreateMat(3,total_points,CV_64FC1);

    CvMat*image_point=cvCreateMat(total_points,3,CV_64FC1);
    CvMat*object_points=cvCreateMat(total_points,3,CV_64FC1);



    for(int j=0; j<total_points; j++)
    {
        CV_MAT_ELEM(*object_points,double,j,0)=CV_MAT_ELEM(*cam_obj_points,double,j,0);
        CV_MAT_ELEM(*object_points,double,j,1)=CV_MAT_ELEM(*cam_obj_points,double,j,1);
        CV_MAT_ELEM(*object_points,double,j,2)=1.0;
        printf("%lf %lf %lf\n",CV_MAT_ELEM(*object_points,double,j,0),CV_MAT_ELEM(*object_points,double,j,1),CV_MAT_ELEM(*object_points,double,j,2));
    }


//Converting input distorted-point set to corresponding 2-channel matrix.
    for(int i=0; i<total_points; i++)
        cvSet1D(dist_points_two_channel,i,cvScalar(CV_MAT_ELEM(*dist_cam_image_points,double,i,0),CV_MAT_ELEM(*dist_cam_image_points,double,i,1)));



    cvUndistortPoints(dist_points_two_channel,undist_points_two_channel,cam_intrinsic_mat,cam_dist_vect,NULL,NULL);




//Converting 2-channel undistoted image points to corresponding 1-channel undistorted image points.
    CvScalar pd;
    for(int k=0; k<total_points; k++)
    {
        pd=cvGet1D(undist_points_two_channel,k);
        CV_MAT_ELEM(*undist_points_one_channel,double,0,k)=(double)pd.val[0];
        CV_MAT_ELEM(*undist_points_one_channel,double,1,k)=(double)pd.val[1];
        CV_MAT_ELEM(*undist_points_one_channel,double,2,k)=1.0;
    }


    cvMatMul(cam_intrinsic_mat,undist_points_one_channel,undist_points_one_channel);

//homogenize..
    for(int g=0; g<total_points; g++)
        for(int c=0; c<3; c++)
            CV_MAT_ELEM(*undist_points_one_channel,double,c,g)/=CV_MAT_ELEM(*undist_points_one_channel,double,2,g);


//Lets copy this array to 'image_points' array..

    for(int g=0; g<total_points; g++)
        for(int n=0; n<3; n++)
        {
            CV_MAT_ELEM(*image_point,double,g,n)=CV_MAT_ELEM(*undist_points_one_channel,double,n,g);
            printf("%lf\n",CV_MAT_ELEM(*image_point,double,g,n));
        }

//Compute homography.
    cvFindHomography(image_point,object_points,cwhomo,CV_RANSAC);

//Now i have the subpixel accurate detected corners of virtual checkerboard.
//Lets compute the corresponding world-coordinates.(to be used in extrinsic calibration of projector)
//I have to find out the homography between camera-space & world-space.
//    findhomography(cam_image_points,cam_obj_points,total_physical_checkerboard_corners,cwhomo);
    printf("In function:\n");
    CvMat*cwhomo_float=cvCreateMat(3,3,CV_32FC1);
    for(int u=0; u<3; u++)
        for(int v=0; v<3; v++)
        {
            CV_MAT_ELEM(*cwhomo_float,float,u,v)=(float)CV_MAT_ELEM(*cwhomo,double,u,v);
            printf("%lf & %f\n",CV_MAT_ELEM(*cwhomo,double,u,v),CV_MAT_ELEM(*cwhomo_float,float,u,v));
        }

//De-allocation...
    cvReleaseMat(&undist_points_two_channel);
    cvReleaseMat(&dist_points_two_channel);
    cvReleaseMat(&undist_points_one_channel);
    cvReleaseMat(&image_point);
    cvReleaseMat(&object_points);



//Done!!

    return;



}



///////////////////////////////////////////////////////////////////////////////////Calibrate projector.
///TO BE DONE:AUTOMATE CORNER-DETECTION!!!!
void callback_get_point(int event,int x,int y,int flags,void* param)
{

    switch(event)
    {
    case CV_EVENT_LBUTTONDOWN:
        detected_points[point_number].x=(float)x;
        detected_points[point_number].y=(float)y;

        break;


    case CV_EVENT_LBUTTONUP:
        ///Do sub-pixel corner detection & draw the detected corner
        cvFindCornerSubPix(gray_preview_image,&detected_points[point_number],1,cvSize(2,2),cvSize(-1,-1),cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,30,0.01));
        cvCircle(cap,cvPointFrom32f(detected_points[point_number]),5,cvScalar(255,0,0));
        point_number++;
        if(point_number==4)
            corners_detected=true;
    }





    return;
}



void detect_wall_points(CvMat*cam_image_points)
{


    printf("\nPlease select 4 wall points for computing camera-wall homography:");
    fflush(stdout);


    cvNamedWindow("Click on 4 points:",0);
    cvNamedWindow("Preview camera-wall alignment",0);

    cvSetMouseCallback("Click on 4 points:",callback_get_point,NULL);

    printf("\nCapture wall position?");
    fflush(stdout);
    //bool alignment_done=false;

    //while(alignment_done==false)
    // {
    /*     memset(&ctrl,0,sizeof(ctrl));
         ctrl.id=V4L2_CID_EXPOSURE_ABSOLUTE;
         ctrl.value=cam_exp;

         if((ioctl(fd_webcam,VIDIOC_S_CTRL,&ctrl)==-1) && (errno!=ERANGE))
             perror("");

      */
    //preview_image=camera_preview_1();
    preview_and_capture();
    cvShowImage("Preview camera-wall alignment",cap);

    //if(cvWaitKey(10)=='y')
    //      alignment_done=true;
    //  }


    printf("\nPlease click on 4 points");
    fflush(stdout);

    //preview_image=camera_capture("Captured_patterns/junk.bmp");
    //preview_and_capture();
    gray_preview_image=cvCreateImage(cvGetSize(cap),cap->depth,1);

    cvCvtColor(cap,gray_preview_image,CV_RGB2GRAY);


    while(1)
    {

        cvWaitKey(30);
        cvShowImage("Click on 4 points:",cap);
        if(corners_detected==true)
            break;
    }








    //Lets copy the image points.
    for(int i=0; i<4; i++)
    {
        CV_MAT_ELEM(*cam_image_points,double,i,0)=(double)detected_points[i].x;
        CV_MAT_ELEM(*cam_image_points,double,i,1)=(double)detected_points[i].y;
        CV_MAT_ELEM(*cam_image_points,double,i,2)=0.0;
    }




    return;
}

//Defines world coordinates of markers on wall.
void define_wall_object_points(CvMat* cam_obj_points)
{
    ///UPDATE: 30 Aug 2012
    /// Calibration rig has 8 points

    ///UPDATE: 04 Sept 2012
    ///Calibration rig with 4 points
    /*
    Structure of rig:

           +                  +
                +       +
                +       +
           +                  +
    */
    /*
        //Outer 4 points
        double X_shift=1177.0;
        double Y_shift=593.0;

        for(int i=0; i<4; i++)
        {


            CV_MAT_ELEM(*cam_obj_points,double,i,0)=X_shift*(i%2);
            CV_MAT_ELEM(*cam_obj_points,double,i,1)=Y_shift*floor(i/2);
            CV_MAT_ELEM(*cam_obj_points,double,i,2)=1.0;

        }

        //Inner 4 points
        double inner_X_shift=341.0;
        double inner_Y_shift=151.0;
        double X_width=500.0;
        double Y_width=300.0;

        for(int j=4;j<8;j++)
        {
        CV_MAT_ELEM(*cam_obj_points,double,j,0)=inner_X_shift+X_width*(j%2);
        CV_MAT_ELEM(*cam_obj_points,double,j,1)=inner_Y_shift+Y_width*(floor(j/2)-2);
        CV_MAT_ELEM(*cam_obj_points,double,j,2)=1.0f;
        }
    */
    double X_shift=116.0;//345.0;//1900.0;
    double Y_shift=64.0;//131.0;//1700.0;

    for(int i=0; i<4; i++)
    {
        CV_MAT_ELEM(*cam_obj_points,double,i,0)=X_shift*(i%2);
        CV_MAT_ELEM(*cam_obj_points,double,i,1)=Y_shift*floor(i/2);
        CV_MAT_ELEM(*cam_obj_points,double,i,2)=0.0;

    }



    return;
}





void calibrate_projector()
{
    CvFileStorage*fs_cam_intrinsic_mat=cvOpenFileStorage("Camera_calibration/Matrices/cam_intrinsic_mat.xml",0,CV_STORAGE_READ);
    CvFileStorage*fs_cam_distortion_vect=cvOpenFileStorage("Camera_calibration/Matrices/cam_distortion_vect.xml",0,CV_STORAGE_READ);
    cam_intrinsic_mat=(CvMat*)cvReadByName(fs_cam_intrinsic_mat,0,"cam_intrinsic_mat");
    cam_dist_vect=(CvMat*)cvReadByName(fs_cam_distortion_vect,0,"cam_distortion_vect");



    int number_of_boards=0;
    printf("\nEnter number of boards for projector calibration:");
    scanf("%d",&number_of_boards);

    CvMat*proj_rot_vect=cvCreateMat(number_of_boards,3,CV_64FC1);
    CvMat*proj_trans_vect=cvCreateMat(number_of_boards,3,CV_64FC1);

    //Lets capture & detect physical checkerboard.
    int found=0;
    preview_and_capture();
    IplImage*gray_temp_camera_capture=cvCreateImage(cvGetSize(cap),IPL_DEPTH_8U,1);
    IplImage*temp_projector_capture;
    IplImage*gray_temp_projector_capture=cvCreateImage(cvGetSize(gray_temp_camera_capture),gray_temp_camera_capture->depth,gray_temp_camera_capture->nChannels);
    int physical_checkerboard_corner_count=0;
    int virtual_checkerboard_corner_count=0;
    IplImage*differential_image=cvCreateImage(cvGetSize(gray_temp_camera_capture),gray_temp_camera_capture->depth,gray_temp_camera_capture->nChannels);

    CvMat*cam_obj_points=cvCreateMat(4,3,CV_64FC1);//assuming 4 coordinates to be sufficient to get homography
    CvMat*cam_image_points=cvCreateMat(4,3,CV_64FC1);
    CvPoint2D32f*detected_physical_checkerboard_corners=new CvPoint2D32f[total_physical_checkerboard_corners];

    CvMat*proj_obj_points=cvCreateMat(total_virtual_checkerboard_corners*number_of_boards,3,CV_64FC1);
    CvMat*proj_image_points=cvCreateMat(total_virtual_checkerboard_corners*number_of_boards,2,CV_64FC1);
    CvMat*proj_detected_corners=cvCreateMat(total_virtual_checkerboard_corners,2,CV_64FC1);//It will be used to temporarily hold the detected virtual checkerboard corners for each of the calibration poses.
    CvPoint2D32f*detected_virtual_checkerboard_corners=new CvPoint2D32f[total_virtual_checkerboard_corners];
    CvMat*cwhomo=cvCreateMat(3,3,CV_64FC1);//This camera to world homography.


    //calibration result.
    proj_intrinsic_mat=cvCreateMat(3,3,CV_64FC1);
    proj_dist_vect=cvCreateMat(5,1,CV_64FC1);


    CvMat* temp_point_count=cvCreateMat(number_of_boards,1,CV_32SC1);//It will contain the number of points detected per checkerboard orientation.
    cvNamedWindow("test_image",0);
    //Lets create some windows onto which we'll disply the preview of live scene.
    int successes=0;
    bool alignment_done=false;
    IplImage*blank=cvCreateImage(cvSize(Projector_imagewidth,Projector_imageheight),8,1);

    cvShowImage("Projector_pattern",blank);
    bool satisfied=false;

    ///Approach 2:In this approach we will project points only within the region withing detected corners area(to get more valid homography)
    detect_wall_points(cam_image_points);
    define_wall_object_points(cam_obj_points);
    cvFindHomography(cam_image_points,cam_obj_points,cwhomo,CV_RANSAC);


    ///FOR VPCLib tool

    FileStorage fs_H("VPCLib_data/H.yml",FileStorage::WRITE);

    fs_H<<"H"<<cwhomo;

    fs_H.FileStorage::release();

    IplImage*temp_virtual_checkerboard=cvCreateImage(cvSize(Projector_imagewidth,Projector_imageheight),8,1);


    //Lets open a file to write projector object points into.
    //FILE*fd_projector_object_points=fopen("../3D_plot/projector_object_points.txt","w+");

    CvMat*temp_proj_image_points=cvCreateMat(total_virtual_checkerboard_corners,2,CV_64FC1);
    CvMat*temp_proj_obj_points=cvCreateMat(total_virtual_checkerboard_corners,3,CV_64FC1);

    printf("\nNow projecting virtual checkerboard for detection...\n");
    while(successes<number_of_boards)
    {

        //Now physical checkerboard corners are detected and are stored in 'detected_physical_checkerboard_corners' array.
        sprintf(filename,"VPCLib_data/match_cam_proj_%d.yml",successes);
        FileStorage fs_match_cam_proj(filename,FileStorage::WRITE);

        printf("\nView-%d\n",successes);
        fflush(stdout);

        cvShowImage("Projector_pattern",virtual_checkerboard_image);
/*
        satisfied=false;

        while(satisfied!=true)
        {
            cvCopyImage(virtual_checkerboard_image,temp_virtual_checkerboard);//for next iteration
            preview_and_capture();
            cvScale(temp_virtual_checkerboard,temp_virtual_checkerboard,2.*(proj_gain/100.),0);
            cvShowImage("Projector_pattern",temp_virtual_checkerboard);
            cvShowImage("Camera_view",cap);
            printf("\nIs it OK??");
            fflush(stdout);
            if(cvWaitKey(0)=='y')
                satisfied=true;
        }
       satisfied=false;
*/

        found=0;
        while(found==0)
        {
            printf("\nPress \'c'\ to capture projector view");
            fflush(stdout);
            preview_and_capture();
            cvCvtColor(cap,gray_temp_projector_capture,CV_RGB2GRAY);
            //Ideally this image should be:-Physical+virtual checkerboard(superimposed).
            //Lets subtract the image of physical checkerboard from this image.(to get the image of virtual checkerboard)
            cvShowImage("Camera_view",cap);//It will show the resulting differential image as interpreted by the camera.(Debugging)
            //subtract_images(gray_temp_projector_capture,gray_temp_camera_capture,differential_image);//This function will return a differential image which contains only the contribution from virtual checkerboard.
            found=cvFindChessboardCorners(gray_temp_projector_capture,cvSize(virtual_checkerboard_X,virtual_checkerboard_Y),detected_virtual_checkerboard_corners,&virtual_checkerboard_corner_count);


            cvWaitKey(30);
        }






        //Lets add the subpixel accuracy function(PROJECTOR).
//        cvFindCornerSubPix(differential_image,detected_virtual_checkerboard_corners,virtual_checkerboard_corner_count,cvSize(11,11),cvSize(-1,-1),cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,30,0.01));

        //Lets draw the detected virtual-checkerboard corners onto the differential image.
        cvDrawChessboardCorners(cap,cvSize(virtual_checkerboard_X,virtual_checkerboard_Y),detected_virtual_checkerboard_corners,virtual_checkerboard_corner_count,found);

//lets show it on a saparate window.
        cvNamedWindow("detected_checkerboard_corners",0);
        cvShowImage("detected_checkerboard_corners",cap);
        sprintf(filename,"Projector_calibration/Captured_images/Difference_images/View_%d.jpg",successes);
        cvSaveImage(filename,cap);


//Detected corners of the projectors need to be stored temporarily to be used later for computing corresponding object points after camera is extrinsically calibrated.


        for(int k=0; k<total_virtual_checkerboard_corners; k++)
        {
            CV_MAT_ELEM(*proj_detected_corners,double,k,0)=(double)detected_virtual_checkerboard_corners[k].x;
            CV_MAT_ELEM(*proj_detected_corners,double,k,1)=(double)detected_virtual_checkerboard_corners[k].y;
        }



        compute_proj_obj_points(cwhomo,proj_detected_corners,temp_proj_obj_points);//This function will compute the object points for the projector to be used for projector calibration.
        //cvPerspectiveTransform(proj_detected_corners,temp_proj_obj_points,cwhomo);
        //Now let me copy the computed projector object points to the proj_obj_points array to be used for projector calibration.
        int n=0;


        /*
                for(int h=successes*total_virtual_checkerboard_corners; h<total_virtual_checkerboard_corners*(successes+1); h++)
                {

                    fprintf(fd_projector_object_points,"%lf\t",CV_MAT_ELEM(*proj_obj_points,double,h,0)=CV_MAT_ELEM(*temp_proj_obj_points,double,n,0));
                    fprintf(fd_projector_object_points,"%lf\t",CV_MAT_ELEM(*proj_obj_points,double,h,1)=CV_MAT_ELEM(*temp_proj_obj_points,double,n,1));
                    fprintf(fd_projector_object_points,"%lf\n",CV_MAT_ELEM(*proj_obj_points,double,h,2)=CV_MAT_ELEM(*temp_proj_obj_points,double,n,2));
                    n++;

                }
        */

//Now 'proj_obj_points' contains the world coordinates of the projected virtual checkerboard.
//Lets compute the virtual checkerboard image points as well!!

        //printf("Printing corresponding projector image points:\n");
        n=0;
        for(int l=successes*total_virtual_checkerboard_corners; l<total_virtual_checkerboard_corners*(successes+1); l++)
        {
            CV_MAT_ELEM(*proj_image_points,double,l,0)=(double)virtual_checkerboard_square_pixels*(double)((l-successes*total_virtual_checkerboard_corners)%virtual_checkerboard_X)+(double)virtual_checkerboard_square_pixels+proj_border_X_pixels;
            CV_MAT_ELEM(*proj_image_points,double,l,1)=(double)virtual_checkerboard_square_pixels*(double)(floorf((float)(l-successes*total_virtual_checkerboard_corners)/(float)virtual_checkerboard_X))+(double)virtual_checkerboard_square_pixels+proj_border_Y_pixels;
            CV_MAT_ELEM(*temp_proj_image_points,double,n,0)=CV_MAT_ELEM(*proj_image_points,double,l,0);
            CV_MAT_ELEM(*temp_proj_image_points,double,n,1)=CV_MAT_ELEM(*proj_image_points,double,l,1);
            n++;
        }


        CV_MAT_ELEM(*temp_point_count,int,successes,0)=virtual_checkerboard_corner_count;

        successes++;

        ///For VPCLib tool(need to have both projector image & object points)
        fs_match_cam_proj<<"CAM_PTS"<<proj_detected_corners;

        fs_match_cam_proj<<"PROJ_PTS"<<temp_proj_image_points;


        fs_match_cam_proj.FileStorage::release();


    }


//    fclose(fd_projector_object_points);//object points written in a common file(for all views)

//Data gathering is done!!
//Lets do projector calibration finally!!
    CvMat*point_count=cvCreateMat(successes,1,CV_32SC1);

    for(int s=0; s<successes; s++)
        CV_MAT_ELEM(*point_count,int,s,0)=CV_MAT_ELEM(*temp_point_count,int,s,0);

    int calib_flags=0;


    calib_flags |= CV_CALIB_ZERO_TANGENT_DIST;
    calib_flags |= CV_CALIB_FIX_K3;
    //calib_flags|=CV_CALIB_FIX_ASPECT_RATIO;


    double reprojection_error=0.0;


    printf("\nCalibrating projector...\n");
//Lets calibrate the projector finally!!
    reprojection_error=cvCalibrateCamera2(proj_obj_points,proj_image_points,point_count,cvSize(Projector_imagewidth,Projector_imageheight),proj_intrinsic_mat,proj_dist_vect,proj_rot_vect,proj_trans_vect,calib_flags);
//Projector calibration done!!
    printf("Projector calibrated successfully!!");

//lets save the projector calibration results.
    sprintf(filename,"Projector_calibration/Matrices/proj_intrinsic_mat.xml");
    cvSave(filename,proj_intrinsic_mat);

    sprintf(filename,"Projector_calibration/Matrices/proj_dist_vect.xml");
    cvSave(filename,proj_dist_vect);

    //Lets save the reprojection error
    FILE*fd_rep_error=fopen("Projector_calibration/Matrices/reprojection_error.txt","w");
    fprintf(fd_rep_error,"%lf",reprojection_error);
    fclose(fd_rep_error);


    CvMat*mat_3_3=cvCreateMat(1,3,CV_64FC1);

    for(int y=0; y<successes; y++)
    {
        //Copying rotation vector.
        for(int i=0; i<3; i++)
            CV_MAT_ELEM(*mat_3_3,double,0,i)=CV_MAT_ELEM(*proj_rot_vect,double,y,i);

        sprintf(filename,"Projector_calibration/Matrices/world_to_proj_rot_vect_%d.xml",y);
        cvSave(filename,mat_3_3);

        //Copying translation vector.
        for(int i=0; i<3; i++)
            CV_MAT_ELEM(*mat_3_3,double,0,i)=CV_MAT_ELEM(*proj_trans_vect,double,y,i);

        sprintf(filename,"Projector_calibration/Matrices/world_to_proj_trans_vect_%d.xml",y);
        cvSave(filename,mat_3_3);
    }

    /*
    //De-allocate storage...
        cvReleaseFileStorage(&fs_cam_intrinsic_mat);
        cvReleaseFileStorage(&fs_cam_distortion_vect);
        cvReleaseMat(&proj_rot_vect);
        cvReleaseMat(&proj_trans_vect);


        cvReleaseImage(&gray_temp_camera_capture);
        cvReleaseImage(&temp_projector_capture);
        cvReleaseImage(&gray_temp_projector_capture);
        cvReleaseImage(&differential_image);

        cvReleaseMat(&cam_obj_points);
        cvReleaseMat(&cam_image_points);
        delete[] detected_physical_checkerboard_corners;

        cvReleaseMat(&proj_obj_points);
        cvReleaseMat(&proj_image_points);
        cvReleaseMat(&proj_detected_corners);
        delete[] detected_virtual_checkerboard_corners;
        cvReleaseMat(&cwhomo);
        cvReleaseMat(&temp_point_count);
        cvReleaseImage(&blank);

        //cvReleaseFileStorage(&&fs_H);

        cvReleaseImage(&temp_virtual_checkerboard);

        cvReleaseMat(&temp_proj_image_points);

        //cvReleaseFileStorage(&&fs_match_cam_proj);
        cvReleaseMat(&temp_proj_obj_points);
        cvReleaseMat(&point_count);
        cvReleaseMat(&mat_3_3);

    */


//Done!!

    return;

}













//Folloing function will do the extrinsic calibration of the system.
//ASSUMPTIONS:-Camera & projector are already intrinsically calibrated.

CvRect ROI;
bool ROI_selected=false;

void get_ROI(int event,int x,int y,int flags,void* param)
{
    switch(event)
    {
    case CV_EVENT_LBUTTONDOWN:
        ROI.x=x;
        ROI.y=y;
        break;

    case CV_EVENT_LBUTTONUP:
        ROI.width=x-ROI.x;
        ROI.height=y-ROI.y;
        ROI_selected=true;
    }
    return;
}



void extrinsic_calibration()
{
    //lets read the camera & projector parameters for use in extrinsic calibration.
//camera.
    CvFileStorage*fs_cam_intrinsic_mat=cvOpenFileStorage("/home/pranav/Desktop/M_tech_project_console/Camera_calibration/Matrices/cam_intrinsic_mat.xml",0,CV_STORAGE_READ);
    CvFileStorage*fs_cam_distortion_vect=cvOpenFileStorage("/home/pranav/Desktop/M_tech_project_console/Camera_calibration/Matrices/cam_distortion_vect.xml",0,CV_STORAGE_READ);
    cam_intrinsic_mat=(CvMat*)cvReadByName(fs_cam_intrinsic_mat,0,"cam_intrinsic_mat");
    cam_dist_vect=(CvMat*)cvReadByName(fs_cam_distortion_vect,0,"cam_distortion_vect");

    //First i have to capture one pose of the calibration board that will be used as 'REFERENCE WORLD-COORDINATE SYSTEM'.

    IplImage*capture;
    IplImage*gray_camera_capture;
    IplImage*gray_projector_capture;
    IplImage*differential_image;
    CvPoint2D32f*detected_physical_checkerboard_corners=new CvPoint2D32f[total_physical_checkerboard_corners];
    int physical_checkerboard_corner_count=0;

    CvMat*cam_obj_points=cvCreateMat(4,3,CV_64FC1);
    CvMat*cam_image_points=cvCreateMat(4,3,CV_64FC1);

    cam_world_rot_vect=cvCreateMat(3,1,CV_64FC1);//Rotation vector.
    cam_world_trans_vect=cvCreateMat(3,1,CV_64FC1);//Translation vector.(Note:Data type for these will be 'double'!!)



    CvPoint2D32f*detected_virtual_checkerboard_corners=new CvPoint2D32f[total_virtual_checkerboard_corners];
    int virtual_checkerboard_corner_count=0;



    CvMat*proj_image_points=cvCreateMat(total_virtual_checkerboard_corners,2,CV_64FC1);//It contains the corresponding image coordinates for the virtual checkerboard.


    proj_world_rot_vect=cvCreateMat(3,1,CV_64FC1);
    proj_world_trans_vect=cvCreateMat(3,1,CV_64FC1);



    IplImage*blank=cvCreateImage(cvSize(Projector_imagewidth,Projector_imageheight),IPL_DEPTH_8U,1);
    cvSet(blank,cvScalar(0));//blank out the image.

    cvShowImage("Projector_pattern",blank);
    CvMat*cwhomo=cvCreateMat(3,3,CV_64FC1);//The camera-world homography matrix.

    detect_wall_points(cam_image_points);
    define_wall_object_points(cam_obj_points);
    cvFindHomography(cam_image_points,cam_obj_points,cwhomo,CV_RANSAC);

    cvFindExtrinsicCameraParams2(cam_obj_points,cam_image_points,cam_intrinsic_mat,cam_dist_vect,cam_world_rot_vect,cam_world_trans_vect);
    cvSave("/home/pranav/Desktop/M_tech_project_console/Triangulation/Camera_extrinsic_parametrs/world_to_cam_rot_vect.xml",cam_world_rot_vect);
    cvSave("/home/pranav/Desktop/M_tech_project_console/Triangulation/Camera_extrinsic_parametrs/world_to_cam_trans_vect.xml",cam_world_trans_vect);
    cvSave("/home/pranav/Desktop/M_tech_project_console/Triangulation/Camera_extrinsic_parametrs/cwhomo.xml",cwhomo);


    /************************************TEST STARTS!!************************************/
    printf("\nCalculating deviation of projected 3D points from detected image points in distorted image...");
    CvMat*cam_sample_obj_points=cvCreateMat(4,4,CV_64FC1);
    CvMat*cam_rot_mat=cvCreateMat(3,3,CV_64FC1);
    CvMat*projected_cam_image_points=cvCreateMat(3,4,CV_64FC1);
    cvRodrigues2(cam_world_rot_vect,cam_rot_mat);

//Lets prepare the [R|T] matrix
    CvMat*R_T=cvCreateMat(3,4,CV_64FC1);

    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            CV_MAT_ELEM(*R_T,double,i,j)=CV_MAT_ELEM(*cam_rot_mat,double,i,j);

    for(int d=0; d<3; d++)
    {
        CV_MAT_ELEM(*R_T,double,d,3)=CV_MAT_ELEM(*cam_world_trans_vect,double,d,0);

    }

//[r11,r12,r13 t1][X]
//[r21,r22,r23,t2][Y]
//[r31,r32,r33,t3][Z]
//                [1]

//Lets initialize the sample object points
    for(int i=0; i<3; i++)
        for(int j=0; j<4; j++)
            CV_MAT_ELEM(*cam_sample_obj_points,double,i,j)=CV_MAT_ELEM(*cam_obj_points,double,j,i);

    for(int p=0; p<4; p++)
        CV_MAT_ELEM(*cam_sample_obj_points,double,3,p)=1.0;



//Transform to camera 3D coordinate system
    CvMat*cam_coordinates_system_obj_points=cvCreateMat(3,4,CV_64FC1);
    cvMatMul(R_T,cam_sample_obj_points,cam_coordinates_system_obj_points);

//Lets homogenize the coordinates
    for(int i=0; i<4; i++)
        for(int p=0; p<3; p++)
            CV_MAT_ELEM(*projected_cam_image_points,double,p,i)=CV_MAT_ELEM(*cam_coordinates_system_obj_points,double,p,i)/CV_MAT_ELEM(*cam_coordinates_system_obj_points,double,2,i);

//Distort image points.
    CvMat*dist_projected_cam_image_points=cvCreateMat(3,4,CV_64FC1);

    double radial_factor=0.0;
    double r=0.0;
    double k1=CV_MAT_ELEM(*cam_dist_vect,double,0,0);
    double k2=CV_MAT_ELEM(*cam_dist_vect,double,1,0);
    double p1=CV_MAT_ELEM(*cam_dist_vect,double,2,0);
    double p2=CV_MAT_ELEM(*cam_dist_vect,double,3,0);
    double k3=CV_MAT_ELEM(*cam_dist_vect,double,4,0);
    /*double x_tangential_factor=;
    double y_tangential_factor=;
    */
    for(int i=0; i<4; i++)
    {
        r=sqrt(pow(CV_MAT_ELEM(*projected_cam_image_points,double,0,i),2)+pow(CV_MAT_ELEM(*projected_cam_image_points,double,1,i),2));
        radial_factor=1.0+k1*pow(r,2)+k2*pow(r,4)+k3*pow(r,6);
//distorted X
        CV_MAT_ELEM(*dist_projected_cam_image_points,double,0,i)=CV_MAT_ELEM(*projected_cam_image_points,double,0,i)*radial_factor+2.0*p1*CV_MAT_ELEM(*projected_cam_image_points,double,0,i)*CV_MAT_ELEM(*projected_cam_image_points,double,1,i)+p2*(pow(r,2)+2.0*pow(CV_MAT_ELEM(*projected_cam_image_points,double,0,i),2));

//distorted Y
        CV_MAT_ELEM(*dist_projected_cam_image_points,double,1,i)=CV_MAT_ELEM(*projected_cam_image_points,double,1,i)*radial_factor+2.0*p2*CV_MAT_ELEM(*projected_cam_image_points,double,0,i)*CV_MAT_ELEM(*projected_cam_image_points,double,1,i)+p1*(pow(r,2)+2.0*pow(CV_MAT_ELEM(*projected_cam_image_points,double,1,i),2));

        CV_MAT_ELEM(*dist_projected_cam_image_points,double,2,i)=1.0;

    }

//Project distorted 3D points to camera ideal plane perspectively
    cvMatMul(cam_intrinsic_mat,dist_projected_cam_image_points,dist_projected_cam_image_points);


//Compare coordinates.
    double projection_error[4];

    for(int i=0; i<4; i++)
    {
        projection_error[i]=sqrt(pow((CV_MAT_ELEM(*cam_image_points,double,i,0)-CV_MAT_ELEM(*dist_projected_cam_image_points,double,0,i)),2)+pow((CV_MAT_ELEM(*cam_image_points,double,i,1)-CV_MAT_ELEM(*dist_projected_cam_image_points,double,1,i)),2));
        printf("\nDeviation from pt-%d:%lf",i,projection_error[i]);
    }

    /*****************************************************TEST DONE!!***********************/
/////////////////EXTRINSICS FOR PROJECTOR......




    printf("\nStarting projector extrinsic calibration...\n ");


    cvNamedWindow("Preview_projector_extrinsic_calibration",0);


    //loading projector & camera parameters...


    CvFileStorage*fs_proj_intrinsic_mat=cvOpenFileStorage("Projector_calibration/Matrices/proj_intrinsic_mat.xml",0,CV_STORAGE_READ);
    CvFileStorage*fs_proj_distortion_vect=cvOpenFileStorage("Projector_calibration/Matrices/proj_distortion_vect.xml",0,CV_STORAGE_READ);
    proj_intrinsic_mat=(CvMat*)cvReadByName(fs_proj_intrinsic_mat,0,"proj_intrinsic_mat");
    proj_dist_vect=(CvMat*)cvReadByName(fs_proj_distortion_vect,0,"proj_distortion_vect");


    IplImage*undist_virtual_checkerboard=cvCloneImage(virtual_checkerboard_image);

    cvUndistort2(virtual_checkerboard_image,undist_virtual_checkerboard,proj_intrinsic_mat,proj_dist_vect);
    cvShowImage("Projector_pattern",undist_virtual_checkerboard);

    IplImage*temp_virtual_checkerboard=cvCreateImage(cvSize(Projector_imagewidth,Projector_imageheight),8,1);


    /*
        while(cvWaitKey(30)!=27)
        {
            cvCopyImage(undist_virtual_checkerboard,temp_virtual_checkerboard);
            /*   memset(&ctrl,0,sizeof(ctrl));
               ctrl.id=V4L2_CID_EXPOSURE_ABSOLUTE;
               ctrl.value=cam_exp;

               if((ioctl(fd_webcam,VIDIOC_S_CTRL,&ctrl)==-1) && (errno!=ERANGE))
                   perror("");

            */

    //capture=camera_preview_1();
    /*       preview_and_capture();
           cvScale(temp_virtual_checkerboard,temp_virtual_checkerboard,2.*(proj_gain/100.),0);
           cvShowImage("Projector_pattern",temp_virtual_checkerboard);
           cvShowImage("Preview_projector_extrinsic_calibration",cap);



       }
    */
    //capture=camera_capture("Captured_patterns/junk.bmp");
    printf("\nPlease press c when virtual checkerboard projected on screen...\n");
    fflush(stdout);
    preview_and_capture();
    gray_projector_capture=cvCreateImage(cvGetSize(cap),IPL_DEPTH_8U,1);
    cvCvtColor(cap,gray_projector_capture,CV_RGB2GRAY);//corresponding gray image.
    IplImage*undist_gray_projector_capture=cvCreateImage(cvGetSize(cap),IPL_DEPTH_8U,1);
    cvUndistort2(gray_projector_capture,undist_gray_projector_capture,cam_intrinsic_mat,cam_dist_vect);


    cvSaveImage("extrinsic_image.jpg",cap);

//Now image of the virtual checkerboard has been captured.
//Lets detect corners in it.
    differential_image=cvCreateImage(cvSize(Camera_imagewidth,Camera_imageheight),IPL_DEPTH_8U,1);

    int found=0;

    //subtract_images(gray_projector_capture,gray_camera_capture,differential_image);//This function will return a differential image which contains only the contribution from virtual checkerboard

    //To avoid collision with marked corners as checkerboard corners, adding selecting ROI feature:


    cvNamedWindow("Virtual checkerboard: Select ROI",0);


    cvSetMouseCallback("Virtual checkerboard: Select ROI",get_ROI,NULL);

    while(ROI_selected==false)
    {
        cvShowImage("Virtual checkerboard: Select ROI",undist_gray_projector_capture);
        cvWaitKey(30);

    }

    cvSetImageROI(undist_gray_projector_capture,ROI);

    found=cvFindChessboardCorners(undist_gray_projector_capture,cvSize(virtual_checkerboard_X,virtual_checkerboard_Y),detected_virtual_checkerboard_corners,&virtual_checkerboard_corner_count);



    if(found==0)
    {
        printf("Virtual checkerboard not found,exiting...\n");
        cvSaveImage("virtual_checkerboard_not_found.jpg",undist_gray_projector_capture);
        exit(0);
    }
//Now corners are detected.
//Lets do the subpixel refinement!!
    cvFindCornerSubPix(undist_gray_projector_capture,detected_virtual_checkerboard_corners,virtual_checkerboard_corner_count,cvSize(11,11),cvSize(-1,-1),cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,30,0.01));

    cvResetImageROI(undist_gray_projector_capture);

    for(int i=0; i<virtual_checkerboard_corner_count; i++)
    {
        detected_virtual_checkerboard_corners[i].x+=ROI.x;
        detected_virtual_checkerboard_corners[i].y+=ROI.y;
    }

    cvDrawChessboardCorners(cap,cvSize(virtual_checkerboard_X,virtual_checkerboard_Y),detected_virtual_checkerboard_corners,virtual_checkerboard_corner_count,found);


    cvShowImage("Preview_projector_extrinsic_calibration",cap);

    cvShowImage("Camera_view",differential_image);

//Let me convert the input 'detected_virtual_checkerboard_corners' to cvmat format to be used in compute_proj_obj_points().
    CvMat*detected_virtual_checkerboard_corners_mat=cvCreateMat(total_virtual_checkerboard_corners,2,CV_64FC1);
    for(int u=0; u<total_virtual_checkerboard_corners; u++)
    {
        CV_MAT_ELEM(*detected_virtual_checkerboard_corners_mat,double,u,0)=(double)detected_virtual_checkerboard_corners[u].x;
        CV_MAT_ELEM(*detected_virtual_checkerboard_corners_mat,double,u,1)=(double)detected_virtual_checkerboard_corners[u].y;
    }


    proj_world_coordinates=cvCreateMat(total_virtual_checkerboard_corners,3,CV_64FC1);



    compute_proj_obj_points(cwhomo,detected_virtual_checkerboard_corners_mat,proj_world_coordinates);

//Lets compute the corresponding image coordinates of virtual checkerboard (to be used in extrinsic calibration of projector)

    for(int d=0; d<total_virtual_checkerboard_corners; d++)
    {
        CV_MAT_ELEM(*proj_image_points,double,d,0)=(double)virtual_checkerboard_square_pixels*(double)(d%virtual_checkerboard_X)+(double)virtual_checkerboard_square_pixels+(double)proj_border_X_pixels;
        CV_MAT_ELEM(*proj_image_points,double,d,1)=(double)virtual_checkerboard_square_pixels*(double)floorf((float)d/(float)virtual_checkerboard_X)+(double)virtual_checkerboard_square_pixels+(double)proj_border_Y_pixels;
        CV_MAT_ELEM(*proj_world_coordinates,double,d,2)=0.0;

    }




//Lets finally calibrate the projector extrinsically!!
    printf("\nCalibrating projector extrinsically...");
    cvFindExtrinsicCameraParams2(proj_world_coordinates,proj_image_points,proj_intrinsic_mat,proj_dist_vect,proj_world_rot_vect,proj_world_trans_vect);
//Lets save the parameters(R & T vectors).
    cvSave("Triangulation/Projector_extrinsic_parametrs/world_to_proj_rot_vect.xml",proj_world_rot_vect);
    cvSave("Triangulation/Projector_extrinsic_parametrs/world_to_proj_trans_vect.xml",proj_world_trans_vect);


//Lets save thses world-coordinates for projector(will be used by plotting app.)
    CvMat*trans_proj_coordinates=cvCreateMat(3,total_virtual_checkerboard_corners,CV_64FC1);
    cvTranspose(proj_world_coordinates,trans_proj_coordinates);
    //cvSave("../3D_plot/virtual_checkerboard_object_points.xml",trans_proj_coordinates);


/// CODE FOR COMPUTING RELATIVE EXTRINSIC GEOMETRY:

    //Lets first calculate the rotation matrix between camera & projector.
    //Req=Rc*transpose(Rp)
    cam_world_rot_mat=cvCreateMat(3,3,CV_64FC1);
    //cam_world_trans_vect=cvCreateMat(3,1,CV_64FC1);

    proj_world_rot_mat=cvCreateMat(3,3,CV_64FC1);
    //proj_world_trans_vect=cvCreateMat(3,1,CV_64FC1);


    cvRodrigues2(cam_world_rot_vect,cam_world_rot_mat);
    cvRodrigues2(proj_world_rot_vect,proj_world_rot_mat);

    proj_cam_rot_mat=cvCreateMat(3,3,CV_64FC1);

    cvTranspose(proj_world_rot_mat,proj_world_rot_mat);
    cvMatMul(cam_world_rot_mat,proj_world_rot_mat,proj_cam_rot_mat);


    //Lets compute the translation vector between camera & projector.
    //Teq=Tc-Rc*Transpose(Rp)*Tp
    //Lets allocate space for translation vectors.
    proj_cam_trans_vect=cvCreateMat(3,1,CV_64FC1);

    cvMatMul(proj_cam_rot_mat,proj_world_trans_vect,proj_cam_trans_vect);
    cvSub(cam_world_trans_vect,proj_cam_trans_vect,proj_cam_trans_vect);//projector-to-camera translation vector.

    //NOTE:-After subtraction last element will be '0' so we will make it '1' explicitly.(is it correct???)
    //ASSUMPTION:-Subtraction in this case is valid,since we are only interested in upper 3X1 part.


    printf("Relative translation of projector with respect to camera COP:\n");
    printf("X=%lf\tY=%lf\tZ=%lf\n",CV_MAT_ELEM(*proj_cam_trans_vect,double,0,0),CV_MAT_ELEM(*proj_cam_trans_vect,double,1,0),CV_MAT_ELEM(*proj_cam_trans_vect,double,2,0));


    //lets save these parameters...
    cvSave("Triangulation/Relative_geometry/proj_cam_rot_mat.xml",proj_cam_rot_mat);
    cvSave("Triangulation/Relative_geometry/proj_cam_trans_vect.xml",proj_cam_trans_vect);

//Done!!
    return;
}




//This routine will be used to load the metrices in the case when no calibration is to be done.
void load_matrices()
{
    //Projector

    CvFileStorage*fs_proj_rot_vect=cvOpenFileStorage("Triangulation/Projector_extrinsic_parametrs/world_to_proj_rot_vect.xml",0,CV_STORAGE_READ);
    proj_world_rot_vect=(CvMat*)cvReadByName(fs_proj_rot_vect,0,"world_to_proj_rot_vect");


    CvFileStorage*fs_proj_trans_vect=cvOpenFileStorage("Triangulation/Projector_extrinsic_parametrs/world_to_proj_trans_vect.xml",0,CV_STORAGE_READ);
    proj_world_trans_vect=(CvMat*)cvReadByName(fs_proj_trans_vect,0,"world_to_proj_trans_vect");

    CvFileStorage*fs_proj_intrinsic=cvOpenFileStorage("Projector_calibration/Matrices/proj_intrinsic_mat.xml",0,CV_STORAGE_READ);
    proj_intrinsic_mat=(CvMat*)cvReadByName(fs_proj_intrinsic,0,"proj_intrinsic_mat");

    CvFileStorage*fs_proj_dist_vect=cvOpenFileStorage("Projector_calibration/Matrices/proj_distortion_vect.xml",0,CV_STORAGE_READ);
    proj_dist_vect=(CvMat*)cvReadByName(fs_proj_dist_vect,0,"proj_distortion_vect");

//Camera.
    CvFileStorage*fs_cam_trans_vect=cvOpenFileStorage("Triangulation/Camera_extrinsic_parametrs/world_to_cam_trans_vect.xml",0,CV_STORAGE_READ);
    cam_world_trans_vect=(CvMat*)cvReadByName(fs_cam_trans_vect,0,"world_to_cam_trans_vect");

    CvFileStorage*fs_cam_rot_vect=cvOpenFileStorage("Triangulation/Camera_extrinsic_parametrs/world_to_cam_rot_vect.xml",0,CV_STORAGE_READ);
    cam_world_rot_vect=(CvMat*)cvReadByName(fs_cam_rot_vect,0,"world_to_cam_rot_vect");

    CvFileStorage*fs_cam_intrisic_mat=cvOpenFileStorage("Camera_calibration/Matrices/cam_intrinsic_mat.xml",0,CV_STORAGE_READ);
    cam_intrinsic_mat=(CvMat*)cvReadByName(fs_cam_intrisic_mat,0,"cam_intrinsic_mat");

    CvFileStorage*fs_cam_dist_vec=cvOpenFileStorage("Camera_calibration/Matrices/cam_distortion_vect.xml",0,CV_STORAGE_READ);
    cam_dist_vect=(CvMat*)cvReadByName(fs_cam_dist_vec,0,"cam_distortion_vect");

    /*
    //Well let me undistort a sample image to check for distortion-parameters correctness.
        IplImage*test_image=cvCreateImage(cvSize(Projector_imagewidth,Projector_imageheight),IPL_DEPTH_8U,1);
        cvUndistort2(virtual_checkerboard_image,test_image,proj_intrinsic_mat,proj_dist_vect);
        cvSaveImage("Undistorted_projector_image.jpg",test_image);



    /// Lets write a calibration-tester for both Camera & projector:
    ///1.Take 3D input from user.
    ///2.Map it to camera/projector image.
        CvMat*sample_point=cvCreateMat(3,1,CV_64FC1);
        CvMat*transformed_sample_point=cvCreateMat(3,1,CV_64FC1);


        printf("\nPlease enter 3D coordinate of the point to be tested:");
        printf("\nX=");
        scanf("%lf",&CV_MAT_ELEM(*sample_point,double,0,0));
        printf("Y=");
        scanf("%lf",&CV_MAT_ELEM(*sample_point,double,1,0));
        printf("Z=");
        scanf("%lf",&CV_MAT_ELEM(*sample_point,double,2,0));

    /// CAMERA:
        cam_world_rot_mat=cvCreateMat(3,3,CV_64FC1);



        printf("\nMapping the point to camera image space...");
        cvRodrigues2(cam_world_rot_vect,cam_world_rot_mat);
        cvMatMul(cam_world_rot_mat,sample_point,transformed_sample_point);


        for(int i=0; i<3; i++)
            CV_MAT_ELEM(*transformed_sample_point,double,i,0)+=CV_MAT_ELEM(*cam_world_trans_vect,double,i,0);



        cvMatMul(cam_intrinsic_mat,transformed_sample_point,transformed_sample_point);

    //Lets do divide by Z to make it a 2D point.

        for(int i=0; i<3; i++)
            CV_MAT_ELEM(*transformed_sample_point,double,i,0)/=CV_MAT_ELEM(*transformed_sample_point,double,2,0);


    //Lets load an undistorted image on which the circle at computed position will be displayed.

        CvPoint test_point;
        test_point.x=(int)CV_MAT_ELEM(*transformed_sample_point,double,0,0);
        test_point.y=(int)CV_MAT_ELEM(*transformed_sample_point,double,1,0);

        IplImage*cam_test_image=cvLoadImage("Triangulation/extrinsic.jpg");
        IplImage*cam_undist_test_image=cvCloneImage(cam_test_image);

        cvUndistort2(cam_test_image,cam_undist_test_image,cam_intrinsic_mat,cam_dist_vect);
        cvCircle(cam_undist_test_image,test_point,5,cvScalar(0,255,0));

        cvNamedWindow("Camera_window",0);
        cvShowImage("Camera_window",cam_undist_test_image);

        cvWaitKey(0);

    ///PROJECTOR:
        proj_world_rot_mat=cvCreateMat(3,3,CV_64FC1);

        printf("\nMapping the point to projector image space...");
        cvRodrigues2(proj_world_rot_vect,proj_world_rot_mat);
        cvMatMul(proj_world_rot_mat,sample_point,transformed_sample_point);

        for(int i=0; i<3; i++)
            CV_MAT_ELEM(*transformed_sample_point,double,i,0)+=CV_MAT_ELEM(*proj_world_trans_vect,double,i,0);

        cvMatMul(proj_intrinsic_mat,transformed_sample_point,transformed_sample_point);

        for(int i=0; i<3; i++)
            CV_MAT_ELEM(*transformed_sample_point,double,i,0)/=CV_MAT_ELEM(*transformed_sample_point,double,2,0);

        IplImage*proj_test_image=cvLoadImage("Projector_calibration/Virtual_calibration_rig/Checkerboard.jpg");
        IplImage*proj_undist_test_image=cvCloneImage(proj_test_image);
        cvUndistort2(proj_test_image,proj_undist_test_image,proj_intrinsic_mat,proj_dist_vect);

        test_point.x=CV_MAT_ELEM(*transformed_sample_point,double,0,0);
        test_point.y=CV_MAT_ELEM(*transformed_sample_point,double,1,0);

        cvCircle(proj_undist_test_image,test_point,5,cvScalar(0,255,0));

        cvNamedWindow("Projector_window",0);
        cvMoveWindow("Projector_window",Monitor_width_pixels,0);//That is,projector is assumed to be present on right side of user panel so it will be shifted by camera width & height(currently it is in the same window as user panel).
        cvSetWindowProperty("Projector_window",CV_WND_PROP_FULLSCREEN,CV_WINDOW_FULLSCREEN);

        cvShowImage("Projector_window",proj_undist_test_image);


        cvWaitKey(0);

        cvDestroyWindow("Camera_window");
        cvDestroyWindow("Projector_window");

    */


    //De-allocate...

    cvReleaseFileStorage(&fs_proj_rot_vect);

    cvReleaseFileStorage(&fs_proj_trans_vect);

    cvReleaseFileStorage(&fs_proj_intrinsic);


    cvReleaseFileStorage(&fs_proj_dist_vect);



    cvReleaseFileStorage(&fs_cam_trans_vect);



    cvReleaseFileStorage(&fs_cam_rot_vect);



    cvReleaseFileStorage(&fs_cam_intrisic_mat);


    cvReleaseFileStorage(&fs_cam_dist_vec);


    return;
}











/// External interface(SIGGRAPH-09 course method)
void system_calibration()
{
    printf("\nCalibrate camera?\n");
    fflush(stdout);
    if(cvWaitKey(0)=='y')
        calibrate_camera();
    printf("\nCalibrate projector??\n");
    fflush(stdout);
    if(cvWaitKey(0)=='y')
    {

        printf("\nStarting projector calibration...\n");
        calibrate_projector();
    }


    printf("\nExtrinsic calibration???\n");
    if(cvWaitKey(0)=='y')
    {
        extrinsic_calibration();

    }
    else
        load_matrices();
    //Done,parameter matrices are stored.!!


//Lets do deallocation...
    //cvReleaseImage(&virtual_checkerboard_image);
    //cvReleaseImage(&preview_image);
    //cvReleaseImage(&gray_preview_image);
    //delete[] detected_points;


    return;
}



//APPROACH-2:
//************************************Implementing S.Zhang's approach for projector calibration*********************************************///

//Approach-2 specific variables:
/*
CvMat*proj_checkerboard_image_points;
CvMat*proj_checkerboard_object_points;


CvMat*camera_image_points;
CvMat*proj_corresponding_image_points;

CvMat*c_p_homography;

//Following function will get camera to projector map & will compute c-p Homography
//Based on compute_coorspondance.cpp
void compute_c_p_homography()
{

    camera_image_points=cvCreateMat(total_camera_pixels,3,CV_64FC1);
    proj_corresponding_image_points=cvCreateMat(total_camera_pixels,3,CV_64FC1);

    c_p_homography=cvCreateMat(3,3,CV_64FC1);


    for(int i=0; i<Camera_imageheight; i++)
        for(int j=0; j<Camera_imagewidth; j++)
        {
            CV_MAT_ELEM(*camera_image_points,double,i*Camera_imageheight+j,0)=j;
            CV_MAT_ELEM(*camera_image_points,double,i*Camera_imagewidth+j,1)=i;
            CV_MAT_ELEM(*camera_image_points,double,i*Camera_imagewidth+j,2)=1.0;
            CV_MAT_ELEM(*proj_corresponding_image_points,double,i*Camera_imagewidth+j,0)=(double)c_p_map[i*Camera_imagewidth+j][0];
            CV_MAT_ELEM(*proj_corresponding_image_points,double,i*Camera_imagewidth+j,0)=(double)c_p_map[i*Camera_imagewidth+j][1];
            CV_MAT_ELEM(*proj_corresponding_image_points,double,i*Camera_imagewidth+j,2)=1.0;
        }


    cvFindHomography(camera_image_points,proj_corresponding_image_points,c_p_homography,CV_RANSAC);


    return;
}



///Function specific variables:
IplImage  **camera_calib_images;
IplImage **proj_calib_images;
IplImage*gray_capture;

CvMat*camera_image_points_calib;
CvMat*proj_calib_image_points;
CvMat*proj_calib_object_points;

CvPoint2D32f*detected_corners=new CvPoint2D32f[total_physical_checkerboard_corners];
int detected_corner_count;

CvMat*global_corner_count;//will store corner count for all views.


//Physical checkerboard captured.
void capture_checkerboard_image(int view_number)
{
    int found=0;
    bool alignment_done=false;

    while(alignment_done==false)
    {
        cap=cvQueryFrame(camera);
        cvShowImage("Preview alignment",cap);
        cvWaitKey(30);
        printf("Is it OK??");
        fflush(stdout);

        if(cvWaitKey(0)=='y')
        alignment_done=true;

    }
//cap=cvLoadImage(filename);
    gray_capture=cvCreateImage(cvGetSize(cap),IPL_DEPTH_8U,1);

    cvCvtColor(cap,gray_capture,CV_RGB2GRAY);

    found=cvFindChessboardCorners(gray_capture,cvSize(physical_checkerboard_X,physical_checkerboard_Y),detected_corners,&detected_corner_count,CV_CALIB_CB_ADAPTIVE_THRESH);


    cvFindCornerSubPix(gray_capture,detected_corners,detected_corner_count,cvSize(11,11),cvSize(-1,-1),cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,30,0.01));

    if(found!=0)
    {
        printf("\nCheckerboard found!!");
        camera_calib_images[view_number]=cvCreateImage(cvGetSize(cap),cap->depth,cap->nChannels);
        proj_calib_images[view_number]=cvCreateImage(cvSize(Projector_imagewidth,Projector_imageheight),IPL_DEPTH_8U,3);
        cvCopy(cap,camera_calib_images[view_number]);
    }

    cvDrawChessboardCorners(cap,cvSize(physical_checkerboard_X,physical_checkerboard_Y),detected_corners,detected_corner_count,found);
    cvShowImage("Preview detected corners",cap);



    //Lets copy this result to camera image points array.
    for(int t=0; t<total_physical_checkerboard_corners; t++)
    {
        CV_MAT_ELEM(*camera_image_points_calib,double,0,t)=detected_corners[t].x;
        CV_MAT_ELEM(*camera_image_points_calib,double,1,t)=detected_corners[t].y;
        CV_MAT_ELEM(*camera_image_points_calib,double,2,t)=1.0;
    }


    CV_MAT_ELEM(*global_corner_count,int,view_number,0)=detected_corner_count;


    return;
}




IplImage*temp_cam_calib_image;
IplImage*temp_proj_calib_image;

//Generate projector image.
void map_image_camera_to_projector(int view_number)
{

    //lets copy the camera calibration image to a temporary variable.

    temp_cam_calib_image=camera_calib_images[view_number];
    temp_proj_calib_image=proj_calib_images[view_number];

    cvWarpPerspective(temp_cam_calib_image,temp_proj_calib_image,c_p_homography);


    return;
}



//Store projector image & object points for view:'view_number'
void record_image_and_object_points(int view_number)
{
    CvMat*temp_proj_calib_points=cvCreateMat(3,total_physical_checkerboard_corners,CV_64FC1);



    cvMatMul(c_p_homography,camera_image_points_calib,temp_proj_calib_points);


    //Homogenize the points:
    for(int t=0; t<total_physical_checkerboard_corners; t++)
        for(int p=0; p<3; p++)
        {
            CV_MAT_ELEM(*temp_proj_calib_points,double,p,t)/=CV_MAT_ELEM(*temp_proj_calib_points,double,2,t);

        }

    for(int r=0; r<total_physical_checkerboard_corners; r++)
        for(int k=0; k<2; k++)
        {
            CV_MAT_ELEM(*proj_calib_image_points,double,r+view_number*total_physical_checkerboard_corners,k)=CV_MAT_ELEM(*temp_proj_calib_points,double,k,r);
        }


    for(int t=0; t<total_physical_checkerboard_corners; t++)
    {
        CV_MAT_ELEM(*proj_calib_object_points,double,t+view_number*total_physical_checkerboard_corners,0)=(double)((physical_checkerboard_square_mm+1.0)*(t%physical_checkerboard_X));
        CV_MAT_ELEM(*proj_calib_object_points,double,t+view_number*total_physical_checkerboard_corners,1)=(double)((physical_checkerboard_square_mm+1.0)*(t/physical_checkerboard_X));
        CV_MAT_ELEM(*proj_calib_object_points,double,t+view_number*total_physical_checkerboard_corners,2)=0.0;
    }



    return;
}



//Call OpenCV routine for projector calibration.
void call_calibration_routine()
{

    double reproj_error=0.0;
    int calib_flags=0;
    calib_flags=CV_CALIB_FIX_K3|CV_CALIB_ZERO_TANGENT_DIST;

    proj_intrinsic_mat=cvCreateMat(3,3,CV_64FC1);
    proj_dist_vect=cvCreateMat(5,1,CV_64FC1);

    reproj_error=cvCalibrateCamera2(proj_calib_object_points,proj_calib_image_points,global_corner_count,cvSize(Projector_imagewidth,Projector_imageheight),proj_intrinsic_mat,proj_dist_vect,NULL,NULL,calib_flags);

    printf("\nProjector calibrated with Model reprojection error:%lf",reproj_error);


    return;
}





void save_calibration_parameters_and_images(int n_views)
{
    //Save calibration images of projector & corresponding camera reference images.
    for(int y=0; y<n_views; y++)
    {
        sprintf(filename,"Projector_calibration/Captured_images/Proj_View_%d.jpg",y);
        cvSaveImage(filename,proj_calib_images[y]);

        sprintf(filename,"/Projector_calibration/Captured_images/Cam_View_%d.jpg",y);
        cvSaveImage(filename,camera_calib_images[y]);
    }

    //Save projector parameters.
    cvSave("Projector_calibration/Matrices/Projector_intrinsic_mat.xml",proj_intrinsic_mat);
    cvSave("Projector_calibration/Matrices/Projector_dist_vect.xml",proj_dist_vect);

    return;
}









// External Interface(S.Zhang's method)

void system_calibration()
{
    int n_views=0;

    printf("\nCamera calibration?");
    fflush(stdout);

    if(cvWaitKey(0)=='y')
        calibrate_camera();

    printf("\nCalibrate projector?");
    fflush(stdout);

    if(cvWaitKey(0)=='y')
    {
        printf("\nPlease enter number of views:");
        scanf("%d",&n_views);

        cvNamedWindow("Preview detected corners",0);
        cvNamedWindow("Preview alignment",0);
        //Lets allocate camera & projector calibration images & point matrices.
        camera_calib_images=new IplImage*[n_views];
        proj_calib_images=new IplImage*[n_views];

        camera_image_points_calib=cvCreateMat(3,total_physical_checkerboard_corners,CV_64FC1);
        proj_calib_image_points=cvCreateMat(total_physical_checkerboard_corners*n_views,2,CV_64FC1);
        proj_calib_object_points=cvCreateMat(total_physical_checkerboard_corners*n_views,3,CV_64FC1);
        global_corner_count=cvCreateMat(n_views,1,CV_32SC1);


        compute_c_p_homography();

        for(int i=0; i<n_views; i++)
        {
            capture_checkerboard_image(i);
            map_image_camera_to_projector(i);
            record_image_and_object_points(i);
        }

        call_calibration_routine();
        save_calibration_parameters_and_images(n_views);

        printf("\nProjector calibrated successfully!!");
    }
    printf("\nExtrinisic calibration?");
    fflush(stdout);

    if(cvWaitKey(0)=='y')
        extrinsic_calibration();

    else
        load_matrices();




    return;
}
*/
