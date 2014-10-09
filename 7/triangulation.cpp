/*Steps:-
1.Transform the origin of pixel space of projector to the corresponding camera space.
2.Define the center of projection of projector(COP2) wrt. new origin & same for all pixels in projector space.
3.Compute vectors P2COP2 & COP1COP2 wrt. this origin.
4.Compute the angle b/w vectors COP1COP2 & P2COP2(beta_angle) similarly b/w COP1COP2 & P1COP1(alpha_angle).
5.Compute 'b' using distance formula(in camera pixel-space).
6.Compute depth using triangulation formula.
7.Scale the computed depth into the world dimention by multiplying it by the scale factor(pixels per inch factor to give the depth in inches).
8.Done.
*/


//EXTENSIONS:-
/*
1.Using cartesian method of intersection of line.
2.Using parametric equation of lines.
3.Computing depth along camera optical ray.
4.Computing depth using stereo-rectification:-disparity map.
5.Accomodate for non-intersection problem of 'practical' triangulation.
*/




//NOTE:-In my code focal length is distance between camera image plane(or sensor) & lens.



//ASSUMPTION:-Distance between camera lens & sensor similarly for projector is known.
//ASSUMPTION:-pixel-to-pixel correspondance is known.
//ASSUMPTION:-Origin of Camera Coordinate System is Center Of Projection.
//ASSUMPTION:-Depth is computed with respect to base-line between corresponding pixels.
//ASSUMPTION:-Angle 'alpha_angle' & 'beta_angle' are in radians.

///UPDATE:9 July 2012
/*
Adding method-3(based on solving system of linear equations) for triangulation.
*/

#include "../PROJECT_GLOBAL/global_cv.h"

//////////////////////////////////////////////PROPRIETRY INFORMATION(MAY OR MAY NOT BE AVAILABLE).
#define camera_focal_length_mm 3.7 //In 'mm'
#define proj_focal_length_mm  23.5 //In 'mm' (Assumed for now)

#define camera_pixel_pitch_X_mm 0.0000028
#define camera_pixel_pitch_Y_mm 0.0000028 //MAKE ACCORDING TO ASPECT RATIO OF THE CAMERA COMPUTED USING CAMERA-CALIBRATION.
#define projector_pixel_pitch_X_mm 0.0000108
#define projector_pixel_pitch_Y_mm 0.0000108

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Global variables.
extern char filename[150];
extern char tagname[50];

extern CvMat*cam_intrinsic_mat;
extern CvMat*cam_dist_vect;

extern CvMat*proj_intrinsic_mat;
extern CvMat*proj_dist_vect;

extern int (*valid_map)[Camera_imageheight];
extern long int (*c_p_map)[2];
//Retrieved vector format variables.



extern CvMat*proj_cam_rot_mat;
extern CvMat*proj_cam_trans_vect;

extern CvMat*cam_world_rot_mat;
extern CvMat*cam_world_trans_vect;
extern CvMat*cam_world_rot_vect;
extern CvMat*proj_world_rot_mat;
extern CvMat*proj_world_trans_vect;
extern CvMat*proj_world_rot_vect;

//For triangulation.
extern double (*intersection_points)[Camera_imageheight][3];//it will contain the coordinates of point of intersection for each camera pixel(origin will be Camera-coordinate-system origin)

extern float gamma_value;



CvMat*v1=cvCreateMat(3,1,CV_64FC1);//Vector for camera optical ray.
CvMat*v2=cvCreateMat(3,1,CV_64FC1);//Vector for projector optical ray.


double v1_dot_v2=0.0;
double v1_square=0.0;//v1*Transpose(v1)
double v2_square=0.0;//v2*Transpose(v2)
double lambda_1=0.0;//global to be used by every pixel.(from camera's view)
double lambda_2=0.0;//from projector's view.

CvMat*q1=cvCreateMat(3,1,CV_64FC1);//First point of camera optical ray.
CvMat*q2=cvCreateMat(3,1,CV_64FC1);//First point of projector optical ray.



CvMat*proj_COP;//center of projection:projector.
CvMat*cam_COP;//center of projection:camera.

CvMat*proj_pixel_3D;//will contain the 3D coordinates of all pixels as viewed from camera center of projection.
CvMat*cam_pixel_3D;//will contain the 3D coordinates of all pixels as viewed from projector center of projection.


double cam_cx;//camera principal point.
double cam_cy;
double proj_cx;//projector principal point.
double proj_cy;



CvMat*P;
CvMat*P_trans;
CvMat*F;
CvMat*A_cam;
CvMat*A_proj;
CvMat*V;//it will contain X,Y,Z coordinates for all camera image points(wrt. world coordinate system)

CvMat*I1;
CvMat*I2;


//CvMat*baseline_vector;//this vector joins the COP1-COP2 point pair in camera coordinate system.
//double alpha_angle[total_camera_pixels];//It will contain angle between A-COP1 & COP1-COP2 for all pixels A.
//double beta_angle[total_projector_pixels];//It will contain angle between B-COP2 & COP1-COP2 for all pixels B.

//CvMat*cam_optical_ray;//optical ray emanating from each camera pixel.
//CvMat*proj_optical_ray;//optical ray emanating from each projector pixel.

//double baseline_length=0.0;//it will contain the length(magnitude) of baseline vector.


//double d[Camera_imagewidth][Camera_imageheight];//This will contain the depth for each camera pixel.(The main result of this program).




CvMat*cam_undist_points_mat;
CvMat*proj_undist_points_mat;

//extern bool assigned[Projector_imagewidth][Projector_imageheight];
//extern int valid_camera_pixel[Projector_imagewidth][Projector_imageheight][2];//contains the corresponding camera pixel coordinates for the valid projector pixels.



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Function definations.
//Following function will read the system calibration paramaters to be used.
void read_parameters()
{
    //Camera intrinsics
    CvFileStorage*fs_cam_intrinisics=cvOpenFileStorage("/home/pranav/Desktop/M_tech_project_console/Camera_calibration/Matrices/cam_intrinsic_mat.xml",0,CV_STORAGE_READ);
    cam_intrinsic_mat=(CvMat*)cvReadByName(fs_cam_intrinisics,0,"cam_intrinsic_mat");


    //Camera distortion parameters
    CvFileStorage*fs_cam_dist=cvOpenFileStorage("/home/pranav/Desktop/M_tech_project_console/Camera_calibration/Matrices/cam_distortion_vect.xml",0,CV_STORAGE_READ);
    cam_dist_vect=(CvMat*)cvReadByName(fs_cam_dist,0,"cam_distortion_vect");


    //Projector intrinisics
    CvFileStorage*fs_proj_intrinsic=cvOpenFileStorage("/home/pranav/Desktop/M_tech_project_console/Projector_calibration/Matrices/proj_intrinsic_mat.xml",0,CV_STORAGE_READ);
    proj_intrinsic_mat=(CvMat*)cvReadByName(fs_proj_intrinsic,0,"proj_intrinsic_mat");


    //Projector distortion parameters
    CvFileStorage*fs_proj_dist=cvOpenFileStorage("/home/pranav/Desktop/M_tech_project_console/Projector_calibration/Matrices/proj_distortion_vect.xml",0,CV_STORAGE_READ);
    proj_dist_vect=(CvMat*)cvReadByName(fs_proj_dist,0,"proj_distortion_vect");


    //De-allocate...
    cvReleaseFileStorage(&fs_cam_intrinisics);
    cvReleaseFileStorage(&fs_cam_dist);
    cvReleaseFileStorage(&fs_proj_intrinsic);
    cvReleaseFileStorage(&fs_proj_dist);



    return;
}








//Following function will compute the relative rotation & translation matrices of projector with respect to camera.
void read_relative_extrinsic_geometry() //Note:These transformation can be used for transforming points from projector to camera space.
{

    CvFileStorage*fs_proj_cam_rot_mat=cvOpenFileStorage("/home/pranav/Desktop/M_tech_project_console/Triangulation/Relative_geometry/proj_cam_rot_mat.xml",0,CV_STORAGE_READ);
    CvFileStorage*fs_proj_cam_trans_vect=cvOpenFileStorage("/home/pranav/Desktop/M_tech_project_console/Triangulation/Relative_geometry/proj_cam_trans_vect.xml",0,CV_STORAGE_READ);

    proj_cam_rot_mat=(CvMat*)cvReadByName(fs_proj_cam_rot_mat,0,"proj_cam_rot_mat");
    proj_cam_trans_vect=(CvMat*)cvReadByName(fs_proj_cam_trans_vect,0,"proj_cam_trans_vect");
    /*
        CvMat*rot_vector=cvCreateMat(3,1,CV_64FC1);

        cvRodrigues2(proj_cam_rot_mat,rot_vector);
        printf("TRANSFORMATION:");
        fflush(stdout);
        for(int i=0;i<3;i++)
        printf("%lf\t",CV_MAT_ELEM(*rot_vector,double,i,0));

    cvWaitKey(0);

    */

    //De-allocate...
    cvReleaseFileStorage(&fs_proj_cam_rot_mat);
    cvReleaseFileStorage(&fs_proj_cam_trans_vect);



    return;

}







//Following function will assign 3d coordinates to COP & pixels of camera image space.
void assign_3d_coordinates()
{
    //COP is at Z=f,coordinates are (0,0,0)
    //pixels are at (x,y,f) assuming ccd sensor is aligned parallel to the lens-plane.(principal-plane)

//lets intialize the coordinate array.
    cam_pixel_3D=cvCreateMat(4,total_camera_pixels,CV_64FC1); //homogenous representation.
    proj_pixel_3D=cvCreateMat(4,total_projector_pixels,CV_64FC1);

    cam_COP=cvCreateMat(4,1,CV_64FC1); //homogenous representation used.
    proj_COP=cvCreateMat(4,1,CV_64FC1);

//lets compute their values!!
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Camera.
//COP.
    CV_MAT_ELEM(*cam_COP,double,0,0)=0.0;
    CV_MAT_ELEM(*cam_COP,double,1,0)=0.0;
    CV_MAT_ELEM(*cam_COP,double,2,0)=0.0;
    CV_MAT_ELEM(*cam_COP,double,3,0)=1.0; //in homogenous form.
//pixels:-I have to undistort the pixels before assigning the 3D position to them.

///  LETS SAVE cam_COP FOR USE BY plot_optical_ray program.
    //cvSave("/home/pranav/Desktop/plot_optical_ray/cam_COP.xml",cam_COP);
//Lets undistort.
    CvMat*cam_dist_points=cvCreateMat(total_camera_pixels,1,CV_64FC2);
    CvMat*cam_undist_points=cvCreateMat(total_camera_pixels,1,CV_64FC2);
    cam_undist_points_mat=cvCreateMat(3,total_camera_pixels,CV_64FC1);

//lets take a set of temporary variable(cam_point & proj_point) for storing the distorted coordinates which are to be undistorted.

    CvMat*cam_point=cvCreateMat(total_camera_pixels,2,CV_64FC1);
    CvMat*proj_point=cvCreateMat(total_projector_pixels,2,CV_64FC1);

//intializing cam_point.
    for(int f=0; f<total_camera_pixels; f++)
    {
        CV_MAT_ELEM(*cam_point,double,f,0)=(double)(f%Camera_imagewidth); //column number.
        CV_MAT_ELEM(*cam_point,double,f,1)=(double)floorf((float)f/(float)Camera_imagewidth);//row number.
    }



//now intializing proj_point.
    for(int s=0; s<total_projector_pixels; s++)
    {
        CV_MAT_ELEM(*proj_point,double,s,0)=(double)(s%Projector_imagewidth);//column number.
        CV_MAT_ELEM(*proj_point,double,s,1)=(double)floorf((float)s/(float)Projector_imagewidth);//row number.
    }





//lets copy the distorted pixel positions first.
    CvScalar pd;
    for(int u=0; u<total_camera_pixels; u++)
    {
        cvSet1D(cam_dist_points,u,cvScalar(CV_MAT_ELEM(*cam_point,double,u,0),CV_MAT_ELEM(*cam_point,double,u,1)));

    }


    cvUndistortPoints(cam_dist_points,cam_undist_points,cam_intrinsic_mat,cam_dist_vect);//this will give me the position of undistorted coordinates for each corresponding pixel.

//lets copy these values to a matrix.
    for(int e=0; e<total_camera_pixels; e++)
    {
        pd=cvGet1D(cam_undist_points,e);
        CV_MAT_ELEM(*cam_undist_points_mat,double,0,e)=(double)pd.val[0];
        CV_MAT_ELEM(*cam_undist_points_mat,double,1,e)=(double)pd.val[1];
        CV_MAT_ELEM(*cam_undist_points_mat,double,2,e)=1.0;
    }


    cvMatMul(cam_intrinsic_mat,cam_undist_points_mat,cam_undist_points_mat);

//Homogenize..
    for(int s=0; s<total_camera_pixels; s++)
        for(int x=0; x<3; x++)
            CV_MAT_ELEM(*cam_undist_points_mat,double,x,s)/=CV_MAT_ELEM(*cam_undist_points_mat,double,2,s);

//Now lets copy the undistorted pixels positions to 'cam_pixel_3D'.

    //Lets read the camera principle point.
    cam_cx=CV_MAT_ELEM(*cam_intrinsic_mat,double,0,2)*(double)camera_pixel_pitch_X_mm;
    cam_cy=CV_MAT_ELEM(*cam_intrinsic_mat,double,1,2)*(double)camera_pixel_pitch_Y_mm*(CV_MAT_ELEM(*cam_intrinsic_mat,double,1,1)/CV_MAT_ELEM(*cam_intrinsic_mat,double,0,0));



//Testing SIGGRAPH-09 approach...
    /*
    double norm=0.0;
    for(int k=0;k<total_camera_pixels;k++)
    {
    norm=sqrt(pow(CV_MAT_ELEM(*cam_undist_points_mat,double,0,k),2.0)+pow(CV_MAT_ELEM(*cam_undist_points_mat,double,1,k),2.0)+1.0);
    CV_MAT_ELEM(*cam_pixel_3D,double,0,k)=CV_MAT_ELEM(*cam_undist_points_mat,double,0,k)/norm;
    CV_MAT_ELEM(*cam_pixel_3D,double,1,k)=CV_MAT_ELEM(*cam_undist_points_mat,double,1,k)/norm;
    CV_MAT_ELEM(*cam_pixel_3D,double,2,k)=1.0/norm;
    }
    */

    for(int k=0; k<total_camera_pixels; k++)
    {

        CV_MAT_ELEM(*cam_pixel_3D,double,0,k)=(CV_MAT_ELEM(*cam_undist_points_mat,double,0,k)-(double)cam_cx)*(double)camera_pixel_pitch_X_mm;//Relative to principal point
        CV_MAT_ELEM(*cam_pixel_3D,double,1,k)=(CV_MAT_ELEM(*cam_undist_points_mat,double,1,k)-(double)cam_cy)*(double)camera_pixel_pitch_Y_mm*(CV_MAT_ELEM(*cam_intrinsic_mat,double,1,1)/CV_MAT_ELEM(*cam_intrinsic_mat,double,0,0));//Relative to principal point.
        CV_MAT_ELEM(*cam_pixel_3D,double,2,k)=(double)camera_focal_length_mm; //all pixels assumed to be at equal depth wrt. camera COP.
        CV_MAT_ELEM(*cam_pixel_3D,double,3,k)=1.0;
    }


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Projector.
//lets define COP first!!
    CV_MAT_ELEM(*proj_COP,double,0,0)=0.0;
    CV_MAT_ELEM(*proj_COP,double,1,0)=0.0;
    CV_MAT_ELEM(*proj_COP,double,2,0)=0.0;
    CV_MAT_ELEM(*proj_COP,double,3,0)=1.0; //in homogenous form.

//NOTE:CURRENT VALUE OF 'COP' IS IN PROJECTORS OWN LOCAL COORDINATE SYSTEM NOT IN COMMON COORDINATE SYSTEM.



//lets undistort the projector coordinates.
//Lets undistort.
    CvMat*proj_dist_points=cvCreateMat(total_projector_pixels,1,CV_64FC2);
    CvMat*proj_undist_points=cvCreateMat(total_projector_pixels,1,CV_64FC2);
    proj_undist_points_mat=cvCreateMat(3,total_projector_pixels,CV_64FC1);

//lets copy the distorted pixel positions first.
    for(int h=0; h<total_projector_pixels; h++)
    {
        cvSet1D(proj_dist_points,h,cvScalar(CV_MAT_ELEM(*proj_point,double,h,0),CV_MAT_ELEM(*proj_point,double,h,1)));
    }


    cvUndistortPoints(proj_dist_points,proj_undist_points,proj_intrinsic_mat,proj_dist_vect);//this will give me the position of undistorted coordinates for each corresponding pixel.

    for(int z=0; z<total_projector_pixels; z++)
    {
        pd=cvGet1D(proj_undist_points,z);
        CV_MAT_ELEM(*proj_undist_points_mat,double,0,z)=(double)pd.val[0];
        CV_MAT_ELEM(*proj_undist_points_mat,double,1,z)=(double)pd.val[1];
        CV_MAT_ELEM(*proj_undist_points_mat,double,2,z)=1.0;
    }

    cvMatMul(proj_intrinsic_mat,proj_undist_points_mat,proj_undist_points_mat);

//Homogenize..
    for(int f=0; f<total_projector_pixels; f++)
        for(int b=0; b<3; b++)
            CV_MAT_ELEM(*proj_undist_points_mat,double,b,f)/=CV_MAT_ELEM(*proj_undist_points_mat,double,2,f);

//Lets read the projector principle point.
    proj_cx=CV_MAT_ELEM(*proj_intrinsic_mat,double,0,2)*(double)projector_pixel_pitch_X_mm;
    proj_cy=CV_MAT_ELEM(*proj_intrinsic_mat,double,1,2)*(double)projector_pixel_pitch_Y_mm;

    /*
    /// Testing SIGGRAPH-09 code
    for(int h=0;h<total_projector_pixels;h++)
    {
        norm=sqrt(pow(CV_MAT_ELEM(*proj_undist_points_mat,double,0,h),2.0)+pow(CV_MAT_ELEM(*proj_undist_points_mat,double,1,h),2.0)+1.0);
        CV_MAT_ELEM(*proj_pixel_3D,double,0,h)=CV_MAT_ELEM(*proj_undist_points_mat,double,0,h)/norm;
        CV_MAT_ELEM(*proj_pixel_3D,double,1,h)=CV_MAT_ELEM(*proj_undist_points_mat,double,1,h)/norm;
        CV_MAT_ELEM(*proj_pixel_3D,double,2,h)=1.0/norm;
    }
    */


//Now lets copy the undistorted pixels positions to 'proj_pixel_3D'.
    for(int k=0; k<total_projector_pixels; k++)
    {
        //ASSUMPTION:-all pixels assumed to be at equal depth wrt. proj COP.
        CV_MAT_ELEM(*proj_pixel_3D,double,0,k)=(CV_MAT_ELEM(*proj_undist_points_mat,double,0,k)-(double)proj_cx)*(double)projector_pixel_pitch_X_mm;
        CV_MAT_ELEM(*proj_pixel_3D,double,1,k)=(CV_MAT_ELEM(*proj_undist_points_mat,double,1,k)-(double)proj_cy)*(double)projector_pixel_pitch_Y_mm;
        CV_MAT_ELEM(*proj_pixel_3D,double,2,k)=(double)proj_focal_length_mm;
        CV_MAT_ELEM(*proj_pixel_3D,double,3,k)=1.0;
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////ASSIGNEMENT DONE,LETS TRANSFORM THE PROJECTOR POINTS NOW!!
///////////////////////////////////////////////////////Projector-coordinates ready to be transformed.


//RESULT:All pixels & COP are assigned local 3D-coordinate system's values & not to the common coordinate system.

//Lets save the camera pixel coordinates for 3D plotting purpose.
    /*
    FILE*fd_camera_pixels=fopen("/home/pranav/Desktop/plot_optical_ray/cam_3d_points.txt","w");
    /// FIXME:PLOTTING LARGE DATA SET(1600x1200) HANGS SYSTEM!!!
    for(int r=0;r<Camera_imageheight;r++)
    for(int c=0;c<Camera_imagewidth;c++)
    if(valid_map[c][r]==1)
    fprintf(fd_camera_pixels,"%lf\t%lf\t%lf\n",CV_MAT_ELEM(*cam_pixel_3D,double,0,r*Camera_imagewidth+c),CV_MAT_ELEM(*cam_pixel_3D,double,1,r*Camera_imagewidth+c),CV_MAT_ELEM(*cam_pixel_3D,double,2,r*Camera_imagewidth+c));


    fclose(fd_camera_pixels);
    */

    //cvSave("/home/pranav/Desktop/plot_optical_ray/cam_3d_points.xml",cam_pixel_3D);

    //De-allocate...
    cvReleaseMat(&cam_dist_points);
    cvReleaseMat(&cam_undist_points);
    cvReleaseMat(&cam_point);
    cvReleaseMat(&proj_point);
    cvReleaseMat(&proj_dist_points);
    cvReleaseMat(&proj_undist_points);



    return;

}







//Following function tranforms the projector pixel & COP coordinates from local coordinate system to common(or camera) coordinate system.
void transform_proj_coordinates()
{
////////////////////////////////////////////////////////Lets compute the projector points(pixels & COP) as vied from camera COP(origin of global coordinate system).
    //Transformation is to be done as:-
    //Pc=Req*Pp+Teq
    //Lets get the 4X4 matrices.

    //CvMat*proj_cam_rot_mat_4_4=cvCreateMat(4,4,CV_64FC1);
    //CvMat*proj_cam_trans_vect_4_1=cvCreateMat(4,1,CV_64FC1);


    CvMat*proj_cam_extrinsic_mat=cvCreateMat(4,4,CV_64FC1);


    for(int y=0; y<3; y++)
        for(int g=0; g<3; g++)
            CV_MAT_ELEM(*proj_cam_extrinsic_mat,double,y,g)=CV_MAT_ELEM(*proj_cam_rot_mat,double,y,g);

    for(int q=0; q<3; q++)
    {
        //CV_MAT_ELEM(*proj_cam_extrinsic_mat,double,q,3)=0.0;
        CV_MAT_ELEM(*proj_cam_extrinsic_mat,double,3,q)=0.0;
    }

    CV_MAT_ELEM(*proj_cam_extrinsic_mat,double,3,3)=1.0;



    for(int w=0; w<3; w++)
        CV_MAT_ELEM(*proj_cam_extrinsic_mat,double,w,3)=CV_MAT_ELEM(*proj_cam_trans_vect,double,w,0);

    /*
    printf("Debug:\n");
    for(int a=0; a<4; a++)
    {
        for(int c=0; c<4; c++)
            printf("%lf\t",CV_MAT_ELEM(*proj_cam_extrinsic_mat,double,a,c));

        printf("\n");
    }
    */

    /*
        printf("From calibration:");
        printf("Rotation matrix:");
        for(int h=0; h<3; h++)
        {
            for(int d=0; d<3; d++)
                printf("%lf\t",CV_MAT_ELEM(*proj_cam_rot_mat,double,h,d));

            printf("\n");
        }

        printf("Translation vector:");
        for(int x=0; x<3; x++)
            printf("%lf\t",CV_MAT_ELEM(*proj_cam_trans_vect,double,x,0));

    */
    //CV_MAT_ELEM(*proj_cam_trans_vect_4_1,double,3,0)=1.0;

    cvMatMul(proj_cam_extrinsic_mat,proj_pixel_3D,proj_pixel_3D);
    cvMatMul(proj_cam_extrinsic_mat,proj_COP,proj_COP);


//Lets homogenize the points.
//pixels.
    for(int q=0; q<total_projector_pixels; q++)
        for(int p=0; p<4; p++)
            CV_MAT_ELEM(*proj_pixel_3D,double,p,q)/=CV_MAT_ELEM(*proj_pixel_3D,double,3,q);


//COP also!!
    for(int z=0; z<4; z++)
        CV_MAT_ELEM(*proj_COP,double,z,0)/=CV_MAT_ELEM(*proj_COP,double,3,0);

    /*
    //Lets TRANSLATE the points!!
        for(int i=0; i<total_projector_pixels; i++)
            for(int j=0; j<3; j++)
                CV_MAT_ELEM(*proj_pixel_3D,double,j,i)+=CV_MAT_ELEM(*proj_cam_trans_vect_4_1,double,j,0);

    //Lets do the same for Center of projection of projector also.
        for(int l=0; l<3; l++)
            CV_MAT_ELEM(*proj_COP,double,l,0)+=CV_MAT_ELEM(*proj_cam_trans_vect_4_1,double,l,0);
    */



///  LETS SAVE proj_COP TO BE USED IN plot_optical_ray PROGRAM.
//   cvSave("/home/pranav/Desktop/plot_optical_ray/proj_COP.xml",proj_COP);


//CAN WE PLOT THESE 3D-MODEL POINTS FOR VERIFICATION????
//WE WILL!!
    /*
    FILE*fd_proj_pixels=fopen("/home/pranav/Desktop/plot_optical_ray/proj_3d_points.txt","w");
    for(int g=0;g<Projector_imagewidth;g++)
    for(int n=0;n<Projector_imageheight;n++)
    if(valid_map[g][n]==1)
    fprintf(fd_proj_pixels,"%lf\t%lf\t%lf\n",CV_MAT_ELEM(*proj_pixel_3D,double,0,n*Projector_imagewidth+g),CV_MAT_ELEM(*proj_pixel_3D,double,1,n*Projector_imagewidth+g),CV_MAT_ELEM(*proj_pixel_3D,double,2,n*Projector_imagewidth+g));


    fclose(fd_proj_pixels);

    */

    // cvSave("/home/pranav/Desktop/plot_optical_ray/proj_3d_points.xml",proj_pixel_3D);


//Now points are transformed!!
//Done!!
//RESULT:'proj_pixel_3D' & 'cam_pixel_3D' contain the 3D coordinates as viewed from COP of camera as origin.
//Also 'proj_COP' contains the transformed COP.


//Deallocate...
    cvReleaseMat(&proj_cam_extrinsic_mat);


    return;

}



/*

//Following procedure will perform dot product between the optical ray vector corresponding to each pixel ray with the base line vector to determine the angle between thwem
double a_dot_baseline_vect(int pixel_index)
{
    double dot_prod=0.0;

    for(int i=0; i<3; i++)
        dot_prod+=CV_MAT_ELEM(*cam_pixel_3D,double,i,pixel_index)*CV_MAT_ELEM(*baseline_vector,double,i,0);

//RESULT:dot_prod contains (a).(baseline)
//Done!!
    return dot_prod;
}





//Following procedure will perform dot product between the optical ray vector corresponding to each pixel ray with the base line vector to determine the angle between thwem
double b_dot_baseline_vect(int pixel_index)
{
    double dot_prod=0.0;

    for(int i=0; i<3; i++)
        dot_prod+=CV_MAT_ELEM(*proj_pixel_3D,double,i,pixel_index)*CV_MAT_ELEM(*baseline_vector,double,i,0)*(-1.0);//-1 multiplied to filp the direction of vector to COP2->COP1,so that correct angle is calculated.

//RESULT:dot_prod=(b).(baseline)
//Done!!
    return dot_prod;
}








//Following code computes alpha_angle & beta_angle for each pixel pair.
void define_alpha_angle_beta_angle()
{
//define baseline COP1->COP2.
    baseline_vector=cvCreateMat(3,1,CV_64FC1); //CURRENTLY DIRECTION OF VECTOR IS ASSUMED TO BE FROM COP1->COP2.(To be used for alpha_angle calculation)
    CV_MAT_ELEM(*baseline_vector,double,0,0)=CV_MAT_ELEM(*proj_COP,double,0,0)-CV_MAT_ELEM(*cam_COP,double,0,0);
    CV_MAT_ELEM(*baseline_vector,double,1,0)=CV_MAT_ELEM(*proj_COP,double,1,0)-CV_MAT_ELEM(*cam_COP,double,1,0);
    CV_MAT_ELEM(*baseline_vector,double,2,0)=CV_MAT_ELEM(*proj_COP,double,2,0)-CV_MAT_ELEM(*cam_COP,double,2,0);
//Now vector is defined lets do the same for A-COP1 & COP1-COP2.
//define vectors containing A-COP1 & B-COP2.

    cam_optical_ray=cvCreateMat(3,total_camera_pixels,CV_64FC1);
    proj_optical_ray=cvCreateMat(3,total_projector_pixels,CV_64FC1);





//optical rays for camera pixels.
    for(int i=0; i<total_camera_pixels; i++)
        for(int p=0; p<3; p++)
        {
            CV_MAT_ELEM(*cam_optical_ray,double,p,i)=CV_MAT_ELEM(*cam_pixel_3D,double,p,i)-CV_MAT_ELEM(*cam_COP,double,p,0);
        }
    printf("Coordinates of camera-COP:\n");
    printf("X=%lf\tY=%lf\tZ=%lf\n",CV_MAT_ELEM(*cam_COP,double,0,0),CV_MAT_ELEM(*cam_COP,double,1,0),CV_MAT_ELEM(*cam_COP,double,2,0));










//optical rays for projector pixels.
    for(int j=0; j<total_projector_pixels; j++)
        for(int y=0; y<3; y++)
            CV_MAT_ELEM(*proj_optical_ray,double,y,j)=CV_MAT_ELEM(*proj_pixel_3D,double,y,j)-CV_MAT_ELEM(*proj_COP,double,y,0);

    printf("Coordinates of projector-COP:\n");
    printf("X=%lf\tY=%lf\tZ=%lf\n",CV_MAT_ELEM(*proj_COP,double,0,0),CV_MAT_ELEM(*proj_COP,double,1,0),CV_MAT_ELEM(*proj_COP,double,2,0));




//compute angle between A-COP1 & COP1-COP2.
    double magn_pixel_optical_ray=0.0;
    double magn_baseline_vector=0.0;
//lets compute the magnitude of baseline once as it is going to remain same.
    for(int z=0; z<3; z++)
        magn_baseline_vector+=(double)powf((float)CV_MAT_ELEM(*baseline_vector,double,z,0),2.0);

    magn_baseline_vector=(double)sqrtf((float)magn_baseline_vector);
//done!!

    for(int q=0; q<total_camera_pixels; q++)
    {
        magn_pixel_optical_ray=0.0;

        for(int e=0; e<3; e++)
            magn_pixel_optical_ray+=(double)powf((float)CV_MAT_ELEM(*cam_optical_ray,double,e,q),2.0);

        magn_pixel_optical_ray=(double)sqrtf((float)magn_pixel_optical_ray);
//done!!

//lets use the computed values in alpha_angle calculation.
        alpha_angle[q]=acosf(a_dot_baseline_vect(q)/(magn_pixel_optical_ray*magn_baseline_vector)); //In radians.


    }




//compute angle between B-COP2 & COP1-COP2(that is 'beta_angle'!!).

    for(int b=0; b<total_projector_pixels; b++)
    {
        magn_pixel_optical_ray=0.0;

        for(int c=0; c<3; c++)
            magn_pixel_optical_ray+=(double)powf((float)CV_MAT_ELEM(*proj_optical_ray,double,c,b),2.0);

        magn_pixel_optical_ray=(double)sqrtf((float)magn_pixel_optical_ray);
//done!!


        beta_angle[b]=(double)acosf((float)b_dot_baseline_vect(b)/(float)(magn_pixel_optical_ray*magn_baseline_vector)); //In radians.

    }



//Done!!
//RESULT:we have alpha_angle and beta_angle in degrees for each pixel in camera & projector pixel array respectively.

    return;

}




//Following function will compute the cartesian distance between points COP1 & COP2(called 'baseline length').
void define_baseline_length()
{
//I can use distance formula to compute the cartesian distance between 2 points(COP1 & COP2).

    for(int r=0; r<3; r++)
        baseline_length+=(double)powf((float)CV_MAT_ELEM(*baseline_vector,double,r,0),2.0);

    baseline_length=(double)sqrtf((float)baseline_length);

//RESULT:-'baseline_length' will contain the cartesian distance between is COP1 & COP2.

//Done!!
    return;
}


//following procedure will contain the 'd' factor:perpendicula distance of all points with respect to the baseline between COP1-COP2.
void compute_d(int (*c_p_map)[2])
{
//here i have to use the correspondance-map to associate an alpha_angle with its correct beta_angle.
    for(int i=0; i<Camera_imageheight; i++)
        for(int j=0; j<Camera_imagewidth; j++)
            if(valid_map[j][i]==1)
                d[j][i]=(double)(tanf((float)alpha_angle[i*Camera_imagewidth+j])*tanf((float)beta_angle[c_p_map[i*Camera_imagewidth+j][1]*Projector_imagewidth+c_p_map[i*Camera_imagewidth+j][0]])*(float)baseline_length)/(double)(tanf((float)alpha_angle[i*Camera_imagewidth+j])+tanf((float)beta_angle[c_p_map[i*Camera_imagewidth+j][1]*Projector_imagewidth+c_p_map[i*Camera_imagewidth+j][0]]));


    printf("Baseline length:%lf",baseline_length);
//RESULT:-this will compute the perpemndicular distance of points from the baseline for all pixels in 'd'.
    cvWaitKey(0);
//Done!!
    return;
}


void save_depth_map()
{
    //Lets convert the depth map into matrix form so that it can be saved.
    CvMat*depth_map=cvCreateMat(Camera_imageheight,Camera_imagewidth,CV_64FC1);
    for(int row=0; row<Camera_imageheight; row++)
        for(int col=0; col<Camera_imagewidth; col++)
            if(valid_map[col][row]==1)
            {
                CV_MAT_ELEM(*depth_map,double,row,col)=d[col][row];
            }

            else
                CV_MAT_ELEM(*depth_map,double,row,col)=-1.0;//invalid value.(Assuming object of interest to be in front.)



    cvSave("Triangulation/Depth_map/depth_map.xml",depth_map);

//Lets compute depth-image also.
    IplImage*depth_image=cvCreateImage(cvSize(Camera_imagewidth,Camera_imageheight),IPL_DEPTH_8U,1);

    for(int i=0; i<Camera_imageheight; i++)
        for(int j=0; j<Camera_imagewidth; j++)
        {
            if(valid_map[j][i]==1)
            {
                depth_image->imageData[i*depth_image->widthStep+j]=(unsigned char)(d[j][i]/10.0);

            }

        }


    cvSaveImage("depth_image.bmp",depth_image);


    return;
}




//Following function will compute depth for each corresponding pixel pairs.(Main triangulation function!!)
void compute_depth_method_1(int (*c_p_map)[2])
{
    define_alpha_angle_beta_angle();//Computes the orientation of optical rays from camera pixels & projector pixels.
    define_baseline_length();//Defines the length of baseline.


    //for each pair of corresponding pixels.
    compute_d(c_p_map);//this will compute the perpendicular distance of every point from

    printf("\nNow saving the depth map...\n");
    save_depth_map();
    printf("Done!!");

//RESULT:-we get the depth-map.
//DONE!!

    return;

}





//Following function will calculate the lambda value which determine the intersection point.
void calculate_lambda(int i,int j)
{
//NOTE:Both lambdas will calculate same coordinate value because both(projector & camera)lines are in same coordinate system.
//Define vectors 1 & 2.
//Lets initialize.
//q1,q2,v1,v2.
    for(int u=0; u<3; u++)
    {
        CV_MAT_ELEM(*q1,double,u,0)=CV_MAT_ELEM(*cam_COP,double,u,0);
        CV_MAT_ELEM(*q2,double,u,0)=CV_MAT_ELEM(*proj_COP,double,u,0);
        CV_MAT_ELEM(*v1,double,u,0)=CV_MAT_ELEM(*cam_pixel_3D,double,u,j*Camera_imagewidth+i)-CV_MAT_ELEM(*cam_COP,double,u,0);
        CV_MAT_ELEM(*v2,double,u,0)=CV_MAT_ELEM(*proj_pixel_3D,double,u,Projector_imagewidth*c_p_map[j*Camera_imagewidth+i][1]+c_p_map[j*Camera_imagewidth+i][0])-CV_MAT_ELEM(*proj_COP,double,u,0);
    }


//v1_square,v2_square,v1_X_v2.(scalar quantities)
    v1_square=0.0;
    v2_square=0.0;
    v1_dot_v2=0.0;
    for(int p=0; p<3; p++)
    {
        v1_square+=CV_MAT_ELEM(*v1,double,p,0)*CV_MAT_ELEM(*v1,double,p,0);
        v2_square+=CV_MAT_ELEM(*v2,double,p,0)*CV_MAT_ELEM(*v2,double,p,0);
        v1_dot_v2+=CV_MAT_ELEM(*v1,double,p,0)*CV_MAT_ELEM(*v2,double,p,0);
    }

    double t1=0.0,t2=0.0,t3=0.0;

///   ||V2||^2*V1^T*(Q2-Q1)

    double t11=0.0,t12=0.0;

    ///  V1^T*(Q1-Q2)
    for(int r=0; r<3; r++)
    {
        t11=CV_MAT_ELEM(*q2,double,r,0)-CV_MAT_ELEM(*q1,double,r,0);
        t12+=CV_MAT_ELEM(*v1,double,r,0)*t11;
    }

    /// ||V2||^2 * t12

    t1=v2_square*t12;


///  (V1^T*V2)*(V2^T*(Q1-Q2))
    double t21=0.0,t22=0.0;

    /// V2^T*(Q1-Q2)
    for(int y=0; y<3; y++)
    {
        t21=CV_MAT_ELEM(*q1,double,y,0)-CV_MAT_ELEM(*q2,double,y,0);
        t22+=CV_MAT_ELEM(*v2,double,y,0)*t21;
    }

    /// (V1^T*V2)*t22
    t2=v1_dot_v2*t22;


/// { ||V1||^2 * ||V2||^2 -(V1^T*V2)^2 }



    t3=v1_square*v2_square-(double)pow(v1_dot_v2,2.0);


    lambda_1=(t1+t2)/(t3); //NOTE:lambda_1 is a global variable.



///  Calculate lambda_2 for projector's optical ray.


/// { V2^T*V1*(V1^T*(Q2-Q1))}
    double t4=0.0,t5=0.0;
    for(int s=0; s<3; s++)
        t4+=CV_MAT_ELEM(*v1,double,s,0)*(CV_MAT_ELEM(*q2,double,s,0)-CV_MAT_ELEM(*q1,double,s,0));

    t5=v1_dot_v2*t4;

/// {||V1||^2*(V2^T*(Q1-Q2))}
    double t6=0.0,t7=0.0;

    for(int a=0; a<3; a++)
        t6+=CV_MAT_ELEM(*v2,double,a,0)*(CV_MAT_ELEM(*q1,double,a,0)-CV_MAT_ELEM(*q2,double,a,0));

    t7=v1_square*t6;


/// {||V1||^2*||V2||^2-(V1^T*V2)^2}
//NOTE:above expression is already computed in 't3'

///lambda_2
    lambda_2=(t5+t7)/t3;


    return ;
}




double calculate_X()
{
    //double t=CV_MAT_ELEM(*q1,double,0,0)+lambda_1*CV_MAT_ELEM(*v1,double,0,0)+CV_MAT_ELEM(*q2,double,0,0)+lambda_2*CV_MAT_ELEM(*v2,double,0,0);
    double t=CV_MAT_ELEM(*q1,double,0,0)+lambda_1*CV_MAT_ELEM(*v1,double,0,0);
    return (t/2.0);
}




double calculate_Y()
{
    //double t=CV_MAT_ELEM(*q1,double,1,0)+lambda_1*CV_MAT_ELEM(*v1,double,1,0)+CV_MAT_ELEM(*q2,double,1,0)+lambda_2*CV_MAT_ELEM(*v2,double,1,0);
    double t=CV_MAT_ELEM(*q1,double,1,0)+lambda_1*CV_MAT_ELEM(*v1,double,1,0);
    return (t/2.0);
}



double calculate_Z()
{
    //double t=CV_MAT_ELEM(*q1,double,2,0)+lambda_1*CV_MAT_ELEM(*v1,double,2,0)+CV_MAT_ELEM(*q2,double,2,0)+lambda_2*CV_MAT_ELEM(*v2,double,2,0);
    double t=CV_MAT_ELEM(*q1,double,2,0)+lambda_1*CV_MAT_ELEM(*v1,double,2,0);
    return (t/2.0);
}




//Following function will compute the intersection point for the optical rays passing through corresponding projector & camera pixel pair.
void compute_intersection()
{
    //We will use the theory:parametric equation of 3d lines.
    //Intersection point can be computed as:

    for(int i=0; i<Camera_imagewidth; i++)
        for(int j=0; j<Camera_imageheight; j++)
        {

            if(valid_map[i][j]==1)
            {
                calculate_lambda(i,j);
                intersection_points[i][j][0]=calculate_X(); //These function will calculate the X,Y,Z values for the intersection point(approximate intersection point)
                intersection_points[i][j][1]=calculate_Y();
                intersection_points[i][j][2]=calculate_Z();
            }
        }

    return;
}

/*
//Interactively take coordinates.
void take_camera_coordinates()
{

}

//Dump line coordinates into a file.(used for 3D plotting)
void save_coordinates()
{
  CvPoint*cam_sp;
  CvPoint*cam_ep;


}
*/
/*
//Following function will take camera-pixel coordinate as input and will plot the optical ray pair for it.
void plot_optical_ray_pair()
{
//Take input coordinates.
//Dump the end points of camera & corresponding projector-optical ray.
take_camera_coordinates();
save_coordinates();

return;
}
*/






/*
//Following function will plot the intersection points for the corresponding pairs computed earlier.(a point-cloud)
void plot_intersection_points()
{
//Now lets dump these intersection values to a file to be read by plot_3D program.
    FILE*fs_intersection_point=fopen("/home/pranav/Desktop/9/3D_plot/intersection_points.txt","w");
    int count=0;
    for(int i=0; i<Camera_imagewidth; i++)
        for(int j=0; j<Camera_imageheight; j++)
        {
            if(valid_map[i][j]==1)
            {

                fprintf(fs_intersection_point,"%f\t%f\t%f\n",(float)intersection_points[i][j][0],(float)intersection_points[i][j][1],(float)intersection_points[i][j][2]);

                count++;
            }

        }




    printf("%d",count);


    fclose(fs_intersection_point);

    return;
}




void compute_depth_method_2()
{

    compute_intersection();
    plot_intersection_points();//Dumps the intersection points to be used by plot3d program.

    return;
}


*/

/// Method-3
///Based on simaltaneous equation solving for camera projector system
///Reference:Section-5.2(AUGMENTING COMPLEX SURFACES WITH PROJECTOR-CAMERA SYSTEMS by Brett R. Jones)

//Global variables for the method.



//A=[camera_mat]*[R|t]
void compute_A()
{
    //NOTE:'cam_world_rot_mat','cam_world_trans_vect' & 'proj_world_rot_mat','proj_world_trans_vect' already contains the rotation matrices for rotation of world coordinate system wrt. camera coordinate system.
    A_cam=cvCreateMat(3,4,CV_64FC1);

    cam_world_rot_mat=cvCreateMat(3,3,CV_64FC1);
    proj_world_rot_mat=cvCreateMat(3,3,CV_64FC1);
    //Lets read the rotation/translation vectors for camera & projector.
    CvFileStorage*fs_cam_world_rot_vect=cvOpenFileStorage("/home/pranav/Desktop/M_tech_project_console/Triangulation/Camera_extrinsic_parametrs/world_to_cam_rot_vect.xml",0,CV_STORAGE_READ);
    cam_world_rot_vect=(CvMat*)cvReadByName(fs_cam_world_rot_vect,0,"world_to_cam_rot_vect");

    cvRodrigues2(cam_world_rot_vect,cam_world_rot_mat);

    CvFileStorage*fs_cam_world_trans_vect=cvOpenFileStorage("/home/pranav/Desktop/M_tech_project_console/Triangulation/Camera_extrinsic_parametrs/world_to_cam_trans_vect.xml",0,CV_STORAGE_READ);
    cam_world_trans_vect=(CvMat*)cvReadByName(fs_cam_world_trans_vect,0,"world_to_cam_trans_vect");

    CvFileStorage*fs_proj_world_rot_vect=cvOpenFileStorage("/home/pranav/Desktop/M_tech_project_console/Triangulation/Projector_extrinsic_parametrs/world_to_proj_rot_vect.xml",0,CV_STORAGE_READ);
    proj_world_rot_vect=(CvMat*)cvReadByName(fs_proj_world_rot_vect,0,"world_to_proj_rot_vect");

    cvRodrigues2(proj_world_rot_vect,proj_world_rot_mat);

    CvFileStorage*fs_proj_world_trans_vect=cvOpenFileStorage("/home/pranav/Desktop/M_tech_project_console/Triangulation/Projector_extrinsic_parametrs/world_to_proj_trans_vect.xml",0,CV_STORAGE_READ);
    proj_world_trans_vect=(CvMat*)cvReadByName(fs_proj_world_trans_vect,0,"world_to_proj_trans_vect");






    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
        {
            CV_MAT_ELEM(*A_cam,double,i,j)=CV_MAT_ELEM(*cam_world_rot_mat,double,i,j);

        }


    for(int i=0; i<3; i++)
        CV_MAT_ELEM(*A_cam,double,i,3)=CV_MAT_ELEM(*cam_world_trans_vect,double,i,0);

    cvMatMul(cam_intrinsic_mat,A_cam,A_cam);


    A_proj=cvCreateMat(3,4,CV_64FC1);

    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
        {
            CV_MAT_ELEM(*A_proj,double,i,j)=CV_MAT_ELEM(*proj_world_rot_mat,double,i,j);
        }


    for(int i=0; i<3; i++)
        CV_MAT_ELEM(*A_proj,double,i,3)=CV_MAT_ELEM(*proj_world_trans_vect,double,i,0);

    cvMatMul(proj_intrinsic_mat,A_proj,A_proj);


//De-allocate...
    cvReleaseFileStorage(&fs_cam_world_rot_vect);
    cvReleaseFileStorage(&fs_cam_world_trans_vect);
    cvReleaseFileStorage(&fs_proj_world_rot_vect);
    cvReleaseFileStorage(&fs_proj_world_trans_vect);

    return;
}



static int init_count=0;
int corresponding_X;
int corresponding_Y;

void compute_P(int matrix_col,int matrix_row)
{
    if(init_count++==0)
    {
        P=cvCreateMat(4,3,CV_64FC1);
        F=cvCreateMat(4,1,CV_64FC1);
        V=cvCreateMat(3,1,CV_64FC1);
        P_trans=cvCreateMat(3,4,CV_64FC1);
        I1=cvCreateMat(3,3,CV_64FC1);
        I2=cvCreateMat(3,4,CV_64FC1);


    }

//P for point (matrix_col,matrix_row)
    corresponding_X=c_p_map[matrix_row*Camera_imagewidth+matrix_col][0];
    corresponding_Y=c_p_map[matrix_row*Camera_imagewidth+matrix_col][1];

    CV_MAT_ELEM(*P,double,0,0)=CV_MAT_ELEM(*A_cam,double,0,0)-CV_MAT_ELEM(*cam_undist_points_mat,double,0,matrix_row*Camera_imagewidth+matrix_col)*CV_MAT_ELEM(*A_cam,double,2,0);
    CV_MAT_ELEM(*P,double,0,1)=CV_MAT_ELEM(*A_cam,double,0,1)-CV_MAT_ELEM(*cam_undist_points_mat,double,0,matrix_row*Camera_imagewidth+matrix_col)*CV_MAT_ELEM(*A_cam,double,2,1);
    CV_MAT_ELEM(*P,double,0,2)=CV_MAT_ELEM(*A_cam,double,0,2)-CV_MAT_ELEM(*cam_undist_points_mat,double,0,matrix_row*Camera_imagewidth+matrix_col)*CV_MAT_ELEM(*A_cam,double,2,2);;


    CV_MAT_ELEM(*P,double,1,0)=CV_MAT_ELEM(*A_cam,double,1,0)-CV_MAT_ELEM(*cam_undist_points_mat,double,1,matrix_row*Camera_imagewidth+matrix_col)*CV_MAT_ELEM(*A_cam,double,2,0);
    CV_MAT_ELEM(*P,double,1,1)=CV_MAT_ELEM(*A_cam,double,1,1)-CV_MAT_ELEM(*cam_undist_points_mat,double,1,matrix_row*Camera_imagewidth+matrix_col)*CV_MAT_ELEM(*A_cam,double,2,1);
    CV_MAT_ELEM(*P,double,1,2)=CV_MAT_ELEM(*A_cam,double,1,2)-CV_MAT_ELEM(*cam_undist_points_mat,double,1,matrix_row*Camera_imagewidth+matrix_col)*CV_MAT_ELEM(*A_cam,double,2,2);


    CV_MAT_ELEM(*P,double,2,0)=CV_MAT_ELEM(*A_proj,double,0,0)-CV_MAT_ELEM(*proj_undist_points_mat,double,0,corresponding_Y*Projector_imagewidth+corresponding_X)*CV_MAT_ELEM(*A_proj,double,2,0);
    CV_MAT_ELEM(*P,double,2,1)=CV_MAT_ELEM(*A_proj,double,0,1)-CV_MAT_ELEM(*proj_undist_points_mat,double,0,corresponding_Y*Projector_imagewidth+corresponding_X)*CV_MAT_ELEM(*A_proj,double,2,1);
    CV_MAT_ELEM(*P,double,2,2)=CV_MAT_ELEM(*A_proj,double,0,2)-CV_MAT_ELEM(*proj_undist_points_mat,double,0,corresponding_Y*Projector_imagewidth+corresponding_X)*CV_MAT_ELEM(*A_proj,double,2,2);

    CV_MAT_ELEM(*P,double,3,0)=CV_MAT_ELEM(*A_proj,double,1,0)-CV_MAT_ELEM(*proj_undist_points_mat,double,1,corresponding_Y*Projector_imagewidth+corresponding_X)*CV_MAT_ELEM(*A_proj,double,2,0);
    CV_MAT_ELEM(*P,double,3,1)=CV_MAT_ELEM(*A_proj,double,1,1)-CV_MAT_ELEM(*proj_undist_points_mat,double,1,corresponding_Y*Projector_imagewidth+corresponding_X)*CV_MAT_ELEM(*A_proj,double,2,1);
    CV_MAT_ELEM(*P,double,3,2)=CV_MAT_ELEM(*A_proj,double,1,2)-CV_MAT_ELEM(*proj_undist_points_mat,double,1,corresponding_Y*Projector_imagewidth+corresponding_X)*CV_MAT_ELEM(*A_proj,double,2,2);



    return;
}




void compute_F(int matrix_col,int matrix_row)
{

    CV_MAT_ELEM(*F,double,0,0)=CV_MAT_ELEM(*A_cam,double,2,3)*CV_MAT_ELEM(*cam_undist_points_mat,double,0,matrix_row*Camera_imagewidth+matrix_col)-CV_MAT_ELEM(*A_cam,double,0,3);
    CV_MAT_ELEM(*F,double,1,0)=CV_MAT_ELEM(*A_cam,double,2,3)*CV_MAT_ELEM(*cam_undist_points_mat,double,1,matrix_row*Camera_imagewidth+matrix_col)-CV_MAT_ELEM(*A_cam,double,1,3);

    corresponding_X=c_p_map[matrix_row*Camera_imagewidth+matrix_col][0];
    corresponding_Y=c_p_map[matrix_row*Camera_imagewidth+matrix_col][1];

    CV_MAT_ELEM(*F,double,2,0)=CV_MAT_ELEM(*A_proj,double,2,3)*CV_MAT_ELEM(*proj_undist_points_mat,double,0,corresponding_Y*Projector_imagewidth+corresponding_X)-CV_MAT_ELEM(*A_proj,double,0,3);
    CV_MAT_ELEM(*F,double,3,0)=CV_MAT_ELEM(*A_proj,double,2,3)*CV_MAT_ELEM(*proj_undist_points_mat,double,1,corresponding_Y*Projector_imagewidth+corresponding_X)-CV_MAT_ELEM(*A_proj,double,1,3);


    return;
}



static int initialization_counter=0;

void compute_X_Y_Z(int matrix_col,int matrix_row)
{

    ///V=(P_t*P)^-1*P_t*F
    cvTranspose(P,P_trans);
    cvMatMul(P_trans,P,I1);
    cvInvert(I1,I1);
    cvMatMul(I1,P_trans,I2);
    cvMatMul(I2,F,V);

//lets initialize the 'intersection_points' accordingly.
    intersection_points[matrix_col][matrix_row][0]=CV_MAT_ELEM(*V,double,0,0);
    intersection_points[matrix_col][matrix_row][1]=CV_MAT_ELEM(*V,double,1,0);
    intersection_points[matrix_col][matrix_row][2]=CV_MAT_ELEM(*V,double,2,0);

    /*  if(/*(intersection_points[matrix_col][matrix_row][2]>0.0)||*/ //(intersection_points[matrix_col][matrix_row][2]>-600.0))
    //valid_map[matrix_col][matrix_row]=0;


    return;
}




void compute_depth_method_3()
{
    //FILE*fd_intersection_points=fopen("../Dimension_tester/intersection_points.txt","w");
    //FILE*fd_valid_map=fopen("../Dimension_tester/valid_map.txt","w");

    compute_A();

    for(int i=0; i<Camera_imageheight; i++)
        for(int j=0; j<Camera_imagewidth; j++)
        {
            //fprintf(fd_valid_map,"%d\n",valid_map[j][i]);


            if(valid_map[j][i]==1)
            {

                compute_P(j,i);
                compute_F(j,i);
                compute_X_Y_Z(j,i);//In this procedure we will modify 'intersection_points' array so that this method is transparent to other modules.
                //fprintf(fd_intersection_points,"%lf\t%lf\t%lf\n",intersection_points[j][i][0],intersection_points[j][i][1],intersection_points[j][i][2]);


            }

        }
    /*

    double k1=0.0;
    double k2=0.0;

            for(int r=0;r<Camera_imageheight;r++)
            for(int c=0;c<Camera_imagewidth;c++)
            {
                if(valid_map[j][i]!=1) //i.e.,within selection region but a invalid point due to strips
                {

                  k1=sqrt()
                  //Do (X,Y,Z) interpolation
                  intersection_points[j][i][0]=k1*intersection_points[j-1][i][0]+k2*intersection_points[j+1][i][0];
                  intersection_points[j][i][1]=k1*intersection_points[j-1][i][1]+k2*intersection_points[j+1][i][1];
                  intersection_points[j][i][2]=k1*intersection_points[j-1][i][2]+k2*intersection_points[j+1][i][2];

                }

            }
    */




//Lets approximate the X,Y,Z for invalid region within point cloud(those points with extraordinary jumps in Z value wrt. neighbourhood)
    /*
        CvPoint start;

        CvPoint end;
        float f1;
        float f2;
        float t;
        float alpha,beta;

        for(int i=0; i<Camera_imageheight; i++)
            for(int j=0; j<Camera_imagewidth-1; j++)
                if(valid_map[j][i]==1)
                    if(fabs(intersection_points[j][i][2]-intersection_points[j+1][i][2])>5.0f)
                    {
    //Continue to find the end of this peak
                        start.x=j;
                        start.y=i;
                        j++;
                        while((j<Camera_imagewidth-1) && (valid_map[j][i]==1) && (fabs(intersection_points[j][i][2]-intersection_points[j+1][i][2])<5.0f))
                        {

                            j++;
                        }

                        if(!((j<Camera_imagewidth-1) && (valid_map[j][i]==1)))
                            break;


                        else
                        {

                            end.x=j+1;
                            end.y=i;


                            if((end.x-start.x)<5)
                                for(int c=start.x+1; c<end.x; c++)
                                {


                                    f1=sqrtf(powf((c-start.x),2)+powf((i-start.y),2));
                                    f2=sqrtf(powf((c-end.x),2)+powf((i-end.y),2));
                                    alpha=1.0f-f1;//more closer more weight.
                                    beta=1.0f-f2;

                                    t=alpha+beta;

                                    intersection_points[c][i][2]=(alpha*intersection_points[start.x][start.y][2]+beta*intersection_points[end.x][end.y][2])/t;
                                    intersection_points[c][i][0]=(alpha*intersection_points[start.x][start.y][0]+beta*intersection_points[end.x][end.y][0])/t;
                                    intersection_points[c][i][1]=(alpha*intersection_points[start.x][start.y][1]+beta*intersection_points[end.x][end.y][1])/t;
                                }
                            //valid_map[j+1][i]=3;
                        }


                    }


    ///FOR HORIZONTAL:

        for(int j=0; j<Camera_imagewidth; j++)
            for(int i=0; i<Camera_imageheight-1; i++)
                if(valid_map[j][i]==1)
                    if(fabs(intersection_points[j][i][2]-intersection_points[j][i+1][2])>5.0f)
                    {
    //Continue to find the end of this peak
                        start.x=j;
                        start.y=i;
                        i++;
                        while((i<Camera_imageheight-1) && (valid_map[j][i]==1) && (fabs(intersection_points[j][i][2]-intersection_points[j][i+1][2])<5.0f))
                        {

                            i++;
                        }

                        if(!((i<Camera_imageheight-1) && (valid_map[j][i]==1)))
                            break;


                        else
                        {

                            end.x=j;
                            end.y=i+1;
                            if((end.y-start.y)<5)
                                for(int c=start.y+1; c<end.y; c++)
                                {


                                    f1=sqrtf(powf((j-start.x),2)+powf((c-start.y),2));
                                    f2=sqrtf(powf((j-end.x),2)+powf((c-end.y),2));

                                    alpha=1.0f-f1;//more closer more weight.
                                    beta=1.0f-f2;

                                    t=alpha+beta;

                                    intersection_points[c][i][2]=(alpha*intersection_points[start.x][start.y][2]+beta*intersection_points[end.x][end.y][2])/t;
                                    intersection_points[c][i][0]=(alpha*intersection_points[start.x][start.y][0]+beta*intersection_points[end.x][end.y][0])/t;
                                    intersection_points[c][i][1]=(alpha*intersection_points[start.x][start.y][1]+beta*intersection_points[end.x][end.y][1])/t;
                                }
                            //valid_map[j+1][i]=3;
                        }
                    }

    */
    //fclose(fd_intersection_points);
    //fclose(fd_valid_map);

    return;
}









///Following function will plot a single column/row of 3D data-set as a representative of improvement of profile by changing the gamma value.
void plot_gamma_result()
{
    int row=-1;
    printf("\nEnter row number to inspect:");
    scanf("%d",&row);

    sprintf(filename,"/home/pranav/Desktop/M_tech_project_console/Gamma_correction/gamma_corrected_%d_%f.txt",row,1.0f);

    FILE*fd_gamma_correct_points=fopen(filename,"w");

    //In a loop:
    for(int col=0; col<Camera_imagewidth; col++)
    {
        if(valid_map[col][row]==1)
        {
            //Select a projector-image point
            //Determine its corresponding 3D coordinate
            //fprintf(fd_gamma_correct_points,"%lf\t%lf\t%lf\n",intersection_points[valid_camera_pixel[col][row][0]][valid_camera_pixel[col][row][1]][0],intersection_points[valid_camera_pixel[col][row][0]][valid_camera_pixel[col][row][1]][1],0.01*intersection_points[valid_camera_pixel[col][row][0]][valid_camera_pixel[col][row][1]][2]);
            fprintf(fd_gamma_correct_points,"%d\t%lf\n",col,intersection_points[col][row][2]);
            //Save it in file for a specific value of Gamma
        }

        //else
        //   fprintf(fd_gamma_correct_points,"%lf\t%lf\t%lf\n",0.0,0.0,0.0);//invalid points.
    }

    fclose(fd_gamma_correct_points);

    /*
        //Call scilab script to plot these values.
        char command[500];
        sprintf(command,"scilab -e exec\\(\\'Gamma_correction/gamma_plot.sci\\'\\)\\;exec\\(gamma_plot\\)\\;");
        system(command);
    */
    return;
}












//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////MAIN PROCEDURE!!
void triangulate()
{
    // fflush(stdin);
    // printf("Read matrices?");
    // fflush(stdout);

    // if(cvWaitKey(0)=='y')
    read_parameters();



    //else.

    read_relative_extrinsic_geometry();

    assign_3d_coordinates();
    transform_proj_coordinates();
    /*

       /// EXPERIMENTAL STEP:
       ///TO REMOVE MULTIPLE CAMERA POINTS MAPPING TO SAME PROJECTOR POINT.

       IplImage*exp_image1=cvLoadImage("Captured_patterns/Fringe_patterns/Vertical/Undistorted/Captured_image_0.bmp");

       IplImage*exp_image2=cvLoadImage("Captured_patterns/Fringe_patterns/Vertical/Undistorted/Captured_image_0.bmp");




       for(int i=0;i<Camera_imageheight;i++)
       for(int j=0;j<Camera_imagewidth;j++)
       {
           if(valid_map[j][i]==1)
           {
           for(int k=i;k<Camera_imageheight;k++)
           for(int h=j+1;h<Camera_imagewidth;h++)
           if(valid_map[h][k]==1)
           {
               if((c_p_map[j+i*Camera_imagewidth][0]==c_p_map[h+k*Camera_imagewidth][0]) && (c_p_map[j+i*Camera_imagewidth][1]==c_p_map[h+k*Camera_imagewidth][1]))
                {
                    valid_map[h][k]=0;//make the point invalid.
                    cvCircle(exp_image2,cvPoint(h,k),1,cvScalar(0,0,0));


                 }
           }
           }
       else
       {
           cvCircle(exp_image1,cvPoint(j,i),1,cvScalar(0,0,0));
           cvCircle(exp_image2,cvPoint(j,i),1,cvScalar(0,0,0));
        }

       }


    cvSaveImage("Experimental_original.bmp",exp_image1);
    cvSaveImage("Experimental_filtered.bmp",exp_image2);

    /// EXPERIMENT ENDS!!
    */
//Lets compute the depth,finally!!
/// METHOD-1:Source:http://en.wikipedia.org/wiki/Triangulation
    //compute_depth_method_1(c_p_map);

/// METHOD-2:Source:http://mesh.brown.edu/byo3d/
    //compute_depth_method_2();

/// METHOD-3:Source:Thesis:AUGMENTING COMPLEX SURFACES WITH PROJECTOR-CAMERA SYSTEMS
    intersection_points=new double[Camera_imagewidth][Camera_imageheight][3];
    compute_depth_method_3();


    //plot_gamma_result();

//    cvReleaseMat(&proj_cam_rot_mat);
//    cvReleaseMat(&proj_cam_trans_vect);


//Lets do deallocation...
    cvReleaseMat(&proj_COP);
    cvReleaseMat(&cam_COP);

    cvReleaseMat(&proj_pixel_3D);
    cvReleaseMat(&cam_pixel_3D);


    cvReleaseMat(&v1);
    cvReleaseMat(&v2);


    cvReleaseMat(&q1);
    cvReleaseMat(&q2);


    cvReleaseMat(&cam_undist_points_mat);
    cvReleaseMat(&proj_undist_points_mat);

    cvReleaseMat(&P);
    cvReleaseMat(&P_trans);
    cvReleaseMat(&F);
    cvReleaseMat(&A_cam);
    cvReleaseMat(&A_proj);
    cvReleaseMat(&V);
    cvReleaseMat(&I1);
    cvReleaseMat(&I2);


//    cvReleaseMat(&baseline_vector);

    // cvReleaseMat(&cam_optical_ray);
    // cvReleaseMat(&proj_optical_ray);


    return;


}

