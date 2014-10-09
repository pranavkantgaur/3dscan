/*
TARGET:
For camera calibration,compare the reprojection accuracy of multiple calibration parameter values used.
For projector calibration,compare the reprojection accuracy multiple calibration parameter values(from OpenCV & VPCLib)
*/
#include"../PROJECT_GLOBAL/global_cv.h"


IplImage*test_patten;
IplImage*cam_test_image;
IplImage*proj_test_image;
IplImage*cam_undist_test_image;
IplImage*proj_undist_test_image;
IplImage*undist_test_pattern;

CvMat*test_points_3d;
CvMat*cam_test_points_2d;
CvMat*proj_test_points_2d;

extern CvCapture*camera;

extern CvMat*cam_intrinsic_mat;
extern CvMat*cam_dist_vect;
extern CvMat*cam_world_rot_mat;
extern CvMat*cam_world_trans_vect;

extern CvMat*proj_intrinsic_mat;
extern CvMat*proj_dist_vect;
extern CvMat*proj_world_rot_mat;
extern CvMat*proj_world_trans_vect;

/// Generates/reads image.
void generate_test_image()
{
//A '+' feature at all corners & image center.
    test_patten=cvCreateImage(cvSize(Projector_imagewidth,Projector_imageheight),IPL_DEPTH_8U,1);
    cvSet(test_patten,cvScalar(255));
    IplImage*undist_test_pattern=cvCreateImage(cvSize(Projector_imagewidth,Projector_imageheight),IPL_DEPTH_8U,1);

    /*
    //Draw pattern
    //"+" on top-left.
    test_patten->imageData[2]=255;
    test_patten->imageData[test_patten->widthStep+2]=255;
    test_patten->imageData[test_patten->widthStep*2]=255;
    test_patten->imageData[test_patten->widthStep*2+1]=255;
    test_patten->imageData[test_patten->widthStep*2+2]=255;
    test_patten->imageData[test_patten->widthStep*2+3]=255;
    test_patten->imageData[test_patten->widthStep*2+4]=255;
    test_patten->imageData[test_patten->widthStep*3+2]=255;
    test_patten->imageData[test_patten->widthStep*4+2]=255;

    //"+" on top-right.
    test_patten->imageData[1021]=255;
    test_patten->imageData[test_patten->widthStep+1021]=255;
    test_patten->imageData[test_patten->widthStep*2+1019]=255;
    test_patten->imageData[test_patten->widthStep*2+1020]=255;
    test_patten->imageData[test_patten->widthStep*2+1021]=255;
    test_patten->imageData[test_patten->widthStep*2+1022]=255;
    test_patten->imageData[test_patten->widthStep*2+1023]=255;
    test_patten->imageData[test_patten->widthStep*3+1021]=255;
    test_patten->imageData[test_patten->widthStep*4+1021]=255;

    //"+" on bottom-left
    test_patten->imageData[763*test_patten->widthStep+2]=255;
    test_patten->imageData[764*test_patten->widthStep+2]=255;
    test_patten->imageData[765*test_patten->widthStep]=255;
    test_patten->imageData[765*test_patten->widthStep+1]=255;
    test_patten->imageData[765*test_patten->widthStep+2]=255;
    test_patten->imageData[765*test_patten->widthStep+3]=255;
    test_patten->imageData[765*test_patten->widthStep+4]=255;
    test_patten->imageData[766*test_patten->widthStep+2]=255;
    test_patten->imageData[767*test_patten->widthStep+2]=255;

    //"+" on bottom-right
    test_patten->imageData[763*test_patten->widthStep+1021]=255;
    test_patten->imageData[764*test_patten->widthStep+1021]=255;
    test_patten->imageData[765*test_patten->widthStep+1019]=255;
    test_patten->imageData[765*test_patten->widthStep+1020]=255;
    test_patten->imageData[765*test_patten->widthStep+1021]=255;
    test_patten->imageData[765*test_patten->widthStep+1022]=255;
    test_patten->imageData[765*test_patten->widthStep+1023]=255;
    test_patten->imageData[766*test_patten->widthStep+1021]=255;
    test_patten->imageData[767*test_patten->widthStep+1021]=255;


    //"+" in image center
    test_patten->imageData[381*test_patten->widthStep+512]=255;
    test_patten->imageData[382*test_patten->widthStep+512]=255;
    test_patten->imageData[383*test_patten->widthStep+510]=255;
    test_patten->imageData[383*test_patten->widthStep+511]=255;
    test_patten->imageData[383*test_patten->widthStep+512]=255;
    test_patten->imageData[383*test_patten->widthStep+513]=255;
    test_patten->imageData[383*test_patten->widthStep+514]=255;
    test_patten->imageData[384*test_patten->widthStep+512]=255;
    test_patten->imageData[385*test_patten->widthStep+512]=255;
    */

    cvUndistort2(test_patten,undist_test_pattern,proj_intrinsic_mat,proj_dist_vect);

    cvSaveImage("Calibration_error/Projector_calibration/Projected_test_image.bmp",undist_test_pattern);

    return;
}


/// Projects test image.
void project_test_image()
{


    cvShowImage("Projector_pattern",undist_test_pattern);
//Now lets capture the projected image.
    printf("\nCapture test pattern?");
    fflush(stdout);

    bool capture_image=false;

    while(capture_image!=true)
    {
        cvShowImage("Camera_view",cvQueryFrame(camera));
        if(cvWaitKey(30)=='c')
            capture_image=true;
    }

    IplImage*undist_captured_image=cvCreateImage(cvSize(Camera_imagewidth,Camera_imageheight),IPL_DEPTH_8U,3);

    cvUndistort2(cvQueryFrame(camera),undist_captured_image,cam_intrinsic_mat,cam_dist_vect);

    cvSaveImage("Calibration_error/Camera_calibration/Captured_test_image.bmp",undist_captured_image);

    return;
}

/// Takes 3d coordinates from user for corners features.
void take_3d_feature_coordinates()
{
    test_points_3d=cvCreateMat(4,4,CV_64FC1);

    printf("Enter 3d coordinates for Top-Left marker:");
    scanf("%lf%lf",&CV_MAT_ELEM(*test_points_3d,double,0,0),&CV_MAT_ELEM(*test_points_3d,double,1,0));
    CV_MAT_ELEM(*test_points_3d,double,2,0)=0.0f;
    CV_MAT_ELEM(*test_points_3d,double,3,0)=1.0f;

    printf("Enter 3d coordinates for Top-right marker:");
    scanf("%lf%lf",&CV_MAT_ELEM(*test_points_3d,double,0,1),&CV_MAT_ELEM(*test_points_3d,double,1,1));
    CV_MAT_ELEM(*test_points_3d,double,2,1)=0.0f;
    CV_MAT_ELEM(*test_points_3d,double,3,1)=1.0f;

    printf("Enter 3d coordinates for Bottom-left marker:");
    scanf("%lf%lf",&CV_MAT_ELEM(*test_points_3d,double,0,2),&CV_MAT_ELEM(*test_points_3d,double,1,2));
    CV_MAT_ELEM(*test_points_3d,double,2,2)=0.0f;
    CV_MAT_ELEM(*test_points_3d,double,3,2)=1.0f;

    printf("Enter 3d coordinates for Bottom-right marker:");
    scanf("%lf%lf",&CV_MAT_ELEM(*test_points_3d,double,0,3),&CV_MAT_ELEM(*test_points_3d,double,1,3));
    CV_MAT_ELEM(*test_points_3d,double,2,3)=0.0f;
    CV_MAT_ELEM(*test_points_3d,double,3,3)=1.0f;


    return;
}




/// Reprojects 3d coordinates from 3D space to camera & projector image.
void reproject_3d_coordinates_onto_image()
{

    //Reprojecting on camera image:
    //Mapping the input 3d points to undistorted 2d camera space.
    CvMat*cam_projective_matrix=cvCreateMat(3,4,CV_64FC1);
    CvMat*proj_projective_matrix=cvCreateMat(3,4,CV_64FC1);

    cam_test_points_2d=cvCreateMat(3,4,CV_64FC1);
    proj_test_points_2d=cvCreateMat(3,4,CV_64FC1);

    /***********************************CAMERA*****************************************************/
    for(int r=0; r<3; r++)
        for(int p=0; p<3; p++)
            CV_MAT_ELEM(*cam_projective_matrix,double,r,p)=CV_MAT_ELEM(*cam_world_rot_mat,double,r,p);

    for(int r=0; r<3; r++)
        CV_MAT_ELEM(*cam_projective_matrix,double,r,3)=CV_MAT_ELEM(*cam_world_trans_vect,double,r,0);

    cvMatMul(cam_projective_matrix,test_points_3d,cam_test_points_2d);

    cvMatMul(cam_intrinsic_mat,cam_test_points_2d,cam_test_points_2d);

    printf("\nPrinting reprojected camera coordinates...\n");
    //Lets homogenize the points:
    for(int n=0; n<4; n++)
    {
        for(int r=0; r<3; r++)
            printf("%lf\t",CV_MAT_ELEM(*cam_test_points_2d,double,r,n)/=CV_MAT_ELEM(*cam_test_points_2d,double,2,n));

        printf("\n");
    }


    //Draw on image:
    cam_test_image=cvLoadImage("Calibration_error/Camera_calibration/Captured_test_image.bmp");




    CvPoint *cam_points=new CvPoint[4];

    for(int r=0; r<4; r++)
    {
        cam_points[r].x=floor(CV_MAT_ELEM(*cam_test_points_2d,double,0,r));
        cam_points[r].y=floor(CV_MAT_ELEM(*cam_test_points_2d,double,1,r));
        cvCircle(cam_test_image,cam_points[r],5,cvScalar(255,0,0));
    }



    cvSave("Calibration_error/cam_reprojected_coordinates.xml",cam_test_points_2d);


    /***********************PROJECTOR************************************************************/
    //Reprojecting on projector image:
    //Mapping the input 3d points to undistorted 2d projector space.
    for(int r=0; r<3; r++)
        for(int p=0; p<3; p++)
            CV_MAT_ELEM(*proj_projective_matrix,double,r,p)=CV_MAT_ELEM(*proj_world_rot_mat,double,r,p);

    for(int r=0; r<3; r++)
        CV_MAT_ELEM(*proj_projective_matrix,double,r,3)=CV_MAT_ELEM(*proj_world_trans_vect,double,r,0);

    cvMatMul(proj_projective_matrix,test_points_3d,proj_test_points_2d);

    cvMatMul(proj_intrinsic_mat,proj_test_points_2d,proj_test_points_2d);


    printf("\nPrinting reprojected Projector coordinates ...\n");
    //Lets homogenize the points:
    for(int n=0; n<4; n++)
    {
        for(int r=0; r<3; r++)
            printf("%lf\t",CV_MAT_ELEM(*proj_test_points_2d,double,r,n)/=CV_MAT_ELEM(*proj_test_points_2d,double,2,n));

        printf("\n");
    }

    //Draw on image:
    proj_test_image=cvLoadImage("Calibration_error/Projector_calibration/Projected_test_image.bmp");
    proj_undist_test_image=cvCloneImage(proj_test_image);



    CvPoint*proj_points=new CvPoint[4];

    for(int r=0; r<4; r++)
    {
        proj_points[r].x=floor(CV_MAT_ELEM(*proj_test_points_2d,double,0,r));
        proj_points[r].y=floor(CV_MAT_ELEM(*proj_test_points_2d,double,1,r));
        cvCircle(proj_test_image,proj_points[r],5,cvScalar(255,0,0));
    }

    cvSave("Calibration_error/proj_reprojected_coordinates.xml",proj_test_points_2d);


    return;
}


/// Save images.
void save_images()
{
    /***Camera***/
    cvSaveImage("Calibration_error/cam_3d_points_projected.bmp",cam_test_image);
    /***Projector***/
    cvSaveImage("Calibration_error/proj_3d_points_projected.bmp",proj_test_image);
    return;
}

/// Main function.
void measure_calibration_errors()
{
    generate_test_image();
    project_test_image();
    take_3d_feature_coordinates();
    reproject_3d_coordinates_onto_image();
    save_images();

    return;
}
