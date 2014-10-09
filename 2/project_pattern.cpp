//TARGET:-This file will project the pattern & allow the camera to capture the project.
//NOTE:-Projector & user panel are in same X-Window.
//TO BE DONE:-Make provision for having projector on saparate X-screen.
/*Steps:-
1.Read the fringe images.
2.Move the window(to projector space).
3.In a loop:-
  a).Project the pattern on fullscreen.
  b).Capture it.
  c).Save the image.
  d).Ask for user to press 'N'(for next frame).

4.Done.
*/


///UPDATE 1:
/*
Undistorted images will be projected.
Captured images will be undistorted.
*/


#include "../PROJECT_GLOBAL/global_cv.h"
#include "../PROJECT_GLOBAL/intermodule_dependencies.h"

//used for moving the projector window.
extern char filename[150];
extern int number_of_pattern;
extern CvCapture*camera;
extern IplImage*cap;

extern CvMat*cam_intrinsic_mat;
extern CvMat*cam_dist_vect;

extern CvMat*proj_intrinsic_mat;
extern CvMat*proj_dist_vect;


extern int number_of_patterns_fringe;


extern int number_of_patterns_binary_horizontal;
extern int number_of_patterns_binary_vertical;

extern int cam_gain;
extern int proj_gain;


///////////////////////////////////////////////////////Global_variables.
IplImage**projected_image_set_fringe;

IplImage**projected_image_set_binary;

IplImage**projected_image_set_gray;

IplImage**projected_image_set_gray_inverse;


IplImage**captured_image_set_fringe;

IplImage**captured_image_set_binary;

IplImage**captured_image_set_gray;

IplImage**captured_image_set_gray_inverse;


//Following fucntion will read the patterns computed by the pattern generator module.
void read_pattern_images(int pattern_type)
{

    projected_image_set_fringe=new IplImage*[number_of_patterns_fringe];

//Vertical.
    if(pattern_type==0)
    {
        projected_image_set_binary=new IplImage*[number_of_patterns_binary_vertical+1];
        projected_image_set_gray=new IplImage*[number_of_patterns_binary_vertical+1];
        projected_image_set_gray_inverse=new IplImage*[number_of_patterns_binary_vertical+1];

        // Vertical fringe patterns...
        for(int i=0; i<number_of_patterns_fringe; i++)
        {

            sprintf(filename,"Generated_patterns/Fringe_patterns/Vertical/Pattern_%d.bmp",i);
            projected_image_set_fringe[i]=cvLoadImage(filename,CV_LOAD_IMAGE_GRAYSCALE);


        }

        //Vertical binary coded patterns...
        for(int k=0; k<number_of_patterns_binary_vertical+1; k++)
        {

            sprintf(filename,"Generated_patterns/Coded_patterns/Binary_coded/Vertical/Pattern_%d.bmp",k);
            projected_image_set_binary[k]=cvLoadImage(filename,CV_LOAD_IMAGE_GRAYSCALE);
        }

        //Vertical gray coded patterns...
        for(int k=0; k<number_of_patterns_binary_vertical+1; k++)
        {

            sprintf(filename,"Generated_patterns/Coded_patterns/Gray_coded/Vertical/Pattern_%d.bmp",k);
            projected_image_set_gray[k]=cvLoadImage(filename,CV_LOAD_IMAGE_GRAYSCALE);

            sprintf(filename,"Generated_patterns/Coded_patterns/Gray_coded/Vertical/inverse_Pattern_%d.bmp",k);
            projected_image_set_gray_inverse[k]=cvLoadImage(filename,CV_LOAD_IMAGE_GRAYSCALE);



        }

    }


//Horizontal.
    if(pattern_type==1)
    {
        projected_image_set_binary=new IplImage*[number_of_patterns_binary_horizontal+1];
        projected_image_set_gray=new IplImage*[number_of_patterns_binary_horizontal+1];


        // Horizontal fringe patterns...
        for(int j=0; j<number_of_patterns_fringe; j++)
        {

            sprintf(filename,"Generated_patterns/Fringe_patterns/Horizontal/Pattern_%d.bmp",j);
            projected_image_set_fringe[j]=cvLoadImage(filename,CV_LOAD_IMAGE_GRAYSCALE);
        }

        // Horizontal binary coded patterns...
        for(int l=0; l<number_of_patterns_binary_horizontal+1; l++)
        {

            sprintf(filename,"Generated_patterns/Coded_patterns/Binary_coded/Horizontal/Pattern_%d.bmp",l);
            projected_image_set_binary[l]=cvLoadImage(filename,CV_LOAD_IMAGE_GRAYSCALE);

        }

        // Horizontal gray coded patterns...
        for(int l=0; l<number_of_patterns_binary_horizontal+1; l++)
        {
            sprintf(filename,"Generated_patterns/Coded_patterns/Gray_coded/Horizontal/Pattern_%d.bmp",l);
            projected_image_set_gray[l]=cvLoadImage(filename,CV_LOAD_IMAGE_GRAYSCALE);

            sprintf(filename,"Generated_patterns/Coded_patterns/Gray_coded/Horizontal/inverse_Pattern_%d.bmp",l);
            projected_image_set_gray_inverse[l]=cvLoadImage(filename,CV_LOAD_IMAGE_GRAYSCALE);
        }

    }


    return;
}



//This function will project the pattern using projector.
void project_pattern(int pattern_type)
{

    char pattern_name[15];

    //cap=cvQueryFrame(camera);//null frame.(CORRECT THIS FUNCTION:PENDING ASSIGNMENT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!)


    //i want live preview & capture.
    bool capture_next=false;

    IplImage*temp_proj_pattern=cvCreateImage(cvSize(Projector_imagewidth,Projector_imageheight),8,1);

    IplImage*undist_cap=cvCreateImage(cvSize(Camera_imagewidth,Camera_imageheight),IPL_DEPTH_8U,3);

    if(pattern_type==0)
        sprintf(pattern_name,"vertical");

    else
        sprintf(pattern_name,"horizontal");






    printf("\nProjecting %s phase shifted fringe patterns...\n",pattern_name);


    /// Project & capture fringe patterns...
    for(int i=0; i<number_of_patterns_fringe; i++)
    {
        cvUndistort2(projected_image_set_fringe[i],temp_proj_pattern,proj_intrinsic_mat,proj_dist_vect);

        //while(capture_next!=true)
        // {
        cvScale(projected_image_set_fringe[i], projected_image_set_fringe[i], 2.*(proj_gain/100.), 0);
        cvShowImage("Projector_pattern",projected_image_set_fringe[i]);
        cvCopyImage(temp_proj_pattern,projected_image_set_fringe[i]);//for next iteration.

        //cap=cvQueryFrame(camera);
        //mainloop(cap);
        preview_and_capture();
        //cvScale(cap, cap, 2.*(cam_gain/100.), 0);

        //cvShowImage("Camera_view",cap);

        //if(cvWaitKey(30)=='c')
        //{
        // capture_next=true;

        if(pattern_type==0)//vertical.
        {
            sprintf(filename,"Captured_patterns/Fringe_patterns/Vertical/Original/Captured_image_%d.bmp",i);

            //cap=camera_capture(filename);
            //cap=cvQueryFrame(camera);
            //mainloop(cap);
            //cvWaitKey(30);

            cvUndistort2(cap,undist_cap,cam_intrinsic_mat,cam_dist_vect);
            sprintf(filename,"Captured_patterns/Fringe_patterns/Vertical/Undistorted/Captured_image_%d.bmp",i);
            cvSaveImage(filename,undist_cap);

        }

        else //horizontal
        {
            sprintf(filename,"Captured_patterns/Fringe_patterns/Horizontal/Original/Captured_image_%d.bmp",i);
            //cap=camera_capture(filename);
            //cap=cvQueryFrame(camera);
            // mainloop(cap);
            cvUndistort2(cap,undist_cap,cam_intrinsic_mat,cam_dist_vect);
            sprintf(filename,"Captured_patterns/Fringe_patterns/Horizontal/Undistorted/Captured_image_%d.bmp",i);
            cvSaveImage(filename,undist_cap);
        }
        //  }


        //else
        //   capture_next=false;//try again user still not satisfied.
        // }

        //capture_next=false;//for the next loop.
        printf("\nCaptured %d of %d frame!!\n",i+1,number_of_patterns_fringe);
        cvReleaseImage(&cap);
    }


    /*

        printf("Now projecting %s binary coded patterns...\n",pattern_name);


        /// Project and capture binary coded patterns...
        if(pattern_type==0)
        {

            for(int j=0; j<number_of_patterns_binary_vertical+1; j++)
            {

                cvUndistort2(projected_image_set_binary[j],temp_proj_pattern,proj_intrinsic_mat,proj_dist_vect);
                //while(capture_next!=true)
                //  {
                cvScale(projected_image_set_binary[j], projected_image_set_binary[j], 2.*(proj_gain/100.), 0);
                cvShowImage("Projector_pattern",projected_image_set_binary[j]);
                cvCopyImage(temp_proj_pattern,projected_image_set_binary[j]);


                //cap=cvQueryFrame(camera);
                //mainloop(cap);

                //cvScale(cap, cap, 2.*(cam_gain/100.), 0);


                preview_and_capture();

                // cvShowImage("Camera_view",cap);

                //  if(cvWaitKey(30)=='c')
                //   {
                //  capture_next=true;

                sprintf(filename,"Captured_patterns/Coded_patterns/Binary_coded/Vertical/Original/Captured_image_%d.bmp",j);

                //cap=cvQueryFrame(camera);
                // mainloop(cap);
                cvUndistort2(cap,undist_cap,cam_intrinsic_mat,cam_dist_vect);
                sprintf(filename,"Captured_patterns/Coded_patterns/Binary_coded/Vertical/Undistorted/Captured_image_%d.bmp",j);
                cvSaveImage(filename,undist_cap);
                //  }

                //   else
                //    capture_next=false;
                // }


                // capture_next=false;
                printf("\nCaptured %d of %d frame!!\n",j+1,number_of_patterns_binary_vertical);
                cvReleaseImage(&cap);
            }

        }


        if(pattern_type==1)
        {

            for(int j=0; j<number_of_patterns_binary_horizontal+1; j++)
            {

                cvUndistort2(projected_image_set_binary[j],temp_proj_pattern,proj_intrinsic_mat,proj_dist_vect);
                //  while(capture_next!=true)
                //   {
                cvScale(projected_image_set_binary[j], projected_image_set_binary[j], 2.*(proj_gain/100.), 0);
                cvShowImage("Projector_pattern",projected_image_set_binary[j]);
                cvCopyImage(temp_proj_pattern,projected_image_set_binary[j]);

                //cap=cvQueryFrame(camera);
                //      mainloop(cap);
                //cvScale(cap, cap, 2.*(cam_gain/100.), 0);
                preview_and_capture();
                //cvShowImage("Camera_view",cap);



                // if(cvWaitKey(30)=='c')
                //    {
                //      capture_next=true;



                sprintf(filename,"Captured_patterns/Coded_patterns/Binary_coded/Horizontal/Original/Captured_image_%d.bmp",j);

                //cap=cvQueryFrame(camera);
                //mainloop(cap);
                cvUndistort2(cap,undist_cap,cam_intrinsic_mat,cam_dist_vect);
                sprintf(filename,"Captured_patterns/Coded_patterns/Binary_coded/Horizontal/Undistorted/Captured_image_%d.bmp",j);
                cvSaveImage(filename,undist_cap);

                // }


                //   else
                //     capture_next=false;
                // }

                //capture_next=false;
                printf("\nCaptured %d of %d frame!!\n",j+1,number_of_patterns_binary_horizontal);
                cvReleaseImage(&cap);
            }

        }

    */

    /// Project & capture gray coded patterns...
    if(pattern_type==0)
    {
        for(int j=0; j<number_of_patterns_binary_vertical+1; j++)
        {
            ///Gray coded
            cvUndistort2(projected_image_set_gray[j],temp_proj_pattern,proj_intrinsic_mat,proj_dist_vect);

            cvScale(projected_image_set_gray[j], projected_image_set_gray[j], 2.*(proj_gain/100.), 0);
            cvShowImage("Projector_pattern",projected_image_set_gray[j]);
            cvCopyImage(temp_proj_pattern,projected_image_set_gray[j]);

            preview_and_capture();

            cvUndistort2(cap,undist_cap,cam_intrinsic_mat,cam_dist_vect);
            sprintf(filename,"Captured_patterns/Coded_patterns/Gray_coded/Vertical/Undistorted/Captured_image_%d.bmp",j);
            cvSaveImage(filename,undist_cap);

            ///Inverse gray coded
            cvUndistort2(projected_image_set_gray_inverse[j],temp_proj_pattern,proj_intrinsic_mat,proj_dist_vect);
            cvScale(projected_image_set_gray_inverse[j], projected_image_set_gray_inverse[j], 2.*(proj_gain/100.), 0);
            cvShowImage("Projector_pattern",projected_image_set_gray_inverse[j]);
            cvCopyImage(temp_proj_pattern,projected_image_set_gray_inverse[j]);

            preview_and_capture();

            cvUndistort2(cap,undist_cap,cam_intrinsic_mat,cam_dist_vect);
            sprintf(filename,"Captured_patterns/Coded_patterns/Gray_coded/Vertical/Undistorted/inverse_Captured_image_%d.bmp",j);
            cvSaveImage(filename,undist_cap);


            printf("\nCaptured %d of %d frame!!\n",j+1,number_of_patterns_binary_vertical);
            cvReleaseImage(&cap);
        }

    }


    if(pattern_type==1)
    {

        for(int j=0; j<number_of_patterns_binary_horizontal+1; j++)
        {
            ///Gray coded
            cvUndistort2(projected_image_set_gray[j],temp_proj_pattern,proj_intrinsic_mat,proj_dist_vect);

            cvScale(projected_image_set_gray[j], projected_image_set_gray[j], 2.*(proj_gain/100.), 0);
            cvShowImage("Projector_pattern",projected_image_set_gray[j]);
            cvCopyImage(temp_proj_pattern,projected_image_set_gray[j]);

            preview_and_capture();

            sprintf(filename,"Captured_patterns/Coded_patterns/Gray_coded/Horizontal/Original/Captured_image_%d.bmp",j);

            cvUndistort2(cap,undist_cap,cam_intrinsic_mat,cam_dist_vect);
            sprintf(filename,"Captured_patterns/Coded_patterns/Gray_coded/Horizontal/Undistorted/Captured_image_%d.bmp",j);
            cvSaveImage(filename,undist_cap);



            ///Inverse gray coded
            cvUndistort2(projected_image_set_gray_inverse[j],temp_proj_pattern,proj_intrinsic_mat,proj_dist_vect);

            cvScale(projected_image_set_gray_inverse[j], projected_image_set_gray_inverse[j], 2.*(proj_gain/100.), 0);
            cvShowImage("Projector_pattern",projected_image_set_gray_inverse[j]);
            cvCopyImage(temp_proj_pattern,projected_image_set_gray_inverse[j]);

            preview_and_capture();

            cvUndistort2(cap,undist_cap,cam_intrinsic_mat,cam_dist_vect);
            sprintf(filename,"Captured_patterns/Coded_patterns/Gray_coded/Horizontal/Undistorted/inverse_Captured_image_%d.bmp",j);
            cvSaveImage(filename,undist_cap);

            printf("\nCaptured %d of %d frame!!\n",j+1,number_of_patterns_binary_horizontal);
            cvReleaseImage(&cap);


        }

    }

    return;
}



/// EXTERNAL INTERFACE
void project_and_capture_pattern(int pattern_type)
{
    read_pattern_images(pattern_type);
    project_pattern(pattern_type);

    return;
}




