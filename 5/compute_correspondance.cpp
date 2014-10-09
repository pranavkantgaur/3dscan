#include"global_cv.h"
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include<cmath>
#include  <signal.h>

#define invalid_unwrapped_phase_value 1000.0
#define virtual_checkerboard_X 10
#define virtual_checkerboard_Y 8
#define square_pixels 75   //side length of virtual checkerboard(in pixels)

extern long int (*c_p_map)[2];
extern int (*code_vertical)[Camera_imageheight];
extern int (*code_horizontal)[Camera_imageheight];
extern int fringe_width_pixels_vertical;
extern int fringe_width_pixels_horizontal;
extern float (*unwrapped_phi_vertical)[Camera_imageheight];
extern float (*unwrapped_phi_horizontal)[Camera_imageheight];

extern int (*valid_map_vertical)[Camera_imageheight];
extern int (*valid_map_horizontal)[Camera_imageheight];
extern int (*valid_map)[Camera_imageheight];

extern char filename[150];
extern int number_of_patterns_fringe;
extern int proj_gain;
//double unwrapped_phase_proj_vert[Projector_imagewidth][Projector_imageheight];
//double unwrapped_phase_proj_horz[Projector_imagewidth][Projector_imageheight];

//extern bool assigned[Projector_imagewidth][Projector_imageheight];//Projector pixels with '1' are the only valid pixels.
//extern int valid_camera_pixel[Projector_imagewidth][Projector_imageheight][2];
extern float gamma_value;

extern CvCapture*camera;
extern IplImage*cap;
//Lets develop a correspondance-checker function:
//It will allow the user to click on a point on the captured image and will show corresponding point in the projector image.

//Lets load a sample captured & corresponding projected image..

IplImage*camera_image;

IplImage*projector_image;


IplImage*temp_projector_image;

IplImage*temp_camera_image;




//double unwrapped_phase_error_vertical[Projector_imagewidth][Projector_imageheight];
//double unwrapped_phase_error_horizontal[Projector_imagewidth][Projector_imageheight];




void merge_valid_maps()
{


    for(int r=0; r<Camera_imageheight; r++)
        for(int c=0; c<Camera_imagewidth; c++)
            if((valid_map_vertical[c][r]==1) && (valid_map_horizontal[c][r]==1))
            {
                valid_map[c][r]=1;

            }

            else
                valid_map[c][r]=0;


    return;
}








CvPoint*point1=new CvPoint;//stores the pixel coordinates of selected point.
bool point_recorded1=false;

void callback_record_pixel1(int event,int x,int y,int flags,void* param)
{
    switch(event)
    {
    case CV_EVENT_LBUTTONDOWN:
        point1->x=x;
        point1->y=y;
        break;


    case CV_EVENT_LBUTTONUP:
        cvCopy(projector_image,temp_projector_image);

        point_recorded1=true;
        /*for(int i=0; i<Camera_imageheight; i++)
            for(int j=0; j<Camera_imagewidth; j++)
                if(valid_map[j][i]==2)
                {
                    if((c_p_map[i*Camera_imagewidth+j][0]==x) && (c_p_map[i*Camera_imagewidth+j][1]==y))
                        cvCircle(temp_camera_image,cvPoint(j,i),10,cvScalar(255,0,0));

                }
            */

        cvCircle(temp_projector_image,cvPoint(c_p_map[point1->y*Camera_imagewidth+point1->x][0],c_p_map[point1->y*Camera_imagewidth+point1->x][1]),5,cvScalar(0,255,0),-1);


    }


    return;
}








void check_correspondance()
{
    printf("\nStarting correspondance checker....\n");

    camera_image=cvLoadImage("/home/pranav/Desktop/M_tech_project_console/Captured_patterns/Fringe_patterns/Vertical/Original/Captured_image_0.jpg");
    projector_image=cvLoadImage("/home/pranav/Desktop/M_tech_project_console/Generated_patterns/Fringe_patterns/Vertical/Pattern_0.jpg");
    temp_projector_image=cvLoadImage("/home/pranav/Desktop/M_tech_project_console/Generated_patterns/Fringe_patterns/Vertical/Pattern_0.jpg");
    temp_camera_image=cvCreateImage(cvGetSize(camera_image),camera_image->depth,camera_image->nChannels);





    cvNamedWindow("Click on a point:",0);

    cvShowImage("Click on a point:",camera_image);
    cvNamedWindow("Corresponding points:",0);
    cvSetMouseCallback("Click on a point:",callback_record_pixel1,(void*)NULL);


    while(1)//sit inside a loop to wait for the user to click on a point.
    {
        cvShowImage("Click on a point:",camera_image);
        if(cvWaitKey(30)==27||point_recorded1==true)
        {

            point_recorded1=false;
            cvShowImage("Corresponding points:",temp_projector_image);
            printf("Want another check?\n");
            fflush(stdout);
            if(cvWaitKey(0)=='y')
                continue;
            else
                break;
        }
    }
    return;



}








/*
//Transform unwrapped phase from camera to projector space.
void map_unwrapped_phase_to_projector_space()
{


    ///Technique:
    /* 1.Create a array 'assigned[proj_width][proj_height]' which will contain bool unwrapped phase assigned/not assigned
     2.For each camera pixel get corresponding projector pixel,if not already assigned,set unwrapped_phase_proj=unwrapped_phase_camera
     3.If multiple assignments exists discard that projector pixel.
     */
/*
    int proj_col,proj_row;

    for(int i=0; i<Projector_imageheight; i++)
        for(int j=0; j<Projector_imagewidth; j++)
        {
            assigned[j][i]=false;

        }

    for(int r=0; r<Projector_imageheight; r++)
        for(int c=0; c<Projector_imagewidth; c++)
        {
            valid_camera_pixel[c][r][0]=-1; //invalid camera row & column values.
            valid_camera_pixel[c][r][1]=-1;
        }


    for(int r=0; r<Camera_imageheight; r++)
        for(int c=0; c<Camera_imagewidth; c++)
            if(valid_map[c][r]==1)
            {

                proj_col=c_p_map[r*Camera_imagewidth+c][0];
                proj_row=c_p_map[r*Camera_imagewidth+c][1];

                if(assigned[proj_col][proj_row]==false)//assign unwrapped phase.
                {

                    assigned[proj_col][proj_row]=true;//to avoid reassignment.
                    unwrapped_phase_proj_vert[proj_col][proj_row]=unwrapped_phi_vertical[c][r];
                    unwrapped_phase_proj_horz[proj_col][proj_row]=unwrapped_phi_horizontal[c][r];
                    valid_camera_pixel[proj_col][proj_row][0]=c;
                    valid_camera_pixel[proj_col][proj_row][1]=r;
                }

                else //if already assigned
                {
                    unwrapped_phase_proj_vert[proj_col][proj_row]=invalid_unwrapped_phase_value;
                    unwrapped_phase_proj_horz[proj_col][proj_row]=invalid_unwrapped_phase_value;
                    assigned[proj_col][proj_row]=false; //discarded from future considerations.
                    valid_camera_pixel[proj_col][proj_row][0]=-1;
                    valid_camera_pixel[proj_col][proj_row][1]=-1;
                }
            }


    return;
}
*/

///NOTE:Pixels with assigned=1 are valid.


/*
//Compute error map with computed-original unwrapped phase.
void compute_error_map()
{
    /*
        double ideal_vertical_unwrapped_phase[Projector_imagewidth][Projector_imageheight];
        double ideal_horizontal_unwrapped_phase[Projector_imagewidth][Projector_imageheight];

        //Initializing ideal unwrapped phase at projector coordinate system.
        int a=0;
        for(int r=0; r<Projector_imageheight; r++)
            for(int c=0; c<Projector_imagewidth; c++)
            {
                if(assigned[c][r]==true)
                {
                    a=c/fringe_width_pixels_vertical;
                    ideal_vertical_unwrapped_phase[c][r]=a*2.0*Pi+(c%fringe_width_pixels_vertical)*(2.0*Pi/fringe_width_pixels_vertical);

                    a=r/fringe_width_pixels_horizontal;
                    ideal_horizontal_unwrapped_phase[c][r]=a*2.0*Pi+(r%fringe_width_pixels_horizontal)*(2.0*Pi/fringe_width_pixels_horizontal);

                }
            }

    */
/*
    int a=0;
    for(int r=0; r<Projector_imageheight; r++)
        for(int c=0; c<Projector_imagewidth; c++)
        {
            if(assigned[c][r]==true)
            {
                a=c/fringe_width_pixels_vertical;
                unwrapped_phase_error_vertical[c][r]=-1.0*(a*2.0*Pi+(c%fringe_width_pixels_vertical)*(2.0*Pi/fringe_width_pixels_vertical)-unwrapped_phase_proj_vert[c][r]);

                a=r/fringe_width_pixels_horizontal;
                unwrapped_phase_error_horizontal[c][r]=-1.0*(a*2.0*Pi+(r%fringe_width_pixels_horizontal)*(2.0*Pi/fringe_width_pixels_horizontal)-unwrapped_phase_proj_horz[c][r]);
            }
        }

    IplImage*unwrapped_phase_vertical_proj_image=cvCreateImage(cvSize(Projector_imagewidth,Projector_imageheight),IPL_DEPTH_8U,1);
    IplImage*unwrapped_phase_horizontal_proj_image=cvCreateImage(cvSize(Projector_imagewidth,Projector_imageheight),IPL_DEPTH_8U,1);

    cvSet(unwrapped_phase_vertical_proj_image,cvScalar(0));//black out
    cvSet(unwrapped_phase_horizontal_proj_image,cvScalar(0));

    for(int r=0; r<Projector_imageheight; r++)
        for(int c=0; c<Projector_imagewidth; c++)
        {
            if(assigned[c][r]==true)
            {
                a=c/fringe_width_pixels_vertical;
                unwrapped_phase_vertical_proj_image->imageData[r*unwrapped_phase_vertical_proj_image->widthStep+c]=(unsigned char)(a*2.0*Pi+(c%fringe_width_pixels_vertical)*(2.0*Pi/fringe_width_pixels_vertical));

                a=r/fringe_width_pixels_horizontal;
                unwrapped_phase_horizontal_proj_image->imageData[r*unwrapped_phase_horizontal_proj_image->widthStep+c]=(unsigned char)(a*2.0*Pi+(r%fringe_width_pixels_horizontal)*(2.0*Pi/fringe_width_pixels_horizontal));
            }
        }

    cvSaveImage("Correspondence_error/ideal_vertical_unwrapped_phase.jpg",unwrapped_phase_vertical_proj_image);
    cvSaveImage("Correspondence_error/ideal_horizontal_unwrapped_phase.jpg",unwrapped_phase_horizontal_proj_image);


//reusing images to store the mapped unwrapped phase also.
    cvSet(unwrapped_phase_vertical_proj_image,cvScalar(0));
    cvSet(unwrapped_phase_horizontal_proj_image,cvScalar(0));




    for(int r=0; r<Projector_imageheight; r++)
        for(int c=0; c<Projector_imagewidth; c++)
        {
            if(assigned[c][r]==true)
            {
                unwrapped_phase_vertical_proj_image->imageData[r*unwrapped_phase_vertical_proj_image->widthStep+c]=(unsigned char)(unwrapped_phase_proj_vert[c][r]);
                unwrapped_phase_horizontal_proj_image->imageData[r*unwrapped_phase_horizontal_proj_image->widthStep+c]=(unsigned char)(unwrapped_phase_proj_horz[c][r]);

            }
        }

    cvSaveImage("Correspondence_error/mapped_vertical_unwrapped_phase.jpg",unwrapped_phase_vertical_proj_image);
    cvSaveImage("Correspondence_error/mapped_horizontal_unwrapped_phase.jpg",unwrapped_phase_horizontal_proj_image);











///Now lets save the error values for script to access.
    //Use values in 'vertical/horizontal_unwrapped_phase' as input to plot function.
//Select a row.

    FILE*fd_vertical_error_plot_data=fopen("Correspondence_error/vertical_unwrapped_phase_error.txt","w");
    FILE*fd_horizontal_error_plot_data=fopen("Correspondence_error/horizontal_unwrapped_phase_error.txt","w");

    ///Error will be plotted as:(X-Y-Error) coordinate system
    for(int r=0; r<Projector_imageheight; r++)
        for(int c=0; c<Projector_imagewidth; c++)
        {
            if(assigned[c][r]==true)
            {
                fprintf(fd_vertical_error_plot_data,"%lf\t%lf\t%lf\n",(double)c,(double)r,unwrapped_phase_error_vertical[c][r]);
                fprintf(fd_horizontal_error_plot_data,"%lf\t%lf\t%lf\n",(double)c,(double)r,unwrapped_phase_error_horizontal[c][r]);
            }
        }

    fcloseall();



    return;
}


*/

/*
//Plot using Scilab/AnuVi
void plot_error_map()
{


    //Execute Scilab/Anuvi script which will plot 'unwrapped_phase_error' matrices.
    //char command[500];
    //sprintf(command,"scilab -e exec\\(\\'error_plot.sci\\'\\)\\;exec\\(error_plot\\)\\;");
    //system(command);


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);


    int valid_proj_pixel_count=0;

    for(int r=0; r<Projector_imageheight; r++)
        for(int c=0; c<Projector_imagewidth; c++)
            if(assigned[c][r]==true)
                valid_proj_pixel_count++;

    printf("%d",valid_proj_pixel_count);
    fflush(stdout);

// Fill in the cloud data
    cloud->width    =valid_proj_pixel_count;
    cloud->height   = 1;
    cloud->is_dense = true;
    cloud->points.resize (cloud->width * cloud->height);

    int y=0;
    for(int i=0; i<Projector_imageheight; i++)
        for(int j=0; j<Projector_imagewidth; j++)
        {
            if(assigned[j][i]==1)
            {
                cloud->points[y].x=(float)j;
                cloud->points[y].y=(float)i;
                cloud->points[y].z=(float)(100.0*unwrapped_phase_error_vertical[j][i]);
                y++;
            }

            else
                continue;
        }


    gamma_value=-1.0;


    printf("\nEnter the gamma value used:");
    scanf("%f",&gamma_value);


    //Lets calculate RMSD:
    double rms_error_vertical=0.0;
    double rms_error_horizontal=0.0;

    for(int r=0; r<Projector_imageheight; r++)
        for(int c=0; c<Projector_imagewidth; c++)
            if(assigned[c][r]==1) //trying mean absolute error.
            {

                rms_error_vertical+=std::abs(unwrapped_phase_error_vertical[c][r]);
                rms_error_horizontal+=std::abs(unwrapped_phase_error_horizontal[c][r]);
            }


    rms_error_vertical/=valid_proj_pixel_count;

    //rms_error_vertical=sqrt(rms_error_vertical);

    rms_error_horizontal/=valid_proj_pixel_count;

    //rms_error_horizontal=sqrt(rms_error_horizontal);



    sprintf(filename,"Correspondence_error/error_vert_%d_%f.txt",number_of_patterns_fringe,gamma_value);
    FILE*fd_ver_error=fopen(filename,"w");
    fprintf(fd_ver_error,"%lf",rms_error_vertical);

    sprintf(filename,"Correspondence_error/error_hor_%d_%f.txt",number_of_patterns_fringe,gamma_value);
    FILE*fd_hor_error=fopen(filename,"w");
    fprintf(fd_hor_error,"%lf",rms_error_horizontal);


    fclose(fd_ver_error);
    fclose(fd_hor_error);

    pcl::io::savePCDFileASCII ("Correspondence_error/error_plot.pcd", *cloud);



    return;
}
*/

void handler_func(int sig)
{
    //cvWaitKey(30);
    return;
}


//Function to quantitatively compute error in estimated camera-projector correspondence.
void estimate_correspondence_error()
{
    IplImage* virtual_checkerboard_image=cvLoadImage("/home/pranav/Desktop/M_tech_project_console/checkerboard.jpg");
    IplImage*temp_virtual_checkerboard=cvCreateImage(cvGetSize(virtual_checkerboard_image),8,3);

    //Project checkerboard pattern
    printf("\nProjecting virtual checkerboard for estimating stereo-correspondence error...");
    printf("\nStart corner detection?");
    fflush(stdout);

    //signal(SIGALRM, handler_func);



    char c='n';

//cvWaitKey(30)!='y'
    /* while(c!='y')
     {

         cvCopyImage(virtual_checkerboard_image,temp_virtual_checkerboard);
         cap=cvQueryFrame(camera);
         cvScale(temp_virtual_checkerboard,temp_virtual_checkerboard,2.*(proj_gain/100.),0);
         cvShowImage("Projector_pattern",temp_virtual_checkerboard);
         cvShowImage("Camera_view",cap);
         c=cvWaitKey(300);
         //alarm(1);
         //c=getchar();

     }

     */

    cap=cvLoadImage("extrinsic_image.jpg");
    //Detect corners


    int found;
    CvPoint2D32f*corners=new CvPoint2D32f[virtual_checkerboard_X*virtual_checkerboard_Y];
    int corner_count=0;
    IplImage*gray_test_image=cvCreateImage(cvSize(Camera_imagewidth,Camera_imageheight),IPL_DEPTH_8U,1);
    cvCvtColor(cap,gray_test_image,CV_RGB2GRAY);

    found=cvFindChessboardCorners(gray_test_image,cvSize(virtual_checkerboard_X,virtual_checkerboard_Y),corners,&corner_count);

    if(found==0)
    {
        printf("\nCheckerboard not found,exiting...");
        exit(0);
    }




    //subpixel corner detection
    cvFindCornerSubPix(gray_test_image,corners,corner_count,cvSize(3,3),cvSize(-1,-1),cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,30,0.01));
    cvDrawChessboardCorners(cap,cvSize(virtual_checkerboard_X,virtual_checkerboard_Y),corners,corner_count,found);
    cvSaveImage("detected_corners_correspondence_test.jpg",cap);
    cvSaveImage("source_image.jpg",gray_test_image);
    //else,if checkerboard was detected.
    //Determine corresponding projector coordinates using c_p_map

    printf("Detected corners are:\n");
    printf("\nX=%f,Y=%f",corners[0].x,corners[0].y);
    printf("\nX=%f,Y=%f",corners[7].x,corners[7].y);
    printf("\nX=%f,Y=%f",corners[40].x,corners[40].y);
    printf("\nX=%f,Y=%f",corners[47].x,corners[47].y);

    long int cam_X,cam_Y;
    int est_proj_X,est_proj_Y;
    int ideal_proj_X,ideal_proj_Y;

    float avg_error=0.0f;

    int total_points=0;

    for(int i=0; i<corner_count; i++) //ASSUMPTION:detected corners are stored in order:Top to bottom,left to right.
    {
        feclearexcept(FE_ALL_EXCEPT);
        cam_X=lrint(corners[i].x);
        if(fetestexcept(FE_INVALID)!=0) //exception occured!!
        {
            printf("\nFE occured!!,rejecting point...");
            continue;
        }

        feclearexcept(FE_ALL_EXCEPT);
        cam_Y=lrint(corners[i].y);
        if(fetestexcept(FE_INVALID)!=0) //exception occured!!
        {
            printf("\nFE occured!!,rejecting point...");
            continue;
        }

        if(valid_map[cam_X][cam_Y]!=1)
        {
            printf("\nInvalid point!! rejecting...");
            continue;
        }

        //else use the point!!
        total_points++;

        est_proj_X=c_p_map[cam_Y*Camera_imagewidth+cam_X][0];//x
        est_proj_Y=c_p_map[cam_Y*Camera_imagewidth+cam_X][1];//y

        ideal_proj_X=((i%virtual_checkerboard_X)+1)*square_pixels+100;
        ideal_proj_Y=(floor(i/virtual_checkerboard_X)+1)*square_pixels+47;

        avg_error+=sqrt(pow((ideal_proj_X-est_proj_X),2)+pow((ideal_proj_Y-est_proj_Y),2));
    }

    printf("\nAbsolute average stereo-correspondence error:%f over %d points",avg_error/=total_points,total_points);


    //Compute error as True_projector_coordinates-estimated_projector_coordinates.

    //Lets deallocate dynamic variables...
    delete[] corners;
    cvReleaseImage(&gray_test_image);
    cvReleaseImage(&virtual_checkerboard_image);
    cvReleaseImage(&temp_virtual_checkerboard);

    return;
}


void generate_projector_view()
{
    IplImage *projector_view=cvCreateImage(cvSize(1024,768),8,3);
    IplImage *camera_view=cvLoadImage("Captured_patterns/Coded_patterns/Vertical/Original/Captured_image_0.jpg");

    cvSet(projector_view,cvScalar(0));


    for(int r=0; r<Camera_imageheight; r++)
        for(int c=0; c<Camera_imagewidth; c++)
            if(valid_map[c][r]==1)
            {
                projector_view->imageData[c_p_map[r*Camera_imageheight+c][1]*projector_view->widthStep+c_p_map[r*Camera_imageheight+c][0]*projector_view->nChannels+0]=(unsigned char)camera_view->imageData[r*camera_view->widthStep+c*camera_view->nChannels+0];
                projector_view->imageData[c_p_map[r*Camera_imageheight+c][1]*projector_view->widthStep+c_p_map[r*Camera_imageheight+c][0]*projector_view->nChannels+1]=(unsigned char)camera_view->imageData[r*camera_view->widthStep+c*camera_view->nChannels+1];
                projector_view->imageData[c_p_map[r*Camera_imageheight+c][1]*projector_view->widthStep+c_p_map[r*Camera_imageheight+c][0]*projector_view->nChannels+2]=(unsigned char)camera_view->imageData[r*camera_view->widthStep+c*camera_view->nChannels+2];
            }

    cvSaveImage("Projector_view.jpg",projector_view);

    return;

}









void compute_c_p_map()
{



    valid_map=new int[Camera_imagewidth][Camera_imageheight];

    merge_valid_maps(); //it will create intersection of both 'valid_map_vertical' & 'valid_map_horizontal'


    c_p_map=new long int[total_camera_pixels][2];

    for(int r=0; r<Camera_imageheight; r++)
        for(int c=0; c<Camera_imagewidth; c++)
        {
            if(valid_map[c][r]==1)
            {
                feclearexcept(FE_ALL_EXCEPT);//to reset all floating point exception flags.
                c_p_map[r*Camera_imagewidth+c][0]=lrint(fringe_width_pixels_vertical*(unwrapped_phi_vertical[c][r]/(2.0*Pi)));
                //check for floating point exception: FE_INVALID
                if(fetestexcept(FE_INVALID)!=0)//floating point error occured
                {
                    printf("\nFloating point error occured!! rejecting point");
                    valid_map[c][r]=0;
                    continue;
                }


                feclearexcept(FE_ALL_EXCEPT);
                c_p_map[r*Camera_imagewidth+c][1]=lrint(fringe_width_pixels_horizontal*(unwrapped_phi_horizontal[c][r]/(2.0*Pi)));
                if(fetestexcept(FE_INVALID)!=0)//floating point error occured
                {
                    printf("\nFloating point error occured!! rejecting point..");
                    valid_map[c][r]=0;
                    continue;
                }





                if((c_p_map[r*Camera_imagewidth+c][0]>(Projector_imagewidth-1)) || (c_p_map[r*Camera_imagewidth+c][1]>(Projector_imageheight-1))||(c_p_map[r*Camera_imagewidth+c][0]<0) ||(c_p_map[r*Camera_imagewidth+c][1]<0) )
                {
                    valid_map[c][r]=0;//discard the point.

                }


            }
        }
    //cvDestroyAllWindows();

    // estimate_correspondence_error();

    //generate_projector_view(); //This function will generate view of scene as from projector point-of-view using the estimated stereo-correspondence.



//    delete(unwrapped_phi_vertical);
//   delete(unwrapped_phi_horizontal);

    //map_unwrapped_phase_to_projector_space();


    //compute_error_map();
    //plot_error_map();


    //Lets check the correspondance.
    //check_correspondance();



//Lets do the deallocation...
    cvReleaseImage(&camera_image);
    cvReleaseImage(&projector_image);
    cvReleaseImage(&temp_projector_image);
    cvReleaseImage(&temp_camera_image);





    return;
}


