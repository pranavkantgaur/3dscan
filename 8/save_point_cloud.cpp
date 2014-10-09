/*
TARGET:To save the depth map for each pixel in .psd form which can be later visualized using a point cloud viewer.
Step:1.Save point cloud data in a pointcloud structure offered by PCL library.
Adding code for adding point cloud data to an existing .ply file to be used in AnuVi viewer also.
*/

///UPDATE-9 OCT 2012
///Adding code for computing X & Y spatial resolution.
///To be added computation of Z-resolution

#include"../PROJECT_GLOBAL/global_cv.h"
#define INVALID_VALUE 10000000.0f


//extern float d[Camera_imagewidth][Camera_imageheight];
extern int (*valid_map)[Camera_imageheight];
extern double (*intersection_points)[Camera_imageheight][3];
extern char filename[150];
extern int number_of_patterns_binary_vertical;



//extern int assigned[Projector_imagewidth][Projector_imagewidth];


void save_point_cloud (unsigned cloud_index)
{

//    Lets first count the number of valid points in the point cloud(only these will be plotted)
    int count=0;
    //float avg_Z=0.0f;

    for(int r=0; r<Camera_imageheight; r++)
        for(int c=0; c<Camera_imagewidth; c++)
            if((valid_map[c][r]==1))
            {
                count++;
                //avg_Z+=intersection_points[c][r][2];
            }

    //avg_Z/=count;


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);


    // Fill in the cloud data
    cloud->width    = count;
    cloud->height   = 1;
    cloud->is_dense = true;
    cloud->points.resize (cloud->width * cloud->height);

    /*    cloud->width    = Camera_imagewidth;
        cloud->height   = Camera_imageheight;
        cloud->is_dense = true;
        cloud->points.resize (cloud->width * cloud->height);
    */

    IplImage*I1=cvLoadImage("Point_cloud/texture.bmp");


    IplImage*red=cvCreateImage(cvSize(Camera_imagewidth,Camera_imageheight),IPL_DEPTH_8U,1);
    IplImage*green=cvCreateImage(cvSize(Camera_imagewidth,Camera_imageheight),IPL_DEPTH_8U,1);
    IplImage*blue=cvCreateImage(cvSize(Camera_imagewidth,Camera_imageheight),IPL_DEPTH_8U,1);

    cvSplit(I1,blue,green,red,NULL);




    //Computing MAE for deviation of plane from Z=0(Assumption:Scanning the reference X-Y plane)

//float MAE=0.0f; //Mean absolute error.
//float RMSE=0.0f; //standard deviation


//Lets first calculate the average z value


//I want to plot Z=0 plane also.




    int y=0;
    for (int i=0; i<Camera_imageheight; i++)
        for(int j=0; j<Camera_imagewidth; j++)
        {
            if(valid_map[j][i]==1)
            {
                cloud->points[y].r=(unsigned char)red->imageData[i*red->widthStep+j];
                cloud->points[y].g=(unsigned char)green->imageData[i*green->widthStep+j];
                cloud->points[y].b=(unsigned char)blue->imageData[i*blue->widthStep+j];
                cloud->points[y].x=(float)intersection_points[j][i][0];
                cloud->points[y].y=(float)intersection_points[j][i][1];
                cloud->points[y].z=(float)intersection_points[j][i][2];

                //MAE+=fabs(cloud->points[y].z-avg_Z); //i.e., deviation wrt. reference plane z=0.

                //RMSE+=pow((cloud->points[y].z-avg_Z),2);


                y++;


                /*
                                cloud->points[i*cloud->width+j].r=(unsigned char)red->imageData[i*red->widthStep+j];
                                cloud->points[i*cloud->width+j].g=(unsigned char)green->imageData[i*green->widthStep+j];
                                cloud->points[i*cloud->width+j].b=(unsigned char)blue->imageData[i*blue->widthStep+j];
                                cloud->points[i*cloud->width+j].x=(float)intersection_points[j][i][0];
                                cloud->points[i*cloud->width+j].y=(float)intersection_points[j][i][1];
                                cloud->points[i*cloud->width+j].z=(float)intersection_points[j][i][2];


                            }

                           else
                            {
                                cloud->points[i*cloud->width+j].r=0;
                                cloud->points[i*cloud->width+j].g=0;
                                cloud->points[i*cloud->width+j].b=0;
                                cloud->points[i*cloud->width+j].x=INVALID_VALUE;
                                cloud->points[i*cloud->width+j].y=INVALID_VALUE;
                                cloud->points[i*cloud->width+j].z=INVALID_VALUE;

                            }

                */






            }
        }

    //  printf("MAE is :%f\n",MAE/=count); //it gives me the average error.

    //   printf("RMSE is %f\n",RMSE=sqrt(RMSE/count));


    //    printf("Average plane Z=%f\n",avg_Z);

    /*
    ///Code to compute X & Y spatial resolutions:
    double res_X=0.0,res_Y=0.0,avg_res_X=0.0,avg_res_Y=0.0;
    int avg_valid_count=0,X_valid_count=0,Y_valid_count=0;

    bool row_is_valid=false,column_is_valid=false;

    ///X-resolution
        for(int r=0; r<Camera_imageheight; r++)
        {
            res_X=0.0;
            X_valid_count=0;
            row_is_valid=false;

            for(int c=0; c<Camera_imagewidth-1; c++)
                if((valid_map[c][r]==1) && (valid_map[c+1][r]==1))
                {
                    res_X+=sqrt(pow((intersection_points[c][r][0]-intersection_points[c+1][r][0]),2.0)+pow((intersection_points[c][r][1]-intersection_points[c+1][r][1]),2.0)+pow((intersection_points[c][r][2]-intersection_points[c+1][r][2]),2.0));
                    X_valid_count++;

                    row_is_valid=true; //Should be executed only till a valid pair is deteced for a row.(FIX IT!!!!)
                }

           if(row_is_valid==true)
            {
            res_X/=X_valid_count; ///X-resolution across a row
            avg_res_X+=res_X;
            avg_valid_count++;
            }
        }

        avg_res_X/=avg_valid_count; ///Average X_resolution across a point cloud


    avg_valid_count=0;

    ///Y-resolution
    for(int c=0; c<Camera_imagewidth; c++)
    {
        res_Y=0.0;
        Y_valid_count=0;
        column_is_valid=false;

       for(int r=0; r<Camera_imageheight-1; r++)
         if((valid_map[c][r]==1) && (valid_map[c][r+1]==1))
           {
             res_Y+=sqrt(pow((intersection_points[c][r][0]-intersection_points[c][r+1][0]),2.0)+pow((intersection_points[c][r][1]-intersection_points[c][r+1][1]),2.0)+pow((intersection_points[c][r][2]-intersection_points[c][r+1][2]),2.0));
             Y_valid_count++;
             column_is_valid=true;//FIX IT!!!!!!!!!
           }
             if(column_is_valid==true)
             {
                 res_Y/=Y_valid_count; ///X-resolution across a column
                 avg_res_Y+=res_Y;
                 avg_valid_count++;
              }
    }
        avg_res_Y/=avg_valid_count; ///Average X_resolution across a point cloud


    //print spatial resolution:
    printf("\n\nX-resolution=%lf\tY_resolution=%lf\n",avg_res_X,avg_res_Y);

    */


    sprintf(filename,"Point_cloud/point_cloud_%d.pcd",cloud_index);
    pcl::io::savePCDFileASCII (filename, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to test_pcd.pcd." << std::endl;


    sprintf(filename,"Point_cloud/point_cloud_%d.ply",cloud_index);
    pcl::io::savePLYFile(filename,*cloud);

    return ;
}

