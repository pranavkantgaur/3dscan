/*
TARGET: To register multiple point clouds to a common coordinate system

STEPS:
For each point cloud index i:
    #1. Read point cloud P_i
    #2. Apply[-T][R_i][T] to all points in P_i
    #3. Add generated points to registered point cloud R.


*/

#include"../PROJECT_GLOBAL/global_cv.h"
#define PI 3.14159265

extern float rot_step;
extern float tx;
extern float ty;
extern float tz;
extern char filename[150];


void register_point_clouds(unsigned num_point_clouds,float tx,float ty,float tz,float rot_step)
{
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr registered_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>); // it will contain the final result of registration

CvMat*R=cvCreateMat(4,4,CV_32FC1);

float  theta;



//Rotation matrix
//Currently, we have single axis rotation about Y-axis
CV_MAT_ELEM(*R,float,1,1)=1.0f;
CV_MAT_ELEM(*R,float,3,3)=1.0f;

/*
CV_MAT_ELEM(*R,float,0,1)=0.0f;
CV_MAT_ELEM(*R,float,0,3)=0.0f;

CV_MAT_ELEM(*R,float,1,0)=0.0f;
CV_MAT_ELEM(*R,float,1,2)=0.0f;
CV_MAT_ELEM(*R,float,1,3)=0.0f;

CV_MAT_ELEM(*R,float,2,1)=0.0f;
CV_MAT_ELEM(*R,float,2,0)=0.0f;

CV_MAT_ELEM(*R,float,3,0)=0.0f;
CV_MAT_ELEM(*R,float,3,1)=0.0f;
CV_MAT_ELEM(*R,float,3,2)=0.0f;
*/

CvMat *point=cvCreateMat(4,1,CV_32FC1);
unsigned count=0;


//Lets first initialize the output point cloud structure (registred_cloud) so as to store newly transformedclouds into it

registered_cloud->width=0;

for(unsigned i=0;i<num_point_clouds;i++)
{
   sprintf(filename,"Point_cloud/point_cloud_%d.ply",i);
   pcl::io::loadPLYFile(filename, *cloud);
   registered_cloud->width+=cloud->width;
}

registered_cloud->height=1;
registered_cloud->is_dense=true;
registered_cloud->points.resize (registered_cloud->width * registered_cloud->height);

printf("\nregistred cloud width:%d",registered_cloud->width);
fflush(stdout);

theta=0.0;
unsigned prev_last_point_id=0;

//lets trasform the point clouds


for(unsigned i=0;i<num_point_clouds;i++)
{
    sprintf(filename,"Point_cloud/point_cloud_%d.ply",i);
    pcl::io::loadPLYFile(filename, *cloud);

    //updating rotation marix as per point cloud view
    CV_MAT_ELEM(*R,float,0,0)=cos(theta*Pi/180.0);
    CV_MAT_ELEM(*R,float,0,2)=-1.0f*sin(theta*Pi/180.0);

    CV_MAT_ELEM(*R,float,2,0)=sin(theta*Pi/180.0);
    CV_MAT_ELEM(*R,float,2,2)=cos(theta*Pi/180.0);

    CV_MAT_ELEM(*R,float,1,1)=1.0f;
    CV_MAT_ELEM(*R,float,3,3)=1.0f;

    printf("\nRotation matrix for cloud : %d",i);

    for(unsigned i=0;i<4;i++)
    {
      printf("\n");
      for(unsigned j=0;j<4;j++)
        printf("%f\t",CV_MAT_ELEM(*R,float,i,j));
    }


    // transform all points of the point cloud
    for(unsigned point_id=0;point_id<cloud->points.size();point_id++)
     {
       CV_MAT_ELEM(*point,float,0,0)=cloud->points[point_id].x;
       CV_MAT_ELEM(*point,float,1,0)=cloud->points[point_id].y;
       CV_MAT_ELEM(*point,float,2,0)=cloud->points[point_id].z;
       CV_MAT_ELEM(*point,float,3,0)=1.0f;

       CV_MAT_ELEM(*point,float,0,0)-=tx;
       CV_MAT_ELEM(*point,float,1,0)-=ty;
       CV_MAT_ELEM(*point,float,2,0)-=tz;

       cvMatMul(R,point,point);

       CV_MAT_ELEM(*point,float,0,0)+=tx;
       CV_MAT_ELEM(*point,float,1,0)+=ty;
       CV_MAT_ELEM(*point,float,2,0)+=tz;

      cloud->points[point_id].x=CV_MAT_ELEM(*point,float,0,0);
      cloud->points[point_id].y=CV_MAT_ELEM(*point,float,1,0);
      cloud->points[point_id].z=CV_MAT_ELEM(*point,float,2,0);
     }

    // add the modified point cloud to the registration result
    count=0;
    for(unsigned i=prev_last_point_id;i<(prev_last_point_id+cloud->points.size());i++)
    {
      registered_cloud->points[i].x=cloud->points[count].x;
      registered_cloud->points[i].y=cloud->points[count].y;
      registered_cloud->points[i].z=cloud->points[count].z;

      registered_cloud->points[i].r=cloud->points[count].r;
      registered_cloud->points[i].g=cloud->points[count].g;
      registered_cloud->points[i].b=cloud->points[count++].b;
    }

    printf("\nRotating by :%f",theta);
    theta+=rot_step;
    prev_last_point_id+=cloud->points.size();

}

// Saving resultant registered point cloud
     pcl::io::savePLYFile("Point_cloud/registered_point_cloud.ply",*registered_cloud);


return;
}
