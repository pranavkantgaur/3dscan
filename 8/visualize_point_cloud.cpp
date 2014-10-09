/*
TARGET:To visulaize the point cloud using PCL library tool called 'cloud viewer'.
Steps:
1.Read the point cloud file.
2.Use cloud viwer to visualize the points.
*/

#include"../PROJECT_GLOBAL/global_cv.h"


int user_data;

void viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (0.0, 0.0, 0.0);
    pcl::PointXYZRGB o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.8, "sphere", 0);
    std::cout << "i only run once" << std::endl;

}

void viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    //static unsigned count = 0;
    //std::stringstream ss;
    //ss << "Once per viewer loop: " << count++;
    //viewer.removeShape ("text", 0);
    //viewer.addText (ss.str(), 200, 300, "text", 0);

    //FIXME: possible race condition here:
    //user_data++;
}

static int click_count=0;
float X1=0.0,Y1=0.0,Z1=0.0,X2=0.0,Y2=0.0,Z2=0.0,length=0.0;

void callback_function(const pcl::visualization::PointPickingEvent& event,void* a)
{


    click_count++;

    if(click_count==1)
    {
        event.getPoint(X1,Y1,Z1);
        printf("Point-1 selected:%f\t%f\t%f\n",X1,Y1,Z1);
        fflush(stdout);
    }

    if(click_count==2)
    {
        event.getPoint(X2,Y2,Z2);
        length=sqrt(pow(X1-X2,2)+pow(Y1-Y2,2)+pow(Z1-Z2,2));
        printf("Point-2 selected:%f\t%f\t%f\n",X2,Y2,Z2);
        printf("Euclidean distance:%f\n",length);
        fflush(stdout);

        click_count=0;//for next selection of points.

    }





    return;
}



void visualize_point_cloud()
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPCDFile ("/home/pranav/Desktop/M_tech_project_console/Point_cloud/point_cloud_0.pcd", *cloud);


    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    boost::signals2::connection connection_object;


    connection_object=viewer.registerPointPickingCallback(&callback_function);







    //blocks until the cloud is actually rendered
    viewer.showCloud(cloud);

    //use the following functions to get access to the underlying more advanced/powerful
    //PCLVisualizer

    //This will only get called once
    viewer.runOnVisualizationThreadOnce (viewerOneOff);

    //This will get called once per visualization iteration
    viewer.runOnVisualizationThread (viewerPsycho);
    while (!viewer.wasStopped ())
    {

    }


    return;
}














