#include <iostream>
#include <algorithm>
#include <cmath>
#include <iterator>

#include <cv.h>
#include <highgui.h>


using namespace std;
using namespace cv;

void mat2vec3f( vector< vector<Point3f> > &v , Mat &m ){

	vector<Point3f> tmp;

	for(int i=0;i<m.rows;i++){
		tmp.push_back( Point3f( m.at<float>(i,0) , m.at<float>(i,1) , 0.0) );
	}

	v.push_back( tmp );
}

void mat2vec2f( vector< vector<Point2f> > &v , Mat &m ){

	vector<Point2f> tmp;

	for(int i=0;i<m.rows;i++){
		tmp.push_back( Point2f( m.at<float>(i,0) , m.at<float>(i,1) ) );
	}

	v.push_back( tmp );
}


void read_data( vector< vector<Point2f> > &cam , vector< vector<Point3f> > &proj , char *path){

	FileStorage fs( path , FileStorage::READ );
	Mat m_tmp; 

	fs["CAM_PTS"] >>m_tmp;
	m_tmp.convertTo( m_tmp , CV_32F);
	mat2vec2f( cam , m_tmp );

	fs["PROJ_PTS"] >>m_tmp;
	m_tmp.convertTo( m_tmp , CV_32F);
	mat2vec3f( proj , m_tmp );

}


int main( int argc , char * argv[] ){

	vector< vector<Point3f> > obj_pts ;
	vector< vector<Point2f> >  image_pts;

	for(int i=1;i<argc;i++){
		read_data( image_pts , obj_pts , argv[i] );
	}

	Mat K = Mat::eye( 3 , 3 , CV_64F );
	Mat dist;
	vector<Mat> rots , trans;
	float err = calibrateCamera( obj_pts , image_pts , Size(3456,2304) , K , dist , rots , trans );

	cout<<K<<endl<<dist<<endl;

	FileStorage fs( "Kcam.yml" , FileStorage::WRITE );
	fs<<"K"<<K;
	fs<<"DIST"<<dist;
	fs<<"ERROR"<<err;

	return 0;
}

