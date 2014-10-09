#include <iostream>
#include <algorithm>
#include <cmath>
#include <iterator>
#include <fstream>
#include <string>
#include <cstdlib>

#include "cv.h"
#include "highgui.h"

#include "vpc.h"

using namespace std;
using namespace cv;


void mat2vec( vector< vector<Point3d> > &v , Mat &m ){

	vector<Point3d> tmp;

	for(int i=0;i<m.rows;i++){
		tmp.push_back( Point3d( m.at<double>(i,0) , m.at<double>(i,1) , 1.0) );
	}

	v.push_back( tmp );
}

void read_camera( Mat &K , char *path ){
	FileStorage fs( path , FileStorage::READ );
	Mat m_tmp; 

	fs["K"] >>m_tmp;
	m_tmp.convertTo( K , CV_64F);
	

}

void read_data( vector< vector<Point3d> > &cam , vector< vector<Point3d> > &proj , char *path){

	FileStorage fs( path , FileStorage::READ );
	Mat m_tmp; 

	fs["CAM_PTS"] >>m_tmp;
	m_tmp.convertTo( m_tmp , CV_64F);
	mat2vec( cam , m_tmp );

	fs["PROJ_PTS"] >>m_tmp;
	m_tmp.convertTo( m_tmp , CV_64F);
	mat2vec( proj , m_tmp );

}


int main( int argc , char* argv[]){


	if(argc<4){
		printf("Usage: %s Kcam.yml matches0.yml matches1.yml..." , argv[0]);
		exit(1);
	}

	vector< vector<Point3d> > proj_pts , cam_pts;

	Mat Kcam;
	read_camera( Kcam , argv[1] );

	for(int i=2;i<argc;i++){
		read_data( cam_pts , proj_pts , argv[i] );
	}




	//VPC::Sampling calibObj( cam_pts , proj_pts , 3375,3600,790,435); 
	VPC::Sampling calibObj( cam_pts , proj_pts , Kcam );
        calibObj.calibrate(false);
	cout<<"[Init Error]: \n\t"<< calibObj.rms()<<endl;
        cout<<"[Bundle Adjustment]: \n\toptimizing ... "<< calibObj.nb_params() <<" parameters.\n";

	calibObj.calibrate(true);
	cout<<"[Final Error]: \n\t"<< calibObj.rms()<<endl;
	cout<<"[Calib Matrix]:\n\t"<<calibObj.K<<endl;



        return 0;
}

