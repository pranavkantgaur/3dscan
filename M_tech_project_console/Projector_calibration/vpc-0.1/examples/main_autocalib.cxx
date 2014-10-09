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

	vector< vector<Point3d> > proj_pts , cam_pts;
	vector< vector<Point3d> > proj_pts_fronto , cam_pts_fronto;

	if( argc < 2 || strcmp( argv[1] , "-h" ) == 0 ){
		cout<<"Usage: "<<argv[0]<<" -f matches_fronto_0.yml matches_fronto_1.yml...-p matches0.yml matches1.yml...\n";
		exit(1);
	}

	/**Parse Data**/
	for(int i=1;i<argc;i++){
		if( strcmp(argv[i] , "-f" ) == 0 && i+1<argc){
			//Fronto-// poses.
			while( i+1<argc && argv[i+1][0] != '-' ){
				read_data( cam_pts_fronto , proj_pts_fronto , argv[i+1]  );
				i++;
			};
		}

		if( strcmp(argv[i] , "-p" ) == 0 && i+1<argc ){
			// arbitrary poses.
			while( i+1<argc && argv[i+1][0] != '-' ){
				read_data( cam_pts , proj_pts , argv[i+1]  );
				i++;
			};
		}


	}

	VPC::AutoCalib calibObj( cam_pts , proj_pts , cam_pts_fronto , proj_pts_fronto , 1.0 ); 
	calibObj.calibrate(false);
	cout<<"[Init Error]: \n\t"<< calibObj.rms()<<endl;
        cout<<"[Bundle Adjustment]: \n\toptimizing ... "<< calibObj.nb_params() <<" parameters.\n";

	calibObj.calibrate(true);
	cout<<"[Final Error]: \n\t"<< calibObj.rms()<<endl;
	cout<<"[Calib Matrix]:\n\t"<<calibObj.K<<endl;

	return 0;
}

