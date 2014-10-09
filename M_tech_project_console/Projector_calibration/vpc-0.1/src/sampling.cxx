#include "sampling.h"

using namespace std;
using namespace cv;

namespace VPC{

	Sampling::Sampling( vector< vector<Point3d> > &cam_pts , vector< vector<Point3d> > &proj_pts , Mat &_Kcam)
		: calibObj(cam_pts,proj_pts) , HTRKI(cam_pts , proj_pts) , have_kcam(true) {

			Kcam = Mat::eye(3,3,CV_64F);
			_Kcam.copyTo( Kcam );
		}

	Sampling::Sampling( vector< vector<Point3d> > &cam_pts , vector< vector<Point3d> > &proj_pts , 
			double f_min , double f_max , double _u0 , double _v0) 
		: calibObj(cam_pts,proj_pts) , HTRKI(cam_pts , proj_pts) ,focal_min(f_min) , focal_max(f_max) , have_kcam(false) {

			Kcam = Mat::eye(3,3,CV_64F);
			Kcam.at<double>(0,2) = _u0;
			Kcam.at<double>(1,2) = _v0;
		}

	

	void Sampling::calibrate( bool with_lm ){

		double dir[3];
		double rot_axis[3];
		Mat rvec(3,1,CV_64F , rot_axis );
		Mat rmat(3,3,CV_64F);
		Mat best_H(3,3,CV_64F);
		Mat H_probe=Mat::eye(3,3,CV_64F);
		Mat _h12 = H_probe.colRange( Range(0,2) );
		double best_rms = 10E8;
		double best_x1 ,best_x2;

		double delta = 0.1;
		double delta_focal = 10;
		double x1_min = -1 , x1_max = +1;
		double x2_min = -1 , x2_max = +1;


		while(delta > 1E-4){

			for( double x1 = x1_min; x1 <= x1_max; x1 += delta ){
				double x1_2 = x1 * x1;

				for( double x2 = x2_min; x2 <= x2_max; x2 += delta ){
					double x2_2 = x2 * x2;
					double sum = x1_2 + x2_2;
					dir[2] = 1 - 2 * sum;

					//Reject out of bounds and negative Z...we only need a hemisphere.
					if( sum >= 1 || dir[2] <= 0 )
						continue;

					double s = sqrt( 1 - sum );
					dir[0] = 2 * x1 * s;
					dir[1] = 2 * x2 * s;

					/* Core calibration....*/
					double angle , nrm ;
					H_probe=Mat::eye(3,3,CV_64F);

					//rotation angle = acos( [0,0,1].dir )
					angle = acos( dir[2] ) ;

					//rotation axis is ([0,0,1] x dir)
					//...scaled by the angle-->Rodrigues form.
					rot_axis[0] =  -dir[1];
					rot_axis[1] = dir[0];
					rot_axis[2] = 0;

					nrm = sqrt( POW2(rot_axis[0])+POW2(rot_axis[1]) );
					rot_axis[0] *= angle / nrm;
					rot_axis[1] *= angle / nrm;


					//Get the corresponding rotation matrix.
					Rodrigues( rvec , rmat );
					(rmat.colRange( Range(0,2) ) ).copyTo( _h12 );

					//Build the homography wall-->camera
					H_probe  = Kcam * H_probe;

					//Build the direct-calibration object
					//...and calibrate.
					calibObj.set_hwc( H_probe );
					calibObj.calibrate( false );

					Mat tmp_rms = calibObj.rms();

					if( tmp_rms.at<double>(0,0) < best_rms ){
						best_rms = tmp_rms.at<double>(0,0);
						H_probe.copyTo( best_H );
						best_x1 = x1;
						best_x2 = x2;
					}
				}

			}

			x1_min = best_x1 - delta;
			x1_max = best_x1 + delta;

			x2_min = best_x2 - delta;
			x2_max = best_x2 + delta;
			delta /= 10;
		}

			calibObj.set_hwc( best_H );
			calibObj.calibrate( false );

			//Copy the results (R,T,K,Hwc)
			calibObj.K.copyTo( K );
			calibObj.Hwc.copyTo( Hwc );
			for(int i=0; i<nb_views();i++){
				calibObj.R[i].copyTo( R[i] );
				calibObj.T[i].copyTo( T[i] );
			}

		if( with_lm ) 
			lm();

	};




}

