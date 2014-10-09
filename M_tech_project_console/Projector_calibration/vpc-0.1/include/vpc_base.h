#ifndef __VPCBASE__H
#define __VPCBASE__H

#include <iostream>
#include <algorithm>
#include <cmath>
#include <iterator>
#include <fstream>
#include <string>
#include <cstdlib>
#include <iosfwd>

#include "cv.h"
#include "highgui.h"

#include "template_header.h"
#include "vpc_bundle.h"

using namespace std;
using namespace cv;



namespace VPC{
	void p3_to_p2( vector<Point3d> &src , vector<Point2f> &dst);

	class Calib{
		public:
			Mat K;//Projector intrinsic.
			Mat err;//[avg_err,min_err,max_err].


			//Calibration main entry.
			virtual void calibrate( bool with_lm )=0;//with or without lm minimization.

			//Levenberg-Marquardt main routine.
			void lm( int MAX_ITER = 100 , double epsilon=1e-10 );

			//Called at the begining of LM.
			virtual void lm_init(){};

			//Called at the end of LM.
			virtual void lm_finalize(){};

			//The Jacobian Matrix (J) and the residual vector (E).
			virtual void jacobian( Mat &J , Mat &E)=0; 

			//Tries a solution update and returns the avg_error.
			virtual double try_update( const Mat &delta)=0;

			//Applies an update to a solution.
			virtual double apply_update( const Mat &delta)=0;

			//Returns the RMS error as [avg_err;min_err;max_err].
			virtual Mat rms()=0;

			//Return number of views
			virtual int nb_views()=0;

			//Returns number of params to bundle adjust.
			virtual int nb_params()=0;

			Calib( ) { K = Mat::zeros(3,3,CV_64F);err=Mat::zeros(3,1,CV_64F);};
			virtual ~Calib(){};

	};


}

/*
//This is a class to hold a solution to the calibration.
//...Maybe derive it for each type of calibration ??
class _VPCSOL_{

	public:
		Mat K;//Projector intrinsic.
		Mat Hwc;//world-->cam homography.
		Mat rep_err;//[avg_err,min_err,max_err].
		vector<Mat> R;//Rotations.
		vector<Mat> T;//Translations.

		//Constructor.
		//n_views: number of poses (EXCLUDING the fronto-// for the selfCalib case !) 
		_VPCSOL_(int n_views )  
							: K(Mat::eye(3,3,CV_64F) ) , Hwc(Mat::eye(3,3,CV_64F) ),
								rep_err(Mat::eye(1,3,CV_64F) ), R(n_views),
								T(n_views){

		};

		int nb_views() const { return R.size();}
		_VPCSOL_& operator=( const _VPCSOL_ &rhs );
		friend std::ostream& operator<<(std::ostream &os, const _VPCSOL_ &sol);

};

//Base class for calibration solutions.
class VpcSol{
	public:
		Mat K;//Projector intrinsic.
		Mat err;//[avg_err,min_err,max_err].
 		
		vector< vector<Point3d> > cam_pts;//Camera points.
		vector< vector<Point3d> > proj_pts;//Corresponding projector points.

		//This function returns the jacobian at a single point.
		virtual void jac_single( const Point3d &src , const Point3d &dst , vector<Mat> &grad , vector<double> &merit){ };
		virtual double try_update( const Mat &delta){ return 0;};
		virtual double apply_update( const Mat &delta){ return 0;};
		virtual void rms( Mat &rms_vec){ rms_vec = Mat::zeros(3,1,CV_64F);};

		virtual ~VpcSol()=0;
};


//Calibration solution for "Moving Projectors" methods.
class VpcSolMP : public VpcSol{

	public:
		Mat Hwc;//world-->cam homography.
		
		//Projetor's extrinsics.
		vector<Mat> R;//Rotations.
		vector<Mat> T;//Translations.

		void jac_single( const Point3d &src , const Point3d &dst , vector<Mat> &grad , vector<double> &merit);
		double try_update( const Mat &delta);
		double apply_update( const Mat &delta);
		void rms( Mat &rms_vec);

		VpcSolMP();
		~VpcSolMP(){ };
};

//An ostream formatter for _VPCSOL_.
//use vpc_fmt( str ) as a modifier, str
//encodes the desired format:
//R: rotations.
//T: translations.
//X: extrinsic (R+T).
//K: intrinsics.
//E: reprojection error.
//H: homography wall-->camera.
//..
//..
//default: "KE"-->intrinsic+rep_error.
//ex: cout<<vpc_fmt("RTKHE")<<solObj<<endl; ...displays 
//    in this order rots,trans,intrinsics,homography 
//    and the error.
class VpcFormatter{

	private:
		char *fmt;

	public:
		VpcFormatter(const char _fmt[]) {
			fmt = strdup( _fmt );
		}

		~VpcFormatter(){ free(fmt);}
		void set_ostream_info(std::ostream& os) const ;


};

VpcFormatter vpc_fmt( const char fmt[]);
std::ostream& operator<<( std::ostream& os, const VpcFormatter & formatter);



//Utilities
Mat vpcError( const vector<vector<Point2f> > &pt_cam , const vector<Point2f> &pt_proj , const Mat &Hc2w , const Mat &Kproj , 
			  const vector<Mat> &rots , const vector<Mat> &trans);
Mat vpcErrorSelf( const vector<vector<Point2f> > &pt_cam , const vector<Point2f> &pt_proj , 
				  const Mat &Hwc , const Mat &Kproj , const vector<Mat> &rots , const vector<Mat> &trans);








//Calibration by orientation sampling.
void vpcSampling( const vector<vector<Point2f> > &pt_cam , const vector<Point2f> &pt_proj ,
				  Mat &Kcam , double focal_min , double focal_max , Mat &Kproj  );

//Planar auto-calibration
void vpcSelf( const vector<Mat> &Hcp , const vector<Mat> &Hcp_f , const vector< vector<Point2f> > &cam_pts , 
			  const vector< vector<Point2f> >  &cam_pts_fronto , const vector< vector<Point2f> > &proj_pts, double aspect,
			  _VPCSOL_ &sol);

*/
#endif
