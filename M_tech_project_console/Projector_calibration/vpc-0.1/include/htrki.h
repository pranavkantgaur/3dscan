
/*HTRKI is a calibration class for procam systems which projection's *
 *error is modeled as: 												 *
 * q'(i) = (H.T.T.K).inv * Q(i)    									 * 
 * err = q(i) - q'(i)/q'(i)_3      									 */

#ifndef __VPC_HTRKI__H
#define __VPC_HTRKI__H

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
#include "vpc_base.h"

using namespace std;
using namespace cv;




namespace VPC{

	class HTRKI : public Calib{
		public:
			Mat Hwc; //world-->cam homography.

			vector< vector<Point3d> > cam_pts;//Camera points.
			vector< vector<Point3d> > proj_pts;//Corresponding projector points.


			//Homog camera-->projectors.
			vector<Mat> Hcp;

			//Projector's extrinsic
			vector<Mat> R;
			vector<Mat> T;


			virtual void calibrate( bool with_lm )=0;
			int nb_views(){return cam_pts.size();}
			int nb_params(){ return nb_param_to_optimize;}

			Mat rms();

			virtual void lm_init();
			virtual void lm_finalize();

			//The Jacobian Matrix (J) and the residual vector (E).
			void jacobian( Mat &J , Mat &E); 

			//Tries a solution update and returns the avg_error.
			double try_update( const Mat &delta);

			//Applies an update to a solution.
			double apply_update( const Mat &delta);

                        //Fix gauge parameters and put projectors in canonical form
                        void fix_gauge();

			HTRKI(){};
			HTRKI( vector< vector<Point3d> > &cam_pts , vector< vector<Point3d> > &proj_pts );
			HTRKI( vector<Mat> &Hwp) ;

		private:
			//Params matrices for swapping.
			//speeds up bundle adjustment.
			Mat s_Hwc , s_K; 
			vector<Mat> s_R;
			vector<Mat> s_T;
			int nb_param_to_optimize;

			//swap params.
			void swap_params(){
				Mat tmp = s_Hwc;
				s_Hwc = Hwc;
				Hwc = tmp;

				tmp = s_K;
				s_K = K;
				K = tmp;

				s_R.swap( R );
				s_T.swap( T );

			};

			//Copy is done with no size/type check !!
			void dup_params(){
				Hwc.copyTo( s_Hwc );
				K.copyTo( s_K );
			
				for(int i=0;i<R.size();i++)
					R[i].copyTo( s_R[i] );

				for(int i=0;i<T.size();i++)
					T[i].copyTo( s_T[i] );

			};


	};


};

#endif
