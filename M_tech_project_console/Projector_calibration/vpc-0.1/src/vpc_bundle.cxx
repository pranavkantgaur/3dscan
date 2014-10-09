#include <iostream>
#include <algorithm>
#include <cmath>
#include <iterator>

#include "cv.h"
#include "highgui.h"

#include "vpc_bundle.h"


using namespace std;
using namespace cv;

namespace VPC{

	 void HTRKI_grad( Mat &_H , Mat &_K , Mat &_R , Mat &_t , Point3d &_Q_ , Point3d &_q_ ,
			 double *merit , Mat &grad_k , Mat &grad_rt , Mat &grad_h ){

		 double k[4];
		 k[0] = _K.at<double>(0,0);
		 k[1] = _K.at<double>(1,1) / _K.at<double>(0,0);
		 k[2] = _K.at<double>(0,2);
		 k[3] = _K.at<double>(1,2);

		 //Invert the intrinsic matrix.
		 Mat _KI = (Mat_<double>(3,3)<< 1.0 / k[0] , 0     , -k[2]/k[0],
				 0 , 1.0 / (k[1]*k[0])  , -k[3] / (k[0]*k[1]),
				 0 ,  0    ,  1 ); 

		 //Matrix of projector's position.
		 Mat _TI=(Mat_<double>(3,3) << 1 , 0 , -_t.at<double>(0,0) / _t.at<double>(2,0),
				 0          ,  1   ,  -_t.at<double>(1,0)/_t.at<double>(2,0) , 
				 0          ,				0     , 1.0/_t.at<double>(2,0) 
				 );

		 Mat _Q = (Mat_<double>(3,1)<< _Q_.x , _Q_.y,1.0);
		 Mat _q = (Mat_<double>(3,1)<< _q_.x , _q_.y,1.0);

		 //Backproject point from projector to the camera.
		 Mat _KIQ = _KI * _Q;
		 Mat _RKIQ = _R.t() * _KIQ;
		 Mat _TRKIQ = _TI * _RKIQ;
		 Mat _HTRKIQ = _H * _TRKIQ;

		 //Intermediate entities.
		 Mat _HTI  = _H * _TI;
		 Mat _HTRI = _HTI * _R.t();
		 Mat _HTR(3,3,CV_64F); invert( _HTRI, _HTR );
		 Mat _HTRIxKIQ = _HTRI*_KIQ;

		 _HTR *= determinant( _HTRI );

		 Mat _HI(3,3,CV_64F); invert( _H , _HI );

		 _HI *= determinant( _H );
		 Mat _tQ = _t.cross( _RKIQ );
		 double m;

		 double *Q , *q , *tQ , *H[3] , *HTR[3] ;
		 double *HTRI[3], *HI[3] , *RKIQ , *KIQ ;
		 double *TRKIQ , *HTRKIQ , *HTRIxKIQ ;
		 double *t;

		 Q 					= 		       _Q.ptr<double>(0);
		 t 					= 		       _t.ptr<double>(0);
		 q 					= 		       _q.ptr<double>(0);
		 tQ 				= 		      _tQ.ptr<double>(0);
		 KIQ 				= 		     _KIQ.ptr<double>(0);
		 RKIQ 				= 		    _RKIQ.ptr<double>(0);
		 TRKIQ 				= 		   _TRKIQ.ptr<double>(0);
		 HTRKIQ 			= 		  _HTRKIQ.ptr<double>(0);
		 HTRIxKIQ 			= 		_HTRIxKIQ.ptr<double>(0);

		 for(int i=0;i<3;i++) {
			 HTR[i] = _HTR.ptr<double>(i);
			 HTRI[i] = _HTRI.ptr<double>(i);
			 HI[i] = _HI.ptr<double>(i);
			 H[i] = _H.ptr<double>(i);
		 }

		 //Loop through the 2 coordinates.
		 for(int i=0;i<2;i++){
			/*Residual*/
			 	merit[i] = q[i] - HTRKIQ[i] / HTRKIQ[2];
			
			 /* Partial derivatives wrt intrinsic parameters */
				m= 1.0 / POW2(-k[0]*k[1]*HTRI[2][2]+k[1]*HTRI[2][0]*(k[2]-Q[0]) + HTRI[2][1]*(k[3]-Q[1]) );

				grad_k.at<double>( i , 0 )  = m * k[1] *(1-2*i)*( k[1]*(Q[0]-k[2])*HTR[1][1-i]  - (Q[1]-k[3])*HTR[0][1-i] );
				grad_k.at<double>( i , 1 )  = m * (1-2*i)*( (Q[0]-k[2])*( Q[1]*HTR[2][1-i]+k[3]*(-HTR[2][1-i])) - k[0]*(Q[1]-k[3])*HTR[0][1-i]  ); 
				grad_k.at<double>( i , 2 )  = m * k[1] * (1-2*i) * ( -(Q[1]-k[3])*HTR[2][1-i] + k[0]*k[1]* HTR[1][1-i] );
				grad_k.at<double>( i , 3 )  = m * k[1] * (1-2*i) * ( (Q[0]-k[2])*HTR[2][1-i] - k[0]*HTR[0][1-i] );

			/* Partial derivatives wrt rotation */
				m = 1.0 / POW2(HTRI[2][0]*KIQ[0] + HTRI[2][1]*KIQ[1] + HTRI[2][2]*KIQ[2]);
				grad_rt.at<double>( i,0 ) = m * ( ( KIQ[2]*HTRI[i][1]-KIQ[1]*HTRI[i][2])*HTRIxKIQ[2]+
												   HTRIxKIQ[i]*(KIQ[1]*HTRI[2][2]-KIQ[2]*HTRI[2][1])  );

				grad_rt.at<double>( i,1 ) = m * ( ( KIQ[0]*HTRI[i][2]-KIQ[2]*HTRI[i][0])*HTRIxKIQ[2]+
												   HTRIxKIQ[i]*(KIQ[2]*HTRI[2][0]-KIQ[0]*HTRI[2][2])  );

				grad_rt.at<double>( i,2 ) = m * ( ( KIQ[1]*HTRI[i][0]-KIQ[0]*HTRI[i][1])*HTRIxKIQ[2]+
												   HTRIxKIQ[i]*(KIQ[0]*HTRI[2][1]-KIQ[1]*HTRI[2][0])  );


			/* Partial derivatives wrt translation */
				m =  RKIQ[2] / POW2(H[2][0]*tQ[1] - H[2][1]*tQ[0] + H[2][2]*RKIQ[2]);
				grad_rt.at<double>( i,3 ) = m * (1-2*i) * (HI[2][1-i]*tQ[0]+HI[1][1-i]*RKIQ[2] );
				grad_rt.at<double>( i,4 ) = m * (1-2*i) * (HI[2][1-i]*tQ[1]-HI[0][1-i]*RKIQ[2] );
				grad_rt.at<double>( i,5 ) = m * (1-2*i) * (HI[2][1-i]*tQ[2]+RKIQ[1]*HI[0][1-i]-RKIQ[0]*HI[1][1-i]) ; 

			/* Partial derivatives wrt homography */
				m = 1.0 / HTRKIQ[2];

				for (int j=0; j<3; j++) {
					grad_h.at<double>(i,j+i*3)   = -m*TRKIQ[j];
					grad_h.at<double>(i,j+(1-i)*3) = 0;
					grad_h.at<double>(i,j+6)   =  m*m*TRKIQ[j]*HTRKIQ[i];
				}

		 }


	 }




}//namespace VPC

