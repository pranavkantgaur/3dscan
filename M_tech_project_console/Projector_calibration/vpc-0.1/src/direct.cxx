#include "direct.h"

using namespace std;
using namespace cv;

namespace VPC{
	 Direct::Direct( vector< vector<Point3d> > &cam_pts , vector< vector<Point3d> > &proj_pts , Mat &Hwc)
	 : HTRKI(cam_pts , proj_pts) {

		 Hwc.copyTo( this->Hwc );
		
		 Hwp.resize( nb_views() );

		 vector<Point2f> cam_2d , proj_2d;
		 for(int i=0;i<nb_views();i++){
			 Hwp[i] = ( Hcp[i] * Hwc );

		 }

	 }

	 //CAUTION!!!: here Hwc=I...it's a hack for Sampling method.
	 Direct::Direct( vector< vector<Point3d> > &cam_pts , vector< vector<Point3d> > &proj_pts )
	 : HTRKI(cam_pts , proj_pts) {

		 Hwp.resize( nb_views() );
		 _Hwp.resize( nb_views() );

		 vector<Point2f> cam_2d , proj_2d;
		 for(int i=0;i<nb_views();i++){
			 _Hwp[i] = ( Hcp[i] * Hwc );

		 }

	 }

	Direct::Direct( vector<Mat> &Hwp) {
		Hwp.resize( nb_views() );

		for(int i=0;i<Hwp.size();i++)
			this->Hwp[i] = Hwp[i].clone() ;

	}


	 void Direct::set_hwc( Mat &Hwc_new ){

		 for(int i=0;i<nb_views();i++)
			 Hwp[i] = _Hwp[i] * Hwc_new ;
            
		 Hwc_new.copyTo( Hwc );
	 }


	void Direct::calibrate( bool with_lm ){
		Mat A( nb_views() * 2 , 5 , CV_64F);

		for(int h=0;h<nb_views();h++){

			//Fill the orthogonality constraints.
			A.at<double>(2*h,0) = Hwp[h].at<double>(0,0) * Hwp[h].at<double>(0,1);

			A.at<double>(2*h,1) = Hwp[h].at<double>(0,1) * Hwp[h].at<double>(2,0) + 
				Hwp[h].at<double>(0,0) * Hwp[h].at<double>(2,1);

			A.at<double>(2*h,2) = Hwp[h].at<double>(1,0) * Hwp[h].at<double>(1,1);

			A.at<double>(2*h,3) = Hwp[h].at<double>(1,1) * Hwp[h].at<double>(2,0) + 
				Hwp[h].at<double>(1,0) * Hwp[h].at<double>(2,1);

			A.at<double>(2*h,4) = Hwp[h].at<double>(2,0) * Hwp[h].at<double>(2,1);


			//Fill the length equity constraints.
			A.at<double>(2*h+1,0) = POW2(Hwp[h].at<double>(0,0) ) - POW2( Hwp[h].at<double>(0,1) );

			A.at<double>(2*h+1,1) = 2*( Hwp[h].at<double>(0,0) * Hwp[h].at<double>(2,0)
					- 
					Hwp[h].at<double>(0,1) * Hwp[h].at<double>(2,1) );

			A.at<double>(2*h+1,2) = POW2(Hwp[h].at<double>(1,0) ) - POW2(Hwp[h].at<double>(1,1) );

			A.at<double>(2*h+1,3) = 2*( Hwp[h].at<double>(1,0) * Hwp[h].at<double>(2,0)
					- 
					Hwp[h].at<double>(1,1) * Hwp[h].at<double>(2,1) );

			A.at<double>(2*h+1,4) = POW2(Hwp[h].at<double>(2,0) ) - POW2(Hwp[h].at<double>(2,1) );

		}

		SVD svdObj(A);
		double *ptr = svdObj.vt.ptr<double>(4);

		K = Mat::eye(3,3,CV_64F);

		double p3= sqrt( (ptr[0]*ptr[2]*ptr[4] - ptr[2] * POW2(ptr[1]) - ptr[0] * POW2(ptr[3] ) )
				/
				( ptr[0] * POW2(ptr[2]) ) );


		K.at<double>(0,0) = sqrt(ptr[2]/(ptr[0])) *p3;
		K.at<double>(1,1) = p3;
		K.at<double>(0,2) =  - ptr[1] / ptr[0];
		K.at<double>(1,2) =  - ptr[3] / ptr[2];

		get_rt( );
	
		if( with_lm ) 
			lm();
	}


	void Direct::get_rt(){
		 Mat Kinv = K.inv();

		 //The homography is: H_i = K * R_i * T_i
		 for(int view=0;view<nb_views();view++){

			 //Init rots and trans
			 R[view] = Mat::eye(3,3,CV_64F);
			 T[view] = Mat::zeros(3,1,CV_64F);


			 //Get rid of K and K^-1
			 //H_ij = K * [r1 r2 t] * K^-1
			 Mat rt = Kinv * Hwp[view] ;
			 SVD svdObj( rt.colRange(0,2) );
			 Mat _r23 = svdObj.vt.t()*svdObj.u.t();
			 Mat _r3 = _r23.row(0).cross( _r23.row(1) );
			 _r23.push_back( _r3 );

			 Mat(_r23.t()).copyTo( R[view] );
			 if( determinant( R[view] ) < 0.0 )
				 R[view] *= -1;

			 double s1 , s2;
			 s1 = s2 = 0.0;

			 for(int i=0;i<3;i++)
				 for(int j=0;j<2;j++){
					 s1 += R[view].at<double>(i,j) * rt.at<double>(i,j);
					 s2 += POW2( rt.at<double>(i,j) );
				 }

			 Mat( R[view].t() * (-s1/s2)*rt.col(2) ).copyTo( T[view] );

			 if( T[view].at<double>(2,0) < 0.0 ){
				 T[view].at<double>(2,0) *= -1;
				 R[view].col(0) *= -1;
				 R[view].col(1) *= -1;
			 }

			 T[view] *= -1;
		 }


	}




}

