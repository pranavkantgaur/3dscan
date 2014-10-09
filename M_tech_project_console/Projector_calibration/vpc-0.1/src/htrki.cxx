#include "htrki.h"


using namespace std;
using namespace cv;

namespace VPC{

	HTRKI::HTRKI( vector< vector<Point3d> > &cam_pts , vector< vector<Point3d> > &proj_pts ){
		 //a shallow-copy would be enough ??
		 this->cam_pts =  cam_pts ;
		 this->proj_pts =  proj_pts ;
		 this->Hwc = Mat::eye( 3,3,CV_64F );

		 R.resize( nb_views() );
		 T.resize( nb_views() );

                 // we have nb_views *(R + trans + nb_pt)+ 4 intrinsic + 9 homog - (global_pos+global_scale+global orient)
		 nb_param_to_optimize = nb_views() * (6+cam_pts[0].size() ) + 4 + 9 - 5;

		 Hcp.reserve( nb_views() );

		 //Estimate the homographies camera-->projector
		 vector<Point2f> cam_2d , proj_2d;
		 for(int i=0;i<nb_views();i++){
			 p3_to_p2( cam_pts[i] , cam_2d );
			 p3_to_p2( proj_pts[i] , proj_2d );

			 Mat _Hcp0=findHomography( Mat(cam_2d) , Mat(proj_2d) );
			 Mat _Hcp(3,3,CV_64F ) ;
			 _Hcp0.convertTo( _Hcp , CV_64F);

			 Hcp.push_back( _Hcp.clone() );

			 Hcp[i] /= Hcp[i].at<double>(2,2);
		 }


	}



	Mat HTRKI::rms( ){
		 Mat err = (Mat_<double>(4,1) << 0 , 0 , DBL_MAX , DBL_MIN);

		if( T.size() != nb_views() || R.size() != nb_views() ){
			cout<<"VPC: Extrinsic parameters are not (completly) computed.\n";
			Mat err = (Mat_<double>(3,1) << -1,-1,-1);

			return err;
		}

		 //Invert the intrinsic matrix.
		 Mat _KI = K.inv();

		 int n_pt = 0;
		 for(int v=0;v<nb_views();v++){
			 //Matrix of projector's position.
			 Mat _TI=( Mat_<double>(3,3) <<  1  ,  0  , -T[v].at<double>(0,0) / T[v].at<double>(2,0),
                                                         0  ,  1  ,  -T[v].at<double>(1,0)/T[v].at<double>(2,0) ,
                                                         0  ,  0  ,  1.0/T[v].at<double>(2,0)
                                                         );


                         //Backproject point from projector to the camera.
                         Mat _HTRKI = Hwc * _TI * R[v].t() * _KI;

                         for(int p=0;p<cam_pts[v].size();p++){

                             Mat _HTRKIQ = _HTRKI * Mat(proj_pts[v][p]);
                             _HTRKIQ /= _HTRKIQ.at<double>(2,0);

                             //Loop through the 2 coordinates.
                             double rep_err;
                             rep_err = 0 ;

                             Mat rep = Mat(cam_pts[v][p]) - _HTRKIQ;

                             rep_err = norm( rep );

                             if( rep_err < err.at<double>(2,0) )
                                 err.at<double>(2,0) = rep_err;

                             if( rep_err > err.at<double>(3,0) )
                                 err.at<double>(3,0) = rep_err;

                             err.at<double>(0,0) += fabs( rep_err );
                             err.at<double>(1,0) += rep_err ;
                             n_pt++;
                         }
                 }

		 err.at<double>(1,0) = err.at<double>(1,0) / n_pt ;
		 err.at<double>(2,0) = err.at<double>(2,0) ;
		 err.at<double>(3,0) = err.at<double>(3,0) ;

		 return err;

	 }

	 void HTRKI::jacobian( Mat &J , Mat &E){
	 	 int n_eq , ndx;
		 double merit[2] ;
		 Mat grad_k(2,4,CV_64F) , grad_rt(2,6,CV_64F) , grad_h(2,9,CV_64F) 	;
		 int n_img = nb_views();
		 int n_params = n_img * 6 + 9 + 4;
		 int n_pts = n_img * cam_pts[0].size();
		 int n_fns = 2 *  n_pts ;

		 J = Mat::zeros( n_fns , n_params , CV_64F );
		 E = Mat::zeros( n_fns , 1 , CV_64F );

		 n_eq = 0;

		 for(int v=0;v<nb_views();v++){
			 for(int p = 0; p < cam_pts[v].size();p++){
				 //Compute the gradients.
				 HTRKI_grad( Hwc , K , R[v] , T[v] , proj_pts[v][p] , cam_pts[v][p] , merit , grad_k , grad_rt , grad_h  );

				 //Copy gradients to the Jacobian matrix.
				 for(int c=0;c<2;c++){

					 for(int d=0;d<4;d++)
						 J.at<double>(2*n_eq+c, d ) = grad_k.at<double>(c,d);

					 for(int d=0;d<9;d++)
						 J.at<double>(2*n_eq+c, d+4 ) = grad_h.at<double>(c,d);

					 for(int d=0;d<6;d++)
						 J.at<double>(2*n_eq+c, 13 + 6*v + d) = grad_rt.at<double>(c,d);

					 //Copy the merit to E.
					 E.at<double>(2*n_eq+c , 0 ) = merit[c];
				 }


				 n_eq++;
			 }
		 }

                /*Parameters fixed when fixing the gauge freedom */
                /*should not be optimized.*/

                //First projector (translation).
                 int v=0;
                 for(int d=3;d<6;d++)
                     for(int c=0;c<2;c++)
                         J.at<double>(2*v+c, 13 + 6*v + d) = 0.0;

                 //Second projector (X translation)
                 v=1;
                 for(int c=0;c<2;c++)
                     J.at<double>(2*v+c, 13 + 6*v + 3) = 0.0;

                //Locate the entry of Hwc that remains constant (the max)
                double mm;
                Point max_loc;

                minMaxLoc( Hwc , &mm , &mm , &max_loc , &max_loc );

                int max_ndx = max_loc.x * 3 + max_loc.y;
                for(v=0;v<J.rows;v++)
                        J.at<double>(v, max_ndx+4 ) = 0.0;

	 }

        void HTRKI::fix_gauge(){

            //The first projector fixes the global translation/scale
            Mat T0 = T[0].clone();
            int nv = nb_views();

            //Gauge Matrix (gm).
            Mat gm = (Mat_<double>(3,3) << T0.at<double>(2,0) , 0     , -T0.at<double>(0,0),
                                            0.0  ,  T0.at<double>(2,0) , -T0.at<double>(1,0),
                                            0.0  ,           0.0      ,  1.0);

            for(int view=0; view<nv; view++){
                //Modify positions accordingly.
                T[view].rowRange(0,2) -= T0.rowRange(0,2);

                //Modify scales accordingly.
                T[view] /= T0.at<double>(2,0);
            }

            //Adapt the homography world-->camera
            Mat Htmp = Hwc * gm;

            //The second projector fixes the rotation around the plane normal.
            double alpha = atan2( T[1].at<double>(0,0) ,  T[1].at<double>(1,0) );
            Mat gm2 = (Mat_<double>(3,3) << cos(alpha) , -sin(alpha)     , 0,
                    sin(alpha)  , cos(alpha) ,  0,
                    0.0  ,           0.0      ,  1.0);

            //Adapt translations and rotations.
            for(int view=0; view<nv; view++){
                T[view] = gm2.t() * T[view];
                R[view] = R[view] * gm2;
            }

            //modify the homog world-->cam accordingly.
            Hwc = Htmp * gm2;

        }

        void HTRKI::lm_init(){
            /*Fix the gauge parameters and put  projectors in */
            /*canonical form...from Peter Sturm's code. */
            fix_gauge();

            //Initialize (and copy) the
            //alternate (swappable) params
            s_Hwc	=Hwc.clone();
            s_K	=K.clone();

            s_R.resize( R.size() );
            s_T.resize( T.size() );

            for(int i=0;i<R.size();i++)
                s_R[i] = R[i].clone();

            for(int i=0;i<T.size();i++)
                s_T[i] = T[i].clone();


        }

        void HTRKI::lm_finalize(){

        }

        double HTRKI::try_update( const Mat &delta){
            //The alternate params are
            //assumed up to date.
            dup_params();

            int n_views = nb_views();
            double f_u = s_K.at<double>(0,0) ;
            double f_v = s_K.at<double>(1,1) ;
            double ar = f_v / f_u;

            //f_u.
            s_K.at<double>(0,0) += delta.at<double>(0,0);

            //f_v...= f_u * aspect
            ar += delta.at<double>(1,0);
            s_K.at<double>(1,1)  = s_K.at<double>(0,0) * ar ;

            //u_0
            s_K.at<double>(0,2) += delta.at<double>(2,0);

            //v_0
            s_K.at<double>(1,2) += delta.at<double>(3,0);

            //Hwc
            for(int i=0;i<9;i++){
                s_Hwc.at<double>(i/3 , i%3) += delta.at<double>( 4 + i , 0 );
            }

            //Extrinsic.
            Mat rtvec(3,1,CV_64F);
            Mat rmat(3,3,CV_64F);
            for(int i=0;i<n_views;i++){
                //Extract the rotation update.
                delta( Range( 13 + i*6 , 13 + i*6 + 3 ) , Range(0,1) ).copyTo( rtvec );
                Rodrigues( rtvec , rmat );
                s_R[i] = rmat.t()*s_R[i];

                //Extract the translation update.
                delta( Range( 13 + i*6 + 3 , 13 + i*6 + 6 ) , Range(0,1) ).copyTo( rtvec );
                s_T[i] += rtvec;
            }

            //Swap params.
            swap_params();

            //Compute new error.
            Mat err = rms();

            //Swap back.
            swap_params();

            //Estimate the new error.
            return err.at<double>(0,0);
        };


        double HTRKI::apply_update( const Mat &delta){ 

            swap_params();

            return 0;

        };

}
