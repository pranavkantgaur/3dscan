#include "autocalib.h"

using namespace std;
using namespace cv;

namespace VPC{

    AutoCalib::AutoCalib(vector< vector<Point3d> > &cam_pts , vector< vector<Point3d> > &proj_pts , 
            vector< vector<Point3d> > &cam_pts_fronto , vector< vector<Point3d> > &proj_pts_fronto , 
            double ar ) : HTRKI(cam_pts , proj_pts) , aspect_ratio(ar) {

        //a shallow-copy would be enough ??
        this->cam_pts_fronto = cam_pts_fronto ;
        this->proj_pts_fronto = proj_pts_fronto ;

        vector<Point2f> cam_2d , proj_2d;


        //...fronto-//
        Hcpf.reserve( nb_views() );
        for(int i=0;i<nb_fronto();i++){
            p3_to_p2( cam_pts_fronto[i] , cam_2d );
            p3_to_p2( proj_pts_fronto[i] , proj_2d );

            Mat _Hcp0=findHomography( Mat(cam_2d) , Mat(proj_2d) );
            Mat _Hcp(3,3,CV_64F ) ;
            _Hcp0.convertTo( _Hcp , CV_64F);

            Hcpf.push_back( _Hcp.clone() );

            Hcpf[i] /= Hcpf[i].at<double>(2,2);
        }

    };


    void AutoCalib::get_rt(vector<Mat> &Hinter){
        Mat Kinv = K.inv();

        //the homography is: H_i = K * R_i * T_i
        for(int view=0;view<nb_views();view++){

            //Init rots and trans
            R[view] = Mat::eye(3,3,CV_64F);
            T[view] = Mat::zeros(3,1,CV_64F);


            //Get rid of K and K^-1
            //H_ij = K * [r1 r2 t] * K^-1
            Mat rt = Kinv * Hinter[view] * K;

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


    void AutoCalib::calibrate( bool with_lm ){

        Mat _err_old = (Mat_<double>(1,3) << DBL_MAX , DBL_MAX , DBL_MIN);
        int best_fr;

        //Foreach fronto-// pose...
        for(int fr = 0; fr < nb_fronto();fr++){

            //perform an autocalib iteration
            do_autocalib( fr );

            //compute the rms.
            Mat _err = rms();

            //Keep track of the best fronto-//
            if( _err.at<double>(0,0) < _err_old.at<double>(0,0) ){
                _err.copyTo( _err_old ) ;
                best_fr = fr;
            }

        }

        //Set the best solution.
        do_autocalib( best_fr );

        //Set the correct aspect_ratio.
        if(aspect_ratio < 0 )
            aspect_ratio = K.at<double>(1,1) / K.at<double>(0,0);

        if( with_lm ) 
            lm();

    }

    void AutoCalib::do_autocalib(int ndx_fronto ){
        Mat Hpc_f ( Hcpf[ndx_fronto].inv() );
        vector<Mat> Hinter;

        Mat A;
        double params[5];

        int nh = nb_views();

        //Compute the inter-image homog:
        // fronto --> View_i
        for(int i=0;i<nb_views();i++){
            Hinter.push_back( Mat(Hcp[i] * Hpc_f).clone() );
            Hinter[i].convertTo(Hinter[i] , CV_64F);
        }

        if( aspect_ratio > 0 ){
            //Known aspect ratio
            A = Mat( nh * 2 , 4 , CV_64F );

            //Start with the linear eqs: h1t.W.h2 = 0
            for(int i=0;i<nh;i++){
                A.at<double>(i,0) = Hinter[i].at<double>(2,0) * Hinter[i].at<double>(0,1) +
                    Hinter[i].at<double>(0,0) * Hinter[i].at<double>(2,1) ;

                A.at<double>(i,1) = Hinter[i].at<double>(1,0) * Hinter[i].at<double>(1,1) +
                    Hinter[i].at<double>(0,0) * Hinter[i].at<double>(0,1) ;

                A.at<double>(i,2) = Hinter[i].at<double>(2,0) * Hinter[i].at<double>(1,1) +
                    Hinter[i].at<double>(1,0) * Hinter[i].at<double>(2,1) ;

                A.at<double>(i,3) = Hinter[i].at<double>(2,0) * Hinter[i].at<double>(2,1) ;

            }
            //The other linear eq: h1t.W.h1 - h2t.W.h2 = 0;
            for(int i=0;i<nh;i++){
                A.at<double>(i+nh,0) = 2 * ( Hinter[i].at<double>(0,0) * Hinter[i].at<double>(2,0) -
                        Hinter[i].at<double>(0,1) * Hinter[i].at<double>(2,1) ) ;

                A.at<double>(i+nh,1) = POW2( Hinter[i].at<double>(1,0) ) - POW2( Hinter[i].at<double>(1,1) ) +
                    POW2( Hinter[i].at<double>(0,0) ) -  POW2( Hinter[i].at<double>(0,1) ) ;

                A.at<double>(i+nh,2) = 2 * ( Hinter[i].at<double>(1,0) * Hinter[i].at<double>(2,0) -
                        Hinter[i].at<double>(1,1) * Hinter[i].at<double>(2,1) ) ;

                A.at<double>(i+nh,3) = POW2( Hinter[i].at<double>(2,0) ) - POW2( Hinter[i].at<double>(2,1) ) ;

            }



        }
        else{
            //Unknown aspect ratio
            //We use the linear eq: h1t.W.h2 = 0...only.

            A = Mat( nh  , 5 , CV_64F );

            for( int i = 0; i < nh; i++){
                A.at<double>(i,0) = Hinter[i].at<double>(0,0) * Hinter[i].at<double>(0,1) ;

                A.at<double>(i,1) = Hinter[i].at<double>(2,0) * Hinter[i].at<double>(0,1) +
                    Hinter[i].at<double>(0,0) * Hinter[i].at<double>(2,1) ;

                A.at<double>(i,2) = Hinter[i].at<double>(1,0) * Hinter[i].at<double>(1,1) ;

                A.at<double>(i,3) = Hinter[i].at<double>(2,0) * Hinter[i].at<double>(1,1) +
                    Hinter[i].at<double>(1,0) * Hinter[i].at<double>(2,1) ;

                A.at<double>(i,4) = Hinter[i].at<double>(2,0) * Hinter[i].at<double>(2,1) ;


            }

        }

        //Precondition
        Mat lambda=Mat::zeros(1,A.cols,CV_64F);

        for(int i=0;i<A.rows;i++)
            for(int j=0;j<A.cols;j++)
                lambda.at<double>(0,j) += POW2( A.at<double>(i,j) );

        for(int j=0;j<A.cols;j++)
            lambda.at<double>(0,j) = sqrt( lambda.at<double>(0,j) );

        for(int i=0;i<A.rows;i++)
            for(int j=0;j<A.cols;j++)
                A.at<double>(i,j)  /= lambda.at<double>(0,j);

        SVD svdObj(A);
        Mat sol = svdObj.vt.row( A.cols - 1 );

        for(int i=0;i<sol.cols;i++)
            sol.at<double>(0,i) /= lambda.at<double>(0,i);

        double *ptr = sol.ptr<double>( 0 );



        if( aspect_ratio > 0 ){
            double nrm = ptr[1];
            for(int i=0;i<A.cols;i++){
                ptr[i] /= nrm;

            }

            //rho
            params[0]  = aspect_ratio;

            //u0
            params[1]  = -ptr[0] / POW2(params[0]); 

            //v0
            params[2]  = -ptr[2];

            //fy
            params[3]  = sqrt( (ptr[3] - (POW2(params[0]*params[1]) + POW2(params[2])) ) / POW2(params[0]) );

            //fx....f*rho
            params[4] =  params[0] * params[3];


        }
        else{
            double nrm = ptr[2];
            for(int i=0;i<A.cols;i++)
                ptr[i] /= nrm;

            //rho
            params[0] = sqrt( ptr[0] );

            //u0
            params[1] = - ptr[1] / (POW2(params[0]));

            //v0
            params[2] = -ptr[3];

            //fy...
            params[3] = sqrt( (ptr[4]-(POW2(params[0]*params[1]) + POW2(params[2])) ) / POW2(params[0]) );

            //fx....f*rho
            params[4] =  params[0] * params[3];

        }

        K = Mat::eye(3,3,CV_64F );

        K.at<double>(0,0) = params[4];
        K.at<double>(0,2) = params[1];
        K.at<double>(1,1) = params[3];
        K.at<double>(1,2) = params[2];


        //Compute the homography world --> camera
        Hwc = Hpc_f * K;

        //Computer extrinsics.
        get_rt( Hinter );
    }

    void AutoCalib::lm_finalize(){
        aspect_ratio = K.at<double>(1,1) / K.at<double>(0,0);
    }


};

