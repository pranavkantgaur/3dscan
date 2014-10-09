#include <iostream>
#include <algorithm>
#include <cmath>
#include <iterator>

#include "cv.h"
#include "highgui.h"

#include "vpc_base.h"


using namespace std;
using namespace cv;

namespace VPC{

	void p3_to_p2( vector<Point3d> &src , vector<Point2f> &dst){

		dst.clear();

		for(int i=0;i<src.size();i++)
			dst.push_back( Point2f(  src[i].x , src[i].y ) );

	}

	 void Calib::lm( int MAX_ITER  , double epsilon ){
		 int n_iter = 0;
		 double lambda = 0.0001;
		 bool improves = true;
		 bool error_criterion = false;
		 Mat J , E ;
		 double old_rms;


		//Call initialization code.
		lm_init();

		Mat err_vec = this->rms( );
		old_rms = err_vec.at<double>(0,0);

		while( n_iter < MAX_ITER && !error_criterion){

			 //Compute gradients.
			 if( improves )
				 jacobian( J , E );

			 //Compute the update.
			 Mat JtE = J.t() * E;
			 Mat delta = -1.0 * (J.t()*J + lambda* Mat::eye(J.cols, J.cols, J.type())).inv()*(JtE);
			 //...and try it
			 double rms = this->try_update( delta );

			 // Did the error improve ??
			 if( rms < old_rms ){
				 cout<<".";
                                 cout.flush();

				 lambda /= 10;
				 this->apply_update( delta );
				 old_rms = rms;

				 improves = true;
			 }else{
				 lambda *= 10;
				 improves = false;
			 }

			 //Check stop criteria
			 if(  norm( E ) < epsilon  || norm( JtE) < epsilon || norm( delta ) < epsilon)
				 error_criterion = true;


			 n_iter++;

		 }


                cout<<endl;

		//Finish properly.
		lm_finalize();

	 }


}//namespace


/*
namespace{
	int const formatted_flag_index   = std::ios_base::xalloc();
	int const format_string_index   = std::ios_base::xalloc();

	bool is_formatted(std::ostream& os) { return (0 != os.iword(formatted_flag_index)); }
	char* const format_string(std::ostream& os) { return  (char*)os.pword(format_string_index) ; }
};

VpcFormatter vpc_fmt( const char fmt[]){ return VpcFormatter( fmt ); }

void VpcFormatter::set_ostream_info(std::ostream& os) const {
	os.iword(formatted_flag_index ) = true;
	os.pword( format_string_index ) = (void*)fmt;
}

std::ostream& operator<<( std::ostream& os, const VpcFormatter &formatter) {
	formatter.set_ostream_info(os);
	return os;
}

std::ostream& operator<<(std::ostream &os, const _VPCSOL_ &sol){

	if(is_formatted(os) ){
		char *fmt = (char*) os.pword( format_string_index );
		int i=0;

		while( fmt[i] != '\0' ){
			switch (fmt[i]){

				case 'X':
					os<<"Extrinsic:\n";
					for(int v=0; v < sol.nb_views(); v++){
						os<<sol.R[v]<<endl;
						os<<sol.T[v]<<endl<<endl;
					}

					break;

				case 'R':
					os<<"Rotations:\n";
					for(int v=0; v < sol.nb_views(); v++)
						os<<sol.R[v]<<endl;

					break;

				case 'T':
					os<<"Translations:\n";
					for(int v=0; v < sol.nb_views(); v++)
						os<<sol.T[v]<<endl;

					break;

				case 'E':
					os<<"Error [avg,min,max]:\n";
					os<<sol.rep_err<<endl;

					break;

				case 'K':
					os<<"Intrinsic :\n";
					os<<sol.K<<endl;

					break;

				case 'H':
					os<<"Homography :\n";
					os<<sol.Hwc<<endl;

					break;

				default:
					os<<fmt[i];

			}

				i++;
		}

	}else
		os<<sol.K<<endl<<"Error: "<<sol.rep_err<<endl;

	os.iword(formatted_flag_index ) = false;

	return os;
}


*/
