#ifndef _SAMPLING_H__
#define _SAMPLING_H__

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

#include "htrki.h"
#include "direct.h"

using namespace std;
using namespace cv;

namespace VPC{

	class Sampling : public HTRKI{
		public:

			//The object used to direct calib.
			Direct calibObj; 

			//Camera's intrinsic...(optional)
			Mat Kcam;

			//Focal range or camera intrinsic must be provided.
			double focal_min , focal_max;
			

			void calibrate( bool with_lm );

			Sampling( vector< vector<Point3d> > &cam_pts , vector< vector<Point3d> > &proj_pts , Mat &Kcam);
			Sampling( vector< vector<Point3d> > &cam_pts , vector< vector<Point3d> > &proj_pts , 
					  double focal_min , double focal_max , double u0 , double v0);
			~Sampling(){};

		private:
			bool have_kcam;
	};

}

#endif
