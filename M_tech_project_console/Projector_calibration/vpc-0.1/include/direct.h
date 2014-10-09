#ifndef _DIRECT_H__
#define _DIRECT_H__


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

using namespace std;
using namespace cv;

namespace VPC{

	class Direct : public HTRKI{
		public:
			//Homog world-->projectors.
			vector<Mat> Hwp;

			//Hack for sampling.h
			vector<Mat> _Hwp;

			void calibrate( bool with_lm );
			void get_rt();

		
			void set_hwc( Mat &Hwc_new );

			Direct( vector< vector<Point3d> > &cam_pts , vector< vector<Point3d> > &proj_pts , Mat &Hwc);
			Direct( vector< vector<Point3d> > &cam_pts , vector< vector<Point3d> > &proj_pts );
			Direct( vector<Mat> &Hwp) ;
			~Direct(){};
	};

}

#endif
