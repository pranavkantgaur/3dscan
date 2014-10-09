#ifndef _AUTOCALIB_H__
#define _AUTOCALIB_H__

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
	class AutoCalib : public HTRKI{
		public:
			double aspect_ratio;

			//Fronto-// correspondances.
			vector< vector<Point3d> > cam_pts_fronto;
			vector< vector<Point3d> > proj_pts_fronto;

			//TODO: move Hcp to HTRKI.h
			//Homographies (cam-->proj and fronto-//)
			vector<Mat> Hcpf;

			void calibrate( bool with_lm );
			void get_rt(vector<Mat> &Hinter);
	
			void lm_finalize();

			//Returns number of fronto-// views
			int nb_fronto(){ return cam_pts_fronto.size(); }

			AutoCalib( vector< vector<Point3d> > &cam_pts , vector< vector<Point3d> > &proj_pts , 
					   vector< vector<Point3d> > &cam_pts_fronto , vector< vector<Point3d> > &proj_pts_fronto , 
					   double ar );

			~AutoCalib(){};

		private:
			void do_autocalib(int ndx_fronto );


	};

}

#endif
