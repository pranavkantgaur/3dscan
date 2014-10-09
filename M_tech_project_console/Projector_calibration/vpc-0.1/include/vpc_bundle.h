#ifndef __VPC_BUNDLE_H
#define __VPC_BUNDLE_H

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

using namespace std;
using namespace cv;



namespace VPC{

	 void HTRKI_grad( Mat &_H , Mat &_K , Mat &_R , Mat &_t , Point3d &_Q_ , Point3d &_q_ ,
			 double *merit , Mat &grad_k , Mat &grad_rt , Mat &grad_H );


}//namespace VPC


#endif
