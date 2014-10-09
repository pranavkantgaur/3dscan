#include <iostream>
#include <algorithm>
#include <cmath>
#include <iterator>

#include <cv.h>
#include <highgui.h>


using namespace std;
using namespace cv;

vector<Mat> img_org , img_bw;
vector<Point2f> prim_pts;
vector< vector<Point2f> > image_pts;

Mat frame;
const int PREF_W = 800;
float zoom_fact ;
int feat_rad = 7;

int nb_cols;
int nb_rows;
int col_0,row_0;
int feat_w , feat_h;

void usage( char *prog ){

	printf("Usage: %s --geometry COLSxROWS+COL_0xROW_0+FEAT_WIDTHxFEAT_HEIGHT img0 img1... \n" , prog);
	exit(1);
}

char* path_prefix(char *path ){

	int sz = strlen(path);
	int i;
	for(i=sz-1;i>=0;i--)
		if(path[i]=='.')
			break;


	return strndup( path , (i==0)?sz:i );
}

void rgb2bw( const vector<Mat> &rgb , vector<Mat> &bw){

	//Avoid mess.
	bw.resize(0);

	for(int i=0; i<rgb.size(); i++){
		bw.push_back( Mat(rgb[i].size() , CV_8UC1 ) );
		cvtColor( rgb[i] ,  bw[i] , CV_BGR2GRAY);
	}

}


void process_mouse(int event, int x, int y, int flags , void *params ){

	if( event == CV_EVENT_LBUTTONDOWN ){
		circle( frame , Point(x,y) , feat_rad , CV_RGB(255,0,0) , 2 );
		imshow("win" , frame );
		prim_pts.push_back( Point2f(x,y) * zoom_fact );
	}

}

//H*src = dst....src=[N,3]
void applyHomog( const Mat &H , const Mat &src , Mat &dst ){


	for(int r = 0; r < src.rows; r++ ){
		Mat _res = H*src.row( r ).t();

		dst.at<double>(r,0) = _res.at<double>(0,0) / _res.at<double>(2,0);
		dst.at<double>(r,1) = _res.at<double>(1,0) / _res.at<double>(2,0);
		dst.at<double>(r,2) = _res.at<double>(2,0) / _res.at<double>(2,0);
	}


}

void applyHomog( const Mat &H , const Mat &src , vector<Point2f> &dst ){

	dst.resize(0);

	for(int r = 0; r < src.rows; r++ ){
		Mat _res = H*src.row( r ).t();

		dst.push_back( Point2f(
					_res.at<double>(0,0) / _res.at<double>(2,0) ,
					_res.at<double>(1,0) / _res.at<double>(2,0) 
					)
				);
	}


}

int main( int argc , char* argv[]){


	bool geometry_set = false;
	nb_cols = nb_rows = col_0=row_0=feat_w=feat_h = 0;

	int offset_img=-1;

	for(int i=1; i<argc; i++){

		if( strcmp( argv[i] , "--geometry") == 0 && i+1<argc ){
			sscanf(argv[i+1] , "%dx%d+%dx%d+%dx%d" , &nb_cols , &nb_rows , 
					&col_0 , &row_0, 
					&feat_w , &feat_h);
			geometry_set = true;
			i++;
			continue;
		}

		if( offset_img == -1 )
			offset_img = i;

		Mat tmp = imread( argv[i] );
		if( !tmp.data )
			continue;

		img_org.push_back( Mat(tmp) );
	}

	if( !geometry_set )
		usage( argv[0] );

	printf("Using...(cols,rows) \n"
			"\tBoard: %dx%d\n"
			"\tFirst Feature: %d,%d\n"
			"\tSquare Geometry: %dx%d\n"
			"\n\n"
			"Shortcut Keys:\n"
			"0     - clear features.\n"
			"Space - compute homography.\n"
			"S     - skip the current frame.\n"
			"Q     - quit the program.\n"
			, nb_cols , nb_rows , col_0 , row_0, feat_w , feat_h
		  );

	rgb2bw( img_org , img_bw );


	vector<Point2f> board_pts;
	board_pts.push_back(Point2f(0,0) );
	board_pts.push_back(Point2f(nb_cols-1,0) );
	board_pts.push_back(Point2f(nb_cols-1,nb_rows-1) );
	board_pts.push_back(Point2f(0,nb_rows-1) );

	Mat src_pts(nb_cols*nb_rows,3,CV_64F);
	vector<Point2f> tmp_board_pts;
	int nb=0;
	for(int i=0;i<nb_cols;i++)
		for(int j=0;j<nb_rows;j++){
			src_pts.at<double>(nb,0) = i;
			src_pts.at<double>(nb,1) = j;
			src_pts.at<double>(nb,2) = 1;

			tmp_board_pts.push_back( Point2f(col_0+feat_w*i,row_0+feat_h*j) );
			nb++;
		}

	namedWindow("win" , 0);
	cvSetMouseCallback( "win" , process_mouse , 0);

	Size sz = img_org[0].size();
	Size zoomed_sz( PREF_W , (PREF_W*sz.height)/sz.width);
	zoom_fact = float(sz.width) / PREF_W;
	cout<<zoom_fact<<endl;

	int ID = 0;
	for(int i=0;i<img_bw.size();i++){

		prim_pts.resize(0);
		cout<<"Checking image "<<i+1<<"/"<<img_bw.size()<<endl;
		resize( img_org[i] , frame , zoomed_sz );
		imshow("win" , frame );

		bool finished=false;

		while( !finished ){
			char key=waitKey(10);

			switch(key){
				case '0':
					//Clear;
					prim_pts.resize(0);
					resize( img_org[i] , frame , zoomed_sz );
					break;

				case ' ':
					//Compute homog
					if( prim_pts.size() == 4 ){

						//Estimate initial homog.
						Mat H0=findHomography( Mat(board_pts) , Mat(prim_pts) );

						//compute initial corners coords.
						applyHomog( H0 , src_pts , prim_pts );

						cornerSubPix(img_bw[i], prim_pts, 
								Size(31, 31), Size(-1, -1), 
								TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1)
								);

						resize( img_org[i] , frame , zoomed_sz );
						for(int f=0;f<prim_pts.size();f++)
							circle( frame , prim_pts[f] * (1.0/zoom_fact)  , feat_rad , CV_RGB(255,0,0) , 2 );

						imshow("win" , frame );
						waitKey(0);
						Mat Hfinal=findHomography( Mat(tmp_board_pts) , Mat(prim_pts) );

						//Save homogs info
						char path[512];
						char *prefix = path_prefix(argv[offset_img + i]);
						sprintf(path,"%s.yml" , prefix);
						FileStorage fs( path , FileStorage::WRITE );

						fs<<"H"<<Hfinal.inv();
						fs<<"CAM_PTS"<<Mat(prim_pts).reshape(1);
						fs<<"PROJ_PTS"<<Mat(tmp_board_pts).reshape(1);

						cout<<Hfinal.inv()<<endl;

						ID++;
						finished = true;
					}else{
						cout<<"WARNING: 4pts needed.\n";
					}

					break;

				case 's':
					//Skip
					finished = true;
					break;

				case 'q':case 27:
					return 0;
					break;
			}


		}
	}


	Mat dist;

	return 0;
}

