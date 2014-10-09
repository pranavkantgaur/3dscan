/*
Steps:
Phase unwrapping:-
  For every pixel:
     i).Read the wrapped phase(vertical or horizontal)
    ii).Read the corresponding binary code.
   iii).Compute unwrapped phase as:phi=wrapped_phi+2*Pi*K
        where 'K' is index of binary code.

*/

#include "../PROJECT_GLOBAL/global_cv.h"

#define INVALID_VALUE 1000000.0f
#define THRESH_VERT 0
#define THRESH_HORZ 0

extern float (*wrapped_phi_vertical)[Camera_imageheight];
extern float (*wrapped_phi_horizontal)[Camera_imageheight];

extern float (*unwrapped_phi_vertical)[Camera_imageheight];
extern float (*unwrapped_phi_horizontal)[Camera_imageheight];


extern int number_of_patterns_binary_vertical;
extern int number_of_patterns_binary_horizontal;


extern int (*code_vertical)[Camera_imageheight];//it will contain the code for each pixel to be used in phase-unwrapping.
extern int (*code_horizontal)[Camera_imageheight];

extern char filename[150];

extern int fringe_width_pixels_vertical;
extern int fringe_width_pixels_horizontal;

extern int number_of_patterns_fringe;

extern int (*valid_map_vertical)[Camera_imageheight];
extern int (*valid_map_horizontal)[Camera_imageheight];

extern int number_of_codes_vertical;
extern int number_of_codes_horizontal;

IplImage**gray_images_binary_codes;
IplImage**gray_images_gray_codes;
IplImage**gray_images_gray_codes_inverse;

unsigned int count;

void read_captured_images(int pattern_type)
{
    if(pattern_type==0)
    {
        if(count==0)
            gray_images_binary_codes=new IplImage*[number_of_patterns_binary_vertical+1];
        else
            {
                gray_images_gray_codes=new IplImage*[number_of_patterns_binary_vertical+1];
                gray_images_gray_codes_inverse=new IplImage*[number_of_patterns_binary_vertical+1];

            }
        for(int num_pattern=0; num_pattern<(number_of_patterns_binary_vertical+1); num_pattern++)
        {
            if(count==0)
            {
                //binary codes
                sprintf(filename,"Captured_patterns/Coded_patterns/Binary_coded/Vertical/Undistorted/Captured_image_%d.bmp",num_pattern);
                gray_images_binary_codes[num_pattern]=cvLoadImage(filename,CV_LOAD_IMAGE_GRAYSCALE);
                sprintf(filename,"Captured_patterns/Coded_patterns/Binary_coded/Vertical/Undistorted/Gray_captured_image_%d.bmp",num_pattern);
                cvSaveImage(filename,gray_images_binary_codes[num_pattern]);
            }

            else
            {
                //gray codes
                sprintf(filename,"Captured_patterns/Coded_patterns/Gray_coded/Vertical/Undistorted/Captured_image_%d.bmp",num_pattern);
                gray_images_gray_codes[num_pattern]=cvLoadImage(filename,CV_LOAD_IMAGE_GRAYSCALE);
                sprintf(filename,"Captured_patterns/Coded_patterns/Gray_coded/Vertical/Undistorted/Gray_captured_image_%d.bmp",num_pattern);
                cvSaveImage(filename,gray_images_gray_codes[num_pattern]);

                //inverse gray codes
                sprintf(filename,"Captured_patterns/Coded_patterns/Gray_coded/Vertical/Undistorted/inverse_Captured_image_%d.bmp",num_pattern);
                gray_images_gray_codes_inverse[num_pattern]=cvLoadImage(filename,CV_LOAD_IMAGE_GRAYSCALE);
                sprintf(filename,"Captured_patterns/Coded_patterns/Gray_coded/Vertical/Undistorted/inverse_Gray_captured_image_%d.bmp",num_pattern);
                cvSaveImage(filename,gray_images_gray_codes_inverse[num_pattern]);


            }
        }
    }

    if(pattern_type==1)
    {
        if(count==0)
            gray_images_binary_codes=new IplImage*[number_of_patterns_binary_horizontal+1];
        else
            {
                gray_images_gray_codes=new IplImage*[number_of_patterns_binary_horizontal+1];
                gray_images_gray_codes_inverse=new IplImage*[number_of_patterns_binary_horizontal+1];
            }
        for(int num_pattern=0; num_pattern<(number_of_patterns_binary_horizontal+1); num_pattern++)
        {
            if(count==0)
            {
                //binary codes
                sprintf(filename,"Captured_patterns/Coded_patterns/Binary_coded/Horizontal/Undistorted/Captured_image_%d.bmp",num_pattern);
                gray_images_binary_codes[num_pattern]=cvLoadImage(filename,CV_LOAD_IMAGE_GRAYSCALE);
                sprintf(filename,"Captured_patterns/Coded_patterns/Binary_coded/Horizontal/Undistorted/Gray_captured_image_%d.bmp",num_pattern);
                cvSaveImage(filename,gray_images_binary_codes[num_pattern]);
            }

            else
            {
                //gray codes
                sprintf(filename,"Captured_patterns/Coded_patterns/Gray_coded/Horizontal/Undistorted/Captured_image_%d.bmp",num_pattern);
                gray_images_gray_codes[num_pattern]=cvLoadImage(filename,CV_LOAD_IMAGE_GRAYSCALE);
                sprintf(filename,"Captured_patterns/Coded_patterns/Gray_coded/Horizontal/Undistorted/Gray_captured_image_%d.bmp",num_pattern);
                cvSaveImage(filename,gray_images_gray_codes[num_pattern]);

                //inverse gray codes
                sprintf(filename,"Captured_patterns/Coded_patterns/Gray_coded/Horizontal/Undistorted/inverse_Captured_image_%d.bmp",num_pattern);
                gray_images_gray_codes_inverse[num_pattern]=cvLoadImage(filename,CV_LOAD_IMAGE_GRAYSCALE);
                sprintf(filename,"Captured_patterns/Coded_patterns/Gray_coded/Horizontal/Undistorted/inverse_Gray_captured_image_%d.bmp",num_pattern);
                cvSaveImage(filename,gray_images_gray_codes_inverse[num_pattern]);
            }
        }
    }

    return;
}


void decode_pixels(int pattern_type)
{

    bool *B[1],*G[1];

    if(pattern_type==0)
    {
        for(int u=0; u<Camera_imageheight; u++)
            for(int v=0; v<Camera_imagewidth; v++)
                code_vertical[v][u]=-1;//initialize by invalid value.

        if(count==0) // binary coded patterns were projected...
        {
            for(int row=0; row<Camera_imageheight; row++)
                for(int col=0; col<Camera_imagewidth; col++)
                {
                    if(valid_map_vertical[col][row]==1)
                    {
                        code_vertical[col][row]=0;//lets make it valid first!!
                        for(int num_pattern=0; num_pattern<number_of_patterns_binary_vertical; num_pattern++)
                        {

                            if(((unsigned char)gray_images_binary_codes[num_pattern]->imageData[row*gray_images_binary_codes[num_pattern]->widthStep+col]-(unsigned char)gray_images_binary_codes[number_of_patterns_binary_vertical]->imageData[row*gray_images_binary_codes[number_of_patterns_binary_vertical]->widthStep+col])>=(unsigned char)THRESH_VERT) //Practically it may require some tolerance.
                                code_vertical[col][row]+=pow(2,num_pattern);
                        }
                    }
                }
        }

        else // gray coded patterns were projected...
        {

            B[0]=new bool[number_of_patterns_binary_vertical];
            G[0]=new bool[number_of_patterns_binary_vertical];

            for(unsigned row=0; row<Camera_imageheight; row++)
                for(unsigned col=0; col<Camera_imagewidth; col++)
                {
                    if(valid_map_vertical[col][row]==1)
                    {
                        for(unsigned i=0; i<number_of_patterns_binary_vertical; i++)
                        {
                            G[0][i]=0;
                            B[0][i]=0;
                        }

                        code_vertical[col][row]=0;
                        for(int i=0; i<number_of_patterns_binary_vertical; i++)
                        {
                            if(((unsigned char)gray_images_gray_codes[i]->imageData[row*gray_images_gray_codes[i]->widthStep+col]-(unsigned char)gray_images_gray_codes_inverse[i]->imageData[row*gray_images_gray_codes_inverse[i]->widthStep+col])>=(unsigned char)THRESH_VERT)
                               //if(((unsigned char)gray_images_gray_codes[i]->imageData[row*gray_images_gray_codes[i]->widthStep+col]-(unsigned char)gray_images_gray_codes[number_of_patterns_binary_vertical]->imageData[row*gray_images_gray_codes[number_of_patterns_binary_vertical]->widthStep+col])>=(unsigned char)THRESH_VERT)
                                G[0][i]=1; /// the actual decode step

                            if(i==0)
                                B[0][i]=G[0][i];

                            else
                                B[0][i]=(B[0][i-1]!=G[0][i])?1:0; // effectively, XOR operation

                            code_vertical[col][row]+=B[0][i]*pow(2,number_of_patterns_binary_vertical-1-i);
                        }

                        if((code_vertical[col][row]>(number_of_codes_vertical-1)) && count==1)
                        {
                            //printf("Code error\n");
                            // valid_map_vertical[col][row]=0;
                        }
                    }
                }
        }
    }


    if(pattern_type==1)
    {
        for(int l=0; l<Camera_imageheight; l++)
            for(int k=0; k<Camera_imagewidth; k++)
                code_horizontal[k][l]=-1;//invalid value.

        if(count==0)
        {
            for(int r=0; r<Camera_imageheight; r++)
                for(int c=0; c<Camera_imagewidth; c++)
                {
                    if(valid_map_horizontal[c][r]==1)
                    {
                        code_horizontal[c][r]=0;
                        for(int n_pattern=0; n_pattern<number_of_patterns_binary_horizontal; n_pattern++)
                        {

                            if(((unsigned char)gray_images_binary_codes[n_pattern]->imageData[r*gray_images_binary_codes[n_pattern]->widthStep+c]-(unsigned char)gray_images_binary_codes[number_of_patterns_binary_horizontal]->imageData[r*gray_images_binary_codes[number_of_patterns_binary_horizontal]->widthStep+c])>=(unsigned char)THRESH_HORZ)
                                code_horizontal[c][r]+=pow(2,n_pattern);
                        }

                    }
                }
        }


        else // gray coded patterns were projected...
        {
            B[0]=new bool[number_of_patterns_binary_horizontal];
            G[0]=new bool[number_of_patterns_binary_horizontal];

            for(unsigned row=0; row<Camera_imageheight; row++)
                for(unsigned col=0; col<Camera_imagewidth; col++)
                {
                    if(valid_map_horizontal[col][row]==1)
                    {
                        for(unsigned i=0; i<number_of_patterns_binary_horizontal; i++)
                        {
                            G[0][i]=0;
                            B[0][i]=0;
                        }

                        code_horizontal[col][row]=0;
                        for(int i=0; i<number_of_patterns_binary_horizontal; i++)
                        {

                            if(((unsigned char)gray_images_gray_codes[i]->imageData[row*gray_images_gray_codes[i]->widthStep+col]-(unsigned char)gray_images_gray_codes_inverse[i]->imageData[row*gray_images_gray_codes_inverse[i]->widthStep+col])>=(unsigned char)THRESH_HORZ)
                              //if(((unsigned char)gray_images_gray_codes[i]->imageData[row*gray_images_gray_codes[i]->widthStep+col]-(unsigned char)gray_images_gray_codes[number_of_patterns_binary_horizontal]->imageData[row*gray_images_gray_codes[number_of_patterns_binary_horizontal]->widthStep+col])>=(unsigned char)THRESH_HORZ)
                                G[0][i]=1;

                            if(i==0)
                                B[0][i]=G[0][i];

                            else
                                B[0][i]=(B[0][i-1]!=G[0][i])?1:0; // effectively, XOR operation

                            code_horizontal[col][row]+=B[0][i]*pow(2,number_of_patterns_binary_horizontal-1-i);
                        }
                    }
                }

        }

    }



    return;
}


void unwrap(int pattern_type)
{
    if(pattern_type==0)
    {
        unwrapped_phi_vertical=new float[Camera_imagewidth][Camera_imageheight];

        for(int row=0; row<Camera_imageheight; row++)
            for(int col=1; col<Camera_imagewidth-1; col++)
            {
                if(valid_map_vertical[col][row]==1)
                {

                    wrapped_phi_vertical[col][row]+=Pi;
                    unwrapped_phi_vertical[col][row]=wrapped_phi_vertical[col][row]+code_vertical[col][row]*2.0*Pi;


                }
            }
    }

    if(pattern_type==1)
    {
        unwrapped_phi_horizontal=new float[Camera_imagewidth][Camera_imageheight];


        for(int col=0; col<Camera_imagewidth; col++)
            for(int row=1; row<Camera_imageheight-1; row++)
            {
                if(valid_map_horizontal[col][row]==1)
                {
                    wrapped_phi_horizontal[col][row]+=Pi;
                    unwrapped_phi_horizontal[col][row]=wrapped_phi_horizontal[col][row]+code_horizontal[col][row]*2.0*Pi;

                }
            }
    }

    return;
}



//Following function will generate image for unwrapped phase.
void save_unwrap_phase_image(int pattern_type)
{
    IplImage*unwrapped_phase_image=cvCreateImage(cvSize(Camera_imagewidth,Camera_imageheight),IPL_DEPTH_8U,1);
    cvSet(unwrapped_phase_image,cvScalar(0));

    float t;

    if(pattern_type==0)
    {
        for(int r=0; r<Camera_imageheight; r++)
            for(int c=0; c<Camera_imagewidth; c++)
                if(valid_map_vertical[c][r]==1)
                {
                    t=unwrapped_phi_vertical[c][r]/(2.0*Pi*number_of_codes_vertical);
                    unwrapped_phase_image->imageData[r*unwrapped_phase_image->widthStep+c]=(unsigned char)(t*255);
                }

        if(count==0)
            cvSaveImage("Unwrapped_phase_images/Binary_coded/Vertical/Unwrapped_phase_vertical.bmp",unwrapped_phase_image);

        else
            cvSaveImage("Unwrapped_phase_images/Gray_coded/Vertical/Unwrapped_phase_vertical.bmp",unwrapped_phase_image);


    }

    if(pattern_type==1)
    {
        for(int r=0; r<Camera_imageheight; r++)
            for(int c=0; c<Camera_imagewidth; c++)
                if(valid_map_horizontal[c][r]==1)
                {
                    t=unwrapped_phi_horizontal[c][r]/(2.0*Pi*number_of_codes_horizontal);
                    unwrapped_phase_image->imageData[r*unwrapped_phase_image->widthStep+c]=(unsigned char)(t*255);
                }
        if(count==0)
            cvSaveImage("Unwrapped_phase_images/Binary_coded/Horizontal/Unwrapped_phase_horizontal.bmp",unwrapped_phase_image);

        else
            cvSaveImage("Unwrapped_phase_images/Gray_coded/Horizontal/Unwrapped_phase_horizontal.bmp",unwrapped_phase_image);
    }

    return;
}


void unwrap_phase(int pattern_type)
{
    count=1;
    // while(count<2)
    // {
    if(pattern_type==0)
        code_vertical=new int[Camera_imagewidth][Camera_imageheight];

    if(pattern_type==1)
        code_horizontal=new int[Camera_imagewidth][Camera_imageheight];


    read_captured_images(pattern_type);

    decode_pixels(pattern_type);

    unwrap(pattern_type);

    save_unwrap_phase_image(pattern_type);

    //  count++;
    // }



    return ;
}
