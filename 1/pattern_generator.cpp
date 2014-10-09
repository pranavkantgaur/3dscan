/*This pattern generator must be configurable to generate fringe images of any phase difference value.

UPDATE:26 June 2012
Adding code for number of phase shifts upto 5(i.e.,3,4,5).

UPDATE: 8 Jan 2013
Adding inverse binary patterns to remove binary-strip edge problem.

UPDATE: 13 Nov 2013
Adding gray coded pattern projection and removing inverse coded pattern apporach from the code.

UPDATE: 30 Jan 2014
Adding inverse gray coded patterns to remove binary-strip edge problem.

*/




#include "/home/pranav/Desktop/PROJECT_GLOBAL/common_variables.h"
#include "/home/pranav/Desktop/PROJECT_GLOBAL/global_cv.h"

////////////////////////////////////////////////////////////////////////////////////////////////////Global variables.


IplImage**phase_shifted_pattern_images_vertical;//This array is for storing the number of pattern images to be flashed onto the the object.
IplImage**phase_shifted_pattern_images_horizontal;

IplImage**binary_coded_pattern_images_vertical;//It will contain the binary coded images.
IplImage**binary_coded_pattern_images_horizontal;


IplImage**gray_coded_patterns_vertical;
IplImage**gray_coded_patterns_horizontal;

IplImage**inverse_gray_coded_patterns_vertical;
IplImage**inverse_gray_coded_patterns_horizontal;


extern int number_of_patterns_fringe;

extern int number_of_patterns_binary_vertical;//because number of patterns can be different for vertical & horizontal types.
extern int number_of_patterns_binary_horizontal;

extern int number_of_codes_vertical;
extern int number_of_codes_horizontal;


extern int fringe_width_pixels_vertical;
extern int fringe_width_pixels_horizontal;

extern char filename[150];


// Generate gray coded patterns...
void generate_gray_coded_patterns()
{

    /// Convert binary to gray
    bool **B[1];
    bool **G[1];


///**********************************************************************************************************///
    /// Vertical

    for(unsigned n=0; n<1; n++)
    {
        B[n]=new bool*[number_of_codes_vertical];
        G[n]=new bool*[number_of_codes_vertical];

        for(unsigned k=0; k<number_of_codes_vertical; k++)
        {
            B[n][k]=new bool[number_of_patterns_binary_vertical];
            G[n][k]=new bool[number_of_patterns_binary_vertical];
        }
    }


    for(unsigned n=0,temp=0; n<number_of_codes_vertical; n++)
    {
        temp=n;
        for(int i=number_of_patterns_binary_vertical-1; i>=0; i--) /// extracting binary code corresponding to position of pixel
        {
            B[0][n][i]=(temp%2)?1:0;
            temp/=2;


        }

        /// Index 0: MSB, Last:LSB (convension of storing codes in the array)

        /// Lets create corresponding gray code...

        for(int i=0; i<number_of_patterns_binary_vertical; i++)
        {
            if(i==0)
                G[0][n][i]=B[0][n][i];
            else
                G[0][n][i]=(B[0][n][i-1]!=B[0][n][i])?1:0; // effectively, XOR operation on binary code bits

            if(n==39)
                printf("\nGray code:%d\t for binary code:%d ",G[0][n][i],B[0][n][i]);
        }
    }



    /// Lets save the image(s) corresponding to computed gray coded patterns.
    unsigned code_number;

    for(unsigned n=0; n<number_of_patterns_binary_vertical; n++)
    {
        for(unsigned r=0; r<Projector_imageheight; r++)
        {
            code_number=0;
            for(unsigned c=0; c<Projector_imagewidth; c+=fringe_width_pixels_vertical)
            {

                for(unsigned offset=0; (offset<fringe_width_pixels_vertical) && ((c+offset)<Projector_imagewidth); offset++)
                {
                    //if((c+offset)==1279 && r==0)
                    //printf("\nbinary code:%d for code number:%d",B[0][code_number][n],code_number);

                    gray_coded_patterns_vertical[n]->imageData[r*gray_coded_patterns_vertical[n]->widthStep+(c+offset)]=G[0][code_number][n]*255;
                }
                code_number++;
            }

        }
    }



///************************************************************************************************************///


///************************************************************************************************************///
/// Horizontal
    for(unsigned n=0; n<1; n++)
    {
        B[n]=new bool*[number_of_codes_horizontal];
        G[n]=new bool*[number_of_codes_horizontal];

        for(unsigned k=0; k<number_of_codes_horizontal; k++)
        {
            B[n][k]=new bool[number_of_patterns_binary_horizontal];
            G[n][k]=new bool[number_of_patterns_binary_horizontal];
        }
    }

    for(unsigned n=0,temp=0; n<number_of_codes_horizontal; n++)
    {
        temp=n;
        for(unsigned i=0; i<number_of_patterns_binary_horizontal; i++) /// extracting binary code corresponding to position of pixel
        {
            B[0][n][number_of_patterns_binary_horizontal-1-i]=(temp%2)?1:0;
            temp/=2;
        }
        /// Lets create corresponding gray code...
        for(int i=0; i<number_of_patterns_binary_horizontal; i++)
        {
            if(i==0)
                G[0][n][i]=B[0][n][i];
            else
                G[0][n][i]=(B[0][n][i-1]!=B[0][n][i])?1:0; // effectively, XOR operation on binary code bits
        }
    }



    /// Lets save the image(s) corresponding to computed gray coded patterns.
    for(unsigned n=0; n<number_of_patterns_binary_horizontal; n++)
    {
        for(unsigned c=0; c<Projector_imagewidth; c++)
        {
            code_number=0;

            for(unsigned r=0; r<Projector_imageheight; r+=fringe_width_pixels_horizontal)
            {
                for(unsigned offset=0; (offset<fringe_width_pixels_horizontal) && ((r+offset)<Projector_imageheight); offset++)
                    gray_coded_patterns_horizontal[n]->imageData[(r+offset)*gray_coded_patterns_horizontal[n]->widthStep+c]=G[0][code_number][n]*255;

                code_number++;

            }
        }

    }


///************************************************************************************************************///



    return;
}


void allocate_memory()
{

    //Now lets allocate memory for fringe images.
    printf("Enter number of fringe patterns to be projected:");
    scanf("%d",&number_of_patterns_fringe);


    phase_shifted_pattern_images_vertical=new IplImage*[number_of_patterns_fringe];
    phase_shifted_pattern_images_horizontal=new IplImage*[number_of_patterns_fringe];

    for(int phase_shifted_image_num=0; phase_shifted_image_num<number_of_patterns_fringe; phase_shifted_image_num++)
    {
        phase_shifted_pattern_images_vertical[phase_shifted_image_num]=cvCreateImage(cvSize(Projector_imagewidth,Projector_imageheight),IPL_DEPTH_8U,1);
        phase_shifted_pattern_images_horizontal[phase_shifted_image_num]=cvCreateImage(cvSize(Projector_imagewidth,Projector_imageheight),IPL_DEPTH_8U,1);
    }


    printf("\nPlease enter fringe width for vertical pattern:");
    scanf("%d",&fringe_width_pixels_vertical);

    printf("\nPlease enter fringe width for horizontal pattern:");
    scanf("%d",&fringe_width_pixels_horizontal);


    //Now,lets allocate memory for Binary code & gray coded patterns also ...
    number_of_codes_vertical=(int)ceil((float)Projector_imagewidth/(float)fringe_width_pixels_vertical);

    number_of_patterns_binary_vertical=(int)ceil((logf((float)number_of_codes_vertical)/logf(2.0)));

    number_of_codes_horizontal=(int)ceil((float)Projector_imageheight/(float)fringe_width_pixels_horizontal);
    number_of_patterns_binary_horizontal=(int)ceil((logf((float)number_of_codes_horizontal)/logf(2.0)));


    binary_coded_pattern_images_vertical=new IplImage*[number_of_patterns_binary_vertical+1];//Note:+1 is for blank image.
    binary_coded_pattern_images_horizontal=new IplImage*[number_of_patterns_binary_horizontal+1];

    gray_coded_patterns_vertical=new IplImage*[number_of_patterns_binary_vertical+1];
    gray_coded_patterns_horizontal=new IplImage*[number_of_patterns_binary_horizontal+1];

    inverse_gray_coded_patterns_vertical=new IplImage*[number_of_patterns_binary_vertical+1];
    inverse_gray_coded_patterns_horizontal=new IplImage*[number_of_patterns_binary_horizontal+1];

    for(int i=0; i<number_of_patterns_binary_vertical+1; i++)
    {
        binary_coded_pattern_images_vertical[i]=cvCreateImage(cvSize(Projector_imagewidth,Projector_imageheight),IPL_DEPTH_8U,1);
        gray_coded_patterns_vertical[i]=cvCreateImage(cvSize(Projector_imagewidth,Projector_imageheight),IPL_DEPTH_8U,1);
        inverse_gray_coded_patterns_vertical[i]=cvCreateImage(cvSize(Projector_imagewidth,Projector_imageheight),IPL_DEPTH_8U,1);
    }


    for(int j=0; j<number_of_patterns_binary_horizontal+1; j++)
    {
        binary_coded_pattern_images_horizontal[j]=cvCreateImage(cvSize(Projector_imagewidth,Projector_imageheight),IPL_DEPTH_8U,1);
        gray_coded_patterns_horizontal[j]=cvCreateImage(cvSize(Projector_imagewidth,Projector_imageheight),IPL_DEPTH_8U,1);
        inverse_gray_coded_patterns_horizontal[j]=cvCreateImage(cvSize(Projector_imagewidth,Projector_imageheight),IPL_DEPTH_8U,1);
    }


//Done,memory allocation for all patterns.
    return;
}



//Following function will return 0 or 1 for a pixel based on its postion,type of pattern,and pattern number.
int give_code_bit(int pattern_type,int pattern_num,int x_pos,int y_pos)
{
    /*
    Steps:-1.Categorize the pixel in code class.
           2.Access the bit no. 'pattern_num' for that class.
           3.Return it.
    */
    int t;

    if(pattern_type==0)
        t=(int)(x_pos/(pow(2,pattern_num)*fringe_width_pixels_vertical));//it is the interval in which i fall.

    if(pattern_type==1)
        t=(int)(y_pos/(pow(2,pattern_num)*fringe_width_pixels_horizontal));


    return t%2;

}

//I1=Imod+Idc*(cos(phi-2pi/3))
//I2=Imod+Idc*(cos(phi))
//I3=Imod+Idc*(cos(phi+2pi/3))

//Following function generates vertical & horizontal fringe patterns.
void fringe_pattern_generate_3()
{
    float t=0.0;

//Lets generate patterns for fringe technique first!!

    //Vertical patterns...
    for(int pattern_num_fringe=0; pattern_num_fringe<number_of_patterns_fringe; pattern_num_fringe++)
        for(int row_num=0; row_num<Projector_imageheight; row_num++)
            for(int col_num=0; col_num<Projector_imagewidth; col_num++)
            {
                t=127.0f+128.0f*cosf(((float)col_num/(float)fringe_width_pixels_vertical)*2.0*Pi-Pi-((Pi)/2.0)+(Pi/2.0)*(float)pattern_num_fringe);
                phase_shifted_pattern_images_vertical[pattern_num_fringe]->imageData[row_num*phase_shifted_pattern_images_vertical[pattern_num_fringe]->widthStep+col_num]=(unsigned char)t;
            }



    //Horizontal patterns...
    for(int pattern_num_fringe=0; pattern_num_fringe<number_of_patterns_fringe; pattern_num_fringe++)
        for(int col_num=0; col_num<Projector_imagewidth; col_num++)
            for(int row_num=0; row_num<Projector_imageheight; row_num++)
            {
                t=127.0f+128.0f*cosf(((float)row_num/(float)fringe_width_pixels_horizontal)*2.0*Pi-Pi-((Pi)/2.0)+(Pi/2.0)*(float)pattern_num_fringe);
                phase_shifted_pattern_images_horizontal[pattern_num_fringe]->imageData[row_num*phase_shifted_pattern_images_horizontal[pattern_num_fringe]->widthStep+col_num]=(unsigned char)t;
            }


//Done!!
    return;
}
//patterns are stored in pattern_images array.


//n=4:
/*
I1=Im+Ia*cos(phi(x,y))
I2=Im+Ia*cos(phi(x,y)+pi/2)
I3=Im+Ia*cos(phi(x,y)+pi)
I4=Im+Ia*cos(phi(x,y)+3*pi/2)
*/


void fringe_pattern_generate_4()
{

    float t=0.0;
//Vertical
    for(int pattern_num=0; pattern_num<number_of_patterns_fringe; pattern_num++)
        for(int h=0; h<Projector_imageheight; h++)
            for(int w=0; w<Projector_imagewidth; w++)
            {
                t=127.0+128.0*cosf(((float)w/(float)fringe_width_pixels_vertical)*(2.0*Pi)-Pi+(Pi/2.0)*(float)pattern_num);
                phase_shifted_pattern_images_vertical[pattern_num]->imageData[h*phase_shifted_pattern_images_vertical[pattern_num]->widthStep+w]=(unsigned char)t;
            }


//Horizontal
    for(int pattern_num=0; pattern_num<number_of_patterns_fringe; pattern_num++)
        for(int w=0; w<Projector_imagewidth; w++)
            for(int h=0; h<Projector_imageheight; h++)
            {
                t=127.0+128.0*cosf(((float)h/(float)fringe_width_pixels_horizontal)*(2.0*Pi)-Pi+(Pi/2.0)*(float)pattern_num);
                phase_shifted_pattern_images_horizontal[pattern_num]->imageData[h*phase_shifted_pattern_images_horizontal[pattern_num]->widthStep+w]=(unsigned char)t;
            }


    return;
}





//n=5:
/*
I1=Im+Ia*cos(phi(x,y)-4pi/5)
I2=Im+Ia*cos(phi(x,y)-2pi/5)
I3=Im+Ia*cos(phi(x,y))
I4=Im+Ia*cos(phi(x,y)+2pi/5)
I5=Im+Ia*cos(phi(x,y)+4pi/5)
*/
void fringe_pattern_generate_5()
{

    float t=0.0;
//Vertical
    for(int pattern_num=0; pattern_num<number_of_patterns_fringe; pattern_num++)
        for(int h=0; h<Projector_imageheight; h++)
            for(int w=0; w<Projector_imagewidth; w++)
            {
                t=127.0f+128.0f*cosf(((float)w/(float)fringe_width_pixels_vertical)*(2.0*Pi)-Pi-2.0*((Pi)/2)+((Pi)/2)*(float)pattern_num);
                phase_shifted_pattern_images_vertical[pattern_num]->imageData[h*phase_shifted_pattern_images_vertical[pattern_num]->widthStep+w]=(unsigned char)t;
            }


//Horizontal
    for(int pattern_num=0; pattern_num<number_of_patterns_fringe; pattern_num++)
        for(int h=0; h<Projector_imageheight; h++)
            for(int w=0; w<Projector_imagewidth; w++)
            {
                t=127.0f+128.0f*cosf(((float)h/(float)fringe_width_pixels_horizontal)*(2.0*Pi)-Pi-2.0*((Pi)/2)+((Pi)/2)*(float)pattern_num);
                phase_shifted_pattern_images_horizontal[pattern_num]->imageData[h*phase_shifted_pattern_images_horizontal[pattern_num]->widthStep+w]=(unsigned char)t;
            }


    return;
}





void binary_pattern_generate()
{
    //Lets do the same for binary code scheme...
    //Vertical...
    for(int pattern_num_binary_vertical=0; pattern_num_binary_vertical<number_of_patterns_binary_vertical; pattern_num_binary_vertical++)
        for(int i=0; i<Projector_imageheight; i++)
            for(int j=0; j<Projector_imagewidth; j++)
                if(give_code_bit(0,pattern_num_binary_vertical,j,i)==1)  //white.
                    binary_coded_pattern_images_vertical[pattern_num_binary_vertical]->imageData[i*binary_coded_pattern_images_vertical[pattern_num_binary_vertical]->widthStep+j]=255;
                else if(give_code_bit(0,pattern_num_binary_vertical,j,i)==0) //black.
                    binary_coded_pattern_images_vertical[pattern_num_binary_vertical]->imageData[i*binary_coded_pattern_images_vertical[pattern_num_binary_vertical]->widthStep+j]=0;

//Horizontal...
    for(int pattern_num_binary_horizontal=0; pattern_num_binary_horizontal<number_of_patterns_binary_horizontal; pattern_num_binary_horizontal++)
        for(int l=0; l<Projector_imageheight; l++)
            for(int k=0; k<Projector_imagewidth; k++)
                if(give_code_bit(1,pattern_num_binary_horizontal,k,l)==1)  //white.
                    binary_coded_pattern_images_horizontal[pattern_num_binary_horizontal]->imageData[l*binary_coded_pattern_images_horizontal[pattern_num_binary_horizontal]->widthStep+k]=255;
                else if(give_code_bit(1,pattern_num_binary_horizontal,k,l)==0) //black.
                    binary_coded_pattern_images_horizontal[pattern_num_binary_horizontal]->imageData[l*binary_coded_pattern_images_horizontal[pattern_num_binary_horizontal]->widthStep+k]=0;

    return;
}



//Following function saves the generated patterns.
void save_pattern_images()
{
    //Lets save the fringe patterns first...
    //Vertical...
    for(int i=0; i<number_of_patterns_fringe; i++)
    {
        sprintf(filename,"Generated_patterns/Fringe_patterns/Vertical/Pattern_%d.bmp",i);
        cvSaveImage(filename,phase_shifted_pattern_images_vertical[i]);
    }

    //Horizontal...
    for(int j=0; j<number_of_patterns_fringe; j++)
    {
        sprintf(filename,"Generated_patterns/Fringe_patterns/Horizontal/Pattern_%d.bmp",j);
        cvSaveImage(filename,phase_shifted_pattern_images_horizontal[j]);
    }


//Same for binary coded patterns...
    //Vertical...
    for(int j=0; j<number_of_patterns_binary_vertical+1; j++)
    {
        // Binary coded
        sprintf(filename,"Generated_patterns/Coded_patterns/Binary_coded/Vertical/Pattern_%d.bmp",j);
        cvSaveImage(filename,binary_coded_pattern_images_vertical[j]);

        // Gray coded
        sprintf(filename,"Generated_patterns/Coded_patterns/Gray_coded/Vertical/Pattern_%d.bmp",j);
        cvSaveImage(filename,gray_coded_patterns_vertical[j]);

        //inverse gray coded
        sprintf(filename,"Generated_patterns/Coded_patterns/Gray_coded/Vertical/inverse_Pattern_%d.bmp",j);
        cvSaveImage(filename,inverse_gray_coded_patterns_vertical[j]);


    }

    //Horizontal...
    for(int j=0; j<number_of_patterns_binary_horizontal+1; j++)
    {
        sprintf(filename,"Generated_patterns/Coded_patterns/Binary_coded/Horizontal/Pattern_%d.bmp",j);
        cvSaveImage(filename,binary_coded_pattern_images_horizontal[j]);

        // Gray coded
        sprintf(filename,"Generated_patterns/Coded_patterns/Gray_coded/Horizontal/Pattern_%d.bmp",j);
        cvSaveImage(filename,gray_coded_patterns_horizontal[j]);

        //inverse gray coded
        sprintf(filename,"Generated_patterns/Coded_patterns/Gray_coded/Horizontal/inverse_Pattern_%d.bmp",j);
        cvSaveImage(filename,inverse_gray_coded_patterns_horizontal[j]);


    }


//Done,saved images!!
    return;
}


void generate_inverse_gray_coded_patterns()
{
    // Simply read gray coded patterns and create patterns which are complement of thses patterns
    /// Vertical
    for(unsigned i=0;i<number_of_patterns_binary_vertical;i++)
      for(unsigned r=0;r<Projector_imageheight;r++)
        for(unsigned c=0;c<Projector_imagewidth;c++)
           inverse_gray_coded_patterns_vertical[i]->imageData[r*inverse_gray_coded_patterns_vertical[i]->widthStep+c]=255-(unsigned char)(gray_coded_patterns_vertical[i]->imageData[r*gray_coded_patterns_vertical[i]->widthStep+c]);

   /// Horizontal
   for(unsigned i=0;i<number_of_patterns_binary_horizontal;i++)
      for(unsigned r=0;r<Projector_imageheight;r++)
        for(unsigned c=0;c<Projector_imagewidth;c++)
           inverse_gray_coded_patterns_horizontal[i]->imageData[r*inverse_gray_coded_patterns_horizontal[i]->widthStep+c]=255-(unsigned char)(gray_coded_patterns_horizontal[i]->imageData[r*gray_coded_patterns_horizontal[i]->widthStep+c]);


    return;
}




//Details:-EXTERNAL INTERFACE.
void generate_pattern()
{

    allocate_memory();

    if(number_of_patterns_fringe==3)
        fringe_pattern_generate_3();

    else if(number_of_patterns_fringe==4)
        fringe_pattern_generate_4();

    else if(number_of_patterns_fringe==5)
        fringe_pattern_generate_5();

    binary_pattern_generate();
    generate_gray_coded_patterns();

    generate_inverse_gray_coded_patterns();

    save_pattern_images();

    //Lets deallocate local/file scope global dynamically allocated storage...
    cvReleaseImage(phase_shifted_pattern_images_vertical);
    cvReleaseImage(phase_shifted_pattern_images_horizontal);

    cvReleaseImage(binary_coded_pattern_images_vertical);
    cvReleaseImage(binary_coded_pattern_images_horizontal);



    return;
}
