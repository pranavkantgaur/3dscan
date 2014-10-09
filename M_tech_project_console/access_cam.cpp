/*
Purpose of this file is to provide functions to interface concerned application to the Canon PowerShot G7.
*/
//#include"dependencies.h"

#include "common_variables.h"

extern CameraWidget *widget;
extern GPContext *context;
//extern Camera *camera;
extern CameraFile* img_file;
extern CameraWidget *child;
extern unsigned long location;
extern JSAMPROW row_pointer[1];
extern char * raw_image;
extern struct jpeg_decompress_struct cinfo;
extern struct jpeg_error_mgr jerr;
extern unsigned long size;
extern const unsigned char *data;
extern int cam_shutter_speed;
extern int cam_aperture;

string powershot_g7_aperture_map[]= {"2.8","3.2","3.5","4","4.5","5.0","5.6","6.3","7.1","8"};
string powershot_g7_shutterspeed_map[]=
{
    "15",
    "13",
    "10",
    "8",
    "6",
    "5",
    "4",
    "3.2",
    "2.5",
    "2",
    "1.6",
    "1.3",
    "1",
    "0.8",
    "0.6",
    "0.5",
    "0.4",
    "0.3",
    "1/4",
    "1/5",
    "1/6",
    "1/8",
    "1/10",
    "1/13",
    "1/15",
    "1/20",
    "1/25",
    "1/30",
    "1/40",
    "1/50",
    "1/60",
    "1/80",
    "1/100",
    "1/125",
    "1/160",
    "1/200",
    "1/250",
    "1/320",
    "1/400",
    "1/500",
    "1/640",
    "1/800",
    "1/1000",
    "1/1250",
    "1/1600",
};


static int _lookup_widget(const char *key, CameraWidget **child)
{
    int ret;
    ret = gp_widget_get_child_by_name (widget, key, child);
    if (ret < GP_OK)
        ret = gp_widget_get_child_by_label (widget, key, child);


    return ret;
}


void set_camera_control(const char  * control_string, const char *val)
{
    //Get the camera control widget
    if(gp_camera_get_config (camera, &widget, context)<GP_OK)
    {
        printf("\nCannot initialize control widget!!");

    }

    //Lookup in the widget for the control named by 'control_string'
    if(_lookup_widget (control_string, &child)<GP_OK)
    {
        printf("\nInvalid control name: %s",control_string);

    }


    //Set the value of control
    if(gp_widget_set_value (child, val)<GP_OK) /// NOTE: 'val' comes from OpenCV shutter speed/aperture control.
    {
        printf("\nFailed to set control value!!");
    }

    //Set the modified configuration back to camera object.
    int t=gp_camera_set_config (camera, widget, context);

    if(t<GP_OK)
    {
        if(strcmp("shutterspeed",control_string)==0)
            printf("\nShutter speed: %s\n",val);

        if(strcmp("aperture",control_string)==0)
            printf("\nAperture: %s\n",val);

        printf("\nFailed to modify camera widget!!, error code:%d",t);
    }

    return;
}


typedef struct my_src_mgr my_src_mgr;

struct my_src_mgr
{
    struct jpeg_source_mgr pub;

    JOCTET eoi_buffer[2];
};


static void init_source(j_decompress_ptr cinfo)
{
}

static int fill_input_buffer(j_decompress_ptr cinfo)
{
    return 1;
}

static void skip_input_data(j_decompress_ptr cinfo, long num_bytes)
{
    my_src_mgr *src = (my_src_mgr *)cinfo->src;

    if (num_bytes > 0)
    {
        while (num_bytes > (long)src->pub.bytes_in_buffer)
        {

            num_bytes -= (long)src->pub.bytes_in_buffer;
            fill_input_buffer(cinfo);
        }
    }

    src->pub.next_input_byte += num_bytes;
    src->pub.bytes_in_buffer -= num_bytes;
}

static void term_source(j_decompress_ptr cinfo)
{
}


void jpeg_memory_src(j_decompress_ptr cinfo, unsigned char const *buffer, size_t bufsize)
{

    my_src_mgr *src;
    if (! cinfo->src)
    {

        cinfo->src = (jpeg_source_mgr*)(*cinfo->mem->alloc_small)((j_common_ptr)cinfo, JPOOL_PERMANENT, sizeof(my_src_mgr));
    }

    src = (my_src_mgr *)cinfo->src;
    src->pub.init_source = init_source;

    src->pub.fill_input_buffer = fill_input_buffer;
    src->pub.skip_input_data = skip_input_data;

    src->pub.resync_to_restart = jpeg_resync_to_restart;
    src->pub.term_source = term_source;

    src->pub.next_input_byte = buffer;
    src->pub.bytes_in_buffer = bufsize;
}



IplImage* camera_preview()
{
//libjpeg variables
    IplImage*imgRGB,*imgBGR;

    cinfo.err = jpeg_std_error( &jerr );

    jpeg_create_decompress( &cinfo );

    gp_file_new(&img_file);

    location=0;
    set_camera_control("shutterspeed",powershot_g7_shutterspeed_map[cam_shutter_speed].c_str());
    set_camera_control("aperture",powershot_g7_aperture_map[cam_aperture].c_str());

    gp_camera_capture_preview(camera,img_file,context);

    gp_file_get_data_and_size (img_file, (const char**)&data, &size);

    jpeg_memory_src(&cinfo, data, size);

    jpeg_read_header( &cinfo, TRUE );

    jpeg_start_decompress( &cinfo );

    imgBGR =cvCreateImageHeader(cvSize( cinfo.output_width,cinfo.output_height), IPL_DEPTH_8U, cinfo.num_components);

    cvCreateData(imgBGR);

    imgRGB =cvCreateImageHeader(cvSize( cinfo.output_width,cinfo.output_height), IPL_DEPTH_8U, cinfo.num_components);

    cvCreateData(imgRGB);


    raw_image = imgBGR->imageData;

    row_pointer[0] = (unsigned char *)malloc( cinfo.output_width*cinfo.num_components );

    while( cinfo.output_scanline < cinfo.image_height )
    {

        jpeg_read_scanlines( &cinfo, row_pointer, 1 );
        for( unsigned int i=0; i<cinfo.image_width*cinfo.num_components; i++)
        {

            raw_image[location++] = row_pointer[0][i];
        }
    }



    cvCvtColor(imgBGR, imgRGB, CV_BGR2RGB);
    jpeg_finish_decompress( &cinfo );

    jpeg_destroy_decompress( &cinfo );
    free( row_pointer[0] );


    return(imgRGB);
}



IplImage* camera_capture(char* image_path)
{

    CameraFilePath camera_file_path;
    int fd,ret;
    CameraFile *file;
    strcpy(camera_file_path.folder, "/");
    strcpy(camera_file_path.name, "capt0000.jpg");
    //set_camera_control("shutterspeed",(powershot_g7_shutterspeed_map+cam_shutter_speed));
    //set_camera_control("aperture",(powershot_g7_aperture_map+cam_aperture));

    ret=gp_camera_capture(camera, GP_CAPTURE_IMAGE, &camera_file_path, context);


    if(ret<GP_OK)
        printf("\nFailed to capture image");
    else
        printf("\nCaptured.");


    fd = open(image_path, O_CREAT | O_WRONLY, 0644);
    gp_file_new_from_fd(&file, fd);

    gp_camera_file_get(camera, camera_file_path.folder, camera_file_path.name,
                       GP_FILE_TYPE_NORMAL, file, context);



    gp_camera_file_delete(camera, camera_file_path.folder, camera_file_path.name, context);


    gp_file_free(file);

    return cvLoadImage(image_path);
}
