/*V4l2 camera control*/
#include"global_cv.h"

/// V4L2-control
uint8_t *buffer;
extern int fd; // file descriptor to access device
extern int exposure;
extern int exposure_range;
extern unsigned char *rgb_buffer;
extern IplImage*cap;

int print_caps()
{
    struct v4l2_capability caps = {};
    if (-1 == ioctl(fd, VIDIOC_QUERYCAP, &caps))
    {
        perror("Querying Capabilities");
        return 1;
    }

    printf( "Driver Caps:\n"
            "  Driver: \"%s\"\n"
            "  Card: \"%s\"\n"
            "  Bus: \"%s\"\n"
            "  Version: %d.%d\n"
            "  Capabilities: %08x\n",
            caps.driver,
            caps.card,
            caps.bus_info,
            (caps.version>>16)&&0xff,
            (caps.version>>24)&&0xff,
            caps.capabilities);


    struct v4l2_cropcap cropcap;
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == ioctl (fd, VIDIOC_CROPCAP, &cropcap))
    {
        perror("Querying Cropping Capabilities");
        return 1;
    }

    printf( "Camera Cropping:\n"
            "  Bounds: %dx%d+%d+%d\n"
            "  Default: %dx%d+%d+%d\n"
            "  Aspect: %d/%d\n",
            cropcap.bounds.width, cropcap.bounds.height, cropcap.bounds.left, cropcap.bounds.top,
            cropcap.defrect.width, cropcap.defrect.height, cropcap.defrect.left, cropcap.defrect.top,
            cropcap.pixelaspect.numerator, cropcap.pixelaspect.denominator);

    int support_grbg10 = 0;

    struct v4l2_fmtdesc fmtdesc = {0};
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    char fourcc[5] = {0};
    char c, e;
    printf("  FMT : CE Desc\n--------------------\n");
    while (0 == ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc))
    {
        strncpy(fourcc, (char *)&fmtdesc.pixelformat, 4);
        if (fmtdesc.pixelformat == V4L2_PIX_FMT_SGRBG10)
            support_grbg10 = 1;
        c = fmtdesc.flags & 1? 'C' : ' ';
        e = fmtdesc.flags & 2? 'E' : ' ';
        printf("  %s: %c%c %s\n", fourcc, c, e, fmtdesc.description);
        fmtdesc.index++;
    }
    /*
    if (!support_grbg10)
    {
        printf("Doesn't support GRBG10.\n");
        return 1;
    }*/

    struct v4l2_format fmt;
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = Camera_imagewidth;
    fmt.fmt.pix.height = Camera_imageheight;
    //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV ;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (-1 == ioctl(fd, VIDIOC_S_FMT, &fmt))
    {
        perror("Failed setting pixel format");
        return 1;
    }

    strncpy(fourcc, (char *)&fmt.fmt.pix.pixelformat, 4);
    printf( "Selected Camera Mode:\n"
            "  Width: %d\n"
            "  Height: %d\n"
            "  PixFmt: %s\n"
            "  Field: %d\n",
            fmt.fmt.pix.width,
            fmt.fmt.pix.height,
            fourcc,
            fmt.fmt.pix.field);
    return 0;
}

int init_mmap()
{
    struct v4l2_requestbuffers req = {0};
    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == ioctl(fd, VIDIOC_REQBUFS, &req))
    {
        perror("Requesting Buffer");
        return 1;
    }

    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    if(-1 == ioctl(fd, VIDIOC_QUERYBUF, &buf))
    {
        perror("Querying Buffer");
        return 1;
    }

    buffer = (unsigned char*)mmap (NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
    printf("Length: %d\nAddress: %p\n", buf.length, buffer);
    printf("Image Length: %d\n", buf.bytesused);

    return 0;
}

void preview_and_capture()
{
    struct v4l2_buffer buf = {0};

    CvSize size = cvSize(Camera_imagewidth, Camera_imageheight);
    int depth  = IPL_DEPTH_8U;
    int channels = 3;
    cap =  cvCreateImageHeader(size, depth,channels);


    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;


    if(-1 == ioctl(fd, VIDIOC_QBUF, &buf))
    {
        perror("Queue Buffer");
        //  return ;
    }


    if(-1 == ioctl(fd, VIDIOC_STREAMON, &buf.type))
    {
        perror("Start Capture");
        //  return ;
    }

    struct v4l2_ext_control ctrl_exp[1];
    //struct v4l2_ext_control ctrl_array[1];
    struct v4l2_queryctrl qctl;
    struct v4l2_ext_controls ctrls= {0};
    struct v4l2_ext_control ctrl_auto_exp[1];


    qctl.id=V4L2_CID_EXPOSURE_ABSOLUTE;


    if(-1==ioctl(fd,VIDIOC_QUERYCTRL,&qctl))
    {
        perror("Not able to query exposure");
        // exit(1);
    }

    printf("\nMinimum exposure value:%d\tMaximum exposure value:%d\n",qctl.minimum,qctl.maximum);


    int exposure_range=qctl.maximum-qctl.minimum;
    static int exposure=0;
    cvCreateTrackbar2("exposure","Camera_view",&exposure,exposure_range,NULL,NULL);

    unsigned char *rgb_buffer;
    rgb_buffer=(unsigned char*)malloc(Camera_imageheight*Camera_imagewidth*3*sizeof(unsigned char)); // 3 channels per pixel


    for(;;)
    {
        /// Lets capture the frame...
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd, &fds);
        struct timeval tv = {0};
        tv.tv_sec = 2;
        tv.tv_usec = 0;

        int r = select(fd+1, &fds, NULL, NULL, &tv);
        if(-1 == r)
        {
            perror("Waiting for Frame");
            //    return 1;
        }

        if(-1 == ioctl(fd, VIDIOC_DQBUF, &buf)) // these calls are for exchanging buffers with the V4L2 driver
        {
            perror("Deqeue buffer");
            //   return ;
        }


        /// YUYV to RGB conversion
        unsigned offset=0;
        unsigned i=0;
        unsigned k=0;
        float r_img,g_img,b_img,c,d,e;
        int r_int,g_int,b_int;
        for(; offset<(Camera_imageheight*Camera_imagewidth*3); offset+=3*2) //initializing 2 pixels per iteration
        {
            i=0;

            for(; i<2; i++)
            {

                c = buffer[k+i*2]-16;
                d = buffer[k+1]- 128;
                e = buffer[k+3] - 128;



                r_img=buffer[k+i*2]+1.403*d;
                g_img=buffer[k+i*2]-0.714*d-0.344*e;
                b_img=buffer[k+i*2]+1.773*e;

                r_int=(int)r_img;
                g_int=(int)g_img;
                b_int=(int)b_img;


                rgb_buffer[(offset+i*3)+0] = (r_int>255)?255:((r_int<0)?0:(unsigned char)r_int);
                rgb_buffer[(offset+i*3)+1] = (g_int>255)?255:((g_int<0)?0:(unsigned char)g_int);
                rgb_buffer[(offset+i*3)+2] = (b_int>255)?255:((b_int<0)?0:(unsigned char)b_int);



            }

            k+=4;
        }



        cap->imageData = (char*)rgb_buffer;

        cvShowImage("Camera_view", cap);




        if(-1 == ioctl(fd, VIDIOC_QBUF, &buf))
        {
            perror("Queue Buffer");
            //    return ;
        }

        /// Lets manipulate the exposure...
        //Set auto exposure to Manual mode
        ctrl_auto_exp[0].id=V4L2_CID_EXPOSURE_AUTO;
        ctrl_auto_exp[0].value=V4L2_EXPOSURE_MANUAL;

        ctrls.ctrl_class=V4L2_CTRL_CLASS_CAMERA;
        ctrls.count=1;
        ctrls.controls=ctrl_auto_exp;

        if(-1==ioctl(fd,VIDIOC_S_EXT_CTRLS,&ctrls))
        {
            perror("Failed to set exposure to manual mode");
            //      exit(1);
        }

        ctrls.ctrl_class=V4L2_CTRL_CLASS_CAMERA;
        ctrls.count=1;
        ctrls.controls=ctrl_exp;


        ctrl_exp[0].id=V4L2_CID_EXPOSURE_ABSOLUTE;
        ctrl_exp[0].value=exposure;


        if(-1==ioctl(fd,VIDIOC_S_EXT_CTRLS,&ctrls))
        {
            perror("Failed to set exposure");
            //      exit(1);
        }

        if(cvWaitKey(30)=='c')
            break;
    }


    if(-1 == ioctl(fd, VIDIOC_STREAMOFF, &buf.type))
    {
        perror("Stop Capture");
        //  return ;
    }




    return ;
}

