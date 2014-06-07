#include "warpimage.h"

ImageWarp::ImageWarp(IplImage *s, double f)
    :dst(NULL)
{
    int width = s->width;
    int height = s->height;
    CvPoint center = cvPoint(height / 2 , width / 2);
}

ImageWarp::~ImageWarp()
{
    cvReleaseImage(&dst);
}