#include "blend.h"
Blend::Blend(IplImage *s1, IplImage *s2, CvMat *H)
    :dst(NULL)
{
    dst = cvCloneImage(ImageTransform(s1, H));
}

IplImage *Blend::GetBlendImage()
{
    return dst;
}

IplImage *Blend::ImageTransform(IplImage *s, CvMat *H)
{
    int width = s->width, height = s->height;
    CvPoint leftTop = PointXform(0, 0, H);
    CvPoint leftBottom = PointXform(height - 1, 0, H);
    CvPoint rightTop = PointXform(0, width - 1, H);
    CvPoint rightBottom = PointXform(height - 1, width - 1, H);

    IplImage *d = cvCreateImage(cvSize(1, 1), s->depth, s->nChannels);
    return d;
}

Blend::~Blend()
{
    cvReleaseImage(&dst);
}

CvPoint Blend::PointXform(int x, int y, CvMat *H)
{
    //point transform
    CvMat XY, UV;
    double xy[3] = { x * 1.0, y * 1.0, 1.0 }, uv[3] = { 0 };

    cvInitMatHeader(&XY, 3, 1, CV_64FC1, xy, CV_AUTOSTEP);
    cvInitMatHeader(&UV, 3, 1, CV_64FC1, uv, CV_AUTOSTEP);
    cvMatMul(H, &XY, &UV);
    CvPoint xpt = cvPoint((int)(uv[0] / uv[2]), (int)(uv[1] / uv[2]));
    return xpt;
}