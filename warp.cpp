#include "warp.h"
#include "ransac.h"
#include <cmath>

ImageWarp::ImageWarp(IplImage *s, double f, std::vector<CvPoint2D64f> &match_point)
    : dst(NULL), H(NULL)
{
    dst = cvCloneImage(CylinderProjection(s, f, match_point));
}

ImageWarp::~ImageWarp()
{
    cvReleaseImage(&dst);
    cvReleaseMat(&H);
}

IplImage *ImageWarp::GetWarpedImage()
{
    return this->dst;
}

IplImage *ImageWarp::CylinderProjection(IplImage *s, double f, std::vector<CvPoint2D64f> &match_point)
{
    int height = s->height, width = s->width;
    double xi, yi, xmin, ymin, xmax, ymax, xc, yc;
    uchar rgb[3];

    xmin = 1.0E+10;
    xmax = -1.0E+10;
    ymin = 1.0E+10;
    ymax = -1.0E+10;

    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            xi = j - width * 0.5;
            yi = i - height * 0.5;

            Coordinate2Cylinder(xi, yi, f, xc, yc);
            xmin = std::min(xc, xmin);
            xmax = std::max(xc, xmax);
            ymin = std::min(yc, ymin);
            ymax = std::max(yc, ymax);
        }
    }

    double dx = (xmax - xmin) / (double)(width - 1);
    double dy = (ymax - ymin) / (double)(height - 1);

    double xmid = (xmin + xmax) * 0.5;
    double ymid = (ymin + ymax) * 0.5;

    IplImage *D = cvCreateImage(cvSize(width, height), s->depth, s->nChannels);


    //feature warp
    int nMatch = static_cast<int>(match_point.size());
    for(int i = 0; i < nMatch; ++i) {
        xc = xmin + match_point[i].x * dx;
        yc = ymin + match_point[i].y * dy;
        Cylinder2Coodinate(xc, yc, f, xi, yi);
        match_point[i].x = xi;
        match_point[i].y = yi;
    }

    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            xc = xmin + j * dx;
            yc = ymin + i * dy;

            Cylinder2Coodinate(xc, yc, f, xi, yi);
            BilinearInterpolate(s, xi, yi, rgb);

            uchar *pd = (uchar *)(D->imageData + i * D->widthStep);
            pd[3 * j] = rgb[0];
            pd[3 * j + 1] = rgb[1];
            pd[3 * j + 2] = rgb[2];
        }
    }

    return D;
}

CvMat *ImageWarp::GetTransformMatrix()
{
    return this->H;
}

void ImageWarp::BilinearInterpolate(IplImage *s, double x, double y, uchar *rgb)
{
    int xmax = s->width;
    int ymax = s->height;
    float xmid = 0.5 * xmax;
    float ymid = 0.5 * ymax;

    x += xmid;
    y += ymid;

    if (x < 0.0 || x > xmax ) {
        rgb[0] = 0.0;
        rgb[1] = 0.0;
        rgb[2] = 0.0;
        return;
    }

    if (y < 0.0 || y > ymax ) {
        rgb[0] = 0.0;
        rgb[1] = 0.0;
        rgb[2] = 0.0;
        return;
    }

    int i = int( floor(x) );
    int j = int( floor(y) );
    assert(i >= 0 && i < xmax);
    assert(j >= 0 && j < ymax);

    double xi  = x - (double)i;
    double eta = y - (double)j;

    assert(xi  >= 0.0 && xi  < 1.0);
    assert(eta >= 0.0 && eta < 1.0);

    double weight[4];

    weight[0] = (1 - xi) * (1 - eta);
    weight[1] = xi * (1 - eta);
    weight[2] = xi * eta;
    weight[3] = (1 - xi) * eta;

    uchar phi[4];

    for (int ic = 0; ic < 3; ic++) {
        phi[0] = GetColor(s, i, j, ic );
        phi[1] = GetColor(s, i + 1, j, ic );
        phi[2] = GetColor(s, i + 1, j + 1, ic );
        phi[3] = GetColor(s, i, j + 1, ic );
        double sum = 0.0;

        for (int k = 0; k < 4; k++) {
            sum += phi[k] * weight[k];
        }

        rgb[ic] = (uchar)sum;
    }
}

uchar ImageWarp::GetColor(IplImage *s, int i, int j, int channel)
{
    uchar *p = (uchar *)(s->imageData + j * s->widthStep);
    return p[3 * i + channel];
}

void Warp::Coordinate2Cylinder(double x, double y, double f, double &Xc, double &Yc)
{
    double theta = atan(x / f);
    double h = y / sqrt(x * x + f * f);
    Xc = f * theta;
    Yc = f * h;
}

void Warp::Cylinder2Coodinate(double xc, double yc, double f, double &X, double &Y)
{
    double theta = xc / f;
    double h = yc / f;
    X = f * tan(theta);
    Y = h * sqrt(X * X + f * f);
}

// My easy version.
/*
    int width = s->width;
    int height = s->height;
    CvPoint center = cvPoint(height / 2 , width / 2);

    dst = cvCreateImage(cvSize(s->width, s->height), s->depth, s->nChannels);

    double theta, h, xcap, ycap, zcap, xn, yn;
    int x_, y_;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            theta = (x - center.x) / f;
            h = (y - center.y) / f;
            xcap = sin(theta);
            ycap = h;
            zcap = cos(theta);

            xn = xcap / zcap;
            yn = ycap / zcap;

            x_ = floor(f * xn + center.x);
            y_ = floor(f * yn + center.y);

            if (x_ > 0 && x_ < width && y_ > 0 && y_ < height) {
                uchar *ps = (uchar *)(s->imageData + y_ * s->widthStep);
                uchar *pd = (uchar *)(dst->imageData + y * dst->widthStep);
                pd[3 * x] = ps[3 * x_];
                pd[3 * x + 1] = ps[3 * x_ + 1];
                pd[3 * x + 2] = ps[3 * x_ + 2];
            }
        }
    }
*/