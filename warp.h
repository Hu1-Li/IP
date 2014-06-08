#ifndef WARPIMAGE_H_
#define WARPIMAGE_H_
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <vector>
#include "ipoint.h"
class Warp {
public:
    Warp()
    {

    }

    ~Warp()
    {

    }

    void Coordinate2Cylinder(double x, double y, double f, double &Xc, double &Yc);
    void Cylinder2Coodinate(double xc, double yc, double f, double &X, double &Y);
};

class ImageWarp : public Warp {
public:
    ImageWarp(IplImage *s1, double focal, std::vector<CvPoint2D64f> &match_point);
    ~ImageWarp();
    IplImage *GetWarpedImage();
    CvMat *GetTransformMatrix();
private:
    IplImage* CylinderProjection(IplImage *s1, double focal, std::vector<CvPoint2D64f> &match_point);
    void BilinearInterpolate(IplImage *s, double x, double y, uchar *rgb);
    uchar GetColor(IplImage *s, int i, int j, int channel);
    IplImage *dst;
    CvMat *H;
};
#endif	//WARPIMAGE_H_