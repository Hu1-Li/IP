#ifndef WARPIMAGE_H_
#define WARPIMAGE_H_
#include <opencv/cv.h>
#include <opencv/cxcore.h>
class ImageWarp {
public:
    ImageWarp(IplImage *s, double focal);
    ~ImageWarp();
    IplImage *dst;
    
private:
    IplImage* CylinderProjection(IplImage *s, double focal);
    void Coordinate2Cylinder(double x, double y, double f, double &Xc, double &Yc);
    void Cylinder2Coodinate(double xc, double yc, double f, double &X, double &Y);
    void BilinearInterpolate(IplImage *s, double x, double y, uchar *rgb);
    uchar GetColor(IplImage *s, int i, int j, int channel);
};
#endif	//WARPIMAGE_H_