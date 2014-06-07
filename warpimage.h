#ifndef WARPIMAGE_H_
#define WARPIMAGE_H_
#include <opencv/cv.h>
#include <opencv/cxcore.h>
class ImageWarp {
public:
    ImageWarp(IplImage *s, double f);
    ~ImageWarp();
private:
    IplImage *dst;
};
#endif	//WARPIMAGE_H_