#ifndef BLEND_H_
#define BLEND_H_
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
class Blend {
public:
    Blend(IplImage *s1, IplImage *s2, CvMat *H);
    ~Blend();
    IplImage *GetBlendImage();
private:
    void FeatherBlend(IplImage *s1, IplImage *s2);
    void MutibandBlend(IplImage *s1, IplImage *s2, int nBand);
    IplImage *ImageTransform(IplImage *s, CvMat *H);
    CvPoint PointXform(int x, int y, CvMat *H);
    IplImage *dst;
};
#endif	//BLEND_H_