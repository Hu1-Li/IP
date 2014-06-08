/********************************************************************
	created:	2014-06-06   9:45
	author:		leah
    to-do:      Levenberg¨CMarquardt algorithm
*********************************************************************/
#ifndef RANSAC_H_
#define RANSAC_H_
#include <vector>
#include <opencv/cxcore.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
typedef std::vector<std::pair<CvPoint2D64f, CvPoint2D64f> > VPP;
class Ransac
{
    public:
        Ransac(VPP &match_pairs, int m);
        Ransac();
        ~Ransac();
        double focal;
        CvMat *homo;
        CvMat *R;
        CvMat *T;
    private:
        double EstimateFocal();
        CvMat *LeastSquaresHomography(int n, std::vector<CvPoint2D64f> &pts, std::vector<CvPoint2D64f> &mpts);
        double HomographyError(CvPoint2D64f pt, CvPoint2D64f mpt, CvMat *H);
        int CalcMinInliers(int n, int m);
        int CalcInliers(VPP &match_pairs, CvMat *H);
        double log_factorial(int n);
        CvPoint2D64f PointXform(CvPoint2D64f pt, CvMat *H);
        void ExtractRT();
        void LM(CvMat *H);
        void calcRTfromHomo(CvMat* H);

        double error_threshold;
        double p_bad_xform;
        double ransac_prob_bad_supp;
        double ransac_inlier_frac_est;
};
#endif	//RANSAC_H_