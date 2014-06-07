#include "ransac.h"
#include <cmath>
#include <ctime>

Ransac::Ransac(IpPairVec &match_pairs, int m)
    : focal(0.0),
      homo(NULL),
      error_threshold(2.0),
      p_bad_xform(0.01),
      ransac_prob_bad_supp(0.10),
      ransac_inlier_frac_est(0.25)
{
    homo = cvCreateMat(3, 3, CV_64FC1);
    int n = static_cast<int>(match_pairs.size());
    double p = 1.0, pInliers = ransac_inlier_frac_est;
    srand((unsigned int)(time(NULL)));
    int nMinInliers = CalcMinInliers(n, m);

    //iteration times;
    int k = 0;

    while (p > p_bad_xform) {
        std::vector<CvPoint2D64f>pts(m), mpts(m);

        for (int i = 0; i < m; ++i) {
            int k = rand() % n;
            pts[i] = cvPoint2D64f(match_pairs[k].first.x, match_pairs[k].first.y);
            mpts[i] = cvPoint2D64f(match_pairs[k].second.x, match_pairs[k].second.y);
        }

        CvMat *H = LeastSquaresHomography(4, pts, mpts);
        int nInliers = CalcInliers(match_pairs, H);

        if (nInliers > nMinInliers) {
            homo = cvCloneMat(H);
            nMinInliers = nInliers;
            pInliers = nInliers / (n * 1.0);
        }

        cvReleaseMat(&H);
        p = pow(1 - pow(pInliers, m), ++k);
    }

    if (homo) {
        EstimateFocal();
    }
}

Ransac::~Ransac()
{
    cvReleaseMat(&homo);
}

int Ransac::CalcInliers(IpPairVec &match_pairs, CvMat *H)
{
    int nInliers = 0;

    for (int i = 0; i < static_cast<int>(match_pairs.size()); ++i) {
        CvPoint2D64f pt = cvPoint2D64f(match_pairs[i].first.x, match_pairs[i].first.y);
        CvPoint2D64f mpt = cvPoint2D64f(match_pairs[i].second.x, match_pairs[i].second.y);
        double p_error = HomographyError(pt, mpt, H);

        if (p_error <= error_threshold) {
            nInliers++;
        }
    }

    return nInliers;
}

int Ransac::CalcMinInliers(int n, int m)
{
    double pi, sum;
    int i, j;

    for (j = m + 1; j <= n; j++) {
        sum = 0;

        for (i = j; i <= n; i++) {
            pi = (i - m) * log(ransac_prob_bad_supp) +
                 (n - i + m) * log(1.0 - ransac_prob_bad_supp) +
                 log_factorial(n - m) -
                 log_factorial(i - m) -
                 log_factorial(n - i);
            sum += exp(pi);
        }

        if (sum < p_bad_xform) {
            break;
        }
    }

    return j;
}

double Ransac::log_factorial(int n)
{
    double f = 0;
    int i;

    for (i = 1; i <= n; i++) {
        f += log(i);
    }

    return f;
}


CvMat *Ransac::LeastSquaresHomography(int n, std::vector<CvPoint2D64f> &pts, std::vector<CvPoint2D64f> &mpts)
{
    CvMat *A, *B, *H, X;
    double x[9];

    A = cvCreateMat(2 * n, 8, CV_64FC1);
    B = cvCreateMat(2 * n, 1, CV_64FC1);
    H = cvCreateMat(3, 3, CV_64FC1);
    X = cvMat(8, 1, CV_64FC1, x);

    cvZero(A);

    for (int i = 0; i < n; ++i) {
        cvmSet(A, i, 0, pts[i].x);
        cvmSet(A, i, 1, pts[i].y);
        cvmSet(A, i, 2, 1.0);

        cvmSet(A, i, 6, -pts[i].x * mpts[i].x);
        cvmSet(A, i, 7, -pts[i].y * mpts[i].x);

        cvmSet(B, i, 0, mpts[i].x);


        cvmSet(A, i + n, 3, pts[i].x);
        cvmSet(A, i + n, 4, pts[i].y);
        cvmSet(A, i + n, 5, 1.0);

        cvmSet(A, i + n, 6, -pts[i].x * mpts[i].y);
        cvmSet(A, i + n, 7, -pts[i].y * mpts[i].y);

        cvmSet(B, i + n, 0, mpts[i].y);
    }

    cvSolve(A, B, &X, CV_SVD);
    x[8] = 1.0;
    X = cvMat(3, 3, CV_64FC1, x);
    cvConvert(&X, H);

    cvReleaseMat(&A);
    cvReleaseMat(&B);
    return H;
}

double Ransac::HomographyError(CvPoint2D64f pt, CvPoint2D64f mpt, CvMat *H)
{
    CvPoint2D64f xpt = PointXform(pt, H);

    //dist from xpt to mpt
    double dx = xpt.x - mpt.x;
    double dy = xpt.y - mpt.y;
    return sqrt(dx * dx + dy * dy);
}

/*
Performs a perspective transformation on a single point.  That is, for a
    point (x, y) and a 3 x 3 matrix T this function returns the point
    (u, v), where

    [x' y' w']^T = T * [x y 1]^T,

    and

    (u, v) = (x'/w', y'/w').

    Note that affine transforms are a subset of perspective transforms.

    @param pt a 2D point
    @param T a perspective transformation matrix

    @return Returns the point (u, v) as above.
*/
CvPoint2D64f Ransac::PointXform(CvPoint2D64f pt, CvMat *H)
{
    //point transform
    CvMat XY, UV;
    double xy[3] = { pt.x, pt.y, 1.0 }, uv[3] = { 0 };

    cvInitMatHeader(&XY, 3, 1, CV_64FC1, xy, CV_AUTOSTEP);
    cvInitMatHeader(&UV, 3, 1, CV_64FC1, uv, CV_AUTOSTEP);
    cvMatMul(H, &XY, &UV);
    CvPoint2D64f xpt = cvPoint2D64f(uv[0] / uv[2], uv[1] / uv[2]);
    return xpt;
}

double Ransac::EstimateFocal()
{
    int size = homo->cols * homo->rows;
    std::vector<double> M(size);

    for (int i = 0; i < size; ++i) {
        M[i] = cvmGet(homo, i / homo->cols, i % homo->rows);
    }

    double f = -M[2] * M[5] * M[6] * M[7] / ((M[0] * M[3] + M[1] * M[4]) * (M[0] * M[1] + M[3] * M[4]));
    focal = 1000 * sqrt(sqrt(f));
    return focal;
}