/***********************************************************
 *  --- OpenSURF ---                                       *
 *  This library is distributed under the GNU GPL. Please   *
 *  use the contact form at http://www.chrisevansdev.com    *
 *  for more information.                                   *
 *                                                          *
 *  C. Evans, Research Into Robust Visual Features,         *
 *  MSc University of Bristol, 2008.                        *
 *                                                          *
 ************************************************************/

#include "surflib.h"
#include <ctime>
#include <iostream>
#include "ransac.h"
#include "warp.h"
#include "blend.h"
#include "utils.h"

int mainStaticMatch(char *i1, char *i2)
{
    IplImage *img1, *img2;
    img1 = cvLoadImage(i1);
    img2 = cvLoadImage(i2);

    IpVec ipts1, ipts2;
    surfDetDes(img1, ipts1, false, 4, 4, 2, 0.0001f);
    surfDetDes(img2, ipts2, false, 4, 4, 2, 0.0001f);

    IpPairVec matches;
    getMatches(ipts1, ipts2, matches);

    int nMatch = static_cast<int>(matches.size());
    std::vector<CvPoint2D64f>vp1(nMatch), vp2(nMatch);
    std::vector<std::pair<CvPoint2D64f, CvPoint2D64f> >vpp;

    for(int i = 0; i < nMatch; ++i) {
        vp1[i] = cvPoint2D64f(matches[i].first.x, matches[i].first.y);
        vp2[i] = cvPoint2D64f(matches[i].second.x, matches[i].second.y);
        vpp.push_back(std::make_pair(vp1[i], vp2[i]));
    }

    Ransac R(vpp, 4);
    std::cout << R.focal << std::endl;
    PrintCvMat(R.homo);

    ImageWarp I1(img1, R.focal, vp1);
    cvSaveImage("warp1.jpg", I1.GetWarpedImage());

    ImageWarp I2(img2, R.focal, vp2);
    cvSaveImage("warp2.jpg", I2.GetWarpedImage());

    std::vector<std::pair<CvPoint2D64f, CvPoint2D64f> >vpp_warp;
    for(int i = 0; i < nMatch; ++i) {
        vpp_warp.push_back(std::make_pair(vp1[i], vp2[i]));
    }
    Ransac RR(vpp_warp, 4);
    PrintCvMat(RR.homo);

    Blend B(I1.GetWarpedImage(), I2.GetWarpedImage(), RR.homo);
    cvSaveImage("trans.jpg", B.GetBlendImage());
    return 0;
}


int main(int argc, char *argv[])
{
    if (argc != 3) {
        std::cout << "Usage " << argv[0] << " img1 img2" << std::endl;
        return -1;
    }
    return mainStaticMatch(argv[1], argv[2]);
}
