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
#include "warpimage.h"

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

    Ransac R(matches, 4);
    std::cout << R.focal << std::endl;

    //print homo
    for(int i = 0; i < R.homo->rows; ++i) {
        for(int j = 0; j < R.homo->cols; ++j) {
            std::cout << cvmGet(R.homo, i, j) << " ";
        }
        std::cout << std::endl;
    }

    ImageWarp I1(img1, R.focal);
    cvSaveImage("warp1.jpg", I1.dst);

    ImageWarp I2(img2, R.focal);
    cvSaveImage("warp2.jpg", I2.dst);

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
