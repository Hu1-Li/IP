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
#include "kmeans.h"
#include <ctime>
#include <iostream>
#include "ransac.h"
//-------------------------------------------------------
// In order to you use OpenSURF, the following illustrates
// some of the simple tasks you can do.  It takes only 1
// function call to extract described SURF features!
// Define PROCEDURE as:
//  - 1 and supply image path to run on static image
//  - 2 to capture from a webcam
//  - 3 to match find an object in an image (work in progress)
//  - 4 to display moving features (work in progress)
//  - 5 to show matches between static images
#define PROCEDURE 5

//-------------------------------------------------------

int mainImage(void)
{
    // Declare Ipoints and other stuff
    IpVec ipts;
    IplImage *img = cvLoadImage("imgs/sf.jpg");

    // Detect and describe interest points in the image
    clock_t start = clock();
    surfDetDes(img, ipts, false, 5, 4, 2, 0.0004f);
    clock_t end = clock();

    std::cout << "OpenSURF found: " << ipts.size() << " interest points" << std::endl;
    std::cout << "OpenSURF took: " << float (end - start) / CLOCKS_PER_SEC << " seconds" << std::endl;

    // Draw the detected points
    drawIpoints(img, ipts);

    // Display the result
    showImage(img);

    return 0;
}

//-------------------------------------------------------

int mainVideo(void)
{
    // Initialise capture device
    CvCapture *capture = cvCaptureFromCAM(CV_CAP_ANY);

    if (!capture) {
        error("No Capture");
    }

    // Initialise video writer
    //cv::VideoWriter vw("c:\\out.avi", CV_FOURCC('D','I','V','X'),10,cvSize(320,240),1);
    //vw << img;

    // Create a window
    cvNamedWindow("OpenSURF", CV_WINDOW_AUTOSIZE);

    // Declare Ipoints and other stuff
    IpVec ipts;
    IplImage *img = NULL;

    // Main capture loop
    while (1) {
        // Grab frame from the capture source
        img = cvQueryFrame(capture);

        // Extract surf points
        surfDetDes(img, ipts, false, 4, 4, 2, 0.004f);

        // Draw the detected points
        drawIpoints(img, ipts);

        // Draw the FPS figure
        drawFPS(img);

        // Display the result
        cvShowImage("OpenSURF", img);

        // If ESC key pressed exit loop
        if ((cvWaitKey(10) & 255) == 27) {
            break;
        }
    }

    cvReleaseCapture(&capture);
    cvDestroyWindow("OpenSURF");
    return 0;
}


//-------------------------------------------------------


int mainMatch(void)
{
    // Initialise capture device
    CvCapture *capture = cvCaptureFromCAM(CV_CAP_ANY);

    if (!capture) {
        error("No Capture");
    }

    // Declare Ipoints and other stuff
    IpPairVec matches;
    IpVec ipts, ref_ipts;

    // This is the reference object we wish to find in video frame
    // Replace the line below with IplImage *img = cvLoadImage("imgs/object.jpg");
    // where object.jpg is the planar object to be located in the video
    IplImage *img = cvLoadImage("beaver.png");

    if (img == NULL) {
        error("Need to load reference image in order to run matching procedure");
    }

    CvPoint src_corners[4] = {
        {0, 0},
        {img->width, 0},
        {img->width, img->height},
        {0, img->height}
    };
    CvPoint dst_corners[4];

    // Extract reference object Ipoints
    surfDetDes(img, ref_ipts, false, 3, 4, 3, 0.004f);
    drawIpoints(img, ref_ipts);
    showImage(img);

    // Create a window
    cvNamedWindow("OpenSURF", CV_WINDOW_AUTOSIZE);

    // Main capture loop
    while (true) {
        // Grab frame from the capture source
        img = cvQueryFrame(capture);

        // Detect and describe interest points in the frame
        surfDetDes(img, ipts, false, 3, 4, 3, 0.004f);

        // Fill match vector
        getMatches(ipts, ref_ipts, matches);

        // This call finds where the object corners should be in the frame
        if (translateCorners(matches, src_corners, dst_corners)) {
            // Draw box around object
            for (int i = 0; i < 4; i++) {
                CvPoint r1 = dst_corners[i % 4];
                CvPoint r2 = dst_corners[(i + 1) % 4];
                cvLine(img, cvPoint(r1.x, r1.y), cvPoint(r2.x, r2.y), cvScalar(255, 255, 255), 3);
            }

            for (unsigned int i = 0; i < matches.size(); ++i) {
                drawIpoint(img, matches[i].first);
            }
        }

        // Draw the FPS figure
        drawFPS(img);

        // Display the result
        cvShowImage("OpenSURF", img);

        // If ESC key pressed exit loop
        if ((cvWaitKey(10) & 255) == 27) {
            break;
        }
    }

    // Release the capture device
    cvReleaseCapture(&capture);
    cvDestroyWindow("OpenSURF");
    return 0;
}


//-------------------------------------------------------


int mainMotionPoints(void)
{
    // Initialise capture device
    CvCapture *capture = cvCaptureFromCAM(CV_CAP_ANY);

    if (!capture) {
        error("No Capture");
    }

    // Create a window
    cvNamedWindow("OpenSURF", CV_WINDOW_AUTOSIZE);

    // Declare Ipoints and other stuff
    IpVec ipts, old_ipts, motion;
    IpPairVec matches;
    IplImage *img;

    // Main capture loop
    while (1) {
        // Grab frame from the capture source
        img = cvQueryFrame(capture);

        // Detect and describe interest points in the image
        old_ipts = ipts;
        surfDetDes(img, ipts, true, 3, 4, 2, 0.0004f);

        // Fill match vector
        getMatches(ipts, old_ipts, matches);

        for (unsigned int i = 0; i < matches.size(); ++i) {
            const float &dx = matches[i].first.dx;
            const float &dy = matches[i].first.dy;
            float speed = sqrt(dx * dx + dy * dy);

            if (speed > 5 && speed < 30) {
                drawIpoint(img, matches[i].first, 3);
            }
        }

        // Display the result
        cvShowImage("OpenSURF", img);

        // If ESC key pressed exit loop
        if ((cvWaitKey(10) & 255) == 27) {
            break;
        }
    }

    // Release the capture device
    cvReleaseCapture(&capture);
    cvDestroyWindow("OpenSURF");
    return 0;
}


//-------------------------------------------------------

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

    IplImage *xformed = cvCreateImage(cvSize(1000, 570), IPL_DEPTH_8U, 3);
    cvWarpPerspective(img2, xformed, R.homo, CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll(0));
    cvSaveImage("xformed.jpg", xformed);

    unsigned int nMatches = matches.size();
    std::cout << "Matches: " << nMatches;

    std::vector<cv::Point2f> vp1(nMatches), vp2(nMatches);
    for (unsigned int i = 0; i < nMatches; ++i) {
        cv::Point2f tmp;
        tmp.x = matches[i].first.x;
        tmp.y = matches[i].first.y;
        vp1[i] = tmp;

        tmp.x = matches[i].second.x;
        tmp.y = matches[i].second.y;
        vp2[i] = tmp;
    }

    cv::Mat homo = cv::findHomography(vp1, vp2, CV_RANSAC, 1);
    std::cout << homo << std::endl;

    return 0;
}

//-------------------------------------------------------

int mainKmeans(void)
{
    IplImage *img = cvLoadImage("beaver_xform.png");
    IpVec ipts;
    Kmeans km;

    // Get Ipoints
    surfDetDes(img, ipts, true, 3, 4, 2, 0.0006f);

    for (int repeat = 0; repeat < 10; ++repeat) {

        IplImage *img = cvLoadImage("beaver_xform.png");
        km.Run(&ipts, 5, true);
        drawPoints(img, km.clusters);

        for (unsigned int i = 0; i < ipts.size(); ++i) {
            cvLine(
                img,
                cvPoint(ipts[i].x, ipts[i].y),
                cvPoint(km.clusters[ipts[i].clusterIndex].x, km.clusters[ipts[i].clusterIndex].y),
                cvScalar(255, 255, 255)
            );
        }

        showImage(img);
    }

    return 0;
}

//-------------------------------------------------------

int main(int argc, char *argv[])
{
    if (argc != 3) {
        std::cout << "Usage " << argv[0] << "img1 img2" << std::endl;
        return -1;
    }

    if (PROCEDURE == 1) {
        return mainImage();
    }

    if (PROCEDURE == 2) {
        return mainVideo();
    }

    if (PROCEDURE == 3) {
        return mainMatch();
    }

    if (PROCEDURE == 4) {
        return mainMotionPoints();
    }

    if (PROCEDURE == 5) {
        return mainStaticMatch(argv[1], argv[2]);
    }

    if (PROCEDURE == 6) {
        return mainKmeans();
    }
}
