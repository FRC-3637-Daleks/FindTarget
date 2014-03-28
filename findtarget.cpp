#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <cmath>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

using namespace cv;
using namespace std;

double
toRadians(double degs)
{
    return (degs * 3.14159265) / 180.0;
}

IplImage * 
thresholdImage(Mat image)
{
    int hueMinFilter =  30;
    int hueMaxFilter = 180;
    int satMinFilter = 100;
    int satMaxFilter = 256;
    int valMinFilter = 180;
    int valMaxFilter = 256;

    IplImage inp = image;
    IplImage *hsv;
    IplImage *bin;

    hsv = cvCreateImage(cvGetSize(&inp), inp.depth, 3);
    bin = cvCreateImage(cvGetSize(&inp), inp.depth, 1);

    cvCvtColor(&inp, hsv, CV_BGR2HSV);
    cvInRangeS(hsv, cvScalar(hueMinFilter, satMinFilter, valMinFilter),
                cvScalar(hueMaxFilter, satMaxFilter, valMaxFilter), bin);

    cvReleaseImage(&hsv);
    return bin;
}

void
processImage(Mat image)
{
    static IplConvKernel *morphKernel = cvCreateStructuringElementEx(3, 3, 1, 1, CV_SHAPE_RECT, NULL);
    CvMat *vProject;
    IplImage *bin;

    int kHoleClosingIterations = 9;
    int smallTargetWidth = 10;
    int wideTargetWidth = smallTargetWidth * 4;
    int kNumPixelsRequired = 5;
    int horizontalOffsetPixels;

    double kHorizontalFOVDeg = 72.0;
    double kHorizontalFOVRad = toRadians(kHorizontalFOVDeg);
    double kCameraPitchDeg = 30.0;
    double kCameraPitchRad = toRadians(kCameraPitchDeg);
    double kCameraHeight = 8.0;
    double kTargetTopHeight = 68.0;

    int avgFrontLen = 0;
    int avgBackLen = 0;

    horizontalOffsetPixels = (int)round(image.cols/kHorizontalFOVDeg);

    // Convert to HSV color space
    bin = thresholdImage(image); 

    // Fill in any gaps using binary morphology
    cvMorphologyEx(bin, bin, NULL, morphKernel, CV_MOP_CLOSE, kHoleClosingIterations);

    // Apply cvReduce to create a 1d array from the rows 
    vProject = cvCreateMat(bin->height, 1, CV_64FC1);
    cvReduce(bin, vProject, 1, CV_REDUCE_SUM);

    vector<int> vertTargetCoords(2, 0);
    vector<int> horizTargetCoords(2, 0);

    for (int i = kNumPixelsRequired; i < vProject->rows-kNumPixelsRequired; i++) {
        int len = (int)(cvmGet(vProject,i,0)/255.0);

        if (len > smallTargetWidth) {
            for(int offset = 0; offset<kNumPixelsRequired; offset++) {
                avgFrontLen += (int)(cvmGet(vProject, i + offset, 0)/255.0);
                avgBackLen  += (int)(cvmGet(vProject, i - offset, 0)/255.0);
            }
            avgFrontLen /= kNumPixelsRequired;
            avgBackLen /= kNumPixelsRequired;
            if(len >= wideTargetWidth) {
                if(horizTargetCoords[0] == 0) {
                    if(avgFrontLen >= wideTargetWidth)
                        horizTargetCoords[0] = i;
                }
                else {
                    if(avgBackLen >= wideTargetWidth)
                        horizTargetCoords[1] = i;
                }
            }
            else {
                if(vertTargetCoords[0] == 0) {
                    if(avgFrontLen > smallTargetWidth && avgFrontLen < wideTargetWidth)
                        vertTargetCoords[0] = i;
                }
                else {
                    if(avgFrontLen > smallTargetWidth && avgFrontLen < wideTargetWidth)
                        vertTargetCoords[1] = i;
                }
            }
        }
    }
    bool isHot = !((horizTargetCoords[0] == 0) && (horizTargetCoords[1] == 0));
    bool found = !((vertTargetCoords[0] == 0) && (vertTargetCoords[1] == 0));

    if(found) {
        double theta = (((double)image.rows - vertTargetCoords[0]) / image.rows * kHorizontalFOVDeg);
        double angleOffset = kHorizontalFOVDeg / 2 - kCameraPitchDeg;
        double thetaOffset = theta - angleOffset;
        double range = (kTargetTopHeight - kCameraHeight)/tan(toRadians(thetaOffset));
        cout << "Calculated range: " << range << " in OR " << range / 12.0 << " ft" << endl;
    }
    else
        cout << "No target found" << endl;

/*
    if(isHot)
        cout << "Goal is hot" << endl;
    else
        cout << "Goal is not hot " << endl;

*/
    cvReleaseMat(&vProject);
    cvReleaseImage(&bin);
}


int 
main(int, char**) 
{
    VideoCapture vcap;
    Mat image;
    int cnt;
    time_t start, stop;

    const string videoStreamAddress = "http://10.36.37.11/mjpg/video.mjpg";
    //const string videoStreamAddress = "rtsp://FRC:FRC@10.36.37.11:554/axis-media/media.amp?videocodec=h264&streamprofile=Balanced";

    if(!vcap.open(videoStreamAddress)) {
        cout << "Error opening video stream or file" << endl;
        return -1;
    }
    cout << "Opened video stream" << endl;

    time(&start);
    for(cnt = 0;;cnt++) {
        if(!vcap.read(image)) {
		cout << "No frame" << endl;
		sleep(1);
		continue;
        }
        processImage(image);
        if(((cnt+1) % 30) == 0) {
            time(&stop);
            printf("Processed %f frames per seconds\n", (double) cnt/(double) (stop - start));
        }
    }
}
