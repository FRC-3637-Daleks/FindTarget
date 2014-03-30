#include "findtarget.h"

#include <arpa/inet.h>
#include <cmath>
#include <iostream>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#ifdef RASPI_COMPILE
#include <raspicam/raspicam_cv.h>
#endif

using namespace cv;
using namespace std;

static bool verbose = false;

//UDP connection variables
struct sockaddr_in serverSocketAddr;
int serverSocket;
char sockBuf[256];

int
initializeSocket()
{
    if ((serverSocket=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
        cerr << "Error creating socket" << endl;
        return 1;
    }

    bzero((char *)&serverSocketAddr, sizeof(sockaddr_in));

    serverSocketAddr.sin_family      = AF_INET;
    serverSocketAddr.sin_port        = htons(SERVER_PORT);
    if (inet_aton(SERVER_ADDR, &serverSocketAddr.sin_addr)==0) {
        cerr << "Error converting ip address" << endl;
        return 1;
    }

    return 0;
}

int
sendPacket(const bool isHot, const int range = RANGE_INVALID)
{
    string hotStr = isHot ? "true" : "false";
    string data = string("HOT=") + hotStr + string(" DIST=") + to_string(range);
    if (verbose)
        cout << "Sending data: " << data << endl;
    strncpy(sockBuf, data.c_str(), sizeof(sockBuf));
    if(sendto(serverSocket, sockBuf, sizeof(sockBuf), 0, (struct sockaddr *)&serverSocketAddr, sizeof(struct sockaddr_in)) == -1) {
        cerr << "Error sending data" << endl;
        return 1;
    }
    return 0;
}

double
toRadians(double degs)
{
    return (degs * 3.14159265) / 180.0;
}

IplImage *
thresholdImage(Mat image)
{
    IplImage inp = image;
    IplImage *hsv;
    IplImage *bin;

    hsv = cvCreateImage(cvGetSize(&inp), inp.depth, 3);
    bin = cvCreateImage(cvGetSize(&inp), inp.depth, 1);

    cvCvtColor(&inp, hsv, CV_BGR2HSV);
    cvInRangeS(hsv, cvScalar(HUE_MIN_FILTER, SAT_MIN_FILTER, VAL_MIN_FILTER),
               cvScalar(HUE_MAX_FILTER, SAT_MAX_FILTER, VAL_MAX_FILTER), bin);

    cvReleaseImage(&hsv);
    return bin;
}

void
processImage(Mat image)
{
    IplImage *bin;
    CvMat *vProject;

    int avgFrontLen = 0;
    int avgBackLen = 0;

    // Threshold image to target values, creating a binary (monocolor) image which is easy to process
    bin = thresholdImage(image);

    // Apply cvReduce to create a 1d array from the rows
    vProject = cvCreateMat(bin->height, 1, CV_64FC1);
    cvReduce(bin, vProject, 1, CV_REDUCE_SUM);

    vector<int> vertTargetCoords(2, 0);
    vector<int> horizTargetCoords(2, 0);

    for (int i = NUM_CONTINOUS_OVER_THRESH; i < vProject->rows-NUM_CONTINOUS_OVER_THRESH; i++) {
        int len = (int)(cvmGet(vProject,i,0)/255.0);

        if (len > SMALL_TARGET_WIDTH) {
            for(int offset = 0; offset<NUM_CONTINOUS_OVER_THRESH; offset++) {
                avgFrontLen += (int)(cvmGet(vProject, i + offset, 0)/255.0);
                avgBackLen  += (int)(cvmGet(vProject, i - offset, 0)/255.0);
            }
            avgFrontLen /= NUM_CONTINOUS_OVER_THRESH;
            avgBackLen /= NUM_CONTINOUS_OVER_THRESH;
            if(len >= LARGE_TARGET_WIDTH) {
                if(horizTargetCoords[0] == 0) {
                    if(avgFrontLen >= LARGE_TARGET_WIDTH)
                        horizTargetCoords[0] = i;
                }
                else {
                    if(avgBackLen >= LARGE_TARGET_WIDTH)
                        horizTargetCoords[1] = i;
                }
            }
            else {
                if(vertTargetCoords[0] == 0) {
                    if(avgFrontLen > SMALL_TARGET_WIDTH && avgFrontLen < LARGE_TARGET_WIDTH)
                        vertTargetCoords[0] = i;
                }
                else {
                    if(avgFrontLen > SMALL_TARGET_WIDTH && avgFrontLen < LARGE_TARGET_WIDTH)
                        vertTargetCoords[1] = i;
                }
            }
        }
    }
    bool isHot = !((horizTargetCoords[0] == 0) && (horizTargetCoords[1] == 0));

    double range = RANGE_INVALID;

#ifdef FEATURE_RANGE
    if(!((vertTargetCoords[0] == 0) && (vertTargetCoords[1] == 0))) {
        double theta = (((double)image.rows - vertTargetCoords[0]) / image.rows * CAMERA_FOV_DEG);
        double angleOffset = CAMERA_FOV_DEG / 2 - CAMERA_OFFSET_DEG;
        double thetaOffset = theta - angleOffset;
        range = (TARGET_TOP_HEIGHT_IN - CAMERA_HEIGHT_IN)/tan(toRadians(thetaOffset));
        cout << "Calculated range: " << range << " in OR " << range / 12.0 << " ft" << endl;
    }
    else
        cout << "No target found for range" << endl;
#endif

    if(isHot)
        cout << "Goal is hot" << endl;
    else
        cout << "Goal is not hot " << endl;

    sendPacket(isHot, range);

    cvReleaseMat(&vProject);
    cvReleaseImage(&bin);
}

int
main(int argc, char **argv)
{
    cout << "Welcome to FindTarget" << endl;

    if(initializeSocket() != 0) {
        cerr << "Error initializing socket" << endl;
        exit(1);
    }

    Mat image;
    int cnt;
    time_t start, stop;

#ifdef RASPI_COMPILE
    raspicam::RaspiCam_Cv cam;
    cam.set(CV_CAP_PROP_FORMAT, IMAGE_FORMAT);
    cam.set(CV_CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH);
    cam.set(CV_CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT);


    if(!cam.open()) {
        cout << "Error opening video stream" << endl;
        return -1;
    }
    cout << "Opened video stream" << endl;

    time(&start);
    for(cnt = 0;;cnt++) {
        if(!cam.grab()) {
            cout << "No frame" << endl;
            sleep(1);
            continue;
        }
        cam.retrieve(image);

#ifdef FEATURE_SAVE_IMAGES
        imwrite("orig.jpg", image);
#endif
        processImage(image);

        if(((cnt+1) % 30) == 0) {
            time(&stop);
            printf("Processed %f frames per seconds\n", (double) cnt/(double) (stop - start));
        }
    }
#else
    cout << "Manual processing mode" << endl;
    verbose = true;
    if (argc > 1) {
        image = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    } else {
        cerr << "Not enough paramaters!" << endl;
        exit(1);
    }
    cout << "Image: " << image.cols << "x" << image.rows << endl;

    processImage(image);
#endif
}
