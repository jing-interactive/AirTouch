//vinjn's wrapper for OpenCV

#pragma once

#pragma warning( disable: 4244 )
#pragma warning( disable: 4996 )
#pragma warning( disable: 4305 )
#pragma warning( disable: 4018 )
#pragma warning( disable: 4099 )
#pragma warning( disable: 4819 )

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/opengl_interop.hpp"

#include <stdio.h>
#include <stdarg.h>
#include <time.h>
#include <vector>

#include "point2d.h"

#define OPENCV_VERSION CVAUX_STR(CV_MAJOR_VERSION)""CVAUX_STR(CV_MINOR_VERSION)""CVAUX_STR(CV_SUBMINOR_VERSION)
// 210/231/243/249/290

// enum T_VideoCodec
// {
// 	T_MPEG = CV_FOURCC('P','I','M','1'),		//	= MPEG-1 codec
// 	T_MJPG = CV_FOURCC('M','J','P','G') ,		//	= motion-jpeg codec (does not work well)
// 	T_MP42 = CV_FOURCC('M', 'P', '4', '2'),		// = MPEG-4.2 codec
// 	T_DIV3 = CV_FOURCC('D', 'I', 'V', '3'),		// = MPEG-4.3 codec
// 	T_DIVX = CV_FOURCC('D', 'I', 'V', 'X'),		// = MPEG-4 codec
// 	T_H263 = CV_FOURCC('U', '2', '6', '3'),		// = H263 codec
// 	T_H263I = CV_FOURCC('I', '2', '6', '3'),	// = H263I codec
// 	T_FLV = CV_FOURCC('F', 'L', 'V', '1'),		// = FLV1 codec
// };

void vFastCopyImageTo(const cv::Mat& src, cv::Mat& dst, const cv::Rect& roi);
void vCopyImageTo(const cv::Mat& src, cv::Mat& dst, const cv::Rect& roi);
void vFlip(cv::Mat& src, int flipHorizontal, int flipVertical);
void vDrawText(cv::Mat& img, int x,int y,char* str, cv::Scalar clr=cv::Scalar(255));
void vPolyLine(cv::Mat& dst, std::vector<cv::Point>& pts, cv::Scalar clr=cv::Scalar(255), int thick = 1);
cv::Scalar vDefaultColor(int idx);

#define show_image(img_name) do{\
    cvNamedWindow(#img_name);\
    cvShowImage(#img_name, img_name);}\
    while(0);

#define show_mat(img_name) do{\
    cv::namedWindow(#img_name);\
    cv::imshow(#img_name, img_name);}\
    while(0);

cv::Scalar rgbColor(int r, int g, int b);
const cv::Scalar CV_RED(0,0,255);
const cv::Scalar CV_GREEN(0,255,0);
const cv::Scalar CV_BLUE(255,0,0);
const cv::Scalar CV_BLACK(0,0,0);
const cv::Scalar CV_WHITE(255,255,255);
const cv::Scalar CV_GRAY(122,122,122); 

cv::Scalar vRandomColor();

#define vDrawRect(image, rc, clr) cv::rectangle(image, cv::Point(rc.x,rc.y), cv::Point(rc.x+rc.width,rc.y+rc.height), clr)

#define WRITE_(key, var) fs<<key<<var
#define WRITE_FS(var) fs<<(#var)<<(var)

#define READ_(key, var) fs[key]>>var
#define READ_FS(var) fs[#var]>>(var)

#define vGrayScale(clr, gray) cv::cvtColor(clr, gray, cv::COLOR_BGR2GRAY) 
#define vColorFul(gray, clr) cv::cvtColor(gray, clr , cv::COLOR_GRAY2BGR) 
#define vThresh(gray, thresh) cv::threshold( gray, gray, thresh, 255, cv::THRESH_BINARY )//if > thresh -> white
#define vThreshInv(gray, thresh) cv::threshold( gray, gray, thresh, 255, cv::THRESH_BINARY_INV )//if < thresh -> white
#define vOpen(img, times) cv::morphologyEx( img, img, cv::MORPH_OPEN, NULL, cv::Point(-1,-1), times );//去除白色小区域
#define vClose(img, times) cv::morphologyEx( img, img, cv::MORPH_CLOSE, NULL, cv::Point(-1,-1), times );//去除黑色小区域
#define vDilate(img, times) cv::morphologyEx( img, img, cv::MORPH_DILATE, NULL, cv::Point(-1,-1), times );
#define vErode(img, times) cv::morphologyEx( img, img, cv::MORPH_ERODE, NULL, cv::Point(-1,-1), times );

#define vFullScreen(win_name) cvSetWindowProperty(win_name, CV_WND_PROP_FULLSCREEN, 1);

#define vCreateGray(clr) cvCreateImage(cvGetSize(clr), 8, 1);
#define vCreateColor(clr) cvCreateImage(cvGetSize(clr), 8, 3);

struct VideoInput
{
    enum e_InputType
    {
        From_Image = 0,
        From_Video,
        From_Camera,
        From_Count,
    }mInputType;

    std::vector<cv::VideoCapture> mCaptures;

    cv::Mat mFrame;

    cv::Size mSize;
    int mChannel;

    int mFrameNum;

    VideoInput();

    bool initFromCamera(int cameraIndex, int captureId = 0);
    bool initFromFile(const std::string& filename, int captureId = 0);
    bool init(int argc, char** argv);

    void skipFrames(int framesToSkip);

    enum
    {
        GET_ALL_FRAMES = -1
    };
    cv::Mat getFrame();
private:
    void postInit(int captureId = 0);
};


//void vRotateImage(IplImage* image, float angle, float centreX, float centreY);

void vHighPass(const cv::Mat& src, cv::Mat& dst, int blurLevel = 10, int noiseLevel = 3);

#define vAddWeighted(src, alpha, dst) cvAddWeighted(src, alpha, dst, 1-alpha, 0, dst);

void vFillPoly(cv::Mat& img, const std::vector<cv::Point>& pt_list, const cv::Scalar& clr = cv::Scalar(255,255,255));
void vLinePoly(cv::Mat& img, const std::vector<cv::Point>& pt_list, const cv::Scalar& clr = cv::Scalar(255,255,255), int thick = 1);
void vLinePoly(cv::Mat& img, const std::vector<cv::Point2f>& pt_list, const cv::Scalar& clr = cv::Scalar(255,255,255), int thick = 1);

inline bool isPointInsideRect(int x, int y, const cv::Rect& rect)
{
    return (x >= rect.x && x <= rect.x+rect.width &&
        y >= rect.y && y <= rect.height);
}

// Object-to-object bounding-box collision detector:
bool vTestRectHitRect(const cv::Rect& object1, const cv::Rect& object2);
