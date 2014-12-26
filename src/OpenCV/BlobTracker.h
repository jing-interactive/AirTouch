/*
* vBlobTracker.h
* by stefanix
* Thanks to touchlib for the best fit algorithm!
*
* This class tracks blobs between frames.
* Most importantly it assignes persistent id to new blobs, correlates
* them between frames and removes them as blobs dissappear. It also
* compensates for ghost frames in which blobs momentarily dissappear.
*
* Based on the trackning it fires events when blobs come into existence,
* move around, and disappear. The object which receives the callbacks
* can be specified with setListener().
*
*/
#pragma once

#include <map>
#include <vector>

#include "Blob.h"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/video/video.hpp"

///////////////////////////////////////////////////////////////////////////////////////////

// This cleans up the foreground segmentation mask derived from calls to cvBackCodeBookDiff
//
// mask			Is a grayscale (8 bit depth) "raw" mask image which will be cleaned up
//
// OPTIONAL PARAMETERS:
// poly1_hull0	If set, approximate connected component by (DEFAULT) polygon, or else convex hull (false)
// areaScale 	Area = image (width*height)*areaScale.  If contour area < this, delete that contour (DEFAULT: 0.1)
//
void vFindBlobs(cv::Mat& src, std::vector<vBlob>& blobs, int minArea = 1, int maxArea = 3072000, bool convexHull=false, bool (*sort_func)(const vBlob& a, const vBlob& b)  = NULL);

void vFindBlobs(cv::Mat& src, std::vector<vBlob>& blobs, std::vector<std::vector<vDefect>>& defects, int minArea=1, int maxArea=3072000);

class vBlobTracker
{
public:
    vBlobTracker();
    void trackBlobs(const std::vector<vBlob>& newBlobs);

    std::vector<vTrackedBlob>	trackedBlobs; //tracked blobs
    std::vector<vTrackedBlob>  deadBlobs;

private:
    unsigned int						IDCounter;	  //counter of last blob

protected:
    //blob Events
    void doBlobOn(vTrackedBlob& b );
    void doBlobMoved(vTrackedBlob& b );
    void doBlobOff(vTrackedBlob& b );
};

struct HaarFinder
{
    std::vector<vBlob> blobs;
    float scale;
    //
    bool init(char* cascade_name);
    void find(const cv::Mat& img, int minArea = 1, bool findAllFaces = true);

    HaarFinder();

protected:

    cv::CascadeClassifier mClassifier;
};


//todo:
struct IBackGround
{
    virtual void init(cv::Mat initial, void* param = NULL) = 0;

    virtual void update(cv::Mat image, int mode = 0) = 0;

    virtual void setIntParam(int idx, int value){}
    virtual cv::Mat getForeground() = 0;
    virtual cv::Mat getBackground() = 0;

    virtual ~IBackGround(){}
};

struct IStaticBackground : IBackGround
{	
    cv::Mat frame;
    cv::Mat bg;
    cv::Mat fore;

    int threshes[2];

    IStaticBackground();

    virtual ~IStaticBackground();
    virtual void setIntParam(int idx, int value);
    cv::Mat getForeground();
    cv::Mat getBackground();
};

#define DETECT_BOTH 0
#define DETECT_DARK 1
#define DETECT_BRIGHT 2

struct vBackGrayDiff: public IStaticBackground
{
    void init(cv::Mat initial, void* param = NULL);
    ///mode: 0-> ¼ì²âÃ÷Óë°µ 1->¼ì²âºÚ°µ 2->¼ì²âÃ÷ÁÁ
    void update(cv::Mat image, int mode = DETECT_BOTH);
};

struct vBackColorDiff: public IStaticBackground
{
    int nChannels;
    void init(cv::Mat initial, void* param = NULL);

    ///mode: 0-> ¼ì²âÃ÷Óë°µ 1->¼ì²âºÚ°µ 2->¼ì²âÃ÷ÁÁ
    void update(cv::Mat image, int mode = DETECT_BOTH);
};

//ÈýÖ¡²îÖµ·¨
struct vThreeFrameDiff: public IStaticBackground
{
    //TODO: how to deal with gray
    cv::Mat grays[3];
    cv::Mat grayDiff ;

    void init(cv::Mat initial, void* param = NULL);

    void update(cv::Mat image, int mode = 0);

    cv::Mat getForeground();
    cv::Mat getBackground();
};

struct vBackGrayAuto: public IStaticBackground
{
    void init(cv::Mat initial, void* param = NULL);
    void update(cv::Mat image, int mode = DETECT_BOTH);

    cv::Mat mBackF32;
};
