/*
* vTrackedBlob.h
* openFrameworks
*
* This class represents a blob with inter-frame information.
* This includes a persistent id to assume a persistent identity over
* time.
*
*/

#pragma once

#include "OpenCV.h"
#include "point2d.h"

using cv::Rect;
using cv::Point;
using cv::RotatedRect;
using cv::Point2f;

struct vBlob
{
    vBlob()
    {
        area = 0;
        angle  = 0;
        length = 0;
        isHole = false;
    }

    vBlob(const vBlob& b):box(b.box),center(b.center),pts(b.pts),rotBox(b.rotBox)
    {
        area = b.area;
        angle = b.angle;
        isHole = b.isHole;
        length = b.length;
    }

    vBlob(Rect rc, Point ct, float _area = 0, float _angle = 0, bool hole = false)
    {
        box = rc;
        center = ct;
        area = _area;
        angle = _angle;
        isHole = hole;
        length = 0;
    }

    vBlob& operator = (const vBlob& b)
    {
        pts = b.pts;
        box = b.box;
        rotBox = b.rotBox;
        center = b.center;
        area = b.area;
        angle = b.angle;
        isHole = b.isHole;
        length = b.length;
        return *this;
    }

    Rect box;
    RotatedRect rotBox;
    float angle;

    Point2f center;
    std::vector<Point> pts;
    float area;
    float length;
    bool isHole;

    bool operator<(const vBlob& other) const 
    {//sorted by Y-coord first then X-coord
        return (center.y < other.center.y) || 
            ( (center.y == other.center.y) && (center.x < other.center.x)); 
    }

    bool similar(const vBlob& other, int k) const 
    {
        return (abs(box.width - other.box.width) < k) && (abs(box.height - other.box.height)<k);
    }

    void boxMerge(const vBlob& other)
    {
        int _x = cv::min<int>(other.box.x, box.x);
        int _y = cv::min<int>(other.box.y, box.y);
        box.width = cv::max<int>(other.box.x + other.box.width, box.x + box.width)- _x;
        box.height = cv::max<int>(other.box.y + other.box.height, box.y + box.height) - _y;

        box.x = _x;
        box.y = _y;

        center.x = box.x + box.width/2;
        center.y = box.y + box.height/2;		
    }
};

enum E_status
{
    statusStill,
    statusEnter,
    statusLeave,
    statusMove,	
};

struct vDefect
{
    vDefect( const Point& _start, const Point& _end, const Point& _depth, float _depthVal):
startPoint(_start), endPoint(_end), depthPoint(_depth), depth(_depthVal),
midPoint((startPoint.x+endPoint.x)/2, (startPoint.y+endPoint.y)/2){}

Point startPoint, endPoint, depthPoint;
Point midPoint;
float depth;

void draw(cv::Mat& img, const cv::Scalar& clr)
{
    cv::line(img, depthPoint, midPoint, clr);
    //  cv::line(img, depthPoint, endPoint, clr);
    cv::circle(img, midPoint,3, clr);
}
};

struct vTrackedBlob : public vBlob 
{
    enum
    {
        BLOB_NEW_ID = -3,
        BLOB_TO_DELETE = -2,		
    };

    E_status status;
    int id;
    Point2f velocity;

    // Used only by BlobTracker
    //
    bool markedForDeletion;
    int framesLeft;

    vTrackedBlob():vBlob() {
        id = BLOB_NEW_ID;
        status = statusStill;
        markedForDeletion = false;
        framesLeft = 0;
    }

    vTrackedBlob( const vBlob& b ):vBlob(b) {
        id = BLOB_NEW_ID;
        status = statusStill;
        markedForDeletion = false;
        framesLeft = 0;
    }

    std::string getStatusString()
    {
        if (status == statusStill)
            return "still";
        else 	if (status == statusEnter)
            return "enter";
        else if (status == statusLeave)
            return "leave";
        else if (status == statusMove)
            return "move";
        else return "";
    }

    bool isDead() const
    {
        return id == BLOB_TO_DELETE;
    }
};
