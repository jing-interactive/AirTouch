/*
* BlobTracker.h
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

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"

#include "point2d.h"

using cv::Rect;
using cv::Point;
using cv::RotatedRect;
using cv::Point2f;

struct Blob
{
    Blob()
    {
        area = 0;
        angle = 0;
        length = 0;
        isHole = false;
    }

    Blob(const Blob &b) : box(b.box), center(b.center), pts(b.pts), rotBox(b.rotBox)
    {
        area = b.area;
        angle = b.angle;
        isHole = b.isHole;
        length = b.length;
    }

    Blob(Rect rc, Point ct, float _area = 0, float _angle = 0, bool hole = false)
    {
        box = rc;
        center = ct;
        area = _area;
        angle = _angle;
        isHole = hole;
        length = 0;
    }

    Blob &operator = (const Blob &b)
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

    bool operator<(const Blob &other) const
    {
        //sorted by Y-coord first then X-coord
        return (center.y < other.center.y) ||
            ((center.y == other.center.y) && (center.x < other.center.x));
    }
};

struct TrackedBlob : public Blob
{
    enum
    {
        BLOB_NEW_ID = -3,
        BLOB_TO_DELETE = -2,
    };

    int id;
    Point2f velocity;

    // Used only by BlobTracker
    //
    bool markedForDeletion;
    int framesLeft;

    TrackedBlob() : Blob()
    {
        id = BLOB_NEW_ID;
        markedForDeletion = false;
        framesLeft = 0;
    }

    TrackedBlob(const Blob &b) : Blob(b)
    {
        id = BLOB_NEW_ID;
        markedForDeletion = false;
        framesLeft = 0;
    }

    bool isDead() const
    {
        return id == BLOB_TO_DELETE;
    }
};

struct BlobFinder
{
    struct Option
    {
        Option();
        int minArea;
        int maxArea;
        bool convexHull;
        bool(*sort_func)(const Blob &a, const Blob &b);
        bool handOnlyMode;
        int handDistance;
    };
    static void execute(cv::Mat &src, std::vector<Blob> &blobs, const Option& option);
};

class BlobTracker
{
public:
    BlobTracker();
    void trackBlobs(const std::vector<Blob> &newBlobs);

    std::vector<TrackedBlob>   trackedBlobs; //tracked blobs
    std::vector<TrackedBlob>  deadBlobs;

private:
    unsigned int                        IDCounter;    //counter of last blob
};
