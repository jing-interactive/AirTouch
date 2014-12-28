#include "BlobTracker.h"
#include "point2d.h"
#include <list>
#include <functional>

using std::vector;

#define OPENCV_VERSION CVAUX_STR(CV_MAJOR_VERSION)""CVAUX_STR(CV_MINOR_VERSION)""CVAUX_STR(CV_SUBMINOR_VERSION)

#if defined _DEBUG
#pragma comment(lib,"opencv_core"OPENCV_VERSION"d.lib")
#pragma comment(lib,"opencv_imgproc"OPENCV_VERSION"d.lib")
#pragma comment(lib,"opencv_features2d"OPENCV_VERSION"d.lib")
#else
#pragma comment(lib,"opencv_core"OPENCV_VERSION".lib")
#pragma comment(lib,"opencv_imgproc"OPENCV_VERSION".lib")
#pragma comment(lib,"opencv_features2d"OPENCV_VERSION".lib")
#endif

using namespace cv;

bool cmp_blob_area(const vBlob& a, const vBlob& b)
{
    return a.area > b.area;
}

#define CVCONTOUR_APPROX_LEVEL  1   // Approx.threshold - the bigger it is, the simpler is the boundary

void vFindBlobs(Mat& img, vector<vBlob>& blobs, int minArea, int maxArea, bool convexHullMode, bool (*sort_func)(const vBlob& a, const vBlob& b))
{
    blobs.clear();
    static vector<Vec4i> hierarchy;
    static vector<vector<Point>> contours0;
	static vector<Point> approx;

    findContours(img, contours0, hierarchy, RETR_EXTERNAL/*RETR_TREE*/, CHAIN_APPROX_SIMPLE);

	for (const auto& contour : contours0)
    {
        bool isHole = false;

        double area = fabs(contourArea(contour));
        if (area >= minArea && area <= maxArea)
        {
            int length = arcLength(contour, true);
            if (convexHullMode) //Convex Hull of the segmentation
                convexHull(contour, approx);
            else //Polygonal approximation of the segmentation
                approxPolyDP(contour, approx, std::min<double>(length*0.003,2.0), true);

            area = contourArea(approx); //update area
            Moments mom = moments(approx);

            blobs.push_back(vBlob());

            vBlob& obj = blobs[blobs.size()-1];
            //fill the blob structure
            obj.area	= fabs(area);
            obj.length =  length;
            obj.isHole	= isHole;
            obj.box	= boundingRect(approx);
            obj.rotBox = minAreaRect(approx);
            obj.angle = (90-obj.rotBox.angle)*GRAD_PI2;//in radians

            if (mom.m10 > -DBL_EPSILON && mom.m10 < DBL_EPSILON)
            {
                obj.center.x = obj.box.x + obj.box.width/2;
                obj.center.y = obj.box.y + obj.box.height/2;
            }
            else
            {
                obj.center.x = mom.m10 / mom.m00;
                obj.center.y = mom.m01 / mom.m00;
            }

            obj.pts = approx;
        }
        isHole = true;
    }

    if (!sort_func) sort_func = &cmp_blob_area;
    std::sort(blobs.begin(), blobs.end(), sort_func);
}

vBlobTracker::vBlobTracker()
{
    IDCounter = 0;
}

void vBlobTracker::trackBlobs( const vector<vBlob>& newBlobs )
{
    deadBlobs.clear();
    const int n_old = trackedBlobs.size();
    const int n_new = newBlobs.size();
    vector<vTrackedBlob> newTrackedBlobs(n_new);
    std::copy(newBlobs.begin(), newBlobs.end(), newTrackedBlobs.begin());

    vector<int> nn_of_a(n_old);//nearest neighbor of pta in ptb
    vector<int> dist_of_a(n_old);//nearest neighbor of pta in ptb
    fill(nn_of_a.begin(), nn_of_a.end(),-1);
    fill(dist_of_a.begin(), dist_of_a.end(),INT_MAX);

    if (n_old != 0 && n_new != 0)
    {
        Mat1f ma(trackedBlobs.size(),2);
        Mat1f mb(newBlobs.size(),2);
        for (int i=0;i<n_old;i++)
        {
            ma(i,0) = trackedBlobs[i].center.x;
            ma(i,1) = trackedBlobs[i].center.y;
        }
        for (int i=0;i<n_new;i++)
        {
            mb(i,0) = newTrackedBlobs[i].center.x;
            mb(i,1) = newTrackedBlobs[i].center.y;
        }

        BFMatcher matcher(NORM_L2);
        static vector<DMatch> matches;
        matcher.match(mb, ma, matches);
        const int n_matches = matches.size();
        for (int i=0;i<n_matches;i++)
        {
            const DMatch& match = matches[i];
            int t_id = match.trainIdx;
            int q_id = match.queryIdx;
            float dist = match.distance;

            //TODO: 200 -> param
            if (dist < 200 && dist < dist_of_a[t_id])
            {
                dist_of_a[t_id] = dist;
                nn_of_a[t_id] = q_id;
            }
        }
    }

    for (int i=0;i<n_old;i++)
    {
        int nn = nn_of_a[i];
        if (nn != -1)
        {
            //moving blobs
            Point2f lastCenter = trackedBlobs[i].center;
            newTrackedBlobs[nn].id = trackedBlobs[i].id;//save id, cause we will overwrite the data
            trackedBlobs[i] = newTrackedBlobs[nn];//update with new data

            // TODO: ....
            trackedBlobs[i].velocity.x = trackedBlobs[i].center.x - lastCenter.x;
            trackedBlobs[i].velocity.y = trackedBlobs[i].center.y - lastCenter.y;
            float posDelta = sqrtf((trackedBlobs[i].velocity.x*trackedBlobs[i].velocity.x)+
                (trackedBlobs[i].velocity.y*trackedBlobs[i].velocity.y));

            // AlexP
            // now, filter the blob position based on MOVEMENT_FILTERING value
            // the MOVEMENT_FILTERING ranges [0,15] so we will have that many filtering steps
            // Here we have a weighted low-pass filter
            // adaptively adjust the blob position filtering strength based on blob movement
            // http://www.wolframalpha.com/input/?i=plot+1/exp(x/15)+and+1/exp(x/10)+and+1/exp(x/5)+from+0+to+100
#define MOVEMENT_FILTERING 2
            float a = 1.0f - 1.0f / expf(posDelta / (1.0f + (float)MOVEMENT_FILTERING*10));
            trackedBlobs[i].center.x = a * trackedBlobs[i].center.x + (1-a) * lastCenter.x;
            trackedBlobs[i].center.y = a * trackedBlobs[i].center.y + (1-a) * lastCenter.y;
        }
        else
        {
			trackedBlobs[i].status = statusLeave;
			deadBlobs.push_back(trackedBlobs[i]);
			trackedBlobs[i].id = vTrackedBlob::BLOB_TO_DELETE;
        }
    }
    trackedBlobs.erase(remove_if(trackedBlobs.begin(), trackedBlobs.end(), std::mem_fun_ref(&vTrackedBlob::isDead)),
        trackedBlobs.end());
    //entering blobs
    for(int i=0; i<n_new; i++)
    {
        if (newTrackedBlobs[i].id == vTrackedBlob::BLOB_NEW_ID)
        {
            //add new track
            if (IDCounter > UINT_MAX)
                IDCounter = 0;
            newTrackedBlobs[i].id=IDCounter++;
            trackedBlobs.push_back(newTrackedBlobs[i]);
        }
    }
}
