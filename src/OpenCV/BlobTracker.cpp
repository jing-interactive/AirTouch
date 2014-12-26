#include "BlobTracker.h"
#include "point2d.h"
#include <list>
#include <functional>

using std::vector;

#define OPENCV_VERSION CVAUX_STR(CV_MAJOR_VERSION)""CVAUX_STR(CV_MINOR_VERSION)""CVAUX_STR(CV_SUBMINOR_VERSION)

// TODO: remove opencv_legacy
#if defined _DEBUG
#pragma comment(lib,"opencv_video"OPENCV_VERSION"d.lib")
#pragma comment(lib,"opencv_objdetect"OPENCV_VERSION"d.lib")
#pragma comment(lib,"opencv_features2d"OPENCV_VERSION"d.lib")
#pragma comment(lib,"opencv_flann"OPENCV_VERSION"d.lib")
#else
#pragma comment(lib,"opencv_video"OPENCV_VERSION".lib")
#pragma comment(lib,"opencv_objdetect"OPENCV_VERSION".lib")
#pragma comment(lib,"opencv_features2d"OPENCV_VERSION".lib")
#pragma comment(lib,"opencv_flann"OPENCV_VERSION".lib")
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
    vector<Vec4i> hierarchy;
    vector<vector<Point>> contours0;

    findContours(img, contours0, hierarchy, RETR_EXTERNAL/*RETR_TREE*/, CHAIN_APPROX_SIMPLE);

    for (size_t k = 0; k < contours0.size(); k++)
    {
        vector<Point> approx;
        bool isHole = false;
        const vector<Point>& contour = contours0[k];

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

void vFindBlobs(Mat& src, vector<vBlob>& blobs, vector<vector<vDefect>>& defects, int minArea/*=1*/, int maxArea/*=3072000*/)
{
#if 0
	static MemStorage contour_mem(cvCreateMemStorage(100));
	static MemStorage hull_mem(cvCreateMemStorage(100));

	CvMoments myMoments;

	cvClearMemStorage(contour_mem);
	cvClearMemStorage(hull_mem);

	blobs.clear();
	defects.clear();

	CvSeq* contour_list = 0;
	cvFindContours(src,contour_mem,&contour_list, sizeof(CvContour), CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE);

	for (CvSeq* d = contour_list; d != NULL; d=d->h_next)
	{
		bool isHole = false;
		CvSeq* c = d;
		while (c != NULL)
		{
			double area = fabs(cvContourArea( c ));
			if( area > minArea && area < maxArea)
			{
				int length = cvArcLength(c);
				cvMoments( c, &myMoments );

				blobs.push_back(vBlob());

				vBlob& obj = blobs.back();
				//fill the blob structure
				obj.area	= area;
				obj.length =  length;
				obj.isHole	= isHole;
				obj.box	= cvBoundingRect(c);
				obj.rotBox = cvMinAreaRect2(c);
				obj.angle = (90-obj.rotBox.angle)*GRAD_PI2;//in radians

				if (myMoments.m10 > -DBL_EPSILON && myMoments.m10 < DBL_EPSILON)
				{
					obj.center.x = obj.box.x + obj.box.width/2;
					obj.center.y = obj.box.y + obj.box.height/2;
				}
				else
				{
					obj.center.x = myMoments.m10 / myMoments.m00;
					obj.center.y = myMoments.m01 / myMoments.m00;
				}

				// get the points for the blob
				Point           pt;
				CvSeqReader       reader;
				cvStartReadSeq( c, &reader, 0 );

				for (int k=0;k<c->total;k++)
				{
					CV_READ_SEQ_ELEM( pt, reader );
					obj.pts.push_back(pt);
				}

				// defect detection
				CvSeq* seqHull = cvConvexHull2( c, hull_mem, CV_COUNTER_CLOCKWISE, 0 );
				CvSeq* defectsSeq = cvConvexityDefects( c, seqHull, NULL );

				CvConvexityDefect defect;
				cvStartReadSeq( defectsSeq, &reader, 0 );
				int numDefects = defectsSeq->total;

				vector<vDefect> one_defect;
				for(int i=0; i<numDefects; ++i){
					CV_READ_SEQ_ELEM( defect, reader );
					Point& startPt = *(defect.start);
					Point& endPt = *(defect.end);
					Point& depthPt = *(defect.depth_point);
					one_defect.push_back(vDefect( startPt, endPt, depthPt, defect.depth));
				}
				defects.push_back(one_defect);
			}//END if( area >= minArea)

			if (isHole)
				c = c->h_next;//one_hole->h_next is another_hole
			else
				c = c->v_next;//one_contour->h_next is one_hole
			isHole = true;
		}//END while (c != NULL)
	}
#endif
}

HaarFinder::HaarFinder()
{ 
    scale = 1.2;
}

bool HaarFinder::init(char* cascade_name)
{
    return mClassifier.load(cascade_name);
}

void HaarFinder::find(const Mat& img, int minArea, bool findAllFaces)
{
    blobs.clear();

    Mat gray(img.rows, img.cols, CV_8UC1);
    vGrayScale(img, gray);

    Mat tiny;
    resize( gray, tiny, Size( cvRound (img.cols/scale), cvRound (img.rows/scale)));
    equalizeHist( tiny, tiny );

    vector<Rect> faces;

    mClassifier.detectMultiScale(tiny, faces,
        1.1, 2, 0
        | findAllFaces ? CASCADE_FIND_BIGGEST_OBJECT|CASCADE_DO_CANNY_PRUNING : CASCADE_DO_CANNY_PRUNING
        //|CASCADE_FIND_BIGGEST_OBJECT
        //|CASCADE_DO_ROUGH_SEARCH
        |CASCADE_SCALE_IMAGE
        ,
        Size(30, 30) );

    int n_faces = faces.size();

    for(int i = 0; i < n_faces; i++ )
    {
        Rect& _r = faces[i];
        Rect r = Rect(_r.x*scale, _r.y*scale, _r.width*scale,_r.height*scale);

        float area          = r.width * r.height;
        if (area < minArea)
            continue;

        blobs.push_back( vBlob() );

        float length        = (r.width * 2)+(r.height * 2);
        float centerx       = (r.x) + (r.width / 2.0);
        float centery       = (r.y) + (r.height / 2.0);
        blobs[i].area		= fabs(area);
        blobs[i].isHole		= false;
        blobs[i].length		= length;
        blobs[i].box			= r;
        blobs[i].center.x		= (int) centerx;
        blobs[i].center.y		= (int) centery;
    }

    if (findAllFaces)
        std::sort(blobs.begin(), blobs.end(), cmp_blob_area);
}

vBlobTracker::vBlobTracker()
{
    IDCounter = 0;
}

/************************************
* Delegate to Callbacks
*************************************/

void vBlobTracker::doBlobOn( vTrackedBlob& b ) {
    b.status = statusEnter;
    //printf("blob: %d enter+\n" , b.id);
}

void vBlobTracker::doBlobMoved( vTrackedBlob& b ) {
    b.status = statusMove;
    //	printf("blob: %d move\n" , b.id);
}

void vBlobTracker::doBlobOff( vTrackedBlob& b ) {
    b.status = statusLeave;
    deadBlobs.push_back(b);
    //printf("blob: %d leave-\n" , b.id);
    b.id = vTrackedBlob::BLOB_TO_DELETE;
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
        vector<DMatch> matches;
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

            doBlobMoved(trackedBlobs[i]);
        }
        else
        {
            //leaving blobs
            doBlobOff(trackedBlobs[i]);
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

            //SEND BLOB ON EVENT
            doBlobOn( trackedBlobs.back());
        }
    }
}

void vBackGrayDiff::init(Mat initial, void* param/* = NULL*/)
{
    Size size(initial.cols, initial.rows);

    frame.create(size, CV_8UC1);
    bg.create(size, CV_8UC1);
    fore.create(size, CV_8UC1);

    if (initial.channels() == 1)
        initial.copyTo(bg);
    else
        vGrayScale(initial, bg);
}

void vBackGrayDiff::update(Mat image, int mode/* = 0*/)
{
    if (image.channels() == 1)
        image.copyTo(frame);
    else
        vGrayScale(image, frame);
    if (mode == DETECT_BOTH)
    {
        fore = Scalar(0,0,0);
        for (int y=0;y<image.rows;y++)
            for (int x=0;x<image.cols;x++)
            {
                int delta = frame.at<uchar>(y, x) - bg.at<uchar>(y, x);
                if (delta >= threshes[0] || delta <= -threshes[1])
                    fore.at<uchar>(y, x) = 255;
            }
    }
    else if (mode == DETECT_DARK)
    {
        fore = bg - frame;
        vThresh(fore, threshes[1]);
    }
    else if (mode == DETECT_BRIGHT)
    {
        fore = frame - bg;
        vThresh(fore, threshes[0]);
    }
}


void vBackColorDiff::init(Mat initial, void* param/* = NULL*/)
{
    nChannels = initial.channels();

    frame = initial.clone();
    bg = initial.clone();
    fore.create(initial.rows, initial.cols, CV_8UC1);

    threshes[0] = 220;
    threshes[1] = 30;
}

void vBackColorDiff::update(Mat image, int mode/* = 0*/)
{
    //	vGrayScale(image, Frame);
    // 	cvCopy(image, frame);
    // 	if (mode == DETECT_BOTH)
    // 	{
    // 		if (nChannels == 1)
    // 		{
    // 			// BwImage frame(Frame);
    // 			// BwImage bg(Bg);
    // 			// BwImage fore(Fore);
    // 
    // 			fore = Scalar(0,0,0);
    // 			for (int y=0;y<image.rows;y++)
    // 				for (int x=0;x<image.cols;x++)
    // 				{
    // 					int delta = frame.at<uchar>(y, x) - bg.at<uchar>(y, x);
    // 					if (delta >= threshes[0] || delta <= -threshes[1])
    // 						fore.at<uchar>(y, x) = 255;
    // 				}
    // 		}
    // 		else
    // 		{
    // 			int min_t = 255-threshes[0];
    // 			int max_t = 255-threshes[1];
    // 			fore = Scalar(0,0,0);
    // 			for (int y=0;y<image.rows;y++)
    // 				for (int x=0;x<image.cols;x++)
    // 				{
    // 					int r = frame.at<uchar>(y, x).r - bg.at<uchar>(y, x).r;
    // 					int g = frame.at<uchar>(y, x).g - bg.at<uchar>(y, x).g;
    // 					int b = frame.at<uchar>(y, x).b - bg.at<uchar>(y, x).b;
    // #if 1
    // 					if ((r >= threshes[0] || r <= -threshes[1])
    // 						&& (g >= threshes[0] || g <= -threshes[1])
    // 						&& (b >= threshes[0] || b <= -threshes[1]))
    // #else
    // 					int delta = r*r+g*g+b*b;
    // 					if (delta >= min_t*min_t && delta <= max_t*max_t)
    // #endif
    // 						fore.at<uchar>(y, x) = 255;
    // 				}
    // 		}
    // 	}
    // 	else if (mode == DETECT_DARK)
    // 	{
    // 		fore = bg - frame;
    // 		vThresh(fore, threshes[1]);
    // 	}
    // 	else if (mode == DETECT_BRIGHT)
    // 	{
    // 		fore = frame - bg;
    // 		vThresh(fore, threshes[0]);
    // 	}
}

void vThreeFrameDiff::init(Mat initial, void* param/* = NULL*/)
{
    for (int i=0;i<3;i++)
    {	
        grays[i].create(initial.rows, initial.cols, CV_8UC1);
        vGrayScale(initial, grays[i]);
    }
}

void vThreeFrameDiff::update(Mat image, int mode/* = 0*/)
{
    vGrayScale(image, grays[2]);

    // BwImage one(gray1);
    // BwImage two(gray2);
    // BwImage three(gray3);
    // BwImage diff(grayDiff);

    grayDiff = Scalar(0,0,0);

    for (int y=0;y<image.rows;y++)
    {
        for (int x=0;x<image.cols;x++)
        {
            if (abs(grays[0].at<uchar>(y, x) - grays[1].at<uchar>(y, x)) > threshes[0] ||
                abs(grays[2].at<uchar>(y, x) - grays[1].at<uchar>(y, x)) > threshes[0])
                grayDiff.at<uchar>(y, x) = 255;
        }
    }

    // 	show_mat<uchar>(gray1);
    // 	show_mat<uchar>(gray2);
    // 	show_mat<uchar>(gray3);

    grays[1].copyTo(grays[0]);
    grays[2].copyTo(grays[1]);

    //if (mode == DETECT_BOTH)
    //	cvAbsDiff(grayFrame, grayBg, grayDiff);
    //else if (mode == DETECT_DARK)
    //	cvSub(grayBg, grayFrame, grayDiff);
    //else if (mode == DETECT_BRIGHT)
    //	cvSub(grayFrame, grayBg, grayDiff);
    //vThresh(grayDiff, thresh);
}

Mat vThreeFrameDiff::getForeground()
{
    return grayDiff;
}

Mat vThreeFrameDiff::getBackground()
{
    return grayDiff;
}

IStaticBackground::IStaticBackground()
{
    threshes[0] = 50;
    threshes[1] = 200;
}

void IStaticBackground::setIntParam( int idx, int value )
{
    CV_Assert(idx >=0 && idx <= 1);
    threshes[idx] =	255-value;
}

Mat IStaticBackground::getForeground()
{
    return fore;
}

Mat IStaticBackground::getBackground()
{
    return bg;
}

IStaticBackground::~IStaticBackground()
{

}

// REFACTOR
void vBackGrayAuto::init( Mat initial, void* param /*= NULL*/ )
{
    Size size(initial.cols, initial.rows);

    frame.create(size, CV_8UC1);
    bg.create(size, CV_8UC1);
    fore.create(size, CV_8UC1);

    if (initial.channels() == 1)
        initial.copyTo(bg);
    else
        vGrayScale(initial, bg);
    mBackF32.create(initial.size(), CV_32FC1);
    bg.convertTo(mBackF32, CV_32FC1);
}

void vBackGrayAuto::update( Mat image, int mode /*= DETECT_BOTH*/ )
{
    const float kLearning = 0.003f;
    vGrayScale(image, frame);
    accumulateWeighted(frame, mBackF32, kLearning);
    mBackF32.convertTo(bg, CV_8UC1);

    if (image.channels() == 1)
        image.copyTo(frame);
    else
        vGrayScale(image, frame);
    if (mode == DETECT_BOTH)
    {
        fore = Scalar(0,0,0);
        for (int y=0;y<image.rows;y++)
            for (int x=0;x<image.cols;x++)
            {
                int delta = frame.at<uchar>(y, x) - bg.at<uchar>(y, x);
                if (delta >= threshes[0] || delta <= -threshes[1])
                    fore.at<uchar>(y, x) = 255;
            }
    }
    else if (mode == DETECT_DARK)
    {
        fore = bg - frame;
        vThresh(fore, threshes[1]);
    }
    else if (mode == DETECT_BRIGHT)
    {
        fore = frame - bg;
        vThresh(fore, threshes[0]);
    }
}