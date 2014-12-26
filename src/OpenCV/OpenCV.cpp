#include "OpenCV.h"

#ifdef _DEBUG
#pragma comment(lib,"opencv_core"OPENCV_VERSION"d.lib")
#pragma comment(lib,"opencv_imgproc"OPENCV_VERSION"d.lib")
#pragma comment(lib,"opencv_highgui"OPENCV_VERSION"d.lib")
#else	//_DEBUG
#pragma comment(lib,"opencv_core"OPENCV_VERSION".lib")
#pragma comment(lib,"opencv_imgproc"OPENCV_VERSION".lib")
#pragma comment(lib,"opencv_highgui"OPENCV_VERSION".lib")
#endif	//_DEBUG

using namespace std;
using namespace cv;

//void vRotateImage(IplImage* image, float angle, float centreX, float centreY){
//   
//   CvPoint2D32f centre;
//   CvMat *translate = cvCreateMat(2, 3, CV_32FC1);
//   cvSetZero(translate);
//   centre.x = centreX;
//   centre.y = centreY;
//   cv2DRotationMatrix(centre, angle, 1.0, translate);
//   cvWarpAffine(image, image, translate, CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, ScalarAll(0));
//   cvReleaseMat(&translate);
//}

/*
flip_param -> flip_mode
NO_FLIP
0	:	horizontal
1	:	vertical
-1	:	both
*/
void vFlip(Mat& src, int flipHorizontal, int flipVertical)
{
#define NO_FLIP 1000
    int flipCode = NO_FLIP;

    if (flipVertical == 1)
    {
        if (flipHorizontal == 1)
            flipCode = 0;
        else
            flipCode = -1;
    }
    else if (flipHorizontal == 1)
    {
        flipCode = 1;    
    }

    if (flipCode != NO_FLIP)
        flip(src, src, flipCode);
}

void vFastCopyImageTo( const Mat& src, Mat& dst, const Rect& roi )
{
    CV_Assert(src.size() == roi.size());
    Mat sub = dst(roi);

    if (src.channels() == 1 && dst.channels() == 3)
    {
        Mat src_clr(src.rows, src.cols, CV_8UC3);
        vColorFul(src, src_clr);
        src_clr.copyTo(sub);
    }
    else
    {
        src.copyTo(sub);
    }
}

void vCopyImageTo(const Mat& src, Mat& dst, const Rect& roi)
{
    Mat sub = dst(roi);

    if (src.channels() == 1 && dst.channels() == 3)
    {
        Mat src_clr(src.rows, src.cols, CV_8UC3);
        vColorFul(src, src_clr);
        resize(src_clr, sub, sub.size());
    }
    else
    {
        resize(src, sub, sub.size());
    }
}

void vDrawText(Mat& img, int x, int y, char* str, Scalar clr)
{
    putText(img, str, Point(x,y), FONT_HERSHEY_SIMPLEX, 0.5, clr);
}

Scalar default_colors[] =
{
    Scalar(169, 176, 155),
    Scalar(169, 176, 155),
    Scalar(168, 230, 29),
    Scalar(200, 0,   0),
    Scalar(79,  84,  33), 
    Scalar(84,  33,  42),
    Scalar(255, 126, 0),
    Scalar(215,  86, 0),
    Scalar(33,  79,  84), 
    Scalar(33,  33,  84),
    Scalar(77,  109, 243), 
    Scalar(37,   69, 243),
    Scalar(77,  109, 243),
    Scalar(69,  33,  84),
    Scalar(229, 170, 122), 
    Scalar(255, 126, 0),
    Scalar(181, 165, 213), 
    Scalar(71, 222,  76),
    Scalar(245, 228, 156), 
    Scalar(77,  109, 243),
};

const int sizeOfColors = sizeof(default_colors)/sizeof(Scalar);
Scalar vDefaultColor(int idx){ return default_colors[idx%sizeOfColors];}

VideoInput::VideoInput()
{
    mInputType = From_Count;
}

bool VideoInput::initFromCamera(int cameraIndex, int captureId)
{
    char buffer[256];
    bool isOpened = false;
    do 
    {
        mCaptures[captureId].open(cameraIndex); 
        if (mCaptures[captureId].isOpened())
        {
#ifdef KINSERVER_OCV_WAR
            mCaptures[captureId].set(cv::CAP_PROP_FPS, 60);
            // TODO:
            mCaptures[captureId].set(cv::CAP_PROP_FRAME_WIDTH, 1024);
            mCaptures[captureId].set(cv::CAP_PROP_FRAME_HEIGHT, 768);
#endif
            mInputType = From_Camera;
            sprintf(buffer, "Reading from camera # %d via DirectShow.", cameraIndex);
            isOpened = true;
            break;
        }
        else
        {
            mCaptures[captureId].open(cameraIndex); 

            if (mCaptures[captureId].isOpened())
            {		
                mInputType = From_Camera;
                sprintf(buffer, "Reading from camera # %d.", cameraIndex);
                isOpened = true;
                break;
            }
            else
            {
                sprintf(buffer, "Failed to open camera # %d", cameraIndex);
                break;
            }
        }
    } while (0);

    if (isOpened)
    {
        postInit(captureId);
    }
    printf("\n%s\n", buffer);

    return isOpened;
}

bool VideoInput::initFromFile(const string& filename, int captureId/* = 0*/)
{
    mFrame = imread(filename);

    if (!mFrame.empty())
    {
        printf("Reading from image %s.\n", filename.c_str());
        mInputType = From_Image;
    }
    else
    {
        mCaptures[captureId].open(filename);
        if(mCaptures[captureId].isOpened())
        {
            printf("Reading from video %s.\n", filename.c_str());
            mInputType = From_Video;
        }
        else
        {
            printf("Could not open file %s.\n", filename.c_str());
            return false;
        }
    }

    postInit();
    return true;
}

bool VideoInput::init(int argc, char** argv)
{
    char* arg1 = argv[1];
    if (argc == 1 || (argc == 2 && strlen(arg1) == 1 && ::isdigit(arg1[0])))
    {
        mCaptures.resize(1);
        return initFromCamera(argc == 2 ? arg1[0] - '0' : 0);
    }

    if (argc == 2)
    {
        if (strlen(arg1) == 3 && arg1[1] == '+')
        {
            mCaptures.resize(2);
            return initFromCamera(arg1[0] - '0', 0) && initFromCamera(arg1[2] - '0', 1);
        }

        mCaptures.resize(1);
        return initFromFile(arg1);
    }
    return false;
}

void VideoInput::skipFrames(int framesToSkip)
{
    if (mInputType == From_Image)
        return;
    for (int i=0;i<framesToSkip;i++)
        getFrame();
}

Mat VideoInput::getFrame()
{
    mFrameNum ++;

    size_t nCaptures = mCaptures.size();
    if (mFrame.empty())
        mFrame.create(mSize.height, nCaptures * mSize.width, CV_8UC3);

    for (size_t i=0; i<nCaptures; i++)
    {
        if (mCaptures[i].isOpened())
        {
            Mat temp;
            do 
            {
                mCaptures[i] >> temp;
            } while (temp.empty());
            temp.copyTo(mFrame(Rect(i * mSize.width, 0, mSize.width, mSize.height)));
        }
    }

    return mFrame;
}

void VideoInput::postInit(int captureId/* = 0*/)
{
    Mat frame;
    do 
    {
        // TODO: fix image
        mCaptures[captureId] >> frame;
    } while (frame.cols == 0 || frame.rows == 0);

    mSize.width = frame.cols;
    mSize.height = frame.rows;
    mChannel = frame.channels();
    mFrameNum = 0;

    printf("Size: <%d,%d>\n",  mSize.width, mSize.height);
}

void vHighPass(const Mat& src, Mat& dst, int blurLevel/* = 10*/, int noiseLevel/* = 3*/)
{
    if (blurLevel > 0 && noiseLevel > 0)
    {
        // create the unsharp mask using a linear average filter
        blur(src, dst, Size(blurLevel*2+1, blurLevel*2+1));

        dst = src - dst;
        //		cvSub(src, dst, dst);

        // filter out the noise using a median filter
        medianBlur(dst, dst, noiseLevel*2+1);
    }
    else
        src.copyTo(dst);
}

void vPolyLine(Mat& dst, vector<Point>& pts, Scalar clr, int thick)
{
    int n = pts.size();
    if (n > 1)
    {
        int k =0;
        for (;k<n-1;k++)
        {
            line(dst, pts[k], pts[k+1], clr, thick);
        }
        line(dst, pts[k], pts[0], clr, thick);
    }
}

bool operator < (const Point& a, const Point& b)
{
    return a.x < b.x && a.y < b.y;
}

void vFillPoly(Mat& img, const vector<Point>& pt_list, const Scalar& clr/* = Scalar(255,255,255)*/)
{
    const Point* pts = &pt_list[0];
    const int npts = pt_list.size();
    fillPoly(img, &pts, &npts, 1, clr);
}

void vLinePoly(Mat& img, const vector<Point>& pt_list, const Scalar& clr/* = Scalar(255,255,255)*/, int thick/* = 1*/)
{
    const Point* pts = &pt_list[0];
    const int npts = pt_list.size();
    polylines(img, &pts, &npts, 1, true, clr, thick);
}

void vLinePoly(Mat& img, const vector<Point2f>& pt_list, const Scalar& clr/* = Scalar(255,255,255)*/, int thick/* = 1*/)
{
    const int npts = pt_list.size();
    Point* pts = new Point[npts];
    for (int i=0;i<npts;i++)
        pts[i] = pt_list[i];

    polylines(img, (const Point**)&pts, &npts, 1, true, clr, thick);

    delete[] pts;
}

bool vTestRectHitRect(const Rect& object1, const Rect& object2)
{
    int left1, left2;
    int right1, right2;
    int top1, top2;
    int bottom1, bottom2;

    left1 = object1.x;
    left2 = object2.x;
    right1 = object1.x + object1.width;
    right2 = object2.x + object2.width;
    top1 = object1.y;
    top2 = object2.y;
    bottom1 = object1.y + object1.height;
    bottom2 = object2.y + object2.height;

    if (bottom1 < top2) return false;
    if (top1 > bottom2) return false;

    if (right1 < left2) return false;
    if (left1 > right2) return false;

    return true;
};

Scalar vRandomColor()
{
    static RNG rng((unsigned)-1);
    int icolor = rng();
    return Scalar(icolor&255, (icolor>>8)&255, (icolor>>16)&255);
}

Scalar rgbColor(int r, int g, int b)
{
    return Scalar(b, g, r);
}
