#include "cinder/app/RendererGl.h"
#include "cinder/app/AppNative.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/params/Params.h"
#include "cinder/Log.h"
#include "cinder/PolyLine.h"

#include "OscSender.h"
#include "cinder/CinderOpenCV.h"
#include "OpenCV/BlobTracker.h"
#include "Cinder-KinectSDK/KinectDevice.h"

#include "VNM/MiniConfig.h"

using namespace ci;
using namespace ci::app;

template <typename T>
void updateTexture(gl::TextureRef &tex, const T &src)
{
    if (!tex)
    {
        tex = gl::Texture2d::create(src);
    }
    else
    {
        tex->update(src);
    }
}

class KinServerApp : public AppBasic
{
public:
    enum
    {
        FAKE_BLOB_ID = 9999
    };
    void setup() override
    {
        readConfig();
        log::manager()->enableFileLogging();
#ifdef KINECT_V2
        mDevice = Kinect::Device::createV2();
#else
        mDevice = Kinect::Device::createV1();
#endif // KINECT_V2
        mDevice->signalDepthDirty.connect(std::bind(&KinServerApp::updateDepthRelated, this));

        mDepthW = mDevice->getWidth();
        mDepthH = mDevice->getHeight();
        mRoi.set(0, 0, mDepthW, mDepthH);
        mDiffMat = cv::Mat1b(mDepthH, mDepthW);
        mDiffChannel = Channel(mDepthW, mDepthH, mDiffMat.step, 1,
            mDiffMat.ptr());

        mParams = params::InterfaceGl::create("params", vec2(300, getConfigUIHeight() + 100));
        setupConfigUI(mParams.get());
        std::vector<string> smoothNames = { "Off", "Light", "Middle", "High" };
        ADD_ENUM_TO_INT(mParams, TRACKING_SMOOTH, smoothNames)
            getWindow()->connectPostDraw(&params::InterfaceGl::draw, mParams.get());

        mFps = 0;
        mFrameCounter = 0;
        mSecondsForFps = getElapsedSeconds();

        mParams->addParam("FPS", &mFps, true);
        mParams->addButton("Set Bg", std::bind(&KinServerApp::updateBack, this));

        mOscSender.setup(ADDRESS, TUIO_PORT);

        getWindow()->setSize(1024, 768);

        mLogo = gl::Texture::create(loadImage(loadAsset("logo.png")));

        mShader = gl::GlslProg::create(loadAsset("depthMap.vs"), loadAsset("depthMap.fs"));
        mShader->uniform("image", 0);

        mToAddFakeId = false;
    }

    void resize() override
    {
        mLayout.width = getWindowWidth();
        mLayout.height = getWindowHeight();
        mLayout.halfW = mLayout.width / 2;
        mLayout.halfH = mLayout.height / 2;
        mLayout.spc = mLayout.width * 0.01;

        for (int x = 0; x < 2; x++)
        {
            for (int y = 0; y < 2; y++)
            {
                mLayout.canvases[y * 2 + x] = Rectf(
                    mLayout.spc + mLayout.halfW * x,
                    mLayout.spc + mLayout.halfH * y,
                    mLayout.halfW * (1 + x) - mLayout.spc,
                    mLayout.halfH * (1 + y) - mLayout.spc
                    );
            }
        }
        if (mLogo)
        {
            mLayout.logoRect = Rectf(
                mLayout.halfW + mLayout.spc,
                mLayout.spc,
                mLayout.width - mLayout.spc,
                mLayout.spc + (mLayout.halfW - mLayout.spc * 2) / mLogo->getAspectRatio()
                );
        }

        mParams->setPosition(mLayout.canvases[1].getUpperLeft());
    }

    void draw() override
    {
        gl::clear(ColorA::gray(0.5f));

        if (mLogo)
        {
            gl::enableAlphaBlending();
            gl::draw(mLogo, mLayout.logoRect);
            gl::disableAlphaBlending();
        }

        if (mDepthTexture)
        {
            //gl::ScopedGlslProg prog(mShader);
            gl::draw(mDepthTexture, mLayout.canvases[0]);
            gl::draw(mBackTexture, mLayout.canvases[3]);
            gl::draw(mDiffTexture, mLayout.canvases[2]);
        }
        visualizeBlobs(mBlobTracker);
    }

    void mouseUp(MouseEvent event)
    {
        float x = event.getX();
        float y = event.getY();
        const Rectf& rect = mLayout.canvases[2];
        x = (x - rect.x1) / rect.getWidth();
        y = (y - rect.y1) / rect.getHeight();
        if (x > 0 && x < 1 && y > 0 && y < 1)
        {
            mFakeBlobPos = vec2(x * mDepthW, y * mDepthH);
            mToAddFakeId = true;
        }
    }

    void keyUp(KeyEvent event) override
    {
        int code = event.getCode();
        if (code == KeyEvent::KEY_ESCAPE)
        {
            quit();
        }
    }

    // TODO: Async image processing
    void update() override
    {
        if (RECT_ROI_ENABLED)
        {
            mRoi.set(
                ROI_X1 * mDepthW,
                ROI_Y1 * mDepthH,
                ROI_X2 * mDepthW,
                ROI_Y2 * mDepthH
                );
        }
        else
        {
            mRoi.set(0, 0, mDepthW, mDepthH);
        }
    }

private:
    void updateDepthRelated()
    {
        mFrameCounter++;
        if (getElapsedSeconds() - mSecondsForFps > 1)
        {
            mSecondsForFps = getElapsedSeconds();
            mFps = mFrameCounter;
            mFrameCounter = 0;
        }

        updateTexture(mDepthTexture, mDevice->depthChannel);

        if (!mBackTexture)
        {
            updateBack();
        }

        mDiffMat.setTo(cv::Scalar::all(0));
        //float x0 = corners[CORNER_DEPTH_LT].x - depthOrigin.x;
        //float x1 = corners[CORNER_DEPTH_RB].x - depthOrigin.x;
        //float y0 = corners[CORNER_DEPTH_LT].y - depthOrigin.y;
        //float y1 = corners[CORNER_DEPTH_RB].y - depthOrigin.y;

        int cx = CENTER_X * mDepthW;
        int cy = CENTER_Y * mDepthH;
        int radius = RADIUS * mDepthH;
        int radius_sq = radius * radius;

        for (int yy = mRoi.y1; yy < mRoi.y2; yy++)
        {
            // TODO: cache row pointer
            int y = yy;
            for (int xx = mRoi.x1; xx < mRoi.x2; xx++)
            {
                int x = LEFT_RIGHT_FLIPPED ? (mDepthW - xx) : xx;
                uint16_t bg = *mBackChannel.getData(x, y);
                uint16_t dep = *mDevice->depthChannel.getData(x, y);
                if (dep > 0 && bg - dep > MIN_THRESHOLD_MM && bg - dep < MAX_THRESHOLD_MM)
                {
                    // TODO: optimize
                    if (!CIRCLE_MASK_ENABLED || (cx - x) * (cx - x) + (cy - y) * (cy - y) < radius_sq)
                    {
                        mDiffMat(yy, xx) = 255;
                    }
                }
            }
        }

        if (TRACKING_SMOOTH > 0)
        {
            cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(TRACKING_SMOOTH * 2 + 1, TRACKING_SMOOTH * 2 + 1),
                cv::Point(TRACKING_SMOOTH, TRACKING_SMOOTH));
            cv::morphologyEx(mDiffMat, mDiffMat, cv::MORPH_OPEN, element);
        }

        updateTexture(mDiffTexture, mDiffChannel);
        std::vector<Blob> blobs;
        BlobFinder::Option option;
        option.minArea = MIN_AREA;
        option.handOnlyMode = FINGER_MODE_ENABLED;
        option.handDistance = FINGER_SIZE;
        BlobFinder::execute(mDiffMat, blobs, option);
        mBlobTracker.trackBlobs(blobs);
        sendTuioMessage(mOscSender, mBlobTracker);
    }

    void visualizeBlobs(const BlobTracker &blobTracker)
    {
        static uint8_t sPalette[][3] =
        {
            { 255, 0, 0 },
            { 122, 0, 0 },
            { 255, 255, 0 },
            { 122, 122, 0 },
            { 255, 0, 255 },
            { 122, 0, 122 },
            { 0, 0, 255 },
            { 0, 0, 122 },
            { 0, 255, 255 },
            { 0, 122, 122 },
        };
        const size_t sPaletteCount = _countof(sPalette);

        vec2 scale;
        scale.x = (mLayout.halfW - mLayout.spc * 2) / mDepthW;
        scale.y = (mLayout.halfH - mLayout.spc * 2) / mDepthH;
        gl::pushModelMatrix();
        gl::translate(mLayout.canvases[2].getUpperLeft());
        gl::scale(scale);

        if (CIRCLE_MASK_ENABLED)
        {
            float cx = CENTER_X * mDepthW;
            float cy = CENTER_Y * mDepthH;
            float radius = RADIUS * mDepthH;
            gl::drawStrokedCircle(vec2(cx, cy), radius);
        }
        if (RECT_ROI_ENABLED)
        {
            gl::drawStrokedRect(mRoi);
        }

        char idName[10];
        for (const auto &blob : blobTracker.trackedBlobs)
        {
            int idx = blob.id % sPaletteCount;
            gl::color(Color8u(sPalette[idx][0], sPalette[idx][1], sPalette[idx][2]));
            PolyLine2 line;
            for (const auto &pt : blob.pts)
            {
                line.push_back(vec2(pt.x, pt.y));
            }
            line.setClosed();
            gl::drawSolid(line);
            sprintf(idName, "#%d", blob.id);
            gl::drawStringCentered(idName, vec2(blob.center.x, blob.center.y));
        }
        gl::color(Color::white());
        gl::popModelMatrix();
    }

    void sendTuioMessage(osc::Sender &sender, const BlobTracker &blobTracker)
    {
        osc::Bundle bundle;

        osc::Message alive;
        {
            alive.setAddress("/tuio/2Dcur");
            alive.addStringArg("alive");
        }

        osc::Message fseq;
        {
            fseq.setAddress("/tuio/2Dcur");
            fseq.addStringArg("fseq");
            fseq.addIntArg(getElapsedFrames());
        }

        for (const auto &blob : blobTracker.trackedBlobs)
        {
            //Point2f center(blob.center.x + depthOrigin.x, blob.center.y + depthOrigin.y);
            vec2 center(blob.center.x, blob.center.y);

            if (!mRoi.contains(center)) continue;

            osc::Message set;
            set.setAddress("/tuio/2Dcur");
            set.addStringArg("set");
            set.addIntArg(blob.id);             // id
            set.addFloatArg((center.x - mRoi.x1) / mRoi.getWidth());
            set.addFloatArg((center.y - mRoi.y1) / mRoi.getHeight());
            set.addFloatArg(blob.velocity.x / mRoi.getWidth());
            set.addFloatArg(blob.velocity.y / mRoi.getHeight());
            set.addFloatArg(0);     // m
            bundle.addMessage(set);                         // add message to bundle

            alive.addIntArg(blob.id);               // add blob to list of ALL active IDs
        }

        if (mToAddFakeId)
        {
            osc::Message set;
            set.setAddress("/tuio/2Dcur");
            set.addStringArg("set");
            set.addIntArg(FAKE_BLOB_ID);             // id
            set.addFloatArg((mFakeBlobPos.x - mRoi.x1) / mRoi.getWidth());
            set.addFloatArg((mFakeBlobPos.y - mRoi.y1) / mRoi.getHeight());
            set.addFloatArg(0);
            set.addFloatArg(0);
            set.addFloatArg(0);     // m
            bundle.addMessage(set);                         // add message to bundle

            alive.addIntArg(FAKE_BLOB_ID);               // add blob to list of ALL active IDs
            mToAddFakeId = false;
        }

        bundle.addMessage(alive);    //add message to bundle
        bundle.addMessage(fseq);     //add message to bundle

        sender.sendBundle(bundle); //send bundle
    }

    void updateBack()
    {
        mBackChannel = mDevice->depthChannel.clone();
        updateTexture(mBackTexture, mBackChannel);
    }

    float mFps;
    float mSecondsForFps;
    int mFrameCounter;

    struct Layout
    {
        float width;
        float height;
        float halfW;
        float halfH;
        float spc;

        Rectf canvases[4];
        Rectf logoRect;
    } mLayout;

    Kinect::DeviceRef mDevice;
    params::InterfaceGlRef mParams;
    osc::Sender mOscSender;
    int mDepthW, mDepthH;

    gl::TextureRef mDepthTexture;

    // vision
    Channel16u mBackChannel;
    gl::TextureRef mBackTexture;
    BlobTracker mBlobTracker;

    Channel mDiffChannel;
    cv::Mat1b mDiffMat;
    gl::TextureRef mDiffTexture;
    Rectf mRoi;

    gl::TextureRef mLogo;

    gl::GlslProgRef	mShader;

    bool mToAddFakeId;
    vec2 mFakeBlobPos;
};

CINDER_APP_NATIVE(KinServerApp, RendererGl)
