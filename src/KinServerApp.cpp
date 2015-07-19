#include "cinder/app/RendererGl.h"
#include "cinder/app/App.h"
#include "cinder/gl/gl.h"
#include "cinder/params/Params.h"
#include "cinder/Log.h"
#include "cinder/PolyLine.h"

#include "OscSender.h"
#include "OpenCV/CinderOpenCV.h"
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

class KinServerApp : public App
{
public:
    void setup() override
    {
        const auto& args = getCommandLineArgs();
        readConfig();
        log::manager()->enableFileLogging();

        {
            mParams = params::InterfaceGl::create("params", vec2(400, getConfigUIHeight() + 100));
            setupConfigUI(mParams.get());
            std::vector<string> smoothNames = { "Off", "Light", "Middle", "High" };
            ADD_ENUM_TO_INT(mParams, TRACKING_SMOOTH, smoothNames);
            getWindow()->getSignalPostDraw().connect(std::bind(&params::InterfaceGl::draw, mParams.get()));

            mFps = 0;
            mFrameCounter = 0;
            mSecondsForFps = getElapsedSeconds();

            mParams->addParam("FPS", &mFps, true);
            mParams->addButton("Set Bg", std::bind(&KinServerApp::updateBack, this));
            mParams->addButton("Reset In/Out", std::bind(&KinServerApp::resetInOut, this));
        }

        Kinect::DeviceType type = Kinect::V1;
#ifdef KINECT_V2
        type = Kinect::V2;
#endif
        mDevice = Kinect::Device::create(type);
        if (!mDevice->isValid())
        {
            quit();
            return;
        }
        mDevice->signalDepthDirty.connect(std::bind(&KinServerApp::updateDepthRelated, this));

        mDepthW = mDevice->getWidth();
        mDepthH = mDevice->getHeight();
        mDiffMat = cv::Mat1b(mDepthH, mDepthW);
        mDiffChannel = Channel(mDepthW, mDepthH, mDiffMat.step, 1,
            mDiffMat.ptr());

        mOscSender.setup(ADDRESS, TUIO_PORT);

        getWindow()->setSize(1024, 768);

        try
        {
            mLogo = gl::Texture::create(loadImage(loadAsset("logo.png")));
        }
        catch (std::exception& e)
        {
            console() << e.what() << std::endl;
        }

        try
        {
            mShader = gl::GlslProg::create(loadAsset("depthMap.vs"), loadAsset("depthMap.fs"));
            mShader->uniform("image", 0);
        }
        catch (std::exception& e)
        {
            console() << e.what() << std::endl;
        }
    }

    void resize() override
    {
        mLayout.width = getWindowWidth();
        mLayout.height = getWindowHeight();
        mLayout.halfW = mLayout.width / 2;
        mLayout.halfH = mLayout.height / 2;
        mLayout.spc = mLayout.width * 0.04;

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

        mParams->draw();
    }

    void keyUp(KeyEvent event) override
    {
        int code = event.getCode();
        if (code == KeyEvent::KEY_ESCAPE)
        {
            quit();
        }
    }

    void update() override
    {
        mInputRoi.set(
            INPUT_X1 * mDepthW,
            INPUT_Y1 * mDepthH,
            INPUT_X2 * mDepthW,
            INPUT_Y2 * mDepthH
            );
        mOutputMap.set(
            OUTPUT_X1 * mDepthW,
            OUTPUT_Y1 * mDepthH,
            OUTPUT_X2 * mDepthW,
            OUTPUT_Y2 * mDepthH
            );
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

        for (int yy = mInputRoi.y1; yy < mInputRoi.y2; yy++)
        {
            // TODO: cache row pointer
            int y = yy;
            for (int xx = mInputRoi.x1; xx < mInputRoi.x2; xx++)
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
            if (LEFT_RIGHT_FLIPPED) cx = mDepthW - cx;
            float cy = CENTER_Y * mDepthH;
            float radius = RADIUS * mDepthH;
            gl::drawStrokedCircle(vec2(cx, cy), radius);
        }
        {
            gl::ScopedColor scope(ColorAf(1, 0, 0, 0.5f));
            gl::drawStrokedRect(mInputRoi);
        }
        {
            gl::ScopedColor scope(ColorAf(0, 1, 0, 0.5f));
            gl::drawStrokedRect(mOutputMap);
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

            if (!mInputRoi.contains(center)) continue;

            osc::Message set;
            set.setAddress("/tuio/2Dcur");
            set.addStringArg("set");
            set.addIntArg(blob.id);             // id
            float mappedX = lmap(center.x / mDepthW, INPUT_X1, INPUT_X2, OUTPUT_X1, OUTPUT_X2);
            float mappedY = lmap(center.y / mDepthH, INPUT_Y1, INPUT_Y2, OUTPUT_Y1, OUTPUT_Y2);
            set.addFloatArg(mappedX);
            set.addFloatArg(mappedY);
            set.addFloatArg(blob.velocity.x / mOutputMap.getWidth());
            set.addFloatArg(blob.velocity.y / mOutputMap.getHeight());
            set.addFloatArg(0);     // m
            bundle.addMessage(set);                         // add message to bundle

            alive.addIntArg(blob.id);               // add blob to list of ALL active IDs
        }

        bundle.addMessage(alive);    //add message to bundle
        bundle.addMessage(fseq);     //add message to bundle

        sender.sendBundle(bundle); //send bundle
    }

    void resetInOut()
    {
        INPUT_X1 = INPUT_Y1 = OUTPUT_X1 = OUTPUT_Y1 = 0;
        INPUT_X2 = INPUT_Y2 = OUTPUT_X2 = OUTPUT_Y2 = 1;
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

    Rectf mInputRoi;
    Rectf mOutputMap;

    gl::TextureRef mLogo;

    gl::GlslProgRef	mShader;
};

CINDER_APP(KinServerApp, RendererGl)
