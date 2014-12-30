#include "cinder/app/RendererGl.h"
#include "cinder/app/AppNative.h"
#include "cinder/gl/Texture.h"
#include "cinder/params/Params.h"
#include "cinder/Log.h"
#include "cinder/PolyLine.h"

#include "OscSender.h"
#include "cinder/CinderOpenCV.h"
#include "OpenCV/BlobTracker.h"
#include "KinectDevice.h"

#include "MiniConfig.h"

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
    void setup() override
    {
        readConfig();
        log::manager()->enableFileLogging();

        mDevice = Kinect::Device::createV2();
        mDevice->signalDepthDirty.connect(std::bind(&KinServerApp::updateDepthRelated, this));

        mRoi.set(0, 0, mDevice->getWidth(), mDevice->getHeight()); // TODO: save / load
        mDiffMat = cv::Mat1b(mDevice->getHeight(), mDevice->getWidth());
        mDiffChannel = Channel(mDevice->getWidth(), mDevice->getHeight(), mDiffMat.step, 1,
                               mDiffMat.ptr());

        mParams = params::InterfaceGl::create("params", vec2(300, getConfigUIHeight() + 100));
        setupConfigUI(mParams.get());
        std::vector<string> smoothNames = { "Off", "Light", "Middle", "High" };
        ADD_ENUM_TO_INT(mParams, SMOOTH, smoothNames)
        getWindow()->connectPostDraw(&params::InterfaceGl::draw, mParams.get());

        mFps = 0;
        mParams->addParam("FPS", &mFps, true);
        mParams->addButton("Set Bg", std::bind(&KinServerApp::updateBack, this));

        mOscSender.setup(ADDRESS, TUIO_PORT);

        getWindow()->setSize(1024, 768);
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

        mParams->setPosition(mLayout.canvases[3].getUpperLeft());
    }

    void draw() override
    {
        gl::clear(ColorA::gray(0.5f));

        if (mDepthTexture)
        {
            gl::draw(mDepthTexture, mDepthTexture->getBounds(), mLayout.canvases[0]);
            gl::draw(mBackTexture, mBackTexture->getBounds(), mLayout.canvases[1]);
            gl::draw(mDiffTexture, mDiffTexture->getBounds(), mLayout.canvases[2]);
            visualizeBlobs(mBlobTracker);
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
        mFps = getFrameRate();
    }

private:
    void updateDepthRelated()
    {
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

        int cx = CENTER_X * mBackChannel.getWidth();
        int cy = CENTER_Y * mBackChannel.getHeight();
        int radius = RADIUS * mBackChannel.getHeight();
        int radius_sq = radius * radius;

        for (int y = mRoi.y1; y < mRoi.y2; y++)
        {
            // TODO: cache row pointer
            for (int x = mRoi.x1; x < mRoi.x2; x++)
            {
                uint16_t bg = *mBackChannel.getData(x, y);
                uint16_t dep = *mDevice->depthChannel.getData(x, y);
                if (dep > 0 && bg - dep > MIN_THRESHOLD_MM && bg - dep < MAX_THRESHOLD_MM)
                {
                    // TODO: optimize
                    if (!MASK_ENABLED || (cx - x) * (cx - x) + (cy - y) * (cy - y) < radius_sq)
                    {
                        mDiffMat(y, x) = 255;
                    }
                }
            }
        }

        if (SMOOTH > 0)
        {
            cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(SMOOTH * 2 + 1, SMOOTH * 2 + 1), cv::Point(SMOOTH, SMOOTH));
            cv::morphologyEx(mDiffMat, mDiffMat, cv::MORPH_OPEN, element);
        }

        updateTexture(mDiffTexture, mDiffChannel);
        std::vector<vBlob> blobs;
        vFindBlobs(mDiffMat, blobs, MIN_AREA);
        mBlobTracker.trackBlobs(blobs);
        sendTuioMessage(mOscSender, mBlobTracker);
    }

    void visualizeBlobs(const vBlobTracker &blobTracker)
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
        scale.x = (mLayout.halfW - mLayout.spc * 2) / mDepthTexture->getWidth();
        scale.y = (mLayout.halfH - mLayout.spc * 2) / mDepthTexture->getHeight();
        gl::pushModelMatrix();
        gl::translate(mLayout.canvases[2].getUpperLeft());
        gl::scale(scale);

        if (MASK_ENABLED)
        {
            float cx = CENTER_X * mBackChannel.getWidth();
            float cy = CENTER_Y * mBackChannel.getHeight();
            float radius = RADIUS * mBackChannel.getHeight();
            gl::drawStrokedCircle(vec2(cx, cy), radius);
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

    void sendTuioMessage(osc::Sender &sender, const vBlobTracker &blobTracker)
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
            // TODO: use RectMapping
            set.addFloatArg((center.x - mRoi.x1) / mRoi.getWidth());
            set.addFloatArg((center.y - mRoi.y1) / mRoi.getHeight());
            set.addFloatArg(blob.velocity.x / mRoi.getWidth());
            set.addFloatArg(blob.velocity.y / mRoi.getHeight());
            set.addFloatArg(0);     // m
            bundle.addMessage(set);                         // add message to bundle

            alive.addIntArg(blob.id);               // add blob to list of ALL active IDs
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
    struct Layout
    {
        float width;
        float height;
        float halfW;
        float halfH;
        float spc;

        Rectf canvases[4];
    } mLayout;

    Kinect::DeviceRef mDevice;
    params::InterfaceGlRef mParams;
    osc::Sender mOscSender;

    gl::TextureRef mDepthTexture;

    // vision
    Channel16u mBackChannel;
    gl::TextureRef mBackTexture;
    vBlobTracker mBlobTracker;

    Channel mDiffChannel;
    cv::Mat1b mDiffMat;
    gl::TextureRef mDiffTexture;
    Rectf mRoi;
};

CINDER_APP_NATIVE(KinServerApp, RendererGl)
