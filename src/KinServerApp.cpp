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
        log::manager()->enableFileLogging();

        mDevice = Kinect::Device::createV2();
        mDevice->signalDepthDirty.connect(std::bind(&KinServerApp::updateDepthRelated, this));

        mRoi.set(0, 0, mDevice->getWidth(), mDevice->getHeight()); // TODO: save / load
        mDiffMat = cv::Mat1b(mDevice->getHeight(), mDevice->getWidth());
        mDiffChannel = Channel(mDevice->getWidth(), mDevice->getHeight(), mDiffMat.step, 1,
                               mDiffMat.ptr());

        mParams = params::InterfaceGl::create("params", vec2(300, getConfigUIHeight() + 100));
        setupConfigUI(mParams.get());
        getWindow()->connectPostDraw(&params::InterfaceGl::draw, mParams.get());
        mParams->addButton("Set Bg", std::bind(&KinServerApp::updateBack, this));

        mOscSender.setup(ADDRESS, OSC_PORT);

        getWindow()->setSize(1024, 768);
    }

    void resize() override
    {
        float spc = getWindowWidth() * 0.01;
        mParams->setPosition(ivec2(getWindowWidth() / 2 + spc, getWindowHeight() / 2 + spc));
    }

    void draw() override
    {
        gl::clear(ColorA::gray(0.5f));

        float width = getWindowWidth();
        float height = getWindowHeight();
        float halfW = width / 2;
        float halfH = height / 2;
        float spc = width * 0.01;
        if (mDepthTexture)
        {
            gl::draw(mDepthTexture, mDepthTexture->getBounds(),
                     Rectf(spc, spc, halfW - spc, halfH - spc));
        }
        if (mBackTexture)
        {
            gl::draw(mBackTexture, mBackTexture->getBounds(),
                     Rectf(halfW + spc, spc, width - spc, halfH - spc));
        }
        if (mDiffTexture)
        {
            gl::draw(mDiffTexture, mDiffTexture->getBounds(),
                     Rectf(spc, halfH + spc, halfW - spc, height - spc));
        }
        visualizeBlobs(mBlobTracker);
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

        for (int y = mRoi.y1; y < mRoi.y2; y++)
        {
            // TODO: cache row pointer
            for (int x = mRoi.x1; x < mRoi.x2; x++)
            {
                uint16_t bg = *mBackChannel.getData(x, y);
                uint16_t dep = *mDevice->depthChannel.getData(x, y);
                if (dep > 0 && bg - dep > DISTANCE_NEAR && bg - dep < DISTANCE_FAR)
                {
                    mDiffMat(y, x) = 255;
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
        for (const auto &blob : blobTracker.trackedBlobs)
        {
            PolyLine2 line;
            for (const auto &pt : blob.pts)
            {
                line.push_back(vec2(pt.x, pt.y));
            }
            line.setClosed();
            gl::draw(line);
        }
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
