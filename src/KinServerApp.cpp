#include "cinder/app/RendererGl.h"
#include "cinder/app/App.h"
#include "cinder/gl/gl.h"
#include "cinder/params/Params.h"
#include "cinder/Log.h"
#include "cinder/PolyLine.h"

#include "cinder/osc/Osc.h"
#include "CinderOpenCV.h"
#include "BlobTracker.h"

#include "DepthSensor.h"
#include "Cinder-VNM/include/MiniConfig.h"
#include "Cinder-VNM/include/TextureHelper.h"

using namespace ci;
using namespace ci::app;

class KinServerApp : public App
{
public:
    void setup() override
    {
        const auto& args = getCommandLineArgs();
        readConfig();
        log::makeLogger<log::LoggerFile>();

        {
            mParams = createConfigUI({ 400, 600 });
            std::vector<string> smoothNames = { "Off", "Light", "Middle", "High" };
            ADD_ENUM_TO_INT(mParams, TRACKING_SMOOTH, smoothNames);

            mParams->addParam("FPS", &mFps, true);
            mParams->addButton("Set Bg", std::bind(&KinServerApp::updateBack, this));
            mParams->addButton("Reset In/Out", [] {
                INPUT_X1 = INPUT_Y1 = OUTPUT_X1 = OUTPUT_Y1 = 0;
                INPUT_X2 = INPUT_Y2 = OUTPUT_X2 = OUTPUT_Y2 = 1;
            });
        }

        ds::DeviceType type = ds::DeviceType(SENSOR_TYPE);
        ds::Option option;
        option.enableDepth = !INFRARED_MODE;
        option.enableInfrared = INFRARED_MODE;
        mDevice = ds::Device::create(type, option);
        if (!mDevice->isValid())
        {
            quit();
            return;
        }

        if (INFRARED_MODE)
        {
            mDirtyConnection = mDevice->signalInfraredDirty.connect(std::bind(&KinServerApp::updateDepthRelated, this));
        }
        else
        {
            mDirtyConnection = mDevice->signalDepthDirty.connect(std::bind(&KinServerApp::updateDepthRelated, this));
        }

        mDepthW = mDevice->getDepthSize().x;
        mDepthH = mDevice->getDepthSize().y;
        mDiffMat = cv::Mat1b(mDepthH, mDepthW);
        mDiffChannel = Channel(mDepthW, mDepthH, mDiffMat.step, 1,
            mDiffMat.ptr());

        mOscSender = std::make_shared<osc::SenderUdp>(10000, ADDRESS, TUIO_PORT);
        mOscSender->bind();

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
        mFps = getAverageFps();

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
        mTargetChannel = INFRARED_MODE ? &mDevice->infraredChannel : &mDevice->depthChannel;

        updateTexture(mDepthTexture, *mTargetChannel);

        if (!mBackTexture)
        {
            updateBack();
        }

        mDiffMat.setTo(cv::Scalar::all(0));

        int cx = CENTER_X * mDepthW;
        int cy = CENTER_Y * mDepthH;
        int radius = RADIUS * mDepthH;
        int radius_sq = radius * radius;

        float depthToMmScale = INFRARED_MODE ? 0.01 :  mDevice->getDepthToMmScale();
        float minThresholdInDepthUnit = MIN_THRESHOLD_MM / depthToMmScale;
        float maxThresholdInDepthUnit = MAX_THRESHOLD_MM / depthToMmScale;

        for (int yy = mInputRoi.y1; yy < mInputRoi.y2; yy++)
        {
            // TODO: cache row pointer
            int y = yy;
            for (int xx = mInputRoi.x1; xx < mInputRoi.x2; xx++)
            {
                int x = LEFT_RIGHT_FLIPPED ? (mDepthW - xx) : xx;
                auto bg = *mBackChannel.getData(x, y);
                auto dep = *mTargetChannel->getData(x, y);
                auto diff = dep - bg;
                if (dep > 0 && diff > minThresholdInDepthUnit && diff < maxThresholdInDepthUnit)
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
        sendTuioMessage(*mOscSender, mBlobTracker);
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
            gl::drawStrokedCircle(vec2(cx, cy), radius, 40);
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

    int remapTuioId(int srcId)
    {
#define kMagicNumber 100
        return (srcId % kMagicNumber) + kMagicNumber * SERVER_ID;
    }

    void sendTuioMessage(osc::SenderUdp &sender, const BlobTracker &blobTracker)
    {
        osc::Bundle bundle;

        osc::Message alive;
        {
            alive.setAddress("/tuio/2Dcur");
            alive.append("alive");
        }

        osc::Message fseq;
        {
            fseq.setAddress("/tuio/2Dcur");
            fseq.append("fseq");
            fseq.append((int32_t)getElapsedFrames());
        }

        SERVER_COUNT = math<int>::max(1, SERVER_COUNT);
        SERVER_ID = math<int>::clamp(SERVER_ID, 0, SERVER_COUNT - 1);
        float newRegion = 1 / (float)SERVER_COUNT;
        for (const auto &blob : blobTracker.trackedBlobs)
        {
            //Point2f center(blob.center.x + depthOrigin.x, blob.center.y + depthOrigin.y);
            vec2 center(blob.center.x, blob.center.y);

            if (!mInputRoi.contains(center)) continue;

            int blobId = remapTuioId(blob.id);
            osc::Message set;
            set.setAddress("/tuio/2Dcur");
            set.append("set");
            set.append(blobId);             // id
            float mappedX = lmap(center.x / mDepthW, INPUT_X1, INPUT_X2, OUTPUT_X1, OUTPUT_X2);
            mappedX = (SERVER_ID + mappedX) * newRegion;
            float mappedY = lmap(center.y / mDepthH, INPUT_Y1, INPUT_Y2, OUTPUT_Y1, OUTPUT_Y2);
            set.append(mappedX);
            set.append(mappedY);
            set.append(blob.velocity.x / mOutputMap.getWidth());
            set.append(blob.velocity.y / mOutputMap.getHeight());
            set.append(0);     // m
            bundle.append(set);                         // add message to bundle

            alive.append(blobId);               // add blob to list of ALL active IDs
        }

        bundle.append(alive);    //add message to bundle
        bundle.append(fseq);     //add message to bundle

        sender.send(bundle); //send bundle
    }

    void updateBack()
    {
        mBackChannel = mTargetChannel->clone();
        updateTexture(mBackTexture, mBackChannel);
    }

    float mFps = 0;

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

    ci::Channel16u* mTargetChannel = nullptr;
    ds::DeviceRef mDevice;
    signals::Connection mDirtyConnection;
    params::InterfaceGlRef mParams;
    std::shared_ptr<osc::SenderUdp> mOscSender;
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
