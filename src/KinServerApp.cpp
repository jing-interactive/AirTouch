#include "cinder/app/RendererGl.h"
#include "cinder/app/AppNative.h"
#include "cinder/gl/Texture.h"
#include "cinder/params/Params.h"
#include "cinder/Log.h"

#include "OscSender.h"
#include "cinder/CinderOpenCV.h"
#include "OpenCV/BlobTracker.h"

#include "KCBv2Lib.h"
#include "MiniConfig.h"

using namespace ci;
using namespace ci::app;

template <typename T>
void updateTexture(gl::TextureRef& tex, const T& src)
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

namespace Kinect
{
typedef std::shared_ptr<struct Device> DeviceRef;

struct Device
{
    static DeviceRef create()
    {
        return DeviceRef(new Device);
    }

    virtual ~Device()
    {
        if (depthFrame != nullptr)
        {
            KCBReleaseDepthFrame(&depthFrame);
        }
        if (sensor != KCB_INVALID_HANDLE)
        {
            KCBCloseSensor(&sensor);
        }
    }

    // Cinder fields
    Channel16u depthChannel;
    gl::TextureRef depthTexture;

protected:
    Device()
    {
        depthFrame = nullptr;
        sensor = KCB_INVALID_HANDLE;

        App::get()->getSignalUpdate().connect(std::bind(&Device::update, this));
    }

    bool setup()
    {
        HRESULT hr = S_OK;

        sensor = KCBOpenDefaultSensor();
        if (KCB_INVALID_HANDLE == sensor)
        {
            hr = E_UNEXPECTED;
        }

        // create re-usable buffer
        if (SUCCEEDED(hr))
        {
            hr = KCBGetDepthFrameDescription(sensor, &depthDesc);
            if (SUCCEEDED(hr))
            {
                hr = KCBCreateDepthFrame(depthDesc, &depthFrame);
                depthChannel = Channel16u(depthDesc.width, depthDesc.height,
                                          depthDesc.bytesPerPixel * depthDesc.width, 1, depthFrame->Buffer);
                depthTexture = gl::Texture::create(depthChannel);
            }
        }

        return SUCCEEDED(hr);
    }

    void update()
    {
        if (sensor == KCB_INVALID_HANDLE)
        {
            if (!setup())
            {
                CI_LOG_E("Failed to connect to Kinect");
                App::get()->quit();
            }
        }

        if (KCBIsFrameReady(sensor, FrameSourceTypes_Depth))
        {
            if (SUCCEEDED(KCBGetDepthFrame(sensor, depthFrame)))
            {
                depthTexture->update(depthChannel);
            }
        }
    }

    // KCB fields
    KCBDepthFrame *depthFrame;
    KCBFrameDescription depthDesc;
    int sensor;
};
}

class KinServerApp : public AppBasic
{
public:
    void setup() override
    {
        mDevice = Kinect::Device::create();

        mParams = params::InterfaceGl::create("params", vec2(300, getConfigUIHeight()));
        setupConfigUI(mParams.get());
        getWindow()->connectPostDraw(&params::InterfaceGl::draw, mParams.get());
        mParams->addButton("Set Bg", std::bind(&KinServerApp::updateBack, this));

        mOscSender.setup(ADDRESS, OSC_PORT);
    }

    void draw() override
    {
		gl::clear();

        float width = getWindowWidth();
        float height = getWindowHeight();
        float halfW = width / 2;
        float halfH = height / 2;
        float spc = width * 0.01;
        gl::draw(mDevice->depthTexture, mDevice->depthTexture->getBounds(),
                 Rectf(spc, spc, halfW - spc, halfH - spc));
        gl::draw(mBackTexture, mBackTexture->getBounds(),
                 Rectf(halfW + spc, spc, width - spc, halfH - spc));
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
    	if (!mBackTexture)
    	{
    		updateBack();
    	}

        osc::Message msg;
        msg.setAddress("/tuio/cursor2d");
        mOscSender.sendMessage(msg);
    }

private:
    void updateBack()
    {
        mBackChannel = mDevice->depthChannel.clone();
        updateTexture(mBackTexture, mBackChannel);
    }

    Kinect::DeviceRef mDevice;
    params::InterfaceGlRef mParams;
    osc::Sender mOscSender;

    // vision
    Channel16u mBackChannel;
    gl::TextureRef mBackTexture;
    vBlobTracker mBlobTracker;
};

CINDER_APP_NATIVE(KinServerApp, RendererGl)
