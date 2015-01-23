#include "KinectDevice.h"
#include "KinectCommonBridgeLib.h"
#include "cinder/app/app.h"
#include "cinder/Log.h"

namespace Kinect
{
using namespace ci;
using namespace ci::app;

struct DeviceV1 : public Device
{
    ~DeviceV1()
    {
        if (depthBuffer != nullptr)
        {
            delete[] depthBuffer;
        }
        if (sensor != KCB_INVALID_HANDLE)
        {
            KinectCloseSensor(sensor);
        }
    }

    int getWidth() const override
    {
        return depthDesc.dwWidth;
    }

    int getHeight() const override
    {
        return depthDesc.dwHeight;
    }

    DeviceV1()
    {
        depthBuffer = nullptr;

        HRESULT hr = S_OK;

        sensor = KinectOpenDefaultSensor();
        if (KCB_INVALID_HANDLE == sensor)
        {
            hr = E_UNEXPECTED;
        }

        if (SUCCEEDED(hr))
        {
            KinectStopColorStream(sensor);
			depthDesc = { sizeof(KINECT_IMAGE_FRAME_FORMAT )};
            KinectGetDepthFrameFormat(sensor, &depthDesc);
            depthBuffer = new uint8_t[depthDesc.cbBufferSize];
            depthChannel = Channel16u(depthDesc.dwWidth, depthDesc.dwHeight,
                                      depthDesc.cbBytesPerPixel * depthDesc.dwWidth, 1, (uint16_t *)depthBuffer);
        }

        if (FAILED(hr))
        {
            CI_LOG_E("Failed to connect to Kinect");
            App::get()->quit();
        }

        App::get()->getSignalUpdate().connect(std::bind(&DeviceV1::update, this));
    }

    void update()
    {
        if (KinectIsDepthFrameReady(sensor))
        {
            if (SUCCEEDED(KinectGetDepthFrame(sensor, depthDesc.cbBufferSize, depthBuffer, nullptr)))
            {
                signalDepthDirty();
            }
        }
    }

    uint8_t *depthBuffer;
    KINECT_IMAGE_FRAME_FORMAT depthDesc;
    int sensor;
};

DeviceRef Device::createV1()
{
    return DeviceRef(new DeviceV1);
}

#if (_WIN32_WINNT >= 0x0602 /*_WIN32_WINNT_WIN8*/)
#include "KCBv2Lib.h"

struct DeviceV2 : public Device
{
    ~DeviceV2()
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

    int getWidth() const override
    {
        return depthDesc.width;
    }

    int getHeight() const override
    {
        return depthDesc.height;
    }

    DeviceV2()
    {
        depthFrame = nullptr;

        HRESULT hr = S_OK;

        sensor = KCBOpenDefaultSensor();
        if (KCB_INVALID_HANDLE == sensor)
        {
            hr = E_UNEXPECTED;
        }

        if (SUCCEEDED(hr))
        {
            hr = KCBGetDepthFrameDescription(sensor, &depthDesc);
            if (SUCCEEDED(hr))
            {
                hr = KCBCreateDepthFrame(depthDesc, &depthFrame);
                depthChannel = Channel16u(depthDesc.width, depthDesc.height,
                                          depthDesc.bytesPerPixel * depthDesc.width, 1, depthFrame->Buffer);
            }
        }

        if (FAILED(hr))
        {
            CI_LOG_E("Failed to connect to Kinect");
            App::get()->quit();
        }

        App::get()->getSignalUpdate().connect(std::bind(&DeviceV2::update, this));
    }

    void update()
    {
        if (KCBIsFrameReady(sensor, FrameSourceTypes_Depth))
        {
            if (SUCCEEDED(KCBGetDepthFrame(sensor, depthFrame)))
            {
                signalDepthDirty();
            }
        }
    }

    KCBDepthFrame *depthFrame;
    KCBFrameDescription depthDesc;
    int sensor;
};

DeviceRef Device::createV2()
{
    return DeviceRef(new DeviceV2);
}

#endif

}
