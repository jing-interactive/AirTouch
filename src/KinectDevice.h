#include "cinder/Cinder.h"
#include "cinder/Channel.h"
#include "cinder/Function.h"

namespace Kinect
{
typedef std::shared_ptr<struct Device> DeviceRef;

struct Device
{
    static DeviceRef createV1();
#if (_WIN32_WINNT >= 0x0602 /*_WIN32_WINNT_WIN8*/)
    static DeviceRef createV2();
#endif

    virtual int getWidth() const = 0;
    virtual int getHeight() const = 0;

    ci::Channel16u depthChannel;

    ci::signals::signal<void()> signalDepthDirty;
};
}
