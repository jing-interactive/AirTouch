#include "cinder/Cinder.h"
#include "cinder/Channel.h"
#include "cinder/Function.h"

namespace Kinect
{
typedef std::shared_ptr<struct Device> DeviceRef;

struct Device
{
    static DeviceRef createV1();
    static DeviceRef createV2();

    virtual int getWidth() const = 0;
    virtual int getHeight() const = 0;

    ci::Channel16u depthChannel;

    ci::signals::signal<void()> signalDepthDirty;
};
}
