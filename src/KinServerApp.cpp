#include "cinder/app/RendererGl.h"
#include "cinder/app/AppNative.h"
#include "cinder/gl/Texture.h"

#include "KCBv2Lib.h"

using namespace ci;
using namespace ci::app;

class KinServerApp : public AppBasic
{
public:
	void draw();
	void setup();
	void keyUp(KeyEvent event)
	{
		int code = event.getCode();
		if (code == KeyEvent::KEY_ESCAPE)
		{
			quit();
		}
	}

	void update()
	{
		if (KCBIsFrameReady(mSensor, FrameSourceTypes_Depth))
		{
			HRESULT hr = KCBGetDepthFrame(mSensor, mDepthFrame);
			mChannel = Channel16u(mDepthFrameDesc.width, mDepthFrameDesc.height, 
				mDepthFrameDesc.bytesPerPixel * mDepthFrameDesc.width, 1, mDepthFrame->Buffer);
			if (!mDepthTexture)
			{
				mDepthTexture = gl::Texture2d::create(mChannel);
			}
			else
			{
				mDepthTexture->update(mChannel);
			}
		}
	}

	void shutdown()
	{
		KCBReleaseDepthFrame(&mDepthFrame);
		KCBCloseSensor(&mSensor);
	}

private:
	Channel16u mChannel;
	KCBDepthFrame* mDepthFrame;
	KCBFrameDescription mDepthFrameDesc;
	int mSensor;
	gl::TextureRef mDepthTexture;
};

void KinServerApp::draw()
{
	if (mChannel.getWidth() != 0)
	{
		gl::draw(mDepthTexture, mDepthTexture->getBounds(), getWindowBounds());
	}
}

void KinServerApp::setup()
{
	HRESULT hr = S_OK;

	mSensor = KCBOpenDefaultSensor();
	if (KCB_INVALID_HANDLE == mSensor)
	{
		hr = E_UNEXPECTED;
	}

	// create re-usable buffer
	if (SUCCEEDED(hr))
	{
		hr = KCBGetDepthFrameDescription(mSensor, &mDepthFrameDesc);
		if (SUCCEEDED(hr))
		{
			hr = KCBCreateDepthFrame(mDepthFrameDesc, &mDepthFrame);
		}
	}
}

CINDER_APP_NATIVE(KinServerApp, RendererGl)
