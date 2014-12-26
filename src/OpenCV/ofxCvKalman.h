/*
 * ofxCvKalman.h
 *
 *  Created on: 23-jun-2009
 *      Author: art
 */

#ifndef OFXCVKALMAN_H_
#define OFXCVKALMAN_H_

#include <opencv2/video/tracking.hpp>

using namespace cv;

class ofxCvKalman {
public:
	ofxCvKalman(float initial)
	{
		measurement = Mat::zeros(1, 1, CV_32F);
		processNoise.create(2, 1, CV_32F);
		state.create(2, 1, CV_32F); /* (var, delta_var) */

		KF.init(2, 1, 0);
        // TODO
		//KF.transitionMatrix = Mat_<float>(2, 2);
  //      KF.transitionMatrix << 1, 1, 0, 1;
		setIdentity(KF.measurementMatrix);
		setIdentity(KF.processNoiseCov, Scalar::all(1e-8));
		setIdentity(KF.measurementNoiseCov, Scalar::all(1e-8));
		setIdentity(KF.errorCovPost, Scalar::all(1e-5));
		KF.statePost.at<float>(0) = initial;

		smooth_value = initial;
	}
	
	void update(float next)
	{
		Mat prediction = KF.predict();
		measurement.at<float>(0) = next;
		Mat correction = KF.correct(measurement); 

		randn( processNoise, Scalar(0), Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));
		state = KF.transitionMatrix*state + processNoise;
		smooth_value = correction.at<float>(0);
	}

	float value() const
	{
		return smooth_value;
	}
	
private:
	cv::KalmanFilter KF;
	cv::Mat state;
	cv::Mat processNoise;
	cv::Mat measurement;

	float smooth_value;
};

#endif /* OFXCVKALMAN_H_ */
