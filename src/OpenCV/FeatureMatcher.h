#ifndef _FEATURE_MATCHER_H_
#define _FEATURE_MATCHER_H_

#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>

using namespace std;

namespace cv
{
class FeatureMatcher
{
public:
	int k_min;
	int k_max;
	FeatureMatcher(string detectorType = "SURF", string extractorType="SURF", string matcherType="FlannBased"):
	_detectorType(detectorType),_extractorType(extractorType),_matcherType(matcherType){
		_detector = FeatureDetector::create(detectorType);
		_extractor = DescriptorExtractor::create(extractorType);
		_matcher = DescriptorMatcher::create(matcherType);

		k_min = 5;
		k_max = 55;

		instinsic_matrix = Mat_<float>(3, 3);
		instinsic_matrix = 0;
		instinsic_matrix(0, 0) = 215;//fx
		instinsic_matrix(1, 1) = 215;//fy
		instinsic_matrix(2, 2) = 1;//22
		instinsic_matrix(0, 2) = 640;//cx
		instinsic_matrix(1, 2) = 480;//cy
	}

	bool setObjectImage(const Mat& src)
	{
		_img1 = src;
		return _calculate(src, keypoints1, descriptors1);
	}

	bool setSceneImage(const Mat& src)
	{
		_img2 = src;
		if (!_calculate(src, keypoints2, descriptors2))
			return false;
		_matcher->match( descriptors1, descriptors2, matches);
#if 1
		double max_dist = 0; 
		double min_dist = 100;

		//-- Quick calculation of max and min distances between keypoints
		for( int i = 0; i < descriptors1.rows; i++ )
		{ 
			double dist = matches[i].distance;
			if( dist < min_dist ) min_dist = dist;
			if( dist > max_dist ) max_dist = dist;
		}
#endif
		//-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist )
		//-- PS.- radiusMatch can also be used here.
		good_matches.clear();

		for( int i = 0; i < descriptors1.rows; i++ )
		{ 
			float dist = matches[i].distance;
			if( dist >= min_dist*k_min*0.01 && dist <= max_dist*k_max*0.01)
			{
				good_matches.push_back( matches[i]);
			}
		}
		return true;
	}

	Mat findHomography(int ransacThresh = 3)
	{
		vector<Point2f> pt1,pt2;

		int n_matches = good_matches.size();
		for(int i = 0; i < n_matches; i++){
			pt1.push_back(keypoints1[good_matches[i].queryIdx].pt);
			pt2.push_back(keypoints2[good_matches[i].trainIdx].pt);
		}
		_homo = cv::findHomography(pt1, pt2, Mat(), CV_RANSAC, ransacThresh);
		return _homo;
	}

	bool solve2()
	{
		Mat_<float> rotation_matrix (3, 3);
		Mat_<float> rotation_vector (3, 1);
		Mat_<float> translation_vector (3, 1);

		Mat inv_instinsic = (Mat_<float>(3,3) << 1/instinsic_matrix(0,0) , 0 , -instinsic_matrix(0,2)/instinsic_matrix(0,0) ,
			0 , 1/instinsic_matrix(1,1) , -instinsic_matrix(1,2)/instinsic_matrix(1,1) ,
			0 , 0 , 1);
		// Column vectors of homography
		Mat h1 = _homo.col(0);//Mat_<float>(3,1) << _homo.at<float>(0,0) , _homo.at<float>(1,0) , _homo.at<float>(2,0));
		Mat h2 = _homo.col(1);//(Mat_<float>(3,1) << _homo.at<float>(0,1) , _homo.at<float>(1,1) , _homo.at<float>(2,1));
		Mat h3 = _homo.col(2);//(Mat_<float>(3,1) << _homo.at<float>(0,2) , _homo.at<float>(1,2) , _homo.at<float>(2,2));

		Mat inverseH1 = inv_instinsic * h1;
		// Calculate a length, for normalizing
		double lambda = sqrt(inverseH1.at<float>(0,0)*inverseH1.at<float>(0,0) +
			inverseH1.at<float>(1,0)*inverseH1.at<float>(1,0) +
			inverseH1.at<float>(2,0)*inverseH1.at<float>(2,0));

		Mat rotationMatrix;

		if(lambda == 0) 
		{
			printf("Lambda was 0...\n");
			return false;
		}
		lambda = 1/lambda;
		// Normalize inversecamera_matrix
		inv_instinsic *= lambda;
// 		inversecamera_matrix.at<float>(0,0) *= lambda;
// 		inversecamera_matrix.at<float>(1,0) *= lambda;
// 		inversecamera_matrix.at<float>(2,0) *= lambda;
// 		inversecamera_matrix.at<float>(0,1) *= lambda;
// 		inversecamera_matrix.at<float>(1,1) *= lambda;
// 		inversecamera_matrix.at<float>(2,1) *= lambda;
// 		inversecamera_matrix.at<float>(0,2) *= lambda;
// 		inversecamera_matrix.at<float>(1,2) *= lambda;
// 		inversecamera_matrix.at<float>(2,2) *= lambda;

		// Column vectors of rotation matrix
		Mat r1 = inv_instinsic * h1;
		Mat r2 = inv_instinsic * h2;
		Mat r3 = r1.cross(r2);    // Orthogonal to r1 and r2

		// Put rotation columns into rotation matrix... with some unexplained sign changes
		rotationMatrix = (Mat_<float>(3,3) <<  r1.at<float>(0,0) , -r2.at<float>(0,0) , -r3.at<float>(0,0) ,
			-r1.at<float>(1,0) , r2.at<float>(1,0) , r3.at<float>(1,0) ,
			-r1.at<float>(2,0) , r2.at<float>(2,0) , r3.at<float>(2,0));

		// Translation vector T
		_Trans = inv_instinsic * h3;
		_Trans.at<float>(0,0) *= 1;
		_Trans.at<float>(1,0) *= -1;
		_Trans.at<float>(2,0) *= -1;

		SVD decomposed(rotationMatrix); // I don't really know what this does. But it works.
		rotationMatrix = decomposed.u * decomposed.vt;

		_ModelView = (Mat_<float>(4,4) << rotationMatrix.at<float>(0,0), rotationMatrix.at<float>(0,1), rotationMatrix.at<float>(0,2), _Trans.at<float>(0,0),
			rotationMatrix.at<float>(1,0), rotationMatrix.at<float>(1,1), rotationMatrix.at<float>(1,2), _Trans.at<float>(1,0),
			rotationMatrix.at<float>(2,0), rotationMatrix.at<float>(2,1), rotationMatrix.at<float>(2,2), _Trans.at<float>(2,0),
			0,0,0,1);
	}

	void solvePnp()
	{
		vector<Point3f> pt1;
		vector<Point2f> pt2;

		int n_matches = good_matches.size();
		for(int i = 0; i < n_matches; i++){
			pt1.push_back(Point3f(keypoints1[good_matches[i].queryIdx].pt));
			pt2.push_back(keypoints2[good_matches[i].trainIdx].pt);
		}

		Mat_<float> R_pnp(3, 1);

		Mat_<float> dist_coef (5, 1);
		dist_coef = 0;

		solvePnPRansac(pt1, pt2, instinsic_matrix, dist_coef, R_pnp, _Trans);

		Rodrigues(R_pnp, _Rot);

		cout<<_Trans<<endl;
		cout<<_Rot<<endl;

		_ModelView = (Mat_<double>(4,4) << _Rot(0,0), _Rot(0,1), _Rot(0,2), _Trans(0,0),
			_Rot(1,0), _Rot(1,1), _Rot(1,2), _Trans(1,0),
			_Rot(2,0), _Rot(2,1), _Rot(2,2), _Trans(2,0),
			0,0,0,1);
		cout<<_ModelView<<endl;

#if 0
		// use the rotation vector generated from OpenCV's cvFindExtrinsicCameraParams2() 
		float rv[] = {rotation->data.fl[0], rotation->data.fl[1], rotation->data.fl[2] }; 

		// use the translation vector generated from OpenCV's cvFindExtrinsicCameraParams2() 
		float tv[] = {translation->data.fl[0], translation->data.fl[1], translation->data.fl[2]} ; 

		float rm[9];
		// rotation matrix
		CvMat* rotMat = cvCreateMat (3, 3, CV_32FC1); 

		// rotation vectors can be converted to a 3-by-3 rotation matrix
		// by calling cvRodrigues2() - Source: O'Reilly Learning OpenCV
		cvRodrigues2(rotation, rotMat, NULL);

		for(int i=0; i<9; i++){
			rm[i] = rotMat->data.fl[i];
		}

		rv[1]=-1.0f * rv[1]; rv[2]=-1.0f * rv[2];
		//Convert the rotation vector into a matrix here.

		//Complete matrix ready to use for OpenGL
		float RTMat[] = {rm[0], rm[3], rm[6], 0.0f,
			rm[1], rm[4], rm[7], 0.0f,
			rm[2], rm[5], rm[8], 0.0f,
			tv[0], -tv[1], -tv[2], 1.0f};
#endif
	}

	void draw(Mat& match_img)
	{
		drawMatches(_img1, keypoints1, _img2, keypoints2, good_matches, match_img, Scalar::all(-1), Scalar::all(-1), 
			vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
	}

	Mat_<double> _Trans;
	Mat_<double> _Rot;
	Mat_<double> _ModelView;

private:
	bool _calculate(const Mat& src, vector<KeyPoint>& pts, Mat& descr)
	{
		_detector->detect(src, pts);
		_extractor->compute(src, pts, descr);

		return pts.size() > 0;
	}

	string _detectorType;
	string _extractorType;
	string _matcherType;

	Ptr<FeatureDetector> _detector;
	Ptr<DescriptorExtractor> _extractor;
	Ptr<DescriptorMatcher> _matcher;

	vector<KeyPoint> keypoints1,keypoints2;
	Mat descriptors1, descriptors2;
	vector<DMatch> matches;	
	vector<DMatch> good_matches;

	Mat _img1,_img2;
	Mat _homo;
	Mat_<float> instinsic_matrix;
};
}
#endif
