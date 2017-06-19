#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "ofxRay.h"
//#include "ofxCvMin.h"

// based on calibration routine for Augmented Hand Series: https://github.com/CreativeInquiry/digital_art_2014/blob/master/HandArtwork/src/LeapToCameraCalibrator.cpp
// and from Tactum:

class CalibrateCoords {
public:
	
    
    void setup(int camWidth, int camHeight);
    void loadFingerTipPoints(string filePath);
    
    void loadPoints(vector<ofVec2f> image, vector<ofVec3f> world);
    
    
    void resetProjector();
    void correctCamera();
    void correctCameraPNP(ofxCv::Calibration & myCalibration);
    void setIntrinsics(cv::Mat cameraMatrix);
    void setExtrinsics(cv::Mat rotation, cv::Mat translation);
    
    void drawWorldPoints();
    void drawImagePoints();
    
    // helpers from ofxCvMin
    ofMatrix4x4 makeProjectionMatrix(cv::Mat cameraMatrix, cv::Size imageSize);
    ofMatrix4x4 makeMatrix(cv::Mat rotation, cv::Mat translation);
    
    
    ofVec2f resolution;
    bool calibrated;
    bool hasFingerCalibPoints;
    bool switchYandZ;
    
    float throwRatioX, throwRatioY;
    float lensOffsetX, lensOffsetY;
    float translationX, translationY, translationZ;
    float rotationX, rotationY, rotationZ;
    float throwRatio;
    ofVec2f lensOffset;
    
    vector<ofVec2f> calibVectorImage;
    vector<ofVec3f> calibVectorWorld;
    cv::Mat camera, distortion;
    cv::Mat rotation, translation;
    ofProjector projector;
    string dirNameLoaded;
    
};
