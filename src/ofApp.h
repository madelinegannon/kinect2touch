#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxGui.h"

// Windows users:
// You MUST install the libfreenect kinect drivers in order to be able to use
// ofxKinect. Plug in the kinect and point your Windows Device Manager to the
// driver folder in:
//
//     ofxKinect/libs/libfreenect/platform/windows/inf
//
// This should install the Kinect camera, motor, & audio drivers.
//
// You CANNOT use this driver and the OpenNI driver with the same device. You
// will have to manually update the kinect device to use the libfreenect drivers
// and/or uninstall/reinstall it in Device Manager.
//
// No way around the Windows driver dance, sorry.

// uncomment this to read from two kinects simultaneously
//#define USE_TWO_KINECTS

class ofApp : public ofBaseApp {
public:
	
	void setup();
	void update();
	void draw();
	void exit();
    
    ofxPanel panel;
    ofParameterGroup params;
    void setupGUI();
	
	void drawPointCloud();
	
	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	
	ofxKinect kinect;
	
#ifdef USE_TWO_KINECTS
	ofxKinect kinect2;
#endif
	
	ofxCvColorImage colorImg;
	
	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
	
	ofxCvContourFinder contourFinder;
	
	bool bThreshWithOpenCV;
	bool bDrawPointCloud;
	
	int nearThreshold;
	int farThreshold;
	
	int angle;
	
	// used for viewing the point cloud
	ofEasyCam easyCam;
    
    bool isWorkspaceDefined = false;
    vector<ofVec3f> workspace; // list of four points that make up the workspace edges
    ofPolyline workspacePlane;
    ofPolyline workspacePlane2D;
    
    void drawWorkspace(bool threeD);
    void drawInteractionZone();
    
    ofMesh interactionZone;
    void buildInteractionZone();
    void calcNormals( ofMesh & mesh, bool bNormalize );

    ofParameter<float> interactionZoneHeight=50;
    void updateInteractionZone(float &height);
    
    ofVec3f baseCentroid;
    ofParameter<float> zOffset=0;
    void updateZOffset(float &offset);
    float prevOffset;
    
    ofVec3f topCentroid;
    ofVec3f btmCentroid;
};
