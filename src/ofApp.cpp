#include "ofApp.h"

/*
    If you are struggling to get the device to connect ( especially Windows Users )
    please look at the ReadMe: in addons/ofxKinect/README.md
*/

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	
	kinect.open();		// opens first available kinect
	//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
	
	// print the intrinsic IR sensor values
	if(kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}
	
#ifdef USE_TWO_KINECTS
	kinect2.init();
	kinect2.open();
#endif
	
	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	
	nearThreshold = 230;
	farThreshold = 70;
	bThreshWithOpenCV = true;
	
	ofSetFrameRate(60);
	
	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
	
	// start from the front
	bDrawPointCloud = false;
    
    interactionZone.enableNormals();
    
    setupGUI();
}

//--------------------------------------------------------------
void ofApp::update() {
	
	ofBackground(100, 100, 100);
	
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
		
		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels());
		
		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
		if(bThreshWithOpenCV) {
			grayThreshNear = grayImage;
			grayThreshFar = grayImage;
			grayThreshNear.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		} else {
			
			// or we do it ourselves - show people how they can work with the pixels
			ofPixels & pix = grayImage.getPixels();
			int numPixels = pix.size();
			for(int i = 0; i < numPixels; i++) {
				if(pix[i] < nearThreshold && pix[i] > farThreshold) {
					pix[i] = 255;
				} else {
					pix[i] = 0;
				}
			}
		}
		
		// update the cv images
		grayImage.flagImageChanged();
		
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);
	}
	
#ifdef USE_TWO_KINECTS
	kinect2.update();
#endif
}

//--------------------------------------------------------------
void ofApp::draw() {
	
	ofSetColor(255, 255, 255);
	
	if(bDrawPointCloud) {
		easyCam.begin();
		drawPointCloud();
        
        // draw the 3D workspace
        drawWorkspace(true);
        // draw the 3D interaction zone
        drawInteractionZone();
        
        
		easyCam.end();
	} else {
		// draw from the live kinect
		kinect.drawDepth(10, 10, kinect.width, kinect.height);
        
        // draw the 2D workspace
        drawWorkspace(false);
        
        
		kinect.draw(kinect.width + 20, 10, kinect.width, kinect.height);
		
		grayImage.draw(kinect.width + 20, kinect.height + 20, kinect.width, kinect.height);
		contourFinder.draw(kinect.width + 20, kinect.height + 20, kinect.width, kinect.height);
		
#ifdef USE_TWO_KINECTS
		kinect2.draw(420, 320, 400, 300);
#endif
	}
	
	// draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream;
        
    if(kinect.hasAccelControl()) {
        reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
        << ofToString(kinect.getMksAccel().y, 2) << " / "
        << ofToString(kinect.getMksAccel().z, 2) << endl;
    } else {
        reportStream << "Note: this is a newer Xbox Kinect or Kinect For Windows device," << endl
		<< "motor / led / accel controls are not currently supported" << endl << endl;
    }
    
	reportStream << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
	<< "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
	<< "set near threshold " << nearThreshold << " (press: + -)" << endl
	<< "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
	<< ", fps: " << ofGetFrameRate() << endl
	<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl;

    if(kinect.hasCamTiltControl()) {
    	reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
        << "press 1-5 & 0 to change the led mode" << endl;
    }
    
//	ofDrawBitmapString(reportStream.str(), 20, 652);
    
    panel.draw();
}


//--------------------------------------------------------------
void ofApp::setupGUI(){
    
    
    params.setName("Touch Parameters");
    params.add(interactionZoneHeight.set("Zone Height", 50, 1, 500));
    
    panel.setup(params);
    
    panel.setPosition(10, ofGetHeight() - 200);
    
    panel.loadFromFile("settings.xml");
    
    
}


//--------------------------------------------------------------
void ofApp::drawWorkspace(bool threeD) {
    
    if (threeD){
        ofPushStyle();
        ofPushMatrix();
        // the projected points are 'upside down' and 'backwards'
        ofScale(1, -1, -1);
        ofTranslate(0, 0, -1000); // center the points a bit
        ofEnableDepthTest();
        
        ofFill();
        ofSetColor(ofColor::magenta, 120);
        workspacePlane.draw();
        
        ofNoFill();
        ofSetLineWidth(3);
        ofSetColor(ofColor::magenta);
        workspacePlane.draw();
        
        ofDisableDepthTest();
        ofPopMatrix();
        
        ofPopStyle();
    }
    else{

        ofPushStyle();
        ofPushMatrix();
        ofTranslate(10, 10);
        ofSetColor(ofColor::magenta, 120);
        ofFill();
        workspacePlane2D.draw();
        
        ofSetColor(ofColor::magenta);
        ofNoFill();
        ofSetLineWidth(3);
        workspacePlane2D.draw();
        
        ofPopMatrix();
        ofPopStyle();
    }
    
}

//--------------------------------------------------------------
void ofApp::drawInteractionZone() {
    
    ofPushStyle();
    ofPushMatrix();
    // the projected points are 'upside down' and 'backwards'
    ofScale(1, -1, -1);
    ofTranslate(0, 0, -1000); // center the points a bit
    ofEnableDepthTest();

    ofSetColor(0,255,255);
    
    interactionZone.drawWireframe();
    for (int i=0; i<interactionZone.getNormals().size(); i++){
        
        ofSetColor(255,0,255);
        
        ofDrawLine(interactionZone.getVertices()[i], interactionZone.getVertices()[i]+interactionZone.getNormals()[i]);
        
//        cout << ofToString(n) << endl;
    }
    
    ofDisableDepthTest();
    ofPopMatrix();
    ofPopStyle();
}

//--------------------------------------------------------------
void ofApp::drawPointCloud() {
	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
				mesh.addColor(kinect.getColorAt(x,y));
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
			}
		}
	}
	glPointSize(3);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards' 
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit
	ofEnableDepthTest();
	mesh.drawVertices();
	ofDisableDepthTest();
	ofPopMatrix();
}

//--------------------------------------------------------------
void ofApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
	
#ifdef USE_TWO_KINECTS
	kinect2.close();
#endif
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
	switch (key) {
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;
			
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;
			
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
			
		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
			
		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
			
		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
			
		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;
			
//		case 'c':
//			kinect.setCameraTiltAngle(0); // zero the tilt
//			kinect.close();
//			break;
			
		case '1':
			kinect.setLed(ofxKinect::LED_GREEN);
			break;
			
		case '2':
			kinect.setLed(ofxKinect::LED_YELLOW);
			break;
			
		case '3':
			kinect.setLed(ofxKinect::LED_RED);
			break;
			
		case '4':
			kinect.setLed(ofxKinect::LED_BLINK_GREEN);
			break;
			
		case '5':
			kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
			break;
			
		case '0':
			kinect.setLed(ofxKinect::LED_OFF);
			break;
			
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;
			
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
        case 'c':
            workspace.clear();
            workspacePlane.clear();
            workspacePlane2D.clear();
            interactionZone.clear();
            isWorkspaceDefined = false;
            break;
	}
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{
	
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{

    if (!isWorkspaceDefined){
        
        workspace.push_back(kinect.getWorldCoordinateAt(x-10, y-10));
        
        workspacePlane.addVertex(workspace.back());
        workspacePlane2D.addVertex(ofVec3f(x-10,y-10,0));
        
        isWorkspaceDefined = workspace.size() == 4;
        
        if (isWorkspaceDefined) {
            workspacePlane.close();
            workspacePlane2D.close();
            
            // create the mesh envelope for the interaction zone
            buildInteractionZone();
        }
    }
    
    
}

//--------------------------------------------------------------
void ofApp::buildInteractionZone(){
    
    // create bottom mesh
    interactionZone.addVertex(workspace[0]);
    interactionZone.addVertex(workspace[1]);
    interactionZone.addVertex(workspace[2]);
    interactionZone.addVertex(workspace[3]);
    
    interactionZone.addIndex(0);
    interactionZone.addIndex(1);
    interactionZone.addIndex(2);
    
    interactionZone.addIndex(0);
    interactionZone.addIndex(2);
    interactionZone.addIndex(3);
    
    // set the bottom normals
    calcNormals(interactionZone, true);
    
    // create the top mesh
    interactionZone.addVertex(workspace[0] + interactionZone.getNormals()[0]); // 4
    interactionZone.addVertex(workspace[1] + interactionZone.getNormals()[1]); // 5
    interactionZone.addVertex(workspace[2] + interactionZone.getNormals()[2]); // 6
    interactionZone.addVertex(workspace[3] + interactionZone.getNormals()[3]); // 7
    
    interactionZone.addIndex(4);
    interactionZone.addIndex(5);
    interactionZone.addIndex(6);
    
    interactionZone.addIndex(4);
    interactionZone.addIndex(6);
    interactionZone.addIndex(7);
    
    // create left side
    interactionZone.addIndex(0);
    interactionZone.addIndex(4);
    interactionZone.addIndex(5);
    
    interactionZone.addIndex(0);
    interactionZone.addIndex(5);
    interactionZone.addIndex(1);
    
    // create front side
    interactionZone.addIndex(1);
    interactionZone.addIndex(5);
    interactionZone.addIndex(6);
    
    interactionZone.addIndex(1);
    interactionZone.addIndex(6);
    interactionZone.addIndex(2);
    
    // create right side
    interactionZone.addIndex(2);
    interactionZone.addIndex(6);
    interactionZone.addIndex(7);
    
    interactionZone.addIndex(2);
    interactionZone.addIndex(7);
    interactionZone.addIndex(3);
    
    // create rear side
    interactionZone.addIndex(3);
    interactionZone.addIndex(7);
    interactionZone.addIndex(4);
    
    interactionZone.addIndex(3);
    interactionZone.addIndex(4);
    interactionZone.addIndex(0);

    
}

//--------------------------------------------------------------------------
void ofApp::calcNormals( ofMesh & mesh, bool bNormalize ){
    
    for( int i=0; i < mesh.getVertices().size(); i++ ) mesh.addNormal(ofPoint(0,0,0));
    
    for( int i=0; i < mesh.getIndices().size(); i+=3 ){
        const int ia = mesh.getIndices()[i];
        const int ib = mesh.getIndices()[i+1];
        const int ic = mesh.getIndices()[i+2];
        
        ofVec3f e1 = mesh.getVertices()[ia] - mesh.getVertices()[ib];
        ofVec3f e2 = mesh.getVertices()[ic] - mesh.getVertices()[ib];
        ofVec3f no = e2.cross( e1 );
        
        // depending on your clockwise / winding order, you might want to reverse the e2 / e1 above if your normals are flipped.
        
        mesh.getNormals()[ia] += no;
        mesh.getNormals()[ib] += no;
        mesh.getNormals()[ic] += no;
    }
    
    if (bNormalize){
        for( int i=0; i < mesh.getVertices().size(); i++ ) mesh.getNormals()[i].scale(interactionZoneHeight);

    }
}

//--------------------------------------------------------------
void ofApp::updateInteractionZone(float height){
    
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{

}
