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
    
    if (useCalibrated){
        readCalibrationFiles("imagePts.txt", "worldPts.txt");
        calibration.setup( 1024, 768); //  cameraWidth		= 1024; cameraHeight	= 768;
        calibration.loadPoints(imagePoints, worldPoints);
        calibration.correctCamera();
        
    }
    
    if (!hasCornerPoints || !hasFingerPoints){
        bDrawProjector = true;
        ofSetFullscreen(true);
    }
    
    
    
    // load fonts
    
    verdana30.load("verdana.ttf", 175, true, true);
    verdana30.setLineHeight(135.0f);
    verdana30.setLetterSpacing(1.035);
    
    verdana14.load("verdana.ttf", 32, true, true);
    verdana14.setLineHeight(38);
    verdana14.setLetterSpacing(1.035);
    
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
		contourFinder.findContours(grayImage, minArea, maxArea, 20, false);
        
        // update finger point
        if (contourFinder.nBlobs > 0){
            
            hull = convexHull.getConvexHull(contourFinder.blobs[0].pts);

            // remove duplicate points
            vector<ofPoint> temp;
            for (int i=0; i<hull.size()-1; i++){
                
                auto pt00 = hull[i];
                auto pt01 = hull[i+1];
                
                float dist = 15;
                if (pt00.squareDistance(pt01) > dist*dist){
                    temp.push_back(pt00);
                    i++;
                }
            }
            
            hull = temp;
            
            // assign finger point
            fingerPt2D = hull[1];
            fingerPt = kinect.getWorldCoordinateAt(fingerPt2D.x, fingerPt2D.y);
            
        }
        
        
        checkForTouch();
        
        
	}
    
    mouse.x = mouseX;
    mouse.y = mouseY;
	
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
    }else if (bDrawProjector){
        
        ofPushStyle();
        ofBackground(0);
        
        // draw corner points
//        if (hasCornerPoints){'
        
        if (showWarning){
            
            ofPushMatrix();
            ofRotate(180);
            ofTranslate(-centroid);
            drawWarning();
            ofPopMatrix();
        }
        else if (showEnergy){
            ofPushMatrix();
            ofRotate(-90);
            ofTranslate(-centroid.y, centroid.x);
            drawEnergy();
            ofPopMatrix();
            
            drawFingers();
        }
        else{
            ofSetLineWidth(5);
            ofSetColor(ofColor::orangeRed);
//            ofNoFill();
            ofBeginShape();
            for (auto &pt : cornerPts){
                ofVertex(pt.x, pt.y);
            }
            ofEndShape(OF_CLOSE);
            
        
            ofSetLineWidth(0);
            ofFill();
            for (auto &pt : cornerPts){
                
                ofSetColor(255);
                ofDrawCircle(pt, 10);
                
                if (pt.squareDistance(mouse) < 25*25){
                    ofSetColor(0,255,255,100);
                    ofDrawCircle(pt, 25);
                }
            }

        }
        
        // draw mouse cross hairs
        ofSetLineWidth(5);
        ofSetColor(ofColor::white);
        ofDrawLine(mouseX, 0, mouseX, ofGetHeight());
        ofDrawLine(0, mouseY, ofGetWidth(), mouseY);
        ofSetColor(ofColor::aqua);
        ofDrawCircle(mouseX, mouseY, 8);
        ofSetColor(ofColor::white);
        ofDrawCircle(mouseX, mouseY, 3);
        
        // draw text feedback
        stringstream ss;
        ss << "Sceen Pt: {" << ofToString(mouseX) << ", " << ofToString(mouseY) << "}\n" <<
            "World Pt: {" << ofToString(fingerPt) << "}\n\n" <<
            "Calibration Point Count: " << calibCount
        ;
        
        ofDrawBitmapString(ss.str(), 10, 10);
        
        ofPopStyle();
        
    }
    else {
		// draw from the live kinect
		kinect.drawDepth(10, 10, kinect.width, kinect.height);
        
        // draw the 2D workspace
        drawWorkspace(false);
        
        if(hasTouch){
            ofPushMatrix();
//            ofPushStyle();
            ofTranslate(10,10);
            for (auto &index : touchIndices)
                contourFinder.blobs[index].draw();
//            ofPopStyle();
            
            ofPopMatrix();
        }
        
        
		kinect.draw(kinect.width + 20, 10, kinect.width, kinect.height);
		
		grayImage.draw(kinect.width + 20, kinect.height + 20, kinect.width, kinect.height);
        
        
		contourFinder.draw(kinect.width + 20, kinect.height + 20, kinect.width, kinect.height);
        
        ofPushMatrix();
        ofPushStyle();
        ofNoFill();
        ofSetLineWidth(3);
        ofSetColor(ofColor::aqua);
        ofTranslate(kinect.width + 20, kinect.height + 20);
        ofBeginShape();
        for (auto &pt : hull)
            ofVertex(pt);
        ofEndShape();
        
        ofSetColor(ofColor::white);
        for (int i=0; i<hull.size(); i++){
            ofDrawBitmapString(ofToString(i), hull[i].x + 5, hull[i].y + 5);
        }
        
        ofSetColor(ofColor::magenta, 120);
        ofDrawCircle(fingerPt2D, 10);
   
        ofPopStyle();
        ofPopMatrix();
		
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
    
    if (!bDrawProjector){
        panelTouch.draw();
        panelCV.draw();
    }
}

//--------------------------------------------------------------
void ofApp::drawWarning(){
    
    float alpha;
    
    alphaIncrement += .05;
    
    alpha = ofMap(sin(alphaIncrement),0,1,0,255);
    
    ofPushStyle();
    ofSetColor(ofColor::yellow, alpha);
    ofDrawCircle(0, 0, 100);
    ofNoFill();
    ofDrawCircle(0, 0, 100);
    
    ofSetColor(0);
    verdana30.drawString("!", -45, 85);
    
    ofPopStyle();
    
}

//--------------------------------------------------------------
void ofApp::drawEnergy(){
    
    float alpha;
    
    alphaIncrement += .005;
    
    alpha = ofMap(sin(alphaIncrement),-1,1,0,101);
    
    string percent = ofToString(int(alpha)) + "%";
    
//    if (alpha < 4*255/4.0 && alpha > 3*255/4.0)
//        percent = "100%";
//    else if (alpha < 3*255/4.0 && alpha > 2*255/4.0)
//        percent = " 75%";
//    else if (alpha < 2*255/4.0 && alpha > 1*255/4.0)
//        percent = " 50%";
//    else
//        percent = " 25%";
    
    ofPushStyle();
    ofFill();
    ofSetColor(ofColor::white);
    int w = 150;
    int h = w/2;
    ofDrawRectangle(-w/2,-h/2, w,   h);
    ofDrawRectangle(-3*w/7, -6*h/7, w/4, w/8);
    ofDrawRectangle( w/6, -6*h/7, w/4, w/8);
//    ofNoFill();
//    ofDrawCircle(0, 0, 100);
    
    ofSetColor(0);
    verdana14.drawString(percent, -w/2+15, w/8);

    ofPopStyle();
    
    
}

void ofApp::drawFingers(){
    
    ofPushStyle();

    // draw fingers
    
    int alpha = int(ofMap(sin(alphaIncrement),-1,1,0,100));
    
    int c = 0.0;
    
    ofFill();
    for (auto &ptList : fingerPts){
       
        if (c != 0){
            
            int a = 255;
            if (c == 1)
                a = ofMap(alpha, 0, 25, 0, 255, true);
            if ( c == 2)
                a = ofMap(alpha, 0, 50, 0, 255, true);
            if (c == 3)
               a = ofMap(alpha, 0, 75, 0, 255, true);
            if (c == 4)
                a = ofMap(alpha, 0, 100, 0, 255, true);
            
            ofSetColor(ofColor::greenYellow, a);//alpha);
            
            ofBeginShape();
            for (auto &pt : ptList)
                ofVertex(pt.x,pt.y);
            
            ofEndShape(OF_CLOSE);
        }
        
        c++;
    }
    
    
    
    ofSetLineWidth(0);
    ofFill();
    for (auto &ptList : fingerPts){
        for (auto &pt : ptList){
            
//            ofSetColor(255);
//            ofDrawCircle(pt, 10);
            float dist = 5;
            if (pt.squareDistance(mouse) < dist*dist){
                ofSetColor(0,255,255,100);
                ofDrawCircle(pt, dist*2);
            }
        }
    }
    
    ofPopStyle();

}

//--------------------------------------------------------------
void ofApp::setupGUI(){
    
    
    paramsTouch.setName("3D Touch Parameters");
    paramsTouch.add(interactionZoneHeight.set("Zone Height", 50, 1, 500));
    paramsTouch.add(zOffset.set("z Offset", 0, -50, 50));
    
    interactionZoneHeight.addListener(this, &ofApp::updateInteractionZone);
    zOffset.addListener(this, &ofApp::updateZOffset);
    
    panelTouch.setDefaultWidth(500);
    panelCV.setDefaultWidth(500);
    
    panelTouch.setup(paramsTouch);
    
    panelTouch.setPosition(10, kinect.height + 20);
    
    
    panelTouch.loadFromFile("settings_touch.xml");
    
    paramsCV.setName("CV Parameters");
    paramsCV.add(nearThreshold.set("Near Threshold", 255, 0, 255));
    paramsCV.add(farThreshold.set("Far Threshold", 234, 0, 255));
    paramsCV.add(minArea.set("Min Area", 1500, 0, 1500));
    paramsCV.add(maxArea.set("Max Area", 15000, 0, 50000));
    
    panelCV.setup(paramsCV);
    panelCV.setPosition(10, panelTouch.getPosition().y + panelTouch.getHeight()+10);
    
    panelCV.loadFromFile("settings_cv.xml");
}

void ofApp::checkForTouch(){
    
    hasTouch = false;
    touchIndices.clear();
    for (int i=0; i< contourFinder.nBlobs; i++){
        
        if (workspacePlane2D.inside(contourFinder.blobs[i].centroid)){
            hasTouch = true;
            touchIndices.push_back(i);
            
        }
        
    }

    
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
    
    
    ofNoFill();
    ofDrawBox(topCentroid, 10);
    ofDrawBox(btmCentroid, 10);
    
    ofSetColor(255,255,0);
    ofDrawBox(baseCentroid, 20);
    
    if (hasTouch){
        ofFill();
        for (auto &index : touchIndices){
            ofPoint pt = contourFinder.blobs[index].centroid;
            ofDrawBox(kinect.getWorldCoordinateAt(pt.x, pt.y), 10);
        }
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
            
                if (kinect.getWorldCoordinateAt(x, y).z < topCentroid.z &&
                    kinect.getWorldCoordinateAt(x, y).z > btmCentroid.z)
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
    
    
    // set top and btm centroids
    btmCentroid = ( interactionZone.getVertices()[0] + interactionZone.getVertices()[1] + interactionZone.getVertices()[2] + interactionZone.getVertices()[3] ) /4;
    topCentroid = (interactionZone.getVertices()[4] + interactionZone.getVertices()[5] + interactionZone.getVertices()[6] + interactionZone.getVertices()[7])  /4;
    
    baseCentroid.set(btmCentroid.x, btmCentroid.y, btmCentroid.z);
}

//--------------------------------------------------------------------------
void ofApp::updateInteractionZone(float &height){
    
    if (!interactionZone.getVertices().empty()){
        
        // update normal length
        interactionZone.getNormals()[0].scale(1).scale(height);
        interactionZone.getNormals()[1].scale(1).scale(height);
        interactionZone.getNormals()[2].scale(1).scale(height);
        interactionZone.getNormals()[3].scale(1).scale(height);
        
        // reset vertices to bottom plane
        interactionZone.getVertices()[4] = interactionZone.getVertices()[0];
        interactionZone.getVertices()[5] = interactionZone.getVertices()[1];
        interactionZone.getVertices()[6] = interactionZone.getVertices()[2];
        interactionZone.getVertices()[7] = interactionZone.getVertices()[3];
        
        // update vertices by new height offset
        interactionZone.getVertices()[4] += interactionZone.getNormals()[0]; // 4
        interactionZone.getVertices()[5] += interactionZone.getNormals()[1]; // 5
        interactionZone.getVertices()[6] += interactionZone.getNormals()[2]; // 6
        interactionZone.getVertices()[7] += interactionZone.getNormals()[3]; // 7
        
        // set top and btm centroids
        topCentroid = ( interactionZone.getVertices()[0] + interactionZone.getVertices()[1] + interactionZone.getVertices()[2] + interactionZone.getVertices()[3] ) /4;
        btmCentroid = (interactionZone.getVertices()[4] + interactionZone.getVertices()[5] + interactionZone.getVertices()[6] + interactionZone.getVertices()[7])  /4;
        
        
    }
    
}

//--------------------------------------------------------------------------
void ofApp::updateZOffset(float &offset){
    
    if (interactionZone.getVertices().size() > 0){
        
        float diff = offset - prevOffset;
        
        for (auto &v : interactionZone.getVertices()){
            v.z += diff;
        }
        
        // set top and btm centroids
        topCentroid = ( interactionZone.getVertices()[0] + interactionZone.getVertices()[1] + interactionZone.getVertices()[2] + interactionZone.getVertices()[3] ) /4;
        btmCentroid = (interactionZone.getVertices()[4] + interactionZone.getVertices()[5] + interactionZone.getVertices()[6] + interactionZone.getVertices()[7])  /4;
        
        prevOffset=offset;

    }

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
void ofApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
    
    panelCV.saveToFile("settings_cv.xml");
    panelTouch.saveToFile("settings_touch.xml");
	
#ifdef USE_TWO_KINECTS
	kinect2.close();
#endif
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
	switch (key) {
		case ' ':
			//bThreshWithOpenCV = !bThreshWithOpenCV;
            
            bDrawProjector = !bDrawProjector;
            ofSetFullscreen(bDrawProjector);
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
			
//		case 'w':
//			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
//			break;
			
		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;
			
        case 'w':
            showWarning = !showWarning;
            break;
        case 'e':
            showEnergy = !showEnergy;
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
    
    if (hasCornerPoints){
        
        ofVec2f m;
        m.x = x; m.y = y;
        
        float dist = 25;
        centroid.set(0,0);
        for (int i=0; i<cornerPts.size(); i++){
            if (cornerPts[i].squareDistance(m) < dist*dist){
                cornerPts[i].set(m);
            }
            centroid += cornerPts[i];
        }
        centroid /= cornerPts.size();
    
    }
    if (showEnergy){
        
        ofVec2f m;
        m.x = x; m.y = y;
        
        float dist = 10;
        for (auto &ptList : fingerPts){
            for (auto &pt : ptList){
                if (pt.squareDistance(m) < dist*dist)
                    pt.set(m);
            }
                
        }
        
    }
    


    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{

    if (!hasCornerPoints){
        
        cornerPts.push_back(ofVec2f(x,y));
        
    
        if (cornerPts.size() == 4){
            hasCornerPoints = true;
            
            centroid.set(0,0);
            
            for (int i=0; i<cornerPts.size(); i++) centroid += cornerPts[i];
            centroid /= cornerPts.size();
        }
    }
    
    if (!hasFingerPoints){
        
        if (fingerPtCount % 4 == 0){
            vector<ofVec2f> pts;
            fingerPts.push_back(pts);
        }
        
        cout << "number of fingers: " << ofToString(fingerPts.size()) << endl;
        
        ofVec2f pt;
        pt.x = x;
        pt.y = y;
        
        fingerPts[fingerPts.size()-1].push_back(pt);
        cout << "number of points per finger: " << ofToString(fingerPts[fingerPts.size()-1].size()) << endl;
        cout << "pt: " << ofToString(pt) << endl;
        fingerPtCount ++;
        
        vector<ofVec2f> list = fingerPts[fingerPts.size()-1];
        cout << "pt: " << ofToString(list[list.size()-1]) << endl;
        
        if (fingerPtCount == 5*4)
            hasFingerPoints = true;
    }
    
//    else if (hasCornerPoints){
//        
//        ofVec2f m;
//        m.x = x; m.y = y;
//        
//        float dist = 25;
//        for (int i=0; i<cornerPts.size(); i++){
//            if (cornerPts[i].squareDistance(m) < dist*dist){
//                cornerPts[i].set(m);
//            }
//        }
//    }
    
    
    
    if (!isCalibrated){
        imagePoints.push_back(ofVec2f(mouseX,mouseY));
        worldPoints.push_back(ofVec3f(fingerPt.x,fingerPt.y,fingerPt.z));
        calibCount++;
        
        // save out file
        if (calibCount == 40){
            
            
            imagePts.open("imagePts.txt",ofFile::WriteOnly);
            worldPts.open("worldPts.txt",ofFile::WriteOnly);
            
            for (int i=0; i<calibCount; i++){
                
                imagePts << ofToString(imagePoints[i].x) << ", " << ofToString(imagePoints[i].y) << endl;
                
                worldPts << ofToString(worldPoints[i].x) << ", " << ofToString(worldPoints[i].y) << ", " << ofToString(worldPoints[i].z) << endl;
            
            }

            imagePts.close();
            worldPts.close();
            
            isCalibrated = true;
            
            bDrawProjector = false;
            ofSetFullscreen(false);
        }
    }
    
    
    
    
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{

    if (!isWorkspaceDefined && isCalibrated){
        
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
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{

}

void ofApp::readCalibrationFiles(string image, string world){
    
    imagePoints.clear();
    worldPoints.clear();
   
    // process the images points first
    ofBuffer buffer = ofBufferFromFile(image);
    
    if(buffer.size()) {
        
        for (ofBuffer::Line it = buffer.getLines().begin(), end = buffer.getLines().end(); it != end; ++it) {
            
            string line = *it;
            
            // copy the line to draw later
            // make sure its not a empty line
            if(line.empty() == false) {
                
                float x = ofToFloat(ofSplitString(line, ", ")[0]);
                float y = ofToFloat(ofSplitString(line, ", ")[1]);
                
                imagePoints.push_back(ofVec2f(x,y));

                // print out the line
                cout << "image point: " << line << endl;
            }
            
            
        }
        
    }
    
    // the process the world points
    buffer = ofBufferFromFile(world);
    
    if(buffer.size()) {
        
        for (ofBuffer::Line it = buffer.getLines().begin(), end = buffer.getLines().end(); it != end; ++it) {
            
            string line = *it;
            
            // copy the line to draw later
            // make sure its not a empty line
            if(line.empty() == false) {
                
                float x = ofToFloat(ofSplitString(line, ", ")[0]);
                float y = ofToFloat(ofSplitString(line, ", ")[1]);
                float z = ofToFloat(ofSplitString(line, ", ")[2]);
                
                worldPoints.push_back(ofVec3f(x,y,z));

                // print out the line
                cout << "world point: " << line << endl;
            }
            
            
        }
        
    }
    
    isCalibrated = true;
}
