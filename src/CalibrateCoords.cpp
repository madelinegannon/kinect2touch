#include "CalibrateCoords.h"
#include "ofxXmlSettings.h"


void CalibrateCoords::setup(int camWidth, int camHeight){
    //  cameraWidth		= 1024;
    //  cameraHeight	= 768;
    resolution.set(camWidth,camHeight);
    calibrated				= false;
    hasFingerCalibPoints	= false;
    switchYandZ				= false;
    
    throwRatioX		= 1.6f;
    throwRatioY		= 1.6f;
    lensOffsetX		= 0.0f;
    lensOffsetY		= -0.5f;
    translationX	= 0.0f;
    translationY	= 0.0f;
    translationZ	= 0.0f;
    rotationX		= 0.0f;
    rotationY		= 0.0f;
    rotationZ		= 0.0f;
    throwRatio		= 1.62f;
    lensOffset		= ofVec2f(0.0f,0.5f);
    
    projector.setFarClip(1000.0f);
    projector.setNearClip(10.0f);
    projector.setWidth(resolution.x);
    projector.setHeight(resolution.y);
    
    
    resetProjector();
    
    dirNameLoaded = "";
}

void CalibrateCoords::loadFingerTipPoints(string filePath){
    
    cout << "calibrate from " << filePath << endl;
    
    hasFingerCalibPoints = false;
    
    ofxXmlSettings XML;
    if( XML.loadFile(filePath) ){
        
        dirNameLoaded = filePath;
        
        calibVectorImage.clear();
        calibVectorWorld.clear();
        
        int totalCalibReads = XML.getNumTags("CALIB_READ");
        int readCount = 0;
        for(int i = 0; i < totalCalibReads; i++){
            
            XML.pushTag("CALIB_READ",i);
            float mx = XML.getValue("MOUSE:X", 0);
            float my = XML.getValue("MOUSE:Y", 0);
            float lx = XML.getValue("FINGER:X", 0);
            float ly = XML.getValue("FINGER:Y", 0);
            float lz = XML.getValue("FINGER:Z", 0);
            
            //cout << "mouse pt " << mx << " " << my << endl;
            //cout << "finger   " << lx << " " << ly << " " << lz << endl;
            
            calibVectorImage.push_back(ofVec2f(mx,my));
            
            if(switchYandZ){ calibVectorWorld.push_back(ofVec3f(lx,lz,ly));}
            else{ calibVectorWorld.push_back(ofVec3f(lx,ly,lz));}
            
            XML.popTag();
            
            if ((mx != 0) && (my != 0)) {
                readCount++;
            }
        }
        
        if (readCount > 0){
            hasFingerCalibPoints = true;
        }
    }
    
    
}

void CalibrateCoords::loadPoints(vector<ofVec2f> image, vector<ofVec3f> world){
    calibVectorImage.clear();
    calibVectorWorld.clear();
    
    for (auto &pt : image){
        calibVectorImage.push_back(ofVec2f(pt.x,pt.y));
    }
    
    for (auto &pt : world){
        if(switchYandZ){ calibVectorWorld.push_back(ofVec3f(pt.x,pt.z,pt.y));}
        else{ calibVectorWorld.push_back(ofVec3f(pt.x,pt.y,pt.z));}
    }

    hasFingerCalibPoints = true;
}

void CalibrateCoords::correctCameraPNP (ofxCv::Calibration & myCalibration){
    
    vector<cv::Point2f> imagePoints;
    vector<cv::Point3f> worldPoints;
    
    if( hasFingerCalibPoints ){
        for (int i=0; i<calibVectorImage.size(); i++){
            imagePoints.push_back(ofxCv::toCv(calibVectorImage[i]));
            worldPoints.push_back(ofxCv::toCv(calibVectorWorld[i]));
        }
    }
    else{
        cout << "not enough control points" << endl;
        return;
    }
    
    cout << "myCalibration.getDistCoeffs() " << myCalibration.getDistCoeffs() << endl;
    cout << "myCalibration.getUndistortedIntrinsics().getCameraMatrix() " << myCalibration.getUndistortedIntrinsics().getCameraMatrix() << endl;
    
    cv::Mat rvec(3,1,cv::DataType<double>::type);
    cv::Mat tvec(3,1,cv::DataType<double>::type);
    
    cv::solvePnP(worldPoints,imagePoints,
                 myCalibration.getUndistortedIntrinsics().getCameraMatrix(), myCalibration.getDistCoeffs(),
                 rvec, tvec, false );
    
    
    //     solvePnP( InputArray objectPoints, InputArray imagePoints,
    //     InputArray cameraMatrix, InputArray distCoeffs,
    //     OutputArray rvec, OutputArray tvec,
    //     bool useExtrinsicGuess=false );
    
    
    calibrated = true;
    
    setExtrinsics(rvec, tvec);
    setIntrinsics(myCalibration.getUndistortedIntrinsics().getCameraMatrix());
    
    this->camera = myCalibration.getUndistortedIntrinsics().getCameraMatrix();
    
    cv::Mat distortionCoefficients = cv::Mat::zeros(5, 1, CV_64F);
    this->distortion = distortionCoefficients;
    
    
    this->rotation = rvec;
    this->translation = tvec;
    
    cout << "camera:\n";
    cout << this->camera << "\n";
    cout << "RVEC:\n";
    cout << rvec << "\n";
    cout << "TVEC:\n";
    cout << tvec << "\n";
    cout << "--------\n";
    
}

void CalibrateCoords::correctCamera(){
    
    //we have to intitialise a basic camera matrix for it to start with (this will get changed by the function call calibrateCamera
    
    vector<cv::Point2f> imagePoints;
    vector<cv::Point3f> worldPoints;
    
    if( hasFingerCalibPoints ){
        for(int i = 0; i < calibVectorImage.size(); i++){
            imagePoints.push_back(ofxCv::toCv(calibVectorImage[i]));
            worldPoints.push_back(ofxCv::toCv(calibVectorWorld[i]));
        }
    }
    else{
        cout << "not enough control points" << endl;
        return;
    }
    
    
    //initialise matrices
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0,0) = resolution.x * 1.62f; // default at 1.4 : 1.0f throw ratio
    cameraMatrix.at<double>(1,1) = resolution.y * 1.62f;
    cameraMatrix.at<double>(0,2) = resolution.x / 2.0f;
    cameraMatrix.at<double>(1,2) = resolution.y * 0.90f; // default at 40% lens offset
    cameraMatrix.at<double>(2,2) = 1;
    cv::Mat distortionCoefficients = cv::Mat::zeros(5, 1, CV_64F);
    
    vector<cv::Mat> rotations, translations;
    
    int flags = CV_CALIB_FIX_K1 | CV_CALIB_FIX_K2 | CV_CALIB_FIX_K3 | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5 | CV_CALIB_FIX_K6 |CV_CALIB_ZERO_TANGENT_DIST | CV_CALIB_USE_INTRINSIC_GUESS;
    
    
    float error = cv::calibrateCamera(vector<vector<cv::Point3f> >(1, worldPoints),
                                      vector<vector<cv::Point2f> >(1, imagePoints),
                                      cv::Size(resolution.x, resolution.y),
                                      cameraMatrix,
                                      distortionCoefficients,
                                      rotations,
                                      translations,
                                      flags);
    
    cout << " cameraMatrix " << cameraMatrix << endl;
    
    calibrated = true;
    
    setExtrinsics(rotations[0], translations[0]);
    setIntrinsics(cameraMatrix);
    
    this->camera = cameraMatrix;
    this->distortion = distortionCoefficients;
    this->rotation = rotations[0];
    this->translation = translations[0];
    
}

void CalibrateCoords::setIntrinsics(cv::Mat cameraMatrix)
{
    float fovx = cameraMatrix.at<double>(0, 0);
    float fovy = cameraMatrix.at<double>(1, 1);
    float ppx = cameraMatrix.at<double>(0, 2);
    float ppy = cameraMatrix.at<double>(1, 2);
    
    throwRatioX = fovx / resolution.x;
    throwRatioY = fovy / resolution.y;
    
    lensOffsetX = (ppx /  resolution.x) - 0.5f; // not sure if this is + or -ve (if wrong, then both this and
    // ofxCvMin::Helpers::makeProjectionMatrix should be switched
    lensOffsetY = (ppy /  resolution.y) - 0.5f;
    
    const auto newProjection = makeProjectionMatrix(cameraMatrix, cv::Size(resolution.x, resolution.y) );
    projector.setProjection(newProjection);
    
    cout << "intrinsic projection matrix: \n" << ofToString(newProjection) << endl;
}

// from ofxCvMin
ofMatrix4x4 CalibrateCoords::makeProjectionMatrix(cv::Mat cameraMatrix, cv::Size imageSize) {
    float focalLengthX = cameraMatrix.at<double>(0, 0);
    float focalLengthY = cameraMatrix.at<double>(1, 1);
    float ppx = cameraMatrix.at<double>(0, 2);
    float ppy = cameraMatrix.at<double>(1, 2);
    
    ofMatrix4x4 projection;
    projection(0, 0) = 2.0f * focalLengthX / (float)imageSize.width;
    projection(1, 1) = -2.0f * focalLengthY / (float)imageSize.height;
    projection(2,3) = 1.0f;
    projection(3,3) = 0.0f;
    
    const ofVec3f lensOffset(2 * (ppx / (float) imageSize.width) - 1.0f, 1.0f - 2 * (ppy / (float) imageSize.height), 0.0f);
    projection.postMultTranslate(lensOffset);
    
    return projection;
}

// from ofxCvMin
ofMatrix4x4 CalibrateCoords::makeMatrix(cv::Mat rotation, cv::Mat translation) {
    cv::Mat rot3x3;
    if(rotation.rows == 3 && rotation.cols == 3) {
        rot3x3 = rotation;
    } else {
        Rodrigues(rotation, rot3x3);
    }
    double* rm = rot3x3.ptr<double>(0);
    double* tm = translation.ptr<double>(0);
    return ofMatrix4x4(rm[0], rm[3], rm[6], 0.0f,
                       rm[1], rm[4], rm[7], 0.0f,
                       rm[2], rm[5], rm[8], 0.0f,
                       tm[0], tm[1], tm[2], 1.0f);
}

//----------
void CalibrateCoords::setExtrinsics(cv::Mat rotation, cv::Mat translation)
{
    const auto rotationMatrix = makeMatrix(rotation, cv::Mat::zeros(3, 1, CV_64F));
    const auto rotationEuler = rotationMatrix.getRotate().getEuler();
    
    translationX = translation.at<double>(0);
    translationY = translation.at<double>(1);
    translationZ = translation.at<double>(2);
    
    rotationX = rotationEuler.x;
    rotationY = rotationEuler.y;
    rotationZ = rotationEuler.z;
    
    projector.setView(makeMatrix(rotation, translation));
}


void CalibrateCoords::resetProjector()
{
    //position = ofVec3f(1.0f,1.0f,1.0f);//cam.getPosition();
    throwRatio = 1.62f;
    lensOffset = ofVec2f(0.0f,0.5f);
    
    ofQuaternion rotation;
    auto rotationQuat = ofQuaternion(rotationX, ofVec3f(1, 0, 0), rotationZ, ofVec3f(0, 0, 1), rotationY, ofVec3f(0, 1, 0));
    ofMatrix4x4 pose = ofMatrix4x4(rotationQuat);
    pose(3,0) = translationX;
    pose(3,1) = translationY;
    pose(3,2) = translationZ;
    projector.setView(pose);
    
    ofMatrix4x4 projection;
    projection(0,0) = throwRatioX;
    projection(1,1) = -throwRatioY;
    projection(2,3) = 1.0f;
    projection(3,3) = 0.0f;
    projection.postMultTranslate(-lensOffsetX, -lensOffsetY, 0.0f);
    projector.setProjection(projection);
    
    projector.setWidth(resolution.x);
    projector.setHeight(resolution.y);
}


void CalibrateCoords::drawWorldPoints(){
    ofPushMatrix();
    ofSetColor(ofColor::red);
    for(int i = 0; i < calibVectorWorld.size(); i++){
        ofDrawSphere(calibVectorWorld[i], 5.0f);
    }
    ofPopMatrix();
}

void CalibrateCoords::drawImagePoints(){
    
    auto count = this->calibVectorImage.size();
    vector<cv::Point2f> evaluatedImagePoints(count);
    
    if (this->calibrated) {
        cv::projectPoints(ofxCv::toCv(this->calibVectorWorld), this->rotation, this->translation, this->camera, this->distortion, evaluatedImagePoints);
    }
    
    ofPushMatrix();
    ofSetColor(ofColor::white);
    for(int i = 0; i < calibVectorImage.size(); i++){
        ofDrawSphere(calibVectorImage[i], 5.0f);
        
        if (this->calibrated) {
            ofLine(calibVectorImage[i], ofxCv::toOf(evaluatedImagePoints[i]));
        }
    }
    ofPopMatrix();
}
