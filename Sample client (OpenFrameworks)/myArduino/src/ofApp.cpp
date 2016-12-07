#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    
    ofSetVerticalSync(false);
    ofSetCircleResolution(30);
    
    // Arduino
    arduino = serial.getDeviceList();
    
    for(auto d : arduino){
        ofLog() << d.getDeviceName();
    }
    
    // Set up serial
    if( serial.setup(arduino[1].getDevicePath(), 115200) == true){
        currentDevice = arduino[1].getDeviceName();
    }
    else{
        currentDevice = "No device";
    }
    
    // Read 3D bunny
    mesh.load("lofi-bunny.ply");
    
    // Send message
    serial.writeByte(' ');

    // 3D cam setting
    cam.enableOrtho();
}

//--------------------------------------------------------------
void ofApp::update(){
    while (serial.available() > 0)
    {
        int myByte = serial.readByte(); // Read the byte.
        
        if (myByte != '\n'){            // End of line character.
            // If it's not the end of the line character.
            inputBuffer += (char)myByte;
            
        }else {
            // Split the data
            values.clear();
            values = ofSplitString(inputBuffer, ",");
            if(values.size() == 4){
                //set Quart
                quatValue.set(ofToFloat(values[0]) * 0.00001,        // Quertenion.x()
                              ofToFloat(values[1]) * 0.00001,        // Quertenion.y()
                              ofToFloat(values[2]) * 0.00001,        // Quertenion.z()
                              ofToFloat(values[3]) * 0.00001);       // Quartenion.scalar()
            }else{
                // Massage or Error
                ofLog() << inputBuffer;
            }
            inputBuffer = "";
        }
        serial.writeByte(' ');
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofBackgroundGradient(ofColor(64), ofColor(0));
    
    ofSetColor(255);
    
    float angle;
    ofVec3f axis;
    ofVec3f euler;
    ofQuaternion tmpQuart = quatValue * offsetValue.inverse();
    
    //Draw 3D model
    {
        ofPushMatrix();
        cam.begin();
        tmpQuart.getRotate(angle, axis);
        ofRotate(angle, -axis.y, axis.z, -axis.x);
        ofSetColor(ofColor::gray);
        
        ofTranslate(0, 0);
        
        ofPushStyle();
        ofSetColor(255, 145, 0,200);
        //mesh.drawWireframe();
        ofPopStyle();
        
        ofPushStyle();
        ofSetLineWidth(5);
        //box.drawAxes(200);
        ofPopStyle();
        
        glPointSize(3);
        ofSetColor(0,252,221,100);
        mesh.drawVertices();
        cam.end();
        ofPopMatrix();
    }
    
    //Draw angel info
    {
        ofQuaternion angleData = ofQuaternion(angle, ofVec3f(-axis.x, axis.z, -axis.x));
        
        euler = angleData.getEuler();
        
        if(ofGetWidth()*2 <= angleDatas.size()){
            angleDatas.clear();
        }else{
            angleDatas.push_back(euler);
        }
        
        ofPushMatrix();
        ofPushStyle();
        
        ofTranslate(0, ofGetHeight()/2);
        
        for(int i=1; i < angleDatas.size(); i++){
            if((i-1)%10 == 0){
                ofSetColor(255, 0, 0,200);
                ofDrawCircle((i-1)/2, angleDatas[i-1].x, 1);
                ofSetColor(0, 255 ,0,100);
                ofDrawCircle((i-1)/2, angleDatas[i-1].y, 1);
                ofSetColor(0, 0, 255,100);
                ofDrawCircle((i-1)/2, angleDatas[i-1].z, 1);
            }
        }
        
        ofPopStyle();
        ofPopMatrix();
    }
    
    //Draw debug info
    {
        ofPushMatrix();
        ofTranslate(0, 50);
        ofDrawBitmapStringHighlight(" Qart["+ofToString(tmpQuart)+"]", 20, 20);
        ofDrawBitmapStringHighlight("Euler["+ofToString(euler)+"]", 20, 50);
        ofDrawBitmapStringHighlight("  FPS["+ofToString(ofGetFrameRate())+"]", 20, 80);
        ofPopMatrix();
    }
}

void ofApp::exit(){
    serial.close();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    if(key == ' '){
        serial.writeByte(' ');
        ofLog() << "send";
    }else if(key == 's'){
        offsetValue = quatValue;
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){
    
}
