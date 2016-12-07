#pragma once

#include "ofMain.h"

class ofApp : public ofBaseApp{
    
public:
    void setup();
    void update();
    void draw();
    void exit();
    
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void mouseEntered(int x, int y);
    void mouseExited(int x, int y);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    
    // Device
    vector <ofSerialDeviceInfo> arduino;
    ofSerial serial;
    string currentDevice;
    
    // Data I/O
    string inputBuffer;
    vector<string> values;
    ofQuaternion quatValue;
    ofQuaternion offsetValue;
    
    // 3D model
    ofMesh mesh;
    ofBoxPrimitive box;
    ofEasyCam cam;
    vector<ofVec3f> angleDatas;
};

