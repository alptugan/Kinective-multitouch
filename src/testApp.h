#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxCvKalman.h"
#include "TuioServer.h"
#include "ofxUI.h"

using namespace TUIO;

class testApp : public ofBaseApp {
public:
	
	void setup();
    void update();
    void draw();
    void exit();
    
    void keyPressed  (int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void windowResized(int w, int h);
    
    ofPoint mapTUIO(ofPoint pos) {
        
        ofPoint _pos;
        _pos.x = (pos.x - tl.x) / (br.x - tl.x);
        _pos.y = (pos.y - tl.y) / (br.y - tl.y);
        
        return _pos;
    };
    
    
    
        
    ofxKinect kinect;
    
    ofxCvGrayscaleImage	grayImage;
    ofxCvGrayscaleImage touchSurfaceImage;
    
    ofxCvContourFinder 	contourFinder;
    TuioServer *tuioServer;
    
    int kinectYOffset;
    int minBlobSize;
    int maxBlobSize;
    
    int xBlobPos;
    int yBlobPos;
    
    // GUI
    ofxUICanvas *gui;   	
	void guiEvent(ofxUIEventArgs &e);
    bool drawPadding; 
    ofImage kinectIcn;
    
    
    //Calibration
    ofPoint tl;
    ofPoint br;
    
    float xPosRatio;
    float yPosRatio;
    
    //Mapping
    bool tlSelected;
    bool brSelected;
};
