#include "testApp.h"
ofxCvKalman *tuioPointSmoothed[32];

int maxIxDThreshold = 2000;
int minIxDThreshold = 0;

TuioPoint updateKalman(int id, TuioPoint tp) {
	if (id>=16) return NULL;
	if(tuioPointSmoothed[id*2] == NULL) {
		tuioPointSmoothed[id*2] = new ofxCvKalman(tp.getX());
		tuioPointSmoothed[id*2+1] = new ofxCvKalman(tp.getY());
	} else {
		tp.update(tuioPointSmoothed[id*2]->correct(tp.getX()),tuioPointSmoothed[id*2+1]->correct(tp.getY()));
	}
    
	return tp;
}

void clearKalman(int id) {
	if (id>=16) return;
	if(tuioPointSmoothed[id*2]) {
		delete tuioPointSmoothed[id*2];
		tuioPointSmoothed[id*2] = NULL;
		delete tuioPointSmoothed[id*2+1];
		tuioPointSmoothed[id*2+1] = NULL;
	}
}


//--------------------------------------------------------------
void testApp::setup()
{
	ofSetWindowTitle("Kinective beta by Filika");
	
    kinectIcn.loadImage("kinect.png");
    
	kinect.init();
    
	kinect.open();
    
    kinectYOffset = 240;
    tlSelected = false;
    brSelected = false;
    tl.set(0,0);
    br.set(640,maxIxDThreshold);
    
	grayImage.allocate(kinect.width, kinect.height);
    touchSurfaceImage.allocate(640,maxIxDThreshold);
	
	
    
    minBlobSize   = 1;
    maxBlobSize   = (kinect.width*kinect.height)/8;

    
	TuioTime::initSession();	
	tuioServer = new TuioServer();
	tuioServer->setSourceName("TuioKinect");
	tuioServer->enableObjectProfile(false);
	tuioServer->enableBlobProfile(false);
	
	for (int i=0;i<32;i++)
		tuioPointSmoothed[i] = NULL;
    
    
    // GUI
    ofSetVerticalSync(true); 
	ofEnableSmoothing();
	
    
    
	float dim = 24; 
	float xInit = OFX_UI_GLOBAL_WIDGET_SPACING; 
    float length = 320-xInit; 
	
    drawPadding = false; 
    
    gui = new ofxUICanvas(0, 0, 720, 400);
    gui->setFontSize(OFX_UI_FONT_SMALL, 5);
  /*  gui->setColorFill(0x333333);
    gui->setColorBack(0x333333);*/
    gui->addWidgetDown(new ofxUISlider("Y OFFSET", 0, 480, kinectYOffset, 15, 160));
    gui->addWidgetDown(new ofxUISlider("MIN BLOB SIZE", 0, 6000, minBlobSize,length+xInit+20, 15));
    gui->addWidgetDown(new ofxUISlider("MAX BLOB SIZE", 0, 6000, maxBlobSize,length+xInit+20, 15));
    gui->addWidgetDown(new ofxUISpacer(length-xInit, 1)); 
    gui->addWidgetDown(new ofxUIButton("CALIBRATION", false, 20, 20));
    
    gui->addWidgetDown(new ofxUILabel("FPS : ", OFX_UI_FONT_SMALL));
    gui->addWidgetRight(new ofxUIFPS(OFX_UI_FONT_SMALL)); 
    
    //gui->loadSettings("GUI/guiSettings.xml");
    
    ofAddListener(gui->newGUIEvent,this,&testApp::guiEvent);	

    
	ofSetFrameRate(30);
}


//--------------------------------------------------------------
void testApp::update()
{
	ofBackground(100, 100, 100);
	kinect.update();
    
    // Depth Image From Gray image Top Left
	unsigned char * pix = grayImage.getPixels();
	int numPixels = grayImage.getWidth() * grayImage.getHeight()-1;

    touchSurfaceImage.set(640*maxIxDThreshold);
    
    unsigned char * touchPix = touchSurfaceImage.getPixels();
    int numTouchPixels = 640*maxIxDThreshold -1;
    
   
    
    for(int j = numTouchPixels; j > 0 ; j--){
        
        touchPix[j] = 0;
	}
   
    

    
    int po = 640*kinectYOffset;
    int po2 = 640 * (kinectYOffset+2); // Thickness of the range
    ofVec3f dist;
    
    for(int i = numPixels; i > 0 ; i--)
    {
		if(i >  po && i < po2){
            
            
            int tRow = kinectYOffset;
            int tCol = i - kinect.width * (kinectYOffset-1);
                
            dist = kinect.getWorldCoordinateAt(tCol,tRow);
            //  cout << "tDepth : " << dist.z << endl;
                
            int tDepth = dist.z ;
            
            if(tDepth > minIxDThreshold && tDepth < maxIxDThreshold)
            {
                //touchPix[tDepth*kinect.width+tCol] = 255;
                for (int j=0; j < 10; j++) {
                    touchPix[(tDepth*kinect.width+tCol)+j*kinect.width] = 255;
                }
            }
            
            pix[i] = 255;
            
		}else{
			pix[i] = 0;
		}
        
	}
    
    
    touchSurfaceImage.setFromPixels(touchPix,640,maxIxDThreshold);
	
	//update the cv image
	grayImage.flagImageChanged();
    

    contourFinder.findContours(touchSurfaceImage, minBlobSize, maxBlobSize, 20, false);
    
    
    
    
	TuioTime frameTime = TuioTime::getSessionTime();
	tuioServer->initFrame(frameTime);
	
	std::vector<ofxCvBlob>::iterator blob;
    
	for (blob=contourFinder.blobs.begin(); blob!= contourFinder.blobs.end(); blob++) {
		float xpos = (*blob).centroid.x;
		float ypos = (*blob).centroid.y;
        
        ofPoint ko(0,0);
        TuioPoint tp(0,0);
    
        if(tlSelected == true && brSelected == true)
        {
            ko =  mapTUIO(ofPoint(xpos/640,1.f-(ypos/maxIxDThreshold)));
            cout << "x y : " << ko.x << endl;
            tp.update(ko.x*640,ko.y*maxIxDThreshold);
        }else{
            
            tp.update(xpos/640,1.f-(ypos/maxIxDThreshold));
        }
                
		
		
		//if ((tp.getY() > 0.8) && (tp.getX()>0.25) && (tp.getX()<0.75)) continue;
		
		TuioCursor *tcur = tuioServer->getClosestTuioCursor(tp.getX(),tp.getY());
        
		if ((tcur==NULL) || (tcur->getDistance(&tp)>0.2)) { 
			tcur = tuioServer->addTuioCursor(tp.getX(), tp.getY());
			updateKalman(tcur->getCursorID(),tcur);
		} else {
			TuioPoint kp = updateKalman(tcur->getCursorID(),tp);
			tuioServer->updateTuioCursor(tcur, kp.getX(), kp.getY());
		}
	}
    
	tuioServer->stopUntouchedMovingCursors();
	
	std::list<TuioCursor*> dead_cursor_list = tuioServer->getUntouchedCursors();
	std::list<TuioCursor*>::iterator dead_cursor;
	for (dead_cursor=dead_cursor_list.begin(); dead_cursor!= dead_cursor_list.end(); dead_cursor++) {
		clearKalman((*dead_cursor)->getCursorID());
	}
	
	tuioServer->removeUntouchedStoppedCursors();
	tuioServer->commitFrame();
}

//--------------------------------------------------------------
void testApp::draw()
{
	ofSetColor(255, 255, 255);
    
    // Kinect Y Offset 
	grayImage.draw(30, 8, 320, 160);
	ofEnableAlphaBlending();
    ofSetColor(255);
	kinectIcn.draw(90,28);
	ofDisableAlphaBlending();
    
    ofRect(540, 8, 160, 355);
    touchSurfaceImage.draw(370, 8,160,355);
    contourFinder.draw(370, 8, 160, 355);
    
	ofSetColor(255, 255, 255);
	
	std::list<TuioCursor*> alive_cursor_list = tuioServer->getTuioCursors();
	std::list<TuioCursor*>::iterator alive_cursor;
    
	for (alive_cursor=alive_cursor_list.begin(); alive_cursor!= alive_cursor_list.end(); alive_cursor++) {
       // cout << "find blob" << endl;
		
        TuioCursor *ac = (*alive_cursor);
		
		
        
        
        if(ac->getX()>0.f && ac->getY()>0.f )
        {
            xBlobPos = 540+ac->getX()*160;
            yBlobPos = 8+ac->getY()*355;
            
            xPosRatio = ac->getX();
            yPosRatio = ac->getY();
        }
            
            
    
        
        // Draw circle blobs
		ofSetColor(0, 0, 0);
		ofCircle(xBlobPos,yBlobPos ,5);
		char idStr[32];
		sprintf(idStr,"%d\n x:%f\n y:%f",ac->getCursorID(),xPosRatio,yPosRatio);
		ofDrawBitmapString(idStr, xBlobPos, yBlobPos+20);
        
        
        ofPoint coords =  kinect.getWorldCoordinateAt(xBlobPos, yBlobPos);
        
        char reportStr[1024];
        
        sprintf(reportStr, "Number Of Blobs: %i \n, fps: %f",(int)contourFinder.blobs.size(), (float)ofGetFrameRate());
        
        ofDrawBitmapString(reportStr, 20, 650);
		
	}
	
	ofSetColor(255, 255, 255);
	
    

}

//--------------------------------------------------------------
void testApp::exit(){
    gui->saveSettings("GUI/guiSettings.xml"); 
    delete gui;
   
	

	kinect.close();
}

//--------------------------------------------------------------
void testApp::keyPressed (int key)
{
    //q : Top left Corner
    //w : Bottom Right Corner
    if((int)contourFinder.blobs.size() == 1)
    {
        if(key == 'q'){
            //ofSetWindowShape(300,300);
            tlSelected = true;
            tl.set(xPosRatio, yPosRatio);
            cout << "Top Left :  " << tl << endl; 
        }
        else if(key == 'w'){
            brSelected = true;
            br.set(xPosRatio, yPosRatio);
            cout << "Bottom Right :  " << br << endl; 
        }
        
        
    }
}

void testApp::guiEvent(ofxUIEventArgs &e)
{
	string name = e.widget->getName(); 
	int kind = e.widget->getKind(); 
	
	if(name == "Y OFFSET")
	{
		ofxUISlider *slider = (ofxUISlider *) e.widget; 
		kinectYOffset = kinect.height - slider->getScaledValue(); 
        cout << "kinectYOffset value: " << slider->getScaledValue() << endl; 
	}else if(name == "MIN BLOB SIZE")
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget; 
		minBlobSize = slider->getScaledValue(); 
        cout << "value: " << slider->getScaledValue() << endl; 
    }else if(name == "MAX BLOB SIZE")
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget; 
		maxBlobSize = slider->getScaledValue(); 
        cout << "value: " << slider->getScaledValue() << endl; 
    }else if(name == "MIN IxD THRESHOLD")
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget; 
		minIxDThreshold = slider->getScaledValue(); 
        cout << "value: " << slider->getScaledValue() << endl; 
    }else if(name == "MAX IxD THRESHOLD")
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget; 
		maxIxDThreshold = slider->getScaledValue(); 
        cout << "value: " << slider->getScaledValue() << endl; 
    }else if(name == "CALIBRATION")
    {
        ofxUIButton *button = (ofxUIButton *) e.widget;
        cout << "calibration clicked " << endl;  
        
    }
    
}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y)
{}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}