#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxImGui.h"

class ofApp : public ofBaseApp {
public:

	void setup();
	void update();
	void draw();
	void exit();

	void drawPointCloud();

	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);

	ofxKinect kinect;

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

	//--------- ofxImGui ------------

void drawGui();
void setupGui();

ofxImGui::Gui gui;
bool     showGui;

ofImage imageButtonSource;
GLuint imageButtonID;

//-----
	vector<int> colors;

};
