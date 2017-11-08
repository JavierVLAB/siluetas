#include "ofApp.h"

//---------
void ofApp::setupGui(){
  ImGuiIO * io = &ImGui::GetIO();
  io->Fonts->AddFontFromFileTTF(&ofToDataPath("Roboto-Medium.ttf")[0], 14.f);
  //io->Fonts->AddFontFromFileTTF(&ofToDataPath("Raleway-Regular.ttf")[0], 12.f);

  gui.setup();

  showGui = false;
}

//----------
void ofApp::drawGui(){

  if(showGui){
    gui.begin();
      ImGui::Separator();
      ImGui::Text("Application average frame rate = %.1f FPS", ImGui::GetIO().Framerate);
      ImGui::Separator();
      ImGui::Text("Press Space Bar to show de Gui");
      ImGui::Separator();
      ImGui::SliderInt("Near threshold", &nearThreshold, 0, 255);
      ImGui::SliderInt("Far threshold", &farThreshold, 0, 255);
      ImGui::Separator();
			ImGui::Text("Application average frame rate = %.1f FPS", ImGui::GetIO().Framerate);

			if(kinect.hasAccelControl()) {
					//ImGui::Text( "accel is: %s / %s / %s ", ofToString(kinect.getMksAccel().x, 2),
					//						ofToString(kinect.getMksAccel().y, 2), ofToString(kinect.getMksAccel().z, 2));
			} else {
					ImGui::Text("Note: this is a newer Xbox Kinect or Kinect For Windows device,");
					ImGui::Text("motor / led / accel controls are not currently supported");
			}

			ImGui::Text("press p to switch between images and point cloud, rotate the point cloud with the mouse");
			//ImGui::Text("using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
			ImGui::Text("set near threshold %i (press: + -)", nearThreshold);
			ImGui::Text("set far threshold %i (press: < >) num blobs found %d, fbs: %.3f", farThreshold, contourFinder.nBlobs, ofGetFrameRate());
			//ImGui::Text("press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl;

			if(kinect.hasCamTiltControl()) {
				ImGui::Text("press UP and DOWN to change the tilt angle: %d degrees", angle);
				ImGui::Text("press 1-5 & 0 to change the led mode");
			}

			//ImGui::ImageButton((ImTextureID)(uintptr_t)imageButtonID, ImVec2(300, 200));

    gui.end();
  }
}



//--------------------------------------------------------------
void ofApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);

	setupGui();

	kinect.setRegistration(true);

	kinect.init();
	kinect.open();

	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);

	nearThreshold = 247;
	farThreshold = 225;
	bThreshWithOpenCV = true;

	ofSetFrameRate(30);

	angle = 0;
	kinect.setCameraTiltAngle(angle);


	colors.resize(10); // allocate space for 100 ints in vector

	for(int i = 0; i < colors.size(); i++){
	  colors[i] = i*25; // set value using index
	}
	ofRandomize(colors);

}

//--------------------------------------------------------------
void ofApp::update() {

	ofBackground(0);

	kinect.update();

	if(kinect.isFrameNew()) {

			grayImage.setFromPixels(kinect.getDepthPixels());

			ofPixels & pix = grayImage.getPixels();
			int numPixels = pix.size();
			for(int i = 0; i < numPixels; i++) {
				if(pix[i] < nearThreshold && pix[i] > farThreshold) {
					pix[i] = 255;
				} else {
					pix[i] = 0;
				}
		}

		/*ofImage img;
		img.setFromPixels(colorImg.getPixels());
		img.resize(160,120);
		imageButtonID = gui.loadImage(img);*/

		grayImage.flagImageChanged();
		grayImage.blur(5);
		contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);
	}




}

//--------------------------------------------------------------
void ofApp::draw() {

	//ofSetColor(255, 255, 255);

	//kinect.drawDepth(10, 10, 400, 300);
	//kinect.draw(420, 10, 400, 300);

	//grayImage.draw(10, 320, 400, 300);
	//contourFinder.draw(10, 320, 400, 300);

  if(contourFinder.nBlobs > 0){
		for(int i=0; i < contourFinder.nBlobs; i++){
	  		ofxCvBlob blob = contourFinder.blobs[i];

	      ofFill();
				ofColor c;
				c.setHsb(colors[i],170,255);
	      ofSetColor(c);
	      ofBeginShape();
	      for (int j = 0; j < blob.nPts; j++){
	             ofVertex(blob.pts[j].x*1.5,blob.pts[j].y*1.5);
	      }
	      ofEndShape(true);
			}
	}


	drawGui();

}

//--------------------------------------------------------------
void ofApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();

}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
	switch (key) {
		case 'g':
			showGui = !showGui;
			break;
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

		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;

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
		case 'f':
			ofToggleFullscreen();
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
