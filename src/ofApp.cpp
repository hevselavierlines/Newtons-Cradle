#include "ofApp.h"
using namespace YAMPE;

//--------------------------------------------------------------
void ofApp::setup() {
    
    ofSetLogLevel(OF_LOG_VERBOSE);
    
    // repatable randomness
    ofSeedRandom(10);
    
    // simulation specific stuff goes here

    gui.setup();
    ImGui::GetIO().MouseDrawCursor = false;

    // load parameters from file
    // loadFromFile("settings.xml");
    
    // instantiate the ground
    ground.set(RANGE, RANGE);
    ground.rotate(90, 1,0,0);
    
    // lift camera to 'eye' level
    easyCam.setDistance(RANGE);
    float d = easyCam.getDistance();
    easyCam.setPosition(ofVec3f(0, cameraHeightRatio*d, d*sqrt(1.0f-cameraHeightRatio*cameraHeightRatio))+easyCamTarget);
    easyCam.setTarget(easyCamTarget);

    //set the contact and force registry as well as the gravity force creator
    contacts = P::ContactRegistry::Ref(new P::ContactRegistry());
    forces = P::ForceGeneratorRegistry::Ref(new P::ForceGeneratorRegistry());
    
    gravity = P::GravityForceGenerator::Ref(new P::GravityForceGenerator({0, -9.81, 0}));
    
    // finally start everything off by resetting the simulation
    reset();
    
}

void ofApp::reset() {
    t = 0.0f;
    particles.clear();
    forces->clear();
    anchorConstraints.clear();
    distance = guiDistance;
    length = guiLength;
    boardHeight = guiBoardHeight;
    float fullDistance = distance * nTotal;
    xStart = -(nTotal + fullDistance) / 2;
    xStart += 0.5f + (distance / 2);
    
    float xPos = xStart;
    
    for(int i = 0; i < nTotal; i++) {
        Particle::Ref ball = Particle::Ref(new Particle());
        
        //set the position of the anchor
        ofVec3f anchor;
        anchor.x = xPos;
        anchor.y = boardHeight;
        anchor.z = 0;
        
        ofVec3f ballPos;
        ballPos.x = xPos;
        ballPos.y = boardHeight;
        ballPos.z = 0;
        
        if(i < nPerturb) {
            float angle = 90.0f - perturbAngle;
            ballPos.x -= (length * cos(ofDegToRad(angle)));
            ballPos.y -= (length * sin((ofDegToRad(angle))));
        } else {
            ballPos.y -= length;
        }
        
        ball->setPosition(ballPos);
        ball->setMass(ballRadius);
        ball->setRadius(ballRadius);
        ball->setBodyColor({255, 0, 0});
        ball->setDamping(1.0f);
        
        xPos += ballRadius * 2.0f + distance;
        forces->add(ball, gravity);
        particles.push_back(ball);
        
        P::EqualityAnchoredConstraint::Ref constraint =
        P::EqualityAnchoredConstraint::Ref(new P::EqualityAnchoredConstraint(ball, anchor, length));
        anchorConstraints.push_back(constraint);
    }
    particleContact.particles = particles;
}

void ofApp::update() {

    float dt = ofClamp(ofGetLastFrameTime(), 0.0, 0.02);
    if (!isRunning) return;
    t += dt;
    
    if(dt > 0) {
        contacts->clear();
        forces->applyForce(dt);
        
        for(int i = 0; i < particles.size(); i++) {
            Particle::Ref particle = particles[i];
            P::EqualityAnchoredConstraint::Ref constraint = anchorConstraints[i];
            
            constraint->generate(contacts);
            particleContact.generate(contacts);
            particle->integrate(dt);
        }
        
        contacts->resolve(dt);
    }
}

void ofApp::draw() {
    
    ofEnableDepthTest();
    ofBackgroundGradient(ofColor(128), ofColor(0), OF_GRADIENT_BAR);
    
    ofPushStyle();
    easyCam.begin();
    
    ofDrawGrid(RANGE/(2*8), 8, false, isXGridVisible, isYGridVisible, isZGridVisible);
    
    if (isAxisVisible) ofDrawAxis(1);
    
    if (isGroundVisible) {
        ofPushStyle();
        ofSetHexColor(0xB87333);
        ground.draw();
        ofPopStyle();
    }
    
    ofSetColor(255, 0, 0);
    ofFill();
    
    float width = abs(xStart) * 2.0f + 3.0f;
    
    ofDrawBox(0.0f, boardHeight,  2.0f, width + 0.5f, 1.0f, 0.5f);
    ofDrawBox(0.0f, boardHeight, -2.0f, width + 0.5f, 1.0f, 0.5f);
    
    ofDrawBox(-width / 2, boardHeight / 2, -2.0f, 0.5f, boardHeight, 0.5f);
    ofDrawBox(-width / 2, boardHeight / 2, 2.0f, 0.5f, boardHeight, 0.5f);
    ofDrawBox(width / 2, boardHeight / 2, -2.0f, 0.5f, boardHeight, 0.5f);
    ofDrawBox(width / 2, boardHeight / 2, 2.0f, 0.5f, boardHeight, 0.5f);
    ofDrawBox(0, 0.5f, 0, width + 0.5f, 1.0f, 4.5f);
    
    float xPos = xStart;
    
    for(Particle::Ref particle : particles) {
        ofSetColor(255, 255, 0);
        ofDrawLine(xPos, boardHeight-0.5f, -2.0, particle->position.x, particle->position.y, 0);
        ofDrawLine(xPos, boardHeight-0.5f, 2.0, particle->position.x, particle->position.y, 0);
        
        ofSetColor(255, 255, 255);
        ofDrawSphere(xPos, boardHeight - 0.5f, -2.0f, 0.1f);
        ofDrawSphere(xPos, boardHeight - 0.5f, 2.0, 0.1f);
        
        particle->draw();
        xPos += ballRadius * 2.0f + distance;
    }
    
    easyCam.end();
    ofPopStyle();

    // draw gui elements
    gui.begin();
    drawAppMenuBar();
    drawMainWindow();
    drawLoggingWindow();
    gui.end();
}


void ofApp::drawAppMenuBar() {
    
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            ImGui::Separator();
            if (ImGui::MenuItem("Quit", "")) quit();
            ImGui::EndMenu();
        }
        
        float d = easyCam.getDistance();
        
        if (ImGui::BeginMenu("View")) {
            if (ImGui::MenuItem(isAxisVisible?"Hide Unit Axis":"Show Unit Axis", "")) isAxisVisible = !isAxisVisible;
            if (ImGui::MenuItem(isGroundVisible?"Hide Ground":"Show Ground", "")) isGroundVisible = !isGroundVisible;
            if (ImGui::MenuItem(isXGridVisible?"Hide Grid (X)":"Show Grid (X)", "")) isXGridVisible = !isXGridVisible;
            if (ImGui::MenuItem(isYGridVisible?"Hide Grid (Y)":"Show Grid (Y)", "")) isYGridVisible = !isYGridVisible;
            if (ImGui::MenuItem(isZGridVisible?"Hide Grid (Z)":"Show Grid (Z)", "")) isZGridVisible = !isZGridVisible;
            ImGui::Separator();
            if (ImGui::MenuItem("Align camera above X axis ", "")) {
                easyCam.setPosition(ofVec3f(d*sqrt(1.0f-cameraHeightRatio*cameraHeightRatio), cameraHeightRatio*d, 0)+easyCamTarget);
                easyCam.setTarget(easyCamTarget);
            }
            if (ImGui::MenuItem("Align camera above Z axis ", "")) {
                easyCam.setPosition(ofVec3f(0, cameraHeightRatio*d, d*sqrt(1.0f-cameraHeightRatio*cameraHeightRatio))+easyCamTarget);
                easyCam.setTarget(easyCamTarget);
                cout <<"here";
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Align camera along X axis ", "")) {
                easyCam.setPosition(ofVec3f(d, 0, 0)+easyCamTarget);
                easyCam.setTarget(easyCamTarget);
            }
            if (ImGui::MenuItem("Align camera along Y axis ", "")) {
                easyCam.setPosition(ofVec3f(0.001, d, 0)+easyCamTarget);
                easyCam.setTarget(easyCamTarget);
            }
            if (ImGui::MenuItem("Align camera along Z axis ", "")) {
                easyCam.setPosition(ofVec3f(0, 0, d)+easyCamTarget);
                easyCam.setTarget(easyCamTarget);
            }
            
            ImGui::EndMenu();
        }
        
        if (ImGui::BeginMenu("Application")) {
            if (ImGui::MenuItem("Add application specific menu items here", "")) {
            }
            ImGui::EndMenu();
        }

        ImGui::EndMainMenuBar();
    }
}


void ofApp::drawMainWindow() {
    

    ImGui::SetNextWindowPos(ImVec2(0,20), ImGuiSetCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(300,300), ImGuiSetCond_FirstUseEver);
    if (ImGui::Begin("Main")) {

        if (ImGui::CollapsingHeader("3D")) {
            if(ImGui::Button("Reset##CameraTarget")) {
                easyCamTarget = ofVec3f(0,5,0);
                easyCam.setTarget(easyCamTarget);
            }

            ImGui::SameLine();
            if (ImGui::InputFloat3("Camera Target", &easyCamTarget.x)) {
                easyCam.setTarget(easyCamTarget);
            }
            if (ImGui::SliderFloat("Camera Height Ratio", &cameraHeightRatio, 0.0f, 1.0f))
                cameraHeightRatioChanged(cameraHeightRatio);
            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        }

        if(ImGui::Button("Reset")) reset();
        ImGui::SameLine();
        if(ImGui::Button(isRunning?"Stop":" Go ")) isRunning = !isRunning;
        ImGui::SameLine();
        ImGui::Text("   Time = %8.1f", t);
        if(ImGui::Button("Quit")) quit();
        
        ImGui::SliderInt("Number of bobs", &nTotal, 1, 20);
        ImGui::SliderInt("Number of perturbe", &nPerturb, 1, nTotal);
        ImGui::SliderFloat("Angle (deg)", &perturbAngle, 0, 90.0f, "%2.1f");
        ImGui::SliderFloat("Distance between bobs", &guiDistance, 0, 1.0f);
        ImGui::SliderFloat("Board height", &guiBoardHeight, 3.5f, 12.0f, "%2.1f");
        ImGui::SliderFloat("Bob length", &guiLength, 1.0f, guiBoardHeight - 1.5f, "%2.1f");
        
        if (ImGui::CollapsingHeader("Numerical Output")) {
            // TODO - numeric output goes here
        }
        
        if (ImGui::CollapsingHeader("Graphical Output")) {
            // TODO - graphical output goes here
        }
    }
    
    // store window size so that camera can ignore mouse clicks
    mainWindowRectangle.setPosition(ImGui::GetWindowPos().x,ImGui::GetWindowPos().y);
    mainWindowRectangle.setSize(ImGui::GetWindowWidth(), ImGui::GetWindowHeight());
    ImGui::End();

}


void ofApp::drawLoggingWindow() {
    ImGui::SetNextWindowSize(ImVec2(290,300), ImGuiSetCond_Always);
    ImGui::SetNextWindowPos(ImVec2(ofGetWindowWidth()-300,20), ImGuiSetCond_Always);
    
    if (ImGui::Begin("Logging")) {
    }
    // store window size so that camera can ignore mouse clicks
    loggingWindowRectangle.setPosition(ImGui::GetWindowPos().x,ImGui::GetWindowPos().y);
    loggingWindowRectangle.setSize(ImGui::GetWindowWidth(), ImGui::GetWindowHeight());
    ImGui::End();
}

//--------------------------------------------------------------
// GUI events and listeners
//--------------------------------------------------------------

void ofApp::keyPressed(int key) {
    
    float d = easyCam.getDistance();
    
    switch (key) {
        
//        case 'h':                               // toggle GUI/HUD
//           isGuiVisible = !isGuiVisible;
//            break;
//        case 'b':                               // toggle debug
//            isDebugVisible = !isDebugVisible;
//            break;
//        case 'a':                               // toggle axis unit vectors
//            isAxisVisible = !isAxisVisible;
//            break;
//        case 'g':                               // toggle ground
//            isGroundVisible = !isGroundVisible;
//            break;
//        case 'u':                               // set the up vecetor to be up (ground to be level)
//            easyCam.setTarget(ofVec3f::zero());
//            break;
//
//        case 'S' :                              // save gui parameters to file
//            gui.saveToFile("settings.xml");
//
//            break;
//        case 'L' :                              // load gui parameters
//            gui.loadFromFile("settings.xml");
//            break;
//
        case 'z':
            easyCam.setPosition(ofVec3f(0, cameraHeightRatio*d, d*sqrt(1.0f-cameraHeightRatio*cameraHeightRatio))+easyCamTarget);
            easyCam.setTarget(easyCamTarget);
            break;
        case 'Z':
            easyCam.setPosition(0, 0, d);
            easyCam.setTarget(ofVec3f::zero());
            break;
        case 'x':
            easyCam.setPosition(ofVec3f(d*sqrt(1.0f-cameraHeightRatio*cameraHeightRatio), cameraHeightRatio*d, 0)+easyCamTarget);
            easyCam.setTarget(easyCamTarget);
            break;
        case 'X':
            easyCam.setPosition(ofVec3f(d, 0, 0)+easyCamTarget);
            easyCam.setTarget(easyCamTarget);
            break;
        case 'Y':
            easyCam.setPosition(ofVec3f(0.001, d, 0)+easyCamTarget);
            easyCam.setTarget(easyCamTarget);
            break;
            
        case 'f':                               // toggle fullscreen
            // BUG: window size is not preserved
            isFullScreen = !isFullScreen;
            if (isFullScreen) {
                ofSetFullscreen(false);
            } else {
                ofSetFullscreen(true);
            }
            break;

        // simulation specific stuff goes here

    }
}


void ofApp::cameraHeightRatioChanged(float & cameraHeightRatio) {
    
    float d = easyCam.getDistance();
    easyCam.setPosition(ofVec3f(0, cameraHeightRatio*d, d*sqrt(1.0f-cameraHeightRatio*cameraHeightRatio))+easyCamTarget);
    easyCam.setTarget(easyCamTarget);
}


void ofApp::quit() {
    ofExit();
}

//--------------------------------------------------------------
// Unused
//--------------------------------------------------------------
void ofApp::keyReleased(int key) {}
void ofApp::mouseMoved(int x, int y ) {}
void ofApp::mouseDragged(int x, int y, int button) {}
void ofApp::mousePressed(int x, int y, int button) {
    // easy camera should ignore GUI mouse clicks
    if (mainWindowRectangle.inside(x,y)||loggingWindowRectangle.inside(x,y))
        easyCam.disableMouseInput();
    else
        easyCam.enableMouseInput();
}
void ofApp::mouseReleased(int x, int y, int button) {}
void ofApp::mouseEntered(int x, int y) {}
void ofApp::mouseExited(int x, int y) {}
void ofApp::windowResized(int w, int h) {}
void ofApp::gotMessage(ofMessage msg) {}
void ofApp::dragEvent(ofDragInfo dragInfo) {}
