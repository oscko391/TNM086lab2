
#include <sgct.h>

#include <iostream>

#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/ComputeBoundsVisitor>
#include <osg/ShapeDrawable>
#include <osgUtil/LineSegmentIntersector>
#include <osgUtil/IntersectionVisitor>
#include <osg/Material>
#include <osg/StateAttribute>

sgct::Engine * gEngine;

#define WAND_SENSOR_IDX 0
#define HEAD_SENSOR_IDX 1

// OSG stuff
osgViewer::Viewer * mViewer;
osg::ref_ptr<osg::Group> mRootNode;
osg::ref_ptr<osg::MatrixTransform> mSGCTTrans;
osg::ref_ptr<osg::MatrixTransform> mSceneTrans;
osg::ref_ptr<osg::FrameStamp> mFrameStamp; //to sync osg animations across cluster
osg::ref_ptr<osg::Geometry> linesGeom;

osg::ref_ptr<osg::MatrixTransform> mCessnaTrans;
osg::ref_ptr<osg::MatrixTransform> mModelTrans;
osg::ref_ptr<osg::PositionAttitudeTransform> heightTrans;
osg::ref_ptr<osg::Material> airplaneMaterial;


// callbacks
void myInitOGLFun();
void myPreSyncFun();
void myPostSyncPreDrawFun();
void myDrawFun();
void myEncodeFun();
void myDecodeFun();
void myCleanUpFun();
void keyCallback(int key, int action);

// other functions
void initOSG();
void createOSGScene();
void setupLightSource();
osg::Geode* createWand();
void checkIntersection();
void setColor(osg::ref_ptr<osg::Node> n, osg::Vec4 color);

//variables to share across cluster
sgct::SharedDouble curr_time(0.0);
sgct::SharedDouble dist(-2.0);
sgct::SharedVector<glm::mat4> sharedTransforms;
sgct::SharedString sharedText;

sgct::SharedBool wireframe(false);
sgct::SharedBool info(false);
sgct::SharedBool stats(false);
sgct::SharedBool takeScreenshot(false);
sgct::SharedBool light(true);
sgct::SharedVector<bool> sharedButton;

bool moving = false;
bool pointMode = false;

// Simple initial navigation based on arrow buttons
bool arrowButtons[4];
enum directions { FORWARD = 0, BACKWARD, LEFT, RIGHT };
const double navigation_speed = 1.0;

int main( int argc, char* argv[] ) {
  gEngine = new sgct::Engine( argc, argv );

  gEngine->setInitOGLFunction( myInitOGLFun );
  gEngine->setPreSyncFunction( myPreSyncFun );
  gEngine->setPostSyncPreDrawFunction( myPostSyncPreDrawFun );
  gEngine->setDrawFunction( myDrawFun );
  gEngine->setCleanUpFunction( myCleanUpFun );
  gEngine->setKeyboardCallbackFunction( keyCallback );
  

  //fix incompability with warping and OSG
  sgct_core::ClusterManager::instance()->setMeshImplementation( sgct_core::ClusterManager::DISPLAY_LIST );

  for(int i=0; i<4; i++)
    arrowButtons[i] = false;

  if( !gEngine->init() ) {
    delete gEngine;
    return EXIT_FAILURE;
  }

  sgct::SharedData::instance()->setEncodeFunction( myEncodeFun );
  sgct::SharedData::instance()->setDecodeFunction( myDecodeFun );

  // Main loop
  gEngine->render();

  // Clean up
  delete gEngine;

  // Exit program
  exit( EXIT_SUCCESS );
}

void myInitOGLFun() {
  initOSG();
  createOSGScene();
  setupLightSource();

  //only store the tracking data on the master node
  if( !gEngine->isMaster() ) return;
  mSceneTrans->setMatrix(osg::Matrix::translate(0.0, 0.0, dist.getVal()));

  for(size_t i = 0; i < sgct::Engine::getTrackingManager()->getNumberOfTrackers(); i++){
    sgct::SGCTTracker * trackerPtr = sgct::Engine::getTrackingManager()->getTrackerPtr(i);

    for(size_t j=0; j<trackerPtr->getNumberOfDevices(); j++){
      sgct::SGCTTrackingDevice * devicePtr = trackerPtr->getDevicePtr(j);

      if (devicePtr->hasSensor())
        sharedTransforms.addVal( glm::mat4(1.0f) );
      
        if ( devicePtr->hasButtons() )
        {
            for ( int k = 0; k < devicePtr->getNumberOfButtons(); k++ )
            {
                sharedButton.addVal(0);
            }
        }
    }

  }
  sharedText.setVal(" "); //< Space since SGCT hangs otherwise
}

void createOSGScene() {

  mRootNode->addChild(createWand());

  osg::ref_ptr<osg::Node>            mModel;
  osg::ref_ptr<osg::Node> mCessnaModel;
  
  

  mSGCTTrans         = new osg::MatrixTransform();
  mSceneTrans        = new osg::MatrixTransform();
  mModelTrans        = new osg::MatrixTransform();
  mCessnaTrans = new osg::MatrixTransform();

  //rotate osg coordinate system to match sgct
  mModelTrans->preMult(osg::Matrix::rotate(glm::radians(-90.0f),
                                           1.0f, 0.0f, 0.0f));
  mCessnaTrans->preMult(osg::Matrix::rotate(glm::radians(-90.0f),
                                           1.0f, 0.0f, 0.0f));

  mRootNode->addChild( mSGCTTrans.get() );
  mSGCTTrans->addChild( mSceneTrans.get() );
  mSceneTrans->addChild( mModelTrans.get() );
  mSceneTrans->addChild( mCessnaTrans.get());

  sgct::MessageHandler::instance()->print("Loading model 'airplane.ive'...\n");
  mModel = osgDB::readNodeFile("airplane.ive");
  mCessnaModel = osgDB::readNodeFile("cessna.osg");

  if (!mModel.valid() || !mCessnaModel) {
    sgct::MessageHandler::instance()->print("Failed to read model!\n");
    return;
  }

  sgct::MessageHandler::instance()->print("Model loaded successfully!\n");
  mModelTrans->addChild(mModel.get());
  mCessnaTrans->addChild(mCessnaModel.get());

  //get the bounding box
  osg::ComputeBoundsVisitor cbv;
  osg::BoundingBox &bb(cbv.getBoundingBox());
  mModel->accept( cbv );

  osg::Vec3f tmpVec;
  tmpVec = bb.center();
  tmpVec.x() += 6;
  

  // translate model center to origin
  mModelTrans->postMult(osg::Matrix::translate( -tmpVec ) );

  // scale model to a manageable size
  double scale = 0.5 / bb.radius();
  mModelTrans->postMult(osg::Matrix::scale(scale, scale, scale));

  sgct::MessageHandler::instance()->print("Model bounding sphere center:\tx=%f\ty=%f\tz=%f\n", tmpVec[0], tmpVec[1], tmpVec[2] );
  sgct::MessageHandler::instance()->print("Model bounding sphere radius:\t%f\n", bb.radius() );

  // Bounding box Cessna
  mCessnaModel->accept( cbv );
  tmpVec = bb.center();
  tmpVec.x() -= 20;
  
  mCessnaTrans->postMult(osg::Matrix::translate(-tmpVec));
  
  scale = 0.5 / bb.radius();
  mCessnaTrans->postMult(osg::Matrix::scale(scale, scale, scale));
  
  sgct::MessageHandler::instance()->print("cessna bounding sphere center:\tx=%f\ty=%f\tz=%f\n", tmpVec[0], tmpVec[1], tmpVec[2]);
sgct::MessageHandler::instance()->print("cessna bounding sphere radius:\t%f\n", bb.radius());
  
  //disable face culling
  mModel->getOrCreateStateSet()->setMode( GL_CULL_FACE,
                                          osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
  mCessnaModel->getOrCreateStateSet()->setMode( GL_CULL_FACE,
                                                osg::StateAttribute::OFF |
                                                osg::StateAttribute::OVERRIDE);
  
  // heightfield
  unsigned int numCols = 30;
  unsigned int numRows = 30;
  osg::ref_ptr<osg::HeightField> heightField = new osg::HeightField;
  heightField->allocate(numCols, numRows);
  heightField->setOrigin(osg::Vec3(0, 0, 0));
  heightField->setXInterval(1.0f);
  heightField->setYInterval(1.0f);
  
  // set height
  for (int row = 0; row < heightField->getNumRows(); row++)
  {
      for (int col = 0; col < heightField->getNumColumns(); col++)
      {
          heightField->setHeight(col, row, 0.5 * (cos(col * 1.0) + sin(row * 2.0)));
      }
  }
  
  osg::ref_ptr<osg::Geode> heightGeode = new osg::Geode();
  heightGeode->addDrawable(new osg::ShapeDrawable(heightField));
  
  
//   osg::Texture2D* tex = new osg::Texture2D(osgDB::readImageFile("groundtex.jpg"));
//   if (!tex.valid()) {
//     sgct::MessageHandler::instance()->print("Failed to read texture!\n");
//     return;
//   }
//   tex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
//   tex->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR);
//   tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
//   tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
//   heightGeode->getOrCreateStateSet()->setTextureAttributeAndModes(0,tex);
  
  heightTrans = new osg::PositionAttitudeTransform();
  
  heightTrans->setPosition(osg::Vec3(-1.2, -0.5 , -1.8));
  heightTrans->setAttitude(osg::Quat(osg::DegreesToRadians(-90.0), osg::Vec3(1,0,0)));
  heightTrans->setScale(osg::Vec3(0.1, 0.1, 0.1));
  heightTrans->addChild(heightGeode);
  mSceneTrans->addChild(heightTrans);
  
  //save airplanes material 
  airplaneMaterial = (osg::Material*)mModel->getOrCreateStateSet()->getAttribute(osg::StateAttribute::MATERIAL);

}

void myPreSyncFun() {
  if (!gEngine->isMaster())
    return;

  curr_time.setVal( sgct::Engine::getTime() );

  if( arrowButtons[FORWARD] )
    dist.setVal( dist.getVal() + (navigation_speed * gEngine->getDt()));

  if( arrowButtons[BACKWARD] )
    dist.setVal( dist.getVal() - (navigation_speed * gEngine->getDt()));

  std::stringstream message;

  size_t index = 0;
  for(size_t i = 0; i < sgct::Engine::getTrackingManager()->getNumberOfTrackers(); i++){
    sgct::SGCTTracker * trackerPtr = sgct::Engine::getTrackingManager()->getTrackerPtr(i);

    for(size_t j = 0; j < trackerPtr->getNumberOfDevices(); j++){
      sgct::SGCTTrackingDevice * devicePtr = trackerPtr->getDevicePtr(j);

      message << "Device " << i << " on tracker " << j << std::endl;

      if( devicePtr->hasSensor() ){
        sharedTransforms.setValAt( index, devicePtr->getWorldTransform() );
        message << "Position:" << std::endl << "  "
                << devicePtr->getPosition().x << ", "
                << devicePtr->getPosition().y << ", "
                << devicePtr->getPosition().z << std::endl;
        message << "Euler angles:" << std::endl << "  "
                << devicePtr->getEulerAngles().x << ", "
                << devicePtr->getEulerAngles().y << ", "
                << devicePtr->getEulerAngles().z << std::endl;
        index++;
      }

      if( devicePtr->hasButtons() ){
        message << "Buttons:" << std::endl << "  ";
        for( int idx = 0 ; idx < devicePtr->getNumberOfButtons() ; ++idx ){
          message << devicePtr->getButton(idx) ? "1" : "0";
          sharedButton.setValAt(idx, devicePtr->getButton(idx));
        }
        message << std::endl;
      }

      if( devicePtr->hasAnalogs() ){
        message << "Analogs:" << std::endl << "  ";
        for( int idx = 0 ; idx < devicePtr->getNumberOfAxes() ; ++idx ){
          message << "  " << devicePtr->getAnalog(idx) << std::endl;
        }
      }

      message << std::endl;
    }
  }

  if (message.str().size() < 2)
    message << " "; //< Space since SGCT hangs otherwise
  sharedText.setVal(message.str());
}

void myPostSyncPreDrawFun() {
  gEngine->setWireframe(wireframe.getVal());
  gEngine->setDisplayInfoVisibility(info.getVal());
  gEngine->setStatsGraphVisibility(stats.getVal());
  
  pointMode = false;

  if (takeScreenshot.getVal()) {
    gEngine->takeScreenshot();
    takeScreenshot.setVal(false);
  }

  if (light.getVal())
    mRootNode->getOrCreateStateSet()->setMode( GL_LIGHTING,
                                               osg::StateAttribute::ON |
                                               osg::StateAttribute::OVERRIDE);
  else
    mRootNode->getOrCreateStateSet()->setMode( GL_LIGHTING,
                                               osg::StateAttribute::OFF |
                                               osg::StateAttribute::OVERRIDE);

  // Simple initial navigation based on arrow buttons
  //mSceneTrans->setMatrix(osg::Matrix::translate(0.0, 0.0, dist.getVal()));

  // SGCT internal transformation from configuration file
  mSGCTTrans->setMatrix(osg::Matrix(glm::value_ptr(gEngine->getModelMatrix())));

  //update the frame stamp in the viewer to sync all
  //time based events in osg
  mFrameStamp->setFrameNumber( gEngine->getCurrentFrameNumber() );
  mFrameStamp->setReferenceTime( curr_time.getVal() );
  mFrameStamp->setSimulationTime( curr_time.getVal() );
  mViewer->setFrameStamp( mFrameStamp.get() );
  mViewer->advance( curr_time.getVal() ); //update



    if ( sharedButton.getValAt(0) )
    {
        pointMode = true;
        moving = true;
    }

  glm::mat4 wand_matrix = sharedTransforms.getValAt(WAND_SENSOR_IDX);
  // Update wand in OSG
  if( sharedTransforms.getSize() > WAND_SENSOR_IDX ){
    

    glm::vec3 wand_position = glm::vec3(wand_matrix*glm::vec4(0,0,0,1));
    //glm::quat wand_orientation = glm::quat_cast(wand_matrix);
    glm::mat3 wand_orientation = glm::mat3(wand_matrix);

    glm::vec3 start = wand_position;
    glm::vec3 end = wand_position + wand_orientation * glm::vec3(0,0,1);
    osg::Vec3d wand_start = osg::Vec3(start.x, start.y, start.z);
    osg::Vec3d wand_end = osg::Vec3(end.x, end.y, end.z);

    osg::Vec3Array* vertices = new osg::Vec3Array();
    vertices->push_back(wand_start);
    vertices->push_back(wand_end);
    linesGeom->setVertexArray(vertices);
  }

  float speed = 0.2 * gEngine->getDt();
  if ( pointMode )
  {
      glm::vec3 translation = glm::mat3(wand_matrix) * glm::vec3(0,0,-1) * speed;
      mSceneTrans->postMult( osg::Matrix::translate(-osg::Vec3(translation.x, 
                                                              translation.y,
                                                              translation.z)));
      
  }

  //traverse if there are any tasks to do
  if (!mViewer->done()) {
    mViewer->eventTraversal();
    //update travelsal needed for pagelod object like terrain data etc.
    mViewer->updateTraversal();
    checkIntersection();
  }
}


void checkIntersection()
{
    if ( sharedTransforms.getSize() > WAND_SENSOR_IDX)
    {
        glm::mat4 wand_matrix = sharedTransforms.getValAt(WAND_SENSOR_IDX);

        glm::vec3 wand_position = glm::vec3(wand_matrix*glm::vec4(0,0,0,1));
        //glm::quat wand_orientation = glm::quat_cast(wand_matrix);
        glm::mat3 wand_orientation = glm::mat3(wand_matrix);

        glm::vec3 start = wand_position;
        glm::vec3 end = wand_position + wand_orientation * glm::vec3(0,0,-1);
    
    
        osg::Vec3 wandStart = osg::Vec3(start.x, start.y, start.z);
        osg::Vec3 wandEnd = osg::Vec3(end.x, end.y, end.z);
        
        osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector;
        osgUtil::IntersectionVisitor visitor;
        
        intersector = new osgUtil::LineSegmentIntersector(wandStart, wandEnd);
        
        visitor.setIntersector(intersector);
        
        mRootNode->accept(visitor);
        
        osg::ref_ptr<osgUtil::LineSegmentIntersector> lsi = dynamic_cast<osgUtil::LineSegmentIntersector*>(visitor.getIntersector());
        
        if (lsi->containsIntersections())
        {
            osg::NodePath nodes = lsi->getFirstIntersection().nodePath;
            for (int i = 0; i < nodes.size(); i++)
            {
                if (nodes[i] == mCessnaTrans)
                {
                    osg::ref_ptr<osg::Node> n = mCessnaTrans->getChild(0);
                    setColor(n, osg::Vec4(1, 0, 0, 1));
                }
                if ( nodes[i] == mModelTrans )
                {
                    osg::ref_ptr<osg::Node> n = mModelTrans->getChild(0);
                    setColor(n, osg::Vec4(1, 0, 0, 1));
                }
                if ( nodes[i] == heightTrans )
                {
                    osg::ref_ptr<osg::Node> n = heightTrans->getChild(0);
                    setColor(n, osg::Vec4(1, 0, 0, 1));
                }
            }
        }
        else
        {
            // cessna
            osg::ref_ptr<osg::Node> cessnaNode = mCessnaTrans->getChild(0);
            setColor(cessnaNode, osg::Vec4(0, 1, 0, 1));
            
            //airplane
            osg::ref_ptr<osg::Node> airNode = mModelTrans->getChild(0);
            setColor(airNode, osg::Vec4(0, 1, 1, 1));
            
            // heightField
            osg::ref_ptr<osg::Node> heightNode = heightTrans->getChild(0);
            setColor(heightNode, osg::Vec4(0, 0, 1, 1));
                    
        }
    }
            
}

void setColor(osg::ref_ptr<osg::Node> n, osg::Vec4 color)
{
    osg::ref_ptr<osg::Material> material = (osg::Material*)n->getOrCreateStateSet()->getAttribute(osg::StateAttribute::MATERIAL);
    
    if (!material)
    {
        material = new osg::Material();
    }
    material->setAmbient(osg::Material::FRONT_AND_BACK, color);
    material->setDiffuse(osg::Material::FRONT_AND_BACK, color);
    n->getOrCreateStateSet()->setAttributeAndModes(material.get(), osg::StateAttribute::OVERRIDE);
}

void myDrawFun() {
  glLineWidth(2.0f);

  const int * curr_vp = gEngine->getCurrentViewportPixelCoords();
  mViewer->getCamera()->setViewport(curr_vp[0], curr_vp[1], curr_vp[2], curr_vp[3]);
  mViewer->getCamera()->setProjectionMatrix( osg::Matrix( glm::value_ptr(gEngine->getCurrentViewProjectionMatrix() ) ));

  mViewer->renderingTraversals();

	// draw text with OpenGL
	float textVerticalPos = static_cast<float>(gEngine->getCurrentWindowPtr()->getYResolution()) - 100.0f;
	int fontSize = 12;

	glColor3f(1.0f, 1.0f, 1.0f);
	sgct_text::print(sgct_text::FontManager::instance()->getFont( "SGCTFont", fontSize ),
		sgct_text::TextAlignMode::TOP_LEFT,
		120.0f, textVerticalPos,
		sharedText.getVal().c_str() );
}

void myEncodeFun() {
  sgct::SharedData::instance()->writeDouble( &curr_time );
  sgct::SharedData::instance()->writeVector( &sharedTransforms );
	sgct::SharedData::instance()->writeString( &sharedText );
  sgct::SharedData::instance()->writeBool( &wireframe );
  sgct::SharedData::instance()->writeBool( &info );
  sgct::SharedData::instance()->writeBool( &stats );
  sgct::SharedData::instance()->writeBool( &takeScreenshot );
  sgct::SharedData::instance()->writeBool( &light );
}

void myDecodeFun() {
  sgct::SharedData::instance()->readDouble( &curr_time );
  sgct::SharedData::instance()->readVector( &sharedTransforms );
  sgct::SharedData::instance()->readString( &sharedText );
  sgct::SharedData::instance()->readBool( &wireframe );
  sgct::SharedData::instance()->readBool( &info );
  sgct::SharedData::instance()->readBool( &stats );
  sgct::SharedData::instance()->readBool( &takeScreenshot );
  sgct::SharedData::instance()->readBool( &light );
}

void myCleanUpFun() {
  sgct::MessageHandler::instance()->print("Cleaning up osg data...\n");
  delete mViewer;
  mViewer = NULL;
}

void keyCallback(int key, int action) {
  if(!gEngine->isMaster())
    return;

  
  switch (key) {
  case 'S':
    if(action == SGCT_PRESS)
      stats.toggle();
    break;

  case 'I':
    if(action == SGCT_PRESS)
      info.toggle();
    break;

  case 'L':
    if(action == SGCT_PRESS)
      light.toggle();
    break;

  case 'W':
    if(action == SGCT_PRESS)
      wireframe.toggle();
    break;

  case 'Q':
    if(action == SGCT_PRESS)
      gEngine->terminate();
    break;

  case 'P':
  case SGCT_KEY_F10:
    if(action == SGCT_PRESS)
      takeScreenshot.setVal( true );
    break;

  case SGCT_KEY_UP:
    arrowButtons[FORWARD] = ((action == SGCT_REPEAT || action == SGCT_PRESS) ? true : false);
    break;

  case SGCT_KEY_DOWN:
    arrowButtons[BACKWARD] = ((action == SGCT_REPEAT || action == SGCT_PRESS) ? true : false);
    break;
  }
}

void initOSG() {
  mRootNode = new osg::Group();
  osg::Referenced::setThreadSafeReferenceCounting(true);

  // Create the osgViewer instance
  mViewer = new osgViewer::Viewer;

  // Create a time stamp instance
  mFrameStamp    = new osg::FrameStamp();

  //run single threaded when embedded
  mViewer->setThreadingModel(osgViewer::Viewer::SingleThreaded);

  // Set up osgViewer::GraphicsWindowEmbedded for this context
  osg::GraphicsContext::Traits *traits =
    new osg::GraphicsContext::Traits;

  osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> graphicsWindow =
    new osgViewer::GraphicsWindowEmbedded(traits);

  mViewer->getCamera()->setGraphicsContext(graphicsWindow.get());

  //SGCT will handle the near and far planes
  mViewer->getCamera()->setComputeNearFarMode(osgUtil::CullVisitor::DO_NOT_COMPUTE_NEAR_FAR);
  mViewer->getCamera()->setClearColor( osg::Vec4( 0.0f, 0.0f, 0.0f, 0.0f) );

  //disable osg from clearing the buffers that will be done by SGCT
  GLbitfield tmpMask = mViewer->getCamera()->getClearMask();
  mViewer->getCamera()->setClearMask(tmpMask & (~(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)));

  mViewer->setSceneData(mRootNode.get());
}

void setupLightSource() {
  osg::Light * light0 = new osg::Light;
  osg::Light * light1 = new osg::Light;
  osg::LightSource* lightSource0 = new osg::LightSource;
  osg::LightSource* lightSource1 = new osg::LightSource;

  light0->setLightNum( 0 );
  light0->setPosition( osg::Vec4( 5.0f, 5.0f, 10.0f, 1.0f ) );
  light0->setAmbient( osg::Vec4( 0.3f, 0.3f, 0.3f, 1.0f ) );
  light0->setDiffuse( osg::Vec4( 0.8f, 0.8f, 0.8f, 1.0f ) );
  light0->setSpecular( osg::Vec4( 0.1f, 0.1f, 0.1f, 1.0f ) );
  light0->setConstantAttenuation( 1.0f );

  lightSource0->setLight( light0 );
  lightSource0->setLocalStateSetModes( osg::StateAttribute::ON );
  lightSource0->setStateSetModes( *(mRootNode->getOrCreateStateSet()), osg::StateAttribute::ON );

  light1->setLightNum( 1 );
  light1->setPosition( osg::Vec4( -5.0f, -2.0f, 10.0f, 1.0f ) );
  light1->setAmbient( osg::Vec4( 0.2f, 0.2f, 0.2f, 1.0f ) );
  light1->setDiffuse( osg::Vec4( 0.5f, 0.5f, 0.5f, 1.0f ) );
  light1->setSpecular( osg::Vec4( 0.2f, 0.2f, 0.2f, 1.0f ) );
  light1->setConstantAttenuation( 1.0f );

  lightSource1->setLight( light1 );
  lightSource1->setLocalStateSetModes( osg::StateAttribute::ON );
  lightSource1->setStateSetModes( *(mRootNode->getOrCreateStateSet()), osg::StateAttribute::ON );

  mRootNode->addChild( lightSource0 );
  mRootNode->addChild( lightSource1 );
}

osg::Geode* createWand() {

  osg::Geode* geode = new osg::Geode();

  linesGeom = new osg::Geometry();

  osg::Vec3Array* vertices = new osg::Vec3Array();
  vertices->push_back(osg::Vec3(0, 0, 0));
  vertices->push_back(osg::Vec3(1, 0, 0));
  linesGeom->setVertexArray(vertices);

  osg::Vec4Array* colors = new osg::Vec4Array;
  colors->push_back(osg::Vec4(0.3f,0.7f,0.4f,1.0f));
  linesGeom->setColorArray(colors, osg::Array::BIND_OVERALL);

  osg::Vec3Array* normals = new osg::Vec3Array;
  normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
  linesGeom->setNormalArray(normals, osg::Array::BIND_OVERALL);

  linesGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,2));
  geode->addDrawable(linesGeom);

  return geode;
}
