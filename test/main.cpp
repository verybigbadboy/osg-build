#include "pch.h"


class myEventHandler
  : public osgGA::GUIEventHandler
{
public:
  myEventHandler(btRigidBody* rb)
    : _rb(rb)
    , _centralImpulse(0, 0, 0)
    , _torqueImpulse(0, 0, 0)
    , _friction(1)
  {
  }


  virtual bool handle(osgGA::Event* event, osg::Object* object, osg::NodeVisitor* nv)
  {
    if (event->getHandled()) return false;

    osgGA::GUIEventAdapter *ea = event->asGUIEventAdapter();

    btVector3 x(0, 0.3, 0);
    btQuaternion q = _rb->getOrientation();
    switch(ea->getEventType())
    {
    case(osgGA::GUIEventAdapter::KEYDOWN):
      {
        _rb->activate(true);
        switch(ea->getUnmodifiedKey())
        {
        case osgGA::GUIEventAdapter::KEY_W:
          _centralImpulse = quatRotate(q, x);
          break;
        case osgGA::GUIEventAdapter::KEY_S:
          _centralImpulse = quatRotate(q, -x);
          break;
        case osgGA::GUIEventAdapter::KEY_A:
          _torqueImpulse = btVector3(0, 0, 0.3);
          break;
        case osgGA::GUIEventAdapter::KEY_D:
          _torqueImpulse = btVector3(0, 0, -0.3);
          break;
        case osgGA::GUIEventAdapter::KEY_E:
          _friction = btScalar(3);
          break;
        }
        break;
      }
    case(osgGA::GUIEventAdapter::KEYUP):
      {
        switch(ea->getUnmodifiedKey())
        {
        case osgGA::GUIEventAdapter::KEY_W:
          _centralImpulse = btVector3(0, 0, 0);
          break;
        case osgGA::GUIEventAdapter::KEY_S:
          _centralImpulse = btVector3(0, 0, 0);
          break;
        case osgGA::GUIEventAdapter::KEY_A:
          _torqueImpulse = btVector3(0, 0, 0);
          break;
        case osgGA::GUIEventAdapter::KEY_D:
          _torqueImpulse = btVector3(0, 0, 0);
          break;
        case osgGA::GUIEventAdapter::KEY_E:
          _friction = btScalar(1);
          break;
        }
        break;
      }
    case (osgGA::GUIEventAdapter::FRAME):
      {
        _rb->applyTorqueImpulse(_torqueImpulse);
        _rb->applyCentralImpulse(_centralImpulse);
        if (_rb->getFriction() != _friction)
          _rb->setFriction(_friction);
        break;
      }
    }

    return false;
  }

  btVector3 _centralImpulse;
  btVector3 _torqueImpulse;
  btScalar  _friction;

  btRigidBody* _rb;
};


btRigidBody* createObject( osg::Group* parent, const osg::Matrix& m, osgbInteraction::SaveRestoreHandler* srh)
{
  osg::Node* node = osgDB::readNodeFile("models/tank.osg");
  osg::MatrixTransform* mt = new osg::MatrixTransform;
  parent->addChild( mt );
  mt->addChild( node );

  osg::ref_ptr< osgbDynamics::CreationRecord > cr = new osgbDynamics::CreationRecord;
  cr->_sceneGraph = mt;
  cr->_shapeType = CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE;
  cr->_parentTransform = m;
  cr->_restitution = 1.f;
  btRigidBody* rb = osgbDynamics::createRigidBody( cr.get() );

  rb->applyCentralImpulse(btVector3(0,0,10));

  mt->setUserData( new osgbCollision::RefRigidBody( rb ) );
  std::ostringstream id;
  id << std::hex << mt;
  srh->add( id.str(), rb );

  return( rb );
}

btRigidBody* createWheel(osg::Group* parent, const osg::Matrix& m, osgbInteraction::SaveRestoreHandler* srh)
{
  osg::Geode* node = new osg::Geode(); 
  node->addDrawable(osgwTools::makeClosedCylinder(5));

  osg::MatrixTransform* mt = new osg::MatrixTransform;
  parent->addChild( mt );
  mt->addChild( node );

  osg::ref_ptr< osgbDynamics::CreationRecord > cr = new osgbDynamics::CreationRecord;
  cr->_sceneGraph = mt;
  cr->_shapeType = CYLINDER_SHAPE_PROXYTYPE;
  cr->_parentTransform = m;
  cr->_restitution = 1.f;
  btRigidBody* rb = osgbDynamics::createRigidBody( cr.get() );

  rb->applyCentralImpulse(btVector3(0,0,2));

  mt->setUserData( new osgbCollision::RefRigidBody( rb ) );
  std::ostringstream id;
  id << std::hex << mt;
  srh->add( id.str(), rb );

  return( rb );
}

btRigidBody* createCow(osg::Group* parent, const osg::Matrix& m, osgbInteraction::SaveRestoreHandler* srh)
{
  osg::Node* node = osgDB::readNodeFile("models/cow.osg");

  osg::MatrixTransform* mt = new osg::MatrixTransform;
  parent->addChild( mt );
  mt->addChild( node );

  osg::ref_ptr< osgbDynamics::CreationRecord > cr = new osgbDynamics::CreationRecord;
  cr->_sceneGraph = mt;
  cr->_shapeType = CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE;
  cr->_parentTransform = m;
  cr->_restitution = 1.f;
  btRigidBody* rb = osgbDynamics::createRigidBody( cr.get() );

  rb->applyCentralImpulse(btVector3(0,0,2));

  mt->setUserData( new osgbCollision::RefRigidBody( rb ) );
  std::ostringstream id;
  id << std::hex << mt;
  srh->add( id.str(), rb );

  return( rb );
}

btDynamicsWorld* initPhysics()
{
  btDefaultCollisionConfiguration * collisionConfiguration = new btDefaultCollisionConfiguration();
  btCollisionDispatcher * dispatcher = new btCollisionDispatcher( collisionConfiguration );
  btConstraintSolver * solver = new btSequentialImpulseConstraintSolver;

  btVector3 worldAabbMin( -10000, -10000, -10000 );
  btVector3 worldAabbMax( 10000, 10000, 10000 );
  btBroadphaseInterface * inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

  btDynamicsWorld * dynamicsWorld = new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );

  dynamicsWorld->setGravity( btVector3( 0, 0, -9.8 ) );

  return( dynamicsWorld );
}


int main( int argc, char** argv )
{
  btDynamicsWorld* bw = initPhysics();
  osg::Group* root = new osg::Group;

  osg::ref_ptr< osgbInteraction::SaveRestoreHandler > srh = new osgbInteraction::SaveRestoreHandler;

  osg::Matrix m;

  m = osg::Matrix::rotate( .4, 0., 0., 1. ) * osg::Matrix::translate( 16., 0., 10. );
  btRigidBody *rb = createObject(root, m, srh.get());
  bw->addRigidBody(rb);
  
  m = osg::Matrix::rotate( osg::PI_2, 0, 1, 0 ) * osg::Matrix::translate( 0., 0., 10. );
  bw->addRigidBody(createWheel(root, m, srh.get()));

  m = osg::Matrix::rotate( 0, 0., 0., 1. ) * osg::Matrix::translate( -20., 0., 0. );
  bw->addRigidBody(createCow(root, m, srh.get()));

  root->addChild( osgbDynamics::generateGroundPlane( osg::Vec4( 0.f, 0.f, 1.f, 0.f ), bw ) );

  osgViewer::Viewer viewer;
  viewer.setUpViewInWindow( 30, 30, 768, 480, 1 );
  viewer.setSceneData( root );
  osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator;
  viewer.setCameraManipulator( tb );

  viewer.realize();
  srh->capture();

  viewer.addEventHandler(new myEventHandler(rb));
  viewer.addEventHandler( srh.get() );
  viewer.addEventHandler( new osgbInteraction::DragHandler(bw, viewer.getCamera() ) );

  double prevSimTime = 0.;
  while( !viewer.done() )
  {
    const double currSimTime = viewer.getFrameStamp()->getSimulationTime();
    bw->stepSimulation( currSimTime - prevSimTime );
    prevSimTime = currSimTime;
    viewer.frame();
  }

  return( 0 );
}
