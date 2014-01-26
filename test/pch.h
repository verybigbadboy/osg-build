

#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osg/MatrixTransform>
#include <osgGA/TrackballManipulator>
#include <osg/ShapeDrawable>
#include <osgDB/WriteFile>
#include <osgwTools/Shapes.h>
#include <osgbDynamics/GroundPlane.h>
#include <osgbDynamics/MotionState.h>
#include <osgbDynamics/RigidBody.h>
#include <osgbCollision/RefBulletObject.h>
#include <osgbCollision/Utils.h>
#include <osgbInteraction/SaveRestoreHandler.h>
#include <osgbInteraction/DragHandler.h>

#include <btBulletDynamicsCommon.h>
#include <sstream>