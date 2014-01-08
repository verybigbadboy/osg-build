/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2013 Robert Osfield
 *
 * This library is open source and may be redistributed and/or modified under
 * the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or
 * (at your option) any later version.  The full license is in LICENSE file
 * included with this distribution, and on the openscenegraph.org website.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * OpenSceneGraph Public License for more details.
*/

#include "Widget.h"

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osg/io_utils>

#include <osgGA/EventVisitor>
#include <osgViewer/View>

using namespace osgUI;

Widget::Widget():
    _focusBehaviour(FOCUS_FOLLOWS_POINTER),
    _hasEventFocus(false),
    _graphicsInitialized(false)

{
    setNumChildrenRequiringEventTraversal(1);
}

Widget::Widget(const Widget& widget, const osg::CopyOp& copyop):
    osg::Group(),
    _focusBehaviour(widget._focusBehaviour),
    _hasEventFocus(false),
    _graphicsInitialized(false)
{
    setNumChildrenRequiringEventTraversal(1);
}

void Widget::setExtents(const osg::BoundingBox& bb)
{
    _extents = bb;
}

void Widget::updateFocus(osg::NodeVisitor& nv)
{
    osgGA::EventVisitor* ev = dynamic_cast<osgGA::EventVisitor*>(&nv);
    osgViewer::View* view = ev ? dynamic_cast<osgViewer::View*>(ev->getActionAdapter()) : 0;
    if (ev && view)
    {
        osgGA::EventQueue::Events& events = ev->getEvents();
        for(osgGA::EventQueue::Events::iterator itr = events.begin();
            itr != events.end();
            ++itr)
        {
            osgGA::GUIEventAdapter* ea = (*itr)->asGUIEventAdapter();
            if (ea)
            {
                bool previousFocus = _hasEventFocus;
                if (_focusBehaviour==CLICK_TO_FOCUS)
                {
                    if (ea->getEventType()==osgGA::GUIEventAdapter::PUSH)
                    {
                        int numButtonsPressed = 0;
                        if (ea->getButtonMask()&osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) ++numButtonsPressed;
                        if (ea->getButtonMask()&osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON) ++numButtonsPressed;
                        if (ea->getButtonMask()&osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON) ++numButtonsPressed;

                        if (numButtonsPressed==1)
                        {
                            osgUtil::LineSegmentIntersector::Intersections intersections;
                            bool withinWidget = view->computeIntersections(*ea, nv.getNodePath(), intersections);
                            if (withinWidget) _hasEventFocus = true;
                            else _hasEventFocus = false;
                        }
                    }
                }
                else if (_focusBehaviour==FOCUS_FOLLOWS_POINTER)
                {
                    bool checkWithinWidget = false;
                    if (!_hasEventFocus)
                    {
                        checkWithinWidget = (ea->getEventType()!=osgGA::GUIEventAdapter::FRAME) && ea->getButtonMask()==0;
                    }
                    else
                    {
                        // if mouse move or mouse release check to see if mouse still within widget to retain focus
                        if (ea->getEventType()==osgGA::GUIEventAdapter::MOVE)
                        {
                            checkWithinWidget = true;
                        }
                        else if (ea->getEventType()==osgGA::GUIEventAdapter::RELEASE)
                        {
                            // if no buttons pressed then check
                            if (ea->getButtonMask()==0) checkWithinWidget = true;
                        }
                    }

                    if (checkWithinWidget)
                    {
                        osgUtil::LineSegmentIntersector::Intersections intersections;
                        bool withinWidget = view->computeIntersections(*ea, nv.getNodePath(), intersections);

                        _hasEventFocus = withinWidget;
                    }
                }

                if (previousFocus != _hasEventFocus)
                {
                    if (_hasEventFocus)
                    {
                        enter();
#if 0
                        if (view->getCameraManipulator())
                        {
                            view->getCameraManipulator()->finishAnimation();
                            view->requestContinuousUpdate( false );
                        }
#endif
                    }
                    else
                    {
                        leave();
                    }
                }

            }
        }
    }
}

void Widget::setHasEventFocus(bool focus)
{
    if (_hasEventFocus == focus) return;

    _hasEventFocus = focus;

    if (_hasEventFocus) enter();
    else leave();
}

bool Widget::getHasEventFocus() const
{
    return _hasEventFocus;
}

void Widget::enter()
{
    OSG_NOTICE<<"enter()"<<std::endl;
}

void Widget::leave()
{
    OSG_NOTICE<<"leave()"<<std::endl;
}

void Widget::traverse(osg::NodeVisitor& nv)
{
    if (!_graphicsInitialized && nv.getVisitorType()!=osg::NodeVisitor::CULL_VISITOR) createGraphics();

    osgGA::EventVisitor* ev = dynamic_cast<osgGA::EventVisitor*>(&nv);
    osgViewer::View* view = ev ? dynamic_cast<osgViewer::View*>(ev->getActionAdapter()) : 0;
    if (ev && view)
    {
        updateFocus(nv);

        if (getHasEventFocus())
        {
            // signify that event has been taken by widget with focus
            ev->setEventHandled(true);

            osgGA::EventQueue::Events& events = ev->getEvents();
            for(osgGA::EventQueue::Events::iterator itr = events.begin();
                itr != events.end();
                ++itr)
            {
                handle(ev, itr->get());

                (*itr)->setHandled(true);
            }
        }
    }
    else
    {
        osg::Group::traverse(nv);
    }
}

bool Widget::handle(osgGA::EventVisitor* /*ev*/, osgGA::Event* /*event*/)
{
    return false;
}

bool Widget::computePositionInLocalCoordinates(osgGA::EventVisitor* ev, osgGA::GUIEventAdapter* event, osg::Vec3& localPosition) const
{
    osgViewer::View* view = ev ? dynamic_cast<osgViewer::View*>(ev->getActionAdapter()) : 0;
    osgUtil::LineSegmentIntersector::Intersections intersections;
    if (view && view->computeIntersections(*event, ev->getNodePath(), intersections))
    {
        localPosition = intersections.begin()->getLocalIntersectPoint();

        return (_extents.contains(localPosition, 1e-6));
    }
    else
    {
        return false;
    }
}


void Widget::createGraphics()
{
    _graphicsInitialized = true;
}

osg::BoundingSphere Widget::computeBound() const
{
    return osg::BoundingSphere(_extents);
}
