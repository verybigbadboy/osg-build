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

#ifndef OSGUI_WIDGET
#define OSGUI_WIDGET

#include <osg/Group>
#include <osg/BoundingBox>
#include <osgGA/Event>
#include <osgGA/EventVisitor>

#define OSGUI_EXPORT

namespace osgUI
{

class OSGUI_EXPORT Widget : public osg::Group
{
public:
    Widget();
    Widget(const Widget& tfw, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY);
    META_Node(osgUI, Widget);

    virtual void traverse(osg::NodeVisitor& nv);

    virtual bool handle(osgGA::EventVisitor* ev, osgGA::Event* event);

    virtual bool computePositionInLocalCoordinates(osgGA::EventVisitor* ev, osgGA::GUIEventAdapter* event, osg::Vec3& localPosition) const;

    virtual void createGraphics();

    virtual void setExtents(const osg::BoundingBox& bb);
    const osg::BoundingBox& getExtents() const { return _extents; }

    enum FocusBehaviour
    {
        CLICK_TO_FOCUS,
        FOCUS_FOLLOWS_POINTER,
        EVENT_DRIVEN_FOCUS_DISABLED
    };

    void setFocusBehaviour(FocusBehaviour behaviour) { _focusBehaviour = behaviour; }
    FocusBehaviour getFocusBehaviour() const { return _focusBehaviour; }

    /** update the focus according to events.*/
    virtual void updateFocus(osg::NodeVisitor& nv);

    /** set whether the widget has focus or not.*/
    virtual void setHasEventFocus(bool focus);

    /** get whether the widget has focus or not.*/
    virtual bool getHasEventFocus() const;

    virtual osg::BoundingSphere computeBound() const;

protected:
    virtual ~Widget() {}

    /** update any focus related graphics+state to the focused state.*/
    virtual void enter();

    /** update any focus related graphics+state to the unfocused state.*/
    virtual void leave();

    FocusBehaviour      _focusBehaviour;
    bool                _hasEventFocus;
    bool                _graphicsInitialized;

    osg::BoundingBox    _extents;
};

}

#endif
