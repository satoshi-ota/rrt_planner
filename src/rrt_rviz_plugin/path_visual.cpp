#include "path_visual.h"

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>
#include <pluginlib/class_list_macros.h>

namespace rrt_rviz_plugin
{

PathVisual::PathVisual()
{
    color_property_ = new rviz::ColorProperty( "Color", QColor( 200, 50, 50 ),
    "Color to draw the acceleration arrows.",
    this, SLOT( updateColorAndAlpha() ));

    alpha_property_ = new rviz::FloatProperty( "Alpha", 1.0,
    "0 is fully transparent, 1.0 is fully opaque.",
    this, SLOT( updateColorAndAlpha() ));
}

void PathVisual::onInitialize()
{
    frame_node_ = scene_node_->createChildSceneNode();
    vis_arrow_.reset(new rviz::Arrow( scene_manager_, frame_node_));
    float alpha = alpha_property_->getFloat();
    Ogre::ColourValue color = color_property_->getOgreColor();
    vis_arrow_->setColor(color.r, color.g, color.b, alpha);
    Ogre::Vector3 arrow_scale(0, 0, 0);
    vis_arrow_->setScale(arrow_scale);
    MFDClass::onInitialize(); // MFDClass := MessageFilterDisplay<message type>
}

PathVisual::~PathVisual(){ }

void PathVisual::reset()
{
    MFDClass::reset();
}

void PathVisual::updateColorAndAlpha()
{
    float alpha = alpha_property_->getFloat();
    Ogre::ColourValue color = color_property_->getOgreColor();
    vis_arrow_->setColor(color.r, color.g, color.b, alpha);
}

void PathVisual::updateHistoryLength(){ }

void PathVisual::processMessage(const rrt_planner::MarkerArrayStamped::ConstPtr& msg)
{
    if(0 < msg->markers.size())
    {
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;

    if( !context_->getFrameManager()->getTransform(msg->markers[0].header.frame_id, msg->markers[0].header.stamp, position, orientation)){
        ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", msg->markers[0].header.frame_id.c_str(), qPrintable( fixed_frame_ ));
        return;
    }

    frame_node_->setPosition(position);
    frame_node_->setOrientation(orientation);


        if(count_ < msg->markers.size()){
            Ogre::Vector3 arrow_dir(msg->markers[count_].points[1].x, msg->markers[count_].points[1].y, msg->markers[count_].points[1].z);
            float arrow_length = arrow_dir.length() * 0.77;
            Ogre::Vector3 arrow_scale(arrow_length, arrow_length, arrow_length);
            vis_arrow_->setScale(arrow_scale);
            vis_arrow_->setDirection(arrow_dir);
            count_++;
        } else {
            count_ = 0;

        }

        // ros::Duration(0.1).sleep();

    // Ogre::Vector3 arrow_dir(msg->markers[0].points[1].x, msg->markers[0].points[1].y, msg->markers[0].points[1].z);
    // float arrow_length = arrow_dir.length() * 0.77;
    // Ogre::Vector3 arrow_scale(arrow_length, arrow_length, arrow_length);
    // vis_arrow_->setScale(arrow_scale);
    // vis_arrow_->setDirection(arrow_dir);
    // Ogre::Vector3 start(msg->markers[0].points[0].x, msg->markers[0].points[0].y, msg->markers[0].points[0].z);
    // Ogre::Vector3 end(msg->markers[0].points[1].x, msg->markers[0].points[1].y, msg->markers[0].points[1].z);
    // vis_line_->setPoints(start, end);
    }
}

} // namespace rrt_rviz_plugin

PLUGINLIB_EXPORT_CLASS(rrt_rviz_plugin::PathVisual, rviz::Display)
