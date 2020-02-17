#ifndef RRT_RVIZ_PLUGIN_PATH_VISUAL_H
#define RRT_RVIZ_PLUGIN_PATH_VISUAL_H

#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>
#include <rviz/message_filter_display.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/line.h>
#include <rrt_planner/MarkerArrayStamped.h>
#endif

namespace Ogre{
    class SceneNode;
}

namespace rviz{
class ColorProperty;
class FloatProperty;
class IntProperty;
}

namespace rrt_rviz_plugin
{

class PathVisual: public rviz::MessageFilterDisplay<rrt_planner::MarkerArrayStamped>
{
    Q_OBJECT
public:
    PathVisual();
    virtual ~PathVisual();

    virtual void onInitialize();
    virtual void reset();

    private Q_SLOTS:
    void updateColorAndAlpha();
    void updateHistoryLength();

private:
    void processMessage(const rrt_planner::MarkerArrayStamped::ConstPtr& msg);
    Ogre::SceneNode* frame_node_;
    boost::shared_ptr<rviz::Arrow> vis_arrow_;
    boost::shared_ptr<rviz::Line> vis_line_;
    rviz::ColorProperty* color_property_;
    rviz::FloatProperty* alpha_property_;

    int count_{0};
};

} // namespace rrt_rviz_plugin

#endif // RRT_RVIZ_PLUGIN_PATH_VISUAL_H
