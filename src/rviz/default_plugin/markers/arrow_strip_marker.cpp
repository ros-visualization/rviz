
#include <rviz/ogre_helpers/ogre_vector.h>
#include <OgreQuaternion.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>

#include <rviz/default_plugin/marker_display.h>
#include <rviz/default_plugin/markers/marker_selection_handler.h>
#include <rviz/display_context.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/validate_floats.h>

#include <rviz/default_plugin/markers/arrow_strip_marker.h>

namespace rviz {
    ArrowStripMarker::ArrowStripMarker(MarkerDisplay* owner, DisplayContext* context, Ogre::SceneNode* parent_node): MarkerBase(owner, context, parent_node) {

    }

    ArrowStripMarker::~ArrowStripMarker() {
        for (Arrow* arrow: arrows_) {
            delete arrow;
        }
    }

    void ArrowStripMarker::onNewMessage(const MarkerConstPtr& /*old_message*/, const MarkerConstPtr& new_message) {
        ROS_ASSERT(new_message->type == visualization_msgs::Marker::ARROW_STRIP);

        Ogre::Vector3 pos, scale;
        Ogre::Quaternion orient;
        if (!transform(new_message, pos, orient, scale))
        {
            scene_node_->setVisible(false);
            return;
        }

        scene_node_->setVisible(true);
        setPosition(pos);
        setOrientation(orient);

        for (Arrow* arrow: arrows_) {
            delete arrow;
            arrows_.clear();
        }

        handler_.reset(new MarkerSelectionHandler(this, MarkerID(new_message->ns, new_message->id), context_));
        if (new_message->points.size() < 2)
        {
            return;
        }
        
        geometry_msgs::Point p = new_message->points.at(0);
        Ogre::Vector3 arrow_start(p.x, p.y, p.z);
        for (int i = 1; i < new_message->points.size(); i++) {
            p = new_message->points.at(i);
            Ogre::Vector3 arrow_end(p.x, p.y, p.z);
            if (!validateFloats(p))
            {
                ROS_WARN("Marker '%s/%d': invalid point[%d] (%.2f, %.2f, %.2f)", new_message->ns.c_str(),
                        new_message->id, i, p.x, p.y, p.z);
                continue;
            }

            arrows_.push_back(new Arrow(context_->getSceneManager(), scene_node_));
            Ogre::Vector3 direction = arrow_end - arrow_start;
            float distance = direction.length();
            float head_length_proportion = 0.23;
            float head_length = head_length_proportion * distance;
            if (new_message->scale.z != 0.0)
            {
                float length = new_message->scale.z;
                head_length = std::max<double>(0.0, std::min<double>(length, distance)); // clamp
            }
            float shaft_length = distance - head_length;
            arrows_.back()->set(shaft_length, new_message->scale.x, head_length, new_message->scale.y);
            arrows_.back()->setPosition(arrow_start);
            arrows_.back()->setDirection(direction.normalisedCopy());
            arrows_.back()->setColor(new_message->color.r, new_message->color.g, new_message->color.b, new_message->color.a);
            handler_->addTrackedObjects(arrows_.back()->getSceneNode());
            arrow_start = arrow_end;
        }
    }
    
} // namespace rviz