#ifndef EFFORT_DISPLAY_H
#define EFFORT_DISPLAY_H

#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>
#endif

#include <sensor_msgs/JointState.h>
#include <rviz/message_filter_display.h>

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class FloatProperty;
class IntProperty;
class StringProperty;
class CategoryProperty;
class BoolProperty;
} // namespace rviz

namespace urdf
{
class Model;
}

#ifndef Q_MOC_RUN
#include <message_filters/subscriber.h>
#endif

#include <rviz/message_filter_display.h>

namespace rviz
{
class JointInfo : public QObject
{
  Q_OBJECT
public:
  JointInfo(const std::string& name, rviz::Property* parent_category);
  ~JointInfo() override;

  void setEffort(double e);
  inline double getEffort()
  {
    return effort_;
  }
  void setMaxEffort(double m);
  inline double getMaxEffort()
  {
    return max_effort_;
  }
  bool getEnabled() const;

  ros::Time last_update_;

public Q_SLOTS:
  void updateVisibility();

private:
  std::string name_;
  double effort_, max_effort_;

  rviz::Property* category_;
  rviz::FloatProperty* effort_property_;
  rviz::FloatProperty* max_effort_property_;
};

typedef std::set<JointInfo*> S_JointInfo;
typedef std::vector<std::string> V_string;

class EffortVisual;

class EffortDisplay : public rviz::MessageFilterDisplay<sensor_msgs::JointState>
{
  Q_OBJECT
public:
  EffortDisplay();
  ~EffortDisplay() override;

  // Overrides of public virtual functions from the Display class.
  void onInitialize() override;
  void reset() override;

private Q_SLOTS:
  // Helper function to apply color and alpha to all visuals.
  void updateColorAndAlpha();
  void updateHistoryLength();
  void updateRobotDescription();
  void updateTfPrefix();

  JointInfo* getJointInfo(const std::string& joint);
  JointInfo* createJoint(const std::string& joint);

protected:
  // overrides from Display
  void onEnable() override;
  void onDisable() override;

  // load
  using MFDClass::load;
  void load();
  void clear();

  // The object for urdf model
  boost::shared_ptr<urdf::Model> robot_model_;

  std::string robot_description_;

private:
  void processMessage(const sensor_msgs::JointState::ConstPtr& msg) override;

  // Storage for the list of visuals.  It is a circular buffer where
  // data gets popped from the front (oldest) and pushed to the back (newest)
  boost::circular_buffer<boost::shared_ptr<EffortVisual> > visuals_;

  typedef std::map<std::string, JointInfo*> M_JointInfo;
  M_JointInfo joints_;

  // Property objects for user-editable properties.
  rviz::FloatProperty *alpha_property_, *width_property_, *scale_property_;
  rviz::IntProperty* history_length_property_;

  rviz::StringProperty* robot_description_property_;
  rviz::StringProperty* tf_prefix_property_;
  rviz::Property* joints_category_;
  rviz::BoolProperty* all_enabled_property_;
};
} // namespace rviz

#endif // EFFORT_DISPLAY_H
