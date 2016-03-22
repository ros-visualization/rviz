// -*- c++-mode -*-
#ifndef POSE_WITH_COVARIANCE_DISPLAY_H
#define POSE_WITH_COVARIANCE_DISPLAY_H

#include <boost/shared_ptr.hpp>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "rviz/message_filter_display.h"
#include "rviz/selection/forwards.h"

namespace rviz
{
class Arrow;
class Axes;
class ColorProperty;
class EnumProperty;
class FloatProperty;
class BoolProperty;
class Shape;

class CovarianceVisual;
class CovarianceProperty;

class PoseWithCovarianceDisplaySelectionHandler;
typedef boost::shared_ptr<PoseWithCovarianceDisplaySelectionHandler> PoseWithCovarianceDisplaySelectionHandlerPtr;

/** @brief Displays the pose from a geometry_msgs::PoseWithCovarianceStamped message. */
class PoseWithCovarianceDisplay: public MessageFilterDisplay<geometry_msgs::PoseWithCovarianceStamped>
{
Q_OBJECT
public:
  enum Shape
  {
    ArrowShape,
    AxesShape,
  };

  PoseWithCovarianceDisplay();
  virtual ~PoseWithCovarianceDisplay();

  virtual void onInitialize();
  virtual void reset();

protected:
  /** @brief Overridden from MessageFilterDisplay to get Arrow/Axes visibility correct. */
  virtual void onEnable();

private Q_SLOTS:
  void updateShapeVisibility();
  void updateColorAndAlpha();
  void updateShapeChoice();
  void updateAxisGeometry();
  void updateArrowGeometry();

private:
  void clear();

  virtual void processMessage( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& message );

  Arrow* arrow_;
  Axes* axes_;
  boost::shared_ptr<CovarianceVisual> covariance_;
  bool pose_valid_;
  PoseWithCovarianceDisplaySelectionHandlerPtr coll_handler_;

  EnumProperty* shape_property_;

  ColorProperty* color_property_;
  FloatProperty* alpha_property_;

  FloatProperty* head_radius_property_;
  FloatProperty* head_length_property_;
  FloatProperty* shaft_radius_property_;
  FloatProperty* shaft_length_property_;

  FloatProperty* axes_length_property_;
  FloatProperty* axes_radius_property_;

  CovarianceProperty* covariance_property_;

  friend class PoseWithCovarianceDisplaySelectionHandler;
};

} // namespace rviz

#endif // POSE_WITH_COVARIANCE_DISPLAY_H
