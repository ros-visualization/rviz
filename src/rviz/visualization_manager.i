%{
#include "visualization_manager.h"

#include "displays/axes_display.h"
#include "displays/grid_display.h"
#include "displays/point_cloud_base.h"
#include "displays/laser_scan_display.h"
#include "displays/marker_display.h"
#include "displays/planning_display.h"
#include "displays/point_cloud_display.h"
#include "displays/robot_model_display.h"
#include "displays/robot_base2d_pose_display.h"
#include "displays/particle_cloud_2d_display.h"
#include "displays/poly_line_2d_display.h"
#include "displays/polygonal_map_display.h"
#include "displays/collision_map_display.h"
#include "displays/map_display.h"
#include "displays/tf_display.h"
%}

%include std_string.i

%include "displays/axes_display.h"
%include "displays/grid_display.h"
%include "displays/point_cloud_base.h"
%include "displays/laser_scan_display.h"
%include "displays/marker_display.h"
%include "displays/planning_display.h"
%include "displays/point_cloud_display.h"
%include "displays/robot_model_display.h"
%include "displays/robot_base2d_pose_display.h"
%include "displays/particle_cloud_2d_display.h"
%include "displays/poly_line_2d_display.h"
%include "displays/polygonal_map_display.h"
%include "displays/collision_map_display.h"
%include "displays/map_display.h"
%include "displays/tf_display.h"

%pythonAppend VisualizationManager "self._setOORInfo(self)"

%include "visualization_manager.h"

%extend rviz::VisualizationManager
{
  %template(createAxesDisplay) createDisplay<rviz::AxesDisplay>;
  %template(createGridDisplay) createDisplay<rviz::GridDisplay>;
  %template(createLaserScanDisplay) createDisplay<rviz::LaserScanDisplay>;
  %template(createMarkerDisplay) createDisplay<rviz::MarkerDisplay>;
  %template(createPlanningDisplay) createDisplay<rviz::PlanningDisplay>;
  %template(createPointCloudDisplay) createDisplay<rviz::PointCloudDisplay>;
  %template(createRobotModelDisplay) createDisplay<rviz::RobotModelDisplay>;
  %template(createRobotBase2DPoseDisplay) createDisplay<rviz::RobotBase2DPoseDisplay>;
  %template(createParticleCloud2DDisplay) createDisplay<rviz::ParticleCloud2DDisplay>;
  %template(createPolyLine2DDisplay) createDisplay<rviz::PolyLine2DDisplay>;
  %template(createPolygonalMapDisplay) createDisplay<rviz::PolygonalMapDisplay>;
  %template(createCollisionMapDisplay) createDisplay<rviz::CollisionMapDisplay>;
  %template(createMapDisplay) createDisplay<rviz::MapDisplay>;
  %template(createTFDisplay) createDisplay<rviz::TFDisplay>;
};

%init %{

%}

