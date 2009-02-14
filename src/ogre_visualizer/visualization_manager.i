%{
#include "visualization_manager.h"

#include "displays/axes_display.h"
#include "displays/grid_display.h"
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

%extend ogre_vis::VisualizationManager
{
  %template(createAxesDisplay) createDisplay<ogre_vis::AxesDisplay>;
  %template(createGridDisplay) createDisplay<ogre_vis::GridDisplay>;
  %template(createLaserScanDisplay) createDisplay<ogre_vis::LaserScanDisplay>;
  %template(createMarkerDisplay) createDisplay<ogre_vis::MarkerDisplay>;
  %template(createPlanningDisplay) createDisplay<ogre_vis::PlanningDisplay>;
  %template(createPointCloudDisplay) createDisplay<ogre_vis::PointCloudDisplay>;
  %template(createRobotModelDisplay) createDisplay<ogre_vis::RobotModelDisplay>;
  %template(createRobotBase2DPoseDisplay) createDisplay<ogre_vis::RobotBase2DPoseDisplay>;
  %template(createParticleCloud2DDisplay) createDisplay<ogre_vis::ParticleCloud2DDisplay>;
  %template(createPolyLine2DDisplay) createDisplay<ogre_vis::PolyLine2DDisplay>;
  %template(createPolygonalMapDisplay) createDisplay<ogre_vis::PolygonalMapDisplay>;
  %template(createCollisionMapDisplay) createDisplay<ogre_vis::CollisionMapDisplay>;
  %template(createMapDisplay) createDisplay<ogre_vis::MapDisplay>;
  %template(createTFDisplay) createDisplay<ogre_vis::TFDisplay>;
};

%init %{

%}

