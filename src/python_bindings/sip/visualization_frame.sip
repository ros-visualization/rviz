namespace rviz
{

class VisualizationFrame: QMainWindow /NoDefaultCtors/
{
%TypeHeaderCode
#define ROS_PACKAGE_NAME "rviz"
#include <rviz/visualization_frame.h>
%End

public:

  VisualizationFrame( QWidget* parent = 0 );
  virtual ~VisualizationFrame();

  /** @brief Call this @e before initialize() to have it take effect. */
  void setShowChooseNewMaster( bool show );

  /** @brief Set the path to the help file.  Should contain HTML.
   * Default is a file in the RViz package. */
  void setHelpPath( const QString& help_path );

  /** @brief Set the path to the "splash" image file.  This image is
   * shown during initialization and loading of the first config file.
   * Default is a file in the RViz package.  To prevent splash image
   * from showing, set this to an empty string. */
  void setSplashPath( const QString& splash_path );

  void initialize( const QString& display_config_file = "" );

//  VisualizationManager* getManager() { return manager_; }

  // overrides from WindowManagerInterface
  virtual QWidget* getParentWindow();
  virtual QDockWidget* addPane( const QString& name,
                                QWidget* panel,
                                Qt::DockWidgetArea area = Qt::LeftDockWidgetArea,
                                bool floating = true );

  /** @brief Load the "general" config file, which has just the few
   * things which should not be saved with a display config.
   *
   * Loads from the file named in persistent_settings_file_. */
  void loadPersistentSettings();

  /** @brief Save the "general" config file, which has just the few
   * things which should not be saved with a display config.
   *
   * Saves to the file named in persistent_settings_file_. */
  void savePersistentSettings();

  /** @brief Load display and other settings from the given file.
   * @param path The full path of the config file to load from. */
  void loadDisplayConfig( const QString& path );

  /** @brief Save display and other settings to the given file.
   * @param path The full path of the config file to save into. */
  void saveDisplayConfig( const QString& path );
};

};
