#include "quick_visualizer_app.h"

#include <QQmlEngine>
#include <QDebug>

#include <boost/program_options.hpp>

#include <ros/console.h>
#include <ros/ros.h>

#include "rviz/selection/selection_manager.h"
#include "rviz/env_config.h"
#include "rviz/ogre_helpers/ogre_logging.h"
#include "rviz/visualization_frame.h"
#include "rviz/visualization_manager.h"
#include "rviz/wait_for_master_dialog.h"
#include "rviz/ogre_helpers/render_system.h"
#include "rviz/quick_visualization_frame.h"
#include "rviz/ogre_helpers/qt_quick_ogre_render_window.h"

#define CATCH_EXCEPTIONS 0

namespace po = boost::program_options;

namespace rviz
{

QuickVisualizerApp::QuickVisualizerApp(QObject *parent)
  : QObject (parent)
{

}

bool QuickVisualizerApp::init(int argc, char **argv)
{
    ROS_INFO( "rviz version %s", get_version().c_str() );
    ROS_INFO( "compiled against Qt version " QT_VERSION_STR );
    ROS_INFO( "compiled against OGRE version %d.%d.%d%s (%s)",
              OGRE_VERSION_MAJOR, OGRE_VERSION_MINOR, OGRE_VERSION_PATCH,
              OGRE_VERSION_SUFFIX, OGRE_VERSION_NAME );

  #ifdef Q_OS_MAC
    ProcessSerialNumber PSN;
    GetCurrentProcess(&PSN);
    TransformProcessType(&PSN,kProcessTransformToForegroundApplication);
    SetFrontProcess(&PSN);
  #endif

  #if CATCH_EXCEPTIONS
    try
    {
  #endif
      ros::init( argc, argv, "rviz", ros::init_options::AnonymousName );

      startContinueChecker();

      po::options_description options;
      options.add_options()
        ("help,h", "Produce this help message")
        ("splash-screen,s", po::value<std::string>(), "A custom splash-screen image to display")
        ("help-file", po::value<std::string>(), "A custom html file to show as the help screen")
        ("display-config,d", po::value<std::string>(), "A display config file (.rviz) to load")
        ("fixed-frame,f", po::value<std::string>(), "Set the fixed frame")
        ("ogre-log,l", "Enable the Ogre.log file (output in cwd) and console output.")
        ("in-mc-wrapper", "Signal that this is running inside a master-chooser wrapper")
        ("opengl", po::value<int>(), "Force OpenGL version (use '--opengl 210' for OpenGL 2.1 compatibility mode)")
        ("disable-anti-aliasing", "Prevent rviz from trying to use anti-aliasing when rendering.")
        ("no-stereo", "Disable the use of stereo rendering.")
        ("verbose,v", "Enable debug visualizations")
        ("log-level-debug", "Sets the ROS logger level to debug.");
      po::variables_map vm;
      std::string display_config, fixed_frame, splash_path, help_path;
      bool enable_ogre_log = false;
      bool in_mc_wrapper = false;
      bool verbose = false;
      int force_gl_version = 0;
      bool disable_anti_aliasing = false;
      bool disable_stereo = false;
      try
      {
        po::store( po::parse_command_line( argc, argv, options ), vm );
        po::notify( vm );

        if( vm.count( "help" ))
        {
          std::cout << "rviz command line options:\n" << options;
          return false;
        }

        if( vm.count( "in-mc-wrapper" ))
        {
          in_mc_wrapper = true;
        }

        if (vm.count("display-config"))
        {
          display_config = vm["display-config"].as<std::string>();
          if( display_config.substr( display_config.size() - 4, 4 ) == ".vcg" )
          {
            std::cerr << "ERROR: the config file '" << display_config << "' is a .vcg file, which is the old rviz config format." << std::endl;
            std::cerr << "       New config files have a .rviz extension and use YAML formatting.  The format changed" << std::endl;
            std::cerr << "       between Fuerte and Groovy.  There is not (yet) an automated conversion program." << std::endl;
            return false;
          }
        }

        if (vm.count("splash-screen"))
        {
          splash_path = vm["splash-screen"].as<std::string>();
        }

        if (vm.count("help-file"))
        {
          help_path = vm["help-file"].as<std::string>();
        }

        if (vm.count("fixed-frame"))
        {
          fixed_frame = vm["fixed-frame"].as<std::string>();
        }

        if (vm.count("ogre-log"))
        {
          enable_ogre_log = true;
        }

        if (vm.count("no-stereo"))
        {
          disable_stereo = true;
        }

        if (vm.count("opengl"))
        {
          //std::cout << vm["opengl"].as<std::string>() << std::endl;
          force_gl_version = vm["opengl"].as<int>();
        }

        if (vm.count("disable-anti-aliasing"))
        {
          disable_anti_aliasing = true;
        }

        if (vm.count("verbose"))
        {
          verbose = true;
        }

        if (vm.count("log-level-debug"))
        {
          if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
          {
            ros::console::notifyLoggerLevelsChanged();
          }
        }
      }
      catch (std::exception& e)
      {
        ROS_ERROR("Error parsing command line: %s", e.what());
        return false;
      }

      if( !ros::master::check() )
      {
        WaitForMasterDialog* dialog = new WaitForMasterDialog;
        if( dialog->exec() != QDialog::Accepted )
        {
          return false;
        }
        delete dialog;
      }

      nh_.reset( new ros::NodeHandle );

      if( enable_ogre_log )
      {
        OgreLogging::useRosLog();
      }

      if ( force_gl_version )
      {
        RenderSystem::forceGlVersion( force_gl_version );
      }

      if (disable_anti_aliasing)
      {
        RenderSystem::disableAntiAliasing();
      }

      if ( disable_stereo )
      {
        RenderSystem::forceNoStereo();
      }

  #if CATCH_EXCEPTIONS
    }
    catch (std::exception& e)
    {
      ROS_ERROR("Caught exception while loading: %s", e.what());
      return false;
    }
  #endif
    return true;
}

void QuickVisualizerApp::startContinueChecker()
{
  connect( &continue_timer_, &QTimer::timeout, this, &QuickVisualizerApp::checkContinue );
  continue_timer_.start( 100 );
}

void QuickVisualizerApp::checkContinue()
{
    if (!ros::ok()) {
        qDebug() << "ROS stopped";
    }
}

} // end namespace rviz
