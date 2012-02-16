namespace rviz
{

class VisualizationFrame: QMainWindow /NoDefaultCtors/
{
%TypeHeaderCode
#include <rviz/visualization_frame.h>
%End

public:

  VisualizationFrame( QWidget* parent = 0 );
  virtual ~VisualizationFrame();

  void initialize( const std::string& display_config_file = "",
                   const std::string& fixed_frame = "",
                   const std::string& target_frame = "",
                   const std::string& splash_path = "",
                   const std::string& help_path = "",
                   bool verbose = false,
                   bool show_choose_new_master_option = false );
};

};
