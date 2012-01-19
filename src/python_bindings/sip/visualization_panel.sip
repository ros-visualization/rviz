namespace rviz
{

class VisualizationPanel: QSplitter /NoDefaultCtors/
{
%TypeHeaderCode
#include <rviz/visualization_panel.h>
%End

public:
  VisualizationPanel( QWidget* parent = 0 );
  ~VisualizationPanel();

  void loadGeneralConfig( const std::string& filepath );
  void loadDisplayConfig( const std::string& filepath );
};

};
