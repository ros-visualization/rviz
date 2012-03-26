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

  void loadDisplayConfig( const QString& filepath );
  void setViewControllerType( const QString& view_type_name );
  void setViewString( const QString& view_string );
  void setTargetFrame( const QString& target_frame );
};

};
