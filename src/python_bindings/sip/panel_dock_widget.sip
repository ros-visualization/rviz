namespace rviz
{

class PanelDockWidget: QDockWidget /NoDefaultCtors/
{
%TypeHeaderCode
#include <rviz/panel_dock_widget.h>
%End

public:
  PanelDockWidget( const QString& name );

  void setContentWidget( QWidget* child );

signals:

  void visibilityChanged( bool );

};

};
