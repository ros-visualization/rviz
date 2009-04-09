%{
#include "generated/rviz_generated.h"
%}

%include typemaps.i
%include my_typemaps.i

%import core.i
%import windows.i

%pythonAppend VisualizationPanelGenerated "self._setOORInfo(self)"

%include "rviz_generated.h"

%init %{

%}


