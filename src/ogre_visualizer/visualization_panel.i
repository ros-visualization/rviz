%{
#include "visualization_panel.h"
%}

%include std_string.i

%pythonAppend VisualizationPanel "self._setOORInfo(self)"

%include "generated/visualization_panel_generated.i"
%include "visualization_panel.h"

%init %{

%}

