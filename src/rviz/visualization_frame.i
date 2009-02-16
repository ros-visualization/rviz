%{
#include "visualization_frame.h"
%}

%include std_string.i

%pythonAppend VisualizationFrame "self._setOORInfo(self)"

%include "generated/visualization_panel_generated.i"
%include "visualization_frame.h"

%init %{

%}

