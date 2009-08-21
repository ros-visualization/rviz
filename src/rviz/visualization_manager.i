%{
#include "visualization_manager.h"
%}

%include std_string.i

%pythonAppend VisualizationManager "self._setOORInfo(self)"

%include "visualization_manager.h"


%init %{

%}

