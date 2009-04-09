%{
#include "display.h"
%}

%include typemaps.i
%include my_typemaps.i

%import core.i
%import windows.i

%pythonAppend VisualizerBase "self._setOORInfo(self)"

%include "properties/forwards.h"
%include "display.h"

%init %{

%}


