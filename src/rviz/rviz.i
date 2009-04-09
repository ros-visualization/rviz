%module rviz
%include "std_string.i"
%include "std_vector.i"

%{
#include "wx/wxPython/wxPython.h"
#include "wx/wxPython/pyclasses.h"
%}

%include typemaps.i
%include my_typemaps.i

%import core.i
%import windows.i

%include "helpers/color.i"
%include "display.i"
%include "visualization_manager.i"
%include "visualization_panel.i"
%include "visualization_frame.i"

%include "helpers/color.i"

%init %{

%}
