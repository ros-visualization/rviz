#!/usr/bin/env python

import roslib
import sys
setattr(sys, 'SELECT_QT_BINDING', 'pyqt')
from python_qt_binding import QT_BINDING

from QtGui import *
from QtCore import *
from rviz import bindings as rviz

c = rviz.Config()
c.mapSetValue( "foo", 17 )
c.mapSetValue( "bar", "seventeen" )
c.mapMakeChild( "biff" ).mapMakeChild( "boff" ).setValue( 3.14159 )

print c.mapGetChild( "foo" ).getValue()
print c.mapGetChild( "bar" ).getValue()
print c.mapGetChild( "baz" ).getValue()
print c.mapGetChild( "biff" ).mapGetChild( "boff" ).getValue()
# print c.mapGetChild( "goo" ).mapGetChild( "biff" ).getValue() # crashes because "goo" does not exist, so getChild("biff") can't be called.

mi = c.mapIterator()
while mi.isValid():
    print "key:", mi.currentKey(), " value:", mi.currentChild().getValue()
    mi2 = mi.currentChild().mapIterator()
    while mi2.isValid():
        print "  key:", mi2.currentKey(), " value:", mi2.currentChild().getValue()
        mi2.advance()
    mi.advance()

c.listAppendNew().setValue( "ay" )
c.listAppendNew().setValue( "bee" )
if c.getType() == rviz.Config.List:
    for i in range( 0, c.listLength() ):
        print c.listChildAt( i ).getValue()

c.mapMakeChild( "chunk" )
print "config has", c.listLength(), "list entries. (should be 0 because it is a map now.)"

r = rviz.YamlConfigReader()
c = rviz.Config()
r.readFile( c, roslib.packages.get_pkg_dir('rviz') + "/default.rviz" )
if r.error():
    print "Error:", r.errorMessage()
else:
    print "default.rviz first two levels of maps:"
    mi = c.mapIterator()
    while mi.isValid():
        print "key:", mi.currentKey(), " value:", mi.currentChild().getValue()
        mi2 = mi.currentChild().mapIterator()
        while mi2.isValid():
            print "  key:", mi2.currentKey(), " value:", mi2.currentChild().getValue()
            mi2.advance()
        mi.advance()

    print "tools:"
    tools = c.mapGetChild( "Visualization Manager" ).mapGetChild( "Tools" )
    for i in range( 0, tools.listLength() ):
        print "  class:", tools.listChildAt( i ).mapGetChild( "Class" ).getValue()

    w = rviz.YamlConfigWriter()
    first_string = w.writeString( c )
    print "Entire default.rviz written to a string:"
    print first_string

    r2 = rviz.YamlConfigReader()
    c2 = rviz.Config()
    r2.readString( c2, first_string )
    second_string = w.writeString( c2 )

    if first_string == second_string:
        print "reading and re-writing first string gave matching result!"
    else:
        print "reading and re-writing first string gave different result:"
        print second_string

    w.writeFile( c2, "config-sample-output.yaml" )
    if not w.error():
        print "wrote file successfully."
    else:
        print "error writing file:", w.errorMessage()
