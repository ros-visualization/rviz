#!/usr/bin/env python

import roslib; roslib.load_manifest('rviz')
import sys
setattr(sys, 'SELECT_QT_BINDING', 'pyside') # Shiboken
#setattr(sys, 'SELECT_QT_BINDING', 'pyqt') # SIP
import python_qt_binding.QtBindingHelper # @UnusedImport

from QtGui import *
from QtCore import *
import rviz

c = rviz.Config()
c.mapSetValue( "foo", 17 )
c.mapSetValue( "bar", "seventeen" )
biff = rviz.Config()
biff.mapSetChild( "boff", rviz.Config( 3.14159 ))
c.mapSetChild( "biff", biff );

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

c.listAppend( rviz.Config( "ay" ))
c.listAppend( rviz.Config( "bee" ))
if c.getType() == rviz.Config.List:
    for i in range( 0, c.listLength() ):
        print c.listChildAt( i ).getValue()

c.mapSetChild( "chunk", rviz.Config() )
print "config has", c.listLength(), "list entries. (should be 0 because it is a map now.)"

r = rviz.YamlConfigReader()
r.readFile( roslib.packages.get_pkg_dir('rviz') + "/default.rviz" )
if r.error():
    print "Error:", r.statusMessage()
else:
    print "default.rviz first two levels of maps:"
    mi = r.config().mapIterator()
    while mi.isValid():
        print "key:", mi.currentKey(), " value:", mi.currentChild().getValue()
        mi2 = mi.currentChild().mapIterator()
        while mi2.isValid():
            print "  key:", mi2.currentKey(), " value:", mi2.currentChild().getValue()
            mi2.advance()
        mi.advance()

    print "tools:"
    tools = r.config().mapGetChild( "Visualization Manager" ).mapGetChild( "Tools" )
    for i in range( 0, tools.listLength() ):
        print "  class:", tools.listChildAt( i ).mapGetChild( "Class" ).getValue()

    w = rviz.YamlConfigWriter()
    first_string = w.writeString( r.config() )
    print "Entire default.rviz written to a string:"
    print first_string

    r2 = rviz.YamlConfigReader()
    r2.readString( first_string )
    second_string = w.writeString( r2.config() )

    if first_string == second_string:
        print "reading and re-writing first string gave matching result!"
    else:
        print "reading and re-writing first string gave different result:"
        print second_string

    w.writeFile( r2.config(), "config-sample-output.yaml" )
    if not w.error():
        print "wrote file successfully."
    else:
        print "error writing file:", w.statusMessage()
