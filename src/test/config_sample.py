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
foo = c.makeChild( "foo" )
foo.setValue( 17 )
bar = c.makeChild( "bar" )
bar.setValue( "seventeen" )
biff = foo.makeChild( "biff" )
biff.setValue( "3.14159" )
print c.getChild( "foo" ).getValue()
print c.getChild( "bar" ).getValue()
print c.getChild( "baz" ).getValue()
print c.getChild( "foo" ).getChild( "biff" ).getValue()
# print c.getChild( "goo" ).getChild( "biff" ).getValue() # crashes because "goo" does not exist, so getChild("biff") can't be called.

mi = c.mapIterator()
while mi.hasNext():
    print "key:", mi.currentKey(), " value:", mi.currentChild().getValue()
    mi2 = mi.currentChild().mapIterator()
    while mi2.hasNext():
        print "  key:", mi2.currentKey(), " value:", mi2.currentChild().getValue()
        mi2.next()
    mi.next()

s = c.makeSequence()
s.makeNext().setValue( "a" );
s.makeNext().setValue( "b" );
s2 = c.getSequence()
while s2.hasNext():
    print s2.getNext().getValue()

c.makeChild( "chunk" )
s3 = c.getSequence()
while s3.hasNext():
    print "s3 should not have anything...", s3.getNext().getValue()
print "done"

r = rviz.YamlConfigReader()
r.readFile( roslib.packages.get_pkg_dir('rviz') + "/default.rviz" )
if r.error():
    print "Error:", r.statusMessage()
else:
    print "default.rviz first two levels of maps:"
    mi = r.config().mapIterator()
    while mi.hasNext():
        print "key:", mi.currentKey(), " value:", mi.currentChild().getValue()
        mi2 = mi.currentChild().mapIterator()
        while mi2.hasNext():
            print "  key:", mi2.currentKey(), " value:", mi2.currentChild().getValue()
            mi2.next()
        mi.next()

    print "tools:"
    si = r.config().getChild( "Visualization Manager" ).getChild( "Tools" ).getSequence()
    while si.hasNext():
        print "  class:" + si.getNext().getChild( "Class" ).getValue()

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
