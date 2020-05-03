#!/usr/bin/env python

import sys

# This setattr() call is useful only to force the use of a particular
# binding scheme.  It's useful for testing rviz, but can be left out
# for most other users.
#setattr(sys, 'SELECT_QT_BINDING', 'pyside') # Shiboken
setattr(sys, 'SELECT_QT_BINDING', 'pyqt') # SIP

from python_qt_binding import QT_BINDING
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import *
from rviz import bindings as rviz

if QT_BINDING == 'pyside':
    print "Using PySide and shiboken for rviz python bindings."
elif QT_BINDING == 'pyqt':
    print "Using PyQt and sip for rviz python bindings."

class SampleWidget( QWidget ):
    def __init__(self):
        QWidget.__init__(self)

        self.grid_display = None
        self.props = []

        layout = QVBoxLayout()

        frame_button = QPushButton("Set Fixed Frame")
        frame_button.clicked.connect( self.onFrameButtonClick )
        layout.addWidget( frame_button )

        enable_button = QPushButton("Toggle Grid Enable")
        enable_button.clicked.connect( self.onEnableButtonClick )
        layout.addWidget( enable_button )

        thickness_slider = QSlider( Qt.Horizontal )
        thickness_slider.setTracking( True )
        thickness_slider.setMinimum( 1 )
        thickness_slider.setMaximum( 1000 )
        thickness_slider.valueChanged.connect( self.onThicknessSliderChanged )
        layout.addWidget( thickness_slider )

        fps_button = QPushButton("Switch to FPS")
        fps_button.clicked.connect( self.onFpsButtonClick )
        layout.addWidget( fps_button )

        h_layout = QHBoxLayout()

        top_button = QPushButton( "Top View" )
        top_button.clicked.connect( self.onTopButtonClick )
        h_layout.addWidget( top_button )

        side_button = QPushButton( "Side View" )
        side_button.clicked.connect( self.onSideButtonClick )
        h_layout.addWidget( side_button )

        look_button = QPushButton( "Look Away" )
        look_button.clicked.connect( self.onLookButtonClick )
        h_layout.addWidget( look_button )

        layout.addLayout( h_layout )

        distance_slider = QSlider( Qt.Horizontal )
        distance_slider.setTracking( True )
        distance_slider.setMinimum( 1 )
        distance_slider.setMaximum( 1000 )
        distance_slider.valueChanged.connect( self.onDistanceSliderChanged )
        layout.addWidget( distance_slider )

        tool_button = QPushButton( "Select" )
        tool_button.clicked.connect( self.onSelectClick )
        layout.addWidget( tool_button )

        coolify_button = QPushButton( "Coolify Displays" )
        coolify_button.clicked.connect( self.onCoolifyClick )
        layout.addWidget( coolify_button )

        self.setLayout( layout )

    def destruct( self ):
        """
        Disconnect internal object connections.  Without something
        like this, it is easy to have a crash when the object goes out
        of scope.
        """
        self.frame = None
        if self.grid_display != None:
            self.grid_display.getParent().takeChild( self.grid_display )
        self.grid_display = None
        for p in self.props:
            p.getParent().takeChild( p )
        self.props = []

    def setFrame( self, vis_frame ):
        self.frame = vis_frame

    def onFrameButtonClick(self):
        self.frame.getManager().setFixedFrame("Python")

    def onEnableButtonClick(self):
        if self.grid_display == None:
            self.grid_display = self.frame.getManager().createDisplay( "rviz/Grid", "Awesome grid", True )
            self.grid_display.subProp( "Line Style" ).setValue( "Billboards" )
        else:
            self.grid_display.setEnabled( not self.grid_display.isEnabled() )

    def onThicknessSliderChanged( self, new_value ):
        if self.grid_display != None:
            self.grid_display.subProp( "Line Style" ).subProp( "Line Width" ).setValue( new_value / 1000.0 )
            prop = rviz.Property( "Prop " + str(new_value), new_value / 1000.0, "Bad idea property generation" )
            self.grid_display.addChild( prop )
            self.props.append( prop )

    def onDistanceSliderChanged( self, new_value ):
        controller = self.frame.getManager().getViewManager().getCurrent()
        if controller != None:
            controller.subProp( "Distance" ).setValue( new_value / 10.0 )

    def onFpsButtonClick( self ):
        self.frame.getManager().getViewManager().setCurrentViewControllerType( "rviz/FPS" )

    def onTopButtonClick( self ):
        self.switchToView( "Top View" );

    def onSideButtonClick( self ):
        self.switchToView( "Side View" );

    def onLookButtonClick( self ):
        controller = self.frame.getManager().getViewManager().getCurrent()
        if controller != None:
            controller.lookAt( 5, 5, 0 )

    def switchToView( self, view_name ):
        view_man = self.frame.getManager().getViewManager()
        for i in range( view_man.getNumViews() ):
            if view_man.getViewAt( i ).getName() == view_name:
                view_man.setCurrentFrom( view_man.getViewAt( i ))
                return
        print( "Did not find view named %s." % view_name )

    def onSelectClick( self ):
        tool_man = self.frame.getManager().getToolManager()
        for i in range( tool_man.numTools() ):
            if tool_man.getTool( i ).getName() == "Select":
                tool_man.setCurrentTool( tool_man.getTool( i ))
                return

    def onCoolifyClick( self ):
        self.coolify( self.frame.getManager().getRootDisplayGroup() )

    def coolify( self, group ):
        for i in range( group.numDisplays() ):
            display = group.getDisplayAt( i )
            display.setName( "Cool " + display.getName() )
            subgroup = group.getGroupAt( i )
            if subgroup != None:
                self.coolify( subgroup )

def fun():
    # rviz.OgreLogging.noLog() # (no log is the default)
    # rviz.OgreLogging.useStandardOut()
    # rviz.OgreLogging.useLogFile( "frame_test.ogre-log" )

    app = QApplication( sys.argv )

    frame = rviz.VisualizationFrame()
    frame.initialize()
    frame.show()

    sw = SampleWidget()
    sw.setFrame( frame )
    sw.show()
    frame.addPane( "Button", sw )

    app.exec_()

    # Without this destruct() call, this program crashes just after
    # the "after exec_()" printout, when "sw" goes out of scope and is
    # destroyed.
    sw.destruct()

    print "after exec_()"

if __name__ == '__main__':
    # This fun() function call is just here to demonstrate rviz
    # objects going out of scope and being cleaned up correctly.
    fun()
    print "after fun()"
