#!/usr/bin/env python

import roslib; roslib.load_manifest('rviz')
import sys
#setattr(sys, 'SELECT_QT_BINDING', 'pyside')
setattr(sys, 'SELECT_QT_BINDING', 'pyqt')
import python_qt_binding.QtBindingHelper # @UnusedImport

from QtGui import *
from QtCore import *
import rviz

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
        
        layout.addLayout( h_layout )

        distance_slider = QSlider( Qt.Horizontal )
        distance_slider.setTracking( True )
        distance_slider.setMinimum( 1 )
        distance_slider.setMaximum( 1000 )
        distance_slider.valueChanged.connect( self.onDistanceSliderChanged )
        layout.addWidget( distance_slider )

        self.setLayout( layout )

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
        
    def switchToView( self, view_name ):
        view_man = self.frame.getManager().getViewManager()
        for i in range( view_man.getNumViews() ):
            if view_man.getViewAt( i ).getName() == view_name:
                view_man.setCurrentFrom( view_man.getViewAt( i ))
                return
        print( "Did not find view named %s." % view_name )

def fun():
    app = QApplication( sys.argv )

    frame = rviz.VisualizationFrame()
    frame.initialize()
    frame.show()

    sw = SampleWidget()
    sw.setFrame( frame )
    sw.show()
    frame.addPane( "Button", sw )

    app.exec_()

if __name__ == '__main__':
    fun()
