#!/usr/bin/env python

import roslib; roslib.load_manifest('rviz')
import sys
setattr(sys, 'SELECT_QT_BINDING', 'pyside')
#setattr(sys, 'SELECT_QT_BINDING', 'pyqt')
import python_qt_binding.QtBindingHelper # @UnusedImport

from QtGui import *
from QtCore import *
import rviz

class SampleWidget( QWidget ):
    def __init__(self):
        QWidget.__init__(self)
        layout = QVBoxLayout()

        frame_button = QPushButton("Set Fixed Frame")
        frame_button.clicked.connect( self.onFrameButtonClick )
        layout.addWidget( frame_button )

        enable_button = QPushButton("Toggle Grid Enable")
        enable_button.clicked.connect( self.onEnableButtonClick )
        layout.addWidget( enable_button )

        self.setLayout( layout )
        self.grid_display = None

    def setFrame( self, vis_frame ):
        self.frame = vis_frame

    def onFrameButtonClick(self):
        self.frame.getManager().setFixedFrame("Python")

    def onEnableButtonClick(self):
        if self.grid_display == None:
            self.grid_display = self.frame.getManager().createDisplay( "rviz/Grid", "Awesome grid", True )
        else:
            self.grid_display.setEnabled( not self.grid_display.isEnabled() )

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
