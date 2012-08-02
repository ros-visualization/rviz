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
        layout = QHBoxLayout()
        layout.addWidget(QLabel("Button:"))
        button = QPushButton("I'm a button")
        layout.addWidget( button )
        self.setLayout( layout )
        button.clicked.connect( self.onButtonClick )

    def setFrame( self, vis_frame ):
        self.frame = vis_frame

    def onButtonClick(self):
        self.frame.getManager().setFixedFrame("Python")

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
