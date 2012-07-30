#!/usr/bin/env python

import roslib; roslib.load_manifest('rviz')
import sys
import python_qt_binding.QtBindingHelper # @UnusedImport

from QtGui import *
from QtCore import *
import rviz

class SampleWidget( QWidget ):
    def __init__(self):
        QWidget.__init__(self)
        layout = QHBoxLayout()
        layout.addWidget(QLabel("Button:"))
        layout.addWidget(QPushButton("I'm a button"))
        self.setLayout( layout )

def fun():
    app = QApplication( sys.argv )

    frame = rviz.VisualizationFrame()
    frame.initialize()
    frame.show()

    sw = SampleWidget()
    sw.show()
    frame.addPane( "Button", sw )

    app.exec_()

if __name__ == '__main__':
    fun()
