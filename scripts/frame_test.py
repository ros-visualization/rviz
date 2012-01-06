#!/usr/bin/env python

import roslib; roslib.load_manifest('rviz')
import sys
from PySide.QtGui import *
from PySide.QtCore import *
import rviz


def fun():
    app = QApplication( sys.argv )

    frame = rviz.VisualizationFrame()
    frame.initialize()
    frame.show()

    app.exec_()

if __name__ == '__main__':
    fun()
