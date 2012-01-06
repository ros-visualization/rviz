#!/usr/bin/env python

import roslib; roslib.load_manifest('rviz')
import sys
from PySide.QtGui import *
from PySide.QtCore import *
import rviz

app = QApplication( sys.argv )

def acceptIt():
    print 'Accepted!'
    app.quit()

def rejectIt():
    print 'Rejected!'
    app.quit()

def fun():
    accept = QPushButton( "Accept" )
    accept.clicked.connect( acceptIt )

    reject = QPushButton( "Reject" )
    reject.clicked.connect( rejectIt )

    button_layout = QVBoxLayout()
    button_layout.addWidget( accept )
    button_layout.addWidget( reject )

    frame = rviz.VisualizationPanel()

    main_layout = QHBoxLayout()
    main_layout.addLayout( button_layout )
    main_layout.addWidget( frame )

    main_window = QWidget()
    main_window.setLayout( main_layout )
    main_window.show()

    app.exec_()

fun()

sys.exit()
