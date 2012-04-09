#!/usr/bin/env python

import roslib; roslib.load_manifest('rviz')
import sys
import rviz
from QtGui import *
from QtCore import *

app = QApplication( sys.argv )

def do_top():
    global frame
    frame.setTargetFrame( "<Fixed Frame>" );
    frame.setViewString( "1.5548 2.3904 10 0 0 0" )

def do_side():
    global frame
    frame.setTargetFrame( "<Fixed Frame>" );
    frame.setViewString( "0.0903987 1.5854 10 0 0 0" )

def do_quit():
    print 'Quitting.'
    app.quit()

def fun():
    global frame

    top = QPushButton( "Top" )
    top.clicked.connect( do_top )

    side = QPushButton( "Side" )
    side.clicked.connect( do_side )

    quit_btn = QPushButton( "Quit" )
    quit_btn.clicked.connect( do_quit )

    button_layout = QVBoxLayout()
    button_layout.addWidget( top )
    button_layout.addWidget( side )
    button_layout.addWidget( quit_btn )

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
