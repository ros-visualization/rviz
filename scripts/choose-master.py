#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys
import os
import os.path
import xmlrpclib
import signal
import subprocess

from PySide.QtGui import *
from PySide.QtCore import *

def sigintHandler( signal, frame ):
    QApplication.quit()

# Return an array of node names, or None on error (like if the master doesn't exist).
def getNodesOnMaster( master_uri ):
    try:
        proxy = xmlrpclib.ServerProxy( master_uri )
        state = proxy.getSystemState("")
        if state[0] != 1:
            return None
        nodes = []
        for s in state[2]:
            for t, l in s:
                nodes.extend(l)
        return list(set(nodes))
    except:
        return None

# Return an array of recently-used master URIs read from a file, or empty array if none.
def readRecentMasters():
    masters = set()
    try:
        filename = os.path.expanduser('~/.ros/recent_masters')
        if os.path.exists( filename ):
            with open( filename ) as file:
                for line in file:
                    line = line.strip()
                    if len( line ) > 0:
                        masters.add( line )
    except:
        pass
    return masters

# Replace the recent_masters file with the contents of master_uris,
# which should be a set of strings.
def writeRecentMasters( master_uris ):
    try:
        filename = os.path.expanduser('~/.ros/recent_masters')
        dirname = os.path.dirname( filename );
        if not os.path.isdir( dirname ):
            os.makedirs( dirname )
        with open( filename, 'w' ) as file:
            for master_uri in master_uris:
                file.write( master_uri + '\n' )
    except:
        pass

# Thread continually runs, checking for a given master over and over
# while self.scan is True.  When self.scan is False, keeps looping but
# does not check for the master.
#
# When it gets an answer for a master, emits signal foundValidMaster
# or foundInvalidMaster.
class ScannerThread( QThread ):
    foundValidMaster = Signal( str, object )
    foundInvalidMaster = Signal( str )

    def __init__( self, master_uri ):
        super(ScannerThread, self).__init__()
        self.master_uri = master_uri
        self.scan = True

    def run( self ):
        while True:
            if self.scan:
                nodes = getNodesOnMaster( self.master_uri )
                if( nodes ):
                    self.foundValidMaster.emit( self.master_uri, nodes )
                else:
                    self.foundInvalidMaster.emit( self.master_uri )
            QThread.msleep( 500 )

class MasterURIItem(QTableWidgetItem):
    def __init__( self, text, valid ):
        super(MasterURIItem, self).__init__( text )
        self.valid = valid
        self.my_text = text

    def __lt__( self, other ):
        if self.valid == other.valid:
            return (self.my_text < other.my_text)
        elif self.valid:
            return True
        else:
            return False

class DeleteButton( QToolButton ):
    masterClicked = Signal( str )

    def __init__( self, master_uri, parent=None ):
        super( DeleteButton, self ).__init__( parent )
        self.setIcon( QIcon( ":/trolltech/styles/commonstyle/images/standardbutton-close-32.png" ))
        self.master_uri = master_uri
        self.clicked.connect( self.onClick )

    def onClick( self ):
        self.masterClicked.emit( self.master_uri )

class ChooserDialog(QDialog):
    def __init__(self, program, master_from_environ, parent=None):
        super(ChooserDialog, self).__init__(parent)

        self.program = program
        program_name = os.path.basename( program )

        main_layout = QVBoxLayout()

        main_layout.addWidget( QLabel( "Choose a ROS master to start " + program_name ))

        self.table = QTableWidget( 0, 3, self )
        self.table.setHorizontalHeaderLabels( ["Master URI", "Nodes", ""] )
        self.table.horizontalHeader().setResizeMode( 0, QHeaderView.Stretch )
        self.table.horizontalHeader().setResizeMode( 1, QHeaderView.Fixed )
        self.table.horizontalHeader().resizeSection( 1, 60 )
        self.table.horizontalHeader().setResizeMode( 2, QHeaderView.Fixed )
        self.table.horizontalHeader().resizeSection( 2, 30 )
        self.table.itemSelectionChanged.connect( self.onSelectionChange )
        self.table.itemActivated.connect( self.start )
        main_layout.addWidget( self.table )

        start_program_layout = QHBoxLayout()
        start_program_layout.addStretch( 1000 )
        self.start_button = QPushButton( "Start " + program_name )
        self.start_button.setEnabled( False )
        self.start_button.clicked.connect( self.start )
        start_program_layout.addWidget( self.start_button )
        main_layout.addLayout( start_program_layout )

        bottom_layout = QHBoxLayout()
        add_label = QLabel( "http://" )
        self.host_entry = QLineEdit("localhost")
        self.host_entry.setMinimumSize( 150, 10 )
        self.host_entry.returnPressed.connect( self.addMaster )
        colon_label = QLabel( ":" )
        self.port_entry = QLineEdit("11311")
        self.port_entry.setMinimumSize( 100, 10 )
        self.port_entry.returnPressed.connect( self.addMaster )
        bottom_layout.addWidget( add_label )
        bottom_layout.addWidget( self.host_entry )
        bottom_layout.addWidget( colon_label )
        bottom_layout.addWidget( self.port_entry )
        add_button = QPushButton( "Add Master" )
        add_button.clicked.connect( self.addMaster )
        bottom_layout.addWidget( add_button )

        main_layout.addLayout( bottom_layout )
        self.setLayout( main_layout )

        self.master_from_env = master_from_environ
        masters = readRecentMasters()
        self.master_to_scanner_map = {}
        for master in masters:
            self.addScanner( master )

        self.show_timer = QTimer()
        self.show_timer.setSingleShot( True )
        self.show_timer.timeout.connect( self.show )
        self.show_timer.start( 200 )

    def addScanner( self, master_uri ):
        if master_uri not in self.master_to_scanner_map:
            thread = ScannerThread( master_uri )
            thread.foundValidMaster.connect( self.insertValidMaster, Qt.QueuedConnection )
            thread.foundInvalidMaster.connect( self.insertInvalidMaster, Qt.QueuedConnection )
            thread.start()
            self.master_to_scanner_map[ master_uri ] = thread

    def removeScanner( self, master_uri ):
        if master_uri in self.master_to_scanner_map:
            thread = self.master_to_scanner_map[ master_uri ]
            thread.scan = False
            thread.foundValidMaster.disconnect( self.insertValidMaster )
            thread.foundInvalidMaster.disconnect( self.insertInvalidMaster )
            del self.master_to_scanner_map[ master_uri ]
            print "waiting for thread", thread.master_uri, "to die."
            thread.wait(0)

    def setScanning( self, scanning ):
        for master, scanner in self.master_to_scanner_map.iteritems():
            scanner.scan = scanning

    def onSelectionChange( self ):
        selection = self.table.selectedItems()
        self.start_button.setEnabled( len( selection ) == 1 )

    def start( self ):
        selection = self.table.selectedItems()
        if len( selection ):
            self.setScanning( False )
            self.hide()
            master = selection[0].text()
            print "running", self.program, "with master", master
            program_result = subprocess.call( [self.program, "--in-mc-wrapper", "__master:=" + master] )
            if program_result == 255:
                selection[0].setSelected( False )
                self.master_from_env = None
                self.show()
                self.setScanning( True )
            else:
                self.close()

    def addMaster( self ):
        host = self.host_entry.text()
        port = self.port_entry.text()
        if len( host ) and len( port ):
            self.addScanner( 'http://' + host + ':' + port )
            writeRecentMasters( self.master_to_scanner_map.keys() )

    def addRow( self, master_uri, valid ):
        row = self.table.rowCount()
        self.table.insertRow( row )

        items = []

        # master URI item
        item = MasterURIItem( master_uri, valid )
        self.table.setItem( row, 0, item )
        items.append( item )

        # node count item
        item = QTableWidgetItem()
        self.table.setItem( row, 1, item )
        items.append( item )

        # delete master item
        item = QTableWidgetItem()
        item.setFlags( Qt.NoItemFlags )
        self.table.setItem( row, 2, item )
        delete_button = DeleteButton( master_uri )
        self.table.setCellWidget( row, 2, delete_button )
        delete_button.masterClicked.connect( self.deleteMaster )

        return items

    def insertValidMaster( self, master_uri, nodes ):
        old_item = self.itemForMaster( master_uri )
        if old_item:
            if old_item.valid:
                return
            else:
                self.removeMasterFromTable( master_uri )

        items = self.addRow( master_uri, True )
        # master URI item
        items[0].setFlags( Qt.ItemIsEnabled | Qt.ItemIsSelectable )
        items[0].setSelected( self.master_from_env == master_uri )

        if self.master_from_env == master_uri and not self.isVisible():
            self.start()

        # node count item
        items[1].setFlags( Qt.NoItemFlags )
        items[1].setBackground( Qt.white )
        items[1].setText( str( len( nodes )))

        self.table.sortItems( 0 )

    def insertInvalidMaster( self, master_uri ):
        old_item = self.itemForMaster( master_uri )
        if old_item:
            if old_item.valid:
                self.removeMasterFromTable( master_uri )
            else:
                return

        items = self.addRow( master_uri, False )
        # master URI item
        items[0].setFlags( Qt.NoItemFlags )

        # node count item
        items[1].setFlags( Qt.NoItemFlags )

        self.table.sortItems( 0 )

    def itemForMaster( self, master_uri ):
        row = 0
        while row < self.table.rowCount():
            item = self.table.item( row, 0 )
            if item.text() == master_uri:
                return item
            row += 1
        return None

    def removeMasterFromTable( self, master_uri ):
        row = 0
        while row < self.table.rowCount():
            if self.table.item( row, 0 ).text() == master_uri:
                self.table.removeRow( row )
            else:
                row += 1

    def deleteMaster( self, master_uri ):
        self.removeScanner( master_uri )
        writeRecentMasters( self.master_to_scanner_map.keys() )
        self.removeMasterFromTable( master_uri )

if __name__ == '__main__':
    signal.signal(signal.SIGINT, sigintHandler)

    app = QApplication( sys.argv )

    if len( sys.argv ) < 2:
        print "USAGE: choose-master.py <program>"
        sys.exit( 1 )

    program = sys.argv[1]

    # Borrowed from rxlaunch:
    # Sets up signal handling so SIGINT closes the application,
    # following the solution given at [1].  Sets up a custom signal
    # handler, and ensures that the Python interpreter runs
    # occasionally so the signal is handled.  The email thread at [2]
    # explains why this is necessary.
    #
    # [1] http://stackoverflow.com/questions/4938723/#4939113
    # [2] http://www.mail-archive.com/pyqt@riverbankcomputing.com/msg13757.html
    timer = QTimer()
    timer.start(250)
    timer.timeout.connect(lambda: None)  # Forces the interpreter to run every 250ms

    dialog = ChooserDialog( program, os.environ[ 'ROS_MASTER_URI' ])

    app.exec_()
