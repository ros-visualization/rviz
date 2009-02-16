#!/usr/bin/python

import os
import sys

WXVER = '2.8'
import wxversion
if wxversion.checkInstalled(WXVER):
  wxversion.select(WXVER)
else:
  print >> sys.stderr, "This application requires wxPython version %s"%(WXVER)
  sys.exit(1)

import wx

import roslib
roslib.load_manifest('rviz')

import rviz
import ogre_tools
    

class VisualizerApp(wx.App):
  def __init__(self):
    wx.App.__init__(self)
  
  def OnInit(self):
    ogre_tools.initializeOgre()
    frame = rviz.VisualizationFrame(None)
    frame.initialize()
    frame.Show(True)
    return True
        
  def OnExit(self):        
    ogre_tools.cleanupOgre()

if __name__ == "__main__":
  app = VisualizerApp()
  app.MainLoop()
