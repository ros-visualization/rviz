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

class VisualizerFrame(wx.Frame):
  def __init__(self, parent, id=wx.ID_ANY, title='Standalone Visualizer', pos=wx.DefaultPosition, size=(800, 600), style=wx.DEFAULT_FRAME_STYLE):
    wx.Frame.__init__(self, parent, id, title, pos, size, style)
    
    visualizer_panel = rviz.VisualizationPanel(self)
    
    self.Layout()
    

class VisualizerApp(wx.App):
  def __init__(self):
    wx.App.__init__(self)
  
  def OnInit(self):
    ogre_tools.initializeOgre()
    frame = VisualizerFrame(None, wx.ID_ANY, "Visualization Panel Test", wx.DefaultPosition, wx.Size( 800, 600 ) )
    frame.Show(True)
    return True
        
  def OnExit(self):        
    ogre_tools.cleanupOgre()

if __name__ == "__main__":
  app = VisualizerApp()
  app.MainLoop()
