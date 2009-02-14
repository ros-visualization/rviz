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
import roslib.packages
roslib.load_manifest('ogre_visualizer')

import shutil
import glob
import ogre_visualizer
import ogre_tools


class VisualizerFrame(wx.Frame):
    _CONFIG_WINDOW_X="/Window/X"
    _CONFIG_WINDOW_Y="/Window/Y"
    _CONFIG_WINDOW_WIDTH="/Window/Width"
    _CONFIG_WINDOW_HEIGHT="/Window/Height"
    
    _CONFIG_EXTENSION="vcg"
    
    def __init__(self, parent, id=wx.ID_ANY, title='Standalone Visualizer', pos=wx.DefaultPosition, size=(800, 600), style=wx.DEFAULT_FRAME_STYLE):
        wx.Frame.__init__(self, parent, id, title, pos, size, style)
        
        ogre_tools.initializeOgre()
        visualizer_panel = ogre_visualizer.VisualizationPanel(self)
        
        self._package_path = roslib.packages.get_pkg_dir('ogre_visualizer')
        self._global_config_path = os.path.join(self._package_path, "configs")
        
        self._visualizer_panel = visualizer_panel
        
        media_path = roslib.packages.get_pkg_dir( "gazebo_robot_description" )
        media_path += "/world/Media/";
        
        media_paths = [media_path]
        media_paths.append( media_path )
        media_paths.append( media_path + "fonts" )
        media_paths.append( media_path + "materials" )
        media_paths.append( media_path + "materials/scripts" )
        media_paths.append( media_path + "materials/programs" )
        media_paths.append( media_path + "materials/textures" )
        media_paths.append( media_path + "models" )
        media_paths.append( media_path + "models/pr2" )
        
        ogre_tools.initializeResources( media_paths )
        
        self.init_config()
        self.init_menu()
        
        manager = visualizer_panel.getManager()
        
        # Load our window options
        (x, y) = self.GetPositionTuple()
        (width, height) = self.GetSizeTuple()
        if (self._config.HasEntry(self._CONFIG_WINDOW_X)):
            x = self._config.ReadInt(self._CONFIG_WINDOW_X)
        if (self._config.HasEntry(self._CONFIG_WINDOW_Y)):
            y = self._config.ReadInt(self._CONFIG_WINDOW_Y)
        if (self._config.HasEntry(self._CONFIG_WINDOW_WIDTH)):
            width = self._config.ReadInt(self._CONFIG_WINDOW_WIDTH)
        if (self._config.HasEntry(self._CONFIG_WINDOW_HEIGHT)):
            height = self._config.ReadInt(self._CONFIG_WINDOW_HEIGHT)
            
        self.SetPosition((x, y))
        self.SetSize((width, height))
        
        manager.loadGeneralConfig(self._config)
        manager.loadDisplayConfig(self._config)
        
        self.Bind(wx.EVT_CLOSE, self.on_close)
        
    def on_close(self, event):
        self.save_config()
        self.Destroy()
        
    def load_config_from_path(self, path):
        manager = self._visualizer_panel.getManager()
        manager.removeAllDisplays()
        config = wx.FileConfig(localFilename=path)
        
        manager.loadGeneralConfig(config)
        manager.loadDisplayConfig(config)
        
    def on_open(self, event):
        dialog = wx.FileDialog(self, "Choose a file to open", self._save_location, wildcard="*."+self._CONFIG_EXTENSION, style=wx.FD_OPEN)
        if dialog.ShowModal() == wx.ID_OK:
            path = dialog.GetPath()
            self.load_config_from_path(path)
    
    def on_save(self, event):
        dialog = wx.FileDialog(self, "Choose a file to save to", self._save_location, wildcard="*."+self._CONFIG_EXTENSION, style=wx.FD_SAVE|wx.FD_OVERWRITE_PROMPT)
        if dialog.ShowModal() == wx.ID_OK:
            path = dialog.GetPath()
            if (not path.endswith("."+self._CONFIG_EXTENSION)):
                path += "."+self._CONFIG_EXTENSION
                
            manager = self._visualizer_panel.getManager()
            config = wx.FileConfig(localFilename=path)
            config.DeleteAll()
            manager.saveGeneralConfig(config)
            manager.saveDisplayConfig(config)
            config.Flush()
            
            self.load_config_menus()
            
    def on_global_config(self, event):
        item = self._menubar.FindItemById(event.GetId())
        filename = item.GetLabel() #item.GetItemLabelText();
        #for some reason all underscores get doubled up
        filename = filename.replace('__', '_')
        path = os.path.join(self._global_config_path, filename + "." + self._CONFIG_EXTENSION)
        self.load_config_from_path(path)
        
    def on_local_config(self, event):
        item = self._menubar.FindItemById(event.GetId())
        filename = item.GetLabel() #item.GetItemLabelText();
        #for some reason all underscores get doubled up
        filename = filename.replace('__', '_')
        path = os.path.join(self._save_location, filename + "." + self._CONFIG_EXTENSION)
        self.load_config_from_path(path)
        
    def load_config_menus(self):
        #first clear the menus
        items = self._local_configs_menu.GetMenuItems()
        for item in items:
            self._local_configs_menu.DestroyItem(item)
            
        items = self._global_configs_menu.GetMenuItems()
        for item in items:
            self._global_configs_menu.DestroyItem(item)

        wildcard = os.path.join(self._save_location, "*."+self._CONFIG_EXTENSION)
        files = glob.glob(wildcard)
        for file in files:
            option = os.path.basename(file)
            # strip off extension
            option = option[:len(option)-4]
            item = self._local_configs_menu.Append(wx.ID_ANY, option)
            self.Bind(wx.EVT_MENU, self.on_local_config, item)
        
        wildcard = os.path.join(self._global_config_path, "*."+self._CONFIG_EXTENSION)
        files = glob.glob(wildcard)
        for file in files:
            option = os.path.basename(file)
            # strip off extension
            option = option[:len(option)-4]
            item = self._global_configs_menu.Append(wx.ID_ANY, option)
            self.Bind(wx.EVT_MENU, self.on_global_config, item)
        
    def init_menu(self):
        self._menubar = wx.MenuBar()
        
        self._filemenu = wx.Menu("")
        item = self._filemenu.Append(wx.ID_OPEN, "&Open Config\tCtrl-O")
        self.Bind(wx.EVT_MENU, self.on_open, item)
        item = self._filemenu.Append(wx.ID_SAVE, "&Save Config\tCtrl-S")
        self.Bind(wx.EVT_MENU, self.on_save, item)
        
        self._local_configs_menu = wx.Menu("")
        self._global_configs_menu = wx.Menu("")
        
        self._filemenu.AppendMenu(wx.ID_ANY, "&Local Configs", self._local_configs_menu)
        self._filemenu.AppendMenu(wx.ID_ANY, "&Global Configs", self._global_configs_menu)
        
        self._filemenu.AppendSeparator()
        item = self._filemenu.Append(wx.ID_EXIT, "&Quit")
        self.Bind(wx.EVT_MENU, self.on_close, item)
        
        self._menubar.Append(self._filemenu, "&File")
        self.SetMenuBar(self._menubar)
        
        self.load_config_menus()
        
    def init_config(self):
        config_dir = wx.StandardPaths.Get().GetUserDataDir()
        config_file = os.path.join(config_dir, 'config')
        # legacy case: copy the old config to the new location
        if os.path.isfile(config_dir):
            print("Migrating old config file to new location (%s to %s)"%(config_dir, config_file))
            backup_file = config_dir + "bak"
            shutil.move(config_dir, backup_file)
            os.mkdir(config_dir)
            shutil.move(backup_file, config_file)
        elif not os.path.exists(config_dir):
            os.mkdir(config_dir)
        
        self._config = wx.FileConfig(localFilename=config_file)
        self._config_dir = config_dir
        self._config_file = config_file
        self._save_location = os.path.join(config_dir, "saved")
        
        if not os.path.exists(self._save_location):
            os.mkdir(self._save_location)
        
    def save_config(self):
        self._config.DeleteAll()
        
        (x, y) = self.GetPositionTuple()
        (width, height) = self.GetSizeTuple()
        self._config.WriteInt(self._CONFIG_WINDOW_X, x)
        self._config.WriteInt(self._CONFIG_WINDOW_Y, y)
        self._config.WriteInt(self._CONFIG_WINDOW_WIDTH, width)
        self._config.WriteInt(self._CONFIG_WINDOW_HEIGHT, height)
        
        self._visualizer_panel.getManager().saveGeneralConfig(self._config)
        self._visualizer_panel.getManager().saveDisplayConfig(self._config)
        self._config.Flush()

class VisualizerApp(wx.App):
    def OnInit(self):
        self.SetAppName("standalone_visualizer")
        frame = VisualizerFrame(None, wx.ID_ANY, "Standalone Visualizer", wx.DefaultPosition, wx.Size( 800, 600 ) )
        frame.Show(True)
        return True
        
    def OnExit(self):        
        ogre_tools.cleanupOgre()
        
        

if __name__ == "__main__":
    app = VisualizerApp()
    app.MainLoop()
