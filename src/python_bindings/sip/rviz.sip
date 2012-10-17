%Module librviz_sip

%Import QtCore/QtCoremod.sip
%Import QtGui/QtGuimod.sip

%DefaultSupertype sip.simplewrapper

%Include visualization_frame.sip
%Include visualization_manager.sip
%Include display.sip
%Include display_group.sip
%Include property.sip
%Include bool_property.sip
%Include panel_dock_widget.sip
%Include view_controller.sip
%Include view_manager.sip
%Include tool.sip
%Include tool_manager.sip
%Include ogre_logging.sip
%Include config.sip
%Include yaml_config_writer.sip
%Include yaml_config_reader.sip
