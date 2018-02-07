#include "displayconfig.h"

#include <ros/ros.h>

#include "rviz/yaml_config_reader.h"

DisplayConfig::DisplayConfig(QObject *parent)
  : QObject(parent)
  , frame_(nullptr)
  , source_("")
  , loaded_(false)
{
  connect(this, &DisplayConfig::sourceChanged, this, &DisplayConfig::updateConfig);
}

DisplayConfig::~DisplayConfig()
{

}

rviz::QuickVisualizationFrame *DisplayConfig::getFrame() const
{
  return frame_;
}

QString DisplayConfig::getSource() const
{
  return source_;
}

bool DisplayConfig::getLoaded() const
{
  return loaded_;
}

void DisplayConfig::setFrame(rviz::QuickVisualizationFrame *frame)
{
  if (frame_ == frame) {
    return;
  }

  frame_ = frame;
  Q_EMIT frameChanged(frame_);

  if (frame_->isInitialized()) {
      initialize();
  }
  else {
    connect(frame_, &rviz::QuickVisualizationFrame::initializedChanged,
            this, &DisplayConfig::initialize);
  }
}

void DisplayConfig::setSource(const QString &source)
{
  if (source_ == source) {
    return;
  }

  source_ = source;
  Q_EMIT sourceChanged(source_);
}

bool DisplayConfig::frameIsInitialized()
{
  return frame_ && frame_->isInitialized();
}

void DisplayConfig::initialize()
{
  updateConfig();
}

void DisplayConfig::updateConfig()
{
  if ((source_.isEmpty()) || (!frameIsInitialized()) || loaded_) {
    return;
  }

  auto reader = rviz::YamlConfigReader();
  auto config = rviz::Config();
  reader.readFile(config, source_);
  frame_->getManager()->load(config.mapGetChild( "Visualization Manager" ));

  loaded_ = true;
  Q_EMIT loadedChanged(loaded_);
}
