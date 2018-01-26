#include <QApplication>
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickView>
#include <QQmlContext>
#include <QUrl>

#include "rviz/quick_visualizer_app.h"
#include "rviz/quick_visualization_frame.h"

int main( int argc, char** argv )
{
  QApplication qapp( argc, argv );

  rviz::QuickVisualizerApp vapp;
  if( vapp.init( argc, argv ))
  {
    rviz::QuickVisualizationFrame::registerTypes();

    QQmlApplicationEngine engine(QUrl("qrc:/qml/main.qml"));
    engine.rootContext()->setContextProperty("RvizApp", &vapp);

    return qapp.exec();
  }
  else
  {
    return 1;
  }
}
