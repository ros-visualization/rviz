import QtQuick 2.0
import QtQuick.Controls 2.0
import Rviz 1.0

ApplicationWindow {
  id: root
  width: 1024
  height: 768
  visible: true

  Rectangle {
    anchors.fill: parent
    color: "lightblue"

    RenderWindow {
      id: renderWindow
      anchors.fill: parent
      anchors.margins: 20
    }
  }

  VisualizationFrame {
    id: visualizationFrame
    anchors.fill: parent
    renderWindow: renderWindow
  }
}

