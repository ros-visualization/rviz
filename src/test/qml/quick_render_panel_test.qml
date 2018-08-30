import QtQuick 2.0
import QtQuick.Controls 2.3
import ros.rviz 1.0
import MyModule 1.0

ApplicationWindow {
  id: root
  width: 1024
  height: 768
  visible: true

  Loader {
    id: loader
    anchors.fill: parent
    sourceComponent: rvizComp
  }

  Component {
    id: rvizComp

    Item {
      VisualizationFrame {
        id: visualizationFrame
        anchors.fill: parent
        renderWindow: renderWindow
      }

      Rectangle {
        anchors.fill: parent
        color: "lightblue"

        RenderWindow {
          id: renderWindow
          anchors.fill: parent
          anchors.margins: 20
        }
      }

      SimpleGrid {
        id: grid
        frame: visualizationFrame
        lineWidth: 10
        color: "lightblue"
      }

      DisplayConfig {
        id: displayConfig
        frame: visualizationFrame
        source: rvizPath + "/src/test/quick_test.rviz"
      }

      Row {
        anchors.bottom: parent.bottom
        anchors.horizontalCenter: parent.horizontalCenter

        Button {
          text: "Red Grid"
          onClicked: grid.color = "red"
        }

        Button {
          text: "Blue Grid"
          onClicked: grid.color = "blue"
        }
      }
    }
  }

  Button {
    anchors.top: parent.top
    anchors.horizontalCenter: parent.horizontalCenter
    text: qsTr("reload")
    onClicked: {
      loader.active = false;
      loader.active = true;
    }
  }
}

