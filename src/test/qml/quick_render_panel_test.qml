import QtQuick 2.0
import QtQuick.Controls 2.0
import Rviz 1.0
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
    Rectangle {
        anchors.fill: parent
        color: "lightblue"

        VisualizationFrame {
            id: visualizationFrame
            anchors.fill: parent
            renderWindow: renderWindow

                SimpleGrid {
                    id: grid
                    frame: visualizationFrame
                    lineWidth: 10
                    color: "lightblue"
                }
            }
        }

        RenderWindow {
            id: renderWindow
            anchors.fill: parent
            anchors.margins: 20


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
