import QtQuick 2.0
import QtQuick.Controls 2.0
import Rviz 1.0

ApplicationWindow {
    id: root
    width: 1024
    height: 768
    visible: true

    VisualizationFrame {
        id: visualizationFrame
        anchors.fill: parent
    }

    //Component.onCompleted: visualizationFrame.initialize("")
}
