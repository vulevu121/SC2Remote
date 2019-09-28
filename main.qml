import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import QtQuick.Controls.Material 2.12

ApplicationWindow {
    visible: true
    width: 1080 * 0.5
    height: 1920 * 0.5
    title: qsTr("SC2")
    color: "black"

    function pixel(p) {
        return p*2;
    }

    SwipeView {
        id: swipeView
        anchors.fill: parent
        currentIndex: tabBar.currentIndex

        Item {
            id: remoteView

            Column {
                anchors {
                    fill: parent
                }

                Image {
                    id: carImage
                    source: "qrc:/images/door_close"
                    fillMode: Image.PreserveAspectFit
                    anchors {
                        left: parent.left
                        right: parent.right
                    }


                }

                Row {
                    anchors {
                        left: parent.left
                        right: parent.right
                        margins: pixel(10)
                    }

                    Column {
                        width: parent.width / 2
//                        anchors {
//                            left: parent.left
//                            right: parent.horizontalCenter
//                        }

                        RoundButton {
                            text: "Front Driver"
                            radius: pixel(2)
                            icon.source: "qrc:/icons/car-door"
                            width: parent.width
                            height: width / 4
                        }

                        RoundButton {
                            text: "Hold to Open"
                            radius: pixel(2)
                            icon.source: "qrc:/icons/car-door"
                            width: parent.width
                            height: width / 4
                        }

                        RoundButton {
                            text: "Hold to Close"
                            radius: pixel(2)
                            icon.source: "qrc:/icons/car-door"
                            width: parent.width
                            height: width / 4
                        }

                    }

                    Column {
                        width: parent.width / 2

                        RoundButton {
                            text: "Front Passenger"
                            radius: pixel(2)
                            icon.source: "qrc:/icons/car-door"
                            width: parent.width
                            height: width / 4
                        }

                        RoundButton {
                            text: "Hold to Open"
                            radius: pixel(2)
                            icon.source: "qrc:/icons/car-door"
                            width: parent.width
                            height: width / 4
                            onClicked: {
                                carImage.source = "qrc:/images/door_open"
                            }
                        }

                        RoundButton {
                            text: "Hold to Close"
                            radius: pixel(2)
                            icon.source: "qrc:/icons/car-door"
                            width: parent.width
                            height: width / 4
                            onClicked: {
                                carImage.source = "qrc:/images/door_close"
                            }

                        }

                    }


//                    RowLayout {}
                }
            }







        }

        Item {
            id: settingsView
        }

//        Page1Form {

//        }

//        Page2Form {
//        }
    }

    footer: TabBar {
        id: tabBar
        currentIndex: swipeView.currentIndex

        TabButton {
            text: qsTr("Remote")
        }
        TabButton {
            text: qsTr("Settings")
        }
    }
}
