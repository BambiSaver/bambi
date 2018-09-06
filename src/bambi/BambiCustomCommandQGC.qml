import QtQuick 2.2

import QGroundControl.Controls      1.0
import QGroundControl.FactSystem    1.0
import QGroundControl.FactControls  1.0
import QGroundControl.Palette       1.0
import QGroundControl.ScreenTools   1.0
import QGroundControl.Controllers   1.0

Rectangle {
    anchors.fill:   parent
    color:          qgcPal.window

    CustomCommandWidgetController {
        id:         controller
        factPanel:  panel
    }

    QGCPalette { id: qgcPal; colorGroupEnabled: enabled }

    Column {
        spacing: ScreenTools.defaultFontPixelHeight

        QGCButton {
            text: "BAMBI MISSION START"
            // Arguments to CustomCommandWidgetController::sendCommand (MAVLink COMMAND_LONG)
            //   command id
            //   component id
            //   confirmation
            //   param 1-7
            onClicked: controller.sendCommand(2720, 240, 0, 1, 10, 46.453072, 11.492048, 0, 10.0, 1445.0)
        }

        QGCButton {
            text: "BAMBI MISSION STOP"
            // Arguments to CustomCommandWidgetController::sendCommand (MAVLink COMMAND_LONG)
            //   command id
            //   component id
            //   confirmation
            //   param 1-7
            onClicked: controller.sendCommand(2720, 240, 0, 0, 10, 46.453072, 11.492048, 0, 10.0, 1445.0)
        }

        // The FactTextField control is bound to the specified parameter. Note that there is no validation.
        // FactTextField {
        //     // The -1 signals default component id.
        //     // You can replace it with a specific component id if you like
        //     fact: controller.getParameterFact(-1, "MAV_SYS_ID")
        // }
    }
}