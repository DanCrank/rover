/******************************************************************
 * telemetry code
 ******************************************************************/

#include "rover.h"

TelemetryMessage * collectTelemetry() {
    auto *msg = new TelemetryMessage();
    // status
    // msg->status->assign("READY FOR ADVENTURE");
    // temporarily using this for lidar data until I add it to the telemetry message
    int obstruction = navFindObstruction();
    if (obstruction > 0) {
        msg->status->assign("Obstruction in sector ");
        msg->status->append(String(obstruction).c_str());
    } else
        msg->status->assign("No obstructions seen");
    return msg;
}
