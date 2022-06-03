/******************************************************************
 * telemetry code
 ******************************************************************/

#include "rover.h"

extern uint16_t loopTime;

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
    // char loopTimeStatus[120];
    // sprintf(loopTimeStatus, "Loop time: %d ms", loopTime);
    // msg->status->assign(loopTimeStatus);
    return msg;
}
