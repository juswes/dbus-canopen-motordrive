#include <canopen.h>
#include <drivers/dmc.h>
#include <localsettings.h>
#include <logger.h>
#include <math.h>
#include <node.h>
#include <notification.h>
#include <stdio.h>
#include <stdlib.h>
#include <velib/vecan/products.h>

// Temperatures are reported as an unsigned byte with a fixed offset.
#define TEMPERATURE_OFFSET 51

typedef struct {
    un8 faultCode;
    char const *error;
} Error;

// Base fault codes, see Sigma2N Traction Advanced Manual V1.11 section 8.1.
// The sub fault code is reported alongside but not decoded, as its meaning
// depends on the base fault code and on the motor module in use.
static Error errorDb[] = {{2, "Voltage getting low"},
                          {3, "Inhibit drive / BDI cut / BCL via CAN"},
                          {4, "Voltage getting high"},
                          {5, "Motor temperature high"},
                          {6, "Controller temperature high"},
                          {7, "Adjustment out of range"},
                          {8, "Default adjustments used"},
                          {9, "Memory chip fault"},
                          {10, "Both forward and reverse inputs active"},
                          {11, "Drive not allowed"},
                          {12, "Power up sequence fault"},
                          {13, "Accelerator more than 50% at power up"},
                          {14, "Bellyswitch and inching fault"},
                          {15, "Supply voltage fault"},
                          {16, "Dual motor soft error"},
                          {17, "Battery voltage too low"},
                          {18, "High sided mosfets short circuit"},
                          {19, "Motor stall protection"},
                          {20, "Hardware over current detected"},
                          {21, "Contactor coil driver fault"},
                          {22, "Battery voltage too high"},
                          {23, "Low sided mosfets short circuit in neutral"},
                          {24, "Hardware fail safe fault"},
                          {25, "Line contactor fault"},
                          {26, "Thermal shutdown fault"},
                          {27, "Low sided mosfets short circuit at power up"},
                          {28, "Wire off detected"},
                          {29, "CAN node fault"},
                          {30, "Motor overspeeding"},
                          {31, "Motor fault"},
                          {32, "Motor module initialization error"},
                          {33, "Motor module configuration inconsistency"},
                          {34, "Motor parameter inconsistency"},
                          {35, "Current sensor calibration fault"},
                          {36, "Controller temperature over 100 degrees"},
                          {39, "Generic time out"},
                          {40, "System fault"}};

static Error *findError(un8 faultCode) {
    size_t dbSize = sizeof(errorDb) / sizeof(Error);
    for (size_t i = 0; i < dbSize; i++) {
        if (errorDb[i].faultCode == faultCode) {
            return &errorDb[i];
        }
    }
    return NULL;
}

static void onBatteryVoltageResponse(CanOpenPendingSdoRequest *request) {
    VeVariant v;
    Node *node;
    float voltage;

    node = (Node *)request->context;
    if (!node->connected) {
        return;
    }

    voltage = ((sn16)request->response.data) * 0.1F;

    veItemOwnerSet(node->device->voltage, veVariantFloat(&v, voltage));
    veItemLocalValue(node->device->current, &v);
    veItemOwnerSet(node->device->power,
                   veVariantSn32(&v, (sn32)(voltage * v.value.Float)));
}

static void onBatteryCurrentResponse(CanOpenPendingSdoRequest *request) {
    VeVariant v;
    Node *node;
    float current;

    node = (Node *)request->context;
    if (!node->connected) {
        return;
    }

    current = ((sn16)request->response.data) * 0.1F;

    veItemOwnerSet(node->device->current, veVariantFloat(&v, current));

    veItemLocalValue(node->device->voltage, &v);
    veItemOwnerSet(node->device->power,
                   veVariantSn32(&v, (sn32)(v.value.Float * current)));
}

static void onMotorRpmResponse(CanOpenPendingSdoRequest *request) {
    VeVariant v;
    Node *node;
    sn32 rpm;
    un8 motorDirection;
    veBool motorDirectionInverted;

    node = (Node *)request->context;
    if (!node->connected) {
        return;
    }

    rpm = (sn32)request->response.data;

    veItemOwnerSet(node->device->motorRpm, veVariantUn16(&v, abs(rpm)));

    veItemLocalValue(node->device->motorDirectionInverted, &v);
    motorDirectionInverted = v.value.SN32 == 1;
    // 0 - neutral, 1 - reverse, 2 - forward
    if (rpm > 0) {
        motorDirection = motorDirectionInverted ? 1 : 2;
    } else if (rpm < 0) {
        motorDirection = motorDirectionInverted ? 2 : 1;
    } else {
        motorDirection = 0;
    }
    veItemOwnerSet(node->device->motorDirection,
                   veVariantUn8(&v, motorDirection));
}

static void onMotorTemperatureResponse(CanOpenPendingSdoRequest *request) {
    Node *node;
    VeVariant v;

    node = (Node *)request->context;
    if (!node->connected) {
        return;
    }

    veItemOwnerSet(
        node->device->motorTemperature,
        veVariantSn16(&v, ((un8)request->response.data) - TEMPERATURE_OFFSET));
}

static void onMotorTorqueResponse(CanOpenPendingSdoRequest *request) {
    Node *node;
    VeVariant v;
    float torque;

    node = (Node *)request->context;
    if (!node->connected) {
        return;
    }

    torque = ((sn16)request->response.data) * 0.1F;

    veItemOwnerSet(node->device->motorTorque,
                   veVariantFloat(&v, fabsf(torque)));
}

static void onControllerTemperatureResponse(CanOpenPendingSdoRequest *request) {
    Node *node;
    VeVariant v;

    node = (Node *)request->context;
    if (!node->connected) {
        return;
    }

    veItemOwnerSet(
        node->device->controllerTemperature,
        veVariantSn16(&v, ((un8)request->response.data) - TEMPERATURE_OFFSET));
}

static void onError(CanOpenPendingSdoRequest *request, CanOpenError error) {
    Node *node;

    node = (Node *)request->context;
    if (!node->connected) {
        return;
    }

    disconnectFromNode(node->device->nodeId);
}

static void readRoutine(Node *node) {
    canOpenReadSdoAsync(node->device->nodeId, 0x383f, 0, node,
                        onBatteryVoltageResponse, onError);
    canOpenReadSdoAsync(node->device->nodeId, 0x383e, 0, node,
                        onBatteryCurrentResponse, onError);
    canOpenReadSdoAsync(node->device->nodeId, 0x606c, 0, node,
                        onMotorRpmResponse, onError);
    canOpenReadSdoAsync(node->device->nodeId, 0x3836, 0, node,
                        onMotorTemperatureResponse, onError);
    canOpenReadSdoAsync(node->device->nodeId, 0x411c, 0, node,
                        onMotorTorqueResponse, onError);
    canOpenReadSdoAsync(node->device->nodeId, 0x3837, 0, node,
                        onControllerTemperatureResponse, onError);
}

static void fastReadRoutine(Node *node) {
    canOpenReadSdoAsync(node->device->nodeId, 0x606c, 0, node,
                        onMotorRpmResponse, onError);
}

// EMCY payload, see DMC Advanced CAN Open manual V1.10:
// bytes 0-1 emergency error code, byte 2 error register, byte 3 DMC fault
// code, bytes 4-7 DMC fault subcode.
static void onEMCYMessage(Node *node, VeRawCanMsg *message) {
    un16 errorCode;
    un8 faultCode;
    un32 faultSubcode;
    Error *error;
    char notificationTitle[255];
    VeStr deviceName;

    errorCode = message->mdata[0] | (message->mdata[1] << 8);
    faultCode = message->mdata[3];
    faultSubcode = message->mdata[4] | (message->mdata[5] << 8) |
                   (message->mdata[6] << 16) | ((un32)message->mdata[7] << 24);

    if (errorCode == 0) {
        // Fault cleared
        return;
    }

    error = findError(faultCode);
    if (error != NULL) {
        snprintf(notificationTitle, sizeof(notificationTitle), "%s (F%d S%u)",
                 error->error, faultCode, faultSubcode);
    } else {
        snprintf(notificationTitle, sizeof(notificationTitle),
                 "Unknown fault (F%d S%u)", faultCode, faultSubcode);
    }

    error("EMCY from node %d: %s", node->device->nodeId, notificationTitle);
    getDeviceDisplayName(node->device, &deviceName);
    queueNotification(node->device->nodeId, NOTIFICATION_TYPE_ERROR,
                      notificationTitle, veStrCStr(&deviceName));
    veStrFree(&deviceName);
}

Driver dmcDriver = {
    .name = "dmc",
    .productId = VE_PROD_ID_DMC_MOTORDRIVE,
    .readRoutine = readRoutine,
    .fastReadRoutine = fastReadRoutine,
    .createDriverContext = NULL,
    .destroyDriverContext = NULL,
    .onEMCYMessage = onEMCYMessage,
};
