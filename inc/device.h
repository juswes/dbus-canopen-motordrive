#ifndef __DEVICE_H__
#define __DEVICE_H__

typedef struct _Device Device;

#include <driver.h>
#include <velib/types/variant_print.h>
#include <velib/types/ve_dbus_item.h>
#include <velib/types/ve_item_def.h>
#include <velib/types/ve_str.h>
#include <velib/utils/ve_item_utils.h>

typedef struct _Device {
    un8 nodeId;

    /**
     * Whether this device has been published on D-Bus. Registration is
     * deliberately not done at creation time: a combined drive is only
     * recognised once both of its controllers have connected, and connections
     * complete one at a time. Registering early and withdrawing later would
     * also strand a VRM device instance, since those are allocated from
     * persistent settings and keyed by identifier.
     */
    veBool exported;

    struct VeDbus *dbus;

    char identifier[64];
    sn32 deviceInstance;
    un32 serialNumber;

    VeItem *root;
    VeItem *voltage;
    VeItem *current;
    VeItem *power;
    VeItem *motorRpm;
    VeItem *motorDirection;
    VeItem *motorTemperature;
    VeItem *motorTorque;
    VeItem *controllerTemperature;
    VeItem *motorDirectionInverted;
    VeItem *customName;

    /**
     * Per-controller values, created only on the primary of a combined drive.
     * The boat page reads the aggregated paths above and sees one gauge; the
     * detail page can show what each controller is doing. Index 0 is the
     * primary, index 1 the secondary.
     */
    VeItem *memberCurrent[2];
    VeItem *memberPower[2];
    VeItem *memberRpm[2];
    VeItem *memberMotorTemperature[2];
    VeItem *memberControllerTemperature[2];

    Driver *driver;
    void *driverContext;
} Device;

void getDeviceDisplayName(Device *device, VeStr *out);

/** Builds the identifier and the local item tree. Touches no bus. */
void createDevice(Device *device, un8 nodeId, un32 serialNumber);

/**
 * Claims a device instance and publishes the tree. Until this is called the
 * tree exists but is private, which is what a combined drive's secondary
 * needs: its driver writes into it exactly as usual and nobody sees it.
 * Doing this twice is a no-op.
 */
void exportDevice(Device *device);

void destroyDevice(Device *device);

#endif