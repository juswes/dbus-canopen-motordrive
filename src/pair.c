#include <array.h>
#include <localsettings.h>
#include <logger.h>
#include <node.h>
#include <notification.h>
#include <pair.h>
#include <stdio.h>
#include <string.h>
#include <velib/types/ve_item_def.h>
#include <velib/types/ve_values.h>
#include <velib/utils/ve_item_utils.h>

static DrivePair drivePairs[MAX_DRIVE_PAIRS];

// DS402 mode of operation per node, read once at connection time. Zero means
// unknown, which is also what a controller that does not implement 0x6061
// leaves behind, so such a node never takes part in a pair.
static un8 nodeModes[127];

static VeItem *combinedDriveAuto;
static VeItem *combinedDrives;
static Un8Array configuredPairs;

static struct VeSettingProperties booleanType = {
    .type = VE_SN32,
    .def.value.SN32 = 1,
    .min.value.SN32 = 0,
    .max.value.SN32 = 1,
};

static struct VeSettingProperties combinedDrivesType = {
    .type = VE_STR,
    .def.value.CPtr = "",
};

static Device *connectedDevice(un8 nodeId) {
    Node *node;

    if (nodeId == 0 || nodeId > 127) {
        return NULL;
    }
    node = &nodes[nodeId - 1];
    return node->connected ? node->device : NULL;
}

// Member items exist only on a device exported as a pair primary. Pairing can
// also be switched on by a setting after a device has been published, in which
// case the aggregate values are still correct and only the per-controller
// detail is missing.
static void setIfPresent(VeItem *item, VeVariant *value) {
    if (item != NULL) {
        veItemOwnerSet(item, value);
    }
}

static veBool readFloat(VeItem *item, float *out) {
    VeVariant v;

    veItemLocalValue(item, &v);
    if (!veVariantIsValid(&v)) {
        return veFalse;
    }
    *out = v.value.Float;
    return veTrue;
}

static veBool readSn32(VeItem *item, sn32 *out) {
    VeVariant v;

    veItemLocalValue(item, &v);
    if (!veVariantIsValid(&v)) {
        return veFalse;
    }
    *out = v.value.SN32;
    return veTrue;
}

static veBool readUn16(VeItem *item, un16 *out) {
    VeVariant v;

    veItemLocalValue(item, &v);
    if (!veVariantIsValid(&v)) {
        return veFalse;
    }
    *out = v.value.UN16;
    return veTrue;
}

static veBool readSn16(VeItem *item, sn16 *out) {
    VeVariant v;

    veItemLocalValue(item, &v);
    if (!veVariantIsValid(&v)) {
        return veFalse;
    }
    *out = v.value.SN16;
    return veTrue;
}

static un16 productIdForNode(un8 nodeId) {
    Node *node;

    if (nodeId == 0 || nodeId > 127) {
        return 0;
    }
    node = &nodes[nodeId - 1];
    if (!node->connected || node->device == NULL) {
        return 0;
    }
    return node->device->driver->productId;
}

static veBool addPair(un8 primaryNodeId, un8 secondaryNodeId) {
    size_t i;

    if (primaryNodeId == 0 || secondaryNodeId == 0 ||
        primaryNodeId == secondaryNodeId) {
        return veFalse;
    }
    if (drivePairForNode(primaryNodeId) != NULL ||
        drivePairForNode(secondaryNodeId) != NULL) {
        return veFalse;
    }

    for (i = 0; i < MAX_DRIVE_PAIRS; i += 1) {
        if (drivePairs[i].primaryNodeId != 0) {
            continue;
        }
        drivePairs[i].primaryNodeId = primaryNodeId;
        drivePairs[i].secondaryNodeId = secondaryNodeId;
        drivePairs[i].membersRead = 0;
        drivePairs[i].degradedReported = veFalse;
        drivePairs[i].divergenceReported = veFalse;
        info("combined drive: node %u primary, node %u secondary",
             primaryNodeId, secondaryNodeId);
        return veTrue;
    }

    warning("more than %d combined drives, ignoring nodes %u and %u",
            MAX_DRIVE_PAIRS, primaryNodeId, secondaryNodeId);
    return veFalse;
}

// One controller running the speed loop and another following torque means the
// two are mechanically coupled: independent motors would each need their own
// speed loop. Only pair when there is exactly one of each for a product, since
// anything else is ambiguous and a wrong guess merges two real motor drives
// into one device.
static void detectPairs(void) {
    un8 nodeId;
    un8 velocityNodeId;
    un8 torqueNodeId;
    un8 velocityCount;
    un8 torqueCount;
    un16 productId;
    un8 candidate;

    for (nodeId = 1; nodeId <= 127; nodeId += 1) {
        productId = productIdForNode(nodeId);
        if (productId == 0 || nodeModes[nodeId - 1] == 0) {
            continue;
        }
        if (drivePairForNode(nodeId) != NULL) {
            continue;
        }

        velocityNodeId = 0;
        torqueNodeId = 0;
        velocityCount = 0;
        torqueCount = 0;

        for (candidate = 1; candidate <= 127; candidate += 1) {
            if (productIdForNode(candidate) != productId) {
                continue;
            }
            if (nodeModes[candidate - 1] == DS402_MODE_PROFILE_VELOCITY) {
                velocityCount += 1;
                velocityNodeId = candidate;
            } else if (nodeModes[candidate - 1] == DS402_MODE_PROFILE_TORQUE) {
                torqueCount += 1;
                torqueNodeId = candidate;
            }
        }

        if (velocityCount == 1 && torqueCount == 1) {
            addPair(velocityNodeId, torqueNodeId);
        }
    }
}

static void pairsFromSetting(void) {
    size_t i;

    un8ArrayDeserialize(&configuredPairs, combinedDrives);
    for (i = 0; i + 1 < configuredPairs.count; i += 2) {
        addPair(configuredPairs.data[i], configuredPairs.data[i + 1]);
    }
}

// A pair outlives one of its halves dropping off the bus. Rebuilding purely
// from connected nodes would quietly turn the survivor into a standalone drive
// reporting half the power, which is the whole failure this feature exists to
// avoid. The entry is only dropped once neither half is there.
static void pruneDeadPairs(void) {
    size_t i;

    for (i = 0; i < MAX_DRIVE_PAIRS; i += 1) {
        if (drivePairs[i].primaryNodeId == 0) {
            continue;
        }
        if (connectedDevice(drivePairs[i].primaryNodeId) == NULL &&
            connectedDevice(drivePairs[i].secondaryNodeId) == NULL) {
            memset(&drivePairs[i], 0, sizeof(drivePairs[i]));
        }
    }
}

void updateDrivePairs(void) {
    VeVariant v;

    if (combinedDrives == NULL) {
        return;
    }

    pruneDeadPairs();

    veItemLocalValue(combinedDriveAuto, &v);
    if (veVariantIsValid(&v) && v.value.SN32 == 0) {
        pairsFromSetting();
        return;
    }

    detectPairs();
}

// Changing how pairing is configured starts from nothing, unlike the ordinary
// update which preserves pairs whose members are temporarily absent.
static void onCombinedDriveSettingChanged(VeItem *item) {
    memset(drivePairs, 0, sizeof(drivePairs));
    updateDrivePairs();
}

DrivePair *drivePairForNode(un8 nodeId) {
    size_t i;

    for (i = 0; i < MAX_DRIVE_PAIRS; i += 1) {
        if (drivePairs[i].primaryNodeId == nodeId ||
            drivePairs[i].secondaryNodeId == nodeId) {
            return &drivePairs[i];
        }
    }
    return NULL;
}

veBool isCombinedSecondary(un8 nodeId) {
    DrivePair *pair;

    pair = drivePairForNode(nodeId);
    return pair != NULL && pair->secondaryNodeId == nodeId ? veTrue : veFalse;
}

void markPairMemberRead(DrivePair *pair, un8 nodeId) {
    if (pair->primaryNodeId == nodeId) {
        pair->membersRead |= PAIR_PRIMARY_READ;
    } else if (pair->secondaryNodeId == nodeId) {
        pair->membersRead |= PAIR_SECONDARY_READ;
    }
}

veBool isPairComplete(DrivePair *pair) {
    return pair->membersRead == PAIR_BOTH_READ ? veTrue : veFalse;
}

void clearPairMembersRead(DrivePair *pair) { pair->membersRead = 0; }

void setNodeModeOfOperation(un8 nodeId, un8 mode) {
    if (nodeId == 0 || nodeId > 127) {
        return;
    }
    nodeModes[nodeId - 1] = mode;
    updateDrivePairs();
}

void forgetNodeModeOfOperation(un8 nodeId) {
    if (nodeId == 0 || nodeId > 127) {
        return;
    }
    nodeModes[nodeId - 1] = 0;
    updateDrivePairs();
}

veBool isPairDegraded(DrivePair *pair) {
    veBool primaryHere;
    veBool secondaryHere;

    primaryHere = connectedDevice(pair->primaryNodeId) != NULL;
    secondaryHere = connectedDevice(pair->secondaryNodeId) != NULL;
    return primaryHere != secondaryHere ? veTrue : veFalse;
}

void invalidatePair(DrivePair *pair) {
    Device *primary;
    VeVariant v;
    VeStr name;
    char title[255];

    primary = connectedDevice(pair->primaryNodeId);
    if (primary == NULL) {
        return;
    }

    veItemOwnerSet(primary->current, veVariantInvalidType(&v, VE_FLOAT));
    veItemOwnerSet(primary->power, veVariantInvalidType(&v, VE_SN32));
    veItemOwnerSet(primary->motorTorque, veVariantInvalidType(&v, VE_UN16));
    veItemOwnerSet(primary->motorRpm, veVariantInvalidType(&v, VE_UN16));
    veItemOwnerSet(primary->motorDirection, veVariantInvalidType(&v, VE_UN8));
    veItemOwnerSet(primary->motorTemperature,
                   veVariantInvalidType(&v, VE_SN16));
    veItemOwnerSet(primary->controllerTemperature,
                   veVariantInvalidType(&v, VE_SN16));

    if (pair->degradedReported) {
        return;
    }
    pair->degradedReported = veTrue;

    snprintf(title, sizeof(title),
             "Controller %u of the combined drive is not "
             "responding",
             pair->secondaryNodeId);
    error("combined drive degraded: node %u is missing", pair->secondaryNodeId);
    getDeviceDisplayName(primary, &name);
    queueNotification(pair->primaryNodeId, NOTIFICATION_TYPE_ERROR, title,
                      veStrCStr(&name));
    veStrFree(&name);
}

void drivePairsInit(VeItem *root, const char *settingsPrefix) {
    memset(drivePairs, 0, sizeof(drivePairs));
    memset(nodeModes, 0, sizeof(nodeModes));
    un8ArrayInit(&configuredPairs);

    combinedDriveAuto = veItemCreateSettingsProxy(
        localSettings, settingsPrefix, root, "CombinedDriveAuto", veVariantFmt,
        &veUnitNone, &booleanType);
    combinedDrives = veItemCreateSettingsProxy(
        localSettings, settingsPrefix, root, "CombinedDrives", veVariantFmt,
        &veUnitNone, &combinedDrivesType);

    veItemSetChanged(combinedDriveAuto, onCombinedDriveSettingChanged);
    veItemSetChanged(combinedDrives, onCombinedDriveSettingChanged);
}

// Each controller of a combined drive measures its own share, which the bench
// pair confirmed: peak battery current was 13.3 A on one and 14.9 A on the
// other, a ratio of 1.12 where a controller reporting the pair's total would
// have given roughly 0.5 or 2.0.
static void aggregateCurrentAndPower(Device *primary, Device *secondary) {
    VeVariant v;
    float currentA;
    float currentB;
    sn32 powerA;
    sn32 powerB;

    if (readFloat(primary->current, &currentA) &&
        readFloat(secondary->current, &currentB)) {
        setIfPresent(primary->memberCurrent[0], veVariantFloat(&v, currentA));
        setIfPresent(primary->memberCurrent[1], veVariantFloat(&v, currentB));
        veItemOwnerSet(primary->current,
                       veVariantFloat(&v, currentA + currentB));
    } else {
        veItemOwnerSet(primary->current, veVariantInvalidType(&v, VE_FLOAT));
    }

    if (readSn32(primary->power, &powerA) &&
        readSn32(secondary->power, &powerB)) {
        setIfPresent(primary->memberPower[0], veVariantSn32(&v, powerA));
        setIfPresent(primary->memberPower[1], veVariantSn32(&v, powerB));
        veItemOwnerSet(primary->power, veVariantSn32(&v, powerA + powerB));
    } else {
        veItemOwnerSet(primary->power, veVariantInvalidType(&v, VE_SN32));
    }
}

// One shaft, so both controllers report the same speed and the primary's value
// is the drive's speed. Averaging would only blur a divergence, which is the
// one thing here worth noticing: on the bench pair the two never differed by
// more than 6 RPM at up to 1,408 RPM.
static void aggregateSpeed(DrivePair *pair, Device *primary,
                           Device *secondary) {
    VeVariant v;
    VeStr name;
    char title[255];
    un16 rpmA;
    un16 rpmB;
    un16 difference;
    un32 limit;

    if (!readUn16(primary->motorRpm, &rpmA) ||
        !readUn16(secondary->motorRpm, &rpmB)) {
        return;
    }

    setIfPresent(primary->memberRpm[0], veVariantUn16(&v, rpmA));
    setIfPresent(primary->memberRpm[1], veVariantUn16(&v, rpmB));

    difference = rpmA > rpmB ? rpmA - rpmB : rpmB - rpmA;
    limit = (un32)rpmA * RPM_DIVERGENCE_PERCENT / 100;
    if (limit < RPM_DIVERGENCE_FLOOR) {
        limit = RPM_DIVERGENCE_FLOOR;
    }

    if (difference <= limit) {
        pair->divergenceReported = veFalse;
        return;
    }
    if (pair->divergenceReported) {
        return;
    }
    pair->divergenceReported = veTrue;

    snprintf(title, sizeof(title),
             "Combined drive halves disagree on speed, %u and %u RPM", rpmA,
             rpmB);
    error("combined drive divergence: node %u at %u RPM, node %u at %u RPM",
          pair->primaryNodeId, rpmA, pair->secondaryNodeId, rpmB);
    getDeviceDisplayName(primary, &name);
    queueNotification(pair->primaryNodeId, NOTIFICATION_TYPE_ERROR, title,
                      veStrCStr(&name));
    veStrFree(&name);
}

// One motor and two inverters. The hottest winding and the hotter inverter are
// what derate first, so an average would mask a failure on one side.
static void aggregateTemperatures(Device *primary, Device *secondary) {
    VeVariant v;
    sn16 a;
    sn16 b;

    if (readSn16(primary->motorTemperature, &a) &&
        readSn16(secondary->motorTemperature, &b)) {
        setIfPresent(primary->memberMotorTemperature[0], veVariantSn16(&v, a));
        setIfPresent(primary->memberMotorTemperature[1], veVariantSn16(&v, b));
        veItemOwnerSet(primary->motorTemperature,
                       veVariantSn16(&v, a > b ? a : b));
    }

    if (readSn16(primary->controllerTemperature, &a) &&
        readSn16(secondary->controllerTemperature, &b)) {
        setIfPresent(primary->memberControllerTemperature[0],
                     veVariantSn16(&v, a));
        setIfPresent(primary->memberControllerTemperature[1],
                     veVariantSn16(&v, b));
        veItemOwnerSet(primary->controllerTemperature,
                       veVariantSn16(&v, a > b ? a : b));
    }
}

// Each controller reports its share of shaft torque in real Nm, so the sum is
// the total. Confirmed by power balance on the bench: summed torque against
// summed electrical power gives 94.4% efficiency, where treating either
// controller's figure as the total would not balance.
static void aggregateTorque(Device *primary, Device *secondary) {
    VeVariant v;
    un16 a;
    un16 b;

    if (readUn16(primary->motorTorque, &a) &&
        readUn16(secondary->motorTorque, &b)) {
        veItemOwnerSet(primary->motorTorque, veVariantUn16(&v, a + b));
    } else {
        veItemOwnerSet(primary->motorTorque, veVariantInvalidType(&v, VE_UN16));
    }
}

void aggregatePair(DrivePair *pair) {
    Device *primary;
    Device *secondary;

    primary = connectedDevice(pair->primaryNodeId);
    secondary = connectedDevice(pair->secondaryNodeId);
    if (primary == NULL || secondary == NULL) {
        return;
    }

    // Both halves are here, so a previous absence has been made good.
    pair->degradedReported = veFalse;

    aggregateCurrentAndPower(primary, secondary);
    aggregateSpeed(pair, primary, secondary);
    aggregateTemperatures(primary, secondary);
    aggregateTorque(primary, secondary);

    // Voltage, RPM and direction are left as the primary wrote them: both
    // controllers sit on one DC bus and one shaft, so the primary's readings
    // are the drive's readings.
}
