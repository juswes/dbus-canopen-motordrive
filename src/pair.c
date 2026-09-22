#include <array.h>
#include <localsettings.h>
#include <logger.h>
#include <node.h>
#include <pair.h>
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

void updateDrivePairs(void) {
    VeVariant v;

    memset(drivePairs, 0, sizeof(drivePairs));

    if (combinedDrives == NULL) {
        return;
    }

    veItemLocalValue(combinedDriveAuto, &v);
    if (veVariantIsValid(&v) && v.value.SN32 == 0) {
        pairsFromSetting();
        return;
    }

    detectPairs();
}

static void onCombinedDriveSettingChanged(VeItem *item) { updateDrivePairs(); }

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
