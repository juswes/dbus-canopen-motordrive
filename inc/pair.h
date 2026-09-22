#ifndef __PAIR_H__
#define __PAIR_H__

typedef struct _DrivePair DrivePair;

#include <velib/types/ve_item_def.h>

/**
 * A combined drive is two controllers turning one shaft, as in a 6-phase
 * motor. It has to look like a single motor drive to the user, which is the
 * opposite of the dual-drive case where two independent motors are deliberately
 * shown separately.
 *
 * Only the primary is exported on D-Bus. The secondary keeps its own Device
 * and item tree, which its driver writes into exactly as it would otherwise,
 * but that tree is never published. Once both members have completed a read
 * cycle the primary publishes the combined values. No driver is aware of any
 * of this.
 */

#define MAX_DRIVE_PAIRS 4

#define PAIR_PRIMARY_READ 0x01
#define PAIR_SECONDARY_READ 0x02
#define PAIR_BOTH_READ (PAIR_PRIMARY_READ | PAIR_SECONDARY_READ)

/**
 * DS402 modes of operation, as reported by 0x6061. A combined drive shows one
 * controller running the speed loop and the other following torque. Two
 * independent motors would each need their own speed loop, so both would
 * report profile velocity.
 */
#define DS402_MODE_PROFILE_VELOCITY 3
#define DS402_MODE_PROFILE_TORQUE 4

typedef struct _DrivePair {
    un8 primaryNodeId;
    un8 secondaryNodeId;
    un8 membersRead;
} DrivePair;

/**
 * Creates the CombinedDriveAuto and CombinedDrives settings under the given
 * prefix, published on the given root alongside DiscoveredNodes.
 */
void drivePairsInit(VeItem *root, const char *settingsPrefix);

/** The pair a node belongs to, or NULL if it stands alone. */
DrivePair *drivePairForNode(un8 nodeId);

/** True when the node is the secondary of a pair, so must not be exported. */
veBool isCombinedSecondary(un8 nodeId);

void markPairMemberRead(DrivePair *pair, un8 nodeId);
veBool isPairComplete(DrivePair *pair);
void clearPairMembersRead(DrivePair *pair);

/**
 * Record a node's DS402 mode of operation, read once at connection time.
 * Controllers that do not implement 0x6061 never take part in a pair.
 */
void setNodeModeOfOperation(un8 nodeId, un8 mode);
void forgetNodeModeOfOperation(un8 nodeId);

/**
 * Recompute the pair table. Uses the explicit CombinedDrives setting when
 * CombinedDriveAuto is clear, otherwise pairs a profile velocity node with a
 * profile torque node of the same product.
 */
void updateDrivePairs(void);

/**
 * Combines both members' readings into the primary's published items, and
 * fills in the per-controller values alongside. Reads everything before it
 * writes anything, so the primary's own share is still in its items when they
 * are replaced by the total.
 */
void aggregatePair(DrivePair *pair);

#endif
