#include <discovery.h>
#include <logger.h>
#include <memory.h>
#include <node.h>
#include <pair.h>
#include <servicemanager.h>
#include <stdlib.h>
#include <string.h>
#include <velib/platform/plt.h>

Node nodes[127];

static void finalizeConnection(ConnectionAttempt *attempt, un32 serialNumber) {
    Node *node;

    node = &nodes[attempt->nodeId - 1];
    node->device = _malloc(sizeof(*node->device));
    CHECK_ALLOC(node->device);
    node->device->driver = attempt->driver;

    createDevice(node->device, attempt->nodeId, serialNumber);
    node->connected = veTrue;
    if (node->device->driver->createDriverContext != NULL) {
        node->device->driverContext =
            node->device->driver->createDriverContext(node);
    }

    _free(attempt);
}

// 0x6061 is the DS402 modes of operation display. It tells a combined drive
// apart from independent motors: one controller runs the speed loop while the
// other follows torque.
//
// Read after the node is connected rather than as part of the handshake, so
// connecting is unchanged for every existing driver. Nothing waits on the
// answer: the request is queued before any read routine can run, and a device
// is not published until a read cycle completes, which cannot happen while the
// queue still holds this request. So the mode is always known in time.
//
// A controller that does not implement 0x6061 simply never takes part in a
// pair, so a failure here is not a reason to reject the node.
static void onModeOfOperationResponse(CanOpenPendingSdoRequest *request) {
    Node *node;

    node = (Node *)request->context;
    if (!node->connected) {
        return;
    }
    setNodeModeOfOperation(node->device->nodeId, (un8)request->response.data);
}

static void onModeOfOperationError(CanOpenPendingSdoRequest *request,
                                   CanOpenError error) {
    Node *node;

    node = (Node *)request->context;
    if (!node->connected) {
        return;
    }
    setNodeModeOfOperation(node->device->nodeId, 0);
}

static void readModeOfOperation(un8 nodeId) {
    canOpenReadSdoAsync(nodeId, 0x6061, 0, &nodes[nodeId - 1],
                        onModeOfOperationResponse, onModeOfOperationError);
}

static void
onControllerSerialNumberResponse(CanOpenPendingSdoRequest *request) {
    un8 nodeId;

    nodeId = ((ConnectionAttempt *)request->context)->nodeId;
    finalizeConnection((ConnectionAttempt *)request->context,
                       request->response.data);
    readModeOfOperation(nodeId);
}

static veBool
shouldFallbackToNodeIdForSerialNumber(CanOpenPendingSdoRequest *request,
                                      CanOpenError error) {
    if (error != SDO_READ_ERROR) {
        return veFalse;
    }
    if (request->response.control != SDO_ABORT_CONTROL) {
        return veFalse;
    }
    return request->response.data == SDO_ABORT_NO_OBJECT ||
           request->response.data == SDO_ABORT_NO_SUB_INDEX;
}

static void onControllerSerialNumberError(CanOpenPendingSdoRequest *request,
                                          CanOpenError error) {
    ConnectionAttempt *attempt;
    un8 nodeId;

    attempt = (ConnectionAttempt *)request->context;
    if (shouldFallbackToNodeIdForSerialNumber(request, error)) {
        // SDO 0x1018.04 (serial number) is not supported by node.
        // Falling back to using CANopen node ID.
        nodeId = attempt->nodeId;
        finalizeConnection(attempt, nodeId);
        readModeOfOperation(nodeId);
        return;
    }

    _free(attempt);
}

static void onDiscoverNodeSuccess(un8 nodeId, void *context, Driver *driver) {
    ConnectionAttempt *attempt;

    attempt = (ConnectionAttempt *)context;
    attempt->driver = driver;

    canOpenReadSdoAsync(attempt->nodeId, 0x1018, 4, attempt,
                        onControllerSerialNumberResponse,
                        onControllerSerialNumberError);
}

static void onDiscoverNodeError(un8 nodeId, void *context) {
    ConnectionAttempt *attempt;

    attempt = (ConnectionAttempt *)context;
    _free(attempt);
}

void connectToNode(un8 nodeId) {
    ConnectionAttempt *attempt;

    attempt = _malloc(sizeof(*attempt));
    CHECK_ALLOC(attempt);

    attempt->nodeId = nodeId;
    attempt->length = 0;
    attempt->driver = NULL;

    discoverNode(nodeId, onDiscoverNodeSuccess, onDiscoverNodeError,
                 (void *)attempt);
}

void disconnectFromNode(un8 nodeId) {
    Node *node;

    node = &nodes[nodeId - 1];
    if (!node->connected) {
        return;
    }
    destroyDevice(node->device);
    if (node->device->driver->destroyDriverContext != NULL) {
        node->device->driver->destroyDriverContext(node,
                                                   node->device->driverContext);
    }
    _free(node->device);
    node->device = NULL;
    node->connected = veFalse;

    // Only once the node no longer counts as connected, otherwise detection
    // would immediately pair it again.
    forgetNodeModeOfOperation(nodeId);
}

void connectToDiscoveredNodes() {
    size_t i;
    un8 *ptr;
    un8 nodeId;
    Node *node;

    for (ptr = serviceManager.discoveredNodeIds.data, i = 0;
         i < serviceManager.discoveredNodeIds.count; i += 1, ptr += 1) {
        nodeId = *ptr;
        node = &nodes[nodeId - 1];
        if (!node->connected) {
            connectToNode(nodeId);
        }
    }
}

static Device *deviceForNode(un8 nodeId) {
    Node *node;

    if (nodeId == 0 || nodeId > 127) {
        return NULL;
    }
    node = &nodes[nodeId - 1];
    return node->connected ? node->device : NULL;
}

static void onReadRoutineComplete(CanOpenPendingSdoRequest *request) {
    Node *node;
    DrivePair *pair;
    Device *primary;

    node = (Node *)request->context;
    if (!node->connected) {
        return;
    }

    pair = drivePairForNode(node->device->nodeId);
    if (pair == NULL) {
        exportDevice(node->device);
        veItemSendPendingChanges(node->device->root);
        return;
    }

    // With one half gone there is nothing to wait for, and the survivor must
    // not keep publishing its own share as though it were the whole drive.
    // If the missing half is the primary, nothing is published at all: the
    // secondary follows torque and has no speed reference of its own, so its
    // readings would be misleading rather than merely incomplete.
    if (isPairDegraded(pair)) {
        primary = deviceForNode(pair->primaryNodeId);
        if (primary != NULL) {
            exportDevice(primary);
            invalidatePair(pair);
            veItemSendPendingChanges(primary->root);
        }
        clearPairMembersRead(pair);
        return;
    }

    // Hold the publish until both halves have reported, so the two samples
    // come from the same cycle. Nodes are read in ascending id order, so
    // publishing on the primary's own callback would always combine the
    // secondary's previous cycle.
    markPairMemberRead(pair, node->device->nodeId);
    if (!isPairComplete(pair)) {
        return;
    }
    clearPairMembersRead(pair);

    primary = deviceForNode(pair->primaryNodeId);
    if (primary == NULL) {
        return;
    }

    // Only the primary is ever published, and it has to be exported before
    // aggregating: exporting is what creates the per-controller items that
    // aggregation writes into.
    exportDevice(primary);
    aggregatePair(pair);
    veItemSendPendingChanges(primary->root);
}

void readFromConnectedNodes(veBool fast) {
    un8 nodeId;
    Node *node;

    // @todo:
    // The queue still hasn't cleared up from the last read cycle,
    // Perhaps we will need a queue per node at some point
    if (canOpenState.pendingSdoRequests->first != NULL) {
        return;
    }

    for (nodeId = 1, node = nodes; nodeId <= 127; nodeId += 1, node += 1) {
        if (node->connected) {
            if (fast == veTrue) {
                node->device->driver->fastReadRoutine(node);
            } else {
                node->device->driver->readRoutine(node);
            }
            canOpenQueueCallbackAsync(node, onReadRoutineComplete);
        }
    }
}

void nodesEmcyHandler(void *context, un8 nodeId, VeRawCanMsg *message) {
    Node *node;

    node = &nodes[nodeId - 1];
    if (node->connected) {
        node->device->driver->onEMCYMessage(node, message);
    } else {
        warning("Unhandled EMCY message from node %u", nodeId);
    }
}

void nodesInit() { memset(nodes, 0, sizeof(nodes)); }

veBool isNodeConnected(un8 nodeId) {
    if (nodeId == 0 || nodeId > 127) {
        return veFalse;
    }
    return nodes[nodeId - 1].connected;
}