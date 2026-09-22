#include <list.h>
#include <logger.h>
#include <memory.h>
#include <node.h>
#include <notification.h>
#include <pair.h>
#include <platform.h>
#include <stdio.h>
#include <string.h>
#include <velib/utils/ve_timer.h>

static List *pendingNotifications;

void notificationsInit() { pendingNotifications = listCreate(); }

void queueNotification(un8 nodeId, NotificationType type, const char *title,
                       const char *description) {
    PendingNotification *notification;
    char combinedTitle[255];

    // Both halves of a combined drive appear as one device, so a fault has to
    // say which controller raised it. The node id rather than an index,
    // because that is what the installer sees when scanning the bus, and
    // because the published detail paths are zero based while people count
    // controllers from one.
    if (drivePairForNode(nodeId) != NULL) {
        snprintf(combinedTitle, sizeof(combinedTitle), "%s [node %u]", title,
                 nodeId);
        title = combinedTitle;
    }

    notification = _malloc(sizeof(PendingNotification));
    CHECK_ALLOC(notification);

    notification->nodeId = nodeId;
    notification->type = type;
    notification->title = _strdup(title);
    CHECK_ALLOC(notification->title);
    notification->description = _strdup(description);
    CHECK_ALLOC(notification->description);
    notification->timeout = pltGetCount1ms();

    listAdd(pendingNotifications, notification);
}

void processPendingNotifications() {
    ListItem *item = pendingNotifications->first;
    while (item) {
        ListItem *next = item->next;
        PendingNotification *notification = (PendingNotification *)item->data;
        if (veTick1ms(&notification->timeout,
                      NOTIFICATION_INJECTION_DELAY_MS)) {

            if (isNodeConnected(notification->nodeId)) {
                injectPlatformNotification(notification->type,
                                           notification->title,
                                           notification->description);
            } else {
                warning("Ignoring notification for node %u since it is not "
                        "connected",
                        notification->nodeId);
            }

            _free(notification->title);
            _free(notification->description);
            _free(notification);
            listRemove(pendingNotifications, item);
        }
        item = next;
    }
}
