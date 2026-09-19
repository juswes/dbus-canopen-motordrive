#include "CanFixture.hpp"

extern "C" {
#include "canopen.h"
#include "drivers/dmc.h"
#include "node.h"
#include "servicemanager.h"
}

static char lastTitle[1024];
static char lastDescription[1024];
static void injectPlatformNotificationLocal(NotificationType type,
                                            char const *title,
                                            char const *description) {
    strncpy(lastTitle, title, sizeof(lastTitle));
    strncpy(lastDescription, description, sizeof(lastDescription));
}

class DmcTest : public CanFixture {
  protected:
    void SetUp() override {
        CanFixture::SetUp();

        injectPlatformNotification_fake.custom_fake =
            injectPlatformNotificationLocal;

        canOpenInit();
        nodesInit();
        notificationsInit();
    }

    void TearDown() override {
        CanFixture::TearDown();

        listDestroy(canOpenState.pendingSdoRequests);
        canOpenState.pendingSdoRequests = NULL;
    }

    // "Sigma2N IPM Traction" is 20 characters, so the controller name is read
    // with a segmented SDO transfer.
    void connectToDmcNode() {
        connectToNode(1);
        canOpenTx();
        this->canMsgReadQueue.push_back(
            {.canId = 0x581,
             .length = 8,
             .mdata = {0x41, 0x08, 0x10, 0x00, 0x14, 0x00, 0x00, 0x00}});
        canOpenRx();
        // "Sigma2N"
        this->canMsgReadQueue.push_back(
            {.canId = 0x581,
             .length = 8,
             .mdata = {0x00, 0x53, 0x69, 0x67, 0x6D, 0x61, 0x32, 0x4E}});
        canOpenRx();
        // " IPM Tr"
        this->canMsgReadQueue.push_back(
            {.canId = 0x581,
             .length = 8,
             .mdata = {0x10, 0x20, 0x49, 0x50, 0x4D, 0x20, 0x54, 0x72}});
        canOpenRx();
        // "action"
        this->canMsgReadQueue.push_back(
            {.canId = 0x581,
             .length = 8,
             .mdata = {0x03, 0x61, 0x63, 0x74, 0x69, 0x6F, 0x6E, 0x00}});
        canOpenRx();
        canOpenTx();
        this->canMsgReadQueue.push_back(
            {.canId = 0x581,
             .length = 8,
             .mdata = {0x43, 0x18, 0x10, 0x04, 0x01, 0x00, 0x00, 0x00}});
        canOpenRx();
    }

    void drainQueue(std::vector<VeRawCanMsg> &queue) {
        while (!queue.empty()) {
            this->canMsgReadQueue.push_back(queue.front());
            queue.erase(queue.begin());
            canOpenTx();
            canOpenRx();
        }
    }
};

TEST_F(DmcTest, readSuccess) {
    VeRawCanMsg message;

    EXPECT_EQ(nodes[0].connected, veFalse);
    connectToDmcNode();
    EXPECT_EQ(nodes[0].connected, veTrue);

    this->canMsgSentLog.clear();

    nodes[0].device->driver->readRoutine(&nodes[0]);
    EXPECT_EQ(listCount(canOpenState.pendingSdoRequests), 6);

    std::vector<VeRawCanMsg> queue;
    // Battery Voltage, 525 = 52.5V
    queue.push_back(
        {.canId = 0x581,
         .length = 8,
         .mdata = {0x4B, 0x3F, 0x38, 0x00, 0x0D, 0x02, 0x00, 0x00}});
    // Battery Current, 100 = 10.0A
    queue.push_back(
        {.canId = 0x581,
         .length = 8,
         .mdata = {0x4B, 0x3E, 0x38, 0x00, 0x64, 0x00, 0x00, 0x00}});
    // Motor RPM, 500
    queue.push_back(
        {.canId = 0x581,
         .length = 8,
         .mdata = {0x43, 0x6C, 0x60, 0x00, 0xF4, 0x01, 0x00, 0x00}});
    // Motor Temperature, 76 = 25°C
    queue.push_back(
        {.canId = 0x581,
         .length = 8,
         .mdata = {0x4F, 0x36, 0x38, 0x00, 0x4C, 0x00, 0x00, 0x00}});
    // Motor Torque, 800 = 80.0Nm
    queue.push_back(
        {.canId = 0x581,
         .length = 8,
         .mdata = {0x4B, 0x1C, 0x41, 0x00, 0x20, 0x03, 0x00, 0x00}});
    // Controller Temperature, 81 = 30°C
    queue.push_back(
        {.canId = 0x581,
         .length = 8,
         .mdata = {0x4F, 0x37, 0x38, 0x00, 0x51, 0x00, 0x00, 0x00}});

    drainQueue(queue);

    EXPECT_EQ(this->canMsgSentLog.size(), 6);

    message = this->canMsgSentLog.at(0);
    EXPECT_EQ(message.canId, 0x601);
    EXPECT_EQ(message.length, 8);
    EXPECT_EQ(message.mdata[0], 0x40);
    EXPECT_EQ(message.mdata[1], 0x3F);
    EXPECT_EQ(message.mdata[2], 0x38);
    EXPECT_EQ(message.mdata[3], 0x00);

    message = this->canMsgSentLog.at(1);
    EXPECT_EQ(message.canId, 0x601);
    EXPECT_EQ(message.length, 8);
    EXPECT_EQ(message.mdata[0], 0x40);
    EXPECT_EQ(message.mdata[1], 0x3E);
    EXPECT_EQ(message.mdata[2], 0x38);
    EXPECT_EQ(message.mdata[3], 0x00);

    message = this->canMsgSentLog.at(2);
    EXPECT_EQ(message.canId, 0x601);
    EXPECT_EQ(message.length, 8);
    EXPECT_EQ(message.mdata[0], 0x40);
    EXPECT_EQ(message.mdata[1], 0x6C);
    EXPECT_EQ(message.mdata[2], 0x60);
    EXPECT_EQ(message.mdata[3], 0x00);

    message = this->canMsgSentLog.at(3);
    EXPECT_EQ(message.canId, 0x601);
    EXPECT_EQ(message.length, 8);
    EXPECT_EQ(message.mdata[0], 0x40);
    EXPECT_EQ(message.mdata[1], 0x36);
    EXPECT_EQ(message.mdata[2], 0x38);
    EXPECT_EQ(message.mdata[3], 0x00);

    message = this->canMsgSentLog.at(4);
    EXPECT_EQ(message.canId, 0x601);
    EXPECT_EQ(message.length, 8);
    EXPECT_EQ(message.mdata[0], 0x40);
    EXPECT_EQ(message.mdata[1], 0x1C);
    EXPECT_EQ(message.mdata[2], 0x41);
    EXPECT_EQ(message.mdata[3], 0x00);

    message = this->canMsgSentLog.at(5);
    EXPECT_EQ(message.canId, 0x601);
    EXPECT_EQ(message.length, 8);
    EXPECT_EQ(message.mdata[0], 0x40);
    EXPECT_EQ(message.mdata[1], 0x37);
    EXPECT_EQ(message.mdata[2], 0x38);
    EXPECT_EQ(message.mdata[3], 0x00);

    EXPECT_EQ(canOpenState.pendingSdoRequests->first, nullptr);

    EXPECT_FLOAT_EQ(nodes[0].device->voltage->variant.value.Float, 52.5F);
    EXPECT_FLOAT_EQ(nodes[0].device->current->variant.value.Float, 10.0F);
    EXPECT_EQ(nodes[0].device->power->variant.value.SN32, 525);
    EXPECT_EQ(nodes[0].device->motorRpm->variant.value.UN16, 500);
    EXPECT_EQ(nodes[0].device->motorTemperature->variant.value.SN16, 25);
    EXPECT_FLOAT_EQ(nodes[0].device->motorTorque->variant.value.Float, 80.0F);
    EXPECT_EQ(nodes[0].device->controllerTemperature->variant.value.SN16, 30);
    EXPECT_EQ(nodes[0].device->motorDirection->variant.value.UN8, 2);
}

// Battery current is signed, so regenerative braking gives a negative current
// and a negative power.
TEST_F(DmcTest, regenerativeBraking) {
    EXPECT_EQ(nodes[0].connected, veFalse);
    connectToDmcNode();
    EXPECT_EQ(nodes[0].connected, veTrue);

    this->canMsgSentLog.clear();

    nodes[0].device->driver->readRoutine(&nodes[0]);

    std::vector<VeRawCanMsg> queue;
    // Battery Voltage, 525 = 52.5V
    queue.push_back(
        {.canId = 0x581,
         .length = 8,
         .mdata = {0x4B, 0x3F, 0x38, 0x00, 0x0D, 0x02, 0x00, 0x00}});
    // Battery Current, -225 = -22.5A
    queue.push_back(
        {.canId = 0x581,
         .length = 8,
         .mdata = {0x4B, 0x3E, 0x38, 0x00, 0x1F, 0xFF, 0x00, 0x00}});
    // Motor RPM, 500
    queue.push_back(
        {.canId = 0x581,
         .length = 8,
         .mdata = {0x43, 0x6C, 0x60, 0x00, 0xF4, 0x01, 0x00, 0x00}});
    // Motor Temperature, 76 = 25°C
    queue.push_back(
        {.canId = 0x581,
         .length = 8,
         .mdata = {0x4F, 0x36, 0x38, 0x00, 0x4C, 0x00, 0x00, 0x00}});
    // Motor Torque, -800 = -80.0Nm
    queue.push_back(
        {.canId = 0x581,
         .length = 8,
         .mdata = {0x4B, 0x1C, 0x41, 0x00, 0xE0, 0xFC, 0x00, 0x00}});
    // Controller Temperature, 81 = 30°C
    queue.push_back(
        {.canId = 0x581,
         .length = 8,
         .mdata = {0x4F, 0x37, 0x38, 0x00, 0x51, 0x00, 0x00, 0x00}});

    drainQueue(queue);

    EXPECT_FLOAT_EQ(nodes[0].device->current->variant.value.Float, -22.5F);
    EXPECT_EQ(nodes[0].device->power->variant.value.SN32, -1181);
    EXPECT_FLOAT_EQ(nodes[0].device->motorTorque->variant.value.Float, 80.0F);
}

// Temperatures are reported as an unsigned byte with a fixed offset of -51,
// so a raw value below 51 is a temperature below zero.
TEST_F(DmcTest, subZeroTemperatures) {
    EXPECT_EQ(nodes[0].connected, veFalse);
    connectToDmcNode();
    EXPECT_EQ(nodes[0].connected, veTrue);

    this->canMsgSentLog.clear();

    nodes[0].device->driver->readRoutine(&nodes[0]);

    std::vector<VeRawCanMsg> queue;
    // Battery Voltage, 525 = 52.5V
    queue.push_back(
        {.canId = 0x581,
         .length = 8,
         .mdata = {0x4B, 0x3F, 0x38, 0x00, 0x0D, 0x02, 0x00, 0x00}});
    // Battery Current, 100 = 10.0A
    queue.push_back(
        {.canId = 0x581,
         .length = 8,
         .mdata = {0x4B, 0x3E, 0x38, 0x00, 0x64, 0x00, 0x00, 0x00}});
    // Motor RPM, 500
    queue.push_back(
        {.canId = 0x581,
         .length = 8,
         .mdata = {0x43, 0x6C, 0x60, 0x00, 0xF4, 0x01, 0x00, 0x00}});
    // Motor Temperature, 41 = -10°C
    queue.push_back(
        {.canId = 0x581,
         .length = 8,
         .mdata = {0x4F, 0x36, 0x38, 0x00, 0x29, 0x00, 0x00, 0x00}});
    // Motor Torque, 0
    queue.push_back(
        {.canId = 0x581,
         .length = 8,
         .mdata = {0x4B, 0x1C, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00}});
    // Controller Temperature, 0 = -51°C
    queue.push_back(
        {.canId = 0x581,
         .length = 8,
         .mdata = {0x4F, 0x37, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00}});

    drainQueue(queue);

    EXPECT_EQ(nodes[0].device->motorTemperature->variant.value.SN16, -10);
    EXPECT_EQ(nodes[0].device->controllerTemperature->variant.value.SN16, -51);
}

TEST_F(DmcTest, skipResponseOnDisconnect) {
    EXPECT_EQ(nodes[0].connected, veFalse);
    connectToDmcNode();
    EXPECT_EQ(nodes[0].connected, veTrue);

    this->canMsgSentLog.clear();

    nodes[0].device->driver->readRoutine(&nodes[0]);
    EXPECT_EQ(listCount(canOpenState.pendingSdoRequests), 6);

    std::vector<VeRawCanMsg> queue;
    queue.push_back(
        {.canId = 0x581,
         .length = 8,
         .mdata = {0x4B, 0x3F, 0x38, 0x00, 0x0D, 0x02, 0x00, 0x00}});
    queue.push_back(
        {.canId = 0x581,
         .length = 8,
         .mdata = {0x4B, 0x3E, 0x38, 0x00, 0x64, 0x00, 0x00, 0x00}});
    queue.push_back(
        {.canId = 0x581,
         .length = 8,
         .mdata = {0x43, 0x6C, 0x60, 0x00, 0xF4, 0x01, 0x00, 0x00}});
    queue.push_back(
        {.canId = 0x581,
         .length = 8,
         .mdata = {0x4F, 0x36, 0x38, 0x00, 0x4C, 0x00, 0x00, 0x00}});
    queue.push_back(
        {.canId = 0x581,
         .length = 8,
         .mdata = {0x4B, 0x1C, 0x41, 0x00, 0x20, 0x03, 0x00, 0x00}});
    queue.push_back(
        {.canId = 0x581,
         .length = 8,
         .mdata = {0x4F, 0x37, 0x38, 0x00, 0x51, 0x00, 0x00, 0x00}});

    disconnectFromNode(1);

    drainQueue(queue);

    EXPECT_EQ(this->canMsgSentLog.size(), 6);

    EXPECT_EQ(listCount(canOpenState.pendingSdoRequests), 0);
}

// Every request in the read routine times out, so onError disconnects the
// node on the first one and returns early for the rest.
TEST_F(DmcTest, readErrorTimeout) {
    EXPECT_EQ(nodes[0].connected, veFalse);
    connectToDmcNode();
    EXPECT_EQ(nodes[0].connected, veTrue);

    this->canMsgSentLog.clear();

    nodes[0].device->driver->readRoutine(&nodes[0]);
    EXPECT_EQ(listCount(canOpenState.pendingSdoRequests), 6);

    while (listCount(canOpenState.pendingSdoRequests) > 0) {
        canOpenTx();
        pltGetCount1ms_fake.return_val += 50;
        canOpenTx();
    }

    EXPECT_EQ(this->canMsgSentLog.size(), 6);
    EXPECT_EQ(listCount(canOpenState.pendingSdoRequests), 0);
    EXPECT_EQ(nodes[0].connected, veFalse);
}

// Motor RPM comes from the DS402 object 0x606C, which is a signed 32 bit
// value, so a negative speed is sign extended across all four data bytes.
TEST_F(DmcTest, motorDirection) {
    VeVariant v;

    EXPECT_EQ(nodes[0].connected, veFalse);
    connectToDmcNode();
    EXPECT_EQ(nodes[0].connected, veTrue);

    this->canMsgSentLog.clear();

    // 500 rpm, direction not inverted
    nodes[0].device->driver->fastReadRoutine(&nodes[0]);
    EXPECT_EQ(listCount(canOpenState.pendingSdoRequests), 1);
    this->canMsgReadQueue.push_back(
        {.canId = 0x581,
         .length = 8,
         .mdata = {0x43, 0x6C, 0x60, 0x00, 0xF4, 0x01, 0x00, 0x00}});
    canOpenTx();
    canOpenRx();
    EXPECT_EQ(nodes[0].device->motorRpm->variant.value.UN16, 500);
    EXPECT_EQ(nodes[0].device->motorDirection->variant.value.UN8, 2);

    // -500 rpm, direction not inverted
    nodes[0].device->driver->fastReadRoutine(&nodes[0]);
    EXPECT_EQ(listCount(canOpenState.pendingSdoRequests), 1);
    this->canMsgReadQueue.push_back(
        {.canId = 0x581,
         .length = 8,
         .mdata = {0x43, 0x6C, 0x60, 0x00, 0x0C, 0xFE, 0xFF, 0xFF}});
    canOpenTx();
    canOpenRx();
    EXPECT_EQ(nodes[0].device->motorRpm->variant.value.UN16, 500);
    EXPECT_EQ(nodes[0].device->motorDirection->variant.value.UN8, 1);

    // 0 rpm, direction not inverted
    nodes[0].device->driver->fastReadRoutine(&nodes[0]);
    EXPECT_EQ(listCount(canOpenState.pendingSdoRequests), 1);
    this->canMsgReadQueue.push_back(
        {.canId = 0x581,
         .length = 8,
         .mdata = {0x43, 0x6C, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00}});
    canOpenTx();
    canOpenRx();
    EXPECT_EQ(nodes[0].device->motorRpm->variant.value.UN16, 0);
    EXPECT_EQ(nodes[0].device->motorDirection->variant.value.UN8, 0);

    // 500 rpm, direction inverted
    nodes[0].device->driver->fastReadRoutine(&nodes[0]);
    EXPECT_EQ(listCount(canOpenState.pendingSdoRequests), 1);
    veItemOwnerSet(nodes[0].device->motorDirectionInverted,
                   veVariantSn32(&v, veTrue));
    this->canMsgReadQueue.push_back(
        {.canId = 0x581,
         .length = 8,
         .mdata = {0x43, 0x6C, 0x60, 0x00, 0xF4, 0x01, 0x00, 0x00}});
    canOpenTx();
    canOpenRx();
    EXPECT_EQ(nodes[0].device->motorRpm->variant.value.UN16, 500);
    EXPECT_EQ(nodes[0].device->motorDirection->variant.value.UN8, 1);

    // -500 rpm, direction inverted
    nodes[0].device->driver->fastReadRoutine(&nodes[0]);
    EXPECT_EQ(listCount(canOpenState.pendingSdoRequests), 1);
    veItemOwnerSet(nodes[0].device->motorDirectionInverted,
                   veVariantSn32(&v, veTrue));
    this->canMsgReadQueue.push_back(
        {.canId = 0x581,
         .length = 8,
         .mdata = {0x43, 0x6C, 0x60, 0x00, 0x0C, 0xFE, 0xFF, 0xFF}});
    canOpenTx();
    canOpenRx();
    EXPECT_EQ(nodes[0].device->motorRpm->variant.value.UN16, 500);
    EXPECT_EQ(nodes[0].device->motorDirection->variant.value.UN8, 2);
}

// EMCY payload: bytes 0-1 emergency error code, byte 2 error register,
// byte 3 DMC fault code, bytes 4-7 DMC fault subcode.
TEST_F(DmcTest, emcyMessage) {
    canOpenRegisterEmcyHandler(nodesEmcyHandler, NULL);
    EXPECT_EQ(injectPlatformNotification_fake.call_count, 0);

    EXPECT_EQ(nodes[0].connected, veFalse);
    connectToDmcNode();
    EXPECT_EQ(nodes[0].connected, veTrue);

    this->canMsgSentLog.clear();

    // Emergency error code 0, the fault was cleared
    this->canMsgReadQueue.push_back(
        {.canId = 0x081,
         .length = 8,
         .mdata = {0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00}});
    canOpenRx();

    pltGetCount1ms_fake.return_val += NOTIFICATION_INJECTION_DELAY_MS;
    processPendingNotifications();

    EXPECT_EQ(injectPlatformNotification_fake.call_count, 0);

    // F5 Motor temperature high, temperature error
    this->canMsgReadQueue.push_back(
        {.canId = 0x081,
         .length = 8,
         .mdata = {0x00, 0x40, 0x08, 0x05, 0x00, 0x00, 0x00, 0x00}});
    canOpenRx();

    pltGetCount1ms_fake.return_val += NOTIFICATION_INJECTION_DELAY_MS;
    processPendingNotifications();

    EXPECT_EQ(injectPlatformNotification_fake.call_count, 1);
    EXPECT_EQ(injectPlatformNotification_fake.arg0_val,
              NOTIFICATION_TYPE_ERROR);
    EXPECT_STREQ(lastTitle, "Motor temperature high (F5 S0)");

    // F17 S2 Battery voltage too low, voltage error
    this->canMsgReadQueue.push_back(
        {.canId = 0x081,
         .length = 8,
         .mdata = {0x00, 0x30, 0x04, 0x11, 0x02, 0x00, 0x00, 0x00}});
    canOpenRx();

    pltGetCount1ms_fake.return_val += NOTIFICATION_INJECTION_DELAY_MS;
    processPendingNotifications();

    EXPECT_EQ(injectPlatformNotification_fake.call_count, 2);
    EXPECT_STREQ(lastTitle, "Battery voltage too low (F17 S2)");

    // A fault code that is not in the error database
    this->canMsgReadQueue.push_back(
        {.canId = 0x081,
         .length = 8,
         .mdata = {0x00, 0x10, 0x01, 0xC8, 0x07, 0x00, 0x00, 0x00}});
    canOpenRx();

    pltGetCount1ms_fake.return_val += NOTIFICATION_INJECTION_DELAY_MS;
    processPendingNotifications();

    EXPECT_EQ(injectPlatformNotification_fake.call_count, 3);
    EXPECT_STREQ(lastTitle, "Unknown fault (F200 S7)");
}
