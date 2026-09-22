#include "CanFixture.hpp"

extern "C" {
#include "canopen.h"
#include "device.h"
#include "node.h"
#include "notification.h"
#include "pair.h"
}

static char lastTitle[1024];
static void injectPlatformNotificationLocal(NotificationType type,
                                            char const *title,
                                            char const *description) {
    strncpy(lastTitle, title, sizeof(lastTitle));
}

class PairTest : public CanFixture {
  protected:
    void SetUp() override {
        CanFixture::SetUp();

        injectPlatformNotification_fake.custom_fake =
            injectPlatformNotificationLocal;
        lastTitle[0] = '\0';

        canOpenInit();
        nodesInit();
        notificationsInit();

        root = veItemGetOrCreateUid(veValueTree(), "test");
        drivePairsInit(root, "Settings/Test");
    }

    void TearDown() override {
        CanFixture::TearDown();

        listDestroy(canOpenState.pendingSdoRequests);
        canOpenState.pendingSdoRequests = NULL;
    }

    VeItem *root;

    void feed(un8 nodeId, std::vector<un8> data) {
        VeRawCanMsg message;

        memset(&message, 0, sizeof(message));
        message.canId = 0x580 + nodeId;
        message.length = (un8)data.size();
        memcpy(message.mdata, data.data(), data.size());
        this->canMsgReadQueue.push_back(message);
        canOpenRx();
    }

    // A Sevcon Gen4, whose name is short enough to arrive expedited, brought
    // up with a chosen DS402 mode of operation. Any driver would do: pairing
    // is decided by 0x6061, not by who made the controller.
    void connectNode(un8 nodeId, un8 modeOfOperation) {
        connectToNode(nodeId);
        canOpenTx();
        feed(nodeId, {0x43, 0x08, 0x10, 0x00, 'G', 'e', 'n', '4'});
        canOpenTx();
        feed(nodeId, {0x42, 0x18, 0x10, 0x04, nodeId, 0x00, 0x00, 0x00});
        canOpenTx();
        feed(nodeId,
             {0x4F, 0x61, 0x60, 0x00, modeOfOperation, 0x00, 0x00, 0x00});
    }

    // A controller that does not implement 0x6061 at all.
    void connectNodeWithoutMode(un8 nodeId) {
        connectToNode(nodeId);
        canOpenTx();
        feed(nodeId, {0x43, 0x08, 0x10, 0x00, 'G', 'e', 'n', '4'});
        canOpenTx();
        feed(nodeId, {0x42, 0x18, 0x10, 0x04, nodeId, 0x00, 0x00, 0x00});
        canOpenTx();
        feed(nodeId, {0x80, 0x61, 0x60, 0x00, 0x00, 0x00, 0x02, 0x06});
    }

    Device *device(un8 nodeId) { return nodes[nodeId - 1].device; }

    void setFloat(VeItem *item, float value) {
        VeVariant v;
        veItemOwnerSet(item, veVariantFloat(&v, value));
    }
    void setSn32(VeItem *item, sn32 value) {
        VeVariant v;
        veItemOwnerSet(item, veVariantSn32(&v, value));
    }
    void setUn16(VeItem *item, un16 value) {
        VeVariant v;
        veItemOwnerSet(item, veVariantUn16(&v, value));
    }
    void setSn16(VeItem *item, sn16 value) {
        VeVariant v;
        veItemOwnerSet(item, veVariantSn16(&v, value));
    }

    float getFloat(VeItem *item) {
        VeVariant v;
        veItemLocalValue(item, &v);
        return v.value.Float;
    }
    sn32 getSn32(VeItem *item) {
        VeVariant v;
        veItemLocalValue(item, &v);
        return v.value.SN32;
    }
    un16 getUn16(VeItem *item) {
        VeVariant v;
        veItemLocalValue(item, &v);
        return v.value.UN16;
    }
    sn16 getSn16(VeItem *item) {
        VeVariant v;
        veItemLocalValue(item, &v);
        return v.value.SN16;
    }
    veBool isValid(VeItem *item) {
        VeVariant v;
        veItemLocalValue(item, &v);
        return veVariantIsValid(&v);
    }

    // Brings a pair up and publishes the primary, which is what creates the
    // per-controller items.
    DrivePair *connectPair() {
        connectNode(1, DS402_MODE_PROFILE_VELOCITY);
        connectNode(2, DS402_MODE_PROFILE_TORQUE);
        exportDevice(device(1));
        return drivePairForNode(1);
    }

    void flushNotifications() {
        pltGetCount1ms_fake.return_val += NOTIFICATION_INJECTION_DELAY_MS;
        processPendingNotifications();
    }
};

TEST_F(PairTest, pairsAVelocityNodeWithATorqueNode) {
    DrivePair *pair;

    connectNode(1, DS402_MODE_PROFILE_VELOCITY);
    EXPECT_EQ(drivePairForNode(1), nullptr);

    connectNode(2, DS402_MODE_PROFILE_TORQUE);

    pair = drivePairForNode(1);
    ASSERT_NE(pair, nullptr);
    EXPECT_EQ(pair->primaryNodeId, 1);
    EXPECT_EQ(pair->secondaryNodeId, 2);
    EXPECT_EQ(drivePairForNode(2), pair);
    EXPECT_EQ(isCombinedSecondary(2), veTrue);
    EXPECT_EQ(isCombinedSecondary(1), veFalse);
}

TEST_F(PairTest, doesNotPairTwoIndependentMotors) {
    // Two propellers each need their own speed loop, so both run profile
    // velocity. Pairing them would merge two real motor drives into one.
    connectNode(1, DS402_MODE_PROFILE_VELOCITY);
    connectNode(2, DS402_MODE_PROFILE_VELOCITY);

    EXPECT_EQ(drivePairForNode(1), nullptr);
    EXPECT_EQ(drivePairForNode(2), nullptr);
}

TEST_F(PairTest, doesNotPairWhenTheModeIsUnknown) {
    connectNodeWithoutMode(1);
    connectNode(2, DS402_MODE_PROFILE_TORQUE);

    EXPECT_EQ(nodes[0].connected, veTrue);
    EXPECT_EQ(drivePairForNode(1), nullptr);
    EXPECT_EQ(drivePairForNode(2), nullptr);
}

TEST_F(PairTest, doesNotPairWhenTheSplitIsAmbiguous) {
    connectNode(1, DS402_MODE_PROFILE_VELOCITY);
    connectNode(2, DS402_MODE_PROFILE_TORQUE);
    connectNode(3, DS402_MODE_PROFILE_TORQUE);

    EXPECT_EQ(drivePairForNode(1), nullptr);
    EXPECT_EQ(drivePairForNode(2), nullptr);
    EXPECT_EQ(drivePairForNode(3), nullptr);
}

TEST_F(PairTest, onlyThePrimaryIsPublished) {
    connectPair();

    EXPECT_EQ(device(1)->exported, veTrue);
    EXPECT_EQ(device(2)->exported, veFalse);

    // One device instance for the pair, not two.
    EXPECT_EQ(veDbusGetVrmDeviceInstanceExt_fake.call_count, 1);
}

TEST_F(PairTest, sumsCurrentPowerAndTorque) {
    DrivePair *pair;

    pair = connectPair();

    // Peak readings from the bench pair.
    setFloat(device(1)->current, 13.3F);
    setFloat(device(2)->current, 14.9F);
    setSn32(device(1)->power, 680);
    setSn32(device(2)->power, 762);
    setUn16(device(1)->motorTorque, 25);
    setUn16(device(2)->motorTorque, 24);

    aggregatePair(pair);

    EXPECT_FLOAT_EQ(getFloat(device(1)->current), 28.2F);
    EXPECT_EQ(getSn32(device(1)->power), 1442);
    EXPECT_EQ(getUn16(device(1)->motorTorque), 49);
}

TEST_F(PairTest, takesTheHigherOfEachTemperature) {
    DrivePair *pair;

    pair = connectPair();

    // The hotter inverter derates first, so an average would mask a failed
    // fan on one side.
    setSn16(device(1)->motorTemperature, 25);
    setSn16(device(2)->motorTemperature, 24);
    setSn16(device(1)->controllerTemperature, 20);
    setSn16(device(2)->controllerTemperature, 21);

    aggregatePair(pair);

    EXPECT_EQ(getSn16(device(1)->motorTemperature), 25);
    EXPECT_EQ(getSn16(device(1)->controllerTemperature), 21);
}

TEST_F(PairTest, keepsEachControllerVisibleSeparately) {
    DrivePair *pair;

    pair = connectPair();

    setFloat(device(1)->current, 13.3F);
    setFloat(device(2)->current, 14.9F);
    setSn16(device(1)->controllerTemperature, 20);
    setSn16(device(2)->controllerTemperature, 21);
    setUn16(device(1)->motorRpm, 1407);
    setUn16(device(2)->motorRpm, 1408);

    aggregatePair(pair);

    EXPECT_FLOAT_EQ(getFloat(device(1)->memberCurrent[0]), 13.3F);
    EXPECT_FLOAT_EQ(getFloat(device(1)->memberCurrent[1]), 14.9F);
    EXPECT_EQ(getSn16(device(1)->memberControllerTemperature[0]), 20);
    EXPECT_EQ(getSn16(device(1)->memberControllerTemperature[1]), 21);
    EXPECT_EQ(getUn16(device(1)->memberRpm[0]), 1407);
    EXPECT_EQ(getUn16(device(1)->memberRpm[1]), 1408);
}

TEST_F(PairTest, staysQuietWhileTheHalvesAgreeOnSpeed) {
    DrivePair *pair;

    pair = connectPair();

    // The widest gap seen on the bench pair, at close to full speed.
    setUn16(device(1)->motorRpm, 1408);
    setUn16(device(2)->motorRpm, 1402);
    aggregatePair(pair);
    flushNotifications();
    EXPECT_EQ(injectPlatformNotification_fake.call_count, 0);

    // Near standstill the floor keeps small absolute differences quiet.
    setUn16(device(1)->motorRpm, 10);
    setUn16(device(2)->motorRpm, 55);
    aggregatePair(pair);
    flushNotifications();
    EXPECT_EQ(injectPlatformNotification_fake.call_count, 0);
}

TEST_F(PairTest, reportsSpeedDivergenceOnceAndRearms) {
    DrivePair *pair;

    pair = connectPair();

    setUn16(device(1)->motorRpm, 1400);
    setUn16(device(2)->motorRpm, 900);
    aggregatePair(pair);
    flushNotifications();
    EXPECT_EQ(injectPlatformNotification_fake.call_count, 1);
    EXPECT_EQ(injectPlatformNotification_fake.arg0_val,
              NOTIFICATION_TYPE_ERROR);

    // Still diverging, so not said again.
    aggregatePair(pair);
    aggregatePair(pair);
    flushNotifications();
    EXPECT_EQ(injectPlatformNotification_fake.call_count, 1);

    // Back in step, then diverging again, which is worth saying.
    setUn16(device(2)->motorRpm, 1399);
    aggregatePair(pair);
    flushNotifications();
    EXPECT_EQ(injectPlatformNotification_fake.call_count, 1);

    setUn16(device(2)->motorRpm, 700);
    aggregatePair(pair);
    flushNotifications();
    EXPECT_EQ(injectPlatformNotification_fake.call_count, 2);
}

TEST_F(PairTest, invalidatesTheDriveWhenHalfOfItDisappears) {
    DrivePair *pair;

    pair = connectPair();

    setFloat(device(1)->current, 13.3F);
    setFloat(device(2)->current, 14.9F);
    setUn16(device(1)->motorRpm, 1400);
    setUn16(device(2)->motorRpm, 1400);
    aggregatePair(pair);
    EXPECT_EQ(isPairDegraded(pair), veFalse);
    EXPECT_FLOAT_EQ(getFloat(device(1)->current), 28.2F);

    disconnectFromNode(2);

    // The pair survives, so the survivor cannot quietly become a standalone
    // drive reporting half the power.
    pair = drivePairForNode(1);
    ASSERT_NE(pair, nullptr);
    EXPECT_EQ(isPairDegraded(pair), veTrue);

    invalidatePair(pair);
    EXPECT_EQ(isValid(device(1)->current), veFalse);
    EXPECT_EQ(isValid(device(1)->power), veFalse);
    EXPECT_EQ(isValid(device(1)->motorRpm), veFalse);
    EXPECT_EQ(isValid(device(1)->motorTorque), veFalse);

    flushNotifications();
    EXPECT_EQ(injectPlatformNotification_fake.call_count, 1);

    // Said once, not on every read cycle.
    invalidatePair(pair);
    invalidatePair(pair);
    flushNotifications();
    EXPECT_EQ(injectPlatformNotification_fake.call_count, 1);
}

TEST_F(PairTest, forgetsThePairOnceNeitherHalfIsThere) {
    connectPair();

    disconnectFromNode(2);
    EXPECT_NE(drivePairForNode(1), nullptr);

    disconnectFromNode(1);
    EXPECT_EQ(drivePairForNode(1), nullptr);
    EXPECT_EQ(drivePairForNode(2), nullptr);
}
