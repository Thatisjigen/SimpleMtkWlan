// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (c) 2026 thanhnndev and contributors
// The AWDL virtual interface, adapted from thanhnndev/mt7925-macos-wifi (see MtkAwdl.h). The interface exists, is registered
// and answers the AWDL peer manager's configure handshake; there is no radio work yet: no channel slicing, no action frames,
// no data path.
//
// Creation order: init -> attach -> deferBSDAttach(true) -> start(ctl) -> register the ifnet -> registerService +
// setRunningState(false) -> setAwdlInterface -> deferBSDAttach(false). The start(ctl) call is mandatory: it is what stores the
// controller at ivars+0x30 (getController), which IO80211VirtualInterface::attachToBpf dereferences from the BSD client's
// start; skipping it panics on the first BSD attach.
#include "MtkAwdl.h"
#include "skywalk_min.h"
#include "io80211/IO80211AWDLProtocol.h"
#include <IOKit/IOLib.h>
#include <pexpert/pexpert.h>
#include <string.h>

extern "C" void mtk_dbg_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

namespace {

template <typename T> T &field(void *base, size_t off) { return *(T *)((uint8_t *)base + off); }

// The AWDL parameter structs are not public. Their sizes are the request lengths IO80211Family's ioctl handlers accept on
// macOS 26 (getAWDL_SYNC_STATE wants 0x18 bytes, getAWDL_MASTER_CHANNEL 8, ...): the Family's own buffers have these sizes, so
// nothing below reads or writes past them.
const size_t kAwdlSyncEnabledLen = 0xc;       // u32 version; u32; u32 enabled
const size_t kAwdlSyncParamsLen = 0x18;
const size_t kAwdlBssidLen = 0x10;            // u32 version; u8 bssid[6]; ...
const size_t kAwdlElectionMetricLen = 0xc;    // u32 version; u32 metric; ...
const size_t kAwdlChanSeqLen = 0x190;
const size_t kAwdlMasterChannelLen = 0x8;     // u32 version; u32 channel
const size_t kAwdlSyncStateLen = 0x18;
const size_t kAwdlElectionParamsLen = 0x38;
const size_t kAwdlPeerTrafficRegLen = 0xb8;

} // namespace

// -----------------------------------------------------------------------------------------------------------------------
// The virtual interface: IO80211VirtualInterface with the AWDL BSD name prefix.
//
// Apple's Broadcom path instantiates AppleBCMWLANProximityInterface (IO80211AWDLProtocol : IO80211VirtualInterface) whose
// getBSDNamePrefix returns "awdl" and whose AWDL protocol slots talk to firmware. The Family's generic
// IO80211VirtualInterface would take the default prefix ("en", IOSkywalkEthernetInterface::getBSDNamePrefix) and could never
// show up as awdl0, so this subclass overrides the prefix and leaves everything else to the Family (init, attach, the ifnet
// registration and - later - createPeerManager -> IO80211AWDLPeerManager -> IO80211P2PSupervisor).
// -----------------------------------------------------------------------------------------------------------------------
// Base is the Family's AWDL protocol class (not plain IO80211VirtualInterface): the Family's routes to the driver
// (apple80211setCOMPANION_SKYWALK_IF_CREATE, apple80211getAWDL_SYNCHRONIZATION_CHANNEL_SEQUENCE, ...) safeMetaCast the
// interface against IO80211AWDLProtocol::gMetaClass and then call its protocol slot (562 for the companion), and the AWDL
// peer manager drives the protocol virtuals. Without the subclass link every AWDL ioctl answers 0xe082280e before reaching
// this driver.
class MtkAwdlInterface : public IO80211AWDLProtocol {
    OSDeclareDefaultStructors(MtkAwdlInterface)

public:
    // IOSkywalkNetworkInterface::getBSDNamePrefix (slot 319): the BSD name the Family builds is <prefix><unit>.
    const char *getBSDNamePrefix() override;
    // Slot 490 is ___cxa_pure_virtual in Apple's binary and in IO80211AWDLProtocol (no override anywhere in the Family - the
    // Broadcom subclass implements it). Nothing on this path calls it; implemented so the subclass is concrete.
    void _unnamedPure490() override;
    // ---------------------------------------------------------------------------------------------------------------
    // IO80211AWDLProtocol (slots 507..569): the Family's AWDL ioctls and the AWDL peer manager call these.
    //
    // IO80211AWDLPeerManager::configure hands the driver its parameters one slot after the other and gives up on the first
    // unsupported answer ("configure Unable to set BSSID ret e00002c7", then "cannot configure AWDL after enabling
    // interface" and AWDL is disabled again); it also reads some of them back right after setting them. So the parameter
    // setters accept (awdlAccept), the values the Family reads back are kept and answered verbatim, and only what needs the
    // radio or a second interface answers unsupported (awdlProto): the out-of-band frames (548, 557, 563), the low-latency
    // companion (562, 564) and the getters nobody reads back. Each slot logs its first call, so the driver log shows which
    // ones the Family drives and in what order.
    IOReturn awdlProto(const char *name);
    IOReturn awdlAccept(const char *name);
    /* 507 */ IOReturn getSSID(apple80211_ssid_data*a1) override { return awdlProto("getSSID"); }
    /* 508 */ IOReturn getCHANNEL(apple80211_channel_data*a1) override { return awdlProto("getCHANNEL"); }
    /* 509 */ IOReturn getBSSID(apple80211_bssid_data*a1) override { return awdlProto("getBSSID"); }
    /* 510 */ IOReturn getSTATE(apple80211_state_data*a1) override { return awdlProto("getSTATE"); }
    /* 511 */ IOReturn getAWDL_SYNC_PARAMS(apple80211_awdl_sync_params*a1) override { return awdlProto("getAWDL_SYNC_PARAMS"); }
    /* 512 */ IOReturn getAWDL_SYNC_ENABLED(apple80211_awdl_sync_enabled*a1) override;
    /* 513 */ IOReturn getAWDL_EXTENSION_STATE_MACHINE_PARAMETERS(apple80211_awdl_extension_state_machine_parameter*a1) override { return awdlProto("getAWDL_EXTENSION_STATE_MACHINE_PARAMETERS"); }
    /* 514 */ IOReturn getAWDL_ELECTION_ID(apple80211_awdl_election_id*a1) override { return awdlProto("getAWDL_ELECTION_ID"); }
    /* 515 */ IOReturn getAWDL_ELECTION_METRIC(apple80211_awdl_election_metric*a1) override { return awdlProto("getAWDL_ELECTION_METRIC"); }
    /* 516 */ IOReturn getAWDL_SYNCHRONIZATION_CHANNEL_SEQUENCE(apple80211_awdl_sync_channel_sequence*a1) override;
    /* 517 */ IOReturn getPEER_CACHE_MAXIMUM_SIZE(apple80211_peer_cache_maximum_size*a1) override { return awdlProto("getPEER_CACHE_MAXIMUM_SIZE"); }
    /* 518 */ IOReturn getAWDL_MASTER_CHANNEL(apple80211_awdl_master_channel*a1) override;
    /* 519 */ IOReturn getAWDL_SYNC_STATE(apple80211_awdl_sync_state*a1) override;
    /* 520 */ IOReturn getAWDL_ELECTION_RSSI_THRESHOLDS(apple80211_awdl_election_rssi_thresholds*a1) override { return awdlProto("getAWDL_ELECTION_RSSI_THRESHOLDS"); }
    /* 521 */ IOReturn getAWDL_PRESENCE_MODE(apple80211_awdl_presence_mode*a1) override { return awdlProto("getAWDL_PRESENCE_MODE"); }
    /* 522 */ IOReturn getAWDL_ELECTION_PARAMS(apple80211_awdl_election_params*a1) override;
    /* 523 */ IOReturn getAWDL_DEVICE_CAPABILITIES(apple80211_awdl_device_capabilities*a1) override { return awdlProto("getAWDL_DEVICE_CAPABILITIES"); }
    /* 524 */ IOReturn getAWDL_SECONDARY_MASTER_CHANNEL(apple80211_awdl_secondary_master_channel*a1) override { return awdlProto("getAWDL_SECONDARY_MASTER_CHANNEL"); }
    /* 525 */ IOReturn getAWDL_AF_TX_MODE(apple80211_awdl_af_tx_mode*a1) override { return awdlProto("getAWDL_AF_TX_MODE"); }
    /* 526 */ IOReturn getAWDL_MIN_RATE(apple80211_awdl_min_rate*a1) override { return awdlProto("getAWDL_MIN_RATE"); }
    /* 527 */ IOReturn getAWDL_ENCRYPTION_TYPE(apple80211_encrypt*a1) override { return awdlProto("getAWDL_ENCRYPTION_TYPE"); }
    /* 528 */ IOReturn getAWDL_TXCAL_PERIOD(apple80211_awdl_txcal_period*a1) override { return awdlProto("getAWDL_TXCAL_PERIOD"); }
    /* 529 */ IOReturn getAWDL_DEV_STATS(apple80211_awdl_dev_stats*a1) override { return awdlProto("getAWDL_DEV_STATS"); }
    /* 530 */ IOReturn getAWDL_DFSP_CONFIG(apple80211_awdl_dfsp_config*a1) override { return awdlProto("getAWDL_DFSP_CONFIG"); }
    /* 531 */ IOReturn getAWDL_CCA(apple80211_awdl_cca*a1) override { return awdlProto("getAWDL_CCA"); }
    /* 532 */ IOReturn getAWDL_AF_SECONDARY_PAYLOAD_CAPABILITIES(apple80211_awdl_cap*a1) override { return awdlProto("getAWDL_AF_SECONDARY_PAYLOAD_CAPABILITIES"); }
    /* 533 */ IOReturn getLLW_PARAMS(apple80211_llw_params*a1) override { return awdlProto("getLLW_PARAMS"); }
    /* 534 */ IOReturn setSSID(apple80211_ssid_data*a1) override { return awdlAccept("setSSID"); }
    /* 535 */ IOReturn setCHANNEL(apple80211_channel_data*a1) override { return awdlAccept("setCHANNEL"); }
    /* 536 */ IOReturn setAWDL_SYNC_PARAMS(apple80211_awdl_sync_params*a1) override;
    /* 537 */ IOReturn setAWDL_SYNC_ENABLED(apple80211_awdl_sync_enabled*a1) override;
    /* 538 */ IOReturn setAWDL_EXTENSION_STATE_MACHINE_PARAMETERS(apple80211_awdl_extension_state_machine_parameter*a1) override { return awdlAccept("setAWDL_EXTENSION_STATE_MACHINE_PARAMETERS"); }
    /* 539 */ IOReturn setAWDL_ELECTION_ID(apple80211_awdl_election_id*a1) override { return awdlAccept("setAWDL_ELECTION_ID"); }
    /* 540 */ IOReturn setAWDL_BSSID(apple80211_awdl_bssid*a1) override;
    /* 541 */ IOReturn setAWDL_ELECTION_METRIC(apple80211_awdl_election_metric*a1) override;
    /* 542 */ IOReturn setAWDL_SYNCHRONIZATION_CHANNEL_SEQUENCE(apple80211_awdl_sync_channel_sequence*a1) override;
    /* 543 */ IOReturn setAWDL_MASTER_CHANNEL(apple80211_awdl_master_channel*a1) override;
    /* 544 */ IOReturn setAWDL_SYNC_STATE(apple80211_awdl_sync_state*a1) override;
    /* 545 */ IOReturn setAWDL_ELECTION_RSSI_THRESHOLDS(apple80211_awdl_election_rssi_thresholds*a1) override { return awdlAccept("setAWDL_ELECTION_RSSI_THRESHOLDS"); }
    /* 546 */ IOReturn setAWDL_PRESENCE_MODE(apple80211_awdl_presence_mode*a1) override { return awdlAccept("setAWDL_PRESENCE_MODE"); }
    /* 547 */ IOReturn setPEER_CACHE_CONTROL(apple80211_peer_cache_control*a1) override { return awdlAccept("setPEER_CACHE_CONTROL"); }
    /* 548 */ IOReturn setAWDL_OOB_REQUEST(apple80211_awdl_oob_request*a1) override { return awdlProto("setAWDL_OOB_REQUEST"); }
    /* 549 */ IOReturn setAWDL_SYNC_FRAME_TEMPLATE(apple80211_awdl_sync_frame_template*a1) override;
    /* 550 */ IOReturn setAWDL_ELECTION_PARAMS(apple80211_awdl_election_params*a1) override;
    /* 551 */ IOReturn setAWDL_PEER_TRAFFIC_REGISTRATION(apple80211_awdl_peer_traffic_registration*a1) override;
    /* 552 */ IOReturn setAWDL_SECONDARY_MASTER_CHANNEL(apple80211_awdl_secondary_master_channel*a1) override { return awdlAccept("setAWDL_SECONDARY_MASTER_CHANNEL"); }
    /* 553 */ IOReturn setAWDL_AF_TX_MODE(apple80211_awdl_af_tx_mode*a1) override { return awdlAccept("setAWDL_AF_TX_MODE"); }
    /* 554 */ IOReturn setAWDL_MIN_RATE(apple80211_awdl_min_rate*a1) override { return awdlAccept("setAWDL_MIN_RATE"); }
    /* 555 */ IOReturn setAWDL_ENCRYPTION_KEYS(apple80211_key*a1) override { return awdlAccept("setAWDL_ENCRYPTION_KEYS"); }
    /* 556 */ IOReturn setAWDL_ENCRYPTION_TYPE(apple80211_encrypt*a1) override { return awdlAccept("setAWDL_ENCRYPTION_TYPE"); }
    /* 557 */ IOReturn setAWDL_OOB_AUTO_REQUEST(apple80211_awdl_oob_request*a1) override { return awdlProto("setAWDL_OOB_AUTO_REQUEST"); }
    /* 558 */ IOReturn setAWDL_TXCAL_PERIOD(apple80211_awdl_txcal_period*a1) override { return awdlAccept("setAWDL_TXCAL_PERIOD"); }
    /* 559 */ IOReturn setAWDL_AWDL_ADVERTISERS(awdl_peer_advert_add*a1) override { return awdlAccept("setAWDL_AWDL_ADVERTISERS"); }
    /* 560 */ IOReturn setAWDL_DFSP_CONFIG(apple80211_awdl_dfsp_config*a1) override { return awdlAccept("setAWDL_DFSP_CONFIG"); }
    /* 561 */ IOReturn setAWDL_DFSP_UCSA_CONFIG(apple80211_awdl_dfsp_ucsa_config*a1) override { return awdlAccept("setAWDL_DFSP_UCSA_CONFIG"); }
    /* 562 */ IOReturn setCOMPANION_SKYWALK_IF_CREATE(apple80211_companion_skywalk_if_create*a1) override;
    /* 563 */ IOReturn setAWDL_OOB_SEC_PAYLOAD_REQUEST(apple80211_awdl_af_sec_payload_request*a1) override { return awdlProto("setAWDL_OOB_SEC_PAYLOAD_REQUEST"); }
    /* 564 */ IOReturn setLLW_PARAMS(apple80211_llw_params*a1) override { return awdlProto("setLLW_PARAMS"); }
    /* 565 */ IOReturn setTX_RATE(apple80211_tx_rate*a1) override { return awdlAccept("setTX_RATE"); }
    /* 566 */ IOReturn setP2P_COEX_PROTECTION(apple80211_p2p_coex_protection*a1) override { return awdlAccept("setP2P_COEX_PROTECTION"); }
    /* 567 */ IOReturn setPSF_DWELL(apple80211_awdl_psf_dwell_params*a1) override { return awdlAccept("setPSF_DWELL"); }
    /* 568 */ IOReturn setAWDL_CONFIG(apple80211_awdl_config_request*a1) override { return awdlAccept("setAWDL_CONFIG"); }
    /* 569 */ IOReturn setAWDL_RTG_PEER_STATS_CONFIG(apple80211_rtg_peer_stats_cfg*a1) override { return awdlAccept("setAWDL_RTG_PEER_STATS_CONFIG"); }

    // Creates the pool and the (three) queues and registers the ifnet from a RegistrationInfo filled like
    // AppleBCMWLANCore::createProxInterface fills it. Keeps the objects for the interface's lifetime.
    IOReturn registerAwdlInterface(const uint8_t *mac);
    void freeAwdlRegistration();

private:
    bool awdlFirstCall(const char *name);      // true once per slot name: the first call of each slot is logged
    bool awdlLogValue();                       // budget for the lines that log a changed parameter
    void awdlAnswered(const char *name, bool set);   // logs the first read-back of each kept parameter
    const char *awdlCalled[64] = {};           // slot names already logged (one entry per slot: 63 slots)
    uint32_t awdlCalledCount = 0;
    uint32_t awdlValueLogs = 0;
    uint8_t awdlAddr[6] = {};                  // awdl0's address: our own top master until the Family names another
    // Kept as the Family set them, answered back by the getters (or used to tell a change from a repeat in the log).
    uint32_t awdlSyncEnabled = 0;
    uint32_t awdlElectionMetric = 0;
    uint32_t awdlTemplateLen = 0xffffffffu;
    uint8_t awdlBssid[6] = {};
    bool awdlBssidSet = false;
    uint8_t awdlSyncParams[kAwdlSyncParamsLen] = {};
    bool awdlSyncParamsSet = false;
    uint8_t awdlChanSeq[kAwdlChanSeqLen] = {};
    bool awdlChanSeqSet = false;
    uint8_t awdlMasterChannel[kAwdlMasterChannelLen] = {};
    bool awdlMasterChannelSet = false;
    uint8_t awdlSyncState[kAwdlSyncStateLen] = {};
    bool awdlSyncStateSet = false;
    uint8_t awdlElectionParams[kAwdlElectionParamsLen] = {};
    bool awdlElectionParamsSet = false;
    IOSkywalkPacketBufferPool *awdlPool = nullptr;
    IOSkywalkPacketQueue *awdlQueues[3] = {};
};

#undef super
#define super IO80211AWDLProtocol
OSDefineMetaClassAndStructors(MtkAwdlInterface, IO80211AWDLProtocol)

const char *MtkAwdlInterface::getBSDNamePrefix() { return "awdl"; }
void MtkAwdlInterface::_unnamedPure490() {}

// The Family polls some slots every second (the out-of-band requests, for one): only the first call of each is logged.
bool MtkAwdlInterface::awdlFirstCall(const char *name)
{
    for (uint32_t i = 0; i < awdlCalledCount; i++)
        if (awdlCalled[i] == name) return false;
    if (awdlCalledCount == sizeof(awdlCalled) / sizeof(awdlCalled[0])) return false;
    awdlCalled[awdlCalledCount++] = name;
    return true;
}

bool MtkAwdlInterface::awdlLogValue()
{
    if (awdlValueLogs >= 96) return false;
    awdlValueLogs++;
    return true;
}

IOReturn MtkAwdlInterface::awdlProto(const char *name)
{
    if (awdlFirstCall(name)) mtk_dbg_log("awdl: %s unsupported\n", name);
    return kIOReturnUnsupported;
}

// A parameter with no radio use yet: taken, so that configure goes on to the next one. Never logs the payload (keys among them).
IOReturn MtkAwdlInterface::awdlAccept(const char *name)
{
    if (awdlFirstCall(name)) mtk_dbg_log("awdl: %s accepted\n", name);
    return kIOReturnSuccess;
}

// The read-backs are where configure stopped before: their first call shows it got past them.
void MtkAwdlInterface::awdlAnswered(const char *name, bool set)
{
    if (awdlFirstCall(name)) mtk_dbg_log("awdl: %s answered %s\n", name, set ? "with what was set" : "with the default");
}

// apple80211_awdl_sync_enabled: +8 enabled. AWDL on or off for the whole interface.
IOReturn MtkAwdlInterface::setAWDL_SYNC_ENABLED(apple80211_awdl_sync_enabled *a1)
{
    if (!a1) return kIOReturnBadArgument;
    uint32_t enabled = field<uint32_t>(a1, 8);
    bool first = awdlFirstCall("setAWDL_SYNC_ENABLED");
    if ((first || enabled != awdlSyncEnabled) && awdlLogValue()) mtk_dbg_log("awdl: sync enabled %u\n", enabled);
    awdlSyncEnabled = enabled;
    return kIOReturnSuccess;
}

IOReturn MtkAwdlInterface::getAWDL_SYNC_ENABLED(apple80211_awdl_sync_enabled *a1)
{
    if (!a1) return kIOReturnBadArgument;
    awdlAnswered("getAWDL_SYNC_ENABLED", true);
    bzero(a1, kAwdlSyncEnabledLen);
    field<uint32_t>(a1, 0) = 1;
    field<uint32_t>(a1, 8) = awdlSyncEnabled;
    return kIOReturnSuccess;
}

// apple80211_awdl_sync_params: the availability window and sync frame timing. Only logged: nothing runs the timeline yet.
IOReturn MtkAwdlInterface::setAWDL_SYNC_PARAMS(apple80211_awdl_sync_params *a1)
{
    if (!a1) return kIOReturnBadArgument;
    bool changed = !awdlSyncParamsSet || memcmp(awdlSyncParams, a1, kAwdlSyncParamsLen) != 0;
    memcpy(awdlSyncParams, a1, kAwdlSyncParamsLen);
    awdlSyncParamsSet = true;
    if (changed && awdlLogValue()) {
        const uint32_t *w = (const uint32_t *)awdlSyncParams;
        mtk_dbg_log("awdl: sync params %x %u %u %u %u %u\n", w[0], w[1], w[2], w[3], w[4], w[5]);
    }
    return kIOReturnSuccess;
}

// apple80211_awdl_bssid: +4 the BSSID of the AWDL network. The first parameter configure hands over.
IOReturn MtkAwdlInterface::setAWDL_BSSID(apple80211_awdl_bssid *a1)
{
    static_assert(4 + 6 <= kAwdlBssidLen, "read inside the struct");
    if (!a1) return kIOReturnBadArgument;
    const uint8_t *b = (const uint8_t *)a1 + 4;
    bool changed = !awdlBssidSet || memcmp(awdlBssid, b, 6) != 0;
    memcpy(awdlBssid, b, 6);
    awdlBssidSet = true;
    if (changed && awdlLogValue())
        mtk_dbg_log("awdl: bssid %02x:%02x:%02x:%02x:%02x:%02x\n", b[0], b[1], b[2], b[3], b[4], b[5]);
    return kIOReturnSuccess;
}

// apple80211_awdl_election_metric: +4 the metric this node runs the master election with.
IOReturn MtkAwdlInterface::setAWDL_ELECTION_METRIC(apple80211_awdl_election_metric *a1)
{
    static_assert(4 + 4 <= kAwdlElectionMetricLen, "read inside the struct");
    if (!a1) return kIOReturnBadArgument;
    uint32_t metric = field<uint32_t>(a1, 4);
    bool first = awdlFirstCall("setAWDL_ELECTION_METRIC");
    if ((first || metric != awdlElectionMetric) && awdlLogValue()) mtk_dbg_log("awdl: election metric %u\n", metric);
    awdlElectionMetric = metric;
    return kIOReturnSuccess;
}

// apple80211_awdl_sync_channel_sequence, laid out as AppleBCMWLANProximityInterface's getter fills it: +5 the number of
// entries, +9 the fill channel, entries from +0xc, each an apple80211_channel {version, channel, flags} of 12 bytes. The
// Family reads it back right after setting it; an unsupported answer there ended configure before the sync frame template.
IOReturn MtkAwdlInterface::setAWDL_SYNCHRONIZATION_CHANNEL_SEQUENCE(apple80211_awdl_sync_channel_sequence *a1)
{
    if (!a1) return kIOReturnBadArgument;
    bool changed = !awdlChanSeqSet || memcmp(awdlChanSeq, a1, kAwdlChanSeqLen) != 0;
    memcpy(awdlChanSeq, a1, kAwdlChanSeqLen);
    awdlChanSeqSet = true;
    if (changed && awdlLogValue()) {
        uint32_t n = awdlChanSeq[5] < 16 ? awdlChanSeq[5] : 16;
        char list[16 * 4 + 1];
        size_t at = 0;
        for (uint32_t i = 0; i < n; i++) {
            uint32_t ch = awdlChanSeq[0xc + 12 * i + 4];
            list[at++] = ' ';
            if (ch >= 100) list[at++] = (char)('0' + ch / 100);
            if (ch >= 10) list[at++] = (char)('0' + ch / 10 % 10);
            list[at++] = (char)('0' + ch % 10);
        }
        list[at] = 0;
        mtk_dbg_log("awdl: channel sequence n=%u fill=%u:%s\n", awdlChanSeq[5], awdlChanSeq[9], list);
    }
    return kIOReturnSuccess;
}

IOReturn MtkAwdlInterface::getAWDL_SYNCHRONIZATION_CHANNEL_SEQUENCE(apple80211_awdl_sync_channel_sequence *a1)
{
    if (!a1) return kIOReturnBadArgument;
    awdlAnswered("getAWDL_SYNCHRONIZATION_CHANNEL_SEQUENCE", awdlChanSeqSet);
    if (awdlChanSeqSet) {
        memcpy(a1, awdlChanSeq, kAwdlChanSeqLen);
        return kIOReturnSuccess;
    }
    // Nothing set yet: one entry, 2.4 GHz channel 6 at 20 MHz, no fill channel.
    uint8_t *b = (uint8_t *)a1;
    bzero(b, kAwdlChanSeqLen);
    b[0] = 1;
    b[5] = 1;
    b[9] = 0xff;
    field<uint32_t>(b, 0xc) = 1;
    field<uint32_t>(b, 0x10) = 6;
    field<uint32_t>(b, 0x14) = 0x0a;
    return kIOReturnSuccess;
}

// apple80211_awdl_master_channel: +4 the channel.
IOReturn MtkAwdlInterface::setAWDL_MASTER_CHANNEL(apple80211_awdl_master_channel *a1)
{
    if (!a1) return kIOReturnBadArgument;
    bool changed = !awdlMasterChannelSet || memcmp(awdlMasterChannel, a1, kAwdlMasterChannelLen) != 0;
    memcpy(awdlMasterChannel, a1, kAwdlMasterChannelLen);
    awdlMasterChannelSet = true;
    if (changed && awdlLogValue()) mtk_dbg_log("awdl: master channel %u\n", field<uint32_t>(awdlMasterChannel, 4));
    return kIOReturnSuccess;
}

IOReturn MtkAwdlInterface::getAWDL_MASTER_CHANNEL(apple80211_awdl_master_channel *a1)
{
    if (!a1) return kIOReturnBadArgument;
    awdlAnswered("getAWDL_MASTER_CHANNEL", awdlMasterChannelSet);
    if (awdlMasterChannelSet) {
        memcpy(a1, awdlMasterChannel, kAwdlMasterChannelLen);
    } else {
        field<uint32_t>(a1, 0) = 1;
        field<uint32_t>(a1, 4) = 6;
    }
    return kIOReturnSuccess;
}

// apple80211_awdl_sync_state: +4 the role (1 master, 2 slave, 3 non-election master), +8 the master's address. The result
// of the Family's election, handed down after every change.
IOReturn MtkAwdlInterface::setAWDL_SYNC_STATE(apple80211_awdl_sync_state *a1)
{
    if (!a1) return kIOReturnBadArgument;
    bool changed = !awdlSyncStateSet || memcmp(awdlSyncState, a1, kAwdlSyncStateLen) != 0;
    memcpy(awdlSyncState, a1, kAwdlSyncStateLen);
    awdlSyncStateSet = true;
    if (changed && awdlLogValue()) {
        const uint8_t *m = awdlSyncState + 8;
        mtk_dbg_log("awdl: sync state %u master %02x:%02x:%02x:%02x:%02x:%02x\n", field<uint32_t>(awdlSyncState, 4),
                    m[0], m[1], m[2], m[3], m[4], m[5]);
    }
    return kIOReturnSuccess;
}

IOReturn MtkAwdlInterface::getAWDL_SYNC_STATE(apple80211_awdl_sync_state *a1)
{
    if (!a1) return kIOReturnBadArgument;
    awdlAnswered("getAWDL_SYNC_STATE", awdlSyncStateSet);
    if (awdlSyncStateSet) memcpy(a1, awdlSyncState, kAwdlSyncStateLen);
    else bzero(a1, kAwdlSyncStateLen);
    return kIOReturnSuccess;
}

// apple80211_awdl_election_params, the election tree as AppleBCMWLANProximityInterface's getter reads it from its firmware:
// +0x16 the top master, +0x1c its metric, +0x20 our own. The election itself runs in the Family, so the truthful answer is
// what it last set; before that, this node is its own top master.
IOReturn MtkAwdlInterface::setAWDL_ELECTION_PARAMS(apple80211_awdl_election_params *a1)
{
    if (!a1) return kIOReturnBadArgument;
    bool changed = !awdlElectionParamsSet || memcmp(awdlElectionParams, a1, kAwdlElectionParamsLen) != 0;
    memcpy(awdlElectionParams, a1, kAwdlElectionParamsLen);
    awdlElectionParamsSet = true;
    if (changed && awdlLogValue()) {
        const uint8_t *t = awdlElectionParams + 0x16;
        mtk_dbg_log("awdl: election top %02x:%02x:%02x:%02x:%02x:%02x metric %u self %u\n", t[0], t[1], t[2], t[3], t[4], t[5],
                    field<uint32_t>(awdlElectionParams, 0x1c), field<uint32_t>(awdlElectionParams, 0x20));
    }
    return kIOReturnSuccess;
}

IOReturn MtkAwdlInterface::getAWDL_ELECTION_PARAMS(apple80211_awdl_election_params *a1)
{
    if (!a1) return kIOReturnBadArgument;
    awdlAnswered("getAWDL_ELECTION_PARAMS", awdlElectionParamsSet);
    if (awdlElectionParamsSet) {
        memcpy(a1, awdlElectionParams, kAwdlElectionParamsLen);
        return kIOReturnSuccess;
    }
    uint8_t *b = (uint8_t *)a1;
    bzero(b, kAwdlElectionParamsLen);
    field<uint32_t>(b, 0) = 1;
    memcpy(b + 0x16, awdlAddr, 6);
    field<uint32_t>(b, 0x1c) = awdlElectionMetric;
    field<uint32_t>(b, 0x20) = awdlElectionMetric;
    return kIOReturnSuccess;
}

// apple80211_awdl_sync_frame_template: +4 the payload length, +8 a pointer to the TLVs of the sync frame the driver is to
// send every sync period. Nothing is sent yet, so only the length is looked at.
IOReturn MtkAwdlInterface::setAWDL_SYNC_FRAME_TEMPLATE(apple80211_awdl_sync_frame_template *a1)
{
    if (!a1) return kIOReturnBadArgument;
    uint32_t len = field<uint32_t>(a1, 4);
    if (len != awdlTemplateLen && awdlLogValue()) mtk_dbg_log("awdl: sync frame template %u bytes\n", len);
    awdlTemplateLen = len;
    return kIOReturnSuccess;
}

// apple80211_awdl_peer_traffic_registration: +0xc the length of the service name at +0x10, +0xa8 active (the offsets
// AppleBCMWLANProximityInterface reads). The Family logs "Can't send peer traffic registration to the driver" when this fails.
IOReturn MtkAwdlInterface::setAWDL_PEER_TRAFFIC_REGISTRATION(apple80211_awdl_peer_traffic_registration *a1)
{
    static_assert(0x10 + 32 <= kAwdlPeerTrafficRegLen && 0xa8 + 4 <= kAwdlPeerTrafficRegLen, "read inside the struct");
    if (!a1) return kIOReturnBadArgument;
    if (awdlLogValue()) {
        const uint8_t *p = (const uint8_t *)a1;
        uint32_t nameLen = field<uint32_t>(a1, 0xc);
        char name[33];
        size_t n = nameLen < sizeof(name) - 1 ? nameLen : sizeof(name) - 1;
        for (size_t i = 0; i < n; i++) name[i] = (p[0x10 + i] >= 0x20 && p[0x10 + i] <= 0x7e) ? (char)p[0x10 + i] : '.';
        name[n] = 0;
        mtk_dbg_log("awdl: peer traffic %s active %u\n", name, field<uint32_t>(a1, 0xa8));
    }
    return kIOReturnSuccess;
}

// Slot 562. AppleBCMWLANProximityInterface::setCOMPANION_SKYWALK_IF_CREATE reads action at +4
// (0 = destroy the companion, 1 = create one), builds apple80211SkywalkIfCreateParams_t {class, MAC, 9, 2}, calls
// AppleBCMWLANCore::createSkywalkInterface(&vif->companion, params, vif), then setCompanionInterfaceId(parent) and copies
// the companion's BSD name into the request at +8 (apple80211_companion_skywalk_if_create: version, action, bsd_name[16]).
// That low-latency (LLW) Skywalk interface does not exist here, so the answer is unsupported: wifip2pd logs a missing
// optional companion and goes on with the normal power-state and eligibility bring-up; AWDL itself does not use it.
IOReturn MtkAwdlInterface::setCOMPANION_SKYWALK_IF_CREATE(apple80211_companion_skywalk_if_create *a1)
{
    uint32_t action = a1 ? field<uint32_t>(a1, 4) : 0xffffffffu;
    if (awdlLogValue()) mtk_dbg_log("awdl: companion create action=%u unsupported\n", action);
    return kIOReturnUnsupported;
}

namespace {

// The registration needs a queue set; the datapath arrives in phase 5, so the actions consume nothing yet.
uint32_t awdlTxSubmissionAction(OSObject *, IOSkywalkTxSubmissionQueue *, IOSkywalkPacket *const *, uint32_t, void *) { return 0; }
uint32_t awdlTxCompletionAction(OSObject *, IOSkywalkTxCompletionQueue *, IOSkywalkPacket **, uint32_t, void *) { return 0; }
uint32_t awdlRxCompletionAction(OSObject *, IOSkywalkRxCompletionQueue *, IOSkywalkPacket **, uint32_t, void *) { return 0; }

const uint32_t kAwdlPoolPackets = 0x100;      // enough for a phase-2 registration; the real datapath sizes this later
const uint32_t kAwdlPoolBufferSize = 0x800;
const uint32_t kAwdlQueueCapacity = 0x100;

} // namespace

void MtkAwdlInterface::freeAwdlRegistration()
{
    for (uint32_t i = 0; i < 3; i++) {
        if (awdlQueues[i]) {
            awdlQueues[i]->release();
            awdlQueues[i] = nullptr;
        }
    }
    if (awdlPool) {
        awdlPool->release();
        awdlPool = nullptr;
    }
}

IOReturn MtkAwdlInterface::registerAwdlInterface(const uint8_t *mac)
{
    memcpy(awdlAddr, mac, sizeof(awdlAddr));
    IOSkywalkPacketBufferPool::PoolOptions po;
    bzero(&po, sizeof(po));
    po.packets0 = po.packets1 = kAwdlPoolPackets;
    po.bufferSize = kAwdlPoolBufferSize;
    po.one = 1;
    po.zero = 0;
    po.flags = 1;
    awdlPool = IOSkywalkPacketBufferPool::withName("MtkAwdlPool", this, 1, &po);
    if (!awdlPool) return kIOReturnNoMemory;

    awdlQueues[0] = IOSkywalkTxSubmissionQueue::withPool(awdlPool, kAwdlQueueCapacity, 0, this, awdlTxSubmissionAction, this, 0);
    awdlQueues[1] = IOSkywalkTxCompletionQueue::withPool(awdlPool, kAwdlQueueCapacity, 1, this, awdlTxCompletionAction, this, 0);
    awdlQueues[2] = IOSkywalkRxCompletionQueue::withPool(awdlPool, kAwdlQueueCapacity, 2, this, awdlRxCompletionAction, this, 0);
    if (!awdlQueues[0] || !awdlQueues[1] || !awdlQueues[2]) {
        freeAwdlRegistration();
        return kIOReturnNoMemory;
    }

    // RegistrationInfo (opaque 0x130 bytes): the fields AppleBCMWLANCore::createProxInterface sets before calling the
    // proximity interface's start: +0xc = 3, +0x14 = 2, +0x24 = 0x8000000080, +0x30 = the name pointer,
    // +0x38 = 0 (BSD unit: registerNetworkInterface sets the IOInterfaceUnit property only when this is >= 0; the BSD client
    // needs that property to build its name), +0x3c = 0x40 (headroom), +0x40 = 4|6 (flags), +0x108 = the MAC.
    uint8_t info[0x130];
    bzero(info, sizeof(info));
    initRegistrationInfo((IOSkywalkEthernetInterface::RegistrationInfo *)info, 1, 0x130);
    field<uint32_t>(info, 0xc) = 3;
    field<uint32_t>(info, 0x14) = 2;
    field<uint64_t>(info, 0x24) = 0x8000000080ULL;
    field<const char *>(info, 0x30) = "awdl";
    field<uint32_t>(info, 0x38) = 0;             // unit 0 -> awdl0
    field<uint16_t>(info, 0x3c) = 0x40;
    field<uint32_t>(info, 0x40) = 6;
    memcpy((uint8_t *)info + 0x108, mac, 6);

    IOReturn rr = registerEthernetInterface((IOSkywalkEthernetInterface::RegistrationInfo *)info, awdlQueues, 3,
                                            awdlPool, awdlPool, 0);
    if (rr != kIOReturnSuccess) freeAwdlRegistration();
    return rr;
}

// -----------------------------------------------------------------------------------------------------------------------
// Boot-arg and layout check
// -----------------------------------------------------------------------------------------------------------------------
// The layouts in io80211/ are fixed when the kext is built, and OpenCore's kernel version range cannot tell two builds of
// the same macOS release apart. If an update changes one of these classes, awdl0's first constructor or virtual call lands
// in the wrong place. So AWDL stays off, capability and interface alike, unless every class it derives from has, in the
// running kernel, the instance size and the parent this build was made with. That cannot see a reordered vtable of the same
// size; it catches added fields and a changed hierarchy.
static bool awdlLayoutsMatch()
{
    struct Layout {
        const char *name;
        const OSMetaClass *meta;
        const OSMetaClass *parent;
        unsigned int size;
    };
    const Layout layouts[] = {
        { "IO80211AWDLProtocol", &IO80211AWDLProtocol::gMetaClass, &IO80211VirtualInterface::gMetaClass, sizeof(IO80211AWDLProtocol) },
        { "IO80211VirtualInterface", &IO80211VirtualInterface::gMetaClass, &IO80211SkywalkInterface::gMetaClass,
          sizeof(IO80211VirtualInterface) },
        { "IO80211SkywalkInterface", &IO80211SkywalkInterface::gMetaClass, &IOSkywalkEthernetInterface::gMetaClass,
          sizeof(IO80211SkywalkInterface) },
        { "IOSkywalkEthernetInterface", &IOSkywalkEthernetInterface::gMetaClass, &IOSkywalkNetworkInterface::gMetaClass,
          sizeof(IOSkywalkEthernetInterface) },
        { "IOSkywalkNetworkInterface", &IOSkywalkNetworkInterface::gMetaClass, &IOSkywalkInterface::gMetaClass,
          sizeof(IOSkywalkNetworkInterface) },
        { "IOSkywalkInterface", &IOSkywalkInterface::gMetaClass, &IOService::gMetaClass, sizeof(IOSkywalkInterface) },
    };
    for (const Layout &l : layouts) {
        unsigned int size = l.meta->getClassSize();
        if (size != l.size || l.meta->getSuperClass() != l.parent) {
            mtk_dbg_log("awdl: off: %s is %#x bytes in the kernel, %#x in this build, parent %s\n", l.name, size, l.size,
                        l.meta->getSuperClass() == l.parent ? "same" : "different");
            return false;
        }
    }
    mtk_dbg_log("awdl: on, the kernel's IO80211 classes match this build\n");
    return true;
}

bool mtkAwdlEnabled()
{
    static int cached = -1;
    if (cached < 0) {
        int v = 0;
        cached = (PE_parse_boot_argn("-mtkawdl", &v, sizeof(v)) && awdlLayoutsMatch()) ? 1 : 0;
    }
    return cached == 1;
}

// -----------------------------------------------------------------------------------------------------------------------
// APPLE80211_IOC_VIRTUAL_IF_CREATE (0x5e), role 6.
// -----------------------------------------------------------------------------------------------------------------------
// Non-virtual and exported by IO80211Family. IO80211Controller's own header is not included in this file.
extern void io80211SetAwdlInterface(IOService *controller, IO80211VirtualInterface *vif)
    __asm__("__ZN17IO80211Controller16setAwdlInterfaceEP23IO80211VirtualInterface");

// One card, one interface. It lives as long as the controller: the Family holds it through setAwdlInterface.
static MtkAwdlInterface *gAwdlVif;

IOReturn mtkAwdlCreateInterface(IOService *controller, const uint8_t *requestedMac, const uint8_t *primaryMac,
                                char *bsdName, size_t bsdNameLen)
{
    if (gAwdlVif) {                              // already created: answer with the existing BSD name
        const char *n = gAwdlVif->getBSDName();
        strlcpy(bsdName, n ? n : "", bsdNameLen);
        return kIOReturnSuccess;
    }

    MtkAwdlInterface *vif = (MtkAwdlInterface *)OSMetaClass::allocClassWithName("MtkAwdlInterface");
    if (!vif) {
        mtk_dbg_log("awdl: vif alloc failed\n");
        return kIOReturnUnsupported;
    }

    // MAC: userland may hand one in; otherwise derive the AWDL MAC from the primary one (locally administered, different
    // last byte), as Apple keeps a separate AWDL address derived from the Wi-Fi one.
    uint8_t mac[6];
    memcpy(mac, requestedMac, 6);
    bool zero = true;
    for (uint32_t i = 0; i < 6; i++) if (mac[i]) zero = false;
    if (zero) {
        memcpy(mac, primaryMac, 6);
        mac[0] |= 0x02;
        mac[5] ^= 0x01;
    }

    // Apple's init returns bool; the generated declaration says IOReturn, so only the low byte means anything.
    if (!(uint8_t)vif->init((IO80211Controller *)controller, (ether_addr *)mac, 6, "awdl")) {
        mtk_dbg_log("awdl: vif init failed\n");
        vif->release();
        return kIOReturnError;
    }
    if (!vif->attach(controller)) {
        mtk_dbg_log("awdl: vif attach failed\n");
        vif->release();
        return kIOReturnError;
    }

    // The Family's own interfaces bracket the whole setup with deferBSDAttach: the BSD client's start fails (defers) while
    // the property is set and re-probes only when it is removed, with the controller in place. start(ctl) also runs
    // IOService::start, so the framework will not call start again.
    vif->deferBSDAttach(true);
    if (!vif->start(controller)) {
        mtk_dbg_log("awdl: vif start failed\n");
        vif->release();
        return kIOReturnError;
    }

    IOReturn rr = vif->registerAwdlInterface(mac);
    if (rr != kIOReturnSuccess) {
        mtk_dbg_log("awdl: vif register rc=%x\n", rr);
        vif->release();
        return rr;
    }

    vif->registerService(kIOServiceAsynchronous);
    vif->setRunningState(false);
    io80211SetAwdlInterface(controller, vif);
    gAwdlVif = vif;
    vif->deferBSDAttach(false);

    // The BSD client builds the name during its asynchronous re-probe, so getBSDName() may still be empty here; the prefix
    // is ours and registerAwdlInterface gave the unit.
    const char *n = vif->getBSDName();
    strlcpy(bsdName, (n && *n) ? n : "awdl0", bsdNameLen);
    mtk_dbg_log("awdl: vif up bsd=%s\n", bsdName);
    return kIOReturnSuccess;
}
