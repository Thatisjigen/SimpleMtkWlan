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
#include <sys/errno.h>

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

class MtkAwdlInterface;
// The AWDL engine, at the end of this file.
static bool engineInit(MtkAwdlInterface *vif, const uint8_t *addr);
static void engineEnable(bool on);
static void engineSyncParams(const uint8_t *raw);
static void engineChanSeq(const uint8_t *s);
static void engineSyncState(uint32_t state, const uint8_t *master);
static void engineTopMaster(const uint8_t *top);
static void engineTemplate(const uint8_t *tlv, uint32_t len);
static IOReturn engineOob(const uint8_t *r, bool autoReq);
static void engineTraffic(bool active);

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
    /* 548 */ IOReturn setAWDL_OOB_REQUEST(apple80211_awdl_oob_request*a1) override { awdlFirstCall("setAWDL_OOB_REQUEST"); return engineOob((const uint8_t *)a1, false); }
    /* 549 */ IOReturn setAWDL_SYNC_FRAME_TEMPLATE(apple80211_awdl_sync_frame_template*a1) override;
    /* 550 */ IOReturn setAWDL_ELECTION_PARAMS(apple80211_awdl_election_params*a1) override;
    /* 551 */ IOReturn setAWDL_PEER_TRAFFIC_REGISTRATION(apple80211_awdl_peer_traffic_registration*a1) override;
    /* 552 */ IOReturn setAWDL_SECONDARY_MASTER_CHANNEL(apple80211_awdl_secondary_master_channel*a1) override { return awdlAccept("setAWDL_SECONDARY_MASTER_CHANNEL"); }
    /* 553 */ IOReturn setAWDL_AF_TX_MODE(apple80211_awdl_af_tx_mode*a1) override { return awdlAccept("setAWDL_AF_TX_MODE"); }
    /* 554 */ IOReturn setAWDL_MIN_RATE(apple80211_awdl_min_rate*a1) override { return awdlAccept("setAWDL_MIN_RATE"); }
    /* 555 */ IOReturn setAWDL_ENCRYPTION_KEYS(apple80211_key*a1) override { return awdlAccept("setAWDL_ENCRYPTION_KEYS"); }
    /* 556 */ IOReturn setAWDL_ENCRYPTION_TYPE(apple80211_encrypt*a1) override { return awdlAccept("setAWDL_ENCRYPTION_TYPE"); }
    /* 557 */ IOReturn setAWDL_OOB_AUTO_REQUEST(apple80211_awdl_oob_request*a1) override { awdlFirstCall("setAWDL_OOB_AUTO_REQUEST"); return engineOob((const uint8_t *)a1, true); }
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
    // The data path (end of this file).
    uint32_t submitData(IOSkywalkPacket *const *packets, uint32_t count);
    void serviceData();
    void deliverData(const uint8_t *frame, uint32_t len, uint8_t part);
    void deliverMsdu(const uint8_t *da, const uint8_t *sa, const uint8_t *pl, uint32_t plen);
    void enableData();

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
    bool awdlQueuesOnLoop = false;
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
    if (enabled) enableData();
    engineEnable(enabled != 0);
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
    engineSyncParams(awdlSyncParams);
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
    engineChanSeq(awdlChanSeq);
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
    engineSyncState(field<uint32_t>(awdlSyncState, 4), awdlSyncState + 8);
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
    engineTopMaster(awdlElectionParams + 0x16);
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
    const uint8_t *tlv = field<const uint8_t *>(a1, 8);
    if (len != awdlTemplateLen && awdlLogValue()) mtk_dbg_log("awdl: sync frame template %u bytes\n", len);
    awdlTemplateLen = len;
    engineTemplate((uintptr_t)tlv >= 0xffffff8000000000ULL ? tlv : nullptr, len);
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
    engineTraffic(field<uint32_t>(a1, 0xa8) != 0);
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

// awdl0's packets go to the radio (submitData); the completion queues are filled by the driver itself.
uint32_t awdlTxSubmissionAction(OSObject *owner, IOSkywalkTxSubmissionQueue *, IOSkywalkPacket *const *packets, uint32_t count,
                                void *)
{
    return owner ? ((MtkAwdlInterface *)owner)->submitData(packets, count) : 0;
}
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
    if (rr != kIOReturnSuccess) {
        freeAwdlRegistration();
        return rr;
    }
    // The queues are event sources: they run only on the interface's work queue (thanhnndev ca94db4).
    IOWorkLoop *wq = (IOWorkLoop *)getWorkQueue();
    uint32_t added = 0;
    for (uint32_t i = 0; wq && i < 3; i++)
        if (wq->addEventSource(awdlQueues[i]) == kIOReturnSuccess) added++;
    awdlQueuesOnLoop = added == 3;
    if (!engineInit(this, mac) || !awdlQueuesOnLoop) mtk_dbg_log("awdl: queues on the work queue %u/3\n", added);
    return kIOReturnSuccess;
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

// wifip2pd takes an interface only when it can read its role from the registry: IO80211VirtualInterfaceRole for the
// peer-to-peer ones, "AirLink" for AWDL (the key and the value IO80211SkywalkInterface::setInterfaceRole picks for role 6).
// That function, which IO80211VirtualInterface::init calls, stores the property through IOService::SetProperties, which
// ends in IORegistryEntry::setProperties: not implemented by this class, so the property never lands and wifip2pd drops
// awdl0 ("Unable to find role for interface"). The infrastructure interface has the same gap (AirportItlwm puts its
// IO80211InterfaceRole back after start). Written directly here, before the BSD interface is attached.
static const char kAwdlRoleKey[] = "IO80211VirtualInterfaceRole";
static const char kAwdlRoleValue[] = "AirLink";

static bool awdlRoleSet(IOService *vif)
{
    OSString *role = OSDynamicCast(OSString, vif->getProperty(kAwdlRoleKey));
    return role != nullptr && role->isEqualTo(kAwdlRoleValue);
}

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
    if (!awdlRoleSet(vif))
        vif->setProperty(kAwdlRoleKey, kAwdlRoleValue);
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

    if (!awdlRoleSet(vif)) {
        vif->setProperty(kAwdlRoleKey, kAwdlRoleValue);
        mtk_dbg_log("awdl: role property put back after start\n");
    }
    vif->registerService(kIOServiceAsynchronous);
    vif->setRunningState(false);
    io80211SetAwdlInterface(controller, vif);
    mtk_dbg_log("awdl: role %s=%s\n", kAwdlRoleKey, awdlRoleSet(vif) ? kAwdlRoleValue : "MISSING");
    gAwdlVif = vif;
    vif->deferBSDAttach(false);

    // The BSD client builds the name during its asynchronous re-probe, so getBSDName() may still be empty here; the prefix
    // is ours and registerAwdlInterface gave the unit.
    const char *n = vif->getBSDName();
    strlcpy(bsdName, (n && *n) ? n : "awdl0", bsdNameLen);
    mtk_dbg_log("awdl: vif up bsd=%s\n", bsdName);
    return kIOReturnSuccess;
}

// -----------------------------------------------------------------------------------------------------------------------
// The AWDL engine: the timeline, the presence schedule, the synchronization and master indication frames, the frames
// heard, and the data path of awdl0. The Family runs the election and builds the TLVs; on Apple hardware the firmware
// does the rest, and here this code does it, with the radio of the hardware layer (MtkAwdl.h).
//
// Time. An availability window (AW) is 16 TU, four AWs make an extended window (EAW, 64 TU), which takes one of the 16
// entries of the channel sequence. AW number `tlBaseSeq` starts at host uptime `tlBaseUs`. As a master the clock runs
// free; as a slave it follows the master's synchronization parameters TLV (type 4): +1 the TU to the sender's next EAW,
// +29 the AW running when the frame was built (thanhnndev, mt7925-macos-wifi 45ac9bf, 62afa98).
//
// Events. At the start of every EAW the Family's own sequence names a channel for, the Family gets
// APPLE80211_M_AWDL_AVAILABILITY_WINDOW_START (0x29); its timeline monitor, election checks and frame cadence run off
// them. Before the next one, the end of the previous one (0x2b), on which the Family runs its master election
// (thanhnndev 45ac9bf, 67fe911; the layouts are read from the Family's own handlers).
//
// Presence. The radio is where the schedule in our frames says it is, and nowhere else: TLV 4 and TLV 18 advertise
// exactly the EAWs the radio will spend on an AWDL channel, and 00 00 for the EAWs left to the access point. On the
// access point's channel the radio does not move: every EAW while AWDL is in use, the Family's sequence at rest. Off
// it the EAWs left to the access point are those of Apple's own schedule for the state AWDL and the access point are
// in (buildPresence()), and a run away is one remain-on-channel period; the firmware tells the access point about each
// absence. With nobody heard for 30 s the radio visits one EAW per sequence.
//
// Stillness. The Family keeps AWDL enabled for the whole boot and at rest still asks for 3 EAWs per sequence on AWDL
// channels (52 0 149 0 0 0 0 0 6 0 149 0 0 0 0 0 with the access point on 52). Apple's firmware makes those visits on its
// own; here each one is firmware commands and interrupts, and the timer runs every EAW. After kStillUs with no peer, no
// data and no request, the timeline stops posting windows and the radio goes back to the station (reception closed,
// OMAC 1 off), as with awdl0 down. A sequence of the Family denser than its idle pattern (a browse or an advertisement:
// AirDrop and AirPlay both start with one), a template of a new length (AirDrop's services), a frame or traffic request,
// or a packet of awdl0 starts it again from the next EAW. Turning AWDL on is not a use: the Family turns it on again
// after each firmware reload, with its idle sequence, and the timeline then waits for one of these.
//
// Threads. The Family's setters, the timer (the controller's work loop), the radio's stamp (radio thread, main gate) and
// the receive path (main work loop) share the state below under `lock`, a leaf: nothing is called with it held.
// -----------------------------------------------------------------------------------------------------------------------
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IOWorkLoop.h>
#include <libkern/OSByteOrder.h>

extern "C" IOReturn mtkAwdlFamilyPostMessage(IO80211Controller *, IO80211SkywalkInterface *, uint32_t, void *, unsigned long,
                                            bool) __asm__("__ZN17IO80211Controller11postMessageEP23IO80211SkywalkInterfacejPvmb");
extern "C" void *mtkAwdlFamilyMgmtPool(IO80211Controller *) __asm__("__ZN17IO80211Controller16getMgmtFramePoolEv");
// Consumes the buffer on every path: the Family returns it to its pool, or hands it to the peer manager.
extern "C" void mtkAwdlFamilyTapInput(IO80211VirtualInterface *, void *, unsigned int, void *, unsigned long)
    __asm__("__ZN23IO80211VirtualInterface11bpfTapInputEP13IO80211BufferjPvm");

namespace {

const uint32_t kTuUs = 1024, kAwUs = 16 * kTuUs, kAwPerEaw = 4, kEawUs = kAwUs * kAwPerEaw;
const uint32_t kTlvMax = 1400, kFrameMax = 1600, kPeers = 8;
// A peer heard or a packet of awdl0 keeps the denser schedule this long: AirDrop's Bonjour traffic on awdl0 comes in
// bursts several seconds apart while a sender is near.
const uint64_t kActiveUs = 30000000, kPeerStaleUs = 8000000;
// With nothing of that, and nothing the Family asks for, for this long the timeline stops and the radio stays with the
// station, as with awdl0 down (see noteActivity()).
const uint64_t kStillUs = 60000000;
// Off the access point's channel, the EAWs left to it (1) in each state, as Apple's IO80211Family builds its sequence
// from the rows of _channelSeqSlotTypes (Tahoe 26.7, read by zheznanohana for ForbiddenItlwm):
//  - no AWDL data: 6, 7, 14 and 15. Apple also turns the radio off in 3-5 and 11-13: here that is time at home;
//  - AWDL data, access point idle: none in Apple's row. 15 stays at home, as in ForbiddenItlwm: a radio that
//    switches channels by remain-on-channel requests needs one way back per sequence;
//  - AWDL data, access point with data: 4, 5 and 12-15;
//  - AWDL data, access point streaming: 4, 5 and 11-15.
// AWDL data: a unicast data frame either way in the last kUcHoldUs. Every row ends at home, so a run never wraps. A
// master that is on AWDL in none of the idle row's EAWs is met in 3-5 and 11-13 as well (kHomeIdleWide, ForbiddenItlwm
// keeps listening there) rather than lost.
const uint8_t kHomeIdle[16] = { 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1 };
const uint8_t kHomeIdleWide[16] = { 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1 };
const uint8_t kHomeDataApIdle[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };
const uint8_t kHomeDataApData[16] = { 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1 };
const uint8_t kHomeDataApStream[16] = { 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1 };
const uint64_t kUcHoldUs = 3000000;
// The access point's traffic as Apple rates it (currentInfraTrafficType): streaming when its packets average at least
// kApStreamBytes (and at least 6 came), data with more than kApDataPps packets a second, otherwise idle.
enum : uint8_t { kApIdle, kApData, kApStream };
const uint64_t kApStreamBytes = 1401, kApDataPps = 100;
const uint8_t kBssid[6] = { 0x00, 0x25, 0x00, 0xff, 0x94, 0x73 };
// What ended a still period, for the driver log.
enum : uint8_t { kWakeNone, kWakeSeq, kWakeTemplate, kWakeFrame, kWakeTraffic, kWakeData };
const char *const kWakeWhy[] = { "", "the Family's channel sequence", "a new sync frame template", "a frame request",
                                 "a traffic registration", "a packet of awdl0" };
const uint32_t kMsgAwStart = 0x29, kMsgAwEnd = 0x2b, kTapActionFrame = 0xc;

// apple80211_availability_window_start (0x24 bytes) and _end (0x28), as AppleBCMWLANProximityInterface builds them and
// IO80211AWDLPeerManager reads them (thanhnndev 45ac9bf, 67fe911).
struct AwStart {
    uint32_t seq, role;
    uint8_t master[6], topMaster[6];
    uint32_t startUsBe, chVersion, channel, chFlags;
};
static_assert(sizeof(AwStart) == 0x24, "availability window start");
struct AwEnd {
    uint32_t channel, timeUs, b8, wc;
    uint8_t stats[0x18];
};
static_assert(sizeof(AwEnd) == 0x28, "availability window end");

// The receive descriptor bpfTapInput(buf, 0xc, &hdr, 0x48) wants beside an action frame: actionFrameInput takes exactly
// 0x48 bytes (isValidAF refuses anything shorter). +0 the sender, +0xc an apple80211_channel, +0x2c the RSSI in dBm (a peer
// is created only at or above the edge threshold, and 0 reads as unreachable), +0x30 the receive time.
struct ActFrmHdr {
    uint8_t sa[6], rsvd0[6];
    uint32_t chVersion, channel, chFlags;
    uint8_t rsvd1[0x10];
    uint32_t rate;
    int32_t rssi;
    uint32_t rxTime;
    uint8_t rsvd2[0x14];
};
static_assert(sizeof(ActFrmHdr) == 0x48, "action frame receive descriptor");

struct Peer {
    uint8_t addr[6];
    bool valid;
    uint8_t n;               // entries in its channel sequence
    uint8_t ch[16];          // primary channel of each entry, 0 = absent
    uint64_t baseUs;         // start of an EAW of its timeline
    uint16_t baseSeq;        // and its AW number
    uint64_t lastUs;
    uint8_t masterIs;        // the master its synchronization parameters name: 1 itself, 2 us, 3 another device
    uint32_t heard, logged;  // frames heard from it, and when it was last logged
    uint8_t capsLogged;      // its HT (bit 0) and VHT (bit 1) capability TLVs, once in the log
};

struct RxDup { uint8_t sa[6]; uint16_t sc; bool valid; };

// The QoS data of one sender and TID. Under a Block Ack agreement a frame the peer sends again comes in a later A-MPDU,
// behind newer ones, so the last sequence number is not enough: hi is the newest number delivered and bit i of seen is
// hi - i, as a recipient's window of 64 (IEEE 802.11 duplicate detection with a scoreboard). There is no reordering:
// a late frame goes up as it comes, and TCP puts it in place. part: the subframes the hardware split out of one A-MSDU
// share its number, so the first decides and the others follow it (cur, curOk).
struct RxWin {
    uint8_t sa[6];
    uint8_t tid;
    bool valid, curOk;
    uint16_t hi, cur;
    uint64_t seen, lastUs;
    uint8_t old;              // frames older than the window in a row: the sender started again
};

struct Engine {
    IOLock *lock;
    MtkAwdlInterface *vif;
    IOTimerEventSource *tlTimer, *dpTimer;
    IOWorkLoop *wl;
    uint8_t addr[6];
    bool enabled, radioOn;
    // timeline
    bool tlRunning;
    uint64_t tlBaseUs;
    uint16_t tlBaseSeq;
    int32_t lastPostedAw;
    bool endPending;
    uint8_t endCh;
    uint64_t endUs, plannedUntilUs;
    // the election's result
    uint32_t familyState, role;
    uint8_t master[6], topMaster[6];
    uint64_t masterHeardUs;
    // the Family's channel sequence, and the channel chosen from it
    uint8_t famCh[16];
    bool famSet;
    uint8_t awdlCh, pres[16];
    uint32_t discoIdx;              // the social channel a discovery schedule starts from, moved every sequence
    uint8_t peerCh;                 // where the last AWDL frame of another device was heard
    uint8_t follow[6];              // a device that names itself master while the Family keeps us master
    int32_t lastRssi;               // of the last AWDL frame heard, as handed to the Family
    uint64_t followUs;
    uint32_t afPeriodTu;
    // the Family's TLVs
    uint8_t psf[kTlvMax], mi[kTlvMax], once[kTlvMax];
    uint32_t psfLen, miLen, onceLen;
    uint8_t ownCapsLogged;          // our HT and VHT capability TLVs (bits 0 and 1), once in the log
    uint8_t onceDa[6];
    uint16_t seqNo, dataSeq;
    Peer peers[kPeers];
    uint64_t peerHeardUs, dataUs;
    uint64_t ucUs;                  // the last unicast data frame, either way
    // the access point's traffic, sampled once a sequence (rateAccessPoint())
    uint64_t apPkts, apBytes, apSampleUs;
    uint8_t apClass;
    RxDup dup[8];
    uint32_t dupNext;
    RxWin win[8];
    uint32_t winNext;
    // data path
    bool dpOn, txBlocked;
    IOSkywalkPacket *txDone[64];
    uint32_t txDoneN;
    // nothing uses AWDL: the timeline posts nothing and the radio stays with the station until wakeReq is set
    bool still, famBusy;
    uint8_t wakeReq;
    uint64_t actUs, stillUs;        // the last request of the Family or the stack; when the still period began
    // counters
    uint32_t eaws, posted, postFail, ended, late, syncs, frames, framesFail, windows;
    uint32_t rxAf, rxAfFail, rxNoRssi, rxData, rxDataUp, rxDataFail, rxDup, rxNotUs;
    uint32_t txIn, txQueued, txFull, txBad, txEarly, rxAmsdu, rxSub, stills, stillS;
    uint32_t rxQos, rxQosDup, rxQosLate, rxQosOld, rxQosRestart, rxParts, rxPartsDropped;
    uint64_t logUs;
};
Engine gE;
uint8_t gTxScratch[kFrameMax];       // timer thread only
uint8_t gTxScratch2[kFrameMax];
uint8_t gDataTx[kFrameMax];          // data submission (work loop) only

inline uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
inline void wr16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
inline void wr32(uint8_t *p, uint32_t v) { for (int i = 0; i < 4; i++) p[i] = (uint8_t)(v >> (8 * i)); }
inline bool isZero6(const uint8_t *a) { return !(a[0] | a[1] | a[2] | a[3] | a[4] | a[5]); }

// A 20 MHz channel without radar duty, as the radio accepts it (mwx_awdl_channel_ok).
bool usableCh(uint32_t ch)
{
    return (ch >= 1 && ch <= 13) || (ch >= 36 && ch <= 48 && !(ch % 4)) || (ch >= 149 && ch <= 165 && (ch % 4) == 1);
}

uint8_t opClass(uint8_t ch)
{
    if (ch >= 1 && ch <= 13) return 81;
    if (ch >= 36 && ch <= 48) return 115;
    if (ch >= 149 && ch <= 161) return 124;
    if (ch == 165) return 125;
    return 0;
}

uint32_t chFlags(uint32_t ch) { return (ch <= 14 ? 0x8u : 0x10u) | 0x2u; }   // band | 20 MHz

// Where AW number `aw` starts on the current timeline. Lock held.
uint64_t awStartUs(uint16_t aw)
{
    return gE.tlBaseUs + (uint64_t)((int64_t)(int16_t)(uint16_t)(aw - gE.tlBaseSeq) * (int64_t)kAwUs);
}

// The AW running at `now` and its start. Lock held.
uint16_t awAt(uint64_t now, uint64_t *start)
{
    int64_t el = (int64_t)(now - gE.tlBaseUs);
    int64_t n = el >= 0 ? el / kAwUs : -((-el + kAwUs - 1) / kAwUs);
    if (start) *start = gE.tlBaseUs + (uint64_t)(n * (int64_t)kAwUs);
    return (uint16_t)(gE.tlBaseSeq + n);
}

// The AWDL channel: the one filling most of the Family's sequence among those the radio can use; 44, then 6, otherwise.
// Lock held.
void chooseChannel()
{
    uint8_t best = 0;
    uint32_t bestN = 0;
    for (uint32_t i = 0; i < 16; i++) {
        uint8_t c = gE.famCh[i];
        if (!usableCh(c) || !mtkAwdlRadioChannelAllowed(c)) continue;
        uint32_t n = 0;
        for (uint32_t j = 0; j < 16; j++) n += gE.famCh[j] == c;
        if (n > bestN || (n == bestN && c > 14 && best <= 14)) { best = c; bestN = n; }
    }
    if (!best) best = mtkAwdlRadioChannelAllowed(44) ? 44 : 6;
    if (best != gE.awdlCh) {
        mtk_dbg_log("awdl: channel %u (from the Family's sequence: %u of 16 slots)\n", best, bestN);
        gE.awdlCh = best;
    }
}

// AWDL's social channels, where devices look for each other until they synchronize: 6, and 149 in the US or 44 in
// Europe and most other countries. The Family picks its 5 GHz channel from the country the driver reports, which is the
// world domain until the access point's Country element has been read, so the channel it picked may not be the one the
// devices around use. Until a peer is heard the schedule visits all of them in turn: the Family's channel first, then 44,
// 6 and 149, those the radio accepts. Lock held.
uint32_t socialChannels(uint8_t *out)
{
    const uint8_t cand[4] = { gE.awdlCh, 44, 6, 149 };
    uint32_t n = 0;
    for (uint32_t i = 0; i < 4; i++) {
        uint8_t c = cand[i];
        bool dup = false;
        for (uint32_t j = 0; j < n; j++) dup = dup || out[j] == c;
        if (c && !dup && usableCh(c) && mtkAwdlRadioChannelAllowed(c)) out[n++] = c;
    }
    return n;
}

const Peer *findPeer(const uint8_t *a)
{
    for (const Peer &p : gE.peers)
        if (p.valid && !memcmp(p.addr, a, 6)) return &p;
    return nullptr;
}

// The Family or the stack asked for AWDL: the timeline keeps running, or runs again from the next EAW. Lock held; true
// when the caller has to kick the timeline (kickTimeline(), outside the lock) to end a still period.
bool noteActivity(uint64_t now, uint8_t why)
{
    gE.actUs = now;
    if (!gE.still || gE.wakeReq) return false;
    gE.wakeReq = why;
    return true;
}

void kickTimeline()
{
    if (gE.tlTimer) gE.tlTimer->setTimeoutUS(1);
}

// The last use of AWDL: a request of the Family or the stack, a peer heard, data either way; 0 for none since the boot.
// Lock held.
uint64_t lastUseUs()
{
    uint64_t last = gE.actUs;
    if (gE.peerHeardUs > last) last = gE.peerHeardUs;
    if (gE.dataUs > last) last = gE.dataUs;
    return last;
}

// Nothing used AWDL for kStillUs: no peer heard, no data either way, no request of the Family or the stack, the Family's
// sequence back to its idle pattern and no frame waiting to go out. Lock held.
bool quietFor(uint64_t now)
{
    uint64_t last = lastUseUs();
    return !gE.famBusy && !gE.onceLen && last < now && now - last > kStillUs;
}

// The access point's traffic class (kApIdle...), from the station's data frames either way, at most once every half
// second: buildPresence() runs once a sequence. Lock held.
void rateAccessPoint(uint64_t now)
{
    uint64_t pkts = 0, bytes = 0;
    if (gE.apSampleUs && now - gE.apSampleUs < 500000) return;
    mtkAwdlRadioStationTraffic(&pkts, &bytes);
    if (gE.apSampleUs && pkts >= gE.apPkts && bytes >= gE.apBytes) {
        uint64_t dp = pkts - gE.apPkts, db = bytes - gE.apBytes, el = now - gE.apSampleUs;
        gE.apClass = (dp >= 6 && db / dp >= kApStreamBytes) ? kApStream : dp * 1000000 > kApDataPps * el ? kApData : kApIdle;
    }
    gE.apPkts = pkts;
    gE.apBytes = bytes;
    gE.apSampleUs = now;
}

// The schedule from where the radio would be in each EAW (cand) and the EAWs left to the access point (row[i] set):
// present on the access point's channel whatever the row, since being there costs nothing; never two different AWDL
// channels back to back. Lock held.
void placeRuns(const uint8_t *cand, const uint8_t *row, uint8_t home)
{
    uint8_t prev = 0;
    for (uint32_t i = 0; i < 16; i++) {
        uint8_t c = cand[i];
        if (!(c && c == home) && (!c || row[i] || (prev && prev != c))) c = 0;
        gE.pres[i] = c;
        prev = (c && c != home) ? c : 0;
    }
}

// The schedule for the next 16 EAWs (see the top). Lock held.
void buildPresence(uint64_t now)
{
    uint8_t ch = gE.awdlCh, home = mtkAwdlRadioHomeChannel();
    uint8_t cand[16] = {};
    bool active = (gE.peerHeardUs && now - gE.peerHeardUs < kActiveUs) || (gE.dataUs && now - gE.dataUs < kActiveUs) ||
                  (gE.actUs && now - gE.actUs < kActiveUs) || gE.famBusy;
    rateAccessPoint(now);
    bzero(gE.pres, sizeof(gE.pres));
    if (!ch) return;
    if (home && home == ch) {
        // The access point is on the AWDL channel: being there costs nothing. At rest the schedule is the Family's
        // sequence; while AWDL is in use, every EAW. The Family keeps its idle sequence during a transfer (3 EAWs in 16
        // on the channel), peers send only in the EAWs we advertise, and our frames go out only in them.
        for (uint32_t i = 0; i < 16; i++) gE.pres[i] = (active || !gE.famSet || gE.famCh[i] == ch) ? ch : 0;
        return;
    }
    uint8_t soc[4];
    uint32_t nsoc = socialChannels(soc);
    if (!active) {
        // Nobody around: one EAW in 16, on each social channel in turn.
        gE.pres[0] = nsoc ? soc[gE.discoIdx++ % nsoc] : ch;
        return;
    }
    const Peer *m = gE.role == 2 ? findPeer(gE.master) : nullptr;
    if (!m && gE.followUs && now - gE.followUs < kPeerStaleUs) m = findPeer(gE.follow);
    bool synced = m && m->n && now - m->lastUs < kPeerStaleUs;
    if (synced) {
        // Synchronized to the master: its present EAWs are ours, its empty ones are our time at home.
        for (uint32_t i = 0; i < 16; i++) {
            uint8_t c = m->ch[i % m->n];
            cand[i] = (usableCh(c) && mtkAwdlRadioChannelAllowed(c)) ? c : 0;
        }
    } else if (gE.peerCh && gE.peerHeardUs && now - gE.peerHeardUs < kActiveUs && mtkAwdlRadioChannelAllowed(gE.peerCh)) {
        // A device was heard: stay on its channel in every EAW the time away from home allows, and stop turning, so the
        // schedule we advertise holds still and the device finds us there (rotating left it 2 EAWs in 16 on its
        // channel: the peer came and went every second and unicast frames went unanswered).
        for (uint32_t i = 0; i < 16; i++) cand[i] = gE.peerCh;
    } else {
        // Looking for peers: the first half's AWDL EAWs on one social channel, the second half's on the next.
        for (uint32_t i = 0; i < 16; i++) cand[i] = nsoc ? soc[(gE.discoIdx + i / 8) % nsoc] : ch;
        gE.discoIdx += 2;
    }
    // The EAWs left to the access point: Apple's row for the state (see kHomeIdle).
    bool data = gE.ucUs && now - gE.ucUs < kUcHoldUs;
    placeRuns(cand, !data ? kHomeIdle : gE.apClass == kApStream ? kHomeDataApStream :
              gE.apClass == kApData ? kHomeDataApData : kHomeDataApIdle, home);
    // A master that is on AWDL in none of the idle row's EAWs is met where Apple's radio is off too (kHomeIdleWide).
    bool met = false;
    for (uint32_t i = 0; i < 16; i++) met = met || gE.pres[i];
    if (!data && synced && !met) placeRuns(cand, kHomeIdleWide, home);
}

// TLV 4, the synchronization parameters (73 value bytes; thanhnndev d4b9008, seemoo-lab owl's layout). The counters at
// +1, +15 and +29 are written again by stampFrame() right before the frame leaves. Lock held.
uint32_t buildSyncTlv(uint8_t *out, uint8_t txCh)
{
    const uint32_t vlen = 39 + 16 * 2 + 2;
    uint8_t *v = out + 3;
    bzero(out, 3 + vlen);
    out[0] = 4;
    wr16(out + 1, (uint16_t)vlen);
    v[0] = txCh;
    v[3] = gE.awdlCh;
    wr16(v + 5, 16);                                    // AW period (TU)
    wr16(v + 7, (uint16_t)(gE.afPeriodTu ? gE.afPeriodTu : 110));   // action frame period (TU)
    wr16(v + 9, 0x1800);                                // AWDL flags
    wr16(v + 11, 16);                                   // AW extension length
    wr16(v + 13, 16);                                   // AW common length
    v[17] = 3; v[18] = 3; v[19] = 3; v[20] = 3;         // extension counts
    memcpy(v + 21, gE.role == 2 && !isZero6(gE.master) ? gE.master : gE.addr, 6);
    v[27] = (uint8_t)kAwPerEaw;                         // presence mode
    v[33] = 15;                                         // 16 entries
    v[34] = 3;                                          // encoding: channel, operating class
    v[36] = 3;                                          // step count: an entry lasts 4 AWs
    v[37] = 0xff; v[38] = 0xff;                         // no fill channel
    for (uint32_t i = 0; i < 16; i++) {
        v[39 + 2 * i] = gE.pres[i];
        v[40 + 2 * i] = gE.pres[i] ? opClass(gE.pres[i]) : 0;
    }
    return 3 + vlen;
}

// TLV 18, the channel sequence, the same 16 entries (41 value bytes, as peers send it). Lock held.
uint32_t buildSeqTlv(uint8_t *out)
{
    const uint32_t vlen = 6 + 32 + 3;
    bzero(out, 3 + vlen);
    out[0] = 18;
    wr16(out + 1, (uint16_t)vlen);
    uint8_t *v = out + 3;
    v[0] = 15; v[1] = 3; v[2] = 0; v[3] = 3; v[4] = 0xff; v[5] = 0xff;
    for (uint32_t i = 0; i < 16; i++) {
        v[6 + 2 * i] = gE.pres[i];
        v[7 + 2 * i] = gE.pres[i] ? opClass(gE.pres[i]) : 0;
    }
    return 3 + vlen;
}

// One action frame: the 802.11 header, the 16-byte AWDL action header (category 0x7f, Apple's OUI, type 8, version
// 1.0, the subtype: 0 PSF, 3 master indication), our TLV 4 and TLV 18, then the Family's TLVs without theirs. 0 when it
// does not fit. Lock held.
uint32_t buildFrame(uint8_t *out, uint32_t cap, uint8_t subtype, const uint8_t *da, const uint8_t *tlv, uint32_t tlvLen,
                    uint8_t txCh)
{
    uint8_t t4[3 + 73], t18[3 + 41];
    uint32_t l4 = buildSyncTlv(t4, txCh), l18 = buildSeqTlv(t18), pos = 40;
    if (pos + l4 + l18 > cap) return 0;
    bzero(out, 40);
    out[0] = 0xd0;
    memcpy(out + 4, da, 6);
    memcpy(out + 10, gE.addr, 6);
    memcpy(out + 16, kBssid, 6);
    wr16(out + 22, (uint16_t)((gE.seqNo++ & 0xfff) << 4));
    out[24] = 0x7f; out[25] = 0x00; out[26] = 0x17; out[27] = 0xf2; out[28] = 0x08; out[29] = 0x10; out[30] = subtype;
    memcpy(out + pos, t4, l4); pos += l4;
    memcpy(out + pos, t18, l18); pos += l18;
    for (uint32_t off = 0; off + 3 <= tlvLen;) {
        uint32_t l = rd16(tlv + off + 1);
        if (off + 3 + l > tlvLen) break;
        if (tlv[off] != 4 && tlv[off] != 18) {
            if (pos + 3 + l > cap) return 0;
            memcpy(out + pos, tlv + off, 3 + l);
            pos += 3 + l;
        }
        off += 3 + l;
    }
    return pos;
}

void armTimer(uint64_t now)
{
    if (!gE.tlTimer) return;
    uint64_t at;
    IOLockLock(gE.lock);
    bool run = gE.enabled && gE.tlRunning && !gE.still;
    uint64_t s = 0;
    uint16_t aw = awAt(now, &s);
    uint32_t k = 1;
    while (((uint16_t)(aw + k) % kAwPerEaw) != 0) k++;
    at = s + (uint64_t)k * kAwUs;
    IOLockUnlock(gE.lock);
    if (!run) return;
    uint64_t d = at > now ? at - now : 200;
    if (d < 200) d = 200;
    gE.tlTimer->setTimeoutUS((uint32_t)d + 100);       // just inside the EAW
}

void postEvents(bool end, const AwEnd &ee, bool start, AwStart &ev)
{
    MtkAwdlInterface *vif = gE.vif;
    IO80211Controller *ctl = vif ? vif->getController() : nullptr;
    if (!ctl) return;
    if (end) {
        AwEnd e = ee;
        if (mtkAwdlFamilyPostMessage(ctl, vif, kMsgAwEnd, &e, sizeof(e), true) == kIOReturnSuccess) gE.ended++;
    }
    if (start) {
        IOReturn r = mtkAwdlFamilyPostMessage(ctl, vif, kMsgAwStart, &ev, sizeof(ev), true);
        if (r == kIOReturnSuccess) gE.posted++;
        else if (++gE.postFail <= 4) mtk_dbg_log("awdl: AW start event refused 0x%x\n", r);
    }
}

void logState(uint64_t now)
{
    if (gE.logUs && now - gE.logUs < 30000000ULL) return;
    gE.logUs = now;
    char radio[480];
    char pres[17];
    for (uint32_t i = 0; i < 16; i++) {
        uint8_t c = gE.pres[i];
        pres[i] = !c ? '.' : c == 6 ? 'a' : c == 44 ? 'b' : c == 149 ? 'c' : c == mtkAwdlRadioHomeChannel() ? 'h' : 'x';
    }
    pres[16] = 0;
    mtkAwdlRadioSummary(radio, sizeof(radio));
    // pres: one character per EAW, a = 6, b = 44, c = 149, h = the access point's channel, x = another, . = at home;
    // ap: the access point's traffic, i(dle), d(ata) or s(treaming)
    mtk_dbg_log("awdl: tl role=%u ch=%u pres=%s ap=%c eaws=%u posted=%u/%u end=%u late=%u syncs=%u "
                "frames=%u/%u rx af=%u/%u norssi=%u data=%u/%u/%u amsdu=%u/%u dup=%u notus=%u tx in=%u q=%u full=%u bad=%u "
                "early=%u still=%u/%us\n",
                gE.role, gE.awdlCh, pres, "ids"[gE.apClass % 3], gE.eaws, gE.posted, gE.postFail, gE.ended, gE.late, gE.syncs,
                gE.frames, gE.framesFail, gE.rxAf, gE.rxAfFail, gE.rxNoRssi, gE.rxData, gE.rxDataUp, gE.rxDataFail,
                gE.rxAmsdu, gE.rxSub, gE.rxDup,
                gE.rxNotUs, gE.txIn, gE.txQueued, gE.txFull, gE.txBad, gE.txEarly, gE.stills, gE.stillS);
    mtk_dbg_log("awdl: radio %s\n", radio);
    mtkAwdlRadioChannels(radio, sizeof(radio));
    mtk_dbg_log("awdl: radio %s\n", radio);
    mtkAwdlRadioBlockAck(radio, sizeof(radio));
    mtk_dbg_log("awdl: radio %s\n", radio);
    // QoS data received: delivered, sent again (dropped), late inside the window (delivered), older than the window
    // (dropped), windows started again; subframes split by the hardware, and those dropped with their A-MSDU
    mtk_dbg_log("awdl: rx qos=%u dup=%u late=%u old=%u restart=%u parts=%u/%u\n", gE.rxQos, gE.rxQosDup, gE.rxQosLate,
                gE.rxQosOld, gE.rxQosRestart, gE.rxParts, gE.rxPartsDropped);
}

// One EAW boundary of the timeline. Controller work loop.
void tlTick()
{
    uint64_t now = mtkAwdlUptimeUs();
    AwStart ev;
    AwEnd ee;
    MtkAwdlWindow w;
    bool doEnd = false, doStart = false, doWin = false;
    uint32_t l1 = 0, l2 = 0;
    uint8_t curCh = 0;

    bzero(&ev, sizeof(ev));
    bzero(&ee, sizeof(ee));
    bzero(&w, sizeof(w));
    IOLockLock(gE.lock);
    if (!gE.enabled || !gE.tlRunning) {
        IOLockUnlock(gE.lock);
        return;
    }
    if (gE.still) {
        // Only a request ends a still period (noteActivity() kicks the timer); a stray firing changes nothing. The
        // schedule is planned again for what woke it, and the next EAW boundary carries on as usual.
        uint8_t why = gE.wakeReq;
        if (why == kWakeNone) {
            IOLockUnlock(gE.lock);
            return;
        }
        uint32_t secs = (uint32_t)((now - gE.stillUs) / 1000000);
        gE.still = false;
        gE.wakeReq = kWakeNone;
        gE.stillS += secs;
        gE.endPending = false;
        gE.plannedUntilUs = 0;
        chooseChannel();
        buildPresence(now);
        IOLockUnlock(gE.lock);
        int rv = mtkAwdlRadioStart(gE.addr);
        gE.radioOn = rv == 0;
        if (gE.txBlocked && gE.dpTimer) gE.dpTimer->setTimeoutMS(5);     // the packets that woke it
        mtk_dbg_log("awdl: awake after %u s still: %s%s\n", secs, why < sizeof(kWakeWhy) / sizeof(kWakeWhy[0]) ?
                    kWakeWhy[why] : "?", rv ? ", but the radio did not start" : "");
        armTimer(now);
        return;
    }
    uint64_t awS = 0;
    uint16_t aw = awAt(now, &awS);
    if ((aw % kAwPerEaw) != 0 || now - awS > kAwUs / 2 || gE.lastPostedAw == (int32_t)aw) {
        if ((aw % kAwPerEaw) != 0 || now - awS > kAwUs / 2) gE.late++;
        IOLockUnlock(gE.lock);
        armTimer(now);
        return;
    }
    gE.lastPostedAw = aw;
    gE.eaws++;
    uint32_t slot = ((uint32_t)aw / kAwPerEaw) & 15;
    uint8_t fam = gE.famSet ? gE.famCh[slot] : gE.awdlCh;
    curCh = gE.pres[slot];                      // this EAW, as its window was planned
    if (slot == 15) {                           // the schedule changes only from one sequence to the next
        chooseChannel();
        buildPresence(now);
    }
    if (gE.famBusy) gE.actUs = now;             // the Family asks for more than its idle presence
    if (quietFor(now)) {
        // Close the window the Family was told about, post no new one, give the radio back to the station and leave
        // the timer off until a request comes.
        if (gE.endPending) {
            doEnd = true;
            ee.channel = gE.endCh;
            ee.timeUs = (uint32_t)gE.endUs;
        }
        gE.endPending = false;
        gE.still = true;
        gE.wakeReq = kWakeNone;
        gE.stillUs = now;
        gE.stills++;
        uint64_t last = lastUseUs();
        IOLockUnlock(gE.lock);
        postEvents(doEnd, ee, false, ev);
        mtkAwdlRadioStop();
        gE.radioOn = false;
        if (last != 0)
            mtk_dbg_log("awdl: still: no AWDL use for %u s\n", (uint32_t)((now - last) / 1000000));
        else
            mtk_dbg_log("awdl: still: no AWDL use since the boot\n");
        gE.logUs = 0;
        logState(now);
        return;
    }
    if (fam && gE.endPending) {
        doEnd = true;
        gE.endPending = false;
        ee.channel = gE.endCh;
        ee.timeUs = (uint32_t)gE.endUs;
    }
    if (fam) {
        doStart = true;
        gE.endPending = true;
        gE.endCh = fam;
        gE.endUs = awS + kAwUs;
        ev.seq = aw;
        ev.role = gE.familyState;
        memcpy(ev.master, gE.role == 2 && !isZero6(gE.master) ? gE.master : gE.addr, 6);
        memcpy(ev.topMaster, !isZero6(gE.topMaster) ? gE.topMaster : ev.master, 6);
        ev.startUsBe = OSSwapHostToBigInt32((uint32_t)awS);
        ev.chVersion = 1;
        ev.channel = fam;
        ev.chFlags = chFlags(fam);
    }
    // The radio's next window, one EAW ahead, covering a run of present EAWs on one channel: away from the access point
    // the whole run, one remain-on-channel period; on its channel up to 3 EAWs, each window chained to the next.
    uint32_t ns = (slot + 1) & 15;
    uint64_t nextStart = awS + kEawUs;
    uint8_t nch = gE.pres[ns];
    if (nch && nextStart >= gE.plannedUntilUs) {
        uint32_t run = 1, most = nch == mtkAwdlRadioHomeChannel() ? 3 : 15;
        while (run < most && gE.pres[(ns + run) & 15] == nch) run++;
        w.channel = nch;
        w.dwellMs = (uint16_t)(run * kEawUs / 1000 - 2);
        w.startUs = nextStart;
        gE.plannedUntilUs = nextStart + (uint64_t)run * kEawUs;
        doWin = true;
    }
    // In a present EAW, our frames go out at its start: the PSF, then the master indication, or a one-shot request.
    if (curCh) {
        static const uint8_t bcast[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
        if (gE.psfLen) l1 = buildFrame(gTxScratch, sizeof(gTxScratch), 0, bcast, gE.psf, gE.psfLen, curCh);
        if (gE.onceLen) {
            l2 = buildFrame(gTxScratch2, sizeof(gTxScratch2), 3, gE.onceDa, gE.once, gE.onceLen, curCh);
            gE.onceLen = 0;
        } else if (gE.miLen) {
            l2 = buildFrame(gTxScratch2, sizeof(gTxScratch2), 3, bcast, gE.mi, gE.miLen, curCh);
        }
    }
    IOLockUnlock(gE.lock);

    postEvents(doEnd, ee, doStart, ev);
    if (doWin && mtkAwdlRadioWindow(&w) == 0) gE.windows++;
    if (l1) { if (mtkAwdlRadioQueue(gTxScratch, l1, curCh, 40) == 0) gE.frames++; else gE.framesFail++; }
    if (l2) { if (mtkAwdlRadioQueue(gTxScratch2, l2, curCh, 40) == 0) gE.frames++; else gE.framesFail++; }
    if (gE.eaws <= 2 || (gE.eaws % 64) == 0) logState(now);
    armTimer(mtkAwdlUptimeUs());
}

void tlTimerFired(OSObject *, IOTimerEventSource *) { tlTick(); }

// The HT (7) and VHT (17) capability TLVs among `tlvs`, in the driver log: what a device says about aggregation, the
// A-MPDU parameters and the largest A-MSDU it takes. Only those not in `done` (bit 0 HT, bit 1 VHT); returns the bits of
// those logged. Never waits, so it can run with the lock held.
uint8_t logCapTlvs(const char *who, const uint8_t *tlvs, uint32_t len, uint8_t done)
{
    uint8_t logged = 0;
    for (uint32_t pos = 0; pos + 3 <= len;) {
        uint32_t l = rd16(tlvs + pos + 1);
        if (pos + 3 + l > len) break;
        uint8_t bit = tlvs[pos] == 7 ? 1 : tlvs[pos] == 17 ? 2 : 0;
        if (bit && !(done & bit)) {
            char hex[3 * 24 + 1];
            uint32_t k = 0;
            for (uint32_t i = 0; i < l && i < 24; i++) k += (uint32_t)snprintf(hex + k, sizeof(hex) - k, " %02x", tlvs[pos + 3 + i]);
            hex[k] = 0;
            mtk_dbg_log("awdl: %s %s capabilities, %u bytes:%s\n", who, bit == 1 ? "HT" : "VHT", l, hex);
            logged |= bit;
        }
        pos += 3 + l;
    }
    return logged;
}

// The master's TLV 4 gives its timeline: re-align ours as its slave (thanhnndev 62afa98, 44d5bd2). Lock held.
void trackPeer(const uint8_t *sa, const uint8_t *body, uint32_t bodyLen, uint64_t rxUs)
{
    const uint8_t *v = nullptr;
    uint32_t vl = 0;
    for (uint32_t pos = 16; pos + 3 <= bodyLen;) {
        uint32_t l = rd16(body + pos + 1);
        if (pos + 3 + l > bodyLen) return;
        if (body[pos] == 4 && l >= 40) { v = body + pos + 3; vl = l; break; }
        pos += 3 + l;
    }
    if (!v) return;
    uint32_t pm = (v[27] >= 1 && v[27] <= 16) ? v[27] : kAwPerEaw;
    uint32_t phyTx = (uint32_t)body[8] | ((uint32_t)body[9] << 8) | ((uint32_t)body[10] << 16) | ((uint32_t)body[11] << 24);
    uint32_t tgtTx = (uint32_t)body[12] | ((uint32_t)body[13] << 8) | ((uint32_t)body[14] << 16) | ((uint32_t)body[15] << 24);
    uint32_t late = phyTx - tgtTx;
    if (late > 50000) late = 0;
    uint32_t dc = rd16(v + 1);
    uint16_t sn = rd16(v + 29);
    if (dc > 16 * pm + 16 || pm != kAwPerEaw) return;
    uint64_t base = rxUs + (uint64_t)dc * kTuUs - late;
    uint16_t seq = (uint16_t)(sn - (sn % pm) + pm);
    // its channel sequence: +33 count - 1, +34 encoding, +39 the entries
    uint32_t n = (uint32_t)v[33] + 1, enc = v[34], esz = enc == 0 ? 1 : 2;
    Peer *p = nullptr;
    for (Peer &q : gE.peers) if (q.valid && !memcmp(q.addr, sa, 6)) { p = &q; break; }
    if (!p) {
        p = &gE.peers[0];
        for (Peer &q : gE.peers) {
            if (!q.valid) { p = &q; break; }
            if (q.lastUs < p->lastUs) p = &q;
        }
        bzero(p, sizeof(*p));
        memcpy(p->addr, sa, 6);
    }
    p->valid = true;
    p->baseUs = base;
    p->baseSeq = seq;
    p->lastUs = rxUs;
    if (n <= 16 && 39 + n * esz <= vl) {
        p->n = (uint8_t)n;
        for (uint32_t i = 0; i < 16; i++) {
            uint8_t c = 0;
            if (i < n) {
                const uint8_t *e = v + 39 + i * esz;
                if (enc == 1) {                 // legacy: flags, center channel; 40 MHz entries name the primary by bits 0-1
                    c = e[1];
                    if (c > 14 && (e[0] & 3) == 1) c = (uint8_t)(c - 2);
                    else if (c > 14 && (e[0] & 3) == 2) c = (uint8_t)(c + 2);
                } else {
                    c = e[0];
                }
            }
            p->ch[i] = usableCh(c) ? c : 0;
        }
    }
    // Who it says the master is (+21 of the synchronization parameters): itself, us, or another device.
    uint8_t claim = !memcmp(v + 21, sa, 6) ? 1 : !memcmp(v + 21, gE.addr, 6) ? 2 : 3;
    bool claimChanged = claim != p->masterIs;
    p->masterIs = claim;
    p->heard++;
    if ((p->capsLogged & 3) != 3 && bodyLen > 16) {
        char who[24];
        snprintf(who, sizeof(who), "peer ..:%02x:%02x", sa[4], sa[5]);
        p->capsLogged |= logCapTlvs(who, body + 16, bodyLen - 16, p->capsLogged);
    }
    if (claimChanged || p->heard - p->logged >= 200 || p->heard == 1) {
        p->logged = p->heard;
        // where its EAWs start within ours, 0 to 65535 us (0: aligned); -1 before our timeline runs
        int64_t off = -1;
        if (gE.tlRunning) {
            int64_t d = (int64_t)(base - gE.tlBaseUs) % (int64_t)kEawUs;
            off = d < 0 ? d + (int64_t)kEawUs : d;
        }
        char seqs[17];
        for (uint32_t i = 0; i < 16; i++) {
            uint8_t c = p->ch[i];
            seqs[i] = !c ? '.' : c == 6 ? 'a' : c == 44 ? 'b' : c == 149 ? 'c' : 'x';
        }
        seqs[16] = 0;
        // rssi: the Family's peer manager takes a new peer only at -70 dBm or above (thanhnndev, mt7925-macos-wifi
        // roadmap 10.86: below it the peer never got a neighbour entry and every unicast to it timed out)
        mtk_dbg_log("awdl: peer ..:%02x:%02x names %s as master, seq=%s n=%u off=%lld us aw=%u heard=%u role=%u rssi=%d\n",
                    sa[4], sa[5], claim == 1 ? "itself" : claim == 2 ? "us" : "another device", seqs, p->n, (long long)off,
                    (unsigned)seq, p->heard, gE.role, (int)gE.lastRssi);
    }
    // The Family elects the master from the election parameters; when it keeps us master while this device declares
    // itself master, neither follows the other and the two meet only by chance (the Pixel: present 1-3 s at a time,
    // unicast unanswered). Then this device's timeline and sequence are taken as a slave's would be; one that names us
    // stays on ours.
    if (gE.role != 2 && gE.tlRunning && claim == 1) {
        if (gE.followUs == 0 || memcmp(gE.follow, sa, 6)) {
            memcpy(gE.follow, sa, 6);
            mtk_dbg_log("awdl: following ..:%02x:%02x, which names itself master\n", sa[4], sa[5]);
        }
        gE.followUs = rxUs;
        gE.tlBaseUs = base;
        gE.tlBaseSeq = seq;
        gE.syncs++;
        return;
    }
    if (gE.role != 2 || !gE.tlRunning) return;
    bool direct = !memcmp(sa, gE.master, 6);
    bool tree = !memcmp(v + 21, gE.master, 6);
    if (!direct && !(tree && (!gE.masterHeardUs || rxUs - gE.masterHeardUs > 1000000))) return;
    if (direct) gE.masterHeardUs = rxUs;
    gE.tlBaseUs = base;
    gE.tlBaseSeq = seq;
    gE.syncs++;
}

void dpTimerFired(OSObject *, IOTimerEventSource *);

} // namespace

// -----------------------------------------------------------------------------------------------------------------------
// Called by the radio (MtkAwdl.h).
// -----------------------------------------------------------------------------------------------------------------------
void mtkAwdlStampFrame(uint8_t *f, uint32_t len, uint64_t nowUs)
{
    if (!f || len < 40 || f[0] != 0xd0 || f[24] != 0x7f || f[26] != 0x17 || f[27] != 0xf2 || f[28] != 0x08 || !gE.lock) return;
    wr32(f + 32, (uint32_t)nowUs);                      // PHY TX time and target TX time: the frame leaves now
    wr32(f + 36, (uint32_t)nowUs);
    for (uint32_t pos = 40; pos + 3 <= len;) {
        uint32_t l = rd16(f + pos + 1);
        if (pos + 3 + l > len) return;
        if (f[pos] == 4 && l >= 31) {
            uint8_t *v = f + pos + 3;
            uint64_t s = 0;
            IOLockLock(gE.lock);
            uint16_t aw = awAt(nowUs, &s);
            uint32_t k = 1;
            while (((uint16_t)(aw + k) % kAwPerEaw) != 0) k++;
            uint64_t next = s + (uint64_t)k * kAwUs;
            uint64_t eawStart = awStartUs((uint16_t)(aw - (aw % kAwPerEaw)));
            IOLockUnlock(gE.lock);
            uint32_t dc = (uint32_t)((next > nowUs ? next - nowUs : 0) + kTuUs - 1) / kTuUs;
            wr16(v + 1, (uint16_t)dc);
            uint64_t into = nowUs - eawStart;
            wr16(v + 15, (uint16_t)(into < kAwUs ? (kAwUs - into) / kTuUs : 0));
            wr16(v + 29, aw);
            return;
        }
        pos += 3 + l;
    }
}

void mtkAwdlRadioRoomFreed()
{
    if (gE.dpTimer && gE.txBlocked) gE.dpTimer->setTimeoutUS(1);
}

void mtkAwdlInput(const uint8_t *frame, uint32_t len, uint32_t channel, int32_t rssi, uint64_t timeUs, uint8_t part)
{
    MtkAwdlInterface *vif = gE.vif;
    if (!vif || !gE.lock || !frame || len < 24) return;
    IOLockLock(gE.lock);
    bool on = gE.enabled;
    IOLockUnlock(gE.lock);
    if (!on) return;
    if ((frame[0] & 0x0c) == 0x08) {
        vif->deliverData(frame, len, part);
        return;
    }
    if (frame[0] != 0xd0 || len < 40) return;
    const uint8_t *body = frame + 24;
    uint32_t bodyLen = len - 24, pos = 16;
    // The sniffer can leave the FCS behind the last TLV: the TLVs say where the frame ends.
    while (pos + 3 <= bodyLen) {
        uint32_t l = rd16(body + pos + 1);
        if (pos + 3 + l > bodyLen) break;
        pos += 3 + l;
    }
    if (pos != bodyLen) {
        if (bodyLen - pos > 4) { gE.rxAfFail++; return; }
        bodyLen = pos;
    }
    if (rssi >= 0 || rssi < -110) {
        gE.rxNoRssi++;
        rssi = -70;            // the frame was decoded, so the link is at least the peer manager's edge (thanhnndev)
    }
    IOLockLock(gE.lock);
    gE.peerHeardUs = timeUs;
    gE.lastRssi = rssi;
    if (usableCh(channel)) gE.peerCh = (uint8_t)channel;
    trackPeer(frame + 10, body, bodyLen, timeUs);
    IOLockUnlock(gE.lock);

    IO80211Controller *ctl = vif->getController();
    void *pool = ctl ? mtkAwdlFamilyMgmtPool(ctl) : nullptr;
    if (!pool) { gE.rxAfFail++; return; }
    // IO80211BufferPool::getBuffer, IO80211Buffer::getCapacity, getBytesNoCopy, setLength, returnBuffer: virtual
    // methods at these offsets from the vtable pointer in this kernel's IO80211Family.
    typedef void *(*GetBuffer)(void *);
    typedef size_t (*GetCapacity)(void *);
    typedef void *(*GetBytes)(void *);
    typedef void (*SetLength)(void *, size_t);
    typedef void (*ReturnBuffer)(void *);
    void *buf = ((GetBuffer)(*(void ***)pool)[0x118 / 8])(pool);
    if (!buf) { gE.rxAfFail++; return; }
    uint8_t *bytes = (uint8_t *)((GetBytes)(*(void ***)buf)[0x158 / 8])(buf);
    if (!bytes || bodyLen > ((GetCapacity)(*(void ***)buf)[0x140 / 8])(buf)) {
        ((ReturnBuffer)(*(void ***)buf)[0x118 / 8])(buf);
        gE.rxAfFail++;
        return;
    }
    memcpy(bytes, body, bodyLen);
    ((SetLength)(*(void ***)buf)[0x138 / 8])(buf, bodyLen);
    ActFrmHdr hdr;
    bzero(&hdr, sizeof(hdr));
    memcpy(hdr.sa, frame + 10, 6);
    hdr.chVersion = 1;
    hdr.channel = channel;
    hdr.chFlags = chFlags(channel);
    hdr.rssi = rssi;
    hdr.rxTime = (uint32_t)timeUs;
    gE.rxAf++;
    mtkAwdlFamilyTapInput(vif, buf, kTapActionFrame, &hdr, sizeof(hdr));
}

// -----------------------------------------------------------------------------------------------------------------------
// The engine's entry points for the interface.
// -----------------------------------------------------------------------------------------------------------------------
static bool engineInit(MtkAwdlInterface *vif, const uint8_t *addr)
{
    if (gE.lock) return true;
    gE.lock = IOLockAlloc();
    if (!gE.lock) return false;
    gE.vif = vif;
    memcpy(gE.addr, addr, 6);
    gE.lastPostedAw = -1;
    gE.familyState = 1;
    gE.role = 1;
    gE.awdlCh = 0;
    // The queues' work loop: the data timer completes their packets, the timeline posts to the same Family.
    IOWorkLoop *wl = (IOWorkLoop *)vif->getWorkQueue();
    if (!wl) wl = vif->getWorkLoop();
    if (wl) {
        gE.tlTimer = IOTimerEventSource::timerEventSource(vif, tlTimerFired);
        gE.dpTimer = IOTimerEventSource::timerEventSource(vif, dpTimerFired);
        if (gE.tlTimer && wl->addEventSource(gE.tlTimer) != kIOReturnSuccess) OSSafeReleaseNULL(gE.tlTimer);
        if (gE.dpTimer && wl->addEventSource(gE.dpTimer) != kIOReturnSuccess) OSSafeReleaseNULL(gE.dpTimer);
        gE.wl = wl;
    }
    mtk_dbg_log("awdl: engine ready timers=%d/%d\n", gE.tlTimer != nullptr, gE.dpTimer != nullptr);
    return true;
}

static void engineEnable(bool on)
{
    if (!gE.lock) return;
    uint64_t now = mtkAwdlUptimeUs();
    IOLockLock(gE.lock);
    bool was = gE.enabled;
    gE.enabled = on;
    if (on && !gE.tlRunning) {
        gE.tlRunning = true;              // our own master until the Family elects another: AW 0 starts now
        gE.tlBaseUs = now;
        gE.tlBaseSeq = 0;
        gE.lastPostedAw = -1;
        gE.plannedUntilUs = 0;
        chooseChannel();
        buildPresence(now);
    }
    // Turning on is not a use. The Family turns AWDL on again after each firmware reload, with its idle sequence, and a
    // minute of windows nobody had asked for followed every reload: with nothing else the timeline stops at its first EAW.
    if (on && !was) gE.actUs = 0;
    if (!on) {
        gE.tlRunning = false;
        gE.endPending = false;
        gE.onceLen = 0;
    }
    if (on != was) {
        gE.still = false;
        gE.wakeReq = kWakeNone;
    }
    IOLockUnlock(gE.lock);
    if (on && !was) {
        int rv = mtkAwdlRadioStart(gE.addr);
        gE.radioOn = rv == 0;
        if (rv) mtk_dbg_log("awdl: the radio did not start (%d)\n", rv);
        armTimer(now);
    } else if (!on && was) {
        if (gE.tlTimer) gE.tlTimer->cancelTimeout();
        mtkAwdlRadioStop();
        gE.radioOn = false;
    }
}

// -----------------------------------------------------------------------------------------------------------------------
// The data path of awdl0 (thanhnndev ca94db4, bd5f763; seemoo-lab owl's framing). On the air an AWDL data frame has no DS
// bits, addr3 the AWDL BSSID, LLC/SNAP with Apple's OUI and protocol 0x0800, then 03 04, a sequence number, two pad bytes
// and the ethertype. Packets the radio cannot take stay in awdl0's queue: the submission action takes no more, and the
// queue is asked again once the radio made room.
// -----------------------------------------------------------------------------------------------------------------------
namespace {

void dpTimerFired(OSObject *owner, IOTimerEventSource *)
{
    if (owner) ((MtkAwdlInterface *)owner)->serviceData();
}

// Whether a unicast QoS data frame of `sa` goes up (RxWin). A window not used for 3 s, or older frames eight times in a
// row, start again from the frame at hand. Work loop.
bool acceptQos(const uint8_t *sa, uint8_t tid, uint16_t sn, uint8_t part, uint64_t now)
{
    RxWin *w = nullptr;
    for (RxWin &x : gE.win) if (x.valid && x.tid == tid && !memcmp(x.sa, sa, 6)) { w = &x; break; }
    if (part == kMtkAwdlMiddlePart || part == kMtkAwdlLastPart) {
        bool ok = w && w->cur == sn && w->curOk;
        if (ok) gE.rxParts++;
        else gE.rxPartsDropped++;
        return ok;
    }
    if (part == kMtkAwdlFirstPart) gE.rxParts++;
    bool ok = true;
    if (!w || now - w->lastUs > 3000000ULL) {
        if (!w) {
            w = &gE.win[gE.winNext++ % (sizeof(gE.win) / sizeof(gE.win[0]))];
            memcpy(w->sa, sa, 6);
            w->tid = tid;
            w->valid = true;
        } else {
            gE.rxQosRestart++;
        }
        w->hi = sn;
        w->seen = 1;
        w->old = 0;
    } else {
        uint16_t back = (uint16_t)((w->hi - sn) & 0xfff);
        if (back == 0 || (back < 64 && (w->seen & (1ULL << back)))) {
            ok = false;                                       // sent again: delivered already
            gE.rxQosDup++;
        } else if (back < 64) {
            w->seen |= 1ULL << back;                          // late, inside the window
            gE.rxQosLate++;
        } else if (back > 2048) {
            uint16_t ahead = (uint16_t)((sn - w->hi) & 0xfff); // newer: the window moves up to it
            w->seen = ahead >= 64 ? 1 : (w->seen << ahead) | 1;
            w->hi = sn;
            w->old = 0;
        } else if (++w->old >= 8) {
            w->hi = sn;
            w->seen = 1;
            w->old = 0;
            gE.rxQosRestart++;
        } else {
            ok = false;
            gE.rxQosOld++;
        }
    }
    w->lastUs = now;
    w->cur = sn;
    w->curOk = ok;
    if (ok) gE.rxQos++;
    return ok;
}

} // namespace

uint32_t MtkAwdlInterface::submitData(IOSkywalkPacket *const *packets, uint32_t count)
{
    static const uint8_t snap[8] = { 0xaa, 0xaa, 0x03, 0x00, 0x17, 0xf2, 0x08, 0x00 };
    uint32_t used = 0;
    if (!packets) return 0;
    if (!gE.dpOn) {
        // Offered before the path was on: they stay in awdl0's queue, which is asked again once it is. Without
        // this the queue waited for a dequeue request nobody sent, and awdl0 sent nothing for the whole boot.
        gE.txEarly++;
        gE.txBlocked = true;
        if (gE.dpTimer) gE.dpTimer->setTimeoutMS(5);
        return 0;
    }
    if (gE.still && gE.lock) {
        // A still timeline runs again at the first packet, which waits in awdl0's queue until the radio is back.
        IOLockLock(gE.lock);
        bool kick = noteActivity(mtkAwdlUptimeUs(), kWakeData);
        bool still = gE.still;
        IOLockUnlock(gE.lock);
        if (kick) kickTimeline();
        if (still) {
            gE.txBlocked = true;
            if (gE.dpTimer) gE.dpTimer->setTimeoutMS(20);
            return 0;
        }
    }
    for (; used < count; used++) {
        IOSkywalkPacket *p = packets[used];
        if (!p) break;
        if (gE.txDoneN >= sizeof(gE.txDone) / sizeof(gE.txDone[0])) { gE.txBlocked = true; break; }
        bool single = p->getPacketBufferCount() == 1;
        uint32_t len = p->getDataLength(), size = single ? p->getBufferSize() : 0;
        uint16_t off = single ? p->getDataOffset() : 0;
        const uint8_t *e = single ? (const uint8_t *)p->getDataVirtualAddress() : nullptr;
        if (e && len >= 14 && off <= size && len <= size - off && 40 + (len - 14) <= sizeof(gDataTx)) {
            e += off;
            uint8_t *f = gDataTx;
            bzero(f, 24);
            f[0] = 0x08;
            memcpy(f + 4, e, 6);
            memcpy(f + 10, gE.addr, 6);
            memcpy(f + 16, kBssid, 6);
            wr16(f + 22, (uint16_t)((gE.dataSeq & 0xfff) << 4));
            memcpy(f + 24, snap, 8);
            f[32] = 0x03; f[33] = 0x04;
            wr16(f + 34, gE.dataSeq);
            f[36] = 0; f[37] = 0;
            f[38] = e[12]; f[39] = e[13];
            memcpy(f + 40, e + 14, len - 14);
            int rv = mtkAwdlRadioQueue(f, 40 + (len - 14), 0, 500);
            if (rv == ENOBUFS) {
                gE.txFull++;
                gE.txBlocked = true;
                break;                         // this packet stays in the queue
            }
            gE.txIn++;
            gE.dataSeq++;
            if (rv == 0) {
                gE.txQueued++;
                gE.dataUs = mtkAwdlUptimeUs();
                if (!(e[0] & 1)) gE.ucUs = gE.dataUs;
            } else {
                gE.txBad++;
            }
        } else {
            gE.txBad++;
        }
        gE.txDone[gE.txDoneN++] = p;
    }
    if (gE.txDoneN && gE.dpTimer) gE.dpTimer->setTimeoutUS(1);
    return used;
}

// Completes the packets the radio copied, and asks awdl0's queue for more once there is room. Controller work loop.
void MtkAwdlInterface::serviceData()
{
    while (gE.txDoneN && awdlQueues[1]) {
        uint32_t n = gE.txDoneN < 32 ? gE.txDoneN : 32;
        if (((IOSkywalkTxCompletionQueue *)awdlQueues[1])->enqueuePackets(gE.txDone, n, 0) != kIOReturnSuccess) break;
        gE.txDoneN -= n;
        memmove(gE.txDone, gE.txDone + n, gE.txDoneN * sizeof(gE.txDone[0]));
    }
    if (gE.txDoneN && gE.dpTimer) gE.dpTimer->setTimeoutMS(2);
    if (gE.txBlocked && gE.dpOn && mtkAwdlRadioRoom() > 0 && gE.txDoneN < 32 && awdlQueues[0]) {
        gE.txBlocked = false;
        ((IOSkywalkTxSubmissionQueue *)awdlQueues[0])->requestDequeue(nullptr, 1);
    } else if (gE.txBlocked && gE.dpTimer && !gE.still) {
        // a full radio queue drains within a window; a radio not started yet may take seconds; a still timeline asks
        // again when it wakes
        gE.dpTimer->setTimeoutMS(gE.radioOn ? 5 : 100);
    }
}

// Air -> stack, from the receive tap: the 802.11 data frame becomes an Ethernet frame on awdl0.
void MtkAwdlInterface::deliverData(const uint8_t *f, uint32_t len, uint8_t part)
{
    gE.rxData++;
    if (!gE.dpOn || !awdlPool || !awdlQueues[2] || len < 24) { gE.rxDataFail++; return; }
    bool qos = (f[0] & 0x80) != 0;                          // a QoS data subtype: QoS control follows
    uint32_t hl = qos ? 26 : 24;
    if (qos && (f[1] & 0x80)) hl += 4;                     // QoS with HT control
    if ((f[1] & 0x47) || len < hl + 8) { gE.rxDataFail++; return; }   // no DS bits, no fragments, not protected
    const uint8_t *da = f + 4, *sa = f + 10;
    if (!(da[0] & 1) && memcmp(da, gE.addr, 6)) { gE.rxNotUs++; return; }
    if (!memcmp(sa, gE.addr, 6) || (sa[0] & 1)) { gE.rxNotUs++; return; }
    uint16_t sc = rd16(f + 22);
    uint64_t now = mtkAwdlUptimeUs();
    if (qos && !(da[0] & 1)) {
        // unicast QoS data, which a Block Ack agreement may bring out of order
        if (!acceptQos(sa, f[24] & 0x0f, sc >> 4, part, now)) return;
    } else {
        RxDup *d = nullptr;
        for (RxDup &x : gE.dup) if (x.valid && !memcmp(x.sa, sa, 6)) { d = &x; break; }
        if (d && d->sc == sc && (f[1] & 0x08)) { gE.rxDup++; return; }      // a retry of a frame already delivered
        if (!d) { d = &gE.dup[gE.dupNext++ % 8]; memcpy(d->sa, sa, 6); d->valid = true; }
        d->sc = sc;
    }
    const uint8_t *pl = f + hl;
    uint32_t plen = len - hl;
    gE.dataUs = now;
    if (!(da[0] & 1)) gE.ucUs = now;
    if (!(qos && (f[24] & 0x80))) {
        deliverMsdu(da, sa, pl, plen);
        return;
    }
    // An A-MSDU (QoS control bit 7): subframes of destination, source, big-endian length and an MSDU, each but the last
    // padded to 4 bytes; every MSDU carries its own LLC/SNAP and AWDL header (seemoo-lab owl, awdl_rx_data_amsdu). The
    // Pixel sends its bulk data this way: dropped whole, a transfer to the Mac stopped half way.
    gE.rxAmsdu++;
    while (plen >= 14) {
        const uint8_t *sda = pl, *ssa = pl + 6;
        uint32_t sl = ((uint32_t)pl[12] << 8) | pl[13];
        if (14 + sl > plen) { gE.rxDataFail++; return; }
        gE.rxSub++;
        deliverMsdu(sda, ssa, pl + 14, sl);
        uint32_t step = (14 + sl + 3) & ~3u;
        if (step >= plen) break;
        pl += step;
        plen -= step;
    }
}

// One MSDU of awdl0: LLC/SNAP with Apple's OUI, the AWDL data header (03 04, sequence, two pad bytes), the ethertype,
// then the packet, which goes up as an Ethernet frame.
void MtkAwdlInterface::deliverMsdu(const uint8_t *da, const uint8_t *sa, const uint8_t *pl, uint32_t plen)
{
    static const uint8_t outer[6] = { 0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00 };
    if (plen >= 8 && !memcmp(pl, outer, 6)) { pl += 8; plen -= 8; }   // an outer LLC header some peers add
    static const uint8_t snap[8] = { 0xaa, 0xaa, 0x03, 0x00, 0x17, 0xf2, 0x08, 0x00 };
    if (plen < 16 || memcmp(pl, snap, 8) || pl[8] != 0x03 || pl[9] != 0x04) { gE.rxDataFail++; return; }
    uint8_t et0 = pl[14], et1 = pl[15];
    const uint8_t *ip = pl + 16;
    uint32_t iplen = plen - 16;
    if (et0 == 0x86 && et1 == 0xdd && iplen >= 40) {                       // the FCS may follow: IPv6 knows its length
        uint32_t want = 40 + (((uint32_t)ip[4] << 8) | ip[5]);
        if (want <= iplen) iplen = want;
    }
    uint32_t elen = 14 + iplen;
    IOSkywalkPacket *p = nullptr;
    IOReturn r = awdlPool->allocatePacket(1, &p, 1);
    if (r != kIOReturnSuccess || !p) { gE.rxDataFail++; return; }
    r = p->prepareWithQueue(nullptr, 0, 0);
    uint8_t *e = (r == kIOReturnSuccess && p->getPacketBufferCount() == 1) ? (uint8_t *)p->getDataVirtualAddress() : nullptr;
    if (!e || p->getBufferSize() < elen) {
        r = kIOReturnNoSpace;
    } else {
        memcpy(e, da, 6);
        memcpy(e + 6, sa, 6);
        e[12] = et0;
        e[13] = et1;
        memcpy(e + 14, ip, iplen);
        r = p->setDataOffsetAndLength(0, elen);
        IOSkywalkNetworkPacket *np = static_cast<IOSkywalkNetworkPacket *>(p);
        if (r == kIOReturnSuccess) r = np->setHeadroom(0);
        if (r == kIOReturnSuccess) r = np->setLinkHeaderLength(14);
        bool bc = (da[0] & da[1] & da[2] & da[3] & da[4] & da[5]) == 0xff;
        if (r == kIOReturnSuccess && bc) r = np->setIsLinkBroadcast(true);
        else if (r == kIOReturnSuccess && (da[0] & 1)) r = np->setIsLinkMulticast(true);
        if (r == kIOReturnSuccess) r = np->setRxChecksumInfo(0, 0);
        if (r == kIOReturnSuccess) r = ((IOSkywalkRxCompletionQueue *)awdlQueues[2])->enqueuePackets(&p, 1, 0);
    }
    if (r != kIOReturnSuccess) {                 // nothing consumed it: the packet goes back to the pool
        awdlPool->deallocatePacket(p);
        if (++gE.rxDataFail <= 4) mtk_dbg_log("awdl: data frame not delivered 0x%x len %u\n", r, elen);
        return;
    }
    gE.rxDataUp++;
}

// The queues run once they are enabled: completions first, the submission last (its action uses the others).
void MtkAwdlInterface::enableData()
{
    if (gE.dpOn || !awdlQueuesOnLoop) return;
    awdlQueues[1]->enable();
    awdlQueues[2]->enable();
    gE.dpOn = true;                     // before the submission queue: its action can run as soon as it is enabled
    awdlQueues[0]->enable();
    mtk_dbg_log("awdl: data path on\n");
    if (gE.txBlocked && gE.dpTimer) gE.dpTimer->setTimeoutUS(1);
}

// -----------------------------------------------------------------------------------------------------------------------
// What the Family hands down, taken into the engine.
// -----------------------------------------------------------------------------------------------------------------------
// apple80211_awdl_sync_params: +0x10 the action frame period in TU.
static void engineSyncParams(const uint8_t *raw)
{
    if (!gE.lock) return;
    uint32_t tu = (uint32_t)raw[0x10] | ((uint32_t)raw[0x11] << 8);
    IOLockLock(gE.lock);
    gE.afPeriodTu = tu >= 16 && tu <= 1024 ? tu : 0;
    IOLockUnlock(gE.lock);
}

// apple80211_awdl_sync_channel_sequence: +5 the entries, from +0xc 12 bytes each, the channel at +4 of an entry.
//
// At rest the Family's sequence has 3 entries on AWDL channels and one at the access point's
// (112 0 149 0 0 0 0 0 6 0 149 0 0 0 0 0); while it has a browse or an advertisement going, 6 to 12 (149 149 149 149 0 0
// 112 112 6 149 149 149 0 0 112 112). Busy keeps the timeline running, and wakes it. Entries at the access point's channel
// alone do not count, unless the sequence is nearly full: they may be AWDL's own when the access point is on 149 or 44.
static void engineChanSeq(const uint8_t *s)
{
    if (!gE.lock) return;
    uint32_t n = s[5] > 16 ? 16 : s[5];
    if (!n) return;
    uint64_t now = mtkAwdlUptimeUs();
    IOLockLock(gE.lock);
    uint8_t home = mtkAwdlRadioHomeChannel();
    uint32_t used = 0, away = 0;
    for (uint32_t i = 0; i < 16; i++) {
        uint8_t c = s[0xc + 12 * (i % n) + 4];
        gE.famCh[i] = usableCh(c) ? c : 0;
        used += c != 0;
        away += c != 0 && c != home;
    }
    gE.famSet = true;
    gE.famBusy = away >= 5 || used >= 9;
    bool kick = gE.famBusy && noteActivity(now, kWakeSeq);
    chooseChannel();
    IOLockUnlock(gE.lock);
    if (kick) kickTimeline();
}

// apple80211_awdl_sync_state: +4 the state (1 master, 2 slave, 3 non-election master), +8 the master. The Family hands the
// elected master with state 1 whoever won: a master that is not us makes us its slave (thanhnndev 44d5bd2).
static void engineSyncState(uint32_t state, const uint8_t *m)
{
    if (!gE.lock || state < 1 || state > 3) return;
    IOLockLock(gE.lock);
    gE.familyState = state;
    uint32_t role = (!isZero6(m) && memcmp(m, gE.addr, 6)) ? 2 : state == 2 ? 1 : state;
    bool changed = role != gE.role || memcmp(m, gE.master, 6);
    gE.role = role;
    memcpy(gE.master, m, 6);
    if (changed && role == 2) {
        // take the master's clock now, from the last frame it was heard with
        const Peer *p = findPeer(m);
        if (p && gE.tlRunning) {
            gE.tlBaseUs = p->baseUs;
            gE.tlBaseSeq = p->baseSeq;
            gE.lastPostedAw = -1;
            gE.syncs++;
        }
        gE.masterHeardUs = 0;
    }
    IOLockUnlock(gE.lock);
    if (changed)
        mtk_dbg_log("awdl: role %u master %02x:%02x:%02x:%02x:%02x:%02x\n", role, m[0], m[1], m[2], m[3], m[4], m[5]);
}

static void engineTopMaster(const uint8_t *top)
{
    if (!gE.lock) return;
    IOLockLock(gE.lock);
    memcpy(gE.topMaster, top, 6);
    IOLockUnlock(gE.lock);
}

// A run of TLVs that ends exactly at its length.
static bool tlvsWellFormed(const uint8_t *t, uint32_t len)
{
    uint32_t off = 0;
    while (off + 3 <= len) {
        uint32_t l = rd16(t + off + 1);
        if (off + 3 + l > len) return false;
        off += 3 + l;
    }
    return off == len;
}

// apple80211_awdl_sync_frame_template: +4 the length, +8 the TLVs of the PSF. A template of a new length comes when the
// Family's services change (137 -> 139 bytes as AirDrop starts browsing or advertising); those it writes while it
// configures AWDL find the timeline running anyway.
static void engineTemplate(const uint8_t *tlv, uint32_t len)
{
    if (!gE.lock) return;
    bool ok = tlv && len <= kTlvMax && tlvsWellFormed(tlv, len);
    uint64_t now = mtkAwdlUptimeUs();
    IOLockLock(gE.lock);
    bool kick = ok && gE.psfLen && len != gE.psfLen && noteActivity(now, kWakeTemplate);
    if (ok) memcpy(gE.psf, tlv, len);
    gE.psfLen = ok ? len : 0;
    IOLockUnlock(gE.lock);
    if (kick) kickTimeline();
}

// A traffic registration with a peer.
static void engineTraffic(bool active)
{
    if (!gE.lock || !active) return;
    uint64_t now = mtkAwdlUptimeUs();
    IOLockLock(gE.lock);
    bool kick = noteActivity(now, kWakeTraffic);
    IOLockUnlock(gE.lock);
    if (kick) kickTimeline();
}

// apple80211_awdl_oob_request: +0x16 the destination, +0x28 the payload length, +0x30 the payload (the master
// indication's TLVs for the automatic request; thanhnndev d4b9008, ForbiddenItlwm storeMasterIndication). Only a kernel
// buffer of plausible size whose TLVs end where it ends is kept.
static IOReturn engineOob(const uint8_t *r, bool autoReq)
{
    if (!gE.lock || !r) return kIOReturnBadArgument;
    uint32_t len = (uint32_t)r[0x28] | ((uint32_t)r[0x29] << 8);
    const uint8_t *pl = *(const uint8_t *const *)(r + 0x30);
    if (!len || !pl) {
        IOLockLock(gE.lock);
        if (autoReq) gE.miLen = 0; else gE.onceLen = 0;
        IOLockUnlock(gE.lock);
        return kIOReturnSuccess;
    }
    if ((uintptr_t)pl < 0xffffff8000000000ULL) return kIOReturnBadArgument;
    if (len > 16 && pl[0] == 0x7f && pl[1] == 0x00 && pl[2] == 0x17 && pl[3] == 0xf2 && pl[4] == 0x08) { pl += 16; len -= 16; }
    if (len > kTlvMax) return kIOReturnNoSpace;
    if (!tlvsWellFormed(pl, len)) return kIOReturnBadArgument;
    const uint8_t *a = r + 0x16;
    uint64_t now = mtkAwdlUptimeUs();
    bool kick = false;
    IOLockLock(gE.lock);
    if (autoReq) {
        memcpy(gE.mi, pl, len);
        gE.miLen = len;
        if ((gE.ownCapsLogged & 3) != 3) gE.ownCapsLogged |= logCapTlvs("our", pl, len, gE.ownCapsLogged);
    } else {
        memcpy(gE.once, pl, len);
        gE.onceLen = len;
        if (isZero6(a) || (a[0] & 1) || !memcmp(a, gE.addr, 6)) memset(gE.onceDa, 0xff, 6);
        else memcpy(gE.onceDa, a, 6);
        kick = noteActivity(now, kWakeFrame);          // the frame goes out in the next present EAW
    }
    IOLockUnlock(gE.lock);
    if (kick) kickTimeline();
    return kIOReturnSuccess;
}
