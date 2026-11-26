# CLAUDE.md

Guidance for working on SimpleMtkWlan, a macOS kext for MediaTek MT7921 / MT7921K
(RZ608) / MT7922 PCIe Wi-Fi.

## Build

```sh
git submodule update --init            # MacKernelSDK (kernel headers + libkmod)
xcodebuild -project SimpleMtkWlan.xcodeproj -scheme SimpleMtkWlan \
  -configuration Debug -derivedDataPath build CODE_SIGNING_ALLOWED=NO build
```

- **Clean build (`rm -rf build`) after touching any header in `mtk80211/` or
  `include/`.** Those directories are in `SYSTEM_HEADER_SEARCH_PATHS`, so Xcode
  does not track them and silently links stale objects (this once left all of
  `net80211` logging to `kprintf` and risked mismatched vtables). Editing only
  `.cpp` files is safe incrementally.
- Check the result links against the running kernel:
  `kmutil libraries -p build/Build/Products/Debug/SimpleMtkWlan.kext --undef-symbols`
  must report no unresolved symbols.
- AirportMtkWlan (native Wi-Fi) is built by `./build-macos.sh <Debug|Release>
  AirportMtkWlan` (`AIRPORT_OS=tahoe` default, or `sequoia`) into
  `build/<Conf>/<os>/`; the script always rebuilds every object. The Xcode
  project only builds SimpleMtkWlan. Check both kexts with `kmutil libraries`.
- `MODULE_VERSION` must stay purely numeric (`1.2.3`); a suffix makes the kernel
  reject the kext. CI derives it from `vX.Y.Z` tags.

## Tests

`./tests/run-all.sh` runs all host tests. They slice real functions out of the
sources (by signature string) and compile them with stub headers under
ASan/UBSan. When you change a function a test extracts, update that test's
stubs (new fields, structs or helper calls) rather than weakening its asserts.

## Layout

- `SimpleMtkWlan/SimpleMtkWlan.cpp`: `IOEthernetController` front end, work loop,
  watchdog (SimpleMtkWlan.kext only).
- `SimpleMtkWlan/MtkLog.cpp`: persistent debug log (`mtk_dbg_log` →
  `MtkDebugLog` property on the PCI device), used by both kexts.
- `AirportMtkWlan/`: IO80211/Skywalk front end (AirportItlwm WCL from
  laobamac/itlwm via pengubow). `IOPCIEDeviceWrapper` matches the card and
  creates `MtkMwx`; `AirportItlwmV2.cpp` is the `IO80211Controller`;
  `AirportItlwmSkywalkInterface.cpp` + `WCL/WCL.cpp` answer airportd's
  requests (scan, join, link state). Everything for it is under `AIRPORT` /
  `AIRPORT_WCL`; SimpleMtkWlan.kext must stay unaffected.
- `include/Airport/macOS15|macOS26`: per-OS IO80211 vtable layouts, chosen by
  `__IO80211_TARGET`. A wrong layout panics at boot, so a kext built for one
  macOS must never load on the other (OpenCore Min/MaxKernel).
- `SimpleMtkWlan/MtkWlanUserClient.cpp`: ioctl-style user client for SimpleMTK.
- `SimpleMtkWlan/hal_mwx/MtkMwx.cpp`: the hardware layer (OpenBSD `mwx` port):
  DMA rings, MCU commands, firmware load, RX/TX, STA/BSS records, keys, BA.
- `SimpleMtkWlan/hal_mwx/if_mwxreg.h`: registers, MCU TLV structs and tags
  (tags follow Linux `mt76_connac_mcu.h`).
- `mtk80211/openbsd/net80211/`: 802.11 stack (state machine, EAPOL, crypto, BA
  reordering). `mtk80211/compat.cpp`: bus_dma, interrupts, timeouts glue.
- `include/FwBinary.cpp`: embedded firmware (MT7961 = MT7921/RZ608, MT7922);
  MT7920 comes from `include/FwMT7920.inc`, generated, not in Git.

## Logging and debugging

- `XYLog` / `printf` in the HAL / `MWX_DEV_LOG` all go to `mtk_dbg_log`, a ring
  buffer published as `MtkDebugLog` on the PCI provider. Read it with
  `tools/mtklog.sh`. Early boot output is otherwise lost (unified log
  overflows, `dmesg` is overwritten, kprintf goes to serial).
- **Never log per packet.** Rate-limit with a static counter like the existing
  code. Per-packet logging visibly cut throughput and evicted useful history.
- `MtkMwx::debugPoll()` runs from the watchdog every second on the main gate:
  periodic `poll`/`stats` lines, TX stall descriptor dump, automatic hard reset
  (firmware reload) if a DMA ring stops moving, or if the data RX ring stays
  silent while frames go out: 6 s for an associated station that is not
  scanning (its access point's beacons come ten times a second), 30 s
  otherwise (without a link: while scans end). A station whose reception
  stopped while it was associated counts as associated until frames come
  again, also after it lost the link to the stall: it is reloaded as soon as
  its stall is seen, three times in a row, then a minute apart. Otherwise
  reloads are at least a minute apart, growing to 8 minutes while it keeps
  coming back. The second stall in a row also stops the AWDL radio's windows
  for ten minutes. The count starts again after ten minutes without a reload.
  The stall's log gives when the last frame came off the data RX ring, and the
  firmware commands and remain-on-channel events from 2 s before it to 1 s
  after (`mwx_mcu_trace_log`), then what the AWDL radio was doing. The ring
  holds about ten minutes while AWDL runs, so every hard-reset report also
  goes to `MtkStallLog` (`mtk_dbg_keep`), 16 KB kept until the next boot.
  Wi-Fi off and on keeps the firmware and does not recover a deaf chip.
- In zsh, `log` is a builtin; use `/usr/bin/log`.

## Hardware lessons (do not regress)

- Firmware is sent in chunks of `4096 - sizeof(mwx_mcu_txd)` so each message
  fits a 4 KB cluster; 16 KB jumbo clusters are not available at boot.
- Interrupts: register the `kIOInterruptTypePCIMessaged` index; never set the
  MSI enable bit by hand. `mwx_intr` re-reads status before re-enabling.
- TX frames made of several mbufs are copied into one cluster
  (`mwx_tx_linearize`); otherwise the WFDMA stalls on the first DHCP frame.
- Group-addressed frames are not hardware-decrypted (no WTBL match,
  `wlan_idx=1023`); keys are also loaded for net80211 software CCMP. net80211
  never software-encrypts TX, so this only affects RX.
- When leaving RUN, disable the BSS and remove the peer STA record in firmware;
  reusing wcid 1 on top of the stale record breaks hardware decryption.
- Before switching APs, flush the queued deauth on the old channel.
- RSSI-triggered roaming is disabled (`ieee80211_bgscan_timeout` is a no-op);
  background scans in RUN are throttled to one per 120 s.
- MCU commands sent from RX/interrupt context (BA setup) must not wait for the
  response; the response arrives through the same interrupt path.
- AWDL remain-on-channel: the grant is waited for up to 1 s before the abort,
  as `mt7921_set_roc` does. The first grant for 149 after the firmware starts
  takes over 300 ms, and an abort sent before a grant that is still coming is
  what the MT7925 firmware did not survive (mt7925-macos-wifi). After the
  abort the sniffer and the RX filter change only 20 ms later, once the radio
  is home: the data RX ring stopped for good when they changed in the same
  millisecond as the abort. It stopped as well 20 ms later, at the sniffer's
  switch-off and the station's filter coming back, about once in a hundred
  windows. So while the timeline runs and the station is associated, the
  reception stays open between windows, and the sniffer only moves back to the
  station's channel, as Linux moves the sniffer of a monitor
  (`mt7921_change_chanctx`). It closes when the timeline stops, when the
  station scans or leaves its network (`mwx_awdl_rx_release`), and when a peer
  has a record on AWDL's BSS. `hold=` in the radio line counts the windows that
  left it open and the closes.
- Only advertise VHT to the firmware (`PHY_TYPE_BIT_VHT`) once VHT is
  negotiated. The operating width comes from the association
  (`mwx_node_chan_width`); the channel is switched to 40/80 MHz on RUN.
- Advertise HT `CBW20_40` in the HT caps even when running VHT80: APs only
  transmit 80 MHz to clients that claim 40 MHz support.
- net80211 sends AUTH/ASSOC_REQ once and waits 5 s; `debugPoll()` retransmits
  them up to 3 times. The chip stays at full power from AUTH to RUN: with
  deep sleep allowed in ASSOC, the first association response was lost at
  every 5 GHz join, and an access point that negotiated 802.11w refuses the
  request sent again.
- Scan completions that arrive in RUN without `F_BGSCAN` are stale and must
  be ignored (they caused re-joins). Prefer 5 GHz within 15 RSSI points.
- BAR frames are not delivered by the firmware (`bar_rx=0` in the `ba` debug
  line); reorder gaps rely on the 100 ms `IEEE80211_BA_GAP_TIMEOUT`.
- A warm reboot keeps the M.2 card powered; attach recovers via WFSYS reset and
  PCIe FLR. When the patch semaphore answers `PATCH_IS_DL` the ROM patch is
  skipped and only the WM firmware is loaded, as in Linux.
- PCIe link power: `mwx_pci_disable_aspm()` clears ASPM L0s/L1 and then the
  ASPM L1.1/L1.2 enables at attach, the card before its upstream port, as
  Linux does at probe (`mt76_pci_disable_aspm`, `pci_disable_link_state`).
  The PCI-PM L1.1/L1.2 enables stay as the system firmware set them: PCI-PM
  L1 only happens with the card in D3, where they let the link reach L1.2
  during an S0 sleep. The `aspm(ep)` and `aspm(parent)` log lines show each
  register that changed; `-mtkallowaspm` keeps the system firmware's setting.
- Restart / power off: `AirportItlwm::systemWillShutdown()` takes the
  interface down like a system sleep, then `MtkMwx::shutdown()` parks the chip
  through `suspend()` and turns bus mastering off. No MCU command after the
  interface stop.
- TX: data uses the LMAC queue of its AC (`mt7921_mac_write_txwi`), management
  stays on VO. No `MT_TXD5_TX_STATUS_HOST`; TXWIs are released only by the TX
  free event. `mwx_start` sets oactive when `mwx_tx_has_room` fails and
  `mwx_tx_kick` (from `mwx_mac_tx_free`) restarts output and calls
  `signalOutputThread()` — never drop packets for a full ring.
- TX A-MSDU: `IEEE80211_C_TX_AMSDU_IN_AMPDU` makes net80211 offer it in ADDBA
  and keep the AP's answer in `ba_params`; the HAL then sets
  `MT_TXD7_HW_AMSDU` and STA_REC_BA `amsdu`, with `STA_REC_HW_AMSDU` in the
  peer record. `-mtknoamsdu` disables it.
- Sleep/wake: the card loses power in S3; in an S0 sleep it keeps it and the
  system may put it in D3. Front ends call
  `MtkHalService::suspend()` / `resume()` from their power state handlers.
  suspend leaves the network and parks the chip as Linux
  `mt7921_pci_suspend()` does before D3 (`mwx_park`: interrupts masked, DMA
  off, firmware own; no MCU command waits from then on), so that it can idle
  and its link reach L1.2 (`suspend: chip parked lpctl=` in the log, bit 2
  set when the firmware owns it). resume does driver own (WFSYS reset
  fallback — needed in practice) and
  reloads the firmware through the hard reset task. While `sc_suspended`,
  `mwx_intr` and `debugPoll` must not touch the hardware. AirportItlwm takes
  its interface down for the sleep; `resume(true)` marks it up again when it
  queues the reset task, so that the driver is available in the dark wake
  instead of only after airportd powers the radio on at the user wake.
- Host tests use ASan; on the hackintosh dev machine any ASan binary dies with
  SIGILL, so run them there with `-fsanitize=undefined` only (CI runs ASan).

## Native Wi-Fi (AirportMtkWlan) notes

- macOS joins by itself: `WCL.cpp` writes the PMK (or a passphrase, derived
  with PBKDF2) into `ic_psk` and net80211 still runs the 4-way handshake.
  Our key, BA and HT/VHT code paths are shared with SimpleMtkWlan.
- airportd asks for short scans of a few channels; `mt7921_mcu_hw_scan`
  honours `ic_wcl_scan_*` (channels, SSID, active). In RUN such restricted
  scans are allowed every 10 s per band, full sweeps every 120 s
  (`mwx_bgscan`): airportd asks for 2.4 GHz and then at once for 5 GHz, and
  with one count for both bands the 5 GHz half was always refused as busy.
  Background scans pass a dwell time of 0, the firmware's own, as Linux mt76
  does; scans without an association dwell 120 ms per channel. macOS's
  passive scans (type 2, auto-join) probe with the wildcard SSID too when
  every channel they cover is 1 to 11 or 36 to 48
  (`AirportWCL::passiveScanMayProbe`): the firmware is told no DFS flags
  (`mt7921_mcu_set_channel_domain`) and one request has one scan type.
  `active=` in the `scan request` line of the log says which it was.
  Outside RUN net80211 does not sweep on its own (`ic_wcl_scan_idle`), and
  `mwx_init` brings the interface up idle in the scan state: a sweep of its
  own made the first scan or join after power on, wake or a private address
  change wait 5.6 s for it. Leaving a network with nothing asked for and
  nothing scanning (`mwx_leave_unasked`) enters the same idle state
  (`mwx_scan_idle`, `leave: scan state idle until macOS asks` in the log):
  the sweep it used to start made the next join wait for it and was still
  running when a system sleep parked the chip.
- A join (`startWCLJoin`) first listens 220 ms on the channel of the
  request's chanspec only (`ic_wcl_join_channel`, spent by the first
  non-background scan); if the BSS is not heard there, net80211 sweeps every
  channel. Before, every join swept 41 channels (5.6 s) first.
- `mwx_init` adopts the MAC address macOS sets (private Wi-Fi address) and
  registers the firmware vif with it, otherwise DHCP replies are filtered.
- `IO80211InterfaceRole` must read `Infrastructure` or airportd ignores the
  interface; the front end puts it back after start.
- The front end's watchdog calls `MtkMwx::debugPoll()` through the gate,
  like SimpleMtkWlan. Do not set the MSI enable bit in `IOPCIEDeviceWrapper`.
- HAL sleeps (`tsleep_nsec`) go through `IOCommandGate::commandSleep` in the
  AIRPORT build: `enable()` runs with the gate held, and the interrupt that
  completes an MCU command needs that gate.
- No AppleVTD support (needs `DisableIoMapper`). WPA3-Personal (SAE) is on by
  default, `-mtkwpa3off` turns it off; see its section.
- AWDL (Tahoe only, on unless `-mtkawdloff`, and only when the kernel's
  IO80211 classes match the layouts in `awdl/io80211`): `AirportMtkWlan/awdl/` creates `awdl0` as a subclass
  of `IO80211AWDLProtocol`, answers the AWDL peer manager's configure handshake
  and runs AWDL on the radio; AirDrop works both ways with a Pixel's Quick Share.
  The hardware layer (`MtkMwx.cpp`, under `MTK_AWDL`) takes remain-on-channel
  windows, opens the sniffer and the RX filter for other devices (at 80 MHz on
  44 and 149; on the station's channel not while a peer has a record on AWDL's
  BSS, which carries AWDL's BSSID) and, for an associated station, leaves them
  open between windows while the timeline runs, sends on ALTX0 from OMAC 1, which holds
  awdl0's address (unicast
  data from 6 to 54 Mb/s, then on 5 GHz at VHT as A-MSDUs of up to 3839 bytes:
  on the station's channel at 80 MHz up to VHT80 two streams MCS 9, in the
  other windows at 20 MHz up to two streams MCS 8, `mtkawdlvht20=0` keeping
  OFDM there; `vht=` and `vht20=` in `awdl: radio ba`), and touches nothing
  until awdl0 starts it. The interface keeps the AWDL timeline,
  posts the availability window events, builds the synchronization and master
  indication frames, hands received action frames to the Family's peer manager
  and carries awdl0's data, A-MSDUs included. Block Ack: a peer in an agreement
  gets a station record in the firmware, on a BSS of AWDL's own on OMAC 1 (a P2P
  device that is not connected; WCIDs 15-17 for the peers, 18 for the BSS's
  group record). With the three taken, a record goes only from a peer quiet for
  3 s, after a DELBA for each of its agreements; a peer moving data keeps its
  own and the newcomer stays on the single-frame path. A peer's ADDBA request is
  answered once the receive agreement is in the WTBL and STA_REC_BA, so the MAC
  answers its A-MPDUs; the hardware
  then splits its A-MSDUs and the tap removes the two alignment bytes, and the
  interface keeps a 64-frame duplicate window per sender and TID (no
  reordering). To a peer we send many unicast frames to on the station's
  channel we ask for an agreement ourselves: its data then goes as QoS TID 0 in
  the station's best-effort queue under the peer's record, at HT MCS 4-7, and
  the MAC aggregates it. Away from the station's channel, or when the first
  aggregated frames do not come back, frames take the single-frame path on
  ALTX0. `-mtkawdlnoba` leaves all of it out; `awdl: radio ba` and
  `awdl: rx qos` in the log count it. A Block Ack action with the Order bit
  set carries an HT Control field before its category; the tap leaves the
  field out (`htc=` counts those frames). Group-addressed AWDL frames (PSF and MIF
  broadcast, mDNS multicast) go out with `MT_TXD3_NO_ACK`, as mt76 sets it for
  every frame mac80211 sends to a group address 1 (on the MT7925 it is
  `MT_TXD3_BCM`; bit 4 here is `MT_TXD3_DAS`): `mtkawdlnoack=<0-3>` picks the
  classes, bit 0 management and bit 1 data, 3 by default, 0 as before; `grpst=`
  and `tries=` in the radio line count those frames as the MAC reports them.
  With the access point on the AWDL channel
  the schedule covers every EAW while AWDL is in use (the Family keeps its idle
  sequence, 3 EAWs in 16, during a transfer), unless the device unicast data
  last came from or went to is on AWDL elsewhere. Off it, the EAWs left to the
  access point are those of Apple's rows by state (`kHome*` in the interface:
  AWDL data in the last 3 s, and the access point idle, with data or streaming
  as Apple rates it from the station's frames); a run away is one
  remain-on-channel period and the radio stays at least 20 ms home between two.
  Until a peer is heard the schedule visits the social channels 6, 44 and 149;
  a device that names itself master is
  followed. The device data last came from or went to sets the channels while
  it is heard, up to 5 minutes after that data, before the master's (an
  AirDrop request waits for its answer in between): a Pixel stays on 44 in
  every EAW, an Apple master may be there in one EAW of 16, and an access
  point on 36 fills most of the Family's low power sequence. Otherwise a
  device heard in the last 8 s that is on one channel in every entry of its
  sequence (a Pixel's Quick Share; no Apple device so far) sets them before
  the master's (`awdl: schedule on <ch>` in the log). Each unicast
  frame of awdl0 also goes to the Family's peer manager as its last link
  activity each way (`setLastUnicastTxLinkActivity`/`Rx`, exported), which
  keeps the Family out of its low power schedule while data moves. With the
  Family on a sparse sequence (6 slots in 16 or fewer), no data peer and no
  unicast data or request of the stack for 5 s, the schedule leaves the access
  point's channel once a sequence at most, each run away in turn (the idle
  cap; `cap=` in the `tl` line, `mtkawdlidlecap=0` turns it off). After 60 s
  with no peer, no data and nothing asked by the Family, the
  timeline stops and the radio goes back to the station, as with awdl0 down; the
  Family's next browse or advertisement (a denser channel sequence, a template of
  a new length), a frame or traffic request, or a packet of awdl0 starts it
  again from the next EAW. Turning AWDL on is not a use: after a firmware
  reload the Family turns it on with its idle sequence, and the timeline stays
  still. For two minutes after a reload for a reception stall
  (`mtkAwdlRadioResting()`) the Family's sequence and templates do not start
  it either; a frame or traffic request, a peer or data still do. After the
  second stall in a row the radio takes no window at all for ten minutes
  (`silent=` in the radio line). The driver
  writes awdl0's
  `IO80211VirtualInterfaceRole` itself (the Family cannot store it on these
  classes, and wifip2pd drops an interface
  without it). An AWDL test needs the driver log, which only `-mtkdbglog`
  publishes (it holds kernel pointers); `tools/mtklog.sh | grep awdl` shows
  the radio, the windows by channel and the peers. The interface file is compiled
  on its own against the layouts in `awdl/io80211`; struct sizes come from the
  request lengths the Family's ioctl handlers check, never from a guess.
  With AWDL on the station takes the access point's country (its Country
  element, as 802.11d) on entering RUN, before the link comes up, and tells
  the front end, which reports it as the card's country code: the Family's
  AWDL peer manager picks its channels from that country whenever the link
  changes (with the world domain "00" it takes 149, which a Pixel never visits;
  with IT it takes 44).
  `mtkawdlsched=<mask>` (0 by default, 15 for all) turns on the parts of
  ForbiddenItlwm's AWDL scheduler that apply here (`kMtkAwdlSched*` in
  `awdl/MtkAwdl.h`). Bit 0: the radio follows the Family's channel plan in
  every EAW and advertises it, instead of the schedule built from the master's
  or the data peer's sequence; while unicast data moved in the last 200 ms a
  free EAW after a run away joins it, up to 6. Bit 1: once a second the
  watchdog hands the station's unicast frames with its access point to the
  infrastructure interface's monitor (`recordInputPacket`,
  `recordOutputPacket`), the counters the Family's AWDL scheduler weighs the
  access point's traffic from. Bit 2: every unicast packet awdl0 delivers or
  queues adds one to the peer manager's received or sent count, which the
  Family's AWDL monitors read (`monitorAWDLDataMode`); where those counts are
  is read once from the Family's getters `getUnicastPacketsInputCount` and
  `getUnicastPacketsOutputCount`, and with any other code there nothing is
  counted. Bit 3: after the master indication's content changes (its TLVs but
  the timing ones), multicast data waits up to 300 ms for the first indication
  that carries it, and that frame goes ahead of the PSF. `awdl: sched` in the
  log counts all four.

## Management frame protection (802.11w)

- Negotiated with access points that advertise it (`IEEE80211_C_MFP`, on with
  WPA3 by default; `-mtkwpa3off` turns both off, `-mtkpmf` then offers it
  alone; `-mtkpmfreq` requires it). Towards the others the RSN element
  is byte for byte the one sent without it. The capabilities reported to
  macOS do not change: its association code reads 802.11w support from
  capability byte 6 for the RSN element it builds itself, of which the front
  end takes the group cipher alone. Capability 75 (byte 9, bit 3) is tested
  before SAE: set unless `-mtkwpa3off`. Capability 76 (byte 9, bit 4) is
  tested before a GCMP group cipher, which the driver does not implement:
  keep it clear.
- The integrity group key stays in software (`mwx_set_key`). Never send it to
  the firmware: a key record with BIP alone replaces the CCMP key of that
  station, and its delete command removes the pairwise key. net80211 verifies
  the MIC of group-addressed management frames itself (BIP).
- Robust management frames to and from the access point use the pairwise key
  in hardware like data: `MT_TXD3_PROTECT_FRAME` follows the Protected bit
  net80211 sets once `IEEE80211_NODE_TXMGMTPROT` is on, and
  `mwx_rx_hwdec_restore` puts the CCMP header back so that net80211 checks
  the packet number (`k_mgmt_rsc`, restarted with every new pairwise key).
- `IEEE80211_NODE_RXMGMTPROT` is set with the pairwise key of message 3,
  `TXMGMTPROT` when the firmware has the keys. Both end in
  `mwx_mgmt_prot_leave`, which also sends the Deauthentication while the
  firmware can still encrypt it: a RUN -> SCAN transition does not go through
  net80211's own exit from RUN.
- An unprotected Deauthentication or Disassociation is dropped. With reason
  6 or 7 it starts an SA Query (`ieee80211_sta_sa_query_start`): no answer
  for 1000 TU means the access point really lost the association.
- An access point that still holds a protected association of ours (its
  answer to our first association request got lost, or we restarted) refuses
  a new request for now (status 30) and checks the old association with an
  SA Query. net80211 asks again once the comeback time of the Timeout
  Interval element has passed (`ieee80211_sta_assoc_comeback`, three times a
  join; `debugPoll()` does not retransmit meanwhile). A Deauthentication with
  reason 6 or 7 while associating makes it authenticate again (twice a join)
  without telling the front end.
- The `pmf` debug line (with the `stats` line) shows what the access point
  advertises (`ap`: 1 capable, 2 required), what was negotiated, and counts
  accepted, unprotected and failing frames, the SA Queries, the associations
  refused for now (`comeback=`) and the authentications sent again
  (`reauth=`).
  `tests/security/` runs the production code against references written from
  the standard.

## WPA3-Personal (SAE)

- On by default (`ic_wpa3`), off with `-mtkwpa3off`; it also sets
  `IEEE80211_C_MFP`. The
  front end then reports byte 6 bit 0 and capability 75, shows macOS the
  access point's real AKMs where its RSN Extension element has the
  hash-to-element bit (`advertisesSAEH2E`), and takes the SAE join airportd
  sends: authentication type 1 << 12, the password itself (cipher 10, up to
  63 bytes; the request's key field holds 64), from which
  `ieee80211_sae_password_to_pt` derives the password token.
- Hash-to-element only, group 19 (P-256): `net80211/ieee80211_sae.c` (OpenBSD)
  over BearSSL's `crypto/i31` and `crypto/ec_p256_m31`. The password element
  comes in `ieee80211_node_join_bss`, the commit and confirm in
  `ieee80211_get_auth`, the peer's in `ieee80211_auth_sae`; the PMK goes to
  the 4-way handshake, which uses descriptor version 0 (AES-CMAC MIC, key
  wrap) and echoes the RSN Extension element in message 2. Message 3 must
  carry the IGTK and the same RSN Extension element as the beacon.
- A failed exchange tries another access point of the network before a new
  scan. Authenticating again while associating (an access point that dropped
  us) starts the exchange over with a new commit (`ieee80211_sae_restart`).
- The deepest SAE call chain (shared secret from the receive path) needs
  about 3 KB of stack, measured with `-fstack-usage`.

## Testing on hardware

The kext is injected by OpenCore (`EFI/OC/Kexts` + `Kernel → Add`), so every
test needs a reboot. On the development machine the EFI partition's disk
number changes between boots: find the 1.2 GB `EFI` partition with
`diskutil list` and verify `EFI/OC/OpenCore.efi` exists before writing to it.
Keep a known-good kext around for rollback. Only one of SimpleMtkWlan.kext /
AirportMtkWlan.kext may be enabled.
