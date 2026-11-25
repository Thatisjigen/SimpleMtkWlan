//
//  MtkMwx.cpp
//  SimpleMtkWlan
//
//  Created by laobamac (王孝慈) on 2026/5/5.
//  Copyright © 2026 laobamac (王孝慈). All rights reserved.
//

/*	$OpenBSD: if_mwx.c,v 1.9 2026/05/09 12:13:15 claudio Exp $ */
/*
 * Copyright (c) 2022 Claudio Jeker <claudio@openbsd.org>
 * Copyright (c) 2021 MediaTek Inc.
 * Copyright (c) 2021 Lorenzo Bianconi <lorenzo.bianconi83@gmail.com>
 * Copyright (c) 2017 Stefan Sperling <stsp@openbsd.org>
 * Copyright (c) 2016 Felix Fietkau <nbd@nbd.name>
 * Copyright (c) 2007-2010 Damien Bergamini <damien.bergamini@free.fr>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "MtkMwx.hpp"
#ifdef AIRPORT_WCL
#include "WCL/WCL.hpp"
#endif

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/bitfield.h>
#include <sys/kpi_mbuf.h>
#include <IOKit/IOLocks.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <sys/_if_ether.h>
#include <sys/_if_media.h>
#include <sys/_malloc.h>
#include <sys/_mbuf.h>
#include <sys/_task.h>
#include <net/ethernet.h>
#include <net80211/ieee80211_proto.h>
#include <net80211/ieee80211_radiotap.h>
#include <FwData.h>

#include "if_mwxreg.h"
#include "RxAmpduStats.hpp"
#include "ApCapabilitySummary.hpp"
#include "RxAmsduMetadata.hpp"
#include "RxAmsduReassembly.hpp"

OSDefineMetaClassAndStructors(MtkMwx, MtkHalService)

#ifndef NBPFILTER
#define NBPFILTER 0
#endif

#define PCI_VENDOR_MEDIATEK            0x14c3
#define PCI_PRODUCT_MEDIATEK_MT7920    0x7920
#define PCI_PRODUCT_MEDIATEK_MT7921    0x7961
#define PCI_PRODUCT_MEDIATEK_MT7921K   0x0608
#define PCI_PRODUCT_MEDIATEK_MT7922    0x7922
#define PCI_PRODUCT_MEDIATEK_MT7922_PCIE 0x0616	/* MT7922 as PCIe endpoint, AMD RZ616 */

/*
 * RX rate histogram from the P-RXV (RXD group 3) of unicast frames, to see
 * which rates the AP actually uses. Field layout from Linux mt76_connac2_mac.h.
 */
#define MWX_PRXV_TX_RATE(v)	((v) & 0x7f)
#define MWX_PRXV_NSTS(v)	(((v) >> 7) & 0x7)
#define MWX_PRXV_LDPC(v)	(((v) >> 11) & 0x1)
#define MWX_PRXV_FRAME_MODE(v)	(((v) >> 12) & 0x7)
#define MWX_PRXV_SGI(v)		(((v) >> 15) & 0x3)
#define MWX_PRXV_STBC(v)	(((v) >> 22) & 0x3)
#define MWX_PRXV_TX_MODE(v)	(((v) >> 24) & 0xf)
#define MWX_HE_RATE_MCS_MASK	0x0f
#define MWX_HE_RATE_DCM		0x80
static uint32_t mwx_rxr_mode[16], mwx_rxr_bw[4], mwx_rxr_nss[4], mwx_rxr_mcs[16];
static uint32_t mwx_rxr_sgi, mwx_rxr_stbc, mwx_rxr_ldpc, mwx_rxr_total;
static uint32_t mwx_rxr_he_gi[4];
/* Last unicast RX rate: the firmware does TX rate control and reports none. */
static int mwx_rxr_last_mcs = -1;
static uint32_t mwx_rxr_last_kbps;

/* HT/VHT 1SS long GI data rates in 100 kbit/s, by bandwidth and MCS. */
static const uint16_t mwx_vht_rate[4][10] = {
	{ 65, 130, 195, 260, 390, 520, 585, 650, 780, 867 },
	{ 135, 270, 405, 540, 810, 1080, 1215, 1350, 1620, 1800 },
	{ 293, 585, 878, 1170, 1755, 2340, 2633, 2925, 3510, 3900 },
	{ 585, 1170, 1755, 2340, 3510, 4680, 5265, 5850, 7020, 7800 },
};

static void
mwx_rx_rate_account(uint32_t v0)
{
	uint32_t mode = MWX_PRXV_TX_MODE(v0), idx = MWX_PRXV_TX_RATE(v0);
	uint32_t nss = MWX_PRXV_NSTS(v0) + 1;

	mwx_rxr_total++;
	mwx_rxr_mode[mode]++;
	mwx_rxr_bw[MWX_PRXV_FRAME_MODE(v0) & 3]++;
	if (mode == 2 || mode == 3) {		/* HT: idx = MCS 0-15 */
		nss = (idx >> 3) + 1;
		idx &= 7;
	}
	mwx_rxr_nss[(nss - 1) & 3]++;
	mwx_rxr_mcs[idx & 0xf]++;
	if (mode >= MT_PHY_TYPE_HE_SU && mode <= MT_PHY_TYPE_HE_MU) {
		idx &= MWX_HE_RATE_MCS_MASK;
		mwx_rxr_he_gi[MWX_PRXV_SGI(v0) & 3]++;
	} else if (MWX_PRXV_SGI(v0))
		mwx_rxr_sgi++;
	if (MWX_PRXV_STBC(v0))
		mwx_rxr_stbc++;
	if (MWX_PRXV_LDPC(v0))
		mwx_rxr_ldpc++;

	if (mode >= MT_PHY_TYPE_HE_SU && mode <= MT_PHY_TYPE_HE_MU) {
		const unsigned bw = MWX_PRXV_FRAME_MODE(v0) & 3;
		const unsigned rsidx = bw * 2 + (nss > 1 ? 1 : 0);
		const struct ieee80211_he_rateset *rs =
		    &ieee80211_std_ratesets_11ax[rsidx];
		if (idx < rs->nrates) {
			mwx_rxr_last_kbps = rs->rates[idx] * 100;
			mwx_rxr_last_mcs = (int)idx;
		}
	} else if (mode >= 2 && mode <= 4 && idx <= 9 &&
	    (MWX_PRXV_FRAME_MODE(v0) & 3) < 4) {
		uint32_t kbps = mwx_vht_rate[MWX_PRXV_FRAME_MODE(v0) & 3][idx] *
		    100 * nss;
		if (MWX_PRXV_SGI(v0))
			kbps = kbps * 10 / 9;
		mwx_rxr_last_kbps = kbps;
		mwx_rxr_last_mcs = mode == 4 ? (int)idx : (int)(idx + 8 * (nss - 1));
	}
}

extern uint32_t ieee80211_bar_rx_count;

/* RX decrypt path counters, reported by MtkMwx::debugPoll(). */
static uint32_t mwx_rx_hwdec_cnt, mwx_rx_swdec_cnt, mwx_rx_hwdec_nopn_cnt;
static uint32_t mwx_rx_mgmt_hwdec_cnt;
static uint32_t mwx_rx_ctl_cnt, mwx_rx_bar_cnt;	/* control frames reaching the driver */

/* RX drop reason counters, reported by MtkMwx::debugPoll(). */
#define MWX_RX_DROP_REASONS	12
static const char *mwx_rx_drop_name[MWX_RX_DROP_REASONS];
static uint32_t mwx_rx_drop_cnt[MWX_RX_DROP_REASONS];

static void
mwx_rx_drop_count(const char *reason)
{
	int i;

	for (i = 0; i < MWX_RX_DROP_REASONS; i++) {
		if (mwx_rx_drop_name[i] == NULL)
			mwx_rx_drop_name[i] = reason;
		if (mwx_rx_drop_name[i] == reason) {
			mwx_rx_drop_cnt[i]++;
			return;
		}
	}
}

/* Route every HAL printf through the persistent debug log. */
#undef printf
#define printf(x...) mtk_dbg_log(x)

/*
 * mwx_mcu_alloc_msg() reserves room for the MCU TXD in front of the payload.
 * Keep payload + TXD within one 4K cluster: a 4096-byte chunk needs a 16K
 * jumbo cluster, which is often unavailable during early boot (ENOMEM).
 */
#define MWX_FW_CHUNK_SIZE	(4096 - sizeof(struct mwx_mcu_txd))

#define MWX_LOG(x...) printf("SimpleMtkWlan-mwx: " x)
#define MWX_DEV_LOG(sc, fmt, args...) \
	printf("SimpleMtkWlan-mwx: %s: " fmt, DEVNAME(sc), ##args)

#define ifnet _ifnet
#define mbuf __mbuf

#undef KASSERT
#define KASSERT(exp, ...) do { if (!(exp)) panic("mwx assertion failed: %s", #exp); } while (0)
#ifndef __unused
#define __unused __attribute__((unused))
#endif
#ifndef ISSET
#define ISSET(t, f) (((t) & (f)) != 0)
#endif
#ifndef IEEE80211_CHAN_40MHZ
#define IEEE80211_CHAN_40MHZ IEEE80211_CHAN_HT40
#endif
#ifndef IEEE80211_CHAN_6GHZ
#define IEEE80211_CHAN_6GHZ 0
#endif
#ifndef IEEE80211_IS_CHAN_6GHZ
#define IEEE80211_IS_CHAN_6GHZ(_c) false
#endif

#ifndef M_DEVBUF
#define M_DEVBUF 0
#endif
#ifndef M_NOWAIT
#define M_NOWAIT 0
#endif
#ifndef M_WAITOK
#define M_WAITOK 0
#endif
#ifndef M_ZERO
#define M_ZERO 0
#endif

static MtkMwx *gMwxHal;

static inline void *
mallocarray(size_t n, size_t size, int type, int flags)
{
	if (size != 0 && n > SIZE_MAX / size)
		return NULL;
	return malloc(n * size, type, flags);
}

static inline void
mwx_free_mem(void *addr)
{
	free(addr);
}

#define free(addr, type, size) mwx_free_mem(addr)

static inline int
loadfirmware(const char *name, u_char **buf, size_t *len)
{
	MWX_LOG("firmware lookup start name=%s\n", name);
	OSData *data = getFWDescByName(name);
	if (data == NULL) {
		MWX_LOG("firmware lookup failed name=%s error=%d\n", name, ENOENT);
		return ENOENT;
	}
	*len = data->getLength();
	*buf = (u_char *)malloc(*len, M_DEVBUF, M_WAITOK);
	if (*buf == NULL) {
		data->release();
		MWX_LOG("firmware copy allocation failed name=%s size=%zu error=%d\n",
		    name, *len, ENOMEM);
		return ENOMEM;
	}
	memcpy(*buf, data->getBytesNoCopy(), *len);
	data->release();
	MWX_LOG("firmware lookup ok name=%s size=%zu\n", name, *len);
	return 0;
}

static inline int
tsleep_nsec(void *ident, int priority, const char *wmesg, uint64_t timo)
{
	if (gMwxHal == NULL)
		return EWOULDBLOCK;
	return gMwxHal->mwxSleepNsec(ident, priority, wmesg, timo);
}

static inline void delay(int usec) {
    if (usec <= 0) return;
    if (usec < 1000) IODelay(usec);
    else IOSleep((usec + 999) / 1000);
}

static inline struct mbuf *
m_pullup(struct mbuf *m, size_t len)
{
	struct mbuf *n = NULL;
	size_t pktlen;
	u_int32_t chunks = 1;
	static uint32_t pullup_rebuild_log_count;

	if (m == NULL)
		return NULL;
	if (mbuf_len(m) >= len)
		return m;

	pktlen = mbuf_pkthdr_len(m);
	if (len > pktlen) {
		m_freem(m);
		return NULL;
	}

	if (pullup_rebuild_log_count < 32) {
		MWX_LOG("m_pullup rebuild len=%zu first_len=%zu pktlen=%zu\n",
		    len, mbuf_len(m), pktlen);
		pullup_rebuild_log_count++;
	}
	if (mbuf_allocpacket(M_DONTWAIT, pktlen, &chunks, &n) != 0) {
		m_freem(m);
		return NULL;
	}
	mbuf_setlen(n, pktlen);
	mbuf_pkthdr_setlen(n, pktlen);
	if (mbuf_copydata(m, 0, pktlen, mtod(n, void *)) != 0) {
		m_freem(n);
		m_freem(m);
		return NULL;
	}
	m_freem(m);
	return n;
}

static inline void
m_adj(struct mbuf *m, int len)
{
	mbuf_adj(m, len);
}

static inline struct mbuf *
m_prepend(struct mbuf *m, size_t len, int how)
{
	return mbuf_prepend(&m, len, how) == 0 ? m : NULL;
}

static inline size_t
m_leadingspace(struct mbuf *m)
{
	return mbuf_leadingspace(m);
}

static inline size_t
m_trailingspace(struct mbuf *m)
{
	return mbuf_trailingspace(m);
}

static inline struct mbuf *
m_gethdr(int how, int type)
{
	struct mbuf *m = NULL;
	mbuf_gethdr(how, type, &m);
	return m;
}

static inline struct mbuf *
MCLGETL(struct mbuf *m, int how, size_t len)
{
	u_int32_t chunks = 1;
	if (m == NULL) {
		if (mbuf_allocpacket(how, len, &chunks, &m) != 0)
			return NULL;
	} else if (mbuf_getcluster(how, mbuf_type(m), len, &m) != 0) {
		return NULL;
	}
	return m;
}

static const struct pci_matchid mwx_devices[] = {
	{ PCI_VENDOR_MEDIATEK, PCI_PRODUCT_MEDIATEK_MT7920 },
	{ PCI_VENDOR_MEDIATEK, PCI_PRODUCT_MEDIATEK_MT7921 },
	{ PCI_VENDOR_MEDIATEK, PCI_PRODUCT_MEDIATEK_MT7921K },
	{ PCI_VENDOR_MEDIATEK, PCI_PRODUCT_MEDIATEK_MT7922 },
	{ PCI_VENDOR_MEDIATEK, PCI_PRODUCT_MEDIATEK_MT7922_PCIE },
};

/* Keep the OpenBSD DPRINTF path available, but do not enable it by default.
 * The PCIe interrupt and MCU paths can otherwise flood the kernel log. */
/* #define MWX_DEBUG	1 */

#define	MT7920_ROM_PATCH	"mwx-mt7961_patch_mcu_1a_2_hdr"
#define	MT7920_FIRMWARE_WM	"mwx-mt7961_ram_code_1a"
#define	MT7921_ROM_PATCH	"mwx-mt7961_patch_mcu_1_2_hdr"
#define	MT7921_FIRMWARE_WM	"mwx-mt7961_ram_code_1"
#define	MT7922_ROM_PATCH	"mwx-mt7922_patch_mcu_1_1_hdr"
#define	MT7922_FIRMWARE_WM	"mwx-mt7922_ram_code_1"

#if NBPFILTER > 0
struct mwx_rx_radiotap_header {
	struct ieee80211_radiotap_header	wr_ihdr;
	uint64_t				wr_tsft;
	uint8_t					wr_flags;
	uint8_t					wr_rate;
	uint16_t				wr_chan_freq;
	uint16_t				wr_chan_flags;
	int8_t					wr_dbm_antsignal;
	int8_t					wr_dbm_antnoise;
} __packed;

#define	MWX_RX_RADIOTAP_PRESENT				\
	((1 << IEEE80211_RADIOTAP_TSFT) |		\
	(1 << IEEE80211_RADIOTAP_FLAGS) |		\
	(1 << IEEE80211_RADIOTAP_RATE) |		\
	(1 << IEEE80211_RADIOTAP_CHANNEL) |		\
	(1 << IEEE80211_RADIOTAP_DBM_ANTSIGNAL) |	\
	(1 << IEEE80211_RADIOTAP_DBM_ANTNOISE))

struct mwx_tx_radiotap_header {
	struct ieee80211_radiotap_header	wt_ihdr;
	uint8_t					wt_flags;
	uint8_t					wt_rate;
	uint16_t				wt_chan_freq;
	uint16_t				wt_chan_flags;
} __packed;

#define	MWX_TX_RADIOTAP_PRESENT			\
	((1 << IEEE80211_RADIOTAP_FLAGS) |	\
	(1 << IEEE80211_RADIOTAP_RATE) |	\
	(1 << IEEE80211_RADIOTAP_CHANNEL))

#endif

struct mwx_txwi {
	struct mt76_txwi		*mt_desc;
	struct mbuf			*mt_mbuf;
	bus_dmamap_t			mt_map;
	LIST_ENTRY(mwx_txwi)		mt_entry;
	u_int32_t			mt_addr;
	u_int				mt_idx;
	int				mt_busy;
};

struct mwx_txwi_desc {
	struct mt76_txwi		*mt_desc;
	struct mwx_txwi			*mt_data;

	u_int				mt_count;
	u_int				mt_nfree;	/* entries on mt_freelist */
	bus_dmamap_t			mt_map;
	bus_dma_segment_t		mt_seg;
	LIST_HEAD(, mwx_txwi)		mt_freelist;
};

struct mwx_queue_data {
	struct mbuf			*md_mbuf;
	struct mwx_txwi			*md_txwi;
	bus_dmamap_t			md_map;
};

/*
 * RX buffers must live below 4GB (Linux mt76 also uses 32-bit DMA), but the
 * generic mbuf cluster pool hands out pages anywhere in physical memory, so
 * every RX buffer was bounced through a full-size copy. A pool backed by
 * IOBufferMemoryDescriptor::inTaskWithPhysicalMask hands out only pages whose
 * physical addresses fit the mask; attaching them to mbufs as clusters lets
 * the hardware write the mbuf directly.
 */
struct mwx_rx_page {
	SLIST_ENTRY(mwx_rx_page)	link;
	uint8_t				*kva;
};

struct mwx_queue {
	uint32_t			mq_regbase;
	u_int				mq_count;
	u_int				mq_prod;
	u_int				mq_cons;

	struct mwx_desc		*mq_desc;
	struct mwx_queue_data		*mq_data;

	bus_dmamap_t			mq_map;
	bus_dma_segment_t		mq_seg;
	int				mq_wakeme;
	bool				mq_refill_pending;
	bool				mq_rx_discard; /* drop a bad chain through LAST */
	struct mbuf		*mq_rx_head; /* detached DMA buffers, across batches */
	struct mbuf		*mq_rx_tail;
	u_int				mq_rx_bytes;
	u_int				mq_rx_segments;
	u_int				mq_rx_wait_idx; /* slot waited for until DMA_DONE shows */
	uint64_t			mq_rx_wait_ns;  /* since when; 0 none */
};

/* How long a slot the DMA index has passed is waited for until DMA_DONE shows in it (mwx_dma_rx_dequeue()). */
#define MWX_RX_DONE_WAIT_NS	2000000ULL

/* RXD byte count is 16 bits; MCU event len excludes the RXD (up to 32
 * bytes on connac3). Keep malformed streams bounded without limiting an
 * otherwise legal MCU response to the 802.11 MPDU limit. */
#define MWX_RX_CHAIN_MAX_BYTES (UINT16_MAX + 8U * sizeof(uint32_t))
#define MWX_RX_CHAIN_MAX_SEGS 64U

struct mwx_hw_capa {
	int8_t		has_2ghz;
	int8_t		has_5ghz;
	int8_t		has_6ghz;
	uint8_t		antenna_mask;
	uint8_t		num_streams;
};

struct mwx_node {
	struct ieee80211_node	ni;
	uint16_t		wcid;
	uint16_t		flags;
};
#define	MWX_NODE_FLAG_HAVE_PAIRWISE_KEY		0x1
#define	MWX_NODE_FLAG_HAVE_GROUP_KEY		0x2

struct mwx_vif {
	struct mwx_node		vif_mn;
	uint8_t			idx;
	uint8_t			omac_idx;
	uint8_t			band_idx;
	uint8_t			wmm_idx;
	uint8_t			scan_seq_num;
};

enum mwx_hw_type {
	MWX_HW_MT7921,
	MWX_HW_MT7922,
	MWX_HW_MT7920,
};

struct mwx_setkey_task_arg {
	struct ieee80211_node	*ni;
	struct ieee80211_key	*k;
};

/*
 * Security fields of a received frame, from RXD DW1 and GROUP_1. The
 * firmware strips the CCMP header of the frames it decrypts and reports
 * the packet number here instead.
 */
struct mwx_rx_sec {
	uint8_t			sec_mode;	/* MT_RXD1_NORMAL_SEC_MODE, mt76_cipher_type */
	uint8_t			key_id;		/* MT_RXD1_NORMAL_KEY_ID */
	uint8_t			pn_valid;	/* pn[] was read from GROUP_1 */
	uint8_t			pn[6];		/* PN0..PN5, the order of the CCMP header */
};

struct mwx_softc {
	struct device		sc_dev;
	struct ieee80211com	sc_ic;
	MtkRxAmpduStats		sc_rx_ampdu_stats;

	enum mwx_hw_type	sc_hwtype;

	struct mwx_queue	sc_txq;
	struct mwx_queue	sc_txmcuq;
	struct mwx_queue	sc_txfwdlq;

	struct mwx_queue	sc_rxq;
	struct mwx_queue	sc_rxmcuq;
	struct mwx_queue	sc_rxfwdlq;

	struct mwx_txwi_desc	sc_txwi;

	bus_space_tag_t		sc_st;
	bus_space_handle_t	sc_memh;
	bus_size_t		sc_mems;
	bus_dma_tag_t		sc_dmat;
	pcitag_t		sc_tag;
	pci_chipset_tag_t	sc_pc;
	void			*sc_ih;

	int			(*sc_newstate)(struct ieee80211com *,
				    enum ieee80211_state, int);

	struct taskq		*sc_nswq;
	struct task		sc_newstate_task;
	struct task		sc_bgscan_done_task;
	struct task		sc_setkey_task;
	struct task		sc_scan_task;
	struct task		sc_reset_task;
	CTimeout		*sc_reset_to;
	u_int			sc_flags;
#define MWX_FLAG_SCANNING		0x01
#define MWX_FLAG_BGSCAN			0x02
	int			sc_coredump_cnt;
	int8_t			sc_resetting;
	int8_t			sc_hard_reset;	/* reload firmware on reset */
	int8_t			sc_tx_amsdu;	/* hardware TX A-MSDU (-mtknoamsdu: off) */
	int8_t			sc_vht160;	/* experimental VHT160 (-mtk160: on) */
	int8_t			sc_wtbl_ba;
	int8_t			sc_he_enabled; /* 802.11ax, on by default (-mtknohe: off) */
	int8_t			sc_he_only; /* experimental HE-only (-mtkheonly) */	/* hardware BA programming (-mtknowtblba: off) */
	volatile int8_t		sc_suspended;	/* system sleep: leave the hardware alone */
	int8_t			sc_stopping;	/* inside mwx_stop_internal() */
	int8_t			sc_mcu_mute;	/* no answer during a stop: no commands until init */
	uint32_t		sc_tx_full_cnt;
	uint32_t		sc_tx_kick_cnt;
	uint32_t		sc_output_start_cnt;
	uint32_t		sc_output_noqueue_cnt;
	uint32_t		sc_output_active_cnt;
	uint32_t		sc_output_capacity_cnt;
	uint32_t		sc_output_dequeue_fail_cnt;
	uint32_t		sc_output_post_active_cnt;
	uint32_t		sc_output_kick_fail_cnt;	/* output paused: no TXWI / ring slot */
	int8_t			sc_sta_added;	/* peer STA record exists in fw */
	bool sc_detaching, sc_net_attached;
	int8_t			sc_fw_loaded;

	enum ieee80211_state	sc_ns_state;
	int			sc_ns_arg;

	struct ieee80211_node_switch_bss_arg	*sc_bgscan_arg;
	size_t			sc_bgscan_arg_size;

	struct mwx_setkey_task_arg	sc_setkey_arg[8];
	int			sc_setkey_cur;
	int			sc_setkey_tail;

#if NBPFILTER > 0
	caddr_t			sc_drvbpf;
	union {
		struct mwx_rx_radiotap_header th;
		uint8_t pad[IEEE80211_RADIOTAP_HDRLEN];
	}			sc_rxtapu;
	int			sc_rxtap_len;
	union {
		struct mwx_tx_radiotap_header th;
		uint8_t pad[IEEE80211_RADIOTAP_HDRLEN];
	}			sc_txtapu;
	int			sc_txtap_len;
#define	sc_rxtap	sc_rxtapu.th
#define	sc_txtap	sc_txtapu.th
#endif

	struct mwx_vif		sc_vif;
	uint32_t		sc_wcid_mask;
	uint32_t		sc_intr_mask;

	/* mcu */
	uint32_t		sc_mcu_seq;
	struct {
		struct mbuf	*mcu_m;
		uint32_t	 mcu_cmd;
		uint32_t	 mcu_int;
	}			sc_mcu_wait[16];
	uint8_t			sc_scan_seq_num;
	uint64_t		sc_last_bgscan_ns;

	/* phy / hw */
	struct mwx_hw_capa	sc_capa;
	uint8_t			sc_lladdr[ETHER_ADDR_LEN];
	char			sc_alpha2[4]; /* regulatory-domain */

	int16_t			sc_coverage_class;
	uint8_t			sc_slottime;

	/* mac specific */
	uint32_t		sc_rxfilter;

	/* diagnostic counters, all rate-limited at print sites */
	uint64_t		sc_rx_dma_bytes;
	uint64_t		sc_rx_dma_copy_bytes;
	uint64_t		sc_rx_dma_copy_saved;
	uint64_t		sc_rx_dma_batches;
	uint64_t		sc_rx_dma_desc;
	uint64_t		sc_rx_input_batches;
	uint64_t		sc_rx_input_packets;
	uint32_t		sc_rx_input_max_batch;
	IOLock			*sc_rx_amsdu_lock;
	IOCommandGate		*sc_rx_amsdu_gate; /* borrowed from the HAL */
	IOSimpleLock		*sc_rx_pool_lock;
	IOBufferMemoryDescriptor	*sc_rx_pool_md;
	uint8_t			*sc_rx_pool_kva;
	uint8_t			*sc_rx_pool_end;
	SLIST_HEAD(, mwx_rx_page)	sc_rx_pool_free;
	struct mwx_rx_page	*sc_rx_pool_pages;
	uint32_t		sc_rx_pool_total;
	uint32_t		sc_rx_pool_free_cnt;
	uint32_t		sc_rx_pool_allocs;
	uint32_t		sc_rx_pool_exhausted;
	uint32_t		sc_rx_pool_fallbacks;
	uint32_t		sc_rx_pool_returned;
	bool			sc_rx_pool_disabled;
	uint32_t		sc_rx_amsdu_transition_depth;
	bool			sc_rx_amsdu_teardown; /* protected by the RX mutex */
	bool			sc_rx_amsdu_irq_quiesced;
	MtkRxAmsduState	sc_rx_amsdu_state;
	uint32_t		sc_rx_amsdu_epoch;
	bool			sc_rx_amsdu_enabled;
	uint32_t		sc_rx_dma_max_batch;
	uint32_t		sc_rx_dma_bad_len;
	uint64_t		sc_rx_dma_notdone;	/* slots found without DMA_DONE behind the DMA index */
	uint64_t		sc_rx_dma_done_missing;	/* ... still without it after MWX_RX_DONE_WAIT_NS */
	uint64_t		sc_rx_data_last_us;	/* uptime of the last slot taken from the data RX ring */
	uint64_t		sc_rx_dma_chain_completed;
	uint64_t		sc_rx_dma_chain_resumed;
	uint64_t		sc_rx_dma_chain_dropped;
	uint32_t		sc_rx_refill_fail;
	uint32_t		sc_rx_refill_recover;
	uint32_t		sc_rx_process_events;
	uint32_t		sc_rx_pkt_event;
	uint32_t		sc_rx_pkt_txrx_notify;
	uint32_t		sc_rx_pkt_normal_mcu;
	uint32_t		sc_rx_pkt_normal;
	uint32_t		sc_rx_pkt_unknown;
	uint32_t		sc_rx_fill_fail;
	uint32_t		sc_rx_delivered;
	uint32_t		sc_scan_done_count;
	uint32_t		sc_scan_last_ieee80211_nodes;
#ifdef MTK_AWDL
	struct mwx_awdl		*sc_awdl;	/* the AWDL radio, allocated when awdl0 starts it */
	int			sc_awdl_ds;	/* the deep sleep setting the station asked for last */
	uint8_t			sc_cc_bssid[6];	/* the access point whose Country element was last read */
#endif

};

/* RX executes under the main workloop gate; state/key tasks execute on
 * thread_call. A transition begins/ends under that same gate, but all MCU
 * sleeps stay outside it. Nested transitions block only the new aggregate
 * path. Lock ordering is always gate -> RX mutex, never the reverse. */
static bool
mwx_rx_amsdu_tearing_down(struct mwx_softc *sc)
{
	if (sc->sc_rx_amsdu_lock == NULL)
		return false; /* partial attach has no running RX or tasks */
	IOLockLock(sc->sc_rx_amsdu_lock);
	const bool teardown = sc->sc_rx_amsdu_teardown;
	IOLockUnlock(sc->sc_rx_amsdu_lock);
	return teardown;
}

static bool
mwx_rx_amsdu_key_cleanup_only(struct mwx_softc *sc)
{
	if (sc->sc_rx_amsdu_lock == NULL)
		return false;
	IOLockLock(sc->sc_rx_amsdu_lock);
	const bool cleanup = sc->sc_rx_amsdu_teardown && sc->sc_rx_amsdu_irq_quiesced;
	IOLockUnlock(sc->sc_rx_amsdu_lock);
	return cleanup;
}

static void
mwx_rx_amsdu_purge(struct mwx_softc *sc, int tid = -1, bool invalid = false)
{
	if (sc->sc_rx_amsdu_lock == NULL)
		return;
	IOLockLock(sc->sc_rx_amsdu_lock);
	mtkRxAmsduPurge(sc->sc_rx_amsdu_state, tid);
	if (invalid)
		++sc->sc_rx_amsdu_state.stats.invalid;
	IOLockUnlock(sc->sc_rx_amsdu_lock);
}

struct MwxRxAmsduBarrier {
	struct mwx_softc *sc;
	int operation; /* 0: end, 1: begin, 2: teardown after cancellation */
	bool finished, accepted;
};

static IOReturn
mwx_rx_amsdu_barrier_gated(OSObject *, void *arg, void *, void *, void *)
{
	MwxRxAmsduBarrier *barrier = (MwxRxAmsduBarrier *)arg;
	struct mwx_softc *sc = barrier->sc;
	IOLockLock(sc->sc_rx_amsdu_lock);
	barrier->finished = true;
	barrier->accepted = false;
	if (barrier->operation == 2) {
		mtkRxAmsduPurge(sc->sc_rx_amsdu_state);
		++sc->sc_rx_amsdu_epoch;
		sc->sc_rx_amsdu_transition_depth = 0;
		barrier->accepted = true;
	} else if (!sc->sc_rx_amsdu_teardown) {
		if (barrier->operation == 1 &&
		    sc->sc_rx_amsdu_transition_depth != UINT32_MAX) {
			mtkRxAmsduPurge(sc->sc_rx_amsdu_state);
			++sc->sc_rx_amsdu_epoch;
			++sc->sc_rx_amsdu_transition_depth;
			barrier->accepted = true;
		} else if (barrier->operation == 0 &&
		    sc->sc_rx_amsdu_transition_depth != 0) {
			--sc->sc_rx_amsdu_transition_depth;
			barrier->accepted = true;
		}
	}
	IOLockUnlock(sc->sc_rx_amsdu_lock);
	return kIOReturnSuccess;
}

/* Cancellable acquisition: detach may be waiting for this thread while it
 * owns the workloop gate. Never block on that gate while holding the mutex.
 * An unavailable/disabled gate fails closed; its caller must skip the change. */
static bool
mwx_rx_amsdu_barrier(struct mwx_softc *sc, int operation)
{
	MwxRxAmsduBarrier barrier = { sc, operation, false, false };
	for (;;) {
		if (mwx_rx_amsdu_tearing_down(sc) || sc->sc_rx_amsdu_gate == NULL)
			return false;
		IOReturn status = sc->sc_rx_amsdu_gate->attemptAction(
		    mwx_rx_amsdu_barrier_gated, &barrier);
		if (barrier.finished)
			return barrier.accepted;
		if (status != kIOReturnCannotLock)
			return false;
		IOSleep(1);
	}
}

/* This guard owns only a transition token, never a lock across its scope.
 * No new task or scheduling queue is introduced. */
class MwxRxAmsduTransition {
	struct mwx_softc *sc;
	bool accepted, owns_token;
public:
	explicit MwxRxAmsduTransition(struct mwx_softc *state, bool bypass = false)
	    : sc(state), accepted(false), owns_token(false)
	{
		if (bypass) {
			accepted = true; /* sole detach owner, after the RX gate barrier */
			return;
		}
		if (mwx_rx_amsdu_tearing_down(sc))
			return;
		if (!sc->sc_rx_amsdu_enabled) {
			accepted = true;
			return;
		}
		accepted = owns_token = mwx_rx_amsdu_barrier(sc, 1);
	}
	~MwxRxAmsduTransition()
	{
		if (owns_token)
			(void)mwx_rx_amsdu_barrier(sc, 0);
	}
	bool ok() const { return accepted; }
private:
	MwxRxAmsduTransition(const MwxRxAmsduTransition &);
	MwxRxAmsduTransition &operator=(const MwxRxAmsduTransition &);
};

/* Publish cancellation before waiting for the gate or draining task callbacks.
 * RX sees teardown immediately; the gate barrier also drains any frame already
 * executing past the snapshot. Ordinary waiters can now return without the
 * gate. After IRQ removal final purge/free use only the RX mutex. */
static void
mwx_rx_amsdu_begin_teardown(struct mwx_softc *sc)
{
	if (sc->sc_rx_amsdu_lock == NULL)
		return;
	IOLockLock(sc->sc_rx_amsdu_lock);
	sc->sc_rx_amsdu_teardown = true;
	IOLockUnlock(sc->sc_rx_amsdu_lock);
	if (!sc->sc_rx_amsdu_enabled)
		return;
	MwxRxAmsduBarrier barrier = { sc, 2, false, false };
	if (sc->sc_rx_amsdu_gate == NULL)
		return; /* partial attach; no interrupt source exists yet */
	for (;;) {
		IOReturn status = sc->sc_rx_amsdu_gate->attemptAction(
		    mwx_rx_amsdu_barrier_gated, &barrier);
		if (barrier.finished)
			return;
		if (status == kIOReturnCannotLock) {
			IOSleep(1);
			continue;
		}
		/* The command gate can be disabled during stop. Cancellation is
		 * already visible, so task waiters cannot form a gate/quiesce cycle.
		 * The public workloop API still serializes this short barrier. */
		IOWorkLoop *loop = sc->sc_rx_amsdu_gate->getWorkLoop();
		if (loop != NULL)
			(void)loop->runAction(mwx_rx_amsdu_barrier_gated, NULL, &barrier);
		return;
	}
}

static void
mwx_rx_amsdu_invalid(struct mwx_softc *sc)
{
	IOLockLock(sc->sc_rx_amsdu_lock);
	++sc->sc_rx_amsdu_state.stats.invalid;
	IOLockUnlock(sc->sc_rx_amsdu_lock);
}

static bool
mwx_rx_amsdu_snapshot(struct mwx_softc *sc, uint32_t *epoch)
{
	IOLockLock(sc->sc_rx_amsdu_lock);
	*epoch = sc->sc_rx_amsdu_epoch;
	const bool accept = !sc->sc_rx_amsdu_teardown &&
	    sc->sc_rx_amsdu_transition_depth == 0;
	IOLockUnlock(sc->sc_rx_amsdu_lock);
	return accept;
}

static uint32_t
mwx_rx_amsdu_epoch(struct mwx_softc *sc)
{
	IOLockLock(sc->sc_rx_amsdu_lock);
	const uint32_t epoch = sc->sc_rx_amsdu_epoch;
	IOLockUnlock(sc->sc_rx_amsdu_lock);
	return epoch;
}

static struct mbuf *
mwx_rx_amsdu_feed(struct mwx_softc *sc, struct mbuf *m,
    MtkRxAmsduMeta &meta, uint64_t now_ms, MtkRxAmsduMeta *first)
{
	IOLockLock(sc->sc_rx_amsdu_lock);
	if (sc->sc_rx_amsdu_teardown || sc->sc_rx_amsdu_transition_depth != 0 ||
	    meta.epoch != sc->sc_rx_amsdu_epoch) {
		++sc->sc_rx_amsdu_state.stats.mismatch;
		IOLockUnlock(sc->sc_rx_amsdu_lock);
		mbuf_freem(m);
		return NULL;
	}
	m = mtkRxAmsduFeed(sc->sc_rx_amsdu_state, m, meta, now_ms, first);
	IOLockUnlock(sc->sc_rx_amsdu_lock);
	return m;
}

static void
mwx_rx_amsdu_expire(struct mwx_softc *sc, uint64_t now_ms)
{
	IOLockLock(sc->sc_rx_amsdu_lock);
	mtkRxAmsduExpire(sc->sc_rx_amsdu_state, now_ms);
	IOLockUnlock(sc->sc_rx_amsdu_lock);
}

static MtkRxAmsduStats
mwx_rx_amsdu_stats(struct mwx_softc *sc)
{
	IOLockLock(sc->sc_rx_amsdu_lock);
	MtkRxAmsduStats stats = sc->sc_rx_amsdu_state.stats;
	IOLockUnlock(sc->sc_rx_amsdu_lock);
	return stats;
}


const uint8_t mwx_channels_2ghz[] = {
	1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14
};

#define MWX_NUM_2GHZ_CHANNELS   nitems(mwx_channels_2ghz)

const uint8_t mwx_channels_5ghz[] = {
	36, 40, 44, 48, 52, 56, 60, 64,
	100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144,
	149, 153, 157, 161, 165, 169, 173
};

#define MWX_NUM_5GHZ_CHANNELS   nitems(mwx_channels_5ghz)

const uint8_t mwx_channels_6ghz[] = {
	/* UNII-5 */
	1, 5, 9, 13, 17, 21, 25, 29, 33, 37, 41, 45, 49, 53, 57,
	61, 65, 69, 73, 77, 81, 85, 89, 93,
	/* UNII-6 */
	97, 101, 105, 109, 113, 117,
	/* UNII-7 */
	121, 125, 129, 133, 137, 141, 145, 149, 153, 157, 161, 165, 169,
	173, 177, 181, 185,
	/* UNII-8 */
	189, 193, 197, 201, 205, 209, 213, 217, 221, 225, 229, 233
};

const struct mwx_rate {
	uint16_t	rate;
	uint16_t	hw_value;
} mt76_rates[] = {
	{	2,	(MT_PHY_TYPE_CCK << 8) | 0 },
	{	4,	(MT_PHY_TYPE_CCK << 8) | 1 },
	{	11,	(MT_PHY_TYPE_CCK << 8) | 2 },
	{	22,	(MT_PHY_TYPE_CCK << 8) | 3 },
	{	12,	(MT_PHY_TYPE_OFDM << 8) | 11 },
	{	18,	(MT_PHY_TYPE_OFDM << 8) | 15 },
	{	24,	(MT_PHY_TYPE_OFDM << 8) | 10 },
	{	36,	(MT_PHY_TYPE_OFDM << 8) | 14 },
	{	48,	(MT_PHY_TYPE_OFDM << 8) | 9 },
	{	72,	(MT_PHY_TYPE_OFDM << 8) | 13 },
	{	96,	(MT_PHY_TYPE_OFDM << 8) | 8 },
	{	108,	(MT_PHY_TYPE_OFDM << 8) | 12 },
};


#define MWX_NUM_6GHZ_CHANNELS   nitems(mwx_channels_6ghz)

#define MWX_RATE_HT_SISO_MCS_INV_PLCP 0x20

#define	DEVNAME(s)	((s)->sc_dev.dv_xname)
#define	DEVDEBUG(x)	((x)->sc_ic.ic_if.if_flags & IFF_DEBUG)

#define MWX_RX_DROP_LOG(sc, reason, rv, rxd0, rxd1, rxd2, rxd3, rxd4) do { \
	mwx_rx_drop_count(reason); \
	if ((sc)->sc_rx_fill_fail < 64) { \
		MWX_DEV_LOG((sc), \
		    "rx drop reason=%s rv=%d rxd=%08x/%08x/%08x/%08x/%08x len=%zu\n", \
		    (reason), (rv), (rxd0), (rxd1), (rxd2), (rxd3), \
		    (rxd4), mbuf_len(m)); \
	} \
} while (0)

#ifdef MWX_DEBUG
#define	DPRINTF(x...)		MWX_LOG(x)
#else
#define	DPRINTF(x...)
#endif

#ifdef MWX_PACKET_HEXDUMP
static void
pkt_hex_dump(struct mbuf *m)
{
	int len, rowsize = 16;
	int i, l, linelen;
	uint8_t *data;

	printf("Packet hex dump:\n");
	data = mtod(m, uint8_t *);
	len = mbuf_len(m);

	for (i = 0; i < len; i += linelen) {
		printf("%04x\t", i);
		linelen = len - i;
		if (len - i > rowsize)
			linelen = rowsize;
		for (l = 0; l < linelen; l++)
			printf("%02X ", (uint32_t)data[l]);
		data += linelen;
		printf("\n");
	}
}
#endif

int	mwx_init(struct ifnet *);
int	mwx_init_hw(struct ifnet *);
void	mwx_stop(struct ifnet *);
static void mwx_stop_internal(struct ifnet *, bool);
void	mwx_watchdog(struct ifnet *);
void	mwx_updateedca(struct ieee80211com *);
void	mwx_start(struct ifnet *);
int	mwx_ioctl(struct ifnet *, u_long, caddr_t);

struct ieee80211_node *mwx_node_alloc(struct ieee80211com *);
int	mwx_media_change(struct ifnet *);
#if NBPFILTER > 0
void	mwx_radiotap_attach(struct mwx_softc *);
#endif

int	mwx_newstate(struct ieee80211com *, enum ieee80211_state, int);
void	mwx_newstate_task(void *);
int	mwx_scan(struct mwx_softc *);
int	mwx_bgscan(struct ieee80211com *);
void	mwx_bgscan_done(struct ieee80211com *,
	    struct ieee80211_node_switch_bss_arg *, size_t);
void	mwx_bgscan_done_task(void *);
int	mwx_set_key(struct ieee80211com *, struct ieee80211_node *,
	    struct ieee80211_key *);
void	mwx_delete_key(struct ieee80211com *, struct ieee80211_node *,
	    struct ieee80211_key *);
void	mwx_setkey_task(void *);

int	mwx_tx(struct mwx_softc *, struct mbuf *, struct ieee80211_node *);
void	mwx_rx(struct mwx_softc *, struct mbuf *, struct mbuf_list *, uint8_t);
int	mwx_count_nodes(struct ieee80211com *);
int	mwx_intr(void *);
int	mwx_preinit(struct mwx_softc *);
void	mwx_attach_hook(struct device *);
bool	mwx_attach(struct device *, struct device *, void *);
int	mwx_activate(struct device *, int);

void	mwx_reset(struct mwx_softc *);
void	mwx_reset_timeo(void *);
void	mwx_reset_task(void *);
int	mwx_txwi_alloc(struct mwx_softc *, int);
void	mwx_txwi_free(struct mwx_softc *);
struct mwx_txwi	*mwx_txwi_get(struct mwx_softc *);
void	mwx_txwi_put(struct mwx_softc *, struct mwx_txwi *);
int	mwx_txwi_enqueue(struct mwx_softc *, struct mwx_txwi *, struct mbuf *);
int	mwx_queue_alloc(struct mwx_softc *, struct mwx_queue *, int, uint32_t);
void	mwx_queue_free(struct mwx_softc *, struct mwx_queue *);
void	mwx_queue_reset(struct mwx_softc *, struct mwx_queue *);
int	mwx_buf_fill(struct mwx_softc *, struct mwx_queue_data *,
	    struct mwx_desc *);
int	mwx_queue_fill(struct mwx_softc *, struct mwx_queue *);
int	mwx_dma_alloc(struct mwx_softc *);
int	mwx_dma_reset(struct mwx_softc *, int);
void	mwx_dma_free(struct mwx_softc *);
int	mwx_dma_tx_enqueue(struct mwx_softc *, struct mwx_queue *,
	    struct mbuf *);
int	mwx_dma_txwi_enqueue(struct mwx_softc *, struct mwx_queue *,
	    struct mwx_txwi *);
void	mwx_dma_tx_cleanup(struct mwx_softc *, struct mwx_queue *);
void	mwx_dma_tx_done(struct mwx_softc *);
void	mwx_dma_rx_process(struct mwx_softc *, struct mbuf_list *,
	    struct mbuf_list *, uint8_t);
bool	mwx_dma_rx_dequeue(struct mwx_softc *, struct mwx_queue *,
	    struct mbuf_list *);
bool	mwx_dma_rx_done(struct mwx_softc *, struct mwx_queue *,
	    struct mbuf_list *);
static void	mwx_add_sta_he_tlvs(struct mbuf *, uint16_t *,
	    struct ieee80211_node *);

struct mbuf	*mwx_mcu_alloc_msg(size_t);
void	mwx_mcu_set_len(struct mbuf *, void *);
int	mwx_mcu_send_mbuf(struct mwx_softc *, uint32_t, struct mbuf *, int *);
int	mwx_mcu_send_msg(struct mwx_softc *, uint32_t, void *, size_t, int *);
int	mwx_mcu_send_wait(struct mwx_softc *, uint32_t, void *, size_t);
int	mwx_mcu_send_mbuf_wait(struct mwx_softc *, uint32_t, struct mbuf *);
void	mwx_mcu_rx_event(struct mwx_softc *, struct mbuf *);
int	mwx_mcu_wait_resp_int(struct mwx_softc *, uint32_t, int, uint32_t *);
int	mwx_mcu_wait_resp_msg(struct mwx_softc *, uint32_t, int,
	    struct mbuf **);

int		mwx_dma_disable(struct mwx_softc *sc, int force);
void		mwx_dma_enable(struct mwx_softc *sc);
int		mwx_pci_flr(struct mwx_softc *, IOPCIDevice *);
void		mwx_pci_disable_aspm(struct mwx_softc *, IOPCIDevice *);
static void	mwx_node_chan_width(struct ieee80211_node *, uint8_t *, uint8_t *);
int		mwx_ampdu_rx_start(struct ieee80211com *, struct ieee80211_node *,
		    uint8_t);
void		mwx_ampdu_rx_stop(struct ieee80211com *, struct ieee80211_node *,
		    uint8_t);
int		mwx_ampdu_tx_start(struct ieee80211com *, struct ieee80211_node *,
		    uint8_t);
void		mwx_ampdu_tx_stop(struct ieee80211com *, struct ieee80211_node *,
		    uint8_t);
int		mwx_mcu_fw_pmctrl(struct mwx_softc *);
int		mwx_mcu_drv_pmctrl(struct mwx_softc *);
int		mwx_wfsys_reset(struct mwx_softc *sc);
uint32_t	mwx_reg_addr(struct mwx_softc *, uint32_t);
int		mwx_init_hardware(struct mwx_softc *);
int		mt7921_mcu_init(struct mwx_softc *);
int		mwx_load_firmware(struct mwx_softc *);
int		mwx_mac_wtbl_update(struct mwx_softc *, int);
void		mwx_mac_init_band(struct mwx_softc *sc, uint32_t);
void		mwx_mac_init(struct mwx_softc *);
int		mwx_mcu_patch_sem_ctrl(struct mwx_softc *, int);
int		mwx_mcu_init_download(struct mwx_softc *, uint32_t,
		    uint32_t, uint32_t);
int		mwx_mcu_send_firmware(struct mwx_softc *, int,
		    u_char *, size_t, size_t);
int		mwx_mcu_start_patch(struct mwx_softc *);
int		mwx_mcu_start_firmware(struct mwx_softc *, uint32_t,
		    uint32_t);
int		mt7921_mcu_get_nic_capability(struct mwx_softc *);
int		mt7921_mcu_fw_log_2_host(struct mwx_softc *, uint8_t);
int		mt7921_mcu_set_eeprom(struct mwx_softc *);
int		mt7921_mcu_set_rts_thresh(struct mwx_softc *, uint32_t,
		    uint8_t);
int		mwx_mcu_set_deep_sleep(struct mwx_softc *, int);
void		mt7921_mcu_low_power_event(struct mwx_softc *, struct mbuf *);
void		mt7921_mcu_tx_done_event(struct mwx_softc *, struct mbuf *);
void		mwx_end_scan_task(void *);
void		mt7921_mcu_scan_event(struct mwx_softc *, struct mbuf *);
int		mt7921_mcu_hw_scan(struct mwx_softc *, int);
int		mt7921_mcu_hw_scan_cancel(struct mwx_softc *);
int		mt7921_mcu_set_mac_enable(struct mwx_softc *, int, int);
int		mt7921_mcu_set_channel_domain(struct mwx_softc *);
uint8_t		mt7921_mcu_chan_bw(struct ieee80211_channel *channel);
int		mt7921_mcu_set_chan_info(struct mwx_softc *, int);
void		mt7921_mcu_build_sku(struct mwx_softc *, int, int8_t *);
int		mt7921_mcu_rate_txpower_band(struct mwx_softc *, int,
		    const uint8_t *, int, int);
int		mt7921_mcu_set_rate_txpower(struct mwx_softc *);
void		mt7921_mac_reset_counters(struct mwx_softc *);
void		mt7921_mac_set_timing(struct mwx_softc *);
int		mt7921_mcu_uni_add_dev(struct mwx_softc *, struct mwx_vif *,
		    struct mwx_node *, int);
int		mt7921_mcu_set_sniffer(struct mwx_softc *, int);
int		mt7921_mcu_set_beacon_filter(struct mwx_softc *, int);
int		mt7921_mcu_set_bss_pm(struct mwx_softc *, int);
int		mt7921_mcu_set_tx(struct mwx_softc *, struct mwx_vif *);
int		mt7921_mac_fill_rx(struct mwx_softc *, struct mbuf *,
		    struct ieee80211_rxinfo *, struct mwx_rx_sec *);
uint32_t	mt7921_mac_tx_rate_val(struct mwx_softc *);
void		mt7921_mac_write_txwi_80211(struct mwx_softc *, struct mbuf *,
		    struct ieee80211_node *, struct mt76_txwi *);
void		mt7921_mac_write_txwi(struct mwx_softc *, struct mbuf *,
		    struct ieee80211_node *, struct mt76_txwi *);
void		mwx_mac_tx_free(struct mwx_softc *, struct mbuf *);
int		mt7921_set_channel(struct mwx_softc *);
int mt7921_mcu_set_associated_bss(struct mwx_softc *, int);
int mt7921_mcu_set_associated_peer(struct mwx_softc *);

uint8_t		 mt7921_get_phy_mode_v2(struct mwx_softc *,
		    struct ieee80211_node *);
struct mbuf	*mwx_alloc_sta_req_tlv(int);
void		*mwx_append_len(struct mbuf *, int);
void		*mwx_append_tlv(struct mbuf *, uint16_t *, int, int);
struct mbuf	*mwx_fill_sta_req_hdr(struct mbuf *, struct mwx_vif *,
		    uint8_t, uint16_t, uint16_t);
void		 mt7921_mcu_add_basic_tlv(struct mbuf *, uint16_t *,
		    struct mwx_softc *, struct ieee80211_node *, int, int);
void		 mt7921_mcu_add_sta_tlv(struct mbuf *, uint16_t *,
		    struct mwx_softc *, struct ieee80211_node *, int, int);
int		 mt7921_mcu_wtbl_generic_tlv(struct mbuf *, uint16_t *,
		    struct mwx_softc *, struct ieee80211_node *);
int		 mt7921_mcu_wtbl_hdr_trans_tlv(struct mbuf *, uint16_t *,
		    struct mwx_softc *, struct ieee80211_node *);
int		 mt7921_mcu_wtbl_ht_tlv(struct mbuf *, uint16_t *,
		    struct mwx_softc *, struct ieee80211_node *);
int		 mt7921_mac_sta_update(struct mwx_softc *,
		    struct ieee80211_node *, int, int);
void		 mt7921_mcu_add_key_tlv(struct mbuf *, uint16_t *,
		    struct ieee80211_key *, int);
int		 mt7921_mcu_sta_key_update(struct mwx_softc *,
		    struct ieee80211_node *, struct ieee80211_key *);
void		 mt7921_mcu_sta_key_delete(struct mwx_softc *,
			    struct ieee80211_node *, struct ieee80211_key *);
#ifdef MTK_AWDL
/* The AWDL radio, at the end of this file. Each of these is called only when sc_awdl is set. */
static bool	mwx_awdl_rx_tap(struct mwx_softc *, struct mbuf *, const struct ieee80211_rxinfo *, uint8_t);
static uint8_t	mwx_awdl_rx_subframe(struct mbuf *, uint16_t);
static bool	mwx_awdl_mcu_event(struct mwx_softc *, uint8_t, uint8_t, uint8_t, struct mbuf *);
static bool	mwx_awdl_tx_done(struct mwx_softc *, uint16_t, uint32_t);
static void	mwx_awdl_sta_traffic(struct mwx_softc *, struct mbuf *, bool);
static bool	mwx_awdl_rx_expected(struct mwx_softc *, uint64_t);
static void	mwx_awdl_stall_log(struct mwx_softc *);
static void	mwx_awdl_rest(struct mwx_softc *, bool);
static void	mwx_awdl_quiesce(struct mwx_softc *);
static void	mwx_awdl_firmware_reset(struct mwx_softc *);
static void	mwx_awdl_resume(struct mwx_softc *);
static void	mwx_awdl_rx_release(struct mwx_softc *);
static void	mwx_awdl_free(struct mwx_softc *);
static void	mwx_awdl_attach(struct mwx_softc *);
/* The access point's country, at the end of this file too; it does not need sc_awdl. */
static void	mwx_ap_country(struct mwx_softc *, struct ieee80211_node *);
bool		mtkAwdlEnabled();
#endif

static inline int
mwx_dbm_to_rssi(int dbm)
{
	if (dbm <= -100)
		return 0;
	if (dbm >= 0)
		return 100;
	return dbm + 100;
}

static inline uint32_t
mwx_read(struct mwx_softc *sc, uint32_t reg)
{
	reg = mwx_reg_addr(sc, reg);
	return bus_space_read_4(sc->sc_st, sc->sc_memh, reg);
}

static inline void
mwx_write(struct mwx_softc *sc, uint32_t reg, uint32_t val)
{
	reg = mwx_reg_addr(sc, reg);
	bus_space_write_4(sc->sc_st, sc->sc_memh, reg, val);
}

static inline void
mwx_barrier(struct mwx_softc *sc)
{
	bus_space_barrier(sc->sc_st, sc->sc_memh, 0, sc->sc_mems,
	    BUS_SPACE_BARRIER_READ | BUS_SPACE_BARRIER_WRITE);
}

static inline uint32_t
mwx_rmw(struct mwx_softc *sc, uint32_t reg, uint32_t val, uint32_t mask)
{
	reg = mwx_reg_addr(sc, reg);
	val |= bus_space_read_4(sc->sc_st, sc->sc_memh, reg) & ~mask;
	bus_space_write_4(sc->sc_st, sc->sc_memh, reg, val);
	return val;
}

static inline uint32_t
mwx_set(struct mwx_softc *sc, uint32_t reg, uint32_t bits)
{
	return mwx_rmw(sc, reg, bits, 0);
}

static inline uint32_t
mwx_clear(struct mwx_softc *sc, uint32_t reg, uint32_t bits)
{
	return mwx_rmw(sc, reg, 0, bits);
}

static inline uint32_t
mt7921_map_reg_l1(struct mwx_softc *sc, uint32_t reg)
{
	uint32_t offset = MWX_HIF_REG_OFFSET(reg);
	uint32_t base = MWX_HIF_REG_BASE(reg);

	mwx_rmw(sc, MT7921_HIF_REMAP_L1,
	    base << MT7921_HIF_REMAP_L1_SHIFT, MT7921_HIF_REMAP_L1_MASK);
	mwx_barrier(sc);

	return MT7921_HIF_REMAP_BASE_L1 + offset;
}

/*
 * Poll for timeout milliseconds or until register reg read the value val
 * after applying the mask to the value read. Returns 0 on success ETIMEDOUT
 * on failure.
 */
int
mwx_poll(struct mwx_softc *sc, uint32_t reg, uint32_t val, uint32_t mask,
    int timeout)
{
	uint32_t cur;

	reg = mwx_reg_addr(sc, reg);
	timeout *= 100;
	do {
		cur = bus_space_read_4(sc->sc_st, sc->sc_memh, reg) & mask;
		if (cur == val)
			return 0;
		delay(10);
	} while (timeout-- > 0);

	DPRINTF("%s: poll timeout reg %x val %x mask %x cur %x\n",
	    DEVNAME(sc), reg, val, mask, cur);
	return ETIMEDOUT;
}

/*
 * ifp specific functions
 */
int
mwx_init(struct ifnet *ifp)
{
#ifdef AIRPORT_WCL
	/*
	 * The native Wi-Fi interface (AirportMtkWlan) reports the adapter to
	 * macOS as resetting, then available or gone, from these events.
	 */
	struct mwx_softc *sc = (struct mwx_softc *)ifp->if_softc;
	struct ieee80211com *ic = &sc->sc_ic;
	int rv;

	if (ic->ic_event_handler)
		ic->ic_event_handler(ic, IEEE80211_EVT_DRIVER_RESET_BEGIN, NULL);
	rv = mwx_init_hw(ifp);
	if (ic->ic_event_handler)
		ic->ic_event_handler(ic, rv ? IEEE80211_EVT_DRIVER_STOPPED :
		    IEEE80211_EVT_DRIVER_READY, NULL);
	return rv;
#else
	return mwx_init_hw(ifp);
#endif
}

int
mwx_init_hw(struct ifnet *ifp)
{
	struct mwx_softc *sc = (struct mwx_softc *)ifp->if_softc;
	struct ieee80211com *ic = &sc->sc_ic;
	struct mwx_node *mn;
	const uint8_t *lladdr = ((struct arpcom *)ifp)->ac_enaddr;
	int rv;

	sc->sc_mcu_mute = 0;
	if (!sc->sc_fw_loaded) {
#ifdef MTK_AWDL
		/* A new firmware has none of the AWDL radio's settings. */
		if (sc->sc_awdl != NULL)
			mwx_awdl_firmware_reset(sc);
#endif
		rv = mwx_preinit(sc);
		if (rv) {
			MWX_DEV_LOG(sc, "init preinit failed error=%d\n", rv);
			return rv;
		}
	}

	/*
	 * Use the address the upper layers set (macOS's private Wi-Fi address
	 * per network), like iwx_preinit(): net80211 sends with ic_myaddr and
	 * the firmware's vif (registered below) filters on it. Otherwise DHCP
	 * runs on the private address while the AP knows us by the factory one.
	 */
	if (!IEEE80211_ADDR_EQ(lladdr, etheranyaddr) &&
	    !IEEE80211_ADDR_EQ(lladdr, ic->ic_myaddr)) {
		IEEE80211_ADDR_COPY(ic->ic_myaddr, lladdr);
		MWX_DEV_LOG(sc, "init mac address now %s\n",
		    ether_sprintf(ic->ic_myaddr));
	}

	MWX_DEV_LOG(sc, "init start opmode=%d flags=0x%x ic_state=%d\n",
	    ic->ic_opmode, ifp->if_flags, ic->ic_state);
	rv = mwx_mcu_set_deep_sleep(sc, 0);
	if (rv)
		MWX_DEV_LOG(sc, "init disable deep sleep returned %d\n", rv);
	else
		MWX_DEV_LOG(sc, "init deep sleep disabled\n");

	rv = mt7921_mcu_set_mac_enable(sc, 0, 1);
	if (rv) {
		MWX_DEV_LOG(sc, "init mac enable failed error=%d\n", rv);
		return rv;
	}
	MWX_DEV_LOG(sc, "init mac enable ok\n");

	rv = mt7921_mcu_set_channel_domain(sc);
	if (rv) {
		MWX_DEV_LOG(sc, "init channel domain failed error=%d\n", rv);
		return rv;
	}
	MWX_DEV_LOG(sc, "init channel domain ok\n");

#if 0
	/* XXX no channel available yet */
	rv = mt7921_mcu_set_chan_info(sc, MCU_EXT_CMD_SET_RX_PATH);
	if (rv)
		return rv;
#endif

	rv = mt7921_mcu_set_rate_txpower(sc);
	if (rv) {
		MWX_DEV_LOG(sc, "init rate txpower failed error=%d\n", rv);
		return rv;
	}
	MWX_DEV_LOG(sc, "init rate txpower ok\n");

	mt7921_mac_reset_counters(sc);
	MWX_DEV_LOG(sc, "init counters reset\n");

	mn = &sc->sc_vif.vif_mn;
	MWX_DEV_LOG(sc, "init vif wcid=%u\n",
	    mn != NULL ? mn->wcid : 0);

	mwx_mac_wtbl_update(sc, mn->wcid);
	MWX_DEV_LOG(sc, "init wtbl updated wcid=%u\n", mn->wcid);

	rv = mt7921_mcu_uni_add_dev(sc, &sc->sc_vif, mn, 1);
	if (rv) {
		MWX_DEV_LOG(sc, "init add dev failed error=%d\n", rv);
		return rv;
	}
	MWX_DEV_LOG(sc, "init add dev ok\n");

	rv = mt7921_mcu_set_tx(sc, &sc->sc_vif);
	if (rv) {
		MWX_DEV_LOG(sc, "init set tx failed error=%d\n", rv);
		return rv;
	}
	MWX_DEV_LOG(sc, "init set tx ok\n");

	if (ic->ic_opmode == IEEE80211_M_MONITOR) {
		rv = mt7921_mcu_set_chan_info(sc, MCU_EXT_CMD_SET_RX_PATH);
		if (rv)
			return rv;
		rv = mt7921_set_channel(sc);
		if (rv)
			return rv;

		rv = mt7921_mcu_set_sniffer(sc, 1);
		MWX_DEV_LOG(sc, "init monitor sniffer enable returned %d\n", rv);
		rv = mt7921_mcu_set_beacon_filter(sc, 0);
		MWX_DEV_LOG(sc, "init monitor beacon filter disable returned %d\n", rv);
		sc->sc_rxfilter = 0;
		mwx_set(sc, MT_DMA_DCR0(0), MT_DMA_DCR0_RXD_G5_EN);
	} else {
		rv = mt7921_mcu_set_sniffer(sc, 0);
		MWX_DEV_LOG(sc, "init sniffer disable returned %d\n", rv);
		sc->sc_rxfilter |= MT_WF_RFCR_DROP_OTHER_UC;
		mwx_clear(sc, MT_DMA_DCR0(0), MT_DMA_DCR0_RXD_G5_EN);
	}
	mwx_write(sc, MT_WF_RFCR(0), sc->sc_rxfilter);
	MWX_DEV_LOG(sc, "init rxfilter set 0x%08x\n", sc->sc_rxfilter);

	ifp->if_flags |= IFF_RUNNING;
	ifq_clr_oactive(&ifp->if_snd);
	MWX_DEV_LOG(sc, "init interface running flags=0x%x\n", ifp->if_flags);

	if (ic->ic_opmode == IEEE80211_M_MONITOR) {
		ic->ic_bss->ni_chan = ic->ic_ibss_chan;
		ieee80211_new_state(ic, IEEE80211_S_RUN, -1);
		MWX_DEV_LOG(sc, "init monitor state run\n");
		return 0;
	}

#ifdef AIRPORT_WCL
	/*
	 * macOS asks for every scan and every join (beginWCLScan(),
	 * startWCLJoin()). A sweep started here kept the radio busy on all
	 * 41 channels for 5.6 s, and the scan or join macOS asked for right
	 * after the interface came up (power on, system wake, private address
	 * change) waited for it. Enter the scan state idle instead, the way
	 * ieee80211_end_scan() leaves it when nothing was asked for: the next
	 * request starts its own hardware scan through ieee80211_next_scan().
	 */
	/*
	 * mwx_preinit() fills the channel table only now, after
	 * ieee80211_ifattach() found it empty: compute the active channel set
	 * from it, as ieee80211_begin_scan() did. Without it
	 * ieee80211_next_scan() finds no channel and ends every scan at once.
	 */
	if (IFM_MODE(ic->ic_media.ifm_cur->ifm_media) == IFM_AUTO)
		ic->ic_curmode = IEEE80211_MODE_AUTO;
	ieee80211_setmode(ic, (enum ieee80211_phymode)ic->ic_curmode);
	ieee80211_reset_scan(ifp);
	ic->ic_state = IEEE80211_S_SCAN;
	sc->sc_ns_state = IEEE80211_S_SCAN;
	ic->ic_wcl_scan_idle = true;
	{
		int active = 0;
		for (int ch = 0; ch <= IEEE80211_CHAN_MAX; ch++)
			if (isset(ic->ic_chan_active, ch))
				active++;
		MWX_DEV_LOG(sc, "init: scan state idle until macOS asks, %d channels\n",
		    active);
	}
#ifdef MTK_AWDL
	/* The station's reception is set again above: the AWDL radio may open windows again. */
	if (sc->sc_awdl != NULL)
		mwx_awdl_resume(sc);
#endif
#else
	MWX_DEV_LOG(sc, "init begin scan\n");
	ieee80211_begin_scan(ifp);
	MWX_DEV_LOG(sc, "init begin scan returned state=%d flags=0x%x sc_flags=0x%x\n",
	    ic->ic_state, ifp->if_flags, sc->sc_flags);

	/*
	 * ieee80211_begin_scan() ends up scheduling mwx_newstate_task().
	 * Wait until the transition to SCAN state has completed.
	 */
#endif

	return 0;
}

void
mwx_stop(struct ifnet *ifp)
{
	mwx_stop_internal(ifp, false);
}

static void
mwx_stop_internal(struct ifnet *ifp, bool teardown_owner)
{
	struct mwx_softc *sc = (struct mwx_softc *)ifp->if_softc;
	struct ieee80211com *ic = &sc->sc_ic;
	int s = splnet();
	int was_running = (ifp->if_flags & IFF_RUNNING) != 0;

	MwxRxAmsduTransition transition(sc, teardown_owner);
	if (!transition.ok()) {
		splx(s);
		return;
	}
	DPRINTF("%s: stop\n", DEVNAME(sc));
#ifdef MTK_AWDL
	/*
	 * Interface down, system sleep, shutdown, reset and detach all come here: the AWDL radio ends its window
	 * and gives a remain-on-channel period back while the firmware still answers, before the deauth below.
	 */
	if (sc->sc_awdl != NULL)
		mwx_awdl_quiesce(sc);
#endif
	sc->sc_stopping = 1;

	/*
	 * Tell the AP we are leaving (shutdown, interface down). Otherwise it
	 * keeps our session and ignores the first association after reboot.
	 */
	if (was_running && ic->ic_state == IEEE80211_S_RUN &&
	    ic->ic_opmode == IEEE80211_M_STA && ic->ic_bss != NULL) {
		int drv = IEEE80211_SEND_MGMT(ic, ic->ic_bss,
		    IEEE80211_FC0_SUBTYPE_DEAUTH, IEEE80211_REASON_AUTH_LEAVE);
		mwx_start(ifp);
		delay(30 * 1000);
		MWX_DEV_LOG(sc, "stop: deauth sent to AP rv=%d\n", drv);
	}

	/* Cancel scheduled tasks and let any stale tasks finish up. */
	if (sc->sc_nswq != NULL) {
		task_del(sc->sc_nswq, &sc->sc_newstate_task);
		task_del(sc->sc_nswq, &sc->sc_bgscan_done_task);
		task_del(sc->sc_nswq, &sc->sc_setkey_task);
		task_del(sc->sc_nswq, &sc->sc_reset_task);
	}
	task_del(systq, &sc->sc_scan_task);
	timeout_del(&sc->sc_reset_to);

	ifp->if_timer = 0;
	ifp->if_flags &= ~IFF_RUNNING;
	ifq_clr_oactive(&ifp->if_snd);

	if (was_running) {
		struct mwx_node *mn;

		free(sc->sc_bgscan_arg, M_DEVBUF, sc->sc_bgscan_arg_size);
		sc->sc_bgscan_arg = NULL;
		sc->sc_bgscan_arg_size = 0;
		if (sc->sc_flags & MWX_FLAG_SCANNING) {
			mt7921_mcu_hw_scan_cancel(sc);
			sc->sc_flags &= ~MWX_FLAG_SCANNING;
		}

		mn = &sc->sc_vif.vif_mn;
		mt7921_mcu_uni_add_dev(sc, &sc->sc_vif, mn, 0);
		mwx_mcu_set_deep_sleep(sc, 1);
		mt7921_mcu_set_mac_enable(sc, 0, 0);
	}

	/* Detach defers software key cleanup until its IRQ source is removed.
	 * Group-key cleanup zeroes each key immediately after delete_key returns. */
	if (!teardown_owner)
		sc->sc_newstate(ic, IEEE80211_S_INIT, -1);
	sc->sc_ns_state = IEEE80211_S_INIT;
	ifp->if_timer = 0;

#ifdef AIRPORT_WCL
	/* A stopped firmware scan cannot deliver its completion event. */
	ic->ic_flags &= ~(IEEE80211_F_BGSCAN | IEEE80211_F_ASCAN);
	ic->ic_wcl_scan_requested = false;
	ic->ic_wcl_scan_active = false;
	ic->ic_wcl_scan_ssid_length = 0;
	ic->ic_wcl_scan_idle = false;
	ic->ic_wcl_join_channel = 0;
	if (ic->ic_event_handler)
		ic->ic_event_handler(ic, IEEE80211_EVT_DRIVER_STOPPED, NULL);
#endif

	sc->sc_stopping = 0;
	splx(s);
}

void
mwx_watchdog(struct ifnet *ifp)
{
	ifp->if_timer = 0;
	ieee80211_watchdog(ifp);
}

static inline int mwx_dma_free_slots(struct mwx_queue *);

/* Free TXWIs and TX ring slots for at least `need` frames. */
static inline int
mwx_tx_has_room(struct mwx_softc *sc, u_int need)
{
	return sc->sc_txwi.mt_nfree >= need &&
	    mwx_dma_free_slots(&sc->sc_txq) >= (int)need;
}

/* Restart output after TX completions freed enough room. */
static void
mwx_tx_kick(struct mwx_softc *sc)
{
	struct ifnet *ifp = &sc->sc_ic.ic_if;

	if (!ifq_is_oactive(&ifp->if_snd) || !mwx_tx_has_room(sc, 1))
		return;
	sc->sc_tx_kick_cnt++;
	ifq_clr_oactive(&ifp->if_snd);
	mwx_start(ifp);
#ifdef __PRIVATE_SPI__
	if (ifp->iface != NULL)
		ifp->iface->signalOutputThread();
#endif
}

void
mwx_start(struct ifnet *ifp)
{
	struct mwx_softc *sc = (struct mwx_softc *)ifp->if_softc;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_node *ni;
	struct ether_header *eh;
	struct mbuf *m;

	if (!(ifp->if_flags & IFF_RUNNING) || ifq_is_oactive(&ifp->if_snd))
		return;

	for (;;) {
		/*
		 * Stop pulling packets while the TXWI pool or the TX ring is
		 * full instead of dropping them (TCP saw that as loss); the TX
		 * free event restarts output (mwx_tx_kick).
		 */
		if (!mwx_tx_has_room(sc, 1)) {
			ifq_set_oactive(&ifp->if_snd);
			sc->sc_tx_full_cnt++;
			break;
		}

		/* need to send management frames even if we're not RUNning */
		m = mq_dequeue(&ic->ic_mgtq);
		if (m) {
			ni = (struct ieee80211_node *)mbuf_pkthdr_rcvif(m);
			goto sendit;
		}

		if (ic->ic_state != IEEE80211_S_RUN ||
		    (ic->ic_xflags & IEEE80211_F_TX_MGMT_ONLY))
			break;

		m = ifq_dequeue(&ifp->if_snd);
		if (!m)
			break;
		if (mbuf_len(m) < sizeof (*eh) &&
		    (m = m_pullup(m, sizeof (*eh))) == NULL) {
			ifp->if_oerrors++;
			continue;
		}
#if NBPFILTER > 0
		if (ifp->if_bpf != NULL)
			bpf_mtap(ifp->if_bpf, m, BPF_DIRECTION_OUT);
#endif
		if ((m = ieee80211_encap(ifp, m, &ni)) == NULL) {
			ifp->if_oerrors++;
			continue;
		}

 sendit:
#if NBPFILTER > 0
		if (ic->ic_rawbpf != NULL)
			bpf_mtap(ic->ic_rawbpf, m, BPF_DIRECTION_OUT);
#endif
		int rv = mwx_tx(sc, m, ni);
		/* mwx_tx copies node metadata into the descriptor; completion uses
		 * the packet/token only, so release the enqueue reference here. */
		ieee80211_release_node(ic, ni);
		if (rv != 0) {
			ifp->if_oerrors++;
			if (ifp->netStat)
				ifp->netStat->outputErrors++;
			continue;
		}
		if (ifp->netStat)
			ifp->netStat->outputPackets++;

		if (ifp->if_flags & IFF_UP)
			ifp->if_timer = 1;
	}
}

int
mwx_ioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
{
	int s, err = 0;

	s = splnet();
	switch (cmd) {
	case SIOCSIFADDR:
		ifp->if_flags |= IFF_UP;
		/* FALLTHROUGH */
	case SIOCSIFFLAGS:
		if (ifp->if_flags & IFF_UP) {
			if (!(ifp->if_flags & IFF_RUNNING)) {
				mwx_stop(ifp);
				err = mwx_init(ifp);
			}
		} else {
			if (ifp->if_flags & IFF_RUNNING)
				mwx_stop(ifp);
		}
		break;
	default:
		err = ieee80211_ioctl(ifp, cmd, data);
		break;
	}

	if (err == ENETRESET) {
		err = 0;
		if ((ifp->if_flags & (IFF_UP | IFF_RUNNING)) ==
		    (IFF_UP | IFF_RUNNING)) {
			mwx_stop(ifp);
			err = mwx_init(ifp);
		}
	}
	splx(s);
	return err;
}

int
mwx_media_change(struct ifnet *ifp)
{
	struct mwx_softc *sc = (struct mwx_softc *)ifp->if_softc;
	struct ieee80211com *ic = &sc->sc_ic;
	int err;

	err = ieee80211_media_change(ifp);
	if (err != ENETRESET)
		return err;

	/* TODO lot more handling here */
	if (ic->ic_fixed_mcs != -1) {
		;
	} else if (ic->ic_fixed_rate != -1) {
		;
	}
	if ((ifp->if_flags & (IFF_UP | IFF_RUNNING)) ==
	    (IFF_UP | IFF_RUNNING)) {
		/* XXX could be a bit harsh */
		mwx_stop(ifp);
		err = mwx_init(ifp);
	}
	return err;
}

/*
 * net80211 specific functions.
 */

struct ieee80211_node *
mwx_node_alloc(struct ieee80211com *ic)
{
	/* XXX this is just wrong */
	static uint32_t wcid = 0;
	struct mwx_softc *sc = (struct mwx_softc *)ic->ic_softc;
	struct mwx_node *mn;

	mn = (struct mwx_node *)malloc(sizeof(struct mwx_node), M_DEVBUF, M_NOWAIT | M_ZERO);
	if (mn == NULL)
		return NULL;
	mn->wcid = 1 + wcid++ % (MWX_WTBL_STA - 1);

	/* init WCID table entry */
	mwx_mac_wtbl_update(sc, mn->wcid);

	return &mn->ni;
}

void
mwx_newassoc(struct ieee80211com *ic, struct ieee80211_node *ni, int isnew)
{
	struct mwx_softc *sc = (struct mwx_softc *)ic->ic_softc;
	struct mwx_node *mn = (struct mwx_node *)ni;
	uint16_t wcid = 0;

	if (isnew && ni->ni_associd != 0) {
		/* only interested in true associations */
		wcid = IEEE80211_AID(ni->ni_associd);

	}
	printf("%s: new assoc isnew=%d addr=%s WCID=%d\n", DEVNAME(sc),
	    isnew, ether_sprintf(ni->ni_macaddr), mn->wcid);

	/* XXX TODO rate handling here */
}

#ifndef IEEE80211_STA_ONLY
void
mwx_node_leave(struct ieee80211com *ic, struct ieee80211_node *ni)
{
#if 0
	struct mwx_softc *sc = (struct mwx_softc *)ic->ic_softc;
	struct mwx_node *mn = (struct mwx_node *)ni;
	uint16_t wcid = mn->wcid;

	/* TODO clear WCID */
#endif
}
#endif

int
mwx_scan(struct mwx_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ifnet *ifp = &ic->ic_if;
	int rv;

	if (sc->sc_flags & MWX_FLAG_BGSCAN) {
		rv = mt7921_mcu_hw_scan_cancel(sc);
		if (rv) {
			printf("%s: could not abort background scan\n",
			    DEVNAME(sc));
			return rv;
		}
	}

	rv = mt7921_mcu_hw_scan(sc, 0);
	if (rv) {
		printf("%s: could not initiate scan\n", DEVNAME(sc));
		return rv;
	}

	/*
	 * The current mode might have been fixed during association.
	 * Ensure all channels get scanned.
	 */
	if (IFM_MODE(ic->ic_media.ifm_cur->ifm_media) != IFM_AUTO)
		ieee80211_setmode(ic, IEEE80211_MODE_AUTO);

	sc->sc_flags |= MWX_FLAG_SCANNING;
	if (ifp->if_flags & IFF_DEBUG)
		printf("%s: %s -> %s\n", ifp->if_xname,
		    ieee80211_state_name[ic->ic_state],
		    ieee80211_state_name[IEEE80211_S_SCAN]);
	if ((sc->sc_flags & MWX_FLAG_BGSCAN) == 0) {
		ieee80211_set_link_state(ic, LINK_STATE_DOWN);
		ieee80211_node_cleanup(ic, ic->ic_bss);
	}
	ic->ic_state = IEEE80211_S_SCAN;

	return 0;
}

int
mwx_newstate(struct ieee80211com *ic, enum ieee80211_state nstate, int arg)
{
	struct mwx_softc *sc = (struct mwx_softc *)ic->ic_softc;

	/*
	 * Prevent attempts to transition towards the same state, unless
	 * we are scanning in which case a SCAN -> SCAN transition
	 * triggers another scan iteration. And AUTH -> AUTH is needed
	 * to support band-steering.
	 */
	if (sc->sc_ns_state == nstate && nstate != IEEE80211_S_SCAN &&
	    nstate != IEEE80211_S_AUTH)
		return 0;

	if (ic->ic_state == IEEE80211_S_RUN) {
		/* cancel other tasks here */
	}

	sc->sc_ns_state = nstate;
	sc->sc_ns_arg = arg;

	task_add(sc->sc_nswq, &sc->sc_newstate_task);
	return 0;
}

struct mwx_task_call {
	void	(*fn)(void *);
	void	*arg;
};

static IOReturn
mwx_task_gated(OSObject *, void *a, void *, void *, void *)
{
	struct mwx_task_call *c = (struct mwx_task_call *)a;

	c->fn(c->arg);
	return kIOReturnSuccess;
}

/*
 * The task queue runs on a thread_call, off the work loop. These tasks send
 * MCU commands and free the firmware's replies, which the interrupt handler
 * stores into sc_mcu_wait[]. When a task and the interrupt overlapped, a reply
 * mbuf was freed while still referenced and the kernel page-faulted in
 * m_freem(): mwx_reset_task() -> mwx_init() during a stop/wake. Run each task
 * through the main command gate, the way enable() already runs mwx_init();
 * tsleep_nsec() opens the gate while a task waits for the firmware, so the
 * interrupt still delivers the replies.
 */
static void
mwx_task_run_gated(struct mwx_softc *sc, void (*fn)(void *), void *arg)
{
	IOCommandGate *gate = sc->sc_rx_amsdu_gate;
	struct mwx_task_call c = { fn, arg };

	if (gate != NULL && !gate->getWorkLoop()->inGate() &&
	    gate->runAction(mwx_task_gated, &c) == kIOReturnSuccess)
		return;
	fn(arg);
}

static void mwx_newstate_task_gated(void *a) { mwx_task_run_gated((struct mwx_softc *)a, mwx_newstate_task, a); }
static void mwx_bgscan_done_task_gated(void *a) { mwx_task_run_gated((struct mwx_softc *)a, mwx_bgscan_done_task, a); }
static void mwx_setkey_task_gated(void *a) { mwx_task_run_gated((struct mwx_softc *)a, mwx_setkey_task, a); }
static void mwx_reset_task_gated(void *a) { mwx_task_run_gated((struct mwx_softc *)a, mwx_reset_task, a); }
static void mwx_end_scan_task_gated(void *a) { mwx_task_run_gated((struct mwx_softc *)a, mwx_end_scan_task, a); }

/*
 * The association protects management frames (802.11w) and we are leaving
 * it. net80211 queues its Deauthentication only after the peer record, and
 * the pairwise key with it, is gone from the firmware: that frame leaves
 * unprotected and the access point, as it must, ignores it. It would keep
 * our association, answer the next association request with "try again
 * later" and spend a second on an SA Query nobody can answer any more.
 * Send the Deauthentication now, while the firmware can still encrypt it,
 * and give it the time to leave, as before a switch to another access
 * point. Not when the access point itself ended the association.
 *
 * From here on management frames are unprotected again in both directions.
 */
static void
mwx_mgmt_prot_leave(struct mwx_softc *sc, int arg)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_node *ni = ic->ic_bss;

	if (ni == NULL)
		return;
	if ((ni->ni_flags & IEEE80211_NODE_TXMGMTPROT) &&
	    arg != IEEE80211_FC0_SUBTYPE_DEAUTH &&
	    arg != IEEE80211_FC0_SUBTYPE_DISASSOC) {
		IEEE80211_SEND_MGMT(ic, ni, IEEE80211_FC0_SUBTYPE_DEAUTH,
		    IEEE80211_REASON_AUTH_LEAVE);
		mwx_start(&ic->ic_if);
		delay(30 * 1000);
		MWX_DEV_LOG(sc, "leave BSS: protected deauth sent\n");
	}
	ni->ni_flags &= ~(IEEE80211_NODE_TXMGMTPROT |
	    IEEE80211_NODE_RXMGMTPROT);
	ieee80211_sta_sa_query_stop(ic);
}

void
mwx_newstate_task(void *ptr)
{
	struct mwx_softc *sc = (struct mwx_softc *)ptr;
	struct ieee80211com *ic = &sc->sc_ic;
	enum ieee80211_state ostate = ic->ic_state;
	enum ieee80211_state nstate = sc->sc_ns_state;
	int arg = sc->sc_ns_arg;
	int s = splnet();
	int rv = 0;

	MwxRxAmsduTransition transition(sc);
	if (!transition.ok()) {
		splx(s);
		return;
	}
	switch (ostate) {
	case IEEE80211_S_RUN:
		if (nstate != ostate)
			mwx_mcu_set_deep_sleep(sc, 1);
#ifdef MTK_AWDL
		/* A reception the AWDL radio holds open between its windows is on this station's channel: it closes now. */
		if (nstate != ostate && sc->sc_awdl != NULL)
			mwx_awdl_rx_release(sc);
#endif
		/*
		 * Leaving the BSS: tell the firmware the BSS is gone and remove
		 * the peer station record (like Linux does on disassoc). Without
		 * this the next association reused wcid 1 on top of the stale
		 * record and the chip stopped decrypting RX in hardware.
		 */
		if (nstate < IEEE80211_S_RUN &&
		    ic->ic_opmode == IEEE80211_M_STA) {
			mwx_mgmt_prot_leave(sc, arg);
			int brv = mt7921_mcu_set_associated_bss(sc, 0);
			int srv = mt7921_mac_sta_update(sc, ic->ic_bss, 0, 0);
			MWX_DEV_LOG(sc, "leave BSS: bss_off=%d sta_remove=%d\n",
			    brv, srv);
			sc->sc_sta_added = 0;
		}
		break;
	case IEEE80211_S_SCAN:
		if (nstate == ostate) {
			if (sc->sc_flags & MWX_FLAG_SCANNING) {
				splx(s);
				return;
			}
		}
		break;
	default:
		break;
	}

	DPRINTF("%s: %s %d -> %d\n", DEVNAME(sc), __func__, ostate, nstate);
	MWX_DEV_LOG(sc, "newstate task %d -> %d arg=%d\n", ostate, nstate, arg);

	switch (nstate) {
	case IEEE80211_S_INIT:
		break;
	case IEEE80211_S_SCAN:
		rv = mwx_scan(sc);
		if (rv)
			break;
		break;
	case IEEE80211_S_AUTH:
		rv = mt7921_set_channel(sc);
		if (rv)
			break;
		mwx_mcu_set_deep_sleep(sc, 0);
		/*
		 * A previous attempt (failed association, or AUTH retry on
		 * another AP) may have left the peer record in firmware; adding
		 * a new peer on top of it breaks hardware decryption.
		 */
		/*
		 * Starting a new connection: send a deauth first so the AP drops
		 * any session it kept from before (e.g. across a reboot); it
		 * otherwise ignored our first association request. An AP that
		 * has no session for us just ignores the frame.
		 */
		if (ostate == IEEE80211_S_SCAN && ic->ic_bss != NULL) {
			IEEE80211_SEND_MGMT(ic, ic->ic_bss,
			    IEEE80211_FC0_SUBTYPE_DEAUTH,
			    IEEE80211_REASON_AUTH_LEAVE);
			mwx_start(&ic->ic_if);
			MWX_DEV_LOG(sc, "AUTH: pre-auth deauth sent to %s\n",
			    ether_sprintf(ic->ic_bss->ni_macaddr));
		}
		if (sc->sc_sta_added) {
			int srv = mt7921_mac_sta_update(sc, sc->sc_ic.ic_bss, 0, 0);
			MWX_DEV_LOG(sc, "AUTH: removed stale peer record rv=%d\n", srv);
			sc->sc_sta_added = 0;
		}
		if (mt7921_mac_sta_update(sc, sc->sc_ic.ic_bss, 1, 1) == 0)
			sc->sc_sta_added = 1;
		break;
	case IEEE80211_S_ASSOC:
		/*
		 * Keep full power, as AUTH left it and RUN wants it: the
		 * association response is due now. With deep sleep allowed
		 * here, the answer to the first association request never
		 * reached us on 5 GHz, although the access point went on with
		 * the 4-way handshake, and the request went out again a second
		 * later; an access point that negotiated 802.11w refuses that
		 * second request.
		 */
		break;
	case IEEE80211_S_RUN:
		if (ic->ic_opmode == IEEE80211_M_MONITOR)
			break;

		mt7921_mcu_hw_scan_cancel(sc);
		mwx_mcu_set_deep_sleep(sc, 0);
		{
			struct ieee80211_node *bni = ic->ic_bss;
			MtkApCapabilitySummary raw = {};
			const bool cache_valid = mtkParseApCapabilities(
			    bni->ni_rsnie_tlv, bni->ni_rsnie_tlv_len, raw);
			MWX_DEV_LOG(sc,
			    "AP raw-cache valid=%d bytes=%u vht=%d caps=0x%08x operation=%d width=%u center0=%u center1=%u he_cap_bytes=%u node_chan2=%u\n",
			    cache_valid, bni->ni_rsnie_tlv_len, raw.vht, raw.vhtCaps,
			    raw.operation, raw.width, raw.center0, raw.center1,
			    raw.heCapabilityBytes, bni->ni_vht_chan2);
			MWX_DEV_LOG(sc,
			    "RUN peer ht=%d vht=%d chw=%d vht_chanwidth=%u vht_chan1=%u center_freq1=%u htcaps=0x%04x vhtcaps=0x%08x\n",
			    !!(bni->ni_flags & IEEE80211_NODE_HT),
			    !!(bni->ni_flags & IEEE80211_NODE_VHT), bni->ni_chw,
			    bni->ni_vht_chanwidth, bni->ni_vht_chan1,
			    bni->ni_chan->ic_center_freq1, bni->ni_htcaps,
			    bni->ni_vhtcaps);
			uint8_t rbw, rcenter;
			mwx_node_chan_width(bni, &rbw, &rcenter);
			if (rbw != CMD_CBW_20MHZ) {
				int crv = mt7921_mcu_set_chan_info(sc,
				    MCU_EXT_CMD_CHANNEL_SWITCH);
				MWX_DEV_LOG(sc, "RUN switch to %s MHz center=%u rv=%d\n",
				    rbw == CMD_CBW_80MHZ ? "80" : "40", rcenter, crv);
			}
		}
		rv = mt7921_mcu_set_associated_bss(sc, 1);
		if (!rv)
			rv = mt7921_mcu_set_associated_peer(sc);
		/* WMM must be programmed after BSS_INFO enables TXCMD mode. */
		if (!rv)
			rv = mt7921_mcu_set_tx(sc, &sc->sc_vif);
		if (rv)
			break;
		mt7921_mcu_set_rts_thresh(sc, 0x92b, 0);
#ifdef MTK_AWDL
		/* The access point's country, before net80211 brings the link up: the Family's AWDL channels follow it. */
		if (mtkAwdlEnabled())
			mwx_ap_country(sc, ic->ic_bss);
#endif
		break;
	}

	if (rv)
		mwx_reset(sc);
	else
		sc->sc_newstate(ic, nstate, arg);
	splx(s);
}

int
mwx_bgscan(struct ieee80211com *ic)
{
	struct mwx_softc *sc = (struct mwx_softc *)ic->ic_softc;
	uint64_t now_ns = 0;
	int err;

	if (sc->sc_flags & MWX_FLAG_SCANNING)
		return 0;

	/*
	 * While associated, a full hardware scan keeps the radio off the
	 * home channel for ~11 s. The client asks for one every ~20 s to
	 * refresh its network list, which halved throughput. Allow at most
	 * one background scan per 2 minutes while in RUN.
	 */
	if (ic->ic_state == IEEE80211_S_RUN) {
		uint64_t min_ns = 120ULL * 1000000000ULL;

#ifdef AIRPORT_WCL
		/*
		 * macOS asks for a few channels at a time (well under a second
		 * off channel); only full sweeps get the 2 minute limit.
		 */
		if (ic->ic_wcl_scan_requested && ic->ic_wcl_scan_restricted)
			min_ns = 10ULL * 1000000000ULL;
#endif
		absolutetime_to_nanoseconds(mach_absolute_time(), &now_ns);
		if (sc->sc_last_bgscan_ns != 0 &&
		    now_ns - sc->sc_last_bgscan_ns < min_ns)
			return EBUSY;
	}

	err = mt7921_mcu_hw_scan(sc, 1);
	if (err) {
		printf("%s: could not initiate scan\n", DEVNAME(sc));
		return err;
	}

	/* Failed submissions must remain immediately retryable, per adapter. */
	if (now_ns != 0)
		sc->sc_last_bgscan_ns = now_ns;
	sc->sc_flags |= MWX_FLAG_BGSCAN;
	return 0;
}

void
mwx_bgscan_done(struct ieee80211com *ic,
    struct ieee80211_node_switch_bss_arg *arg, size_t arg_size)
{
	struct mwx_softc *sc = (struct mwx_softc *)ic->ic_softc;

	free(sc->sc_bgscan_arg, M_DEVBUF, sc->sc_bgscan_arg_size);
	sc->sc_bgscan_arg = arg;
	sc->sc_bgscan_arg_size = arg_size;
	task_add(sc->sc_nswq, &sc->sc_bgscan_done_task);
}

void
mwx_bgscan_done_task(void *arg)
{
	struct mwx_softc *sc = (struct mwx_softc *)arg;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_node *ni = ic->ic_bss;
	int err = 0;
	int s = splnet();

#ifdef AIRPORT_WCL
	/* A WCL lock can arrive while this automatic BSS switch is queued. */
	if (ic->ic_state == IEEE80211_S_RUN && ic->ic_wcl_roam_locked) {
		ic->ic_flags &= ~IEEE80211_F_BGSCAN;
		ic->ic_xflags &= ~IEEE80211_F_TX_MGMT_ONLY;
		err = EBUSY;
		goto done;
	}
#endif
	if (ic->ic_state != IEEE80211_S_RUN ||
	    (ic->ic_flags & IEEE80211_F_BGSCAN) == 0) {
		err = ENXIO;
		goto done;
	}

	/*
	 * net80211 queued a deauth for the current AP. Push it out on the
	 * current channel before switching; otherwise the AP keeps a stale
	 * session and rejects us (disassoc 8 / deauth 9) when we come back.
	 */
	mwx_start(&ic->ic_if);
	delay(30 * 1000);

	/*
	 * Remove installed crypto keys while we still have access to them.
	 * Once mwx_newstate() is entered ic_bss will already contain
	 * information about our next AP.
	 */
	if (ic->ic_flags & IEEE80211_F_RSNON) {
		struct ieee80211_key *k;

		if (ni->ni_pairwise_key.k_cipher != IEEE80211_CIPHER_NONE)
			(*ic->ic_delete_key)(ic, ni, &ni->ni_pairwise_key);

		if (ic->ic_def_txkey == 1 || ic->ic_def_txkey == 2) {
			k = &ic->ic_nw_keys[ic->ic_def_txkey];
			if ((k->k_flags & IEEE80211_KEY_GROUP) &&
			    k->k_cipher == IEEE80211_CIPHER_CCMP)
				(*ic->ic_delete_key)(ic, ni, k);
		}

		if (ic->ic_igtk_kid == 4 || ic->ic_igtk_kid == 5) {
			k = &ic->ic_nw_keys[ic->ic_igtk_kid];
			if (k->k_flags & IEEE80211_KEY_IGTK)
				(*ic->ic_delete_key)(ic, ni, k);
		}

		ni->ni_port_valid = 0;
		ni->ni_flags &= ~IEEE80211_NODE_TXRXPROT;
		ni->ni_flags &= ~IEEE80211_NODE_TXMGMTPROT;
		ni->ni_flags &= ~IEEE80211_NODE_RXMGMTPROT;
		ni->ni_rsn_supp_state = RSNA_SUPP_INITIALIZE;
	}

	/*
	 * The local net80211 port switches BSS from ni_unref_cb when the
	 * current node's final reference is released. Install the same
	 * callback after the driver has removed hardware keys.
	 */
	ni->ni_unref_arg = sc->sc_bgscan_arg;
	ni->ni_unref_arg_size = sc->sc_bgscan_arg_size;
	ni->ni_unref_cb = ieee80211_node_switch_bss;
	sc->sc_bgscan_arg = NULL;
	sc->sc_bgscan_arg_size = 0;
	if (ni->ni_flags & IEEE80211_NODE_MFP)
		ieee80211_node_switch_bss(ic, ni);

 done:
	if (err) {
		free(sc->sc_bgscan_arg, M_DEVBUF, sc->sc_bgscan_arg_size);
		sc->sc_bgscan_arg = NULL;
		sc->sc_bgscan_arg_size = 0;
	}
	splx(s);
}

int
mwx_set_key(struct ieee80211com *ic, struct ieee80211_node *ni,
    struct ieee80211_key *k)
{
	struct mwx_softc *sc = (struct mwx_softc *)ic->ic_softc;
	struct mwx_setkey_task_arg *a;

	DPRINTF("%s: set_key: k_id %d, k_flags %x k_cipher %d\n",
	    DEVNAME(sc), k->k_id, k->k_flags, k->k_cipher);

	/*
	 * The integrity group key (802.11w) stays in software. The firmware
	 * hands group-addressed management frames over as received, MMIE
	 * included, and net80211 checks their MIC (BIP). It must not go
	 * through the key task: a key record with BIP alone replaces the
	 * CCMP key of that station in the firmware (Linux sends both ciphers
	 * in one record) and unicast traffic would stop being decrypted.
	 */
	if (k->k_flags & IEEE80211_KEY_IGTK) {
		int err;

		if (k->k_cipher != IEEE80211_CIPHER_BIP || k->k_len != 16 ||
		    (k->k_id != 4 && k->k_id != 5))
			return EINVAL;
		err = (k->k_flags & IEEE80211_KEY_SWCRYPTO) ? 0 :
		    ieee80211_set_key(ic, ni, k);
		MWX_DEV_LOG(sc, "igtk id=%d sw crypto setup=%d\n", k->k_id, err);
		return err;
	}

	/*
	 * MT7921 does not reliably decrypt group-addressed frames with the
	 * GTK installed in hardware; those arrive without RXI_HWDEC and were
	 * dropped as wepfail (broadcast ARP / DHCP never reached the stack).
	 * Also set up the software crypto context for group keys so net80211
	 * can decrypt them itself. Group keys are RX-only for a station.
	 */
	/*
	 * Same fallback for the pairwise key: after a reassociation the chip
	 * sometimes stops decrypting unicast frames too. net80211 never
	 * encrypts TX in software, so this only affects RX.
	 */
	if (!(k->k_flags & IEEE80211_KEY_SWCRYPTO)) {
		int err = ieee80211_set_key(ic, ni, k);
		MWX_DEV_LOG(sc, "%s key id=%d cipher=%d sw crypto setup=%d\n",
		    (k->k_flags & IEEE80211_KEY_GROUP) ? "group" : "pairwise",
		    k->k_id, k->k_cipher, err);
	}

	if ((sc->sc_setkey_cur + 1) % nitems(sc->sc_setkey_arg) ==
	    sc->sc_setkey_tail)
		return ENOSPC;

	a = &sc->sc_setkey_arg[sc->sc_setkey_cur];
	a->ni = ni;
	a->k = k;
	sc->sc_setkey_cur = (sc->sc_setkey_cur + 1) % nitems(sc->sc_setkey_arg);
	task_add(sc->sc_nswq, &sc->sc_setkey_task);
	return EBUSY;
}

void
mwx_delete_key(struct ieee80211com *ic, struct ieee80211_node *ni,
    struct ieee80211_key *k)
{
	struct mwx_softc *sc = (struct mwx_softc *)ic->ic_softc;

	DPRINTF("%s: delete_key: k_id %d, k_flags %x k_cipher %d\n",
	    DEVNAME(sc), k->k_id, k->k_flags, k->k_cipher);

	MwxRxAmsduTransition transition(sc);
	if (!transition.ok()) {
		/* Only final ifdetach cleanup, after interrupts were removed, may
		 * release software keys following canceled transition acquisition. */
		if (mwx_rx_amsdu_key_cleanup_only(sc) &&
		    (k->k_flags & IEEE80211_KEY_SWCRYPTO))
			ieee80211_delete_key(ic, ni, k);
		return;
	}
	/*
	 * The integrity group key was never given to the firmware (see
	 * mwx_set_key()), and the delete command for its station would
	 * remove the pairwise key instead.
	 */
	if (k->k_flags & IEEE80211_KEY_IGTK) {
		ieee80211_delete_key(ic, ni, k);
		return;
	}
	mt7921_mcu_sta_key_delete(sc, ni, k);
	/* release the software crypto context set up in mwx_set_key() */
	if (k->k_flags & IEEE80211_KEY_SWCRYPTO)
		ieee80211_delete_key(ic, ni, k);
}

void
mwx_setkey_task(void *arg)
{
	struct mwx_softc *sc = (struct mwx_softc *)arg;
	struct mwx_setkey_task_arg *a;
	int s = splnet();
	int rv;

	while (sc->sc_setkey_tail != sc->sc_setkey_cur) {
		MwxRxAmsduTransition transition(sc);
		if (!transition.ok()) {
			splx(s);
			return;
		}
		a = &sc->sc_setkey_arg[sc->sc_setkey_tail];
		rv = mt7921_mcu_sta_key_update(sc, a->ni, a->k);
		/*
		 * From here on the firmware decrypts with this key and reports
		 * its packet numbers, which start over. During a rekey, frames
		 * decrypted with the previous pairwise key while this one was
		 * queued have already raised the replay counters of the new
		 * key: left there, every frame of the new key would look like
		 * a replay to ieee80211_input_hwdecrypt().
		 */
		if (rv == 0 && !(a->k->k_flags & IEEE80211_KEY_GROUP)) {
			memset(a->k->k_rsc, 0, sizeof(a->k->k_rsc));
			/* protected management frames count separately */
			a->k->k_mgmt_rsc = 0;
		}
		a->ni = NULL;
		a->k = NULL;

		sc->sc_setkey_tail = (sc->sc_setkey_tail + 1) %
		    nitems(sc->sc_setkey_arg);
	}

	splx(s);
}

#if NBPFILTER > 0
void
mwx_radiotap_attach(struct mwx_softc *sc)
{
	bpfattach(&sc->sc_drvbpf, &sc->sc_ic.ic_if, DLT_IEEE802_11_RADIO,
	    sizeof(struct ieee80211_frame) + IEEE80211_RADIOTAP_HDRLEN);

	sc->sc_rxtap_len = sizeof sc->sc_rxtapu;
	sc->sc_rxtap.wr_ihdr.it_len = htole16(sc->sc_rxtap_len);
	sc->sc_rxtap.wr_ihdr.it_present = htole32(MWX_RX_RADIOTAP_PRESENT);

	sc->sc_txtap_len = sizeof sc->sc_txtapu;
	sc->sc_txtap.wt_ihdr.it_len = htole16(sc->sc_txtap_len);
	sc->sc_txtap.wt_ihdr.it_present = htole32(MWX_TX_RADIOTAP_PRESENT);
}
#endif

/*
 * Copy a TX frame into one freshly allocated cluster so the firmware sees a
 * single contiguous buffer. Experiment: the WFDMA stalled on the first DHCP
 * frame (a multi-mbuf chain) on MT7921.
 */
static struct mbuf *
mwx_tx_linearize(struct mbuf *m)
{
	size_t len = mbuf_pkthdr_len(m);
	struct mbuf *n;

	if (mbuf_next(m) == NULL || len == 0 || len > MCLBYTES * 2)
		return m;
	{
		static uint32_t lin_log_count;
		if (lin_log_count < 48) {
			struct mbuf *c;
			char chain[96];
			size_t off = 0;

			chain[0] = 0;
			for (c = m; c != NULL && off < sizeof(chain) - 12;
			    c = mbuf_next(c))
				off += snprintf(chain + off, sizeof(chain) - off,
				    "%zu%s,", mbuf_len(c),
				    (mbuf_flags(c) & MBUF_EXT) ? "e" : "");
			MWX_LOG("tx linearize len=%zu chain=%s\n", len, chain);
			lin_log_count++;
		}
	}
	n = MCLGETL(NULL, M_DONTWAIT, len);
	if (n == NULL)
		return m;
	if (mbuf_next(n) != NULL) {
		m_freem(n);
		return m;
	}
	mbuf_setlen(n, len);
	mbuf_pkthdr_setlen(n, len);
	if (mbuf_copydata(m, 0, len, mtod(n, void *)) != 0) {
		m_freem(n);
		return m;
	}
	m_freem(m);
	return n;
}

int
mwx_tx(struct mwx_softc *sc, struct mbuf *m, struct ieee80211_node *ni)
{
	struct mwx_node *mn = (struct mwx_node *)ni;
	struct mwx_txwi *mt;
	struct mt76_txwi *txp;
	int rv;
#ifdef AIRPORT_WCL
	struct ieee80211_frame *wcl_wh;
	bool wcl_data;
	unsigned wcl_ac;
#endif

	if ((mt = mwx_txwi_get(sc)) == NULL) {
		m_freem(m);
		return ENOBUFS;
	}
	m = mwx_tx_linearize(m);
	/* XXX DMA memory access without BUS_DMASYNC_PREWRITE */
	txp = mt->mt_desc;
	memset(txp, 0, sizeof(*txp));
	mt7921_mac_write_txwi(sc, m, ni, txp);

#ifdef AIRPORT_WCL
	/* Per access category data counters for macOS's link quality. */
	wcl_wh = mtod(m, struct ieee80211_frame *);
	wcl_data = (wcl_wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) == IEEE80211_FC0_TYPE_DATA &&
	    !(wcl_wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_NODATA);
	wcl_ac = ieee80211_has_qos(wcl_wh) ?
	    ieee80211_wcl_ac(ieee80211_get_qos(wcl_wh) & IEEE80211_QOS_TID) : EDCA_AC_BE;
#endif
#ifdef MTK_AWDL
	if (sc->sc_awdl != NULL)
		mwx_awdl_sta_traffic(sc, m, false);
#endif
	rv = mwx_txwi_enqueue(sc, mt, m);
	if (rv != 0) {
		/* Mapping failed before ownership reached the TXWI. */
		m_freem(m);
		mwx_txwi_put(sc, mt);
		return rv;
	}
#ifdef AIRPORT_WCL
	if (wcl_data)
		++sc->sc_ic.ic_wcl_tx_packets[wcl_ac];
#endif

	static uint32_t tx_log_count;
	if (tx_log_count < 32) {
		MWX_DEV_LOG(sc, "tx frame wcid=%u pid=%d len=%zu\n",
		    mn->wcid, mt->mt_idx, mbuf_pkthdr_len(m));
		MWX_DEV_LOG(sc,
		    "txwi %08x %08x %08x %08x %08x %08x %08x %08x\n",
		    txp->txwi[0], txp->txwi[1], txp->txwi[2], txp->txwi[3],
		    txp->txwi[4], txp->txwi[5], txp->txwi[6], txp->txwi[7]);
		MWX_DEV_LOG(sc,
		    "tx desc msdu=%u/%u/%u/%u len=%u/%u/%u/%u\n",
		    txp->msdu_id[0], txp->msdu_id[1], txp->msdu_id[2],
		    txp->msdu_id[3], txp->ptr[0].len0, txp->ptr[0].len1,
		    txp->ptr[1].len0, txp->ptr[1].len1);
		tx_log_count++;
	}

	rv = mwx_dma_txwi_enqueue(sc, &sc->sc_txq, mt);
	if (rv != 0)
		mwx_txwi_put(sc, mt);
	return rv;
}

/*
 * With management frame protection (802.11w) the access point encrypts the
 * robust management frames it addresses to us with the pairwise key, and
 * the firmware decrypts them like data. net80211 then needs the same proof
 * as for data, the Protected bit and the packet number, or it has to treat
 * the frame as one anybody could have sent. Only for frames that arrived
 * marked Protected, to the address of this station, once the association
 * protects management frames; group-addressed ones are not encrypted but
 * authenticated by a MIC that net80211 checks itself.
 */
static bool
mwx_rx_hwdec_mgmt(struct mwx_softc *sc, const struct ieee80211_frame *wh)
{
	struct ieee80211_node *bss = sc->sc_ic.ic_bss;

	if ((wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) != IEEE80211_FC0_TYPE_MGT ||
	    !(wh->i_fc[1] & IEEE80211_FC1_PROTECTED) ||
	    IEEE80211_IS_MULTICAST(wh->i_addr1) || bss == NULL ||
	    !(bss->ni_flags & IEEE80211_NODE_RXMGMTPROT))
		return false;
	switch (wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK) {
	case IEEE80211_FC0_SUBTYPE_DISASSOC:
	case IEEE80211_FC0_SUBTYPE_DEAUTH:
	case IEEE80211_FC0_SUBTYPE_ACTION:
	case IEEE80211_FC0_SUBTYPE_ACTION_NOACK:
		return true;
	}
	return false;
}

/*
 * The firmware removes the CCMP header and MIC of the frames it decrypts
 * and leaves the packet number in GROUP_1 of the RX descriptor. The frame
 * reached net80211 without the header and with the Protected bit clear,
 * so ieee80211_input_hwdecrypt() skipped its replay check and trusted the
 * hardware; this hardware does not check packet numbers, and a captured
 * unicast data frame sent again was accepted a second time.
 *
 * Put the CCMP header back from the descriptor, as mt76_insert_ccmp_hdr()
 * does in Linux, and keep the Protected bit: net80211 then compares the
 * packet number with the last one seen for that key and TID after Block
 * Ack reordering, the only point where the comparison is valid, and strips
 * the header again. The MIC is already gone. TKIP and WEP keep the old
 * behaviour, header stripped and no check, as mt76 checks CCMP only. A
 * CCMP frame without a packet number is dropped. Returns NULL when the
 * frame was consumed.
 *
 * The protected management frames of mwx_rx_hwdec_mgmt() get the same
 * header: net80211 keeps one more replay counter for them.
 */
static struct mbuf *
mwx_rx_hwdec_restore(struct mwx_softc *sc, struct mbuf *m,
    struct ieee80211_rxinfo *rxi, const struct mwx_rx_sec *sec)
{
	struct ieee80211_frame *wh;
	uint8_t *data, *iv;
	size_t pktlen;
	u_int hdrlen;
	bool mgmt;

	if (!(rxi->rxi_flags & IEEE80211_RXI_HWDEC))
		return m;
	wh = mtod(m, struct ieee80211_frame *);
	mgmt = mwx_rx_hwdec_mgmt(sc, wh);
	if (sec->sec_mode != MT_CIPHER_AES_CCMP ||
	    ((wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) != IEEE80211_FC0_TYPE_DATA &&
	    !mgmt)) {
		wh->i_fc[1] &= ~IEEE80211_FC1_PROTECTED;
		return m;
	}
	if (mgmt)
		mwx_rx_mgmt_hwdec_cnt++;
	hdrlen = ieee80211_get_hdrlen(wh);
	pktlen = mbuf_pkthdr_len(m);
	if (!sec->pn_valid || pktlen < hdrlen) {
		if (mwx_rx_hwdec_nopn_cnt < 8)
			MWX_DEV_LOG(sc,
			    "rx drop reason=hwdec-no-pn pn_valid=%u len=%zu hdrlen=%u\n",
			    sec->pn_valid, pktlen, hdrlen);
		mwx_rx_hwdec_nopn_cnt++;
		m_freem(m);
		return NULL;
	}
	if (mbuf_next(m) == NULL && mbuf_len(m) == pktlen &&
	    !mbuf_mclhasreference(m) &&
	    m_leadingspace(m) >= IEEE80211_CCMP_HDRLEN) {
		/* The RX descriptor just removed in front leaves the room. */
		data = (uint8_t *)wh - IEEE80211_CCMP_HDRLEN;
		if (mbuf_setdata(m, data, pktlen + IEEE80211_CCMP_HDRLEN) != 0) {
			m_freem(m);
			return NULL;
		}
		mbuf_pkthdr_setlen(m, pktlen + IEEE80211_CCMP_HDRLEN);
	} else {
		m = m_prepend(m, IEEE80211_CCMP_HDRLEN, M_DONTWAIT);
		if (m == NULL)
			return NULL;
		m = m_pullup(m, hdrlen + IEEE80211_CCMP_HDRLEN);
		if (m == NULL)
			return NULL;
		data = mtod(m, uint8_t *);
	}
	memmove(data, data + IEEE80211_CCMP_HDRLEN, hdrlen);
	iv = data + hdrlen;
	iv[0] = sec->pn[0];
	iv[1] = sec->pn[1];
	iv[2] = 0;
	iv[3] = IEEE80211_WEP_EXTIV | (sec->key_id << 6);
	iv[4] = sec->pn[2];
	iv[5] = sec->pn[3];
	iv[6] = sec->pn[4];
	iv[7] = sec->pn[5];
	data[1] |= IEEE80211_FC1_PROTECTED;
	return m;
}

void
mwx_rx(struct mwx_softc *sc, struct mbuf *m, struct mbuf_list *ml, uint8_t rxqueue)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ifnet *ifp = &ic->ic_if;
	struct ieee80211_node *ni;
	struct ieee80211_frame *wh;
	struct ieee80211_rxinfo rxi = { 0 };
	struct mwx_rx_sec sec;
	MtkRxDescriptorMeta descriptor = {};
	int aggregate = 0, rv;
	uint32_t rx_epoch = 0;
#ifdef MTK_AWDL
	uint8_t awdl_part = 0;
#endif

	/* Opt-in until hardware-format and throughput validation is complete. */
	if (sc->sc_rx_amsdu_enabled) {
		aggregate = mtkRxAmsduReadDescriptor(m, descriptor);
#ifdef MTK_AWDL
		/*
		 * A subframe of an A-MSDU the hardware split, sent by an AWDL peer: the hardware splits them once the
		 * peer has a station record. It is no station frame (no FromDS); the AWDL tap takes it below.
		 */
		if (aggregate < 0 && sc->sc_awdl != NULL &&
		    (awdl_part = mwx_awdl_rx_subframe(m, descriptor.hdr_gap)) != 0)
			aggregate = 0;
#endif
		if (aggregate != 0 && !mwx_rx_amsdu_snapshot(sc, &rx_epoch)) {
			m_freem(m);
			return;
		}
		static uint32_t rx_amsdu_descriptor_logs;
		if (aggregate != 0 && rx_amsdu_descriptor_logs < 8) {
			MWX_DEV_LOG(sc,
			    "rxamsdu descriptor queue=%u result=%d format=%u wcid=%u cipher=%u key=%u pn_valid=%d protected=%d len=%zu hdrgap=%u hdrlen=%u\n",
			    rxqueue, aggregate, descriptor.format, descriptor.wcid,
			    descriptor.cipher, descriptor.key, descriptor.pn_valid,
			    descriptor.protected_frame, mbuf_len(m), descriptor.hdr_gap,
			    descriptor.hdrlen);
			rx_amsdu_descriptor_logs++;
		}
		if (aggregate < 0) {
			mwx_rx_amsdu_purge(sc, -1, true);
			ifp->if_ierrors++;
			m_freem(m);
			return;
		}
	}
#ifdef MTK_AWDL
	else if (sc->sc_awdl != NULL)
		awdl_part = mwx_awdl_rx_subframe(m, 0);
#endif
	rv = mt7921_mac_fill_rx(sc, m, &rxi, &sec);
	if (rv != 0) {
		if (aggregate)
			mwx_rx_amsdu_purge(sc);
		if (sc->sc_rx_fill_fail < 32) {
			MWX_DEV_LOG(sc,
			    "rx fill failed rv=%d len=%zu fail_count=%u\n",
			    rv, mbuf_len(m), sc->sc_rx_fill_fail + 1);
		}
		sc->sc_rx_fill_fail++;
		ifp->if_ierrors++;
		m_freem(m);
		return;
	}
#ifdef MTK_AWDL
	/* AWDL frames go to awdl0; while other devices' frames may arrive, the station's filter runs here. */
	if (sc->sc_awdl != NULL && mwx_awdl_rx_tap(sc, m, &rxi, awdl_part)) {
		m_freem(m);
		return;
	}
#endif

	if (aggregate) {
		if (!mtkRxAmsduRemovePad(m, descriptor)) {
			mwx_rx_amsdu_purge(sc, -1, true);
			ifp->if_ierrors++;
			m_freem(m);
			return;
		}
		MtkRxAmsduMeta meta = {}, first = {};
		meta.epoch = rx_epoch;
		meta.queue = rxqueue;
		meta.wcid = descriptor.wcid;
		meta.seqctl = descriptor.seqctl;
		meta.tid = descriptor.tid;
		meta.cipher = descriptor.cipher;
		meta.key = descriptor.key;
		meta.format = descriptor.format;
		meta.hdrlen = descriptor.hdrlen;
		meta.protected_frame = descriptor.protected_frame;
		meta.hwdec = descriptor.hwdec;
		meta.pn_valid = descriptor.pn_valid;
		memcpy(meta.pn, descriptor.pn, sizeof(meta.pn));
		meta.rxi = rxi;
		uint64_t now_ns = 0;
		absolutetime_to_nanoseconds(mach_absolute_time(), &now_ns);
		m = mwx_rx_amsdu_feed(sc, m, meta, now_ns / 1000000, &first);
		if (m == NULL)
			return;
		descriptor.wcid = first.wcid;
		descriptor.seqctl = first.seqctl;
		descriptor.tid = first.tid;
		descriptor.cipher = first.cipher;
		descriptor.key = first.key;
		descriptor.format = first.format;
		descriptor.hdrlen = first.hdrlen;
		descriptor.protected_frame = first.protected_frame;
		descriptor.hwdec = first.hwdec;
		descriptor.pn_valid = first.pn_valid;
		memcpy(descriptor.pn, first.pn, sizeof(descriptor.pn));
		rxi = first.rxi;
		if (first.epoch != mwx_rx_amsdu_epoch(sc)) {
			m_freem(m);
			return;
		}
		/* Restore exactly one IV. net80211 validates PN after BA reorder
		 * and removes this IV; the hardware already removed the MIC. */
		m = mtkRxAmsduRestoreCcmp(m, descriptor);
		if (m == NULL) {
			mwx_rx_amsdu_invalid(sc);
			ifp->if_ierrors++;
			return;
		}
	} else {
		m = mwx_rx_hwdec_restore(sc, m, &rxi, &sec);
		if (m == NULL) {
			ifp->if_ierrors++;
			return;
		}
	}

	wh = mtod(m, struct ieee80211_frame *);
	if (sc->sc_rx_delivered < 32) {
		uint8_t type = wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK;
		uint8_t subtype = wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK;

		MWX_DEV_LOG(sc,
		    "rx frame delivered type=0x%02x subtype=0x%02x chan=%u rssi=%d len=%zu\n",
		    type, subtype, rxi.rxi_chan, rxi.rxi_rssi, mbuf_len(m));
	}
	{
		static uint32_t rx_run_log_count;
		uint8_t type = wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK;
		uint8_t subtype = wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK;

		if (rx_run_log_count < 48 &&
		    (type == IEEE80211_FC0_TYPE_DATA ||
		    (ic->ic_state == IEEE80211_S_RUN &&
		    (rxi.rxi_flags & IEEE80211_RXI_HWDEC)))) {
			MWX_DEV_LOG(sc,
			    "rx run frame state=%d fc=%02x/%02x type=0x%02x subtype=0x%02x rxi_flags=0x%x chan=%u rssi=%d len=%zu\n",
			    ic->ic_state, wh->i_fc[0], wh->i_fc[1], type, subtype,
			    rxi.rxi_flags, rxi.rxi_chan, rxi.rxi_rssi, mbuf_len(m));
			rx_run_log_count++;
		}
	}
	sc->sc_rx_delivered++;
	if ((wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) == IEEE80211_FC0_TYPE_CTL) {
		mwx_rx_ctl_cnt++;
		if ((wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK) ==
		    IEEE80211_FC0_SUBTYPE_BAR)
			mwx_rx_bar_cnt++;
	}
	if (rxi.rxi_flags & IEEE80211_RXI_HWDEC)
		mwx_rx_hwdec_cnt++;
	else if (wh->i_fc[1] & IEEE80211_FC1_PROTECTED)
		mwx_rx_swdec_cnt++;

#if NBPFILTER > 0
	if (__predict_false(sc->sc_drvbpf != NULL)) {
		struct mwx_rx_radiotap_header *tap = &sc->sc_rxtap;
		uint32_t tsf_lo, tsf_hi;
		/* get timestamp (low and high 32 bits) */
		tsf_hi = 0;
		tsf_lo = 0;
		tap->wr_tsft = htole64(((uint64_t)tsf_hi << 32) | tsf_lo);
		tap->wr_flags = 0;
		tap->wr_rate = 2;	/* XXX */
		tap->wr_chan_freq =
		    htole16(ic->ic_channels[rxi.rxi_chan].ic_freq);
		tap->wr_chan_flags =
		    ic->ic_channels[rxi.rxi_chan].ic_flags;
		tap->wr_dbm_antsignal = rxi.rxi_rssi;
		bpf_mtap_hdr(sc->sc_drvbpf, tap, sc->sc_rxtap_len, m,
		    BPF_DIRECTION_IN);
	}
#endif

	/* grab a reference to the source node */
	ni = ieee80211_find_rxnode(ic, wh);

	if (ic->ic_max_rssi && rxi.rxi_rssi < 0)
		rxi.rxi_rssi = mwx_dbm_to_rssi(rxi.rxi_rssi);

#ifdef MTK_AWDL
	if (sc->sc_awdl != NULL)
		mwx_awdl_sta_traffic(sc, m, true);
#endif
	/* send the frame to the 802.11 layer */
	ieee80211_inputm(ifp, m, ni, &rxi, ml);

	/* node is no longer needed */
	ieee80211_release_node(ic, ni);
}

int
mwx_count_nodes(struct ieee80211com *ic)
{
	struct ieee80211_node *ni;
	int count = 0;

	RB_FOREACH(ni, ieee80211_tree, &ic->ic_tree)
		count++;
	return count;
}

/*
 * Driver specific functions.
 */
static uint32_t mwx_intr_calls;

int
mwx_intr(void *arg)
{
	struct mwx_softc *sc = (struct mwx_softc *)arg;
	struct mbuf_list mlout = MBUF_LIST_INITIALIZER();
	uint32_t intr, intr_sw, waiting;
	uint32_t mask = sc->sc_intr_mask;
	static uint32_t intr_log_count;
	static uint32_t intr_unhandled_log_count;

	mwx_intr_calls++;

	/*
	 * Asleep, or the card is not powered yet after wake: its registers
	 * read all ones and the DMA rings are stale. Touching them panicked
	 * (NULL mbuf in mwx_dma_rx_dequeue) on wake.
	 */
	if (sc->sc_suspended)
		return 0;

	mwx_write(sc, MT_WFDMA0_HOST_INT_ENA, 0);
	intr = mwx_read(sc, MT_WFDMA0_HOST_INT_STA);
	if (intr == 0xffffffff)
		return 0;
	/* A ring stopped at a slot still without DMA_DONE is read again although nothing new was signalled. */
	waiting = (sc->sc_rxq.mq_rx_wait_ns != 0 ? MT_INT_RX_DONE_DATA : 0) |
	    (sc->sc_rxmcuq.mq_rx_wait_ns != 0 ? MT_INT_RX_DONE_WM2 : 0) |
	    (sc->sc_rxfwdlq.mq_rx_wait_ns != 0 ? MT_INT_RX_DONE_WM : 0);
	if (intr == 0 && waiting == 0) {
		mwx_write(sc, MT_WFDMA0_HOST_INT_ENA, mask);
		return 0;
	}

	/* TODO power management */
//	mt76_connac_pm_ref(&dev->mphy, &dev->pm);

	if (intr_log_count < 16) {
		MWX_DEV_LOG(sc, "interrupt status=0x%08x mask=0x%08x\n",
		    intr, mask);
		intr_log_count++;
	}

	if (intr & ~mask && intr_unhandled_log_count < 16) {
		printf("%s: unhandled interrupt %08x\n", DEVNAME(sc),
		    intr & ~mask);
		intr_unhandled_log_count++;
	}
	/* ack interrupts */
	intr &= mask;
	mwx_write(sc, MT_WFDMA0_HOST_INT_STA, intr);
	intr |= waiting & mask;

	if (intr & MT_INT_MCU_CMD) {
		intr_sw = mwx_read(sc, MT_MCU_CMD);
		if (intr_log_count < 16)
			MWX_DEV_LOG(sc, "mcu interrupt status=0x%08x\n", intr_sw);
		/* ack MCU2HOST_SW_INT_STA */
		mwx_write(sc, MT_MCU_CMD, intr_sw);
		if (intr_sw & MT_MCU_CMD_WAKE_RX_PCIE)
			intr |= MT_INT_RX_DONE_DATA;
	}

	bool rx_more = false;
	for (int pass = 0; ; pass++) {
		if (intr & MT_INT_TX_DONE_ALL)
			mwx_dma_tx_done(sc);

		if (intr & MT_INT_RX_DONE_WM)
			rx_more |= mwx_dma_rx_done(sc, &sc->sc_rxfwdlq, &mlout);
		if (intr & MT_INT_RX_DONE_WM2)
			rx_more |= mwx_dma_rx_done(sc, &sc->sc_rxmcuq, &mlout);
		if (intr & MT_INT_RX_DONE_DATA)
			rx_more |= mwx_dma_rx_done(sc, &sc->sc_rxq, &mlout);

		/*
		 * MSI is edge triggered: events that arrived while the host
		 * interrupt enable was cleared do not raise a new message, so
		 * drain them here before re-enabling.
		 */
		if (pass >= 8)
			break;
		intr = mwx_read(sc, MT_WFDMA0_HOST_INT_STA) & mask;
		if (intr == 0)
			break;
		mwx_write(sc, MT_WFDMA0_HOST_INT_STA, intr);
		if (intr & MT_INT_MCU_CMD) {
			intr_sw = mwx_read(sc, MT_MCU_CMD);
			mwx_write(sc, MT_MCU_CMD, intr_sw);
			if (intr_sw & MT_MCU_CMD_WAKE_RX_PCIE)
				intr |= MT_INT_RX_DONE_DATA;
		}
	}

	/* Deliver one data batch across all RX queues and drain passes. MCU
	 * events and management frames were processed immediately above. */
	if (!ml_empty(&mlout)) {
		const u_int packets = ml_len(&mlout);
		sc->sc_rx_input_batches++;
		sc->sc_rx_input_packets += packets;
		if (packets > sc->sc_rx_input_max_batch)
			sc->sc_rx_input_max_batch = packets;
		if_input(&sc->sc_ic.ic_if, &mlout);
	}

	/* TODO power management */
//	mt76_connac_pm_unref(&dev->mphy, &dev->pm);

	mwx_write(sc, MT_WFDMA0_HOST_INT_ENA, mask);
	if (rx_more)
		pci_intr_reschedule(sc->sc_pc, sc->sc_ih);

	return 1;
}

static void
mwx_init_he_caps(struct mwx_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_he_cap_elem *cap = &ic->ic_he_cap_elem;
	struct ieee80211_he_mcs_nss_supp *mcs = &ic->ic_he_mcs_nss_supp;
	const unsigned nss = sc->sc_capa.num_streams > 1 ? 2 : 1;
	uint16_t map = 0;

	memset(cap, 0, sizeof(*cap));
	memset(mcs, 0, sizeof(*mcs));
	memset(ic->ic_ppe_thres, 0, sizeof(ic->ic_ppe_thres));

	for (unsigned i = 0; i < 8; i++) {
		const unsigned support = i < nss ?
		    IEEE80211_HE_MCS_SUPPORT_0_11 : IEEE80211_HE_MCS_NOT_SUPPORTED;
		map |= support << (i * 2);
	}

	cap->mac_cap_info[0] = IEEE80211_HE_MAC_CAP0_HTC_HE;
	cap->mac_cap_info[1] = IEEE80211_HE_MAC_CAP1_TF_MAC_PAD_DUR_16US;
	cap->mac_cap_info[3] = IEEE80211_HE_MAC_CAP3_OMI_CONTROL |
	    IEEE80211_HE_MAC_CAP3_MAX_AMPDU_LEN_EXP_VHT_2;
	cap->mac_cap_info[4] = IEEE80211_HE_MAC_CAP4_AMDSU_IN_AMPDU;
	cap->phy_cap_info[1] = IEEE80211_HE_PHY_CAP1_LDPC_CODING_IN_PAYLOAD |
	    IEEE80211_HE_PHY_CAP1_DEVICE_CLASS_A |
	    IEEE80211_HE_PHY_CAP1_HE_LTF_AND_GI_FOR_HE_PPDUS_0_8US;
	cap->phy_cap_info[2] |= IEEE80211_HE_PHY_CAP2_NDP_4x_LTF_AND_3_2US;
	if (sc->sc_capa.has_2ghz) {
		cap->phy_cap_info[0] |=
		    IEEE80211_HE_PHY_CAP0_CHANNEL_WIDTH_SET_40MHZ_IN_2G |
		    IEEE80211_HE_PHY_CAP0_CHANNEL_WIDTH_SET_RU_MAPPING_IN_2G;
		cap->phy_cap_info[8] |=
		    IEEE80211_HE_PHY_CAP8_20MHZ_IN_40MHZ_HE_PPDU_IN_2G;
	}
	if (sc->sc_capa.has_5ghz) {
		cap->phy_cap_info[0] |=
		    IEEE80211_HE_PHY_CAP0_CHANNEL_WIDTH_SET_40MHZ_80MHZ_IN_5G |
		    IEEE80211_HE_PHY_CAP0_CHANNEL_WIDTH_SET_RU_MAPPING_IN_5G;
		if (sc->sc_vht160)
			cap->phy_cap_info[0] |=
			    IEEE80211_HE_PHY_CAP0_CHANNEL_WIDTH_SET_160MHZ_IN_5G;
	}

	/* Match the Linux mt7921/7922 station HE PHY advertisement. */
	cap->phy_cap_info[3] |=
	    IEEE80211_HE_PHY_CAP3_DCM_MAX_CONST_TX_QPSK |
	    IEEE80211_HE_PHY_CAP3_DCM_MAX_TX_NSS_2 |
	    IEEE80211_HE_PHY_CAP3_DCM_MAX_CONST_RX_QPSK |
	    IEEE80211_HE_PHY_CAP3_DCM_MAX_RX_NSS_2;
	cap->phy_cap_info[4] |=
	    IEEE80211_HE_PHY_CAP4_SU_BEAMFORMEE |
	    IEEE80211_HE_PHY_CAP4_BEAMFORMEE_MAX_STS_UNDER_80MHZ_4;
	cap->phy_cap_info[5] |=
	    IEEE80211_HE_PHY_CAP5_NG16_SU_FEEDBACK |
	    IEEE80211_HE_PHY_CAP5_NG16_MU_FEEDBACK;
	cap->phy_cap_info[6] |=
	    IEEE80211_HE_PHY_CAP6_CODEBOOK_SIZE_42_SU |
	    IEEE80211_HE_PHY_CAP6_CODEBOOK_SIZE_75_MU |
	    IEEE80211_HE_PHY_CAP6_TRIG_CQI_FB |
	    IEEE80211_HE_PHY_CAP6_PARTIAL_BW_EXT_RANGE;
	cap->phy_cap_info[7] |=
	    IEEE80211_HE_PHY_CAP7_POWER_BOOST_FACTOR_AR |
	    IEEE80211_HE_PHY_CAP7_HE_SU_MU_PPDU_4XLTF_AND_08_US_GI;
	cap->phy_cap_info[8] |= IEEE80211_HE_PHY_CAP8_DCM_MAX_RU_484;
	cap->phy_cap_info[9] |=
	    IEEE80211_HE_PHY_CAP9_LONGER_THAN_16_SIGB_OFDM_SYM |
	    IEEE80211_HE_PHY_CAP9_NON_TRIGGERED_CQI_FEEDBACK |
	    IEEE80211_HE_PHY_CAP9_TX_1024_QAM_LESS_THAN_242_TONE_RU |
	    IEEE80211_HE_PHY_CAP9_RX_1024_QAM_LESS_THAN_242_TONE_RU |
	    IEEE80211_HE_PHY_CAP9_RX_FULL_BW_SU_USING_MU_WITH_COMP_SIGB |
	    IEEE80211_HE_PHY_CAP9_RX_FULL_BW_SU_USING_MU_WITH_NON_COMP_SIGB;
	if (sc->sc_vht160) {
		cap->phy_cap_info[4] |=
		    IEEE80211_HE_PHY_CAP4_BEAMFORMEE_MAX_STS_ABOVE_80MHZ_4;
		cap->phy_cap_info[8] |=
		    IEEE80211_HE_PHY_CAP8_20MHZ_IN_160MHZ_HE_PPDU |
		    IEEE80211_HE_PHY_CAP8_80MHZ_IN_160MHZ_HE_PPDU;
	}

	cap->phy_cap_info[6] |=
	    IEEE80211_HE_PHY_CAP6_PPE_THRESHOLD_PRESENT;
	{
		static const uint8_t ppet16_ppet8_ru3_ru0[] = {
		    0x1c, 0xc7, 0x71
		};
		const uint8_t ru_mask = sc->sc_capa.has_5ghz ? 0x0f : 0x03;
		const unsigned ru_count = hweight8(ru_mask);
		const unsigned ppet_bits = IEEE80211_PPE_THRES_INFO_PPET_SIZE *
		    nss * ru_count * 2;
		const unsigned ppet_size = (ppet_bits + 7) / 8;

		ic->ic_ppe_thres[0] = ((nss - 1) <<
		    IEEE80211_PPE_THRES_NSS_POS) |
		    (ru_mask << IEEE80211_PPE_THRES_RU_INDEX_BITMASK_POS);
		for (unsigned i = 0; i < ppet_size; i++)
			ic->ic_ppe_thres[i + 1] =
			    ppet16_ppet8_ru3_ru0[i % 3];
		ic->ic_ppe_thres[ppet_size] &=
		    0xff >> (8 - (ppet_bits - 1) % 8);
	}

	mcs->rx_mcs_80 = htole16(map);
	mcs->tx_mcs_80 = htole16(map);
	if (sc->sc_vht160) {
		mcs->rx_mcs_160 = htole16(map);
		mcs->tx_mcs_160 = htole16(map);
	}
}

/* Linux tries ten times; three keep a dead card from holding up the boot. */
#define	MWX_INIT_HARDWARE_TRIES	3

int
mwx_preinit(struct mwx_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ifnet *ifp = &ic->ic_if;
	int rv, i;
	uint8_t chan;

	MWX_DEV_LOG(sc, "preinit start fw_loaded=%d\n", sc->sc_fw_loaded);

	/*
	 * A chip the previous driver session left running can miss the first
	 * MCU handshake. A failed mwx_init_hardware() ends with a DMA and
	 * WFSYS reset, so start again from there, as Linux
	 * mt7921_init_hardware() does.
	 */
	for (i = 1; (rv = mwx_init_hardware(sc)) != 0; i++) {
		MWX_DEV_LOG(sc, "preinit hardware init failed error=%d attempt=%d of %d\n",
		    rv, i, MWX_INIT_HARDWARE_TRIES);
		if (i >= MWX_INIT_HARDWARE_TRIES)
			return rv;
	}
	MWX_DEV_LOG(sc, "preinit hardware init ok\n");

	if ((rv = mwx_mcu_set_deep_sleep(sc, 1)) != 0) {
		MWX_DEV_LOG(sc, "preinit deep sleep enable failed error=%d\n", rv);
		return rv;
	}
	MWX_DEV_LOG(sc, "preinit deep sleep enabled\n");

	ic->ic_sup_rates[IEEE80211_MODE_11B] = ieee80211_std_rateset_11b;
	ic->ic_sup_rates[IEEE80211_MODE_11G] = ieee80211_std_rateset_11g;
	MWX_DEV_LOG(sc, "preinit capability 2ghz=%d 5ghz=%d 6ghz=%d streams=%u antenna_mask=0x%x\n",
	    sc->sc_capa.has_2ghz, sc->sc_capa.has_5ghz,
	    sc->sc_capa.has_6ghz, sc->sc_capa.num_streams,
	    sc->sc_capa.antenna_mask);

	if (sc->sc_capa.has_2ghz) {
		for (i = 0; i < MWX_NUM_2GHZ_CHANNELS; i++) {
			chan = mwx_channels_2ghz[i];
			ic->ic_channels[chan].ic_freq =
			    ieee80211_ieee2mhz(chan, IEEE80211_CHAN_2GHZ);
			ic->ic_channels[chan].ic_flags =
			    IEEE80211_CHAN_CCK | IEEE80211_CHAN_OFDM |
			    IEEE80211_CHAN_DYN | IEEE80211_CHAN_2GHZ |
			    IEEE80211_CHAN_HT20;
		}

	}
	if (sc->sc_capa.has_5ghz) {
		ic->ic_sup_rates[IEEE80211_MODE_11A] =
		    ieee80211_std_rateset_11a;
		/* set supported .11a channels */
		for (i = 0; i < MWX_NUM_5GHZ_CHANNELS; i++) {
			chan = mwx_channels_5ghz[i];
			ic->ic_channels[chan].ic_freq =
			    ieee80211_ieee2mhz(chan, IEEE80211_CHAN_5GHZ);
			/*
			 * Capability flags only; the operating width comes
			 * from what net80211 negotiates (ni_chw).
			 */
			ic->ic_channels[chan].ic_flags = IEEE80211_CHAN_A |
			    IEEE80211_CHAN_HT20 | IEEE80211_CHAN_HT40 |
			    IEEE80211_CHAN_VHT20 |
			    IEEE80211_CHAN_VHT40 | IEEE80211_CHAN_VHT80 |
			    (sc->sc_vht160 ? IEEE80211_CHAN_VHT160 : 0);
		}
	}
#ifdef NOTYET
	/* TODO support for 6GHz */
	if (sc->sc_capa.has_6ghz) {
		for (i = 0; i < MWX_NUM_6GHZ_CHANNELS; i++) {
		}
	}
#endif

	/* HT MCS set: MCS 0-7 per spatial stream. */
	memset(ic->ic_sup_mcs, 0, sizeof(ic->ic_sup_mcs));
	ic->ic_sup_mcs[0] = 0xff;
	if (sc->sc_capa.num_streams >= 2)
		ic->ic_sup_mcs[1] = 0xff;
	ic->ic_tx_mcs_set = IEEE80211_TX_MCS_SET_DEFINED;

	{
		int off = 0;
		/*
		 * MT7920 has the HE capabilities of the MT7921 in Linux
		 * mt7921: HE80, no 160 MHz, which mwx_init_he_caps() keeps
		 * for MT7922 through sc_vht160.
		 */
		sc->sc_he_enabled = !PE_parse_boot_argn("-mtknohe", &off,
		    sizeof(off)) && (sc->sc_hwtype == MWX_HW_MT7922 ||
		    sc->sc_hwtype == MWX_HW_MT7920);
	}	{
		int only = 0;
		sc->sc_he_only = sc->sc_he_enabled &&
		    PE_parse_boot_argn("-mtkheonly", &only, sizeof(only));
	}
	mwx_init_he_caps(sc);
	/* Configure channel information obtained from firmware. */
	ieee80211_channel_init(ifp);
	if (sc->sc_he_enabled)
		ic->ic_modecaps |= 1 << IEEE80211_MODE_11AX;
	else
		ic->ic_modecaps &= ~(1 << IEEE80211_MODE_11AX);
	MWX_DEV_LOG(sc, "preinit channel table ready vht160=%d wtbl_ba=%d he=%d heonly=%d\n",
	    sc->sc_vht160, sc->sc_wtbl_ba, sc->sc_he_enabled,
	    sc->sc_he_only);

	if (IEEE80211_ADDR_EQ(etheranyaddr, sc->sc_ic.ic_myaddr))
		IEEE80211_ADDR_COPY(ic->ic_myaddr, sc->sc_lladdr);

	/* Configure MAC address. */
	rv = if_setlladdr(ifp, ic->ic_myaddr);
	if (rv)
		printf("%s: could not set MAC address (error %d)\n",
		    DEVNAME(sc), rv);
	else
		MWX_DEV_LOG(sc, "preinit mac address=%s\n",
		    ether_sprintf(ic->ic_myaddr));

	ieee80211_media_init(ifp);
	/*
	 * Enable QoS (WMM) directly: ieee80211_configure_ampdu_tx() only does
	 * so together with TX A-MPDU, which the driver does not support yet.
	 */
	ic->ic_flags |= IEEE80211_F_QOS;
	if (sc->sc_he_enabled)
		ic->ic_flags |= IEEE80211_F_HEON;
	else
		ic->ic_flags &= ~IEEE80211_F_HEON;
	if ((ic->ic_modecaps & (1 << IEEE80211_MODE_11AC)) &&
	    !sc->sc_he_only)
		ic->ic_flags |= IEEE80211_F_VHTON;
	else
		ic->ic_flags &= ~IEEE80211_F_VHTON;
	MWX_DEV_LOG(sc, "preinit 11n htcaps=0x%04x mcs=%02x%02x hton=%d qos=%d vhton=%d vhtcaps=0x%08x\n",
	    ic->ic_htcaps, ic->ic_sup_mcs[1], ic->ic_sup_mcs[0],
	    !!(ic->ic_flags & IEEE80211_F_HTON),
	    !!(ic->ic_flags & IEEE80211_F_QOS),
	    !!(ic->ic_flags & IEEE80211_F_VHTON), ic->ic_vhtcaps);

	sc->sc_sta_added = 0;	/* fresh firmware has no peer records */
	sc->sc_fw_loaded = 1;
	MWX_DEV_LOG(sc, "preinit complete\n");
	return 0;
}

void
mwx_attach_hook(struct device *self)
{
	struct mwx_softc *sc = (struct mwx_softc *)self;

	MWX_DEV_LOG(sc, "attach hook start\n");
	mwx_preinit(sc);
	MWX_DEV_LOG(sc, "attach hook done fw_loaded=%d\n", sc->sc_fw_loaded);
}

static const char *
mwx_hw_name(enum mwx_hw_type type)
{
	switch (type) {
	case MWX_HW_MT7920:
		return "MT7920";
	case MWX_HW_MT7922:
		return "MT7922";
	default:
		return "MT7921";
	}
}

/*
 * MT7920 is MT7921 silicon: the chip ID reads 0x7961 on both. Like Linux
 * mt7921_pci_probe(), bit 7 of MT_HW_BOUND picks the type, because it decides
 * which firmware the chip needs. Other chip IDs keep the type from the PCI ID.
 */
static enum mwx_hw_type
mwx_hw_type_from_chip(enum mwx_hw_type type, uint32_t hwid, uint32_t hwbound)
{
	if (hwid != 0x7961)
		return type;
	return (hwbound & MT_HW_BOUND_MT7920) != 0 ? MWX_HW_MT7920 : MWX_HW_MT7921;
}

static void
mwx_hw_firmware(enum mwx_hw_type type, const char **rompatch, const char **fw)
{
	switch (type) {
	case MWX_HW_MT7920:
		*rompatch = MT7920_ROM_PATCH;
		*fw = MT7920_FIRMWARE_WM;
		break;
	case MWX_HW_MT7922:
		*rompatch = MT7922_ROM_PATCH;
		*fw = MT7922_FIRMWARE_WM;
		break;
	default:
		*rompatch = MT7921_ROM_PATCH;
		*fw = MT7921_FIRMWARE_WM;
		break;
	}
}

struct mwx_mountroot_call {
	void		(*fn)(struct device *);
	struct device	*self;
};

static IOReturn
mwx_mountroot_gated(OSObject *, void *arg, void *, void *, void *)
{
	struct mwx_mountroot_call *call = (struct mwx_mountroot_call *)arg;

	call->fn(call->self);
	return kIOReturnSuccess;
}

/*
 * OpenBSD runs the mountroot hook under the kernel lock, which the interrupt
 * handler takes as well. Here the hook ran on the thread that attaches the
 * driver while the interrupt source was already live on the work loop, and
 * the two walked the firmware download ring at the same time: each freed a
 * chunk the other had just freed (MEXT_REF(m) != 0 in m_freem). Close the
 * gate around the hook; tsleep_nsec() opens it whenever the hook waits for
 * the firmware, so interrupts still get through.
 */
static inline void
config_mountroot(struct device *self, void (*fn)(struct device *))
{
	struct mwx_softc *sc = (struct mwx_softc *)self;
	struct mwx_mountroot_call call = { fn, self };
	IOCommandGate *gate = sc->sc_rx_amsdu_gate;	/* the main command gate */

	if (gate == NULL ||
	    gate->runAction(mwx_mountroot_gated, &call) != kIOReturnSuccess)
		fn(self);
}

bool
mwx_attach(struct device *parent, struct device *self, void *aux)
{
	struct mwx_softc *sc = (struct mwx_softc *)self;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ifnet *ifp = &ic->ic_if;
	struct pci_attach_args *pa = (struct pci_attach_args *)aux;
	pci_intr_handle_t ih;
	pcireg_t memtype;
	uint32_t hwid, hwrev, hwbound;
	int error;

	MWX_DEV_LOG(sc, "attach start pci_id=0x%08x vendor=0x%04x product=0x%04x\n",
	    pa->pa_id, PCI_VENDOR(pa->pa_id), PCI_PRODUCT(pa->pa_id));
	sc->sc_pc = pa->pa_pc;
	sc->sc_tag = pa->pa_tag;
	sc->sc_dmat = pa->pa_dmat;
	sc->sc_intr_mask = MT_INT_RX_DONE_ALL | MT_INT_TX_DONE_ALL |
	    MT_INT_MCU_CMD;
	if (PCI_PRODUCT(pa->pa_id) == PCI_PRODUCT_MEDIATEK_MT7922 ||
	    PCI_PRODUCT(pa->pa_id) == PCI_PRODUCT_MEDIATEK_MT7922_PCIE)
		sc->sc_hwtype = MWX_HW_MT7922;
	else if (PCI_PRODUCT(pa->pa_id) == PCI_PRODUCT_MEDIATEK_MT7920)
		sc->sc_hwtype = MWX_HW_MT7920;
	else
		sc->sc_hwtype = MWX_HW_MT7921;
	MWX_DEV_LOG(sc, "attach hardware type=%s\n", mwx_hw_name(sc->sc_hwtype));
	/* Resolve this before advertising VHT capabilities, not in preinit. */
	{
		int flag = 0;
		sc->sc_vht160 = sc->sc_hwtype == MWX_HW_MT7922 &&
		    PE_parse_boot_argn("-mtk160", &flag, sizeof(flag)) &&
		    !PE_parse_boot_argn("-mtkno160", &flag, sizeof(flag));
		sc->sc_wtbl_ba = !PE_parse_boot_argn("-mtknowtblba", &flag,
		    sizeof(flag));
	}

	memtype = pci_mapreg_type(pa->pa_pc, pa->pa_tag, PCI_MAPREG_START);
	if (pci_mapreg_map(pa, PCI_MAPREG_START, memtype, 0,
	    &sc->sc_st, &sc->sc_memh, NULL, &sc->sc_mems, 0)) {
		printf("%s: can't map mem space\n", DEVNAME(sc));
		return false;
	}
	MWX_DEV_LOG(sc, "attach BAR mapped memtype=0x%x size=%zu\n",
	    memtype, (size_t)sc->sc_mems);

	if (pci_intr_map_msix(pa, 0, &ih) &&
	    pci_intr_map_msi(pa, &ih) &&
	    pci_intr_map(pa, &ih)) {
		printf("%s: can't map interrupt\n", DEVNAME(sc));
		/* HAL detach owns the mapping on failure. */
		return false;
	}
	MWX_DEV_LOG(sc, "attach interrupt mapped source=%s\n",
	    pci_intr_string(pa->pa_pc, ih));

	if (mwx_mcu_fw_pmctrl(sc) != 0 ||
	    mwx_mcu_drv_pmctrl(sc) != 0) {
		/*
		 * A warm reboot does not power-cycle the M.2 card, so the chip
		 * can be left wedged by the previous OS session. Try to recover
		 * instead of failing: WFSYS reset first, then a PCIe FLR.
		 */
		MWX_DEV_LOG(sc, "attach MCU power control failed lpctl=0x%08x, "
		    "trying WFSYS reset\n", mwx_read(sc, MT_CONN_ON_LPCTL));
		mwx_wfsys_reset(sc);
		if (mwx_mcu_fw_pmctrl(sc) != 0 ||
		    mwx_mcu_drv_pmctrl(sc) != 0) {
			MWX_DEV_LOG(sc, "still failing lpctl=0x%08x, trying PCIe FLR\n",
			    mwx_read(sc, MT_CONN_ON_LPCTL));
			if (mwx_pci_flr(sc, pa->pa_tag) != 0 ||
			    mwx_mcu_fw_pmctrl(sc) != 0 ||
			    mwx_mcu_drv_pmctrl(sc) != 0) {
				MWX_DEV_LOG(sc, "attach MCU power control failed after "
				    "recovery lpctl=0x%08x\n",
				    mwx_read(sc, MT_CONN_ON_LPCTL));
				goto fail;
			}
		}
		MWX_DEV_LOG(sc, "attach MCU power control recovered\n");
	}
	MWX_DEV_LOG(sc, "attach MCU power control ok\n");
	mwx_pci_disable_aspm(sc, pa->pa_tag);

	hwid = mwx_read(sc, MT_HW_CHIPID) & 0xffff;
	hwrev = mwx_read(sc, MT_HW_REV) & 0xff;
	hwbound = mwx_read(sc, MT_HW_BOUND);
	sc->sc_hwtype = mwx_hw_type_from_chip(sc->sc_hwtype, hwid, hwbound);

	MWX_DEV_LOG(sc, "attach chip id=0x%04x hwrev=0x%02x hwbound=0x%08x type=%s interrupt=%s\n",
	    hwid, hwrev, hwbound, mwx_hw_name(sc->sc_hwtype),
	    pci_intr_string(pa->pa_pc, ih));

	mwx_set(sc, MT_MCU2HOST_SW_INT_ENA, MT_MCU_CMD_WAKE_RX_PCIE);
	if (mwx_wfsys_reset(sc) != 0) {
		MWX_DEV_LOG(sc, "attach WFSYS reset failed\n");
		goto fail;
	}
	MWX_DEV_LOG(sc, "attach WFSYS reset ok\n");

	mwx_write(sc, MT_WFDMA0_HOST_INT_ENA, 0);
	mwx_write(sc, MT_PCIE_MAC_INT_ENABLE, 0xff);

	sc->sc_ih = pci_intr_establish(pa->pa_pc, ih, IPL_NET,
	    mwx_intr, sc, DEVNAME(sc));
	if (sc->sc_ih == NULL) {
		printf("%s: can't establish interrupt\n", DEVNAME(sc));
		goto fail;
	}
	MWX_DEV_LOG(sc, "attach interrupt established\n");

	if ((error = mwx_txwi_alloc(sc, MWX_TXWI_MAX)) != 0) {
		printf("%s: failed to allocate DMA resources %d\n",
		    DEVNAME(sc), error);
		goto fail;
	}

	if ((error = mwx_dma_alloc(sc)) != 0) {
		printf("%s: failed to allocate DMA resources %d\n",
		    DEVNAME(sc), error);
		goto fail;
	}
	MWX_DEV_LOG(sc, "attach DMA resources ready\n");

	/* set regulatory domain to '00' */
	sc->sc_alpha2[0] = '0';
	sc->sc_alpha2[1] = '0';

	ic->ic_phytype = IEEE80211_T_OFDM;	/* not only, but not used */
	ic->ic_opmode = IEEE80211_M_STA;	/* default to BSS mode */
	ic->ic_state = IEEE80211_S_INIT;

	/* Set device capabilities. */
	ic->ic_caps =
#if NOTYET
	    IEEE80211_C_QOS | IEEE80211_C_TX_AMPDU | /* A-MPDU */
	    IEEE80211_C_ADDBA_OFFLOAD | /* device sends ADDBA/DELBA frames */
#endif
	    IEEE80211_C_WEP |		/* WEP */
	    IEEE80211_C_RSN |		/* WPA/RSN */
	    IEEE80211_C_SCANALL |	/* device scans all channels at once */
	    IEEE80211_C_SCANALLBAND |	/* device scans all bands at once */
	    IEEE80211_C_MONITOR |	/* monitor mode supported */
#ifndef IEEE80211_STA_ONLY
	    IEEE80211_C_IBSS |		/* IBSS mode supported */
	    IEEE80211_C_HOSTAP |	/* HostAP mode supported */
	    IEEE80211_C_APPMGT |	/* HostAP power management */
#endif
	    IEEE80211_C_SHSLOT |	/* short slot time supported */
	    IEEE80211_C_SHPREAMBLE |	/* short preamble supported */
	    IEEE80211_C_QOS |		/* WMM, required for 11n */
	    IEEE80211_C_TX_AMPDU;	/* TX A-MPDU, BA set up in firmware */

	{
		int off = 0;
		/*
		 * A-MSDUs inside received A-MPDUs are accepted unless -mtknorxamsdu:
		 * refusing them in the ADDBA answer kept an MT7921 near 40 Mbps
		 * (Arxusion's 485d6a8 turns them on in the BA agreement).
		 */
		sc->sc_rx_amsdu_enabled = !PE_parse_boot_argn("-mtknorxamsdu", &off, sizeof(off));
		if (sc->sc_rx_amsdu_enabled)
			ic->ic_caps |= IEEE80211_C_AMSDU_IN_AMPDU;
		MWX_DEV_LOG(sc, "RX A-MSDU reassembly enabled=%d max_body=3839\n",
		    sc->sc_rx_amsdu_enabled);
	}

	/*
	 * Hardware TX A-MSDU (MT_TXD7_HW_AMSDU), like Linux mt7921 and
	 * pengubow's MT7922 port: the MAC packs several MSDUs of a TID with a
	 * Block Ack agreement into one A-MSDU when the AP allows A-MSDU in
	 * A-MPDU. Fewer, larger frames: more upload.
	 */
	{
		int off = 0;
		sc->sc_tx_amsdu = !PE_parse_boot_argn("-mtknoamsdu", &off,
		    sizeof(off));
		if (sc->sc_tx_amsdu)
			ic->ic_caps |= IEEE80211_C_TX_AMSDU_IN_AMPDU;
	}

	/*
	 * Management frame protection (802.11w), negotiated with the access
	 * points that advertise it: without it anyone in range disconnects
	 * the station with a forged Deauthentication or Disassociation.
	 * Robust management frames to and from the access point use the
	 * pairwise key in hardware, like data (MT_TXD3_PROTECT_FRAME on
	 * transmit, mwx_rx_hwdec_restore() on receive); the group-addressed
	 * ones the access point sends are verified in software (BIP, see
	 * mwx_set_key()). WPA3-Personal (SAE), on by default, needs it, so it is
	 * offered with it; -mtkwpa3off turns both off, and then -mtkpmf offers
	 * 802.11w alone. -mtkpmfreq also refuses access points that do not
	 * support it.
	 */
	{
		int flag = 0;

		ic->ic_wpa3 = !PE_parse_boot_argn("-mtkwpa3off", &flag,
		    sizeof(flag));
		if (ic->ic_wpa3 ||
		    PE_parse_boot_argn("-mtkpmf", &flag, sizeof(flag)) ||
		    PE_parse_boot_argn("-mtkpmfreq", &flag, sizeof(flag))) {
			ic->ic_caps |= IEEE80211_C_MFP;
			if (PE_parse_boot_argn("-mtkpmfreq", &flag,
			    sizeof(flag)))
				ic->ic_flags |= IEEE80211_F_MFPR;
		}
		MWX_DEV_LOG(sc,
		    "management frame protection offered=%d required=%d wpa3=%d\n",
		    !!(ic->ic_caps & IEEE80211_C_MFP),
		    !!(ic->ic_flags & IEEE80211_F_MFPR), ic->ic_wpa3);
	}

	/*
	 * 802.11n. 20/40 MHz must be advertised even when only VHT80 is used:
	 * an AP treats a station without the HT "20/40" bit as 20 MHz-only and
	 * never transmits 80 MHz to it. 40 MHz is only actually used on 5 GHz
	 * (2.4 GHz channels are not marked HT40, so net80211 masks the bit).
	 */
	ic->ic_htcaps = IEEE80211_HTCAP_LDPC |
	    IEEE80211_HTCAP_SGI20 | IEEE80211_HTCAP_SGI40 |
	    IEEE80211_HTCAP_CBW20_40 |
	    (1U << IEEE80211_HTCAP_RXSTBC_SHIFT);	/* RX STBC, 1 stream */
	ic->ic_htcaps |=
	    (IEEE80211_HTCAP_SMPS_DIS << IEEE80211_HTCAP_SMPS_SHIFT);
	ic->ic_htxcaps = 0;
	ic->ic_txbfcaps = 0;
	ic->ic_aselcaps = 0;
	ic->ic_ampdu_params = (IEEE80211_AMPDU_PARAM_SS_4 | 0x3 /* 64k */);

	/*
	 * 802.11ac: 80 MHz, SGI80, RX LDPC, MCS 0-9 on 2 streams. MPDUs are
	 * capped at 3895 bytes and A-MPDUs at 64K (RX buffers are 2 KB clusters).
	 */
	/*
	 * SU/MU beamformee with 4 sounding dimensions (STS 3), like Linux
	 * mt7921 and pengubow's MT7922 port: lets the AP beamform toward us,
	 * worth a few dB (higher MCS) on a weak link. The firmware answers the
	 * sounding (NDPA/NDP) by itself.
	 */
	ic->ic_vhtcaps = IEEE80211_VHTCAP_MAX_MPDU_LENGTH_3895 |
	    IEEE80211_VHTCAP_RXLDPC | IEEE80211_VHTCAP_SHORT_GI_80 |
	    IEEE80211_VHTCAP_RXSTBC_1 |
	    IEEE80211_VHTCAP_SU_BEAMFORMEE_CAPABLE |
	    IEEE80211_VHTCAP_MU_BEAMFORMEE_CAPABLE |
	    (3U << IEEE80211_VHTCAP_BEAMFORMEE_STS_SHIFT) |
	    (3U << IEEE80211_VHTCAP_MAX_A_MPDU_LENGTH_EXPONENT_SHIFT) |
	    IEEE80211_VHTCAP_RX_ANTENNA_PATTERN |
	    IEEE80211_VHTCAP_TX_ANTENNA_PATTERN;
	if (sc->sc_vht160)
		ic->ic_vhtcaps |= IEEE80211_VHTCAP_SUPP_CHAN_WIDTH_160MHZ |
		    IEEE80211_VHTCAP_SHORT_GI_160;
	/* 2 bits per NSS: 2 = MCS 0-9, 3 = not supported */
	ic->ic_vht_rx_mcs_map = 0xfffa;
	ic->ic_vht_tx_mcs_map = 0xfffa;
	ic->ic_vht_rx_highest = 0;
	ic->ic_vht_tx_highest = 0;

#if NOTYET
	ic->ic_vhtcaps = IEEE80211_VHTCAP_MAX_MPDU_LENGTH_11454 |
	    (IEEE80211_VHTCAP_MAX_AMPDU_LEN_64K <<
	    IEEE80211_VHTCAP_MAX_AMPDU_LEN_SHIFT) |
	    (IEEE80211_VHTCAP_CHAN_WIDTH_80 <<
	    IEEE80211_VHTCAP_CHAN_WIDTH_SHIFT) | IEEE80211_VHTCAP_SGI80 |
	    IEEE80211_VHTCAP_RX_ANT_PATTERN | IEEE80211_VHTCAP_TX_ANT_PATTERN;
#endif

	/* IBSS channel undefined for now. */
	ic->ic_ibss_chan = &ic->ic_channels[1];

	/* HW supports up to 288 STAs in HostAP and IBSS modes */
	ic->ic_max_aid = min(IEEE80211_AID_MAX, MWX_WTBL_STA - 1);

	ic->ic_max_rssi = 100;	/* rxi_rssi is normalized for ni_rssi. */

	ifp->if_softc = sc;
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_ioctl = mwx_ioctl;
	ifp->if_start = mwx_start;
	ifp->if_watchdog = mwx_watchdog;
	memcpy(ifp->if_xname, DEVNAME(sc), IFNAMSIZ);

	if_attach(ifp);
	ieee80211_ifattach(ifp, gMwxHal->mwxController());
	sc->sc_net_attached = true;
	ieee80211_media_init(ifp);
	MWX_DEV_LOG(sc, "attach net80211 attached caps=0x%x max_aid=%u\n",
	    ic->ic_caps, ic->ic_max_aid);

#if NBPFILTER > 0
	mwx_radiotap_attach(sc);
#endif

	ic->ic_node_alloc = mwx_node_alloc;
	ic->ic_newassoc = mwx_newassoc;
	ic->ic_updateedca = mwx_updateedca;
#ifndef IEEE80211_STA_ONLY
	ic->ic_node_leave = mwx_node_leave;
#endif
	ic->ic_bgscan_start = mwx_bgscan;
	ic->ic_bgscan_done = mwx_bgscan_done;
	ic->ic_set_key = mwx_set_key;
	ic->ic_ampdu_rx_start = mwx_ampdu_rx_start;
	ic->ic_ampdu_rx_stop = mwx_ampdu_rx_stop;
	ic->ic_ampdu_tx_start = mwx_ampdu_tx_start;
	ic->ic_ampdu_tx_stop = mwx_ampdu_tx_stop;
	ic->ic_delete_key = mwx_delete_key;

	/* Override 802.11 state transition machine. */
	sc->sc_newstate = ic->ic_newstate;
	ic->ic_newstate = mwx_newstate;

	sc->sc_nswq = taskq_create("mwxns", 1, IPL_NET, 0);
	if (sc->sc_nswq == NULL) {
		printf("%s: can't create task queue\n", DEVNAME(sc));
		goto fail;
	}

	task_set(&sc->sc_newstate_task, mwx_newstate_task_gated, sc, "mwx_newstate");
	task_set(&sc->sc_bgscan_done_task, mwx_bgscan_done_task_gated, sc, "mwx_bgscan");
	task_set(&sc->sc_setkey_task, mwx_setkey_task_gated, sc, "mwx_setkey");
	task_set(&sc->sc_reset_task, mwx_reset_task_gated, sc, "mwx_reset");
	task_set(&sc->sc_scan_task, mwx_end_scan_task_gated, sc, "mwx_scan");
	timeout_set(&sc->sc_reset_to, mwx_reset_timeo, sc);

	/*
	 * We cannot read the MAC address without loading the
	 * firmware from disk. Postpone until mountroot is done.
	 */
	config_mountroot(self, mwx_attach_hook);

	MWX_DEV_LOG(sc, "attach complete\n");
	return true;

fail:
	/* The HAL caller owns partial-attach cleanup through detach(). */
	return false;
}

int
mwx_activate(struct device *self, int act)
{
	/* XXX TODO */
	return 0;
}

#if 0
/* OpenBSD autoconf metadata is replaced by SimpleMtkWlan's IOKit matching path. */
struct cfdriver mwx_cd = {
	NULL, "mwx", DV_IFNET
};

struct cfattach mwx_ca = {
	sizeof(struct mwx_softc), mwx_match, mwx_attach,
	NULL, mwx_activate
};
#endif

void
mwx_reset(struct mwx_softc *sc)
{
	if (sc->sc_resetting || sc->sc_detaching)
		return;
	sc->sc_resetting = 1;
	task_add(sc->sc_nswq, &sc->sc_reset_task);
}

void
mwx_reset_timeo(void *arg)
{
	mwx_reset((struct mwx_softc *)arg);
}

void
mwx_reset_task(void *arg)
{
	struct mwx_softc *sc = (struct mwx_softc *)arg;
	struct ifnet *ifp = &sc->sc_ic.ic_if;
	int fatal = 0;

	if (ifp->if_flags & IFF_RUNNING)
		mwx_stop(ifp);

	if (sc->sc_hard_reset) {
		/* mwx_init() -> mwx_preinit() does WFSYS reset + firmware load. */
		MWX_DEV_LOG(sc, "hard reset: reloading firmware\n");
		sc->sc_hard_reset = 0;
		sc->sc_fw_loaded = 0;
	}

	if (!fatal && (ifp->if_flags & (IFF_UP | IFF_RUNNING)) == IFF_UP) {
		int rv = mwx_init(ifp);
		MWX_DEV_LOG(sc, "reset done init=%d fw_loaded=%d\n", rv,
		    sc->sc_fw_loaded);
	}
	sc->sc_resetting = 0;
	sc->sc_coredump_cnt = 0;
}

int
mwx_txwi_alloc(struct mwx_softc *sc, int count)
{
	int error, nsegs, i;
	struct mwx_txwi_desc *q = &sc->sc_txwi;
	bus_size_t size = count * sizeof(*q->mt_desc);
	uint32_t addr;

	MWX_DEV_LOG(sc, "txwi alloc start count=%d size=%zu\n",
	    count, (size_t)size);
	LIST_INIT(&q->mt_freelist);
	q->mt_nfree = 0;
	q->mt_count = count;

	error = bus_dmamap_create(sc->sc_dmat, size, 1, size, 0,
	    BUS_DMA_NOWAIT, &q->mt_map);
	if (error != 0) {
		printf("%s: could not create desc TWXI map\n", DEVNAME(sc));
		goto fail;
	}

	error = bus_dmamem_alloc(sc->sc_dmat, size, PAGE_SIZE, 0, &q->mt_seg,
	    1, &nsegs, BUS_DMA_NOWAIT | BUS_DMA_ZERO);
	if (error != 0) {
		printf("%s: could not allocate TWXI memory\n", DEVNAME(sc));
		goto fail;
	}

	error = bus_dmamem_map(sc->sc_dmat, &q->mt_seg, nsegs, size,
	    (void **)&q->mt_desc, BUS_DMA_NOWAIT);
	if (error != 0) {
		printf("%s: can't map desc DMA memory\n", DEVNAME(sc));
		goto fail;
	}

	error = bus_dmamap_load(sc->sc_dmat, q->mt_map, q->mt_desc,
	    size, NULL, BUS_DMA_NOWAIT);
	if (error != 0) {
		printf("%s: could not load desc DMA map\n", DEVNAME(sc));
		goto fail;
	}

	addr = q->mt_map->dm_segs[0].ds_addr;

	q->mt_data = (struct mwx_txwi *)mallocarray(count, sizeof(*q->mt_data),
	    M_DEVBUF, M_NOWAIT | M_ZERO);
	if (q->mt_data == NULL) {
		printf("%s: could not allocate soft data\n", DEVNAME(sc));
		error = ENOMEM;
		goto fail;
	}

	for (i = 0; i < count; i++) {
		q->mt_data[i].mt_idx = i;
		q->mt_data[i].mt_desc = &q->mt_desc[i];
		q->mt_data[i].mt_addr = addr + i * sizeof(*q->mt_desc);
		error = bus_dmamap_create(sc->sc_dmat, MT_TXD_LEN_MASK,
		    MT_MAX_SCATTER, MT_TXD_LEN_MASK, 0, BUS_DMA_NOWAIT,
		    &q->mt_data[i].mt_map);
		if (error != 0) {
			printf("%s: could not create data DMA map\n",
			    DEVNAME(sc));
			goto fail;
		}
	}

	for (i = count - 1; i >= MT_PACKET_ID_FIRST; i--) {
		LIST_INSERT_HEAD(&q->mt_freelist, &q->mt_data[i], mt_entry);
		q->mt_nfree++;
	}

	MWX_DEV_LOG(sc, "txwi alloc ok count=%d dma=0x%08x\n", count, addr);
	return 0;

fail:
	MWX_DEV_LOG(sc, "txwi alloc failed error=%d\n", error);
	mwx_txwi_free(sc);
	return error;
}

void
mwx_txwi_free(struct mwx_softc *sc)
{
	struct mwx_txwi_desc *q = &sc->sc_txwi;

	if (q->mt_data != NULL) {
		int i;
		for (i = 0; i < q->mt_count; i++) {
			struct mwx_txwi *mt = &q->mt_data[i];
			bus_dmamap_destroy(sc->sc_dmat, mt->mt_map);
			m_freem(mt->mt_mbuf);
			if (i >= MT_PACKET_ID_FIRST)
				LIST_REMOVE(mt, mt_entry);
		}
		free(q->mt_data, M_DEVBUF, q->mt_count * sizeof(*q->mt_data));
	}

	if (q->mt_desc != NULL) {
		bus_dmamap_sync(sc->sc_dmat, q->mt_map, 0,
			q->mt_map->dm_mapsize, BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(sc->sc_dmat, q->mt_map);
	}

	/*
	 * XXX TODO this is probably not correct as a check, should use
	 * some state variable bitfield to decide which steps need to be run.
	 */
	if (q->mt_seg.ds_len != 0)
		bus_dmamem_unmap(sc->sc_dmat, (caddr_t)q->mt_desc,
			q->mt_count * sizeof(*q->mt_desc));
	if (q->mt_map != NULL)
		bus_dmamem_free(sc->sc_dmat, &q->mt_seg, 1);

	memset(q, 0, sizeof(*q));
}

struct mwx_txwi *
mwx_txwi_get(struct mwx_softc *sc)
{
	struct mwx_txwi *mt;

	mt = LIST_FIRST(&sc->sc_txwi.mt_freelist);
	if (mt == NULL)
		return NULL;
	LIST_REMOVE(mt, mt_entry);
	sc->sc_txwi.mt_nfree--;
	mt->mt_busy = 1;
	return mt;
}

void
mwx_txwi_put(struct mwx_softc *sc, struct mwx_txwi *mt)
{
	if (mt->mt_busy == 0)
		return;

	if (mt->mt_mbuf != NULL) {
		bus_dmamap_sync(sc->sc_dmat, mt->mt_map, 0,
		    mt->mt_map->dm_mapsize, BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(sc->sc_dmat, mt->mt_map);
		m_freem(mt->mt_mbuf);
		mt->mt_mbuf = NULL;
	}

	memset(mt->mt_desc, 0, sizeof(*mt->mt_desc));
	mt->mt_busy = 0;

	if (mt->mt_idx < MT_PACKET_ID_FIRST)
		return;
	LIST_INSERT_HEAD(&sc->sc_txwi.mt_freelist, mt, mt_entry);
	sc->sc_txwi.mt_nfree++;
}

int
mwx_txwi_enqueue(struct mwx_softc *sc, struct mwx_txwi *mt, struct mbuf *m)
{
	struct mwx_txwi_desc *q = &sc->sc_txwi;
	struct mt76_txwi *txp = mt->mt_desc;
	struct mt76_connac_txp_ptr *ptr = &txp->ptr[0];
	uint32_t addr;
	uint16_t len;
	int i, nsegs, rv;

	rv = bus_dmamap_load_mbuf(sc->sc_dmat, mt->mt_map, m,
	    BUS_DMA_WRITE | BUS_DMA_NOWAIT);
	if (rv == EFBIG && m_defrag(m, M_DONTWAIT) == 0)
		rv = bus_dmamap_load_mbuf(sc->sc_dmat, mt->mt_map, m,
		    BUS_DMA_WRITE | BUS_DMA_NOWAIT);
	if (rv != 0)
		return rv;

	nsegs = mt->mt_map->dm_nsegs;

	{
		static uint32_t seg_log_count;
		if (seg_log_count < 64) {
			struct mbuf *c;
			int nmb = 0;
			char chain[96];
			size_t off = 0;

			chain[0] = 0;
			for (c = m; c != NULL && off < sizeof(chain) - 12;
			    c = mbuf_next(c), nmb++)
				off += snprintf(chain + off, sizeof(chain) - off,
				    "%zu%s,", mbuf_len(c),
				    (mbuf_flags(c) & MBUF_EXT) ? "e" : "");
			MWX_DEV_LOG(sc,
			    "txseg idx=%u len=%zu nmbuf=%d chain=%s nsegs=%d bounce=%d s0=0x%llx/%llu s1=0x%llx/%llu s2=0x%llx/%llu s3=0x%llx/%llu\n",
			    mt->mt_idx, mbuf_pkthdr_len(m), nmb, chain, nsegs,
			    mt->mt_map->dm_bounce,
			    nsegs > 0 ? (unsigned long long)mt->mt_map->dm_segs[0].ds_addr : 0ULL,
			    nsegs > 0 ? (unsigned long long)mt->mt_map->dm_segs[0].ds_len : 0ULL,
			    nsegs > 1 ? (unsigned long long)mt->mt_map->dm_segs[1].ds_addr : 0ULL,
			    nsegs > 1 ? (unsigned long long)mt->mt_map->dm_segs[1].ds_len : 0ULL,
			    nsegs > 2 ? (unsigned long long)mt->mt_map->dm_segs[2].ds_addr : 0ULL,
			    nsegs > 2 ? (unsigned long long)mt->mt_map->dm_segs[2].ds_len : 0ULL,
			    nsegs > 3 ? (unsigned long long)mt->mt_map->dm_segs[3].ds_addr : 0ULL,
			    nsegs > 3 ? (unsigned long long)mt->mt_map->dm_segs[3].ds_len : 0ULL);
			seg_log_count++;
		}
	}
	if (nsegs > MT_HW_TXP_MAX_BUF_NUM) {
		MWX_DEV_LOG(sc, "txseg idx=%u too many segments %d, dropping\n",
		    mt->mt_idx, nsegs);
		bus_dmamap_unload(sc->sc_dmat, mt->mt_map);
		return EFBIG;
	}

	bus_dmamap_sync(sc->sc_dmat, mt->mt_map, 0, mt->mt_map->dm_mapsize,
	    BUS_DMASYNC_PREWRITE);

	txp->msdu_id[0] = htole16(mt->mt_idx | MT_MSDU_ID_VALID);
	mt->mt_mbuf = m;

	bus_dmamap_sync(sc->sc_dmat, q->mt_map, 0, q->mt_map->dm_mapsize,
	    BUS_DMASYNC_PREWRITE);
	for (i = 0; i < nsegs; i++) {
		KASSERT(mt->mt_map->dm_segs[i].ds_addr <= UINT32_MAX);
		KASSERT(mt->mt_map->dm_segs[i].ds_len <= MT_TXD_LEN_MASK);
		addr = mt->mt_map->dm_segs[i].ds_addr;
		len = mt->mt_map->dm_segs[i].ds_len;

		if (i == nsegs - 1)
			len |= MT_TXD_LEN_LAST;

		if ((i & 1) == 0) {
			ptr->buf0 = htole32(addr);
			ptr->len0 = htole16(len);
		} else {
			ptr->buf1 = htole32(addr);
			ptr->len1 = htole16(len);
			ptr++;
		}
	}
	bus_dmamap_sync(sc->sc_dmat, q->mt_map, 0, q->mt_map->dm_mapsize,
	    BUS_DMASYNC_POSTWRITE);

	return 0;
}

int
mwx_queue_alloc(struct mwx_softc *sc, struct mwx_queue *q, int count,
    uint32_t regbase)
{
	int error, nsegs, i;
	bus_size_t size = count * sizeof(*q->mq_desc);

	MWX_DEV_LOG(sc, "queue alloc start regbase=0x%08x count=%d size=%zu\n",
	    regbase, count, (size_t)size);
	q->mq_regbase = regbase;
	q->mq_count = count;

	error = bus_dmamap_create(sc->sc_dmat, size, 1, size, 0,
	    BUS_DMA_NOWAIT, &q->mq_map);
	if (error != 0) {
		printf("%s: could not create desc DMA map\n", DEVNAME(sc));
		goto fail;
	}

	error = bus_dmamem_alloc(sc->sc_dmat, size, PAGE_SIZE, 0, &q->mq_seg,
	    1, &nsegs, BUS_DMA_NOWAIT | BUS_DMA_ZERO);
	if (error != 0) {
		printf("%s: could not allocate DMA memory\n", DEVNAME(sc));
		goto fail;
	}

	error = bus_dmamem_map(sc->sc_dmat, &q->mq_seg, nsegs, size,
	    (void **)&q->mq_desc, BUS_DMA_NOWAIT);
	if (error != 0) {
		printf("%s: can't map desc DMA memory\n", DEVNAME(sc));
		goto fail;
	}

	error = bus_dmamap_load(sc->sc_dmat, q->mq_map, q->mq_desc,
	    size, NULL, BUS_DMA_NOWAIT);
	if (error != 0) {
		printf("%s: could not load desc DMA map\n", DEVNAME(sc));
		goto fail;
	}

	q->mq_data = (struct mwx_queue_data *)mallocarray(count, sizeof(*q->mq_data),
	    M_DEVBUF, M_NOWAIT | M_ZERO);
	if (q->mq_data == NULL) {
		printf("%s: could not allocate soft data\n", DEVNAME(sc));
		error = ENOMEM;
		goto fail;
	}

	for (i = 0; i < count; i++) {
		error = bus_dmamap_create(sc->sc_dmat, MT_MAX_SIZE,
		    MT_MAX_SCATTER, MT_MAX_SIZE, 0, BUS_DMA_NOWAIT,
		    &q->mq_data[i].md_map);
		if (error != 0) {
			printf("%s: could not create data DMA map\n",
			    DEVNAME(sc));
			goto fail;
		}
	}

	mwx_queue_reset(sc, q);
	MWX_DEV_LOG(sc, "queue alloc ok regbase=0x%08x count=%d dma=0x%llx\n",
	    regbase, count, (unsigned long long)q->mq_map->dm_segs[0].ds_addr);
	return 0;

fail:
	MWX_DEV_LOG(sc, "queue alloc failed regbase=0x%08x error=%d\n",
	    regbase, error);
	mwx_queue_free(sc, q);
	return error;
}

void
mwx_queue_free(struct mwx_softc *sc, struct mwx_queue *q)
{
	/* A partial frame no longer belongs to any ring slot. */
	m_freem(q->mq_rx_head);
	q->mq_rx_head = q->mq_rx_tail = NULL;
	q->mq_rx_bytes = q->mq_rx_segments = 0;
	if (q->mq_data != NULL) {
		int i;
		for (i = 0; i < q->mq_count; i++) {
			struct mwx_queue_data  *md = &q->mq_data[i];
			bus_dmamap_destroy(sc->sc_dmat, md->md_map);
			m_freem(md->md_mbuf);
		}
		free(q->mq_data, M_DEVBUF, q->mq_count * sizeof(*q->mq_data));
	}

	if (q->mq_desc != NULL) {
		bus_dmamap_sync(sc->sc_dmat, q->mq_map, 0,
			q->mq_map->dm_mapsize, BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(sc->sc_dmat, q->mq_map);
	}

	/*
	 * XXX TODO this is probably not correct as a check, should use
	 * some state variable bitfield to decide which steps need to be run.
	 */
	if (q->mq_seg.ds_len != 0)
		bus_dmamem_unmap(sc->sc_dmat, (caddr_t)q->mq_desc,
			q->mq_count * sizeof(*q->mq_desc));
	if (q->mq_map != NULL)
		bus_dmamem_free(sc->sc_dmat, &q->mq_seg, 1);

	memset(q, 0, sizeof(*q));
}

void
mwx_queue_reset(struct mwx_softc *sc, struct mwx_queue *q)
{
	int i;
	uint32_t dmaaddr;
	struct mwx_queue_data *md;

	/* Reset must release detached fragments as well as posted buffers. */
	m_freem(q->mq_rx_head);
	q->mq_rx_head = q->mq_rx_tail = NULL;
	q->mq_rx_bytes = q->mq_rx_segments = 0;
	q->mq_rx_wait_ns = 0;

	/* clear descriptors */
	bus_dmamap_sync(sc->sc_dmat, q->mq_map, 0, q->mq_map->dm_mapsize,
	    BUS_DMASYNC_PREWRITE);

	for (i = 0; i < q->mq_count; i++) {
		q->mq_desc[i].buf0 = 0;
		q->mq_desc[i].buf1 = 0;
		q->mq_desc[i].info = 0;
		q->mq_desc[i].ctrl = htole32(MT_DMA_CTL_DMA_DONE);
	}

	bus_dmamap_sync(sc->sc_dmat, q->mq_map, 0, q->mq_map->dm_mapsize,
	    BUS_DMASYNC_POSTWRITE);

	/* reset DMA registers */
	KASSERT(q->mq_map->dm_nsegs == 1);
	KASSERT(q->mq_map->dm_segs[0].ds_addr <= UINT32_MAX);
	dmaaddr	= q->mq_map->dm_segs[0].ds_addr;
	mwx_write(sc, q->mq_regbase + MT_DMA_DESC_BASE, dmaaddr);
	mwx_write(sc, q->mq_regbase + MT_DMA_RING_SIZE, q->mq_count);
	mwx_write(sc, q->mq_regbase + MT_DMA_CPU_IDX, 0);
	mwx_write(sc, q->mq_regbase + MT_DMA_DMA_IDX, 0);
	q->mq_cons = 0;
	q->mq_prod = 0;
	q->mq_refill_pending = false;
	q->mq_rx_discard = false;

	/* free buffers */
	for (i = 0; i < q->mq_count; i++) {
		md = &q->mq_data[i];
		if (md->md_mbuf != NULL) {
			bus_dmamap_sync(sc->sc_dmat, md->md_map, 0,
			    md->md_map->dm_mapsize,
			    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
			bus_dmamap_unload(sc->sc_dmat, md->md_map);
			m_freem(md->md_mbuf);
			md->md_mbuf = NULL;
		}
		if (md->md_txwi != NULL) {
			/* return TXWI (and its frame) owned by this slot */
			mwx_txwi_put(sc, md->md_txwi);
			md->md_txwi = NULL;
		}
	}
}

/*
 * Page pool for DMA-safe RX buffers. All pages come from one allocation
 * whose physical addresses satisfy the device's 32-bit mask, so RX DMA
 * writes the mbuf cluster directly and the bounce copy disappears.
 * mwx_buf_fill runs on the interrupt/workloop thread; the free callback
 * can run on any thread, hence the simple lock.
 */
#define MWX_RX_POOL_PAGES	16384	/* 64 MiB / 4 KiB */
#define MWX_RX_POOL_MASK	0x00000000ffffffffULL

static void
mwx_rx_pool_free_page(caddr_t addr, u_int size, caddr_t arg)
{
	struct mwx_softc *sc = (struct mwx_softc *)arg;
	struct mwx_rx_page *page = NULL;

	if (sc == NULL || sc->sc_rx_pool_pages == NULL)
		return;
	/* The mbuf hands back the page start; locate the page arithmetically. */
	const uint64_t offset = (uint64_t)((uint8_t *)addr - sc->sc_rx_pool_kva);
	if (offset <= (uint64_t)(sc->sc_rx_pool_end - sc->sc_rx_pool_kva) &&
	    (offset & (PAGE_SIZE - 1)) == 0 &&
	    (offset >> PAGE_SHIFT) < sc->sc_rx_pool_total)
		page = &sc->sc_rx_pool_pages[offset >> PAGE_SHIFT];
	if (page == NULL)
		return;
	IOSimpleLockLock(sc->sc_rx_pool_lock);
	SLIST_INSERT_HEAD(&sc->sc_rx_pool_free, page, link);
	sc->sc_rx_pool_free_cnt++;
	sc->sc_rx_pool_returned++;
	IOSimpleLockUnlock(sc->sc_rx_pool_lock);
	(void)size;
}

static void
mwx_rx_pool_free(struct mwx_softc *sc)
{
	/* Pages and the page array live in/from the descriptor memory. */
	if (sc->sc_rx_pool_md != NULL) {
		sc->sc_rx_pool_md->release();
		sc->sc_rx_pool_md = NULL;
	}
	if (sc->sc_rx_pool_pages != NULL) {
		mwx_free_mem(sc->sc_rx_pool_pages);
		sc->sc_rx_pool_pages = NULL;
	}
	SLIST_INIT(&sc->sc_rx_pool_free);
	sc->sc_rx_pool_kva = NULL;
	sc->sc_rx_pool_end = NULL;
	sc->sc_rx_pool_total = 0;
	sc->sc_rx_pool_free_cnt = 0;
}

/* Physical address of a pool page: the descriptor reports each page's bus
 * address, which equals the physical address on this platform (no IOMMU
 * translation for the pool's own mapping). */
static uint32_t
mwx_rx_pool_page_phys(struct mwx_softc *sc, uint8_t *kva)
{
	uint32_t offset = (uint32_t)(kva - sc->sc_rx_pool_kva);
	IOByteCount remaining = 0;
	/* kIOMemoryMapperNone yields the host physical address directly. */
	addr64_t seg = sc->sc_rx_pool_md->getPhysicalSegment(offset,
	    &remaining, kIOMemoryMapperNone);
	return (uint32_t)seg;
}

static int
mwx_rx_pool_alloc(struct mwx_softc *sc)
{
	int off = 0;

	sc->sc_rx_pool_md = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
	    kernel_task, kIODirectionIn, MWX_RX_POOL_PAGES * PAGE_SIZE,
	    MWX_RX_POOL_MASK);
	if (sc->sc_rx_pool_md == NULL)
		return ENOMEM;
	sc->sc_rx_pool_kva = (uint8_t *)sc->sc_rx_pool_md->getBytesNoCopy();
	if (sc->sc_rx_pool_kva == NULL) {
		mwx_rx_pool_free(sc);
		return ENOMEM;
	}
	sc->sc_rx_pool_end = sc->sc_rx_pool_kva + MWX_RX_POOL_PAGES * PAGE_SIZE;
	sc->sc_rx_pool_pages = (struct mwx_rx_page *)mallocarray(
	    MWX_RX_POOL_PAGES, sizeof(struct mwx_rx_page), M_DEVBUF,
	    M_NOWAIT | M_ZERO);
	if (sc->sc_rx_pool_pages == NULL) {
		mwx_rx_pool_free(sc);
		return ENOMEM;
	}
	sc->sc_rx_pool_total = MWX_RX_POOL_PAGES;
	for (uint32_t i = 0; i < MWX_RX_POOL_PAGES; i++) {
		sc->sc_rx_pool_pages[i].kva =
		    sc->sc_rx_pool_kva + (off + i) * PAGE_SIZE;
		SLIST_INSERT_HEAD(&sc->sc_rx_pool_free,
		    &sc->sc_rx_pool_pages[i], link);
	}
	sc->sc_rx_pool_free_cnt = MWX_RX_POOL_PAGES;
	return 0;
}

int
mwx_buf_fill(struct mwx_softc *sc, struct mwx_queue_data *md,
    struct mwx_desc *desc)
{
	struct mbuf *m = NULL;
	struct mwx_rx_page *page = NULL;
	uint32_t buf0, len0, ctrl;
	int rv;

	/* Never overwrite a slot whose buffer is still owned by the ring. */
	if (md->md_mbuf != NULL)
		return EBUSY;

	/* Prefer a DMA-safe pool page: no bounce copy on completion. */
	if (!sc->sc_rx_pool_disabled && sc->sc_rx_pool_lock != NULL &&
	    sc->sc_rx_pool_kva != NULL) {
		IOSimpleLockLock(sc->sc_rx_pool_lock);
		page = SLIST_FIRST(&sc->sc_rx_pool_free);
		if (page != NULL) {
			SLIST_REMOVE_HEAD(&sc->sc_rx_pool_free, link);
			sc->sc_rx_pool_free_cnt--;
		}
		IOSimpleLockUnlock(sc->sc_rx_pool_lock);
		sc->sc_rx_pool_allocs++;
		if (page == NULL)
			sc->sc_rx_pool_exhausted++;
	}

	if (page != NULL) {
		if (mbuf_gethdr(M_DONTWAIT, MBUF_TYPE_DATA, &m) != 0) {
			IOSimpleLockLock(sc->sc_rx_pool_lock);
			SLIST_INSERT_HEAD(&sc->sc_rx_pool_free, page, link);
			sc->sc_rx_pool_free_cnt++;
			IOSimpleLockUnlock(sc->sc_rx_pool_lock);
			return ENOMEM;
		}
		if (mbuf_attachcluster(M_DONTWAIT, MBUF_TYPE_DATA, &m,
		    (caddr_t)page->kva, mwx_rx_pool_free_page, PAGE_SIZE,
		    (caddr_t)sc) != 0) {
			mbuf_free(m);
			IOSimpleLockLock(sc->sc_rx_pool_lock);
			SLIST_INSERT_HEAD(&sc->sc_rx_pool_free, page, link);
			sc->sc_rx_pool_free_cnt++;
			IOSimpleLockUnlock(sc->sc_rx_pool_lock);
			return ENOMEM;
		}
		/* The RXD lives at the buffer start; keep the full page. */
		mbuf_setlen(m, MT_RX_BUF_SIZE);
		mbuf_pkthdr_setlen(m, MT_RX_BUF_SIZE);
		md->md_mbuf = m;
		buf0 = mwx_rx_pool_page_phys(sc, page->kva);
		/* No load was performed for pool pages; publish the segment
		 * metadata the RX completion path validates against. */
		md->md_map->dm_mbuf = m;
		md->md_map->dm_bounce = false;
		md->md_map->dm_nsegs = 1;
		md->md_map->dm_mapsize = PAGE_SIZE;
		md->md_map->dm_segs[0].ds_addr = buf0;
		md->md_map->dm_segs[0].ds_len = MT_RX_BUF_SIZE;

		len0 = MT_RX_BUF_SIZE;
		ctrl = MT_DMA_CTL_SD_LEN0(len0);
		ctrl |= MT_DMA_CTL_LAST_SEC0;
		desc->buf0 = htole32(buf0);
		desc->buf1 = 0;
		desc->info = 0;
		desc->ctrl = htole32(ctrl);
		return 0;
	}

	sc->sc_rx_pool_fallbacks++;
	m = MCLGETL(NULL, M_DONTWAIT, MT_RX_BUF_SIZE);
	if (m == NULL)
		return (ENOMEM);

	mbuf_setlen(m, MT_RX_BUF_SIZE);
	mbuf_pkthdr_setlen(m, MT_RX_BUF_SIZE);

	rv = bus_dmamap_load_mbuf(sc->sc_dmat, md->md_map, m,
	     BUS_DMA_READ | BUS_DMA_NOWAIT);
	if (rv != 0) {
		printf("%s: could not load data, %d\n", DEVNAME(sc), rv);
		bus_dmamap_unload(sc->sc_dmat, md->md_map);
		m_freem(m);
		return (rv);
	}

	bus_dmamap_sync(sc->sc_dmat, md->md_map, 0,
	    md->md_map->dm_mapsize, BUS_DMASYNC_PREREAD);

	md->md_mbuf = m;

	KASSERT(md->md_map->dm_nsegs == 1);
	KASSERT(md->md_map->dm_segs[0].ds_addr <= UINT32_MAX);
	buf0 = md->md_map->dm_segs[0].ds_addr;
	len0 = md->md_map->dm_segs[0].ds_len;
	ctrl = MT_DMA_CTL_SD_LEN0(len0);
	ctrl |= MT_DMA_CTL_LAST_SEC0;

	desc->buf0 = htole32(buf0);
	desc->buf1 = 0;
	desc->info = 0;
	desc->ctrl = htole32(ctrl);

	return 0;
}

int
mwx_queue_fill(struct mwx_softc *sc, struct mwx_queue *q)
{
	u_int idx, last;
	int rv = 0;

	last = (q->mq_count + q->mq_cons - 1) % q->mq_count;
	idx = q->mq_prod;

	bus_dmamap_sync(sc->sc_dmat, q->mq_map, 0, q->mq_map->dm_mapsize,
	    BUS_DMASYNC_PREWRITE);

	while (idx != last) {
		rv = mwx_buf_fill(sc, &q->mq_data[idx], &q->mq_desc[idx]);
		if (rv != 0) {
			sc->sc_rx_refill_fail++;
			if (!q->mq_refill_pending)
				MWX_DEV_LOG(sc, "rx refill deferred reg=0x%x slot=%u rv=%d\n",
				    q->mq_regbase, idx, rv);
			q->mq_refill_pending = true;
			break;
		}
		idx = (idx + 1) % q->mq_count;
	}

	/* Publish every successfully filled slot, including a partial batch.
	 * A retry starts at the first missing slot, never at an owned mbuf. */
	bus_dmamap_sync(sc->sc_dmat, q->mq_map, 0, q->mq_map->dm_mapsize,
	    BUS_DMASYNC_POSTWRITE);
	if (idx != q->mq_prod) {
		q->mq_prod = idx;
		mwx_write(sc, q->mq_regbase + MT_DMA_CPU_IDX, q->mq_prod);
	}
	if (rv == 0) {
		if (q->mq_refill_pending)
			sc->sc_rx_refill_recover++;
		q->mq_refill_pending = false;
	}
	return rv;
}

/* The watchdog invokes debugPoll under the driver gate. One bounded fill
 * attempt per RX ring per poll also recovers a ring with no new interrupts. */
static void
mwx_rx_refill_retry(struct mwx_softc *sc)
{
	if (!sc->sc_fw_loaded || sc->sc_detaching || sc->sc_resetting ||
	    sc->sc_suspended || !(sc->sc_ic.ic_if.if_flags & IFF_RUNNING))
		return;
	struct mwx_queue *queues[] = {
		&sc->sc_rxq, &sc->sc_rxmcuq, &sc->sc_rxfwdlq
	};
	for (u_int i = 0; i < nitems(queues); i++) {
		struct mwx_queue *q = queues[i];
		if (q->mq_refill_pending && q->mq_count > 1 &&
		    q->mq_data != NULL && q->mq_desc != NULL && q->mq_map != NULL)
			mwx_queue_fill(sc, q);
	}
}

int
mwx_dma_alloc(struct mwx_softc *sc)
{
	int rv;

	MWX_DEV_LOG(sc, "dma alloc start\n");
	{
		int on = 0;
		sc->sc_rx_pool_disabled =
		    PE_parse_boot_argn("-mtknopool", &on, sizeof(on));
	}
	if (!sc->sc_rx_pool_disabled) {
		rv = mwx_rx_pool_alloc(sc);
		if (rv != 0)
			MWX_DEV_LOG(sc, "rx pool alloc failed rv=%d, using bounce\n", rv);
		else
			MWX_DEV_LOG(sc, "rx pool ready pages=%u mask=4G\n",
			    sc->sc_rx_pool_total);
	}
	/* TX queues */
	if ((rv = mwx_queue_alloc(sc, &sc->sc_txq, 2048,
	    MT_TX_DATA_RING_BASE)) != 0)
		return rv;
	if ((rv = mwx_queue_alloc(sc, &sc->sc_txmcuq, 256,
	    MT_TX_MCU_RING_BASE)) != 0)
		return rv;
	if ((rv = mwx_queue_alloc(sc, &sc->sc_txfwdlq, 16 /* XXX */,
	    MT_TX_FWDL_RING_BASE)) != 0)
		return rv;

	/* RX queues */
	if ((rv = mwx_queue_alloc(sc, &sc->sc_rxq, 1536,
	    MT_RX_DATA_RING_BASE)) != 0 ||
	    (rv = mwx_queue_fill(sc, &sc->sc_rxq)) != 0)
		return rv;
	if ((rv = mwx_queue_alloc(sc, &sc->sc_rxmcuq, 256,
	    MT_RX_MCU_RING_BASE)) != 0 ||
	    (rv = mwx_queue_fill(sc, &sc->sc_rxmcuq)) != 0)
		return rv;
	if ((rv = mwx_queue_alloc(sc, &sc->sc_rxfwdlq, 16 /* XXX */,
	    MT_RX_FWDL_RING_BASE)) != 0 ||
	    (rv = mwx_queue_fill(sc, &sc->sc_rxfwdlq)) != 0)
		return rv;

	/* enable DMA engine */
	mwx_dma_enable(sc);

	MWX_DEV_LOG(sc, "dma alloc complete\n");
	return 0;
}

int
mwx_dma_reset(struct mwx_softc *sc, int fullreset)
{
	int rv;

	DPRINTF("%s: DMA reset\n", DEVNAME(sc));

	if ((rv = mwx_dma_disable(sc, fullreset)) != 0)
		return rv;
	if (fullreset)
		if ((rv = mwx_wfsys_reset(sc)) != 0)
			return rv;

	/* TX queues */
	mwx_queue_reset(sc, &sc->sc_txq);
	mwx_queue_reset(sc, &sc->sc_txmcuq);
	mwx_queue_reset(sc, &sc->sc_txfwdlq);

	/* RX queues */
	mwx_queue_reset(sc, &sc->sc_rxq);
	mwx_queue_reset(sc, &sc->sc_rxmcuq);
	mwx_queue_reset(sc, &sc->sc_rxfwdlq);

	/* TDOD mt76_tx_status_check */

	/* refill RX queues */
	if ((rv = mwx_queue_fill(sc, &sc->sc_rxq)) != 0 ||
	    (rv = mwx_queue_fill(sc, &sc->sc_rxmcuq)) != 0 ||
	    (rv = mwx_queue_fill(sc, &sc->sc_rxfwdlq)) != 0)
		return rv;

	/* enable DMA engine */
	mwx_dma_enable(sc);

	return 0;
}

void
mwx_dma_free(struct mwx_softc *sc)
{
	mwx_rx_amsdu_purge(sc);
	mwx_rx_pool_free(sc);
	/* TX queues */
	mwx_queue_free(sc, &sc->sc_txq);
	mwx_queue_free(sc, &sc->sc_txmcuq);
	mwx_queue_free(sc, &sc->sc_txfwdlq);

	/* RX queues */
	mwx_queue_free(sc, &sc->sc_rxq);
	mwx_queue_free(sc, &sc->sc_rxmcuq);
	mwx_queue_free(sc, &sc->sc_rxfwdlq);
}

static inline int
mwx_dma_free_slots(struct mwx_queue *q)
{
	int free = q->mq_count - 1;
	free += q->mq_cons;
	free -= q->mq_prod;
	free %= q->mq_count;
	return free;
}

int
mwx_dma_tx_enqueue(struct mwx_softc *sc, struct mwx_queue *q, struct mbuf *m)
{
	struct mwx_queue_data *md;
	struct mwx_desc *desc;
	int i, nsegs, idx, rv;

	idx = q->mq_prod;
	md = &q->mq_data[idx];
	desc = &q->mq_desc[idx];

	rv = bus_dmamap_load_mbuf(sc->sc_dmat, md->md_map, m,
	    BUS_DMA_WRITE | BUS_DMA_NOWAIT);
	if (rv == EFBIG && m_defrag(m, M_DONTWAIT) == 0)
		rv = bus_dmamap_load_mbuf(sc->sc_dmat, md->md_map, m,
		    BUS_DMA_WRITE | BUS_DMA_NOWAIT);
	if (rv != 0)
		return rv;

	nsegs = md->md_map->dm_nsegs;

	/* check if there is enough space */
	if ((nsegs + 1)/2 > mwx_dma_free_slots(q)) {
		bus_dmamap_unload(sc->sc_dmat, md->md_map);
		return EBUSY;
	}

	bus_dmamap_sync(sc->sc_dmat, md->md_map, 0, md->md_map->dm_mapsize,
	    BUS_DMASYNC_PREWRITE);
	md->md_mbuf = m;
	md->md_txwi = NULL;

	bus_dmamap_sync(sc->sc_dmat, q->mq_map, 0, q->mq_map->dm_mapsize,
	    BUS_DMASYNC_PREWRITE);
	for (i = 0; i < nsegs; i += 2) {
		uint32_t	buf0, buf1 = 0;
		uint32_t	len0, len1 = 0, ctrl;

		KASSERT(md->md_map->dm_segs[i].ds_addr <= UINT32_MAX);
		buf0 = md->md_map->dm_segs[i].ds_addr;
		len0 = md->md_map->dm_segs[i].ds_len;
		ctrl = MT_DMA_CTL_SD_LEN0(len0);

		if (i < nsegs - 1) {
			KASSERT(md->md_map->dm_segs[i + 1].ds_addr <=
			    UINT32_MAX);
			buf1 = md->md_map->dm_segs[i + 1].ds_addr;
			len1 = md->md_map->dm_segs[i + 1].ds_len;
			ctrl |= MT_DMA_CTL_SD_LEN1(len1);
		}

		if (i == nsegs - 1)
			ctrl |= MT_DMA_CTL_LAST_SEC0;
		else if (i == nsegs - 2)
			ctrl |= MT_DMA_CTL_LAST_SEC1;

		desc->buf0 = htole32(buf0);
		desc->buf1 = htole32(buf1);
		desc->info = 0;
		desc->ctrl = htole32(ctrl);

		idx = (idx + 1) % q->mq_count;
		KASSERT(idx != q->mq_cons);
		md = &q->mq_data[idx];
		desc = &q->mq_desc[idx];
	}
	bus_dmamap_sync(sc->sc_dmat, q->mq_map, 0, q->mq_map->dm_mapsize,
	    BUS_DMASYNC_POSTWRITE);

	q->mq_prod = idx;

	mwx_write(sc, q->mq_regbase + MT_DMA_CPU_IDX, q->mq_prod);

	return 0;
}

int
mwx_dma_txwi_enqueue(struct mwx_softc *sc, struct mwx_queue *q,
    struct mwx_txwi *mt)
{
	struct mwx_queue_data *md;
	struct mwx_desc *desc;
	uint32_t buf0, len0, ctrl;
	int idx;

	idx = q->mq_prod;
	md = &q->mq_data[idx];
	desc = &q->mq_desc[idx];

	/* check if there is enough space */
	if (1 > mwx_dma_free_slots(q)) {
		bus_dmamap_unload(sc->sc_dmat, md->md_map);
		return EBUSY;
	}

	md->md_txwi = mt;
	md->md_mbuf = NULL;

	bus_dmamap_sync(sc->sc_dmat, q->mq_map, 0, q->mq_map->dm_mapsize,
	    BUS_DMASYNC_PREWRITE);

	buf0 = mt->mt_addr;
	len0 = sizeof(*mt->mt_desc);
	ctrl = MT_DMA_CTL_SD_LEN0(len0);
	ctrl |= MT_DMA_CTL_LAST_SEC0;

	static uint32_t txwi_dma_log_count;
	if (txwi_dma_log_count < 16) {
		MWX_DEV_LOG(sc, "txwi dma idx=%d addr=0x%08x len=%u\n",
		    idx, buf0, len0);
		txwi_dma_log_count++;
	}

	desc->buf0 = htole32(buf0);
	desc->buf1 = 0;
	desc->info = 0;
	desc->ctrl = htole32(ctrl);

	idx = (idx + 1) % q->mq_count;
	KASSERT(idx != q->mq_cons);

	bus_dmamap_sync(sc->sc_dmat, q->mq_map, 0, q->mq_map->dm_mapsize,
	    BUS_DMASYNC_POSTWRITE);

	q->mq_prod = idx;

	mwx_write(sc, q->mq_regbase + MT_DMA_CPU_IDX, q->mq_prod);

	return 0;
}

void
mwx_dma_tx_cleanup(struct mwx_softc *sc, struct mwx_queue *q)
{
	struct mwx_queue_data *md;
	struct mwx_desc *desc;
	int idx, last;

	idx = q->mq_cons;
	last = mwx_read(sc, q->mq_regbase + MT_DMA_DMA_IDX);

	if (idx == last)
		return;

	bus_dmamap_sync(sc->sc_dmat, q->mq_map, 0, q->mq_map->dm_mapsize,
	     BUS_DMASYNC_PREWRITE);

	while (idx != last) {
		md = &q->mq_data[idx];
		desc = &q->mq_desc[idx];

		if (md->md_mbuf != NULL) {
			bus_dmamap_sync(sc->sc_dmat, md->md_map, 0,
			    md->md_map->dm_mapsize, BUS_DMASYNC_POSTWRITE);
			bus_dmamap_unload(sc->sc_dmat, md->md_map);
			m_freem(md->md_mbuf);
			md->md_mbuf = NULL;
		}
		if (md->md_txwi != NULL) {
			static uint32_t txwi_ack_log_count;

			/* TXWI cleanup is driven by the firmware TX free event. */
			md->md_txwi = NULL;
			if (txwi_ack_log_count < 32) {
				MWX_DEV_LOG(sc, "txwi acked idx=%d\n", idx);
				txwi_ack_log_count++;
			}
		}

		/* clear DMA desc just to be sure */
		desc->buf0 = 0;
		desc->buf1 = 0;
		desc->info = 0;
		desc->ctrl = htole32(MT_DMA_CTL_DMA_DONE);

		idx = (idx + 1) % q->mq_count;

		/* check if more data made it in */
		/* XXX should we actually do that? */
		if (idx == last)
			last = mwx_read(sc, q->mq_regbase + MT_DMA_DMA_IDX);
	}

	bus_dmamap_sync(sc->sc_dmat, q->mq_map, 0, q->mq_map->dm_mapsize,
	    BUS_DMASYNC_POSTWRITE);

	q->mq_cons = idx;
	if (q->mq_wakeme) {
		q->mq_wakeme = 0;
		wakeup(q);
	}
}

void
mwx_dma_tx_done(struct mwx_softc *sc)
{
	mwx_dma_tx_cleanup(sc, &sc->sc_txq);
	mwx_dma_tx_cleanup(sc, &sc->sc_txmcuq);
	mwx_dma_tx_cleanup(sc, &sc->sc_txfwdlq);

	/* XXX do we need to wakeup someone */
}

/* XXX wrong place */
void
mwx_dma_rx_process(struct mwx_softc *sc, struct mbuf_list *ml,
    struct mbuf_list *mlout, uint8_t rxqueue)
{
	struct mbuf *m;
	uint32_t *data, rxd, type, flag;
	uint32_t event_count = 0, notify_count = 0, normal_mcu_count = 0;
	uint32_t normal_count = 0, unknown_count = 0;
	uint32_t first_unknown_type = 0, first_unknown_flag = 0, first_unknown_rxd = 0;

	while ((m = ml_dequeue(ml)) != NULL) {
		data = mtod(m, uint32_t *);
		rxd = le32toh(data[0]);

		type = MT_RXD0_PKT_TYPE_GET(rxd);
		flag = (rxd & MT_RXD0_PKT_FLAG_MASK) >> MT_RXD0_PKT_FLAG_SHIFT;

		if (type == PKT_TYPE_RX_EVENT && flag == 0x1)
			type = PKT_TYPE_NORMAL_MCU;

		switch (type) {
		case PKT_TYPE_RX_EVENT:
			event_count++;
			mwx_mcu_rx_event(sc, m);
			break;
		case PKT_TYPE_TXRX_NOTIFY:
			notify_count++;
			mwx_mac_tx_free(sc, m);
			break;
		case PKT_TYPE_TXS:
#if TODO
			for (rxd += 2; rxd + 8 <= end; rxd += 8)
				mt7921_mac_add_txs(dev, rxd);
#endif
			m_freem(m);
			break;
		case PKT_TYPE_NORMAL_MCU:
			normal_mcu_count++;
			mwx_rx(sc, m, mlout, rxqueue);
			break;
		case PKT_TYPE_NORMAL:
			normal_count++;
			mwx_rx(sc, m, mlout, rxqueue);
			break;
		default:
			if (unknown_count == 0) {
				first_unknown_type = type;
				first_unknown_flag = flag;
				first_unknown_rxd = rxd;
			}
			unknown_count++;
			m_freem(m);
			break;
		}
	}

	if (event_count || notify_count || normal_mcu_count ||
	    normal_count || unknown_count) {
		sc->sc_rx_pkt_event += event_count;
		sc->sc_rx_pkt_txrx_notify += notify_count;
		sc->sc_rx_pkt_normal_mcu += normal_mcu_count;
		sc->sc_rx_pkt_normal += normal_count;
		sc->sc_rx_pkt_unknown += unknown_count;
		if (sc->sc_rx_process_events < 64 || unknown_count != 0) {
			MWX_DEV_LOG(sc,
			    "rx process event=%u notify=%u normal_mcu=%u normal=%u unknown=%u totals e=%u n=%u nmcu=%u norm=%u unk=%u\n",
			    event_count, notify_count, normal_mcu_count,
			    normal_count, unknown_count, sc->sc_rx_pkt_event,
			    sc->sc_rx_pkt_txrx_notify,
			    sc->sc_rx_pkt_normal_mcu, sc->sc_rx_pkt_normal,
			    sc->sc_rx_pkt_unknown);
			if (unknown_count != 0) {
				MWX_DEV_LOG(sc,
				    "rx unknown first type=%u flag=0x%x rxd0=0x%08x\n",
				    first_unknown_type, first_unknown_flag,
				    first_unknown_rxd);
			}
		}
		sc->sc_rx_process_events++;
	}
}

bool
mwx_dma_rx_dequeue(struct mwx_softc *sc, struct mwx_queue *q,
    struct mbuf_list *ml)
{
	static const u_int quota = 64;
	struct mwx_queue_data *md;
	struct mwx_desc *desc;
	struct mbuf *m;
	int idx, last;
	u_int consumed = 0;
	bool waiting = false;

	sc->sc_rx_dma_batches++;
	idx = q->mq_cons;
	last = mwx_read(sc, q->mq_regbase + MT_DMA_DMA_IDX);
	if (idx == last || last < 0 || last >= q->mq_count)
		return false;
	if (q->mq_rx_head != NULL)
		sc->sc_rx_dma_chain_resumed++;

	bus_dmamap_sync(sc->sc_dmat, q->mq_map, 0, q->mq_map->dm_mapsize,
	    BUS_DMASYNC_PREREAD);

	while (idx != last) {
		if (consumed >= quota)
			break;
		md = &q->mq_data[idx];
		desc = &q->mq_desc[idx];
		m = md->md_mbuf;
		if (m == NULL) {
			static uint32_t rx_empty_log;
			if (rx_empty_log++ < 8)
				MWX_DEV_LOG(sc, "rx slot %d without buffer (dma idx %d)\n",
				    idx, last);
			if (q->mq_rx_head != NULL)
				sc->sc_rx_dma_chain_dropped++;
			m_freem(q->mq_rx_head);
			q->mq_rx_head = q->mq_rx_tail = NULL;
			q->mq_rx_bytes = q->mq_rx_segments = 0;
			q->mq_rx_discard = true;
			mwx_reset(sc);
			break;
		}

		/*
		 * A slot is the hardware's until DMA_DONE shows in it, as
		 * mt76_dma_dequeue() checks; the DMA index can run ahead of the
		 * descriptor's write-back. Read before that, the slot still holds
		 * the refill's full length and LAST_SEC0: a whole buffer of stale
		 * bytes taken for a frame, the next segment of a long frame taken
		 * for a new packet with a header of payload bytes, and the slot
		 * armed again while the hardware still completes it. Such a slot
		 * is waited for through an interrupt rescheduled until DMA_DONE
		 * shows; one that never shows it is taken as before after
		 * MWX_RX_DONE_WAIT_NS, and counted.
		 */
		uint32_t ctrl = le32toh(desc->ctrl);
		if (!(ctrl & MT_DMA_CTL_DMA_DONE)) {
			uint64_t now_ns = 0;

			absolutetime_to_nanoseconds(mach_absolute_time(), &now_ns);
			if (q->mq_rx_wait_ns == 0 || q->mq_rx_wait_idx != (u_int)idx) {
				q->mq_rx_wait_idx = idx;
				q->mq_rx_wait_ns = now_ns;
				sc->sc_rx_dma_notdone++;
			}
			if (now_ns - q->mq_rx_wait_ns < MWX_RX_DONE_WAIT_NS) {
				waiting = true;
				break;
			}
			sc->sc_rx_dma_done_missing++;
		}
		q->mq_rx_wait_ns = 0;

		/* Validate descriptor lengths and aggregate bounds before copying
		 * payload from a bounce buffer or changing an mbuf's bounds. */
		const size_t len = MT_DNA_CTL_SD_GET_LEN0(ctrl);
		const bool last_segment = (ctrl & MT_DMA_CTL_LAST_SEC0) != 0;
		bool valid = len != 0 && len <= md->md_map->dm_mapsize &&
		    len <= mbuf_len(m) &&
		    (q->mq_rx_head != NULL || q->mq_rx_discard ||
		    len >= sizeof(uint32_t));
		if (valid && !q->mq_rx_discard &&
		    (q->mq_rx_segments >= MWX_RX_CHAIN_MAX_SEGS ||
		    len > MWX_RX_CHAIN_MAX_BYTES - q->mq_rx_bytes))
			valid = false;
		if (!valid) {
			sc->sc_rx_dma_bad_len++;
			if (!q->mq_rx_discard)
				sc->sc_rx_dma_chain_dropped++;
			q->mq_rx_discard = true;
		}
		if (valid)
			sc->sc_rx_dma_bytes += len;
		if (!q->mq_rx_discard) {
			if (md->md_map->dm_bounce) {
				sc->sc_rx_dma_copy_bytes += len;
				sc->sc_rx_dma_copy_saved += md->md_map->dm_mapsize - len;
			}
			bus_dmamap_sync(sc->sc_dmat, md->md_map, 0, len,
			    BUS_DMASYNC_POSTREAD);
			/* The normal decoder reads its base RXD directly from the
			 * first mbuf. Other packet types only need their first word
			 * here and perform their own pullup before deeper parsing. */
			if (q->mq_rx_head == NULL) {
				const uint32_t rxd0 = le32toh(*mtod(m, uint32_t *));
				const uint32_t type = MT_RXD0_PKT_TYPE_GET(rxd0);
				if ((type == PKT_TYPE_NORMAL ||
				    type == PKT_TYPE_NORMAL_MCU) &&
				    len < 6U * sizeof(uint32_t)) {
					sc->sc_rx_dma_bad_len++;
					sc->sc_rx_dma_chain_dropped++;
					q->mq_rx_discard = true;
				}
			}
		}
		bus_dmamap_unload(sc->sc_dmat, md->md_map);
		md->md_mbuf = NULL;

		/* Continuations must not carry an independent packet header. */
		if (!q->mq_rx_discard && q->mq_rx_head != NULL &&
		    mbuf_setflags_mask(m, 0, MBUF_PKTHDR) != 0) {
			sc->sc_rx_dma_chain_dropped++;
			q->mq_rx_discard = true;
		}
		if (q->mq_rx_discard) {
			m_freem(m);
			m_freem(q->mq_rx_head);
			q->mq_rx_head = q->mq_rx_tail = NULL;
			q->mq_rx_bytes = q->mq_rx_segments = 0;
			if (last_segment)
				q->mq_rx_discard = false;
		} else {
			mbuf_setlen(m, len);
			if (q->mq_rx_head == NULL)
				q->mq_rx_head = m;
			else
				mbuf_setnext(q->mq_rx_tail, m);
			q->mq_rx_tail = m;
			q->mq_rx_bytes += len;
			q->mq_rx_segments++;
			mbuf_pkthdr_setlen(q->mq_rx_head, q->mq_rx_bytes);
			if (last_segment) {
				if (q->mq_rx_segments > 1)
					sc->sc_rx_dma_chain_completed++;
				struct mbuf *complete = q->mq_rx_head;
				q->mq_rx_head = q->mq_rx_tail = NULL;
				q->mq_rx_bytes = q->mq_rx_segments = 0;
				ml_enqueue(ml, complete);
			}
		}

		consumed++;
		idx = (idx + 1) % q->mq_count;
		if (idx == last) {
			last = mwx_read(sc, q->mq_regbase + MT_DMA_DMA_IDX);
			if (last < 0 || last >= q->mq_count)
				last = idx;
		}
	}

	/* Keep a bounded, detached prefix until the next batch supplies LAST.
	 * Reset/free release it; malformed chains stay in discard mode through
	 * LAST so their continuation cannot be mistaken for a new RXD. */
	bus_dmamap_sync(sc->sc_dmat, q->mq_map, 0, q->mq_map->dm_mapsize,
	    BUS_DMASYNC_POSTREAD);
	q->mq_cons = idx;
	if (consumed != 0 && q == &sc->sc_rxq) {
		uint64_t ns = 0;

		absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
		sc->sc_rx_data_last_us = ns / 1000;
	}
	sc->sc_rx_dma_desc += consumed;
	if (consumed > sc->sc_rx_dma_max_batch)
		sc->sc_rx_dma_max_batch = consumed;
	return idx != last || waiting;
}

bool
mwx_dma_rx_done(struct mwx_softc *sc, struct mwx_queue *q,
    struct mbuf_list *mlout)
{
	struct mbuf_list ml = MBUF_LIST_INITIALIZER();
	const u_int old_cons = q->mq_cons;
	bool more;

	more = mwx_dma_rx_dequeue(sc, q, &ml);
	/* Invalid or incomplete frames still consume DMA slots. */
	if (q->mq_cons != old_cons || q->mq_refill_pending)
		mwx_queue_fill(sc, q);
	if (!ml_empty(&ml))
		mwx_dma_rx_process(sc, &ml, mlout,
		    q == &sc->sc_rxq ? 0 : q == &sc->sc_rxmcuq ? 1 : 2);
	return more;
}

struct mbuf *
mwx_mcu_alloc_msg(size_t len)
{
	const int headspace = sizeof(struct mwx_mcu_txd);
	struct mbuf *m;

	/* Allocate mbuf with enough space */
	m = MCLGETL(NULL, M_DONTWAIT, len + headspace);
	if (m == NULL)
		return NULL;
	mbuf_setlen(m, len + headspace);
	mbuf_pkthdr_setlen(m, len + headspace);
	m_adj(m, headspace);

	return m;
}

void
mwx_mcu_set_len(struct mbuf *m, void *end)
{
	uint8_t *start = mtod(m, uint8_t *);
	uint8_t *finish = (uint8_t *)end;
	int len = mbuf_pkthdr_len(m), adj;

	KASSERT(start <= finish && finish - start <= len);
	adj = len - (finish - start);
	m_adj(m, -adj);
}

/*
 * The firmware commands sent, and the remain-on-channel events received, with their time, for the log of a reception
 * stall: the step that stops the data RX ring is not known. 1024 entries reach back past the 30 s a stall takes to be
 * noticed, at the dozen commands a second of AWDL's search windows. Each keeps bytes 4-13 of the payload, which tell
 * the commands apart (mwx_mcu_trace_name). Any thread; a torn entry only spoils one line of the log.
 */
#define MWX_MCU_TRACE_ROC_EVENT	0xffff0001	/* not a command: an MCU_UNI_EVENT_ROC */
static struct {
	uint64_t	us;
	uint32_t	cmd;
	uint8_t		b[10];
} mwx_mcu_trace[1024];
static uint32_t mwx_mcu_trace_next;

static void
mwx_mcu_trace_add(uint32_t cmd, const uint8_t *b, size_t len)
{
	uint64_t ns = 0;
	uint32_t i = mwx_mcu_trace_next++ % nitems(mwx_mcu_trace);

	absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
	mwx_mcu_trace[i].us = ns / 1000;
	mwx_mcu_trace[i].cmd = cmd;
	memset(mwx_mcu_trace[i].b, 0, sizeof(mwx_mcu_trace[i].b));
	memcpy(mwx_mcu_trace[i].b, b, len < sizeof(mwx_mcu_trace[i].b) ? len : sizeof(mwx_mcu_trace[i].b));
}

/*
 * One entry as text. b holds bytes 4-13 of the payload, laid out as in the requests of mwx_awdl_roc(),
 * mwx_awdl_roc_abort(), mwx_awdl_sniffer(), mwx_awdl_sniffer_config() and mwx_awdl_rxfilter(); for a remain-on-channel
 * event, its tag, token, status and channel.
 */
static void
mwx_mcu_trace_name(char *s, size_t n, uint32_t cmd, const uint8_t *b)
{
	switch (cmd) {
	case MWX_MCU_TRACE_ROC_EVENT:
		if (b[0] == 0)				/* UNI_EVENT_ROC_GRANT */
			snprintf(s, n, "grant:t%u:s%u:%u", b[1], b[2], b[3]);
		else
			snprintf(s, n, "rocevent%u:t%u", b[0], b[1]);
		break;
	case MCU_UNI_CMD_ROC:				/* tag 0 acquire, with token and channel; 1 abort */
		if (b[0] == 0)
			snprintf(s, n, "roc:t%u:%u", b[5], b[6]);
		else
			snprintf(s, n, "abort:t%u", b[5]);
		break;
	case MCU_UNI_CMD_SNIFFER:			/* tag 0 enable, with its flag; 1 config, control channel */
		if (b[0] == 0)
			snprintf(s, n, "sniffer:%u", b[4]);
		else
			snprintf(s, n, "sniffer@%u", b[8]);
		break;
	case MCU_CE_CMD_SET_RX_FILTER:			/* mode, then the filter flags */
		snprintf(s, n, "filter:%u:%x", b[0], (uint32_t)b[4] | (uint32_t)b[5] << 8 | (uint32_t)b[6] << 16 |
		    (uint32_t)b[7] << 24);
		break;
	default:
		snprintf(s, n, "%x", cmd);
		break;
	}
}

/*
 * For a reception stall: when the last frame was taken from the data RX ring, then the commands and remain-on-channel
 * events from 2 s before it to 1 s after, oldest first, timed from it in ms, eight to a line.
 */
static void
mwx_mcu_trace_log(struct mwx_softc *sc, uint64_t now_us)
{
	uint64_t last = sc->sc_rx_data_last_us;
	uint32_t next = mwx_mcu_trace_next, i, shown = 0, k = 0;
	uint32_t kept = next < nitems(mwx_mcu_trace) ? next : (uint32_t)nitems(mwx_mcu_trace);
	char line[8 * 32 + 1], name[24];

	MWX_DEV_LOG(sc, "last data frame at %llu.%03llu s, %llu ms before; commands and remain-on-channel events around it "
	    "(ms from it):\n", (unsigned long long)(last / 1000000), (unsigned long long)(last / 1000 % 1000),
	    (unsigned long long)((now_us - last) / 1000));
	line[0] = 0;
	for (i = next - kept; i != next; i++) {
		uint32_t j = i % nitems(mwx_mcu_trace);
		int64_t d = (int64_t)(mwx_mcu_trace[j].us - last);

		if (d < -2000000 || d > 1000000)
			continue;
		mwx_mcu_trace_name(name, sizeof(name), mwx_mcu_trace[j].cmd, mwx_mcu_trace[j].b);
		k += (uint32_t)snprintf(line + k, sizeof(line) - k, " %s@%s%llu", name, d < 0 ? "-" : "+",
		    (unsigned long long)((d < 0 ? -d : d) / 1000));
		if (k >= sizeof(line))
			k = sizeof(line) - 1;
		if (++shown % 8 == 0) {
			MWX_DEV_LOG(sc, " %s\n", line);
			line[0] = 0;
			k = 0;
		}
	}
	if (k != 0)
		MWX_DEV_LOG(sc, " %s\n", line);
	if (shown == 0)
		MWX_DEV_LOG(sc, " none of the last %u is that close\n", kept);
}

int
mwx_mcu_send_mbuf(struct mwx_softc *sc, uint32_t cmd, struct mbuf *m, int *seqp)
{
	struct mwx_uni_txd *uni_txd;
	struct mwx_mcu_txd *mcu_txd;
	struct mwx_queue *q;
	uint32_t *txd, val;
	int s, rv, txd_len = 0, mcu_cmd = cmd & MCU_CMD_FIELD_ID_MASK;
	int tot_len, len = mbuf_pkthdr_len(m);
	size_t lead;
	uint8_t seq = 0;

	/*
	 * The firmware left a command of this stop unanswered. Every further
	 * one would wait its three seconds too, and the stop runs inside the
	 * power-off the kernel times: three of them panicked a system sleep.
	 */
	if (sc->sc_mcu_mute) {
		m_freem(m);
		return ETIMEDOUT;
	}
	{
		uint8_t b[10] = { 0 };

		if (len > 4)
			mbuf_copydata(m, 4, (size_t)len - 4 < sizeof(b) ? (size_t)len - 4 : sizeof(b), b);
		mwx_mcu_trace_add(cmd, b, sizeof(b));
	}

	if (cmd == MCU_CMD_FW_SCATTER) {
		q = &sc->sc_txfwdlq;
		goto enqueue;
	}

	seq = ++sc->sc_mcu_seq & 0x0f;
	if (seq == 0)
		seq = ++sc->sc_mcu_seq & 0x0f;

	KASSERT(seq < nitems(sc->sc_mcu_wait));
	txd_len = cmd & MCU_CMD_FIELD_UNI ? sizeof(*uni_txd) : sizeof(*mcu_txd);
	tot_len = txd_len + len;
	lead = m_leadingspace(m);
	if (lead < (size_t)txd_len) {
		static uint32_t rebuild_log_count;
		struct mbuf *n;

		if (rebuild_log_count < 32) {
			MWX_DEV_LOG(sc,
			    "MCU tx mbuf rebuild cmd=0x%08x len=%d lead=%zu txd_len=%d\n",
			    cmd, len, lead, txd_len);
			rebuild_log_count++;
		}
		n = MCLGETL(NULL, M_DONTWAIT, len + txd_len);
		if (n == NULL) {
			m_freem(m);
			return ENOMEM;
		}
		mbuf_setlen(n, len + txd_len);
		mbuf_pkthdr_setlen(n, len + txd_len);
		if (len != 0 &&
		    mbuf_copydata(m, 0, len, mtod(n, uint8_t *) + txd_len) != 0) {
			m_freem(n);
			m_freem(m);
			return EINVAL;
		}
		m_freem(m);
		m = n;
	} else {
		m = m_prepend(m, txd_len, M_DONTWAIT);
		if (m == NULL)
			return ENOMEM;
	}
	txd = mtod(m, uint32_t *);
	memset(txd, 0, txd_len);

	val = (tot_len & MT_TXD0_TX_BYTES_MASK) |
	    MT_TX_TYPE_CMD | MT_TXD0_Q_IDX(MT_TX_MCU_PORT_RX_Q0);
	txd[0] = htole32(val);

	val = MT_TXD1_LONG_FORMAT | MT_HDR_FORMAT_CMD;
	txd[1] = htole32(val);

	if (cmd & MCU_CMD_FIELD_UNI) {
		uni_txd = (struct mwx_uni_txd *)txd;
		uni_txd->len = htole16(tot_len - sizeof(uni_txd->txd));
		uni_txd->option = MCU_CMD_UNI_EXT_ACK;
		uni_txd->cid = htole16(mcu_cmd);
		uni_txd->s2d_index = CMD_S2D_IDX_H2N;
		uni_txd->pkt_type = MCU_PKT_ID;
		uni_txd->seq = seq;
	} else {
		mcu_txd = (struct mwx_mcu_txd *)txd;
		mcu_txd->len = htole16(tot_len - sizeof(uni_txd->txd));
		mcu_txd->pq_id = htole16(MCU_PQ_ID(MT_TX_PORT_IDX_MCU,
			MT_TX_MCU_PORT_RX_Q0));
		mcu_txd->pkt_type = MCU_PKT_ID;
		mcu_txd->seq = seq;
		mcu_txd->cid = mcu_cmd;
		mcu_txd->s2d_index = CMD_S2D_IDX_H2N;
		mcu_txd->ext_cid = MCU_GET_EXT_CMD(cmd);

		if (mcu_txd->ext_cid || (cmd & MCU_CMD_FIELD_CE)) {
			if (cmd & MCU_CMD_FIELD_QUERY)
				mcu_txd->set_query = MCU_Q_QUERY;
			else
				mcu_txd->set_query = MCU_Q_SET;
			mcu_txd->ext_cid_ack = !!mcu_txd->ext_cid;
		} else {
			mcu_txd->set_query = MCU_Q_NA;
		}
	}

	if (seqp != NULL) {
		memset(&sc->sc_mcu_wait[seq], 0, sizeof(sc->sc_mcu_wait[0]));
		sc->sc_mcu_wait[seq].mcu_cmd = cmd;
		*seqp = seq;
	}
	q = &sc->sc_txmcuq;
enqueue:

	if (cmd != MCU_CMD_FW_SCATTER) {
		static uint32_t mcu_tx_log_count;

		if (mcu_tx_log_count < 96) {
			MWX_DEV_LOG(sc,
			    "mcu tx cmd=0x%08x seq=%u len=%d txd_len=%d q_reg=0x%08x\n",
			    cmd, seq, len, txd_len, q->mq_regbase);
			mcu_tx_log_count++;
		}
#ifdef MWX_PACKET_HEXDUMP
		pkt_hex_dump(m);
#endif
	}

	s = splnet();
	while (1) {
		rv = mwx_dma_tx_enqueue(sc, q, m);
		if (rv != EBUSY)
			break;
		q->mq_wakeme = 1;
		tsleep_nsec(q, 0, "mwxq", MSEC_TO_NSEC(100));
	}
	splx(s);
	if (rv != 0) {
		if (seqp != NULL)
			memset(&sc->sc_mcu_wait[seq], 0,
			    sizeof(sc->sc_mcu_wait[0]));
		m_freem(m);
	}
	return rv;
}

int
mwx_mcu_send_msg(struct mwx_softc *sc, uint32_t cmd, void *data, size_t len,
    int *seqp)
{
	struct mbuf *m;

	m = mwx_mcu_alloc_msg(len);
	if (m == NULL) {
		MWX_LOG("mcu msg alloc failed cmd=0x%08x len=%zu\n", cmd, len);
		return ENOMEM;
	}

	if (len != 0)
		memcpy(mtod(m, caddr_t), data, len);

	return mwx_mcu_send_mbuf(sc, cmd, m, seqp);
}

int
mwx_mcu_send_wait(struct mwx_softc *sc, uint32_t cmd, void *data, size_t len)
{
	int rv, seq;

	rv = mwx_mcu_send_msg(sc, cmd, data, len, &seq);
	if (rv != 0)
		return rv;
	return mwx_mcu_wait_resp_int(sc, cmd, seq, NULL);
}

int
mwx_mcu_send_mbuf_wait(struct mwx_softc *sc, uint32_t cmd, struct mbuf *m)
{
	int rv, seq;

	rv = mwx_mcu_send_mbuf(sc, cmd, m, &seq);
	if (rv != 0)
		return rv;
	return mwx_mcu_wait_resp_int(sc, cmd, seq, NULL);
}

/* Inspect fire-and-forget STA/WTBL results without blocking the RX work loop.
 * This is diagnostic only: enqueue success still is not firmware success. */
static void
mwx_mcu_trace_sta_status(struct mwx_softc *sc, uint8_t eid, uint8_t seq,
    struct mbuf *m)
{
	if (eid != MCU_UNI_EVENT_RESULT ||
	    mbuf_len(m) < sizeof(struct mt7921_mcu_uni_event))
		return;
	const struct mt7921_mcu_uni_event *event =
	    mtod(m, struct mt7921_mcu_uni_event *);
	if (event->cid != (MCU_UNI_CMD_STA_REC_UPDATE & MCU_CMD_FIELD_ID_MASK))
		return;
	MWX_DEV_LOG(sc, "STA_REC firmware result seq=%u status=%u\n",
	    seq, le32toh(event->status));
}

void
mwx_mcu_rx_event(struct mwx_softc *sc, struct mbuf *m)
{
	struct mwx_mcu_rxd *rxd;
	uint32_t cmd, mcu_int = 0;
	uint8_t rxd_eid, rxd_ext_eid, rxd_seq, rxd_option;
	int len, rxd_size;

	rxd_size = MT7921_MCU_RXD_SIZE;
	if ((m = m_pullup(m, sizeof(*rxd) + rxd_size)) == NULL)
		return;
	m_adj(m, rxd_size);
	rxd = mtod(m, struct mwx_mcu_rxd *);

	if (rxd->ext_eid == MCU_EXT_EVENT_RATE_REPORT) {
		printf("%s: MCU_EXT_EVENT_RATE_REPORT COMMAND\n", DEVNAME(sc));
		m_freem(m);
		return;
	}

	len = le16toh(rxd->len);
	/* make sure all the data is in one mbuf; an event longer than what came is logged with its kind and sizes */
	{
		uint8_t eid = rxd->eid, ext_eid = rxd->ext_eid;
		int have = (int)mbuf_pkthdr_len(m);

		if ((m = m_pullup(m, len)) == NULL) {
			printf("%s: mwx_mcu_rx_event m_pullup failed: event %02x/%02x says %d bytes, %d came\n",
			    DEVNAME(sc), eid, ext_eid, len, have);
			return;
		}
	}
	/* refetch after pullup */
	rxd = mtod(m, struct mwx_mcu_rxd *);
	rxd_eid = rxd->eid;
	rxd_ext_eid = rxd->ext_eid;
	rxd_seq = rxd->seq;
	rxd_option = rxd->option;
	(void)rxd_option;
	m_adj(m, sizeof(*rxd));
	mwx_mcu_trace_sta_status(sc, rxd_eid, rxd_seq, m);

	switch (rxd_eid) {
	case MCU_EVENT_SCHED_SCAN_DONE:
	case MCU_EVENT_SCAN_DONE:
		mt7921_mcu_scan_event(sc, m);
		break;
#if 0
	case MCU_EVENT_BSS_BEACON_LOSS:
		mt7921_mcu_connection_loss_event(dev, skb);
		break;
	case MCU_EVENT_BSS_ABSENCE:
		mt7921_mcu_bss_event(dev, skb);
		break;
	case MCU_EVENT_DBG_MSG:
		mt7921_mcu_debug_msg_event(dev, skb);
		break;
#endif
	case MCU_EVENT_COREDUMP:
		/* it makes little sense to write the coredump down */
		if (!sc->sc_resetting)
			printf("%s: coredump event\n", DEVNAME(sc));
		mwx_reset(sc);
		break;
	case MCU_EVENT_LP_INFO:
		mt7921_mcu_low_power_event(sc, m);
		break;
	case MCU_EVENT_TX_DONE:
		mt7921_mcu_tx_done_event(sc, m);
		break;
	case 0x6:
		printf("%s: MAGIC COMMAND\n", DEVNAME(sc));
	default:
#ifdef MTK_AWDL
		/* The remain-on-channel grant, or the answer to one of the AWDL radio's own commands. */
		if (sc->sc_awdl != NULL &&
		    mwx_awdl_mcu_event(sc, rxd_eid, rxd_seq, rxd_option, m))
			break;
#endif
		if (rxd_seq == 0 || rxd_seq >= nitems(sc->sc_mcu_wait)) {
			printf("%s: mcu rx bad seq %x\n", DEVNAME(sc),
			    rxd_seq);
			break;
		}

		cmd = sc->sc_mcu_wait[rxd_seq].mcu_cmd;

		if (cmd == MCU_CMD_PATCH_SEM_CONTROL ||
		    cmd == MCU_CMD_PATCH_FINISH_REQ) {
			mcu_int = rxd_ext_eid;
			MWX_DEV_LOG(sc, "MCU patch response cmd=0x%08x seq=%u status=0x%x\n",
			    cmd, rxd_seq, mcu_int);
		} else if (cmd == MCU_EXT_CMD_THERMAL_CTRL) {
			if (mbuf_len(m) < sizeof(uint32_t) * 2)
				break;
			mcu_int = le32toh(mtod(m, uint32_t *)[1]);
		} else if (cmd == MCU_EXT_CMD_EFUSE_ACCESS) {
			//ret = mt7921_mcu_parse_eeprom(sc, m);
			printf("%s: mcu resp no handled yet\n", DEVNAME(sc));
		} else if (cmd == MCU_UNI_CMD_DEV_INFO_UPDATE ||
		    cmd == MCU_UNI_CMD_BSS_INFO_UPDATE ||
		    cmd == MCU_UNI_CMD_STA_REC_UPDATE ||
		    cmd == MCU_UNI_CMD_HIF_CTRL ||
		    cmd == MCU_UNI_CMD_OFFLOAD ||
		    cmd == MCU_UNI_CMD_SUSPEND) {
			struct mt7921_mcu_uni_event *event;

			if (mbuf_len(m) < sizeof(*event))
				break;
			event = mtod(m, struct mt7921_mcu_uni_event *);
			mcu_int = le32toh(event->status);
		} else if (cmd == MCU_CE_QUERY_REG_READ) {
			struct mt7921_mcu_reg_event *event;

			if (mbuf_len(m) < sizeof(*event))
				break;
			event = mtod(m, struct mt7921_mcu_reg_event *);
			mcu_int = le32toh(event->val);
		}

		sc->sc_mcu_wait[rxd_seq].mcu_int = mcu_int;
		sc->sc_mcu_wait[rxd_seq].mcu_m = m;
		wakeup(&sc->sc_mcu_wait[rxd_seq]);
		return;
	}

	m_freem(m);
}

int
mwx_mcu_wait_resp_int(struct mwx_softc *sc, uint32_t cmd, int seq,
    uint32_t *val)
{
	int rv;

	KASSERT(seq < nitems(sc->sc_mcu_wait));

	rv = tsleep_nsec(&sc->sc_mcu_wait[seq], 0, "mwxwait", SEC_TO_NSEC(3));
	if (rv != 0) {
		printf("%s: command %x timeout\n", DEVNAME(sc), cmd);
		if (sc->sc_stopping) {
			/* The card is going down: nothing to reset, and no more waits. */
			sc->sc_mcu_mute = 1;
			return rv;
		}
		mwx_reset(sc);
		return rv;
	}

	if (sc->sc_mcu_wait[seq].mcu_m != NULL) {
		m_freem(sc->sc_mcu_wait[seq].mcu_m);
		sc->sc_mcu_wait[seq].mcu_m = NULL;
	}
	if (val != NULL)
		*val = sc->sc_mcu_wait[seq].mcu_int;
	return 0;
}

int
mwx_mcu_wait_resp_msg(struct mwx_softc *sc, uint32_t cmd, int seq,
    struct mbuf **mp)
{
	int rv;

	KASSERT(seq < nitems(sc->sc_mcu_wait));

	rv = tsleep_nsec(&sc->sc_mcu_wait[seq], 0, "mwxwait", SEC_TO_NSEC(3));
	if (rv != 0) {
		printf("%s: command %x timeout\n", DEVNAME(sc), cmd);
		if (sc->sc_stopping) {
			/* The card is going down: nothing to reset, and no more waits. */
			sc->sc_mcu_mute = 1;
			return rv;
		}
		mwx_reset(sc);
		return rv;
	}
	if (sc->sc_mcu_wait[seq].mcu_m == NULL) {
		printf("%s: command response missing\n", DEVNAME(sc));
		return ENOENT;
	}
	if (mp != NULL)
		*mp = sc->sc_mcu_wait[seq].mcu_m;
	else
		m_freem(sc->sc_mcu_wait[seq].mcu_m);
	sc->sc_mcu_wait[seq].mcu_m = NULL;
	return 0;
}

int
mwx_dma_disable(struct mwx_softc *sc, int force)
{
	int rv;

	/* disable WFDMA0 */
	mwx_clear(sc, MT_WFDMA0_GLO_CFG,
	    MT_WFDMA0_GLO_CFG_TX_DMA_EN | MT_WFDMA0_GLO_CFG_RX_DMA_EN |
	    MT_WFDMA0_GLO_CFG_CSR_DISP_BASE_PTR_CHAIN_EN |
	    MT_WFDMA0_GLO_CFG_OMIT_TX_INFO |
	    MT_WFDMA0_GLO_CFG_OMIT_RX_INFO |
	    MT_WFDMA0_GLO_CFG_OMIT_RX_INFO_PFET2);

	rv = mwx_poll(sc, MT_WFDMA0_GLO_CFG, 0,
	    MT_WFDMA0_GLO_CFG_TX_DMA_BUSY | MT_WFDMA0_GLO_CFG_RX_DMA_BUSY, 100);
	if (!force && rv != 0)
		return rv;

	/* disable DMASHDL and set bypass */
	mwx_clear(sc, MT_WFDMA0_GLO_CFG_EXT0, MT_WFDMA0_CSR_TX_DMASHDL_ENABLE);
	mwx_set(sc, MT_DMASHDL_SW_CONTROL, MT_DMASHDL_DMASHDL_BYPASS);

	if (force) {
		/* reset */
		mwx_clear(sc, MT_WFDMA0_RST, MT_WFDMA0_RST_DMASHDL_ALL_RST |
		    MT_WFDMA0_RST_LOGIC_RST);
		mwx_set(sc, MT_WFDMA0_RST, MT_WFDMA0_RST_DMASHDL_ALL_RST |
		    MT_WFDMA0_RST_LOGIC_RST);
		delay(20);
	}

	return 0;
}

void
mwx_dma_enable(struct mwx_softc *sc)
{
#define PREFETCH(base, depth)   ((base) << 16 | (depth))
	/* configure perfetch settings */
	mwx_write(sc, MT_WFDMA0_RX_RING0_EXT_CTRL, PREFETCH(0x0, 0x4));
	mwx_write(sc, MT_WFDMA0_RX_RING2_EXT_CTRL, PREFETCH(0x40, 0x4));
	mwx_write(sc, MT_WFDMA0_RX_RING3_EXT_CTRL, PREFETCH(0x80, 0x4));
	mwx_write(sc, MT_WFDMA0_RX_RING4_EXT_CTRL, PREFETCH(0xc0, 0x4));
	mwx_write(sc, MT_WFDMA0_RX_RING5_EXT_CTRL, PREFETCH(0x100, 0x4));

	mwx_write(sc, MT_WFDMA0_TX_RING0_EXT_CTRL, PREFETCH(0x140, 0x4));
	mwx_write(sc, MT_WFDMA0_TX_RING1_EXT_CTRL, PREFETCH(0x180, 0x4));
	mwx_write(sc, MT_WFDMA0_TX_RING2_EXT_CTRL, PREFETCH(0x1c0, 0x4));
	mwx_write(sc, MT_WFDMA0_TX_RING3_EXT_CTRL, PREFETCH(0x200, 0x4));
	mwx_write(sc, MT_WFDMA0_TX_RING4_EXT_CTRL, PREFETCH(0x240, 0x4));
	mwx_write(sc, MT_WFDMA0_TX_RING5_EXT_CTRL, PREFETCH(0x280, 0x4));
	mwx_write(sc, MT_WFDMA0_TX_RING6_EXT_CTRL, PREFETCH(0x2c0, 0x4));
	mwx_write(sc, MT_WFDMA0_TX_RING16_EXT_CTRL, PREFETCH(0x340, 0x4));
	mwx_write(sc, MT_WFDMA0_TX_RING17_EXT_CTRL, PREFETCH(0x380, 0x4));

	/* reset dma idx */
	mwx_write(sc, MT_WFDMA0_RST_DTX_PTR, ~0);

	/* configure delay interrupt */
	mwx_write(sc, MT_WFDMA0_PRI_DLY_INT_CFG0, 0);

	mwx_set(sc, MT_WFDMA0_GLO_CFG,
	    MT_WFDMA0_GLO_CFG_TX_WB_DDONE |
	    MT_WFDMA0_GLO_CFG_FIFO_LITTLE_ENDIAN |
	    MT_WFDMA0_GLO_CFG_CLK_GAT_DIS |
	    MT_WFDMA0_GLO_CFG_OMIT_TX_INFO |
	    MT_WFDMA0_GLO_CFG_DMA_SIZE(3) |
	    MT_WFDMA0_GLO_CFG_FIFO_DIS_CHECK |
	    MT_WFDMA0_GLO_CFG_RX_WB_DDONE |
	    MT_WFDMA0_GLO_CFG_CSR_DISP_BASE_PTR_CHAIN_EN |
	    MT_WFDMA0_GLO_CFG_OMIT_RX_INFO_PFET2);

	mwx_barrier(sc);
	mwx_set(sc, MT_WFDMA0_GLO_CFG,
	    MT_WFDMA0_GLO_CFG_TX_DMA_EN | MT_WFDMA0_GLO_CFG_RX_DMA_EN);

	mwx_set(sc, MT_WFDMA_DUMMY_CR, MT_WFDMA_NEED_REINIT);

	/* enable interrupts for TX/RX rings */
	mwx_write(sc, MT_WFDMA0_HOST_INT_ENA, sc->sc_intr_mask);
	mwx_set(sc, MT_MCU2HOST_SW_INT_ENA, MT_MCU_CMD_WAKE_RX_PCIE);
	mwx_write(sc, MT_PCIE_MAC_INT_ENABLE, 0xff);
}

/*
 * PCIe Function Level Reset: resets the whole function like a power cycle.
 * Config space (BARs, command, MSI) is saved and restored around it.
 */
int
mwx_pci_flr(struct mwx_softc *sc, IOPCIDevice *dev)
{
	UInt8 cap = 0;
	UInt32 devcap;
	UInt16 devctl;

	if (dev == NULL ||
	    !dev->findPCICapability(kIOPCICapabilityIDPCIExpress, &cap) ||
	    cap == 0) {
		MWX_DEV_LOG(sc, "FLR: no PCIe capability\n");
		return ENODEV;
	}
	devcap = dev->configRead32(cap + 0x04);
	if (!(devcap & (1U << 28))) {
		MWX_DEV_LOG(sc, "FLR: not supported devcap=0x%08x\n", devcap);
		return ENODEV;
	}
	dev->saveDeviceState();
	devctl = dev->configRead16(cap + 0x08);
	dev->configWrite16(cap + 0x08, devctl | (1U << 15));
	IOSleep(150);
	dev->restoreDeviceState();
	dev->setMemoryEnable(true);
	dev->setBusMasterEnable(true);
	MWX_DEV_LOG(sc, "FLR done vendor=0x%04x\n", dev->configRead16(0));
	return 0;
}

/*
 * PCIe link power states, turned off as Linux mt76 does at probe
 * (mt76_pci_disable_aspm() in mt76/pci.c): read the ASPM control of the card
 * and of the port above it, stop if both ends are already off, and otherwise
 * clear the card before its upstream port. The L1 PM Substates enables go
 * too, as with Linux's pci_disable_link_state(): L1.1 and L1.2 are only
 * entered through L1, so L1 is turned off first and the substates after it,
 * each at the downstream end before the upstream one, in the order Linux
 * uses. Every register that changes is logged with the value read back.
 * -mtkallowaspm leaves the link as the firmware set it.
 */
#define	MWX_PCIE_LNKCTL			0x10		/* in the PCIe capability */
#define	MWX_PCIE_LNKCTL_ASPM		0x0003		/* ASPM Control: L0s, L1 */
#define	MWX_PCIE_L1SS_CTL1		0x08		/* in the L1 PM Substates capability */
#define	MWX_PCIE_L1SS_CTL1_EN		0x0000000f	/* PCI-PM and ASPM L1.2, L1.1 */

struct mwx_link_end {
	IOPCIDevice	*dev;
	const char	*name;
	UInt8		 exp;		/* PCIe capability, 0 if none */
	IOByteCount	 l1ss;		/* L1 PM Substates capability, 0 if none */
	uint16_t	 lnkctl;
	uint32_t	 l1ss_ctl1;
};

static void
mwx_link_end_read(struct mwx_link_end *end, IOPCIDevice *dev, const char *name)
{
	bzero(end, sizeof(*end));
	end->dev = dev;
	end->name = name;
	if (dev == NULL)
		return;
	if (dev->findPCICapability(kIOPCICapabilityIDPCIExpress, &end->exp) == 0)
		end->exp = 0;
	if (end->exp != 0)
		end->lnkctl = dev->configRead16(end->exp + MWX_PCIE_LNKCTL);
	if (dev->extendedFindPCICapability(kIOPCIExpressCapabilityIDL1PMSubstates,
	    &end->l1ss) == 0 || end->l1ss < 0x100)
		end->l1ss = 0;
	if (end->l1ss != 0)
		end->l1ss_ctl1 = dev->extendedConfigRead32(end->l1ss +
		    MWX_PCIE_L1SS_CTL1);
}

static void
mwx_link_end_l1ss_off(struct mwx_softc *sc, const struct mwx_link_end *end)
{
	IOByteCount reg = end->l1ss + MWX_PCIE_L1SS_CTL1;

	if (end->dev == NULL || !(end->l1ss_ctl1 & MWX_PCIE_L1SS_CTL1_EN))
		return;
	end->dev->extendedConfigWrite32(reg,
	    end->l1ss_ctl1 & ~MWX_PCIE_L1SS_CTL1_EN);
	MWX_DEV_LOG(sc, "aspm(%s): L1SS CTL1 0x%08x -> 0x%08x\n", end->name,
	    end->l1ss_ctl1, end->dev->extendedConfigRead32(reg));
}

static void
mwx_link_end_aspm_off(struct mwx_softc *sc, const struct mwx_link_end *end)
{
	UInt8 reg = end->exp + MWX_PCIE_LNKCTL;

	if (end->dev == NULL || !(end->lnkctl & MWX_PCIE_LNKCTL_ASPM))
		return;
	end->dev->configWrite16(reg, end->lnkctl & ~MWX_PCIE_LNKCTL_ASPM);
	MWX_DEV_LOG(sc, "aspm(%s): LNKCTL 0x%04x -> 0x%04x\n", end->name,
	    end->lnkctl, end->dev->configRead16(reg));
}

void
mwx_pci_disable_aspm(struct mwx_softc *sc, IOPCIDevice *dev)
{
	struct mwx_link_end card, port;
	IOPCIDevice *up = NULL;
	int allow = 0;

	if (dev == NULL ||
	    PE_parse_boot_argn("-mtkallowaspm", &allow, sizeof(allow)))
		return;
	/* The upstream port is the nearest PCI device above the card. */
	for (IORegistryEntry *e = dev->getParentEntry(gIOServicePlane);
	    e != NULL && up == NULL; e = e->getParentEntry(gIOServicePlane))
		up = OSDynamicCast(IOPCIDevice, e);
	mwx_link_end_read(&card, dev, "ep");
	mwx_link_end_read(&port, up, "parent");
	if (!(card.lnkctl & MWX_PCIE_LNKCTL_ASPM) &&
	    !(card.l1ss_ctl1 & MWX_PCIE_L1SS_CTL1_EN) &&
	    !(port.lnkctl & MWX_PCIE_LNKCTL_ASPM) &&
	    !(port.l1ss_ctl1 & MWX_PCIE_L1SS_CTL1_EN))
		return;		/* already off at both ends */
	mwx_link_end_aspm_off(sc, &card);
	mwx_link_end_aspm_off(sc, &port);
	mwx_link_end_l1ss_off(sc, &card);
	mwx_link_end_l1ss_off(sc, &port);
}

int
mwx_mcu_fw_pmctrl(struct mwx_softc *sc)
{
	int i;

	for (i = 0; i < MWX_MCU_INIT_RETRY_COUNT; i++) {
		mwx_write(sc, MT_CONN_ON_LPCTL, PCIE_LPCR_HOST_SET_OWN);
		if (mwx_poll(sc, MT_CONN_ON_LPCTL, PCIE_LPCR_HOST_OWN_SYNC,
		    4, 50) == 0)
			break;
	}

	if (i == MWX_MCU_INIT_RETRY_COUNT) {
		printf("%s: firmware own failed\n", DEVNAME(sc));
		return EIO;
	}

	return 0;
}

int
mwx_mcu_drv_pmctrl(struct mwx_softc *sc)
{
	int i;

	for (i = 0; i < MWX_MCU_INIT_RETRY_COUNT; i++) {
		mwx_write(sc, MT_CONN_ON_LPCTL, PCIE_LPCR_HOST_CLR_OWN);
		delay(3000);
		if (mwx_poll(sc, MT_CONN_ON_LPCTL, 0,
		    PCIE_LPCR_HOST_OWN_SYNC, 50) == 0)
			break;
	}

	if (i == MWX_MCU_INIT_RETRY_COUNT) {
		printf("%s: driver own failed\n", DEVNAME(sc));
		return EIO;
	}

	return 0;
}

int
mwx_wfsys_reset(struct mwx_softc *sc)
{
	DPRINTF("%s: WFSYS reset\n", DEVNAME(sc));

	mwx_clear(sc, MT_WFSYS_SW_RST_B, WFSYS_SW_RST_B);
	delay(50 * 1000);
	mwx_set(sc, MT_WFSYS_SW_RST_B, WFSYS_SW_RST_B);

	return mwx_poll(sc, MT_WFSYS_SW_RST_B, WFSYS_SW_INIT_DONE,
	    WFSYS_SW_INIT_DONE, 500);
}

/*
 * To be honest this is ridiculous.
 */
uint32_t
mwx_reg_addr(struct mwx_softc *sc, uint32_t reg)
{
	static const struct {
		uint32_t phys;
		uint32_t mapped;
		uint32_t size;
	} fixed_map[] = {
	{ 0x820d0000, 0x30000, 0x10000 }, /* WF_LMAC_TOP (WF_WTBLON) */
	{ 0x820ed000, 0x24800, 0x00800 }, /* WF_LMAC_TOP BN0 (WF_MIB) */
	{ 0x820e4000, 0x21000, 0x00400 }, /* WF_LMAC_TOP BN0 (WF_TMAC) */
	{ 0x820e7000, 0x21e00, 0x00200 }, /* WF_LMAC_TOP BN0 (WF_DMA) */
	{ 0x820eb000, 0x24200, 0x00400 }, /* WF_LMAC_TOP BN0 (WF_LPON) */
	{ 0x820e2000, 0x20800, 0x00400 }, /* WF_LMAC_TOP BN0 (WF_AGG) */
	{ 0x820e3000, 0x20c00, 0x00400 }, /* WF_LMAC_TOP BN0 (WF_ARB) */
	{ 0x820e5000, 0x21400, 0x00800 }, /* WF_LMAC_TOP BN0 (WF_RMAC) */
	{ 0x00400000, 0x80000, 0x10000 }, /* WF_MCU_SYSRAM */
	{ 0x00410000, 0x90000, 0x10000 }, /* WF_MCU_SYSRAM (conf register) */
	{ 0x40000000, 0x70000, 0x10000 }, /* WF_UMAC_SYSRAM */
	{ 0x54000000, 0x02000, 0x01000 }, /* WFDMA PCIE0 MCU DMA0 */
	{ 0x55000000, 0x03000, 0x01000 }, /* WFDMA PCIE0 MCU DMA1 */
	{ 0x58000000, 0x06000, 0x01000 }, /* WFDMA PCIE1 MCU DMA0 (MEM_DMA) */
	{ 0x59000000, 0x07000, 0x01000 }, /* WFDMA PCIE1 MCU DMA1 */
	{ 0x7c000000, 0xf0000, 0x10000 }, /* CONN_INFRA */
	{ 0x7c020000, 0xd0000, 0x10000 }, /* CONN_INFRA, WFDMA */
	{ 0x7c060000, 0xe0000, 0x10000 }, /* CONN_INFRA, conn_host_csr_top */
	{ 0x80020000, 0xb0000, 0x10000 }, /* WF_TOP_MISC_OFF */
	{ 0x81020000, 0xc0000, 0x10000 }, /* WF_TOP_MISC_ON */
	{ 0x820c0000, 0x08000, 0x04000 }, /* WF_UMAC_TOP (PLE) */
	{ 0x820c8000, 0x0c000, 0x02000 }, /* WF_UMAC_TOP (PSE) */
	{ 0x820cc000, 0x0e000, 0x01000 }, /* WF_UMAC_TOP (PP) */
	{ 0x820cd000, 0x0f000, 0x01000 }, /* WF_MDP_TOP */
	{ 0x820ce000, 0x21c00, 0x00200 }, /* WF_LMAC_TOP (WF_SEC) */
	{ 0x820cf000, 0x22000, 0x01000 }, /* WF_LMAC_TOP (WF_PF) */
	{ 0x820e0000, 0x20000, 0x00400 }, /* WF_LMAC_TOP BN0 (WF_CFG) */
	{ 0x820e1000, 0x20400, 0x00200 }, /* WF_LMAC_TOP BN0 (WF_TRB) */
	{ 0x820e9000, 0x23400, 0x00200 }, /* WF_LMAC_TOP BN0 (WF_WTBLOFF) */
	{ 0x820ea000, 0x24000, 0x00200 }, /* WF_LMAC_TOP BN0 (WF_ETBF) */
	{ 0x820ec000, 0x24600, 0x00200 }, /* WF_LMAC_TOP BN0 (WF_INT) */
	{ 0x820f0000, 0xa0000, 0x00400 }, /* WF_LMAC_TOP BN1 (WF_CFG) */
	{ 0x820f1000, 0xa0600, 0x00200 }, /* WF_LMAC_TOP BN1 (WF_TRB) */
	{ 0x820f2000, 0xa0800, 0x00400 }, /* WF_LMAC_TOP BN1 (WF_AGG) */
	{ 0x820f3000, 0xa0c00, 0x00400 }, /* WF_LMAC_TOP BN1 (WF_ARB) */
	{ 0x820f4000, 0xa1000, 0x00400 }, /* WF_LMAC_TOP BN1 (WF_TMAC) */
	{ 0x820f5000, 0xa1400, 0x00800 }, /* WF_LMAC_TOP BN1 (WF_RMAC) */
	{ 0x820f7000, 0xa1e00, 0x00200 }, /* WF_LMAC_TOP BN1 (WF_DMA) */
	{ 0x820f9000, 0xa3400, 0x00200 }, /* WF_LMAC_TOP BN1 (WF_WTBLOFF) */
	{ 0x820fa000, 0xa4000, 0x00200 }, /* WF_LMAC_TOP BN1 (WF_ETBF) */
	{ 0x820fb000, 0xa4200, 0x00400 }, /* WF_LMAC_TOP BN1 (WF_LPON) */
	{ 0x820fc000, 0xa4600, 0x00200 }, /* WF_LMAC_TOP BN1 (WF_INT) */
	{ 0x820fd000, 0xa4800, 0x00800 }, /* WF_LMAC_TOP BN1 (WF_MIB) */
	};
	int i;

	if (reg < 0x100000)
		return reg;

	for (i = 0; i < nitems(fixed_map); i++) {
		uint32_t ofs;

		if (reg < fixed_map[i].phys)
			continue;

		ofs = reg - fixed_map[i].phys;
		if (ofs > fixed_map[i].size)
			continue;

		return fixed_map[i].mapped + ofs;
	}

	if ((reg >= 0x18000000 && reg < 0x18c00000) ||
	    (reg >= 0x70000000 && reg < 0x78000000) ||
	    (reg >= 0x7c000000 && reg < 0x7c400000))
		return mt7921_map_reg_l1(sc, reg);

	panic("%s: Access to currently unsupported address %08x\n",
	    DEVNAME(sc), reg);
}

int
mwx_init_hardware(struct mwx_softc *sc)
{
	int rv;

	MWX_DEV_LOG(sc, "hardware init start\n");
	/* reset dma */
	rv = mwx_dma_reset(sc, 1);
	if (rv != 0) {
		MWX_DEV_LOG(sc, "hardware init dma reset failed error=%d\n", rv);
		return rv;
	}
	MWX_DEV_LOG(sc, "hardware init dma reset ok\n");

	/*
	 * force firmware operation mode into normal state,
	 * which should be set before firmware download stage.
	 */
	mwx_write(sc, MT_SWDEF_MODE, MT_SWDEF_NORMAL_MODE);
	mwx_barrier(sc);
	MWX_DEV_LOG(sc, "hardware init normal firmware mode set\n");

	rv = mt7921_mcu_init(sc);
	if (rv != 0) {
		MWX_DEV_LOG(sc, "hardware init MCU init failed error=%d\n", rv);
		goto fail;
	}
	MWX_DEV_LOG(sc, "hardware init MCU init ok\n");
	/* TODO override eeprom for systems with FDT */
	rv = mt7921_mcu_set_eeprom(sc);
	if (rv != 0) {
		MWX_DEV_LOG(sc, "hardware init EEPROM setup failed error=%d\n", rv);
		goto fail;
	}
	MWX_DEV_LOG(sc, "hardware init EEPROM setup ok\n");
	mwx_mac_init(sc);

	sc->sc_vif.vif_mn.wcid = MWX_WTBL_RESERVED;

	/* MAYBE alloc beacon and mgmt frame wcid 0 here */

	MWX_DEV_LOG(sc, "hardware init complete\n");
	return 0;

 fail:
	/* reset dma */
	MWX_DEV_LOG(sc, "hardware init recovery dma reset start\n");
	rv = mwx_dma_reset(sc, 1);
	if (rv != 0) {
		MWX_DEV_LOG(sc, "hardware init recovery dma reset failed error=%d\n",
		    rv);
		return rv;
	}
	return EAGAIN;
}

int
mt7921_mcu_init(struct mwx_softc *sc)
{
	int rv;

	MWX_DEV_LOG(sc, "MCU init start\n");
	/* this read is needed to make interrupts work */
	(void) mwx_read(sc, MT_TOP_LPCR_HOST_BAND0);
	mwx_write(sc, MT_TOP_LPCR_HOST_BAND0, MT_TOP_LPCR_HOST_DRV_OWN);
	if (mwx_poll(sc, MT_TOP_LPCR_HOST_BAND0, 0, MT_TOP_LPCR_HOST_FW_OWN,
	    5000) != 0) {
	    MWX_DEV_LOG(sc, "MCU init timeout for driver own\n");
	    return EIO;
	}
	MWX_DEV_LOG(sc, "MCU init driver own ok\n");

	mwx_set(sc, MT_PCIE_MAC_PM, MT_PCIE_MAC_PM_L0S_DIS);

	if ((rv = mwx_load_firmware(sc)) != 0) {
		MWX_DEV_LOG(sc, "MCU init firmware load failed error=%d\n", rv);
		return rv;
	}
	MWX_DEV_LOG(sc, "MCU init firmware load ok\n");

	if ((rv = mt7921_mcu_get_nic_capability(sc)) != 0) {
		MWX_DEV_LOG(sc, "MCU init NIC capability failed error=%d\n", rv);
		return rv;
	}
	MWX_DEV_LOG(sc, "MCU init NIC capability ok\n");
	if ((rv = mt7921_mcu_fw_log_2_host(sc, 1)) != 0) {
		MWX_DEV_LOG(sc, "MCU init firmware log enable failed error=%d\n",
		    rv);
		return rv;
	}

	/* TODO mark MCU running */

	MWX_DEV_LOG(sc, "MCU init complete\n");
	return 0;
}

static inline uint32_t
mwx_get_data_mode(struct mwx_softc *sc, uint32_t info)
{
	uint32_t mode = DL_MODE_NEED_RSP;

	if (info == PATCH_SEC_NOT_SUPPORT)
		return mode;
	switch (info & PATCH_SEC_ENC_TYPE_MASK) {
	case PATCH_SEC_ENC_TYPE_PLAIN:
		break;
	case PATCH_SEC_ENC_TYPE_AES:
		mode |= DL_MODE_ENCRYPT;
		mode |= (info << DL_MODE_KEY_IDX_SHIFT) & DL_MODE_KEY_IDX_MASK;
		mode |= DL_MODE_RESET_SEC_IV;
		break;
	case PATCH_SEC_ENC_TYPE_SCRAMBLE:
		mode |= DL_MODE_ENCRYPT;
		mode |= DL_CONFIG_ENCRY_MODE_SEL;
		mode |= DL_MODE_RESET_SEC_IV;
		break;
	default:
		printf("%s: encryption type not supported\n", DEVNAME(sc));
	}
	return mode;
}

static inline uint32_t
mwx_mcu_gen_dl_mode(uint8_t feature_set)
{
	uint32_t ret = DL_MODE_NEED_RSP;

	if (feature_set & FW_FEATURE_SET_ENCRYPT)
		ret |= (DL_MODE_ENCRYPT | DL_MODE_RESET_SEC_IV);
	if (feature_set & FW_FEATURE_ENCRY_MODE)
		ret |= DL_CONFIG_ENCRY_MODE_SEL;

	/* FW_FEATURE_SET_KEY_IDX_MASK == DL_MODE_KEY_IDX_MASK */
	ret |= feature_set & FW_FEATURE_SET_KEY_IDX_MASK;

	return ret;
}


int
mwx_load_firmware(struct mwx_softc *sc)
{
	struct mwx_patch_hdr *hdr;
	struct mwx_fw_trailer *fwhdr;
	const char *rompatch, *fw;
	u_char *buf = NULL, *fwbuf = NULL, *dl;
	size_t buflen = 0, fwlen = 0, offset = 0;
	uint32_t reg, n_region, override = 0, option = 0;
	int i, rv, sem;

	MWX_DEV_LOG(sc, "firmware load start hwtype=%s\n",
	    mwx_hw_name(sc->sc_hwtype));
	reg = mwx_read(sc, MT_CONN_ON_MISC) & MT_TOP_MISC2_FW_N9_RDY;
	if (reg != 0) {
		MWX_DEV_LOG(sc, "firmware already downloaded status=0x%08x\n", reg);
		return 0;
	}

	mwx_hw_firmware(sc->sc_hwtype, &rompatch, &fw);
	MWX_DEV_LOG(sc, "firmware selected patch=%s wm=%s\n", rompatch, fw);

	if ((rv = loadfirmware(rompatch, &buf, &buflen)) != 0) {
		MWX_DEV_LOG(sc, "patch firmware load failed name=%s error=%d\n",
		    rompatch, rv);
		return rv;
	}
	if ((rv = loadfirmware(fw, &fwbuf, &fwlen)) != 0) {
		MWX_DEV_LOG(sc, "WM firmware load failed name=%s error=%d\n",
		    fw, rv);
		free(buf, M_DEVBUF, buflen);
		return rv;
	}
	MWX_DEV_LOG(sc, "firmware blobs loaded patch_size=%zu wm_size=%zu\n",
	    buflen, fwlen);

	rv = mwx_mcu_patch_sem_ctrl(sc, 1);
	if (rv == -1) {
		/*
		 * PATCH_IS_DL: the chip still holds a ROM patch, which happens
		 * when the card kept power since another driver loaded one (a
		 * warm reboot). Like Linux mt76_connac2_load_patch(), skip the
		 * patch and download only the WM firmware: no semaphore was
		 * taken, so there is none to release.
		 */
		MWX_DEV_LOG(sc, "patch already downloaded, loading the WM firmware only\n");
		rv = 0;
		goto wm;
	}
	if (rv != 0) {
		MWX_DEV_LOG(sc, "patch semaphore acquire failed error=%d\n", rv);
		goto fail;
	}
	MWX_DEV_LOG(sc, "patch semaphore acquired\n");

	if (buflen < sizeof(*hdr)) {
		MWX_DEV_LOG(sc, "invalid patch firmware size=%zu min=%zu\n",
		    buflen, sizeof(*hdr));
		rv = EINVAL;
		goto out;
	}
	hdr = (struct mwx_patch_hdr *)buf;
	n_region = be32toh(hdr->desc.n_region);
	if (buflen < sizeof(*hdr) + n_region * sizeof(struct mwx_patch_sec)) {
		MWX_DEV_LOG(sc, "invalid patch firmware short header size=%zu regions=%u\n",
		    buflen, n_region);
		rv = EINVAL;
		goto out;
	}
	MWX_DEV_LOG(sc, "patch header hw_sw=0x%x patch_ver=0x%x regions=%u build=%.15s\n",
	    be32toh(hdr->hw_sw_ver), be32toh(hdr->patch_ver),
	    n_region, hdr->build_date);

	for (i = 0; i < n_region; i++) {
		struct mwx_patch_sec *sec;
		uint32_t len, addr, mode, sec_info;

		sec = (struct mwx_patch_sec *)(buf + sizeof(*hdr) +
		    i * sizeof(*sec));
		if ((be32toh(sec->type) & PATCH_SEC_TYPE_MASK) !=
		    PATCH_SEC_TYPE_INFO) {
			MWX_DEV_LOG(sc, "invalid patch region index=%d type=0x%x\n",
			    i, be32toh(sec->type));
			rv = EINVAL;
			goto out;
		}

		addr = be32toh(sec->info.addr);
		len = be32toh(sec->info.len);
		dl = buf + be32toh(sec->offs);
		sec_info = be32toh(sec->info.sec_key_idx);
		mode = mwx_get_data_mode(sc, sec_info);
		MWX_DEV_LOG(sc, "patch region index=%d addr=0x%08x len=%u mode=0x%08x sec=0x%08x\n",
		    i, addr, len, mode, sec_info);

		if (dl + len > buf + buflen) {
			MWX_DEV_LOG(sc, "patch region index=%d exceeds payload len=%u\n",
			    i, len);
			rv = EINVAL;
			goto out;
		}

		rv = mwx_mcu_init_download(sc, addr, len, mode);
		if (rv != 0) {
			MWX_DEV_LOG(sc, "patch download request failed index=%d error=%d\n",
			    i, rv);
			goto out;
		}
		rv = mwx_mcu_send_firmware(sc, MCU_CMD_FW_SCATTER,
		    dl, len, MWX_FW_CHUNK_SIZE);
		if (rv != 0) {
			MWX_DEV_LOG(sc, "patch send failed index=%d error=%d\n",
			    i, rv);
			goto out;
		}
	}

	delay(5000);
	rv = mwx_mcu_start_patch(sc);
	if (rv != 0) {
		MWX_DEV_LOG(sc, "patch start failed error=%d\n", rv);
		goto out;
	}
	MWX_DEV_LOG(sc, "patch start ok\n");

out:
	sem = mwx_mcu_patch_sem_ctrl(sc, 0);
	if (sem != 0)
		rv = sem;
	if (rv != 0)
		goto fail;

wm:
	if (fwlen < sizeof(*fwhdr)) {
		MWX_DEV_LOG(sc, "invalid WM firmware size=%zu min=%zu\n",
		    fwlen, sizeof(*fwhdr));
		rv = EINVAL;
		goto fail;
	}
	fwhdr = (struct mwx_fw_trailer *)(fwbuf + fwlen - sizeof(*fwhdr));
	MWX_DEV_LOG(sc, "WM header version=%.10s build=%.15s chip=0x%02x eco=0x%02x regions=%u\n",
	    fwhdr->fw_ver, fwhdr->build_date, fwhdr->chip_id,
	    fwhdr->eco_code, fwhdr->n_region);

	if (fwlen < sizeof(*fwhdr) +
	    fwhdr->n_region * sizeof(struct mwx_fw_region)) {
		MWX_DEV_LOG(sc, "invalid WM firmware short header size=%zu regions=%u\n",
		    fwlen, fwhdr->n_region);
		rv = EINVAL;
		goto fail;
	}

	for (i = 0; i < fwhdr->n_region; i++) {
		struct mwx_fw_region *region;
		uint32_t len, addr, mode;

		region = (struct mwx_fw_region *)((u_char *)fwhdr -
		    (fwhdr->n_region - i) * sizeof(*region));

		addr = le32toh(region->addr);
		len = le32toh(region->len);
		mode = mwx_mcu_gen_dl_mode(region->feature_set);

		if (region->feature_set & FW_FEATURE_OVERRIDE_ADDR)
			override = addr;
		MWX_DEV_LOG(sc, "WM region index=%d addr=0x%08x len=%u mode=0x%08x feature=0x%02x offset=%zu\n",
		    i, addr, len, mode, region->feature_set, offset);

		if (offset + len > fwlen - sizeof(*fwhdr) -
		    fwhdr->n_region * sizeof(struct mwx_fw_region)) {
			MWX_DEV_LOG(sc, "WM region index=%d exceeds payload len=%u offset=%zu\n",
			    i, len, offset);
			rv = EINVAL;
			goto fail;
		}

		if (region->feature_set & FW_FEATURE_NON_DL) {
			offset += len;
			continue;
		}

		rv = mwx_mcu_init_download(sc, addr, len, mode);
		if (rv != 0) {
			MWX_DEV_LOG(sc, "WM download request failed index=%d error=%d\n",
			    i, rv);
			goto fail;
		}

		rv = mwx_mcu_send_firmware(sc, MCU_CMD_FW_SCATTER,
		    fwbuf + offset, len, MWX_FW_CHUNK_SIZE);
		if (rv != 0) {
			MWX_DEV_LOG(sc, "WM send failed index=%d error=%d\n",
			    i, rv);
			goto fail;
		}
		offset += len;
	}

	if (override != 0)
		option |= FW_START_OVERRIDE;

	rv = mwx_mcu_start_firmware(sc, override, option);
	if (rv != 0) {
		MWX_DEV_LOG(sc, "firmware start failed override=0x%08x option=0x%08x error=%d\n",
		    override, option, rv);
		goto fail;
	}
	MWX_DEV_LOG(sc, "firmware start ok override=0x%08x option=0x%08x\n",
	    override, option);

	/* XXX should not busy poll here */
	if (mwx_poll(sc, MT_CONN_ON_MISC, MT_TOP_MISC2_FW_N9_RDY,
	    MT_TOP_MISC2_FW_N9_RDY, 1500) != 0) {
		MWX_DEV_LOG(sc, "firmware ready poll timed out\n");
		rv = EIO;
		goto fail;
	}

	MWX_DEV_LOG(sc, "firmware loaded and ready\n");
	rv = 0;

fail:
	if (buf != NULL)
		free(buf, M_DEVBUF, buflen);
	if (fwbuf != NULL)
		free(fwbuf, M_DEVBUF, fwlen);
	return rv;
}

int
mwx_mac_wtbl_update(struct mwx_softc *sc, int idx)
{
	mwx_rmw(sc, MT_WTBL_UPDATE,
	    (idx & MT_WTBL_UPDATE_WLAN_IDX_MASK) | MT_WTBL_UPDATE_ADM_COUNT_CLEAR,
	    MT_WTBL_UPDATE_WLAN_IDX_MASK);

	return mwx_poll(sc, MT_WTBL_UPDATE, 0, MT_WTBL_UPDATE_BUSY, 5000);
}

void
mwx_mac_init_band(struct mwx_softc *sc, uint32_t band)
{
	mwx_rmw(sc, MT_TMAC_CTCR0(band), 0x3f, MT_TMAC_CTCR0_INS_DDLMT_REFTIME);
	mwx_set(sc, MT_TMAC_CTCR0(band),
	    MT_TMAC_CTCR0_INS_DDLMT_VHT_SMPDU_EN |
	    MT_TMAC_CTCR0_INS_DDLMT_EN);

	mwx_set(sc, MT_WF_RMAC_MIB_TIME0(band), MT_WF_RMAC_MIB_RXTIME_EN);
	mwx_set(sc, MT_WF_RMAC_MIB_AIRTIME0(band), MT_WF_RMAC_MIB_RXTIME_EN);

	/* enable MIB tx-rx time reporting */
	mwx_set(sc, MT_MIB_SCR1(band), MT_MIB_TXDUR_EN);
	mwx_set(sc, MT_MIB_SCR1(band), MT_MIB_RXDUR_EN);

	mwx_rmw(sc, MT_DMA_DCR0(band),
	    1536 << MT_DMA_DCR0_MAX_RX_LEN_SHIFT, MT_DMA_DCR0_MAX_RX_LEN_MASK);
	/* disable rx rate report by default due to hw issues */
	mwx_clear(sc, MT_DMA_DCR0(band), MT_DMA_DCR0_RXD_G5_EN);

	/* filter out non-resp frames and get instantaneous signal reporting */
	mwx_rmw(sc, MT_WTBLOFF_TOP_RSCR(band),
	    0 << MT_WTBLOFF_TOP_RSCR_RCPI_MODE_SHIFT |
	    0x3 << MT_WTBLOFF_TOP_RSCR_RCPI_PARAM_SHIFT,
	    MT_WTBLOFF_TOP_RSCR_RCPI_MODE_MASK |
	    MT_WTBLOFF_TOP_RSCR_RCPI_PARAM_MASK);
}

void
mwx_mac_init(struct mwx_softc *sc)
{
	int i;

	mwx_rmw(sc, MT_MDP_DCR1, 1536 << MT_MDP_DCR1_MAX_RX_LEN_SHIFT,
	    MT_MDP_DCR1_MAX_RX_LEN_MASK);

	/* enable hardware de-agg */
	mwx_set(sc, MT_MDP_DCR0, MT_MDP_DCR0_DAMSDU_EN);
#if 0
	/* not enabled since our stack does not handle 802.3 frames */
	/* enable hardware rx header translation */
	mwx_set(sc, MT_MDP_DCR0, MT_MDP_DCR0_RX_HDR_TRANS_EN);
#endif

	for (i = 0; i < MWX_WTBL_SIZE; i++)
		mwx_mac_wtbl_update(sc, i);

	mwx_mac_init_band(sc, 0);
	mwx_mac_init_band(sc, 1);

	sc->sc_rxfilter = mwx_read(sc, MT_WF_RFCR(0));
}

int
mwx_mcu_patch_sem_ctrl(struct mwx_softc *sc, int semget)
{
#define	PATCH_SEM_RELEASE		0
#define PATCH_SEM_GET			1
#define	PATCH_NOT_DL_SEM_FAIL		0
#define	PATCH_IS_DL			1
#define	PATCH_NOT_DL_SEM_SUCCESS	2
#define	PATCH_REL_SEM_SUCCESS		3

	uint32_t op = semget ? PATCH_SEM_GET : PATCH_SEM_RELEASE;
	struct {
		uint32_t op;
	} req = {
		.op = htole32(op),
	};
	int rv, seq;
	uint32_t sem;

	rv = mwx_mcu_send_msg(sc, MCU_CMD_PATCH_SEM_CONTROL,
	    &req, sizeof(req), &seq);
	if (rv != 0)
		return rv;

	rv = mwx_mcu_wait_resp_int(sc, MCU_CMD_PATCH_SEM_CONTROL, seq, &sem);
	if (rv != 0)
		return rv;

	if (semget) {
		switch (sem) {
		case PATCH_IS_DL:
			/* Not an error: nothing to download, no semaphore held. */
			return -1;
		case PATCH_NOT_DL_SEM_SUCCESS:
			return 0;
		default:
			DPRINTF("%s: failed to %s patch semaphore\n",
			    DEVNAME(sc), "get");
			return EAGAIN;
		}
	} else {
		switch (sem) {
		case PATCH_REL_SEM_SUCCESS:
			return 0;
		default:
			DPRINTF("%s: failed to %s patch semaphore\n",
			    DEVNAME(sc), "release");
			return EAGAIN;
		}
	}
}

int
mwx_mcu_init_download(struct mwx_softc *sc, uint32_t addr,
    uint32_t len, uint32_t mode)
{
	struct {
		uint32_t addr;
		uint32_t len;
		uint32_t mode;
	} req = {
		.addr = htole32(addr),
		.len = htole32(len),
		.mode = htole32(mode),
	};
	int cmd;

	if (addr == 0x200000 || addr == 0x900000)
		cmd = MCU_CMD_PATCH_START_REQ;
	else
		cmd = MCU_CMD_TARGET_ADDRESS_LEN_REQ;

	return mwx_mcu_send_wait(sc, cmd, &req, sizeof(req));
}

int
mwx_mcu_send_firmware(struct mwx_softc *sc, int cmd, u_char *data,
    size_t len, size_t max_len)
{
	size_t cur_len;
	int rv;

	while (len > 0) {
		cur_len = len;
		if (cur_len > max_len)
			cur_len = max_len;

		rv = mwx_mcu_send_msg(sc, cmd, data, cur_len, NULL);
		if (rv != 0)
			return rv;

		data += cur_len;
		len -= cur_len;

		mwx_dma_tx_cleanup(sc, &sc->sc_txfwdlq);
	}

	return 0;
}

int
mwx_mcu_start_patch(struct mwx_softc *sc)
{
	struct {
		uint8_t check_crc;
		uint8_t reserved[3];
	} req = {
		.check_crc = 0,
	};

	return mwx_mcu_send_wait(sc, MCU_CMD_PATCH_FINISH_REQ, &req,
	    sizeof(req));
}

int
mwx_mcu_start_firmware(struct mwx_softc *sc, uint32_t addr, uint32_t option)
{
	struct {
		uint32_t option;
		uint32_t addr;
	} req = {
		.option = htole32(option),
		.addr = htole32(addr),
	};

	return mwx_mcu_send_wait(sc, MCU_CMD_FW_START_REQ, &req, sizeof(req));
}

int
mt7921_mcu_get_nic_capability(struct mwx_softc *sc)
{
	struct mt76_connac_cap_hdr {
		uint16_t	n_elements;
		uint16_t	pad;
	} __packed *hdr;
	struct tlv_hdr {
		uint32_t	type;
		uint32_t	len;
	} __packed *tlv;
	struct mt76_connac_phy_cap {
		uint8_t		ht;
		uint8_t		vht;
		uint8_t		_5g;
		uint8_t		max_bw;
		uint8_t		nss;
		uint8_t		dbdc;
		uint8_t		tx_ldpc;
		uint8_t		rx_ldpc;
		uint8_t		tx_stbc;
		uint8_t		rx_stbc;
		uint8_t		hw_path;
		uint8_t		he;
	} __packed *cap;
	struct mbuf *m;
	int rv, seq, count, i;

	rv = mwx_mcu_send_msg(sc, MCU_CE_CMD_GET_NIC_CAPAB, NULL, 0, &seq);
	if (rv != 0)
		return rv;

	rv = mwx_mcu_wait_resp_msg(sc, MCU_CE_CMD_GET_NIC_CAPAB, seq, &m);
	if (rv != 0)
		return rv;

	/* the message was already pulled up by mwx_mcu_rx_event() */
	if (mbuf_len(m) < sizeof(*hdr)) {
		printf("%s: GET_NIC_CAPAB response size error\n", DEVNAME(sc));
		m_freem(m);
		return EINVAL;
	}
	hdr = mtod(m, struct mt76_connac_cap_hdr *);
	count = le16toh(hdr->n_elements);
	m_adj(m, sizeof(*hdr));

	for (i = 0; i < count; i++) {
		uint32_t type, len;

		if (mbuf_len(m) < sizeof(*tlv)) {
			printf("%s: GET_NIC_CAPAB tlv size error\n",
			    DEVNAME(sc));
			m_freem(m);
			return EINVAL;
		}

		tlv = mtod(m, struct tlv_hdr *);
		type = le32toh(tlv->type);
		len = le32toh(tlv->len);
		m_adj(m, sizeof(*tlv));

		if (mbuf_len(m) < len)
			break;
		switch (type) {
		case MT_NIC_CAP_6G:
			/* TODO 6GHZ SUPPORT */
			sc->sc_capa.has_6ghz = /* XXX 1 */ 0;
			break;
		case MT_NIC_CAP_MAC_ADDR:
			if (len < ETHER_ADDR_LEN)
				break;
			memcpy(sc->sc_lladdr, mtod(m, caddr_t), ETHER_ADDR_LEN);
			break;
		case MT_NIC_CAP_PHY:
			if (len < sizeof(*cap))
				break;
			cap = mtod(m, struct mt76_connac_phy_cap *);

			sc->sc_capa.num_streams = cap->nss;
			sc->sc_capa.antenna_mask = (1U << cap->nss) - 1;
			sc->sc_capa.has_2ghz = cap->hw_path & 0x01;
			sc->sc_capa.has_5ghz = cap->hw_path & 0x02;
			break;
		case MT_NIC_CAP_TX_RESOURCE:
			/* unused on PCIe devices */
			break;
		}
		m_adj(m, len);
	}

	printf("%s: address %s\n", DEVNAME(sc), ether_sprintf(sc->sc_lladdr));

	m_freem(m);
	return 0;
}

int
mt7921_mcu_fw_log_2_host(struct mwx_softc *sc, uint8_t ctrl)
{
	struct {
		uint8_t	ctrl;
		uint8_t	pad[3];
	} req = {
		.ctrl = ctrl,
	};

	return mwx_mcu_send_msg(sc, MCU_CE_CMD_FWLOG_2_HOST, &req,
	    sizeof(req), NULL);
}

int
mt7921_mcu_set_eeprom(struct mwx_softc *sc)
{
	struct req_hdr {
		uint8_t	buffer_mode;
		uint8_t	format;
		uint8_t pad[2];
	} req = {
		.buffer_mode = EE_MODE_EFUSE,
		.format = EE_FORMAT_WHOLE,
	};

	return mwx_mcu_send_wait(sc, MCU_EXT_CMD_EFUSE_BUFFER_MODE, &req,
	    sizeof(req));
}

int
mt7921_mcu_set_rts_thresh(struct mwx_softc *sc, uint32_t val, uint8_t band)
{
	struct {
		uint8_t		prot_idx;
		uint8_t		band;
		uint8_t		rsv[2];
		uint32_t	len_thresh;
		uint32_t	pkt_thresh;
	} __packed req = {
		.prot_idx = 1,
		.band = band,
		.len_thresh = htole32(val),
		.pkt_thresh = htole32(0x2),
	};

	return mwx_mcu_send_wait(sc, MCU_EXT_CMD_PROTECT_CTRL, &req,
	    sizeof(req));
}

int
mwx_mcu_set_deep_sleep(struct mwx_softc *sc, int ena)
{
	struct mwx_connac_config req = {
		.resp_type = 0,
	};

	DPRINTF("%s: %s deep sleep\n", DEVNAME(sc), ena ? "enable" : "disable");
#ifdef MTK_AWDL
	sc->sc_awdl_ds = ena;	/* the AWDL radio keeps full power for its windows and then gives this back */
#endif
	snprintf((char *)req.data, sizeof(req.data), "KeepFullPwr %d", !ena);
	return mwx_mcu_send_msg(sc, MCU_CE_CMD_CHIP_CONFIG, &req,
	     sizeof(req), NULL);
}

void
mt7921_mcu_low_power_event(struct mwx_softc *sc, struct mbuf *m)
{
	struct mt7921_mcu_lp_event {
		uint8_t state;
		uint8_t reserved[3];
	} __packed *event;

	if (mbuf_len(m) < sizeof(*event))
		return;
	event = mtod(m, struct mt7921_mcu_lp_event *);
	DPRINTF("%s: low power event state %d\n", DEVNAME(sc), event->state);
}

void
mt7921_mcu_tx_done_event(struct mwx_softc *sc, struct mbuf *m)
{
	struct mt7921_mcu_tx_done_event {
		uint8_t		pid;
		uint8_t		status;
		uint16_t	seq;
		uint8_t		wlan_idx;
		uint8_t		tx_cnt;
		uint16_t	tx_rate;
		uint8_t		flag;
		uint8_t		tid;
		uint8_t		rsp_rate;
		uint8_t		mcs;
		uint8_t		bw;
		uint8_t		tx_pwr;
		uint8_t		reason;
		uint8_t		rsv0[1];
		uint32_t	delay;
		uint32_t	timestamp;
		uint32_t	applied_flag;
		uint8_t		txs[28];
		uint8_t		rsv1[32];
	} __packed *event;

	if (mbuf_len(m) < sizeof(*event))
		return;
	event = mtod(m, struct mt7921_mcu_tx_done_event *);
	// TODO mt7921_mac_add_txs(dev, event->txs);
}

int
mt7921_mcu_hw_scan(struct mwx_softc *sc, int bgscan)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_channel *c;
	struct mt76_connac_hw_scan_req	*req;
	struct mbuf *m;
	int n_ssids = 0;
	int rv;
	uint16_t dwell;
	uint8_t nchan, n_2g = 0, n_5g = 0;
	uint8_t join_channel = 0;

	m = mwx_mcu_alloc_msg(sizeof(*req));
	if (m == NULL)
		return ENOMEM;
	req = mtod(m, struct mt76_connac_hw_scan_req *);
	/* The mbuf payload is not initialized by allocation. */
	memset(req, 0, sizeof(*req));

	sc->sc_flags |= MWX_FLAG_SCANNING;
#ifdef MTK_AWDL
	/* A reception the AWDL radio holds open between its windows goes back to the station's before the sweep. */
	if (sc->sc_awdl != NULL)
		mwx_awdl_rx_release(sc);
#endif
	sc->sc_scan_seq_num = (sc->sc_scan_seq_num + 1) & 0x7f;

	req->seq_num = sc->sc_scan_seq_num /* | sc->sc_band_idx << 7 */;
	req->bss_idx = /* mvif->idx */ 0;
	req->scan_type = /* sreq->n_ssids ? 1 : */ 0;
	req->probe_req_num = /* sreq->n_ssids ? 2 : */ 0;
	req->version = 1;

#ifdef AIRPORT_WCL
	/*
	 * A scan macOS asked for (AirportMtkWlan): active when it wants probes,
	 * with its SSID (hidden networks), on its channels. Scans the driver
	 * starts itself stay passive over every channel, as with SimpleMtkWlan.
	 * Like Linux mt76: probing with no SSID is the wildcard (ssid_type 1).
	 */
	if (ic->ic_wcl_scan_requested &&
	    AirportWCL::scanUsesProbes(true, ic->ic_wcl_scan_active)) {
		req->scan_type = 1;
		req->probe_req_num = 2;
		if (ic->ic_wcl_scan_ssid_length > 0 &&
		    ic->ic_wcl_scan_ssid_length <= IEEE80211_NWID_LEN) {
			req->ssids[0].ssid_len = htole32(ic->ic_wcl_scan_ssid_length);
			memcpy(req->ssids[0].ssid, ic->ic_wcl_scan_ssid,
			    ic->ic_wcl_scan_ssid_length);
			n_ssids = 1;
		}
	}
	/*
	 * The first scan of a join (startWCLJoin()) listens only on the
	 * channel macOS named, for two beacon intervals, instead of sweeping
	 * every channel first. It is used once: if the network is not heard
	 * there, the next scan of the join sweeps every channel.
	 */
	if (!bgscan && ic->ic_wcl_join_requested && ic->ic_wcl_join_channel) {
		join_channel = ic->ic_wcl_join_channel;
		ic->ic_wcl_join_channel = 0;
	}
#endif

#ifdef NOTYET
	for (i = 0; i < sreq->n_ssids; i++) {
		if (!sreq->ssids[i].ssid_len)
			continue;
		req->ssids[i].ssid_len = htole32(sreq->ssids[i].ssid_len);
		memcpy(req->ssids[i].ssid, sreq->ssids[i].ssid,
		    sreq->ssids[i].ssid_len);
		n_ssids++;
	}
#endif

	req->ssid_type = n_ssids ? 0x4 : 0x1;
	req->ssid_type_ext = n_ssids ? 1 : 0;
	req->ssids_num = n_ssids;

	for (nchan = 0, c = &ic->ic_channels[1];
	    c <= &ic->ic_channels[IEEE80211_CHAN_MAX] &&
	    nchan < 64; c++) {
		struct mt76_connac_mcu_scan_channel *chan;
		uint8_t channel_num;

		if (c->ic_flags == 0)
			continue;
#ifdef AIRPORT_WCL
		if (join_channel != 0) {
			if (ieee80211_mhz2ieee(c->ic_freq, 0) != join_channel)
				continue;
		} else if (!AirportWCL::scanChannelSelected(ic->ic_wcl_scan_channels,
		    ic->ic_wcl_scan_requested && ic->ic_wcl_scan_restricted,
		    ieee80211_mhz2ieee(c->ic_freq, 0)))
			continue;
#endif

		if (nchan < 32)
			chan = &req->channels[nchan];
		else
			chan = &req->ext_channels[nchan - 32];

		channel_num = ieee80211_mhz2ieee(c->ic_freq, 0);
		/* TODO IEEE80211_IS_CHAN_6GHZ -> chan->band = 3*/
		if (IEEE80211_IS_CHAN_2GHZ(c)) {
			chan->band = 1;
			n_2g++;
		} else {
			chan->band = 2;
			n_5g++;
		}
		chan->channel_num = channel_num;
		nchan++;
	}

	if (nchan <= 32) {
		req->channels_num = nchan;
	} else {
		req->channels_num = 32;
		req->ext_channels_num = nchan - 32;
	}

	req->channel_type = nchan ? 4 : 0;
	/*
	 * A background scan takes the radio off the BSS channel for the dwell
	 * time of every channel it visits, and macOS may ask for one every
	 * ten seconds while associated. Like Linux mt76 on these chips, pass
	 * 0 there and let the firmware use its own channel time; a scan with
	 * no association to keep alive still dwells 120 ms. The one channel of
	 * a join dwells 220 ms, two beacon intervals of 102.4 ms: the scan is
	 * passive, and a missed beacon costs a sweep of every channel.
	 */
	dwell = bgscan ? 0 : (join_channel != 0 && nchan == 1 ? 220 : 120);
	req->timeout_value = htole16(nchan * dwell);
	req->channel_min_dwell_time = htole16(dwell);
	req->channel_dwell_time = htole16(dwell);


#ifdef NOTYET
	if (sreq->ie_len > 0) {
		memcpy(req->ies, sreq->ie, sreq->ie_len);
		req->ies_len = htole16(sreq->ie_len);
	}
#endif

	req->scan_func |= SCAN_FUNC_SPLIT_SCAN;

	/* wildcard BSSID */
	memset(req->bssid, 0xff, ETHER_ADDR_LEN);
#ifdef NOTYET
	if (sreq->flags & NL80211_SCAN_FLAG_RANDOM_ADDR) {
		get_random_mask_addr(req->random_mac, sreq->mac_addr,
		    sreq->mac_addr_mask);
		req->scan_func |= SCAN_FUNC_RANDOM_MAC;
	}
#endif

	MWX_DEV_LOG(sc,
	    "scan request seq=%u channels=%u 2g=%u 5g=%u primary=%u ext=%u timeout=%u dwell=%u min_dwell=%u scan_func=0x%08x ssid_type=0x%x join_channel=%u\n",
	    sc->sc_scan_seq_num, nchan, n_2g, n_5g, req->channels_num,
	    req->ext_channels_num, le16toh(req->timeout_value),
	    le16toh(req->channel_dwell_time),
	    le16toh(req->channel_min_dwell_time),
	    le32toh(req->scan_func), req->ssid_type, join_channel);

	rv = mwx_mcu_send_mbuf(sc, MCU_CE_CMD_START_HW_SCAN, m, NULL);
	if (rv != 0)
		sc->sc_flags &= ~(MWX_FLAG_SCANNING | MWX_FLAG_BGSCAN);

	return rv;
}

int
mt7921_mcu_hw_scan_cancel(struct mwx_softc *sc)
{
	struct {
		uint8_t		seq_num;
		uint8_t		is_ext_channel;
		uint8_t		rsv[2];
	} __packed req = {
		.seq_num = sc->sc_scan_seq_num,
	};
	int rv;

	rv = mwx_mcu_send_msg(sc, MCU_CE_CMD_CANCEL_HW_SCAN, &req,
	    sizeof(req), NULL);
	if (rv == 0)
		sc->sc_flags &= ~(MWX_FLAG_SCANNING | MWX_FLAG_BGSCAN);
	return rv;
}

void
mwx_end_scan_task(void *arg)
{
	struct mwx_softc *sc = (struct mwx_softc *)arg;
	struct ieee80211com *ic = &sc->sc_ic;
	int s;

	s = splnet();
	/*
	 * Entering RUN cancels the hardware scan, but the firmware can still
	 * report its completion afterwards. Ending that stale scan made
	 * net80211 choose and re-join the BSS we had just associated to
	 * (without a deauth), which the AP then ignored. Only background
	 * scans may complete while associated.
	 */
	if (ic->ic_state == IEEE80211_S_RUN &&
	    (ic->ic_flags & IEEE80211_F_BGSCAN) == 0) {
		MWX_DEV_LOG(sc, "ignoring stale scan completion in RUN\n");
		splx(s);
		return;
	}
	ieee80211_end_scan(&ic->ic_if);
	splx(s);
}

void
mt7921_mcu_scan_event(struct mwx_softc *sc, struct mbuf *m)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct mt76_connac_hw_scan_done *event;
	uint32_t probe_sum = 0, mdrdy_sum = 0;
	int i, nodes_before;

	sc->sc_scan_done_count++;
	nodes_before = mwx_count_nodes(ic);
	sc->sc_scan_last_ieee80211_nodes = nodes_before;

	if (mbuf_len(m) >= sizeof(*event)) {
		event = mtod(m, struct mt76_connac_hw_scan_done *);
		for (i = 0; i < MT76_HW_SCAN_DONE_MAX_CHANNEL_NUM; i++) {
			probe_sum += event->beacon_probe_num[i];
			mdrdy_sum += event->mdrdy_count[i];
		}
		MWX_DEV_LOG(sc,
		    "scan done #%u seq=%u complete_ch=%u state=%u version=%u beacon_scan=%u probe_sum=%u mdrdy_sum=%u beacon_2g=%u beacon_5g=%u nodes_before=%d rx_norm=%u rx_fail=%u\n",
		    sc->sc_scan_done_count, event->seq_num,
		    event->complete_channel_num, event->current_state,
		    event->version, le32toh(event->beacon_scan_num),
		    probe_sum, mdrdy_sum, le32toh(event->beacon_2g_num),
		    le32toh(event->beacon_5g_num), nodes_before,
		    sc->sc_rx_pkt_normal, sc->sc_rx_fill_fail);
	} else {
		MWX_DEV_LOG(sc,
		    "scan done #%u short event len=%zu nodes_before=%d rx_norm=%u rx_fail=%u\n",
		    sc->sc_scan_done_count, mbuf_len(m), nodes_before,
		    sc->sc_rx_pkt_normal, sc->sc_rx_fill_fail);
	}

	if (mt7921_mcu_hw_scan_cancel(sc) != 0)
		return;
	task_add(systq, &sc->sc_scan_task);
}

int
mt7921_mcu_set_mac_enable(struct mwx_softc *sc, int band, int enable)
{
	struct {
		uint8_t		enable;
		uint8_t		band;
		uint8_t		rsv[2];
	} __packed req = {
		.enable = (uint8_t)enable,
		.band = (uint8_t)band,
	};

	return mwx_mcu_send_wait(sc, MCU_EXT_CMD_MAC_INIT_CTRL, &req,
	    sizeof(req));
}

int
mt7921_mcu_set_channel_domain(struct mwx_softc *sc)
{
	struct {
		uint8_t		alpha2[4]; /* regulatory_request.alpha2 */
		uint8_t		bw_2g;	/* BW_20_40M		0
					 * BW_20M		1
					 * BW_20_40_80M		2
					 * BW_20_40_80_160M	3
					 * BW_20_40_80_8080M	4
					 */
		uint8_t		bw_5g;
		uint8_t		bw_6g;
		uint8_t		pad;
		uint8_t		n_2ch;
		uint8_t		n_5ch;
		uint8_t		n_6ch;
		uint8_t		pad2;
	} __packed *hdr;
	struct {
		uint16_t	hw_value;
		uint16_t	pad;
		uint32_t	flags;
	} __packed *channel;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_channel *chan;
	struct mbuf *m;
	int i, len, rv;
	int n_2ch = 0, n_5ch = 0, n_6ch = 0;

	len = sizeof(*hdr) + IEEE80211_CHAN_MAX * sizeof(channel);
	m = mwx_mcu_alloc_msg(len);
	if (m == NULL)
		return ENOMEM;
	hdr = (decltype(hdr))mbuf_data(m);

	hdr->alpha2[0] = '0';
	hdr->alpha2[1] = '0';

	channel = (decltype(channel))(hdr + 1);

	hdr->bw_2g = 0;	/* BW_20_40M */
	for (i = 0; i <= IEEE80211_CHAN_MAX; i++) {
		chan = &ic->ic_channels[i];
		if (!IEEE80211_IS_CHAN_2GHZ(chan))
			continue;

		channel->hw_value = htole16(ieee80211_chan2ieee(ic, chan));
		channel->flags = htole32(0);	/* XXX */

		channel++;
		n_2ch++;
	}
	hdr->bw_5g = 3; /* BW_20_40_80_160M */
	for (i = 0; i <= IEEE80211_CHAN_MAX; i++) {
		chan = &ic->ic_channels[i];
		if (!IEEE80211_IS_CHAN_5GHZ(chan))
			continue;

		channel->hw_value = htole16(ieee80211_chan2ieee(ic, chan));
		channel->flags = htole32(0);	/* XXX */

		channel++;
		n_5ch++;
	}
#ifdef NOTYET
	/* 6GHz handling */
	hdr->bw_6g = 3; /* BW_20_40_80_160M */
	for (i = 0; i <= IEEE80211_CHAN_MAX; i++) {
		chan = &ic->ic_channels[i];
		if (!IEEE80211_IS_CHAN_6GHZ(chan))
			continue;

		channel->hw_value = htole16(ieee80211_chan2ieee(ic, chan));
		channel->flags = htole32(0);	/* XXX */

		channel++;
		n_6ch++;
	}
#endif

	memcpy(hdr->alpha2, sc->sc_alpha2, sizeof(sc->sc_alpha2));
	hdr->n_2ch = n_2ch;
	hdr->n_5ch = n_5ch;
	hdr->n_6ch = n_6ch;

	mwx_mcu_set_len(m, channel);
	rv = mwx_mcu_send_mbuf(sc, MCU_CE_CMD_SET_CHAN_DOMAIN, m, NULL);
	return rv;
}

uint8_t
mt7921_mcu_chan_bw(struct ieee80211_channel *channel)
{
	/*
	 * following modes are not yet supported
	 *  CMD_CBW_5MHZ, CMD_CBW_10MHZ, CMD_CBW_8080MHZ
	 */
	if (channel->ic_flags & IEEE80211_CHAN_VHT160)
		return CMD_CBW_160MHZ;
	if (channel->ic_flags & IEEE80211_CHAN_VHT80)
		return CMD_CBW_80MHZ;
	if (channel->ic_flags & IEEE80211_CHAN_40MHZ)
		return CMD_CBW_40MHZ;
	return CMD_CBW_20MHZ;
}

/*
 * Operating bandwidth and center channel negotiated with the AP: VHT80,
 * HT40 or 20 MHz.
 */
static void
mwx_node_chan_width(struct ieee80211_node *ni, uint8_t *bw, uint8_t *center)
{
	*bw = CMD_CBW_20MHZ;
	*center = ieee80211_mhz2ieee(ni->ni_chan->ic_freq, ni->ni_chan->ic_flags);
	if (ni->ni_chan->ic_center_freq1 == 0)
		return;
	if ((ni->ni_flags & IEEE80211_NODE_VHT) &&
	    ni->ni_chw == IEEE80211_CHAN_WIDTH_160) {
		*bw = CMD_CBW_160MHZ;
		*center = ieee80211_mhz2ieee(ni->ni_chan->ic_center_freq1,
		    ni->ni_chan->ic_flags);
	} else if ((ni->ni_flags & IEEE80211_NODE_VHT) &&
	    ni->ni_chw == IEEE80211_CHAN_WIDTH_80) {
		*bw = CMD_CBW_80MHZ;
		*center = ieee80211_mhz2ieee(ni->ni_chan->ic_center_freq1,
		    ni->ni_chan->ic_flags);
	} else if ((ni->ni_flags & IEEE80211_NODE_HT) &&
	    ni->ni_chw == IEEE80211_CHAN_WIDTH_40) {
		*bw = CMD_CBW_40MHZ;
		*center = ieee80211_mhz2ieee(ni->ni_chan->ic_center_freq1,
		    ni->ni_chan->ic_flags);
	}
}

int
mt7921_mcu_set_chan_info(struct mwx_softc *sc, int cmd)
{
	struct ieee80211_channel *channel;
	struct {
		uint8_t		control_ch;
		uint8_t		center_ch;
		uint8_t		bw;
		uint8_t		tx_streams_num;
		uint8_t		rx_streams;	/* mask or num */
		uint8_t		switch_reason;
		uint8_t		band_idx;
		uint8_t		center_ch2;	/* for 80+80 only */
		uint16_t	cac_case;
		uint8_t		channel_band;
		uint8_t		rsv0;
		uint32_t	outband_freq;
		uint8_t		txpower_drop;
		uint8_t		ap_bw;
		uint8_t		ap_center_ch;
		uint8_t		rsv1[57];
	} __packed req = {
		.tx_streams_num = sc->sc_capa.num_streams,
		.rx_streams = sc->sc_capa.antenna_mask,
		.band_idx = 0,	/* XXX 0 or 1 */
	};

	if (sc->sc_ic.ic_opmode == IEEE80211_M_STA && sc->sc_ic.ic_bss != NULL)
		channel = sc->sc_ic.ic_bss->ni_chan;
	else
		channel = sc->sc_ic.ic_ibss_chan;

	req.control_ch = ieee80211_mhz2ieee(channel->ic_freq,
		    channel->ic_flags);
	req.center_ch = req.control_ch;
	req.bw = CMD_CBW_20MHZ;
	/* Use the negotiated operating width, not the channel capabilities. */
	if (sc->sc_ic.ic_opmode == IEEE80211_M_STA && sc->sc_ic.ic_bss != NULL &&
	    channel == sc->sc_ic.ic_bss->ni_chan)
		mwx_node_chan_width(sc->sc_ic.ic_bss, &req.bw, &req.center_ch);

#ifdef NOTYET
	if (channel->ic_flags & IEEE80211_CHAN_6GHZ)
		req.channel_band = 2;
	else
#endif
	if (channel->ic_flags & IEEE80211_CHAN_5GHZ)
		req.channel_band = 1;
	else
		req.channel_band = 0;

#ifdef NOTYET
	if (dev->mt76.hw->conf.flags & IEEE80211_CONF_OFFCHANNEL)
		req.switch_reason = CH_SWITCH_SCAN_BYPASS_DPD;
	else if ((chandef->chan->flags & IEEE80211_CHAN_RADAR) &&
		 chandef->chan->dfs_state != NL80211_DFS_AVAILABLE)
		req.switch_reason = CH_SWITCH_DFS;
	else
#endif
		req.switch_reason = CH_SWITCH_NORMAL;

	if (cmd == MCU_EXT_CMD_CHANNEL_SWITCH)
		req.rx_streams = sc->sc_capa.num_streams;

#ifdef NOTYET
	if (chandef->width == NL80211_CHAN_WIDTH_80P80) {
		int freq2 = chandef->center_freq2;
		req.center_ch2 = ieee80211_frequency_to_channel(freq2);
	}
#endif

	return mwx_mcu_send_wait(sc, cmd, &req, sizeof(req));
}

/* hardcoded version of what linux does */
void
mt7921_mcu_build_sku(struct mwx_softc *sc, int band, int8_t *sku)
{
	int max_power = 127;
	int i, offset = 4;

	memset(sku, max_power, MT_SKU_POWER_LIMIT);

	if (band == MT_TX_PWR_BAND_2GHZ) {
		/* cck */
		memset(sku, 0x28, 4);
	}

	/* ofdm */
	memset(sku + offset, 0x28, 8);
	offset += 8;

	/* ht */
	for (i = 0; i < 2; i++) {
		memset(sku + offset, 0x28, 8);
		offset += 8;
	}
	sku[offset++] = 0x28;

	/* vht */
	for (i = 0; i < 4; i++) {
		/* this only sets 10 out of 12 bytes on purpose */
		memset(sku + offset, 0x28, 10);
		offset += 12;
	}

	/* he */
	for (i = 0; i < 7; i++) {
		memset(sku + offset, 0x28, 12);
		offset += 12;
	}
}

int
mt7921_mcu_rate_txpower_band(struct mwx_softc *sc, int band,
    const uint8_t *chans, int n_chans, int is_last)
{
	struct mt76_connac_sku_tlv *sku_tlv;
	struct mt76_connac_tx_power_limit_tlv *tx_power_tlv;
	struct mbuf *m;
	int batch_size = 8;
	const int len = sizeof(*tx_power_tlv) + batch_size * sizeof(*sku_tlv);
	int rv = 0, idx, j;

	for (idx = 0; idx < n_chans; ) {
		int num_ch = batch_size;

		m = mwx_mcu_alloc_msg(len);
		if (m == NULL)
			return ENOMEM;
		tx_power_tlv = mtod(m, struct mt76_connac_tx_power_limit_tlv *);
		tx_power_tlv->n_chan = num_ch;
		tx_power_tlv->band = band;
		memcpy(tx_power_tlv->alpha2, sc->sc_alpha2,
		    sizeof(sc->sc_alpha2));

		if (n_chans - idx < batch_size) {
			num_ch = n_chans - idx;
			if (is_last)
				tx_power_tlv->last_msg = 1;
		}

		sku_tlv = (struct mt76_connac_sku_tlv *)(tx_power_tlv + 1);

		for (j = 0; j < num_ch; j++, idx++) {
			sku_tlv->channel = chans[idx];
			mt7921_mcu_build_sku(sc, band, sku_tlv->pwr_limit);
			sku_tlv++;
		}

		mwx_mcu_set_len(m, sku_tlv);
		rv = mwx_mcu_send_mbuf(sc, MCU_CE_CMD_SET_RATE_TX_POWER, m,
		    NULL);
		if (rv != 0)
			break;
	}

	return rv;
}

int
mt7921_mcu_set_rate_txpower(struct mwx_softc *sc)
{
	static const uint8_t chan_list_2ghz[] = {
	    1, 2,  3,  4,  5,  6,  7,
	    8, 9, 10, 11, 12, 13, 14
	};
	static const uint8_t chan_list_5ghz[] = {
	    36,  38,  40,  42,  44,  46,  48,
	    50,  52,  54,  56,  58,  60,  62,
	    64, 100, 102, 104, 106, 108, 110,
	    112, 114, 116, 118, 120, 122, 124,
	    126, 128, 132, 134, 136, 138, 140,
	    142, 144, 149, 151, 153, 155, 157,
	    159, 161, 165
	};
	static const uint8_t chan_list_6ghz[] = {
	    1,   3,   5,   7,   9,  11,  13,
	    15,  17,  19,  21,  23,  25,  27,
	    29,  33,  35,  37,  39,  41,  43,
	    45,  47,  49,  51,  53,  55,  57,
	    59,  61,  65,  67,  69,  71,  73,
	    75,  77,  79,  81,  83,  85,  87,
	    89,  91,  93,  97,  99, 101, 103,
	    105, 107, 109, 111, 113, 115, 117,
	    119, 121, 123, 125, 129, 131, 133,
	    135, 137, 139, 141, 143, 145, 147,
	    149, 151, 153, 155, 157, 161, 163,
	    165, 167, 169, 171, 173, 175, 177,
	    179, 181, 183, 185, 187, 189, 193,
	    195, 197, 199, 201, 203, 205, 207,
	    209, 211, 213, 215, 217, 219, 221,
	    225, 227, 229, 233
	};
	int rv = 0;

	if (sc->sc_capa.has_2ghz)
		rv = mt7921_mcu_rate_txpower_band(sc, MT_TX_PWR_BAND_2GHZ,
		    chan_list_2ghz, nitems(chan_list_2ghz),
		    !(sc->sc_capa.has_5ghz || sc->sc_capa.has_6ghz));
	if (rv == 0 && sc->sc_capa.has_5ghz)
		rv = mt7921_mcu_rate_txpower_band(sc, MT_TX_PWR_BAND_5GHZ,
		    chan_list_5ghz, nitems(chan_list_5ghz),
		    !sc->sc_capa.has_6ghz);
	if (rv == 0 && sc->sc_capa.has_6ghz)
		rv = mt7921_mcu_rate_txpower_band(sc, MT_TX_PWR_BAND_6GHZ,
		    chan_list_6ghz, nitems(chan_list_6ghz), 1);
	return rv;
}

void
mt7921_mac_reset_counters(struct mwx_softc *sc)
{
	int i;

	for (i = 0; i < 4; i++) {
		mwx_read(sc, MT_TX_AGG_CNT(0, i));
		mwx_read(sc, MT_TX_AGG_CNT2(0, i));
	}

	/* XXX TODO stats in softc */

	/* reset airtime counters */
	mwx_read(sc, MT_MIB_SDR9(0));
	mwx_read(sc, MT_MIB_SDR36(0));
	mwx_read(sc, MT_MIB_SDR37(0));
	mwx_set(sc, MT_WF_RMAC_MIB_TIME0(0), MT_WF_RMAC_MIB_RXTIME_CLR);
	mwx_set(sc, MT_WF_RMAC_MIB_AIRTIME0(0), MT_WF_RMAC_MIB_RXTIME_CLR);
}

void
mt7921_mac_set_timing(struct mwx_softc *sc)
{
	uint16_t coverage_class = 0;		/* XXX */
	uint32_t val, reg_offset;
	uint32_t cck = MT_TIMEOUT_CCK_DEF_VAL;
	uint32_t ofdm = MT_TIMEOUT_OFDM_DEF_VAL;
	uint32_t offset;
	int is_2ghz = 1; /* XXX get from ic_bss node */
	uint32_t sifs = is_2ghz ? 10 : 16;
	uint32_t slottime = IEEE80211_DUR_DS_SHSLOT;	/* XXX get from stack */

#ifdef NOTYET
	if (!test_bit(MT76_STATE_RUNNING, &phy->mt76->state))
		return;
#endif

	mwx_set(sc, MT_ARB_SCR(0),
	    MT_ARB_SCR_TX_DISABLE | MT_ARB_SCR_RX_DISABLE);
	delay(1);

	offset = 3 * coverage_class;
	reg_offset = offset | (offset << 16);

	mwx_write(sc, MT_TMAC_CDTR(0), cck + reg_offset);
	mwx_write(sc, MT_TMAC_ODTR(0), ofdm + reg_offset);
	mwx_write(sc, MT_TMAC_ICR0(0),
	    MT_IFS_EIFS_DEF | MT_IFS_RIFS_DEF |
	    (sifs << MT_IFS_SIFS_SHIFT) |
	    (slottime << MT_IFS_SLOT_SHIFT));

	if (slottime < 20 || !is_2ghz)
		val = MT7921_CFEND_RATE_DEFAULT;
	else
		val = MT7921_CFEND_RATE_11B;

	mwx_rmw(sc, MT_AGG_ACR0(0), val, MT_AGG_ACR_CFEND_RATE_MASK);
	mwx_clear(sc, MT_ARB_SCR(0),
	    MT_ARB_SCR_TX_DISABLE | MT_ARB_SCR_RX_DISABLE);
}

int
mt7921_mcu_uni_add_dev(struct mwx_softc *sc, struct mwx_vif *mvif,
    struct mwx_node *mn, int enable)
{
	struct {
		struct {
			uint8_t		omac_idx;
			uint8_t		band_idx;
			uint16_t	pad;
		} __packed hdr;
		struct req_tlv {
			uint16_t	tag;
			uint16_t	len;
			uint8_t		active;
			uint8_t		pad;
			uint8_t		omac_addr[ETHER_ADDR_LEN];
		} __packed tlv;
	} dev_req = {
		.hdr = {
			.omac_idx = mvif->omac_idx,
			.band_idx = mvif->band_idx,
		},
		.tlv = {
			.tag = htole16(DEV_INFO_ACTIVE),
			.len = htole16(sizeof(dev_req.tlv)),
			.active = (uint8_t)enable,
		},
	};
	struct {
		struct {
			uint8_t		bss_idx;
			uint8_t		pad[3];
		} __packed hdr;
		struct mt76_connac_bss_basic_tlv basic;
	} basic_req = {
		.hdr = {
			.bss_idx = mvif->idx,
		},
		.basic = {
			.tag = htole16(UNI_BSS_INFO_BASIC),
			.len = htole16(
			    sizeof(struct mt76_connac_bss_basic_tlv)),
			.omac_idx = mvif->omac_idx,
			.band_idx = mvif->band_idx,
			.wmm_idx = mvif->wmm_idx,
			.active = (uint8_t)enable,
			.bmc_tx_wlan_idx = htole16(mn->wcid),
			.sta_idx = htole16(mn->wcid),
			.conn_state = 1,
		},
	};
	int rv, idx, cmd, len;
	void *data;

	switch (sc->sc_ic.ic_opmode) {
	case IEEE80211_M_MONITOR:
#ifndef IEEE80211_STA_ONLY
	case IEEE80211_M_HOSTAP:
#endif
		basic_req.basic.conn_type =
		    htole32(STA_TYPE_AP | NETWORK_INFRA);
		break;
	case IEEE80211_M_STA:
		basic_req.basic.conn_type =
		    htole32(STA_TYPE_STA | NETWORK_INFRA);
		break;
#ifndef IEEE80211_STA_ONLY
	case IEEE80211_M_IBSS:
		basic_req.basic.conn_type =
		    htole32(STA_TYPE_ADHOC | NETWORK_IBSS);
		break;
#endif
	default:
		panic("%s: unknown operation mode", DEVNAME(sc));
	}

	idx = mvif->omac_idx > EXT_BSSID_START ? HW_BSSID_0 : mvif->omac_idx;
	basic_req.basic.hw_bss_idx = idx;

	/* The address the vif answers to: the current one, not the factory one. */
	memcpy(dev_req.tlv.omac_addr,
	    IEEE80211_ADDR_EQ(sc->sc_ic.ic_myaddr, etheranyaddr) ?
	    sc->sc_lladdr : sc->sc_ic.ic_myaddr, ETHER_ADDR_LEN);

	if (enable) {
		cmd = MCU_UNI_CMD_DEV_INFO_UPDATE;
		data = &dev_req;
		len = sizeof(dev_req);
	} else {
		cmd = MCU_UNI_CMD_BSS_INFO_UPDATE;
		data = &basic_req;
		len = sizeof(basic_req);
	}

printf("%s: %s cmd %x mvif idx %d omac %d band %d wmm %d\n", DEVNAME(sc), __func__, cmd, mvif->idx, mvif->omac_idx, mvif->band_idx, mvif->wmm_idx);
	rv = mwx_mcu_send_wait(sc, cmd, data, len);
	if (rv < 0)
		return rv;

	if (enable) {
		cmd = MCU_UNI_CMD_BSS_INFO_UPDATE;
		data = &basic_req;
		len = sizeof(basic_req);
	} else {
		cmd = MCU_UNI_CMD_DEV_INFO_UPDATE;
		data = &dev_req;
		len = sizeof(dev_req);
	}

printf("%s: %s cmd %x wcid %d\n", DEVNAME(sc), __func__, cmd, mn->wcid);
	return mwx_mcu_send_wait(sc, cmd, data, len);
}

int
mt7921_mcu_set_sniffer(struct mwx_softc *sc, int enable)
{
	struct {
		uint8_t		band_idx;
		uint8_t		pad[3];
		struct sniffer_enable_tlv {
			uint16_t	tag;
			uint16_t	len;
			uint8_t		enable;
			uint8_t		pad[3];
		}		enable;
	} req = {
		.band_idx = 0,
		.enable = {
			.tag = htole16(0),
			.len = htole16(sizeof(req.enable)),
			.enable = (uint8_t)enable,
		},
	};

	return mwx_mcu_send_wait(sc, MCU_UNI_CMD_SNIFFER, &req, sizeof(req));
}

int
mt7921_mcu_set_beacon_filter(struct mwx_softc *sc, int enable)
{
	int rv;

	if (enable) {
#ifdef NOTYET
		rv = mt7921_mcu_uni_bss_bcnft(dev, vif, true);
		if (rv)
			return rv;
#endif
		mwx_set(sc, MT_WF_RFCR(0), MT_WF_RFCR_DROP_OTHER_BEACON);
	} else {
		rv = mt7921_mcu_set_bss_pm(sc, 0);
		if (rv)
			return rv;
		mwx_clear(sc, MT_WF_RFCR(0), MT_WF_RFCR_DROP_OTHER_BEACON);
	}
	return 0;
}

int
mt7921_mcu_set_bss_pm(struct mwx_softc *sc, int enable)
{
#ifdef NOTYET
	struct {
		uint8_t		bss_idx;
		uint8_t		dtim_period;
		uint16_t	aid;
		uint16_t	bcn_interval;
		uint16_t	atim_window;
		uint8_t		uapsd;
		uint8_t		bmc_delivered_ac;
		uint8_t		bmc_triggered_ac;
		uint8_t		pad;
	} req = {
		.bss_idx = mvif->mt76.idx,
		.aid = htole16(vif->cfg.aid),
		.dtim_period = vif->bss_conf.dtim_period,
		.bcn_interval = htole16(vif->bss_conf.beacon_int),
	};
#endif
	struct {
		uint8_t		bss_idx;
		uint8_t		pad[3];
	} req_hdr = {
		.bss_idx = /* mvif->mt76.idx XXX */ 0,
	};
	int rv;

	rv = mwx_mcu_send_msg(sc, MCU_CE_CMD_SET_BSS_ABORT,
		&req_hdr, sizeof(req_hdr), NULL);
#ifdef NOTYET
	if (rv != 0 || !enable)
		return rv;
	rv = mwx_mcu_send_msg(sc, MCU_CE_CMD_SET_BSS_CONNECTED,
		&req, sizeof(req), NULL);
#endif
	return rv;
}

/* Association-response parsing runs before NODE_QOS is set; RUN applies
 * the first parameter set explicitly. Later beacon updates can submit here. */
void
mwx_updateedca(struct ieee80211com *ic)
{
	struct mwx_softc *sc = (struct mwx_softc *)ic->ic_softc;

	if (sc == NULL || sc->sc_suspended ||
	    !(ic->ic_if.if_flags & IFF_RUNNING) ||
	    ic->ic_state != IEEE80211_S_RUN || !sc->sc_sta_added)
		return;

	int rv = mt7921_mcu_set_tx(sc, &sc->sc_vif);
	if (rv) {
		/* Retry even if the next beacon has the same update count. */
		ic->ic_edca_updtcount = (u_int)-1;
		MWX_DEV_LOG(sc, "EDCA update failed rv=%d; retry next beacon\n", rv);
	}
}

#define IEEE80211_NUM_ACS	4
int
mt7921_mcu_set_tx(struct mwx_softc *sc, struct mwx_vif *mvif)
{
	struct ieee80211com *ic = &sc->sc_ic;
	const int qos = (ic->ic_flags & IEEE80211_F_QOS) &&
	    sc->sc_sta_added && ic->ic_bss != NULL &&
	    IEEE80211_AID(ic->ic_bss->ni_associd) != 0 &&
	    (ic->ic_bss->ni_flags & IEEE80211_NODE_QOS) &&
	    (ic->ic_state == IEEE80211_S_ASSOC ||
	    ic->ic_state == IEEE80211_S_RUN);
	struct edca {
		uint16_t	cw_min;
		uint16_t	cw_max;
		uint16_t	txop;
		uint16_t	aifs;
		uint8_t		guardtime;
		uint8_t		acm;
	} __packed;
	struct mt7921_mcu_tx {
		struct edca	edca[IEEE80211_NUM_ACS];
		uint8_t		bss_idx;
		uint8_t		qos;
		uint8_t		wmm_idx;
		uint8_t		pad;
	} __packed req = {
		.bss_idx = mvif->idx,
		.qos = !!qos,
		.wmm_idx = mvif->wmm_idx,
	};
#ifdef NOTYET
	struct mu_edca {
		uint8_t		cw_min;
		uint8_t		cw_max;
		uint8_t		aifsn;
		uint8_t		acm;
		uint8_t		timer;
		uint8_t		padding[3];
	};
	struct mt7921_mcu_mu_tx {
		uint8_t		ver;
		uint8_t		pad0;
		uint16_t	len;
		uint8_t		bss_idx;
		uint8_t		qos;
		uint8_t		wmm_idx;
		uint8_t		pad1;
		struct mu_edca	edca[IEEE80211_NUM_ACS];
		uint8_t		pad3[32];
	} __packed req_mu = {
		.bss_idx = mvif->mt76.idx,
		.qos = vif->bss_conf.qos,
		.wmm_idx = mvif->mt76.wmm_idx,
	};
#endif
	/* net80211 BE/BK/VI/VO -> firmware BE/BK/VI/VO slots.
	 * Linux's {1,0,2,3} map starts from its different VO/VI/BE/BK order. */
	static const int to_aci[EDCA_NUM_AC] = { 2, 3, 0, 1 };
	int ac, rv;

	for (ac = 0; ac < EDCA_NUM_AC; ac++) {
		const struct ieee80211_edca_ac_params *q = &ic->ic_edca_ac[ac];
		struct edca *e = &req.edca[to_aci[ac]];

		/* Keep the existing non-QoS initialization defaults. */
		e->aifs = htole16(2);
		e->cw_min = htole16(5);
		e->cw_max = htole16(10);
		if (!qos)
			continue;
		if (q->ac_aifsn == 0 || q->ac_aifsn > 15 ||
		    q->ac_ecwmin > 15 || q->ac_ecwmax > 15 ||
		    q->ac_ecwmin > q->ac_ecwmax)
			return EINVAL;

		e->aifs = htole16(q->ac_aifsn);
		e->txop = htole16(q->ac_txoplimit); /* 32 us units */
		/* SET_EDCA_PARMS takes actual CW values, not ECW exponents. */
		e->cw_min = htole16((1U << q->ac_ecwmin) - 1);
		e->cw_max = htole16((1U << q->ac_ecwmax) - 1);
		e->acm = q->ac_acm & 1;
		MWX_DEV_LOG(sc, "EDCA ac=%d slot=%d qos=1 aifs=%u cw=%u/%u txop=%u acm=%u\n",
		    ac, to_aci[ac], q->ac_aifsn,
		    (1U << q->ac_ecwmin) - 1, (1U << q->ac_ecwmax) - 1,
		    q->ac_txoplimit, e->acm);
	}

	rv = mwx_mcu_send_msg(sc, MCU_CE_CMD_SET_EDCA_PARMS, &req,
	    sizeof(req), NULL);

#ifdef NOTYET
	if (rv)
		return rv;
	if (!vif->bss_conf.he_support)
		return 0;

	for (ac = 0; ac < IEEE80211_NUM_ACS; ac++) {
		struct ieee80211_he_mu_edca_param_ac_rec *q;
		struct mu_edca *e;

		if (!mvif->queue_params[ac].mu_edca)
			break;

		q = &mvif->queue_params[ac].mu_edca_param_rec;
		e = &(req_mu.edca[to_aci[ac]]);

		e->cw_min = q->ecw_min_max & 0xf;
		e->cw_max = (q->ecw_min_max & 0xf0) >> 4;
		e->aifsn = q->aifsn;
		e->timer = q->mu_edca_timer;
	}

	rv = mt76_mcu_send_msg(&dev->mt76, MCU_CE_CMD(SET_MU_EDCA_PARMS),
	    &req_mu, sizeof(req_mu), false);
#endif
	return rv;
}

int
mt7921_mac_fill_rx(struct mwx_softc *sc, struct mbuf *m,
    struct ieee80211_rxinfo *rxi, struct mwx_rx_sec *sec)
{
	struct ieee80211com *ic = &sc->sc_ic;
	uint32_t *rxd, *rxv = NULL;
	uint32_t rxd0, rxd1, rxd2, rxd3, rxd4;
//	uint32_t mode = 0;
	uint16_t hdr_gap /*, seq_ctrl = 0, fc = 0 */;
	uint8_t chfnum, remove_pad /*, qos_ctl = 0, amsdu_info */;
	int idx, unicast, num_rxd = 6;
	uint32_t ampdu_timestamp = 0;
	int i;
//	bool insert_ccmp_hdr = false;

	memset(sec, 0, sizeof(*sec));
	if (mbuf_len(m) < num_rxd * sizeof(uint32_t)) {
		if (sc->sc_rx_fill_fail < 64) {
			MWX_DEV_LOG(sc,
			    "rx drop reason=short-rxd len=%zu need=%zu\n",
			    mbuf_len(m), num_rxd * sizeof(uint32_t));
		}
		return -1;
	}

	rxd = mtod(m, uint32_t *);
	rxd0 = le32toh(rxd[0]);
	rxd1 = le32toh(rxd[1]);
	rxd2 = le32toh(rxd[2]);
	rxd3 = le32toh(rxd[3]);
	rxd4 = le32toh(rxd[4]);

	/* Validate the entire optional descriptor before reading RXV words. The
	 * later length check is too late for GROUP_3/GROUP_5's direct loads. */
	{
		size_t words = 6;
		if (rxd1 & MT_RXD1_NORMAL_GROUP_4)
			words += 4;
		if (rxd1 & MT_RXD1_NORMAL_GROUP_1)
			words += 4;
		if (rxd1 & MT_RXD1_NORMAL_GROUP_2)
			words += 2;
		if (rxd1 & MT_RXD1_NORMAL_GROUP_3) {
			words += 2;
			if (rxd1 & MT_RXD1_NORMAL_GROUP_5)
				words += 18;
		}
		if (mbuf_len(m) < words * sizeof(uint32_t))
			return -1;
	}

	if (rxd1 & MT_RXD1_NORMAL_BAND_IDX) {
		MWX_RX_DROP_LOG(sc, "band-idx", -1, rxd0, rxd1, rxd2,
		    rxd3, rxd4);
		return -1;
	}

	if (rxd2 & MT_RXD2_NORMAL_AMSDU_ERR) {
		MWX_RX_DROP_LOG(sc, "amsdu-err", -1, rxd0, rxd1, rxd2,
		    rxd3, rxd4);
		return -1;
	}

	if (rxd2 & MT_RXD2_NORMAL_HDR_TRANS) {
		MWX_RX_DROP_LOG(sc, "hdr-trans", -1, rxd0, rxd1, rxd2,
		    rxd3, rxd4);
		return -1;
	}

	/* ICV error or CCMP/BIP/WPI MIC error */
	if (rxd1 & MT_RXD1_NORMAL_ICV_ERR) {
		MWX_RX_DROP_LOG(sc, "icv", -1, rxd0, rxd1, rxd2, rxd3,
		    rxd4);
		ic->ic_stats.is_rx_decryptcrc++;
		return -1;
	}

	if (rxd1 & MT_RXD1_NORMAL_FCS_ERR) {
		MWX_RX_DROP_LOG(sc, "fcs", -1, rxd0, rxd1, rxd2, rxd3,
		    rxd4);
		return -1;
	}

	if (rxd1 & MT_RXD1_NORMAL_TKIP_MIC_ERR) {
		/* report MIC failures to net80211 for TKIP */
		MWX_RX_DROP_LOG(sc, "tkip-mic", -1, rxd0, rxd1, rxd2,
		    rxd3, rxd4);
		ic->ic_stats.is_rx_locmicfail++;
		ieee80211_michael_mic_failure(ic, 0/* XXX */);
		return -1;
	}


	chfnum = (rxd3 & MT_RXD3_NORMAL_CH_NUM_MASK) >>
	    MT_RXD3_NORMAL_CH_NUM_SHIFT;
	unicast = (rxd3 & MT_RXD3_NORMAL_ADDR_TYPE_MASK) == MT_RXD3_NORMAL_U2M;
	idx = rxd1 & MT_RXD1_NORMAL_WLAN_IDX_MASK;

#if 0
	status->wcid = mt7921_rx_get_wcid(dev, idx, unicast);
	if (status->wcid) {
		struct mt7921_sta *msta;

		msta = container_of(status->wcid, struct mt7921_sta, wcid);
		spin_lock_bh(&dev->sta_poll_lock);
		if (list_empty(&msta->poll_list))
			list_add_tail(&msta->poll_list, &dev->sta_poll_list);
		spin_unlock_bh(&dev->sta_poll_lock);
	}
#endif

#if NOTYET
	if ((rxd0 & (MT_RXD0_NORMAL_IP_SUM | MT_RXD0_NORMAL_UDP_TCP_SUM)) ==
	    (MT_RXD0_NORMAL_IP_SUM | MT_RXD0_NORMAL_UDP_TCP_SUM)) {
		m->m_pkthdr.csum_flags = M_IPV4_CSUM_IN_OK |
		    M_TCP_CSUM_IN_OK | M_UDP_CSUM_IN_OK;
	}
#endif

	sec->sec_mode = (rxd1 & MT_RXD1_NORMAL_SEC_MODE_MASK) >>
	    MT_RXD1_NORMAL_SEC_MODE_SHIFT;
	sec->key_id = (rxd1 & MT_RXD1_NORMAL_KEY_ID_MASK) >>
	    MT_RXD1_NORMAL_KEY_ID_SHIFT;
	if ((rxd1 & MT_RXD1_NORMAL_SEC_MODE_MASK) != 0 &&
	    !(rxd1 & (MT_RXD1_NORMAL_CLM | MT_RXD1_NORMAL_CM))) {
		rxi->rxi_flags |= IEEE80211_RXI_HWDEC;
	} else if (ic->ic_state == IEEE80211_S_RUN) {
		/* Diagnose why the chip did not decrypt this frame. */
		static uint32_t nodec_log_count;
		if (nodec_log_count < 24) {
			MWX_DEV_LOG(sc,
			    "rx no-hwdec wlan_idx=%u sec_mode=%u cm=%d clm=%d sec_done=%d unicast=%d rxd=%08x/%08x/%08x/%08x\n",
			    rxd1 & MT_RXD1_NORMAL_WLAN_IDX_MASK,
			    (rxd1 & MT_RXD1_NORMAL_SEC_MODE_MASK) >> 16,
			    !!(rxd1 & MT_RXD1_NORMAL_CM),
			    !!(rxd1 & MT_RXD1_NORMAL_CLM),
			    !!(rxd1 & MT_RXD1_NORMAL_SEC_DONE),
			    (rxd3 & MT_RXD3_NORMAL_ADDR_TYPE_MASK) ==
			    MT_RXD3_NORMAL_U2M, rxd0, rxd1, rxd2, rxd3);
			nodec_log_count++;
		}
	}

	remove_pad = (rxd2 & MT_RXD2_NORMAL_HDR_OFFSET_MASK) >>
	    MT_RXD2_NORMAL_HDR_OFFSET_SHIFT;

	if (rxd2 & MT_RXD2_NORMAL_MAX_LEN_ERROR) {
		MWX_RX_DROP_LOG(sc, "max-len", -EINVAL, rxd0, rxd1, rxd2,
		    rxd3, rxd4);
		return -EINVAL;
	}

	rxd += 6;

	if (rxd1 & MT_RXD1_NORMAL_GROUP_4) {
		num_rxd += 4;
		rxd += 4;
	}
	if (rxd1 & MT_RXD1_NORMAL_GROUP_1) {
		/*
		 * The first six bytes hold the packet number of a decrypted
		 * frame, PN0 first, as Linux mt7921_mac_fill_rx() reads it.
		 */
		if (rxi->rxi_flags & IEEE80211_RXI_HWDEC) {
			memcpy(sec->pn, rxd, sizeof(sec->pn));
			sec->pn_valid = 1;
		}
		num_rxd += 4;
		rxd += 4;
	}
	if (rxd1 & MT_RXD1_NORMAL_GROUP_2) {
		ampdu_timestamp = le32toh(rxd[0]);
		num_rxd += 2;
		rxd += 2;
	}
	if (rxd1 & MT_RXD1_NORMAL_GROUP_3) {	
		uint32_t v0, v1;
		int8_t chain[4], signal;

		rxv = rxd;
		num_rxd += 2;
		rxd += 2;

		v0 = le32toh(rxv[0]); /* XXX not implemented yet */ 
		v1 = le32toh(rxv[1]);
		if ((rxd3 & MT_RXD3_NORMAL_ADDR_TYPE_MASK) == MT_RXD3_NORMAL_U2M)
			mwx_rx_rate_account(v0);

		if (rxd1 & MT_RXD1_NORMAL_GROUP_5) {
			num_rxd += 18;
			rxd += 6;
			rxv = rxd;
			v1 = le32toh(rxv[0]); 
			rxd += 12;
		}
	

		chain[0] = rcpi_to_rssi(MT_PRXV_RCPI0_SHIFT, v1);
		chain[1] = rcpi_to_rssi(MT_PRXV_RCPI1_SHIFT, v1);
		chain[2] = rcpi_to_rssi(MT_PRXV_RCPI2_SHIFT, v1);
		chain[3] = rcpi_to_rssi(MT_PRXV_RCPI3_SHIFT, v1);

		signal = -128;
		for (i = 0; i < sc->sc_capa.num_streams; i++) {
			if (!(sc->sc_capa.antenna_mask & (1U << i)))
				continue;	
			if (chain[i] >= 0)
				continue;
			signal = MAX(signal, chain[i]);
		}
		rxi->rxi_rssi = signal;
		}

	if (mbuf_len(m) < num_rxd * sizeof(uint32_t)) {
		if (sc->sc_rx_fill_fail < 64) {
			MWX_DEV_LOG(sc,
			    "rx drop reason=short-groups len=%zu need=%zu rxd=%08x/%08x/%08x/%08x/%08x\n",
			    mbuf_len(m), num_rxd * sizeof(uint32_t),
			    rxd0, rxd1, rxd2, rxd3, rxd4);
		}
		return -1;
	}

#if 0
	if (rxd1 & MT_RXD1_NORMAL_GROUP_4) {
		uint32_t v0 = le32toh(rxd[0]);
		uint32_t v2 = le32toh(rxd[2]);

		fc = htole16(FIELD_GET(MT_RXD6_FRAME_CONTROL, v0));
		seq_ctrl = FIELD_GET(MT_RXD8_SEQ_CTRL, v2);
		qos_ctl = FIELD_GET(MT_RXD8_QOS_CTL, v2);

		rxd += 4;
	}

	if (rxd1 & MT_RXD1_NORMAL_GROUP_1) {
		u8 *data = (u8 *)rxd;

		if (status->flag & RX_FLAG_DECRYPTED) {
			switch (FIELD_GET(MT_RXD1_NORMAL_SEC_MODE, rxd1)) {
			case MT_CIPHER_AES_CCMP:
			case MT_CIPHER_CCMP_CCX:
			case MT_CIPHER_CCMP_256:
				insert_ccmp_hdr =
					(rxd2 & MT_RXD2_NORMAL_FRAG);
				/* FALLTHROUGH */
			case MT_CIPHER_TKIP:
			case MT_CIPHER_TKIP_NO_MIC:
			case MT_CIPHER_GCMP:
			case MT_CIPHER_GCMP_256:
				status->iv[0] = data[5];
				status->iv[1] = data[4];
				status->iv[2] = data[3];
				status->iv[3] = data[2];
				status->iv[4] = data[1];
				status->iv[5] = data[0];
				break;
			default:
				break;
			}
		}
		rxd += 4;
	}

	if (rxd1 & MT_RXD1_NORMAL_GROUP_2) {
		status->timestamp = le32_to_cpu(rxd[0]);
		status->flag |= RX_FLAG_MACTIME_START;

		if (!(rxd2 & MT_RXD2_NORMAL_NON_AMPDU)) {
			status->flag |= RX_FLAG_AMPDU_DETAILS;

			/* all subframes of an A-MPDU have the same timestamp */
			if (phy->rx_ampdu_ts != status->timestamp) {
				if (!++phy->ampdu_ref)
					phy->ampdu_ref++;
			}
			phy->rx_ampdu_ts = status->timestamp;

			status->ampdu_ref = phy->ampdu_ref;
		}

		rxd += 2;
	}

	/* RXD Group 3 - P-RXV */
	if (rxd1 & MT_RXD1_NORMAL_GROUP_3) {
		u8 stbc, gi;
		u32 v0, v1;
		bool cck;

		rxv = rxd;
		rxd += 2;

		v0 = le32_to_cpu(rxv[0]);
		v1 = le32_to_cpu(rxv[1]);

		if (v0 & MT_PRXV_HT_AD_CODE)
			status->enc_flags |= RX_ENC_FLAG_LDPC;

		status->chains = mphy->antenna_mask;
		status->chain_signal[0] = to_rssi(MT_PRXV_RCPI0, v1);
		status->chain_signal[1] = to_rssi(MT_PRXV_RCPI1, v1);
		status->chain_signal[2] = to_rssi(MT_PRXV_RCPI2, v1);
		status->chain_signal[3] = to_rssi(MT_PRXV_RCPI3, v1);
		status->signal = -128;
		for (i = 0; i < hweight8(mphy->antenna_mask); i++) {
			if (!(status->chains & BIT(i)) ||
			    status->chain_signal[i] >= 0)
				continue;

			status->signal = max(status->signal,
					     status->chain_signal[i]);
		}

		stbc = FIELD_GET(MT_PRXV_STBC, v0);
		gi = FIELD_GET(MT_PRXV_SGI, v0);
		cck = false;

		idx = i = FIELD_GET(MT_PRXV_TX_RATE, v0);
		mode = FIELD_GET(MT_PRXV_TX_MODE, v0);

		switch (mode) {
		case MT_PHY_TYPE_CCK:
			cck = true;
			fallthrough;
		case MT_PHY_TYPE_OFDM:
			i = mt76_get_rate(&dev->mt76, sband, i, cck);
			break;
		case MT_PHY_TYPE_HT_GF:
		case MT_PHY_TYPE_HT:
			status->encoding = RX_ENC_HT;
			if (i > 31)
				return -EINVAL;
			break;
		case MT_PHY_TYPE_VHT:
			status->nss =
				FIELD_GET(MT_PRXV_NSTS, v0) + 1;
			status->encoding = RX_ENC_VHT;
			if (i > 9)
				return -EINVAL;
			break;
		case MT_PHY_TYPE_HE_MU:
		case MT_PHY_TYPE_HE_SU:
		case MT_PHY_TYPE_HE_EXT_SU:
		case MT_PHY_TYPE_HE_TB:
			status->nss =
				FIELD_GET(MT_PRXV_NSTS, v0) + 1;
			status->encoding = RX_ENC_HE;
			i &= GENMASK(3, 0);

			if (gi <= NL80211_RATE_INFO_HE_GI_3_2)
				status->he_gi = gi;

			status->he_dcm = !!(idx & MT_PRXV_TX_DCM);
			break;
		default:
			return -EINVAL;
		}

		status->rate_idx = i;

		switch (FIELD_GET(MT_PRXV_FRAME_MODE, v0)) {
		case IEEE80211_STA_RX_BW_20:
			break;
		case IEEE80211_STA_RX_BW_40:
			if (mode & MT_PHY_TYPE_HE_EXT_SU &&
			    (idx & MT_PRXV_TX_ER_SU_106T)) {
				status->bw = RATE_INFO_BW_HE_RU;
				status->he_ru =
					NL80211_RATE_INFO_HE_RU_ALLOC_106;
			} else {
				status->bw = RATE_INFO_BW_40;
			}
			break;
		case IEEE80211_STA_RX_BW_80:
			status->bw = RATE_INFO_BW_80;
			break;
		case IEEE80211_STA_RX_BW_160:
			status->bw = RATE_INFO_BW_160;
			break;
		default:
			return -EINVAL;
		}

		status->enc_flags |= RX_ENC_FLAG_STBC_MASK * stbc;
		if (mode < MT_PHY_TYPE_HE_SU && gi)
			status->enc_flags |= RX_ENC_FLAG_SHORT_GI;

		if (rxd1 & MT_RXD1_NORMAL_GROUP_5) {
			rxd += 18;
		}
	}

	amsdu_info = FIELD_GET(MT_RXD4_NORMAL_PAYLOAD_FORMAT, rxd4);
	status->amsdu = !!amsdu_info;
	if (status->amsdu) {
		status->first_amsdu = amsdu_info == MT_RXD4_FIRST_AMSDU_FRAME;
		status->last_amsdu = amsdu_info == MT_RXD4_LAST_AMSDU_FRAME;
	}
#endif

	hdr_gap = num_rxd * sizeof(uint32_t) + 2 * remove_pad;
	if (mbuf_len(m) < hdr_gap + sizeof(struct ieee80211_frame_min)) {
		if (sc->sc_rx_fill_fail < 64) {
			MWX_DEV_LOG(sc,
			    "rx drop reason=short-80211 len=%zu hdr_gap=%u rxd=%08x/%08x/%08x/%08x/%08x\n",
			    mbuf_len(m), hdr_gap, rxd0, rxd1, rxd2, rxd3,
			    rxd4);
		}
		return -1;
	}
	m_adj(m, hdr_gap);
#if 0
	if (status->amsdu) {
		memmove(skb->data + 2, skb->data,
			ieee80211_get_hdrlen_from_skb(skb));
		skb_pull(skb, 2);
	}

	struct ieee80211_hdr *hdr;

	if (insert_ccmp_hdr) {
		u8 key_id = FIELD_GET(MT_RXD1_NORMAL_KEY_ID, rxd1);

		mt76_insert_ccmp_hdr(skb, key_id);
	}

	hdr = mt76_skb_get_hdr(skb);
	fc = hdr->frame_control;
	if (ieee80211_is_data_qos(fc)) {
		seq_ctrl = le16_to_cpu(hdr->seq_ctrl);
		qos_ctl = *ieee80211_get_qos_ctl(hdr);
	}

	if (!status->wcid || !ieee80211_is_data_qos(fc))
		return 0;

	status->aggr = unicast && !ieee80211_is_qos_nullfunc(fc);
	status->seqno = IEEE80211_SEQ_TO_SN(seq_ctrl);
	status->qos_ctl = qos_ctl;
#endif
	rxi->rxi_chan = chfnum;

	/* Observe only unicast data MPDUs; do not count management/control traffic
	 * or hardware A-MSDU subframes as independent aggregate members. */
	{
		struct ieee80211_frame *wh = mtod(m, struct ieee80211_frame *);
		if (unicast && (wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) ==
		    IEEE80211_FC0_TYPE_DATA &&
		    !(wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_NODATA))
			mtkRxAmpduObserve(sc->sc_rx_ampdu_stats,
			    (rxd1 & MT_RXD1_NORMAL_GROUP_2) != 0, ampdu_timestamp,
			    (rxd2 & MT_RXD2_NORMAL_NON_AMPDU) != 0,
			    (rxd4 & MT_RXD4_NORMAL_PAYLOAD_FORMAT) != 0);
	}

	return 0;
}

uint32_t
mt7921_mac_tx_rate_val(struct mwx_softc *sc)
{
	int rateidx = 0, offset = 4;
	uint32_t rate, mode;

	/* XXX TODO basic_rates
	rateidx = ffs(vif->bss_conf.basic_rates) - 1;
	*/

	if (IEEE80211_IS_CHAN_2GHZ(sc->sc_ic.ic_bss->ni_chan))
		offset = 0;
	/* pick the lowest rate for hidden nodes */
	if (rateidx < 0)
		rateidx = 0;

	rateidx += offset;

	if (rateidx >= nitems(mt76_rates))
		rateidx = offset;

	rate =  mt76_rates[rateidx].hw_value;
	mode = (rate >> 8) << MT_TX_RATE_MODE_SHIFT;
	rate &= 0xff;

	return (rate & MT_TX_RATE_IDX_MASK) | (mode & MT_TX_RATE_MODE_MASK);
}

/* Is this 802.11 data frame an EAPOL frame (LLC/SNAP, ethertype 0x888e)? (luzility 110c92c) */
static int
mwx_is_eapol(struct mbuf *m, u_int hdrlen)
{
	static const uint8_t eapol_llc[8] = {
		0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00, 0x88, 0x8e
	};
	uint8_t llc[sizeof(eapol_llc)];

	if (mbuf_pkthdr_len(m) < hdrlen + sizeof(llc))
		return 0;
	if (mbuf_copydata(m, hdrlen, sizeof(llc), llc) != 0)
		return 0;
	return memcmp(llc, eapol_llc, sizeof(llc)) == 0;
}

void
mt7921_mac_write_txwi_80211(struct mwx_softc *sc, struct mbuf *m,
    struct ieee80211_node *ni, struct mt76_txwi *txp)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_frame *wh;
	uint32_t val;
	uint8_t fc_type, type, subtype, tid = 0;
	u_int hdrlen;
	int multicast;


	wh = mtod(m, struct ieee80211_frame *);
	/*
	 * fc_type keeps net80211's in-place encoding for comparisons with IEEE80211_FC0_TYPE_*; type/subtype are
	 * shifted for the TXD fields. Comparing the shifted value with IEEE80211_FC0_TYPE_DATA (0x08) took every
	 * data frame for management: fixed basic rate (1 Mbit/s CCK on 2.4 GHz, 6 Mbit/s OFDM on 5 GHz) and
	 * BA_DISABLE on all TX (upload ~1 / ~5 Mbit/s). Same fix as luzility/SimpleMtkWlan 110c92c.
	 */
	fc_type = wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK;
	type = fc_type >> IEEE80211_FC0_TYPE_SHIFT;
	subtype = wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK;
	subtype >>= IEEE80211_FC0_SUBTYPE_SHIFT;
	multicast = IEEE80211_IS_MULTICAST(wh->i_addr1);

	if (fc_type == IEEE80211_FC0_TYPE_CTL)
		hdrlen = sizeof(struct ieee80211_frame_min);
	else
		hdrlen = ieee80211_get_hdrlen(wh);

	/* Put QoS frames on the data queue which maps to their TID. */
	if (ieee80211_has_qos(wh)) {
		uint16_t qos = ieee80211_get_qos(wh);

		tid = qos & IEEE80211_QOS_TID;
	}

	/*
	 * Like Linux: aggregate only TIDs with a TX Block Ack agreement, and
	 * let the MAC build A-MSDUs where the AP allowed A-MSDU in A-MPDU.
	 */
	if (type == (IEEE80211_FC0_TYPE_DATA >> IEEE80211_FC0_TYPE_SHIFT) &&
	    !multicast) {
		struct ieee80211_tx_ba *ba = ieee80211_has_qos(wh) && tid < 8 ?
		    &ni->ni_tx_ba[tid] : NULL;

		if (ba == NULL || ba->ba_state != IEEE80211_BA_AGREED)
			txp->txwi[3] |= htole32(MT_TXD3_BA_DISABLE);
		else if (sc->sc_tx_amsdu &&
		    (ba->ba_params & IEEE80211_ADDBA_AMSDU))
			txp->txwi[7] |= htole32(MT_TXD7_HW_AMSDU);
	}

#ifdef NOTYET
	if (ieee80211_is_action(fc) &&
	    mgmt->u.action.category == WLAN_CATEGORY_BACK &&
	    mgmt->u.action.u.addba_req.action_code == WLAN_ACTION_ADDBA_REQ) {
		u16 capab = le16_to_cpu(mgmt->u.action.u.addba_req.capab);

		txp->txwi[5] |= htole32(MT_TXD5_ADD_BA);
		tid = (capab >> 2) & IEEE80211_QOS_CTL_TID_MASK;
	} else if (ieee80211_is_back_req(hdr->frame_control)) {
		struct ieee80211_bar *bar = (struct ieee80211_bar *)hdr;
		u16 control = le16_to_cpu(bar->control);

		tid = FIELD_GET(IEEE80211_BAR_CTRL_TID_INFO_MASK, control);
	}
#endif

	/* Let the firmware know about our ADDBA requests (as Linux does). */
	if (type == (IEEE80211_FC0_TYPE_MGT >> IEEE80211_FC0_TYPE_SHIFT) &&
	    subtype == (IEEE80211_FC0_SUBTYPE_ACTION >>
	    IEEE80211_FC0_SUBTYPE_SHIFT) &&
	    mbuf_len(m) >= hdrlen + 5) {
		uint8_t *act = mtod(m, uint8_t *) + hdrlen;
		if (act[0] == IEEE80211_CATEG_BA &&
		    act[1] == IEEE80211_ACTION_ADDBA_REQ) {
			uint16_t capab = act[3] | (act[4] << 8);
			txp->txwi[5] |= htole32(MT_TXD5_ADD_BA);
			tid = (capab >> 2) & 0xf;
		}
	}

	val = MT_HDR_FORMAT_802_11 | MT_TXD1_HDR_INFO(hdrlen / 2) |
	    MT_TXD1_TID(tid);
	txp->txwi[1] |= htole32(val);

	val = MT_TXD2_FRAME_TYPE(type) | MT_TXD2_SUB_TYPE(subtype);
	if (multicast)
		val |= MT_TXD2_MULTICAST;

#ifdef NOTYET
	if (key && multicast && ieee80211_is_robust_mgmt_frame(skb) &&
	    key->cipher == WLAN_CIPHER_SUITE_AES_CMAC) {
		val |= MT_TXD2_BIP;
		txp->txwi[3] &= ~htole32(MT_TXD3_PROTECT_FRAME);
	}
#endif

	/* Like mt76 with IEEE80211_TX_CTL_USE_MINRATE, keep EAPOL at the basic rate. */
	if (multicast || fc_type != IEEE80211_FC0_TYPE_DATA ||
	    mwx_is_eapol(m, hdrlen)) {
		/* Fixed rata is available just for 802.11 txd */
		uint32_t rate, val6;

		val |= MT_TXD2_FIX_RATE;
		/* hardware won't add HTC for mgmt/ctrl frame */
		val |= htole32(MT_TXD2_HTC_VLD);

		rate = mt7921_mac_tx_rate_val(sc);

		val6 = MT_TXD6_FIXED_BW;
		val6 |= (rate << MT_TXD6_TX_RATE_SHIFT) & MT_TXD6_TX_RATE_MASK;
		txp->txwi[6] |= htole32(val6);
		txp->txwi[3] |= htole32(MT_TXD3_BA_DISABLE);
	}

	txp->txwi[2] |= htole32(val);

#ifdef NOTYET
	if (ieee80211_is_beacon(fc)) {
		txp->txwi[3] &= ~htole32(MT_TXD3_SW_POWER_MGMT);
		txp->txwi[3] |= htole32(MT_TXD3_REM_TX_COUNT);
	}

	if (info->flags & IEEE80211_TX_CTL_INJECTED) {
		u16 seqno = le16_to_cpu(hdr->seq_ctrl);

		if (ieee80211_is_back_req(hdr->frame_control)) {
			struct ieee80211_bar *bar;

			bar = (struct ieee80211_bar *)skb->data;
			seqno = le16_to_cpu(bar->start_seq_num);
		}

		val = MT_TXD3_SN_VALID |
			FIELD_PREP(MT_TXD3_SEQ, IEEE80211_SEQ_TO_SN(seqno));
		txp->txwi[3] |= htole32(val);
		txp->txwi[7] &= ~htole32(MT_TXD7_HW_AMSDU);
	}
#endif

	val = MT_TXD7_TYPE(type) | MT_TXD7_SUB_TYPE(subtype);
	txp->txwi[7] |= htole32(val);

#if NBPFILTER > 0
	if (__predict_false(sc->sc_drvbpf != NULL)) {
		struct mwx_tx_radiotap_header *tap = &sc->sc_txtap;
		uint16_t chan_flags;

		tap->wt_flags = 0;
		tap->wt_chan_freq = htole16(ni->ni_chan->ic_freq);
		chan_flags = ni->ni_chan->ic_flags;
		if (ic->ic_curmode != IEEE80211_MODE_11N &&
			ic->ic_curmode != IEEE80211_MODE_11AC) {
			chan_flags &= ~IEEE80211_CHAN_HT;
			chan_flags &= ~IEEE80211_CHAN_40MHZ;
		}
		if (ic->ic_curmode != IEEE80211_MODE_11AC)
			chan_flags &= ~IEEE80211_CHAN_VHT;
		tap->wt_chan_flags = htole16(chan_flags);
#ifdef NOTYET
		if ((ni->ni_flags & IEEE80211_NODE_HT) &&
		    !IEEE80211_IS_MULTICAST(wh->i_addr1) &&
		    type == IEEE80211_FC0_TYPE_DATA &&
		    rinfo->ht_plcp != MWX_RATE_HT_SISO_MCS_INV_PLCP) {
			tap->wt_rate = (0x80 | rinfo->ht_plcp);
		} else
			tap->wt_rate = rinfo->rate;
#endif
		tap->wt_rate = 2;
		if ((ic->ic_flags & IEEE80211_F_WEPON) &&
		    (wh->i_fc[1] & IEEE80211_FC1_PROTECTED))
			tap->wt_flags |= IEEE80211_RADIOTAP_F_WEP;

		bpf_mtap_hdr(sc->sc_drvbpf, tap, sc->sc_txtap_len,
		    m, BPF_DIRECTION_OUT);
	}
#endif

}

static inline uint8_t
mt7921_lmac_mapping(uint8_t ac)
{
	/* LMAC uses the reverse order of mac80211 AC indexes */
	return 3 - ac;
}

void
mt7921_mac_write_txwi(struct mwx_softc *sc, struct mbuf *m,
    struct ieee80211_node *ni, struct mt76_txwi *txp)
{
	struct mwx_node *mn = (struct mwx_node *)ni;
	struct ieee80211_frame *wh;
	uint32_t val, p_fmt, omac_idx;
	uint8_t q_idx, wmm_idx, band_idx;
	uint8_t phy_idx = 0;
	/*
	 * No per-frame TX status: TXWIs are released by the TX free event,
	 * and a TXS event for every frame only loaded the RX ring (Linux
	 * asks for status only when mac80211 needs it).
	 */
	int pid = MT_PACKET_ID_NO_SKB;
	enum mt76_txq_id qid = MT_TXQ_BE;
	/* mac80211 AC index (VO=0, VI=1, BE=2, BK=3) of the hardware queue */
	uint8_t ac = 0;

	wh = mtod(m, struct ieee80211_frame *);
	/*
	 * Data goes to the queue of its access category; management and
	 * control frames stay on VO, like mac80211. Everything used to go to
	 * VO, where the hardware does not aggregate best-effort traffic well.
	 */
	if ((wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) == IEEE80211_FC0_TYPE_DATA) {
		static const uint8_t tid_to_ac[8] = { 2, 3, 3, 2, 1, 1, 0, 0 };

		ac = ieee80211_has_qos(wh) ?
		    tid_to_ac[ieee80211_get_qos(wh) & IEEE80211_QOS_TID & 7] : 2;
	}
	omac_idx = sc->sc_vif.omac_idx << MT_TXD1_OWN_MAC_SHIFT;
	wmm_idx = sc->sc_vif.wmm_idx;
	band_idx = sc->sc_vif.band_idx;

	if (qid >= MT_TXQ_PSD) {
		p_fmt = MT_TX_TYPE_CT;
		q_idx = MT_LMAC_ALTX0;
	} else {
		p_fmt = MT_TX_TYPE_CT;
		q_idx = wmm_idx * MWX_MAX_WMM_SETS + mt7921_lmac_mapping(ac);

#ifdef NOTYET
		/* counting non-offloading skbs */
		wcid->stats.tx_bytes += skb->len;
		wcid->stats.tx_packets++;
#endif
	}

	val = ((mbuf_pkthdr_len(m) + MT_TXD_SIZE) & MT_TXD0_TX_BYTES_MASK) |
		p_fmt | MT_TXD0_Q_IDX(q_idx);
	txp->txwi[0] = htole32(val);

	val = MT_TXD1_LONG_FORMAT | (mn->wcid & MT_TXD1_WLAN_IDX_MASK) |
	    (omac_idx & MT_TXD1_OWN_MAC_MASK);
	if (phy_idx || band_idx)
		val |= MT_TXD1_TGID;
	txp->txwi[1] = htole32(val);
	txp->txwi[2] = 0;

	val = 15 << MT_TXD3_REM_TX_COUNT_SHIFT;
	if (wh->i_fc[1] & IEEE80211_FC1_PROTECTED)
		val |= MT_TXD3_PROTECT_FRAME;
#ifdef NOTYET
	if (info->flags & IEEE80211_TX_CTL_NO_ACK)
		val |= MT_TXD3_NO_ACK;
#endif
	txp->txwi[3] = htole32(val);
	txp->txwi[4] = 0;

	val = pid & MT_TXD5_PID;
	if (pid >= MT_PACKET_ID_FIRST)
		val |= MT_TXD5_TX_STATUS_HOST;
	txp->txwi[5] = htole32(val);
	txp->txwi[6] = 0;
	txp->txwi[7] = /* XXX wcid->amsdu ? htole32(MT_TXD7_HW_AMSDU) : */ 0;

#ifdef NOTYET
	if (is_8023)
		mt76_connac2_mac_write_txwi_8023(txp, m, wcid);
	else
#endif
		mt7921_mac_write_txwi_80211(sc, m, ni, txp);
}

void
mwx_mac_tx_free(struct mwx_softc *sc, struct mbuf *m)
{
	struct mwx_txwi *mt;
	uint32_t *txfree;
	uint32_t txval;
	int count, i;

	mwx_dma_tx_cleanup(sc, &sc->sc_txq);

	/* Both the event header and each PAIR/MSDU word must be present. */
	if (mbuf_pkthdr_len(m) < 2 * sizeof(txval))
		goto out;

	if ((m = m_pullup(m, mbuf_pkthdr_len(m))) == NULL)
		return;

	txfree = mtod(m, uint32_t *);
	txval = le32toh(txfree[0]);
	m_adj(m, 2 * sizeof(txval));

	count = MT_TX_FREE0_MSDU_CNT_GET(txval);

	if (count * sizeof(txval) > mbuf_len(m))
		goto out;

	txfree = mtod(m, uint32_t *);
	for (i = 0; i < count; i++) {
		uint16_t msdu;

		/* PAIR words extend count, but never the received payload. */
		if ((size_t)i >= mbuf_len(m) / sizeof(*txfree))
			break;
		txval = le32toh(txfree[i]);
		if (txval & MT_TX_FREE_PAIR) {
			count++;
			continue;
		}

		msdu = MT_TX_FREE_MSDU_ID_GET(txval);
		if (msdu >= sc->sc_txwi.mt_count)
			continue;
		mt = &sc->sc_txwi.mt_data[msdu];
		if (mt->mt_busy == 0)
			continue;
#ifdef MTK_AWDL
		if (sc->sc_awdl != NULL)
			(void)mwx_awdl_tx_done(sc, msdu, txval);
#endif
		mwx_txwi_put(sc, mt);
	}

out:
	m_freem(m);
	mwx_tx_kick(sc);
}

int
mt7921_set_channel(struct mwx_softc *sc)
{
	int rv;

	/* stop queues, block other configs (MT76_RESET) */
	// XXX NOTYET mt76_set_channel(sc);

	rv = mt7921_mcu_set_chan_info(sc, MCU_EXT_CMD_CHANNEL_SWITCH);
	if (rv)
		return rv;
	mt7921_mac_set_timing(sc);
	mt7921_mac_reset_counters(sc);

	/* restart queues */
	return 0;
}

uint8_t
mt7921_get_phy_mode_v2(struct mwx_softc *sc, struct ieee80211_node *ni)
{
	uint8_t mode = 0;

	if (ni == NULL)
		ni = sc->sc_ic.ic_bss;

	if (IEEE80211_IS_CHAN_2GHZ(ni->ni_chan)) {
		mode |= PHY_TYPE_BIT_HR_DSSS | PHY_TYPE_BIT_ERP;
		if (ieee80211_node_supports_ht(ni))
			mode |= PHY_TYPE_BIT_HT;
		if ((ni->ni_flags & IEEE80211_NODE_HE) && sc->sc_he_enabled)
			mode |= PHY_TYPE_BIT_HE;
	} else if (IEEE80211_IS_CHAN_5GHZ(ni->ni_chan) /* || CHAN_6GHZ */) {
		mode |= PHY_TYPE_BIT_OFDM;
		if (ieee80211_node_supports_ht(ni))
			mode |= PHY_TYPE_BIT_HT;
		/* only once VHT is actually negotiated (not just supported) */
		if (ni->ni_flags & IEEE80211_NODE_VHT)
			mode |= PHY_TYPE_BIT_VHT;
		if ((ni->ni_flags & IEEE80211_NODE_HE) && sc->sc_he_enabled)
			mode |= PHY_TYPE_BIT_HE;
#ifdef NOTYET
		if (ieee80211_node_supports_he(ni))
			mode |= PHY_TYPE_BIT_HE;
#endif
	}
	return mode;
}

struct mbuf *
mwx_alloc_sta_req_tlv(int len)
{
	struct mbuf *m;

	/* Allocate mbuf cluster with enough space */
	m = MCLGETL(NULL, M_DONTWAIT, MCLBYTES);
	if (m == NULL)
		return NULL;

	/* align to have space for the mcu header */
	mbuf_setdata(m, (uint8_t *)mbuf_data(m) + sizeof(struct mwx_mcu_txd) + len, 0);
	mbuf_setlen(m, 0);
	mbuf_pkthdr_setlen(m, 0);

	return m;
}

void *
mwx_append_len(struct mbuf *m, int len)
{
	caddr_t p;
	size_t oldlen;

	KASSERT(m_trailingspace(m) >= len);

	oldlen = mbuf_len(m);
	p = mtod(m, caddr_t) + oldlen;
	mbuf_setlen(m, oldlen + len);
	mbuf_pkthdr_setlen(m, mbuf_len(m));
	memset(p, 0, len);

	return p;
}

/*
 * Reserve len bytes at the end of mbuf m, return to start of that area
 * after initializing the data. It also sets the tag and len hdr.
 */
void *
mwx_append_tlv(struct mbuf *m, uint16_t *tlvnum, int tag, int len)
{
	struct {
		uint16_t	tag;
		uint16_t	len;
	} tlv = {
		.tag = htole16(tag),
		.len = htole16(len),
	};
	caddr_t p;

	p = (caddr_t)mwx_append_len(m, len);
	memcpy(p, &tlv, sizeof(tlv));

	*tlvnum += 1;

	return p;
}

struct mbuf *
mwx_fill_sta_req_hdr(struct mbuf *m, struct mwx_vif *mvif,
    uint8_t muar_idx, uint16_t wcid, uint16_t tlvnum)
{
	struct sta_req_hdr *hdr;

	KASSERT(m_leadingspace(m) >= sizeof(struct sta_req_hdr));
	m = m_prepend(m, sizeof(struct sta_req_hdr), M_DONTWAIT);
	if (m == NULL)
		return NULL;
	hdr = mtod(m, struct sta_req_hdr *);
	memset(hdr, 0, sizeof(struct sta_req_hdr));

	hdr->bss_idx = mvif->idx;
	hdr->wlan_idx_lo = wcid & 0xff;
	hdr->wlan_idx_hi = wcid >> 8;
	hdr->muar_idx = muar_idx;
	hdr->is_tlv_append = 1;
	hdr->tlv_num = htole16(tlvnum);
	return m;
}

void
mt7921_mcu_add_basic_tlv(struct mbuf *m, uint16_t *tlvnum, struct mwx_softc *sc,
    struct ieee80211_node *ni, int add, int isnew)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct sta_rec_basic *basic;

	basic = (struct sta_rec_basic *)mwx_append_tlv(m, tlvnum,
	    STA_REC_BASIC, sizeof(*basic));

	basic->extra_info = htole16(EXTRA_INFO_VER);
	if (add) {
		if (isnew)
			basic->extra_info |= htole16(EXTRA_INFO_NEW);
		basic->conn_state = CONN_STATE_PORT_SECURE;
	} else {
		basic->conn_state = CONN_STATE_DISCONNECT;
	}

	if (ni == NULL) {
		basic->conn_type = htole32(STA_TYPE_BC | NETWORK_INFRA);
		memset(basic->peer_addr, 0xff, sizeof(basic->peer_addr));
		return;
	}

	switch (ic->ic_opmode) {
#ifndef IEEE80211_STA_ONLY
	case IEEE80211_M_HOSTAP:
		basic->conn_type = htole32(STA_TYPE_STA | NETWORK_INFRA);
		break;
#endif
	case IEEE80211_M_STA:
		basic->conn_type = htole32(STA_TYPE_AP | NETWORK_INFRA);
		break;
#ifndef IEEE80211_STA_ONLY
	case IEEE80211_M_IBSS:
	case IEEE80211_M_AHDEMO:
		basic->conn_type = htole32(STA_TYPE_ADHOC | NETWORK_IBSS);
		break;
#endif
	case IEEE80211_M_MONITOR:
		panic("mt7921_mcu_sta_basic_tlv unexpected operation mode");
	}

	basic->aid = htole16(IEEE80211_AID(ni->ni_associd));
	memcpy(basic->peer_addr, ni->ni_macaddr, IEEE80211_ADDR_LEN);
	basic->qos = (ni->ni_flags & IEEE80211_NODE_QOS) != 0;
}

void
mt7921_mcu_add_sta_tlv(struct mbuf *m, uint16_t *tlvnum, struct mwx_softc *sc,
    struct ieee80211_node *ni, int add, int isnew)
{
	//struct ieee80211com *ic = &sc->sc_ic;
	struct sta_rec_ra_info *ra_info;
	struct sta_rec_state *state;
	struct sta_rec_phy *phy;
	uint16_t supp_rates;

#ifdef NOTYET
	/* sta rec ht */
	if (sta->deflink.ht_cap.ht_supported) {
		struct sta_rec_ht *ht;

		ht = mwx_append_tlv(m, tlvnum, STA_REC_HT, sizeof(*ht));
		ht->ht_cap = htole16(sta->deflink.ht_cap.cap);
	}

	/* sta rec vht */
	if (sta->deflink.vht_cap.vht_supported) {
		struct sta_rec_vht *vht;

		vht = mwx_append_tlv(m, tlvnum, STA_REC_VHT,
		    sizeof(*vht));
		vht->vht_cap = htole32(sta->deflink.vht_cap.cap);
		vht->vht_rx_mcs_map = sta->deflink.vht_cap.vht_mcs.rx_mcs_map;
		vht->vht_tx_mcs_map = sta->deflink.vht_cap.vht_mcs.tx_mcs_map;
	}

	/* sta rec uapsd */
	/* from function:
		if (vif->type != NL80211_IFTYPE_AP || !sta->wme)
			return;
	*/
	mt7921_mcu_sta_uapsd(m, tlvnum, vif, ni);
#endif

#ifdef NOTYET
	if (sta->deflink.ht_cap.ht_supported || sta->deflink.he_cap.has_he)
		mt76_connac_mcu_sta_amsdu_tlv(skb, sta, vif);

	/* sta rec he */
	if (sta->deflink.he_cap.has_he) {
		mt76_connac_mcu_sta_he_tlv(skb, sta);
		if (band == NL80211_BAND_6GHZ &&
		    sta_state == MT76_STA_INFO_STATE_ASSOC) {
			struct sta_rec_he_6g_capa *he_6g_capa;

			he_6g_capa = mwx_append_tlv(m, tlvnum,
			    STA_REC_HE_6G, sizeof(*he_6g_capa));
			he_6g_capa->capa = sta->deflink.he_6ghz_capa.capa;
		}
	}
#endif

	phy = (struct sta_rec_phy *)mwx_append_tlv(m, tlvnum, STA_REC_PHY,
	    sizeof(*phy));
	/* XXX basic_rates: bitmap of basic rates, each bit stands for an
	 *      index into the rate table configured by the driver in
	 *      the current band.
	 */
	phy->basic_rate = htole16(0x0150); /* XXX */
	phy->phy_type = mt7921_get_phy_mode_v2(sc, ni);
#ifdef NOTYET
	phy->ampdu = FIELD_PREP(IEEE80211_HT_AMPDU_PARM_FACTOR,
	    sta->deflink.ht_cap.ampdu_factor) |
	    FIELD_PREP(IEEE80211_HT_AMPDU_PARM_DENSITY,
	    sta->deflink.ht_cap.ampdu_density);
#endif
	// XXX phy->rcpi = rssi_to_rcpi(-ewma_rssi_read(&sc->sc_vif.rssi));
	phy->rcpi = 0xdc;	/* XXX STOLEN FROM LINUX DUMP */

#ifdef HACK
	supp_rates = sta->deflink.supp_rates[band];
	if (band == NL80211_BAND_2GHZ)
		supp_rates = FIELD_PREP(RA_LEGACY_OFDM, supp_rates >> 4) |
		FIELD_PREP(RA_LEGACY_CCK, supp_rates & 0xf);
	else
		supp_rates = FIELD_PREP(RA_LEGACY_OFDM, supp_rates);
#else
	supp_rates = RA_LEGACY_OFDM;
#endif

	ra_info = (struct sta_rec_ra_info *)mwx_append_tlv(m, tlvnum,
	    STA_REC_RA, sizeof(*ra_info));
	ra_info->legacy = htole16(supp_rates);
#ifdef NOTYET
	if (sta->deflink.ht_cap.ht_supported)
		memcpy(ra_info->rx_mcs_bitmask,
			sta->deflink.ht_cap.mcs.rx_mask,
			HT_MCS_MASK_NUM);
#endif

	if ((ni->ni_flags & IEEE80211_NODE_HE) && sc->sc_he_enabled)
		mwx_add_sta_he_tlvs(m, tlvnum, ni);
	state = (struct sta_rec_state *)mwx_append_tlv(m, tlvnum,
	    STA_REC_STATE, sizeof(*state));
	state->state = /* XXX sta_state */ 0;
#ifdef NOTYET
	if (sta->deflink.vht_cap.vht_supported) {
		state->vht_opmode = sta->deflink.bandwidth;
		state->vht_opmode |= (sta->deflink.rx_nss - 1) <<
			IEEE80211_OPMODE_NOTIF_RX_NSS_SHIFT;
	}
#endif
}

int
mt7921_mcu_wtbl_generic_tlv(struct mbuf *m, uint16_t *tlvnum,
    struct mwx_softc *sc, struct ieee80211_node *ni)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct wtbl_generic *generic;
	struct wtbl_rx *rx;

	generic = (struct wtbl_generic *)mwx_append_tlv(m, tlvnum,
	    WTBL_GENERIC, sizeof(*generic));

	if (ni) {
		generic->partial_aid = htole16(IEEE80211_AID(ni->ni_associd));
		memcpy(generic->peer_addr, ni->ni_macaddr, IEEE80211_ADDR_LEN);
		generic->muar_idx = sc->sc_vif.omac_idx;
		generic->qos = (ni->ni_flags & IEEE80211_NODE_QOS) != 0;
	} else {
		memset(generic->peer_addr, 0xff, IEEE80211_ADDR_LEN);
		generic->muar_idx = 0xe;
	}

	rx = (struct wtbl_rx *)mwx_append_tlv(m, tlvnum, WTBL_RX,
	    sizeof(*rx));
#ifndef IEEE80211_STA_ONLY
	rx->rca1 = ni ? ic->ic_opmode != IEEE80211_M_HOSTAP : 1;
#else
	rx->rca1 = 1;
#endif
	rx->rca2 = 1;
	rx->rv = 1;

	return sizeof(*generic) + sizeof(*rx);
}

int
mt7921_mcu_wtbl_hdr_trans_tlv(struct mbuf *m, uint16_t *tlvnum,
    struct mwx_softc *sc, struct ieee80211_node *ni)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct wtbl_hdr_trans *htr;

	htr = (struct wtbl_hdr_trans *)mwx_append_tlv(m, tlvnum,
	    WTBL_HDR_TRANS, sizeof(*htr));

	/* no hdr decapsulation offload */
	htr->no_rx_trans = 1;

	if (ic->ic_opmode == IEEE80211_M_STA)
		htr->to_ds = 1;
	else
		htr->from_ds = 1;

	return sizeof(*htr);
}

int
mt7921_mcu_wtbl_ht_tlv(struct mbuf *m, uint16_t *tlvnum,
    struct mwx_softc *sc, struct ieee80211_node *ni)
{
	struct wtbl_ht *ht = NULL;
	struct wtbl_smps *smps;
	int len = 0;

	/* Connac2 hardware needs these in addition to the STA_REC capabilities. */
	if (ni->ni_flags & IEEE80211_NODE_HT) {
		ht = (struct wtbl_ht *)mwx_append_tlv(m, tlvnum,
		    WTBL_HT, sizeof(*ht));
		ht->ht = 1;
		ht->ldpc = !!(ni->ni_htcaps & IEEE80211_HTCAP_LDPC);
		ht->af = ni->ni_ampdu_param & 0x3;
		ht->mm = (ni->ni_ampdu_param >> 2) & 0x7;
		len += sizeof(*ht);
	}
	if (ni->ni_flags & IEEE80211_NODE_VHT) {
		struct wtbl_vht *vht = (struct wtbl_vht *)mwx_append_tlv(m,
		    tlvnum, WTBL_VHT, sizeof(*vht));
		vht->vht = 1;
		vht->ldpc = !!(ni->ni_vhtcaps & IEEE80211_VHTCAP_RXLDPC);
		uint8_t af = (ni->ni_vhtcaps &
		    IEEE80211_VHTCAP_MAX_A_MPDU_LENGTH_EXPONENT_MASK) >>
		    IEEE80211_VHTCAP_MAX_A_MPDU_LENGTH_EXPONENT_SHIFT;
		if (ht && af > ht->af)
			ht->af = af;
		len += sizeof(*vht);
	}

	smps = (struct wtbl_smps *)mwx_append_tlv(m, tlvnum, WTBL_SMPS,
	    sizeof(*smps));
	/* spatial multiplexing power save mode, off for now */
	//smps->smps = (sta->deflink.smps_mode == IEEE80211_SMPS_DYNAMIC);

	return len + sizeof(*smps);
}

int
mt7921_mac_sta_update(struct mwx_softc *sc, struct ieee80211_node *ni,
    int add, int isnew)
{
	struct mwx_node *mw = (struct mwx_node *)ni;
	struct mwx_vif *mvif = &sc->sc_vif;
	struct sta_rec_wtbl *wtbl;
	struct mbuf *m = NULL;
	uint16_t tlvnum = 0, wnum = 0;
	int wlen = 0;

	m = mwx_alloc_sta_req_tlv(sizeof(struct sta_req_hdr));
	if (m == NULL)
		return ENOBUFS;

	if (ni != NULL)
		mt7921_mcu_add_basic_tlv(m, &tlvnum, sc, ni, add, isnew);

	if (ni != NULL && add)
		mt7921_mcu_add_sta_tlv(m, &tlvnum, sc, ni, add, isnew);

	wtbl = (struct sta_rec_wtbl *)mwx_append_tlv(m, &tlvnum,
	    STA_REC_WTBL, sizeof(*wtbl));
	wtbl->wlan_idx_lo = mw ? mw->wcid & 0xff : 0;
	wtbl->wlan_idx_hi = mw ? mw->wcid >> 8 : 0;
	wtbl->operation = WTBL_RESET_AND_SET;

	if (add) {
		wlen += mt7921_mcu_wtbl_generic_tlv(m, &wnum, sc, ni);
		wlen += mt7921_mcu_wtbl_hdr_trans_tlv(m, &wnum, sc, ni);

		if (ni)
			wlen += mt7921_mcu_wtbl_ht_tlv(m, &wnum, sc, ni);
	}

	wtbl->tlv_num = htole16(wnum);
	wtbl->len = htole16(le16toh(wtbl->len) + wlen);

	m = mwx_fill_sta_req_hdr(m, mvif, ni ? mvif->omac_idx : 0,
	    mw ? mw->wcid : 0, tlvnum);
	if (m == NULL)
		return ENOBUFS;

	{
		int rv = mwx_mcu_send_mbuf_wait(sc, MCU_UNI_CMD_STA_REC_UPDATE, m);
		MWX_DEV_LOG(sc, "sta_update add=%d new=%d wcid=%u peer=%s rv=%d\n",
		    add, isnew, mw ? mw->wcid : 0,
		    ni ? ether_sprintf(ni->ni_macaddr) : "-", rv);
		return rv;
	}
}

static int
mt7921_key_to_cipher_id(struct ieee80211_key *k)
{
	switch (k->k_cipher) {
	case IEEE80211_CIPHER_CCMP:
		 return MCU_CIPHER_AES_CCMP;
	case IEEE80211_CIPHER_TKIP:
		return MCU_CIPHER_TKIP;
	case IEEE80211_CIPHER_WEP40:
		return MCU_CIPHER_WEP40;
	case IEEE80211_CIPHER_WEP104:
		 return MCU_CIPHER_WEP104;
	case IEEE80211_CIPHER_BIP:
		return MCU_CIPHER_BIP_CMAC_128;
	default:
		return MCU_CIPHER_NONE;
	}
}

void
mt7921_mcu_add_key_tlv(struct mbuf *m, uint16_t *tlvnum,
    struct ieee80211_key *k, int add)
{
	struct sta_rec_sec *sec;
	struct sta_rec_sec_key *key;

	sec = (struct sta_rec_sec *)mwx_append_tlv(m, tlvnum, STA_REC_KEY,
	    sizeof(*sec));

	if (add) {
		sec->n_cipher = 1;
		sec->add = 0;	/* connac2 has inverted logic */
		sec->len = htole16(sizeof(*sec) + sizeof(*key));

		key = (struct sta_rec_sec_key *)mwx_append_len(m, sizeof(*key));
		key->cipher_id = mt7921_key_to_cipher_id(k);
		key->cipher_len = sizeof(*key);
		key->key_id = k->k_id;
		key->key_len = k->k_len;

		KASSERT(k->k_len <= sizeof(key->key));
		memcpy(key->key, k->k_key, k->k_len);

		if (key->cipher_id == MCU_CIPHER_TKIP &&
		    k->k_len == sizeof(key->key)) {
			/* Rx/Tx MIC keys are swapped. */
			memcpy(key->key + 16, k->k_key + 24, 8);
			memcpy(key->key + 24, k->k_key + 16, 8);
		}
	} else {
		sec->add = 1;
		sec->n_cipher = 0;
	}
}

int
mt7921_mcu_sta_key_update(struct mwx_softc *sc, struct ieee80211_node *ni,
    struct ieee80211_key *k)
{
	struct mwx_vif *mvif = &sc->sc_vif;
	struct mwx_node *mn = (struct mwx_node *)ni;
	struct mbuf *m;
	int want_keymask = (MWX_NODE_FLAG_HAVE_PAIRWISE_KEY |
	    MWX_NODE_FLAG_HAVE_GROUP_KEY);
	int rv;
	uint16_t tlvnum = 0, wcid = 0;
	uint8_t muar_idx = 0;

	if (ni == NULL) {
		ni = sc->sc_ic.ic_bss;
		mn = (struct mwx_node *)ni;
	}
	if (ni == NULL)
		return ENXIO;

	if ((k->k_flags & IEEE80211_KEY_GROUP) != 0) {
		wcid = mvif->vif_mn.wcid;
		muar_idx = 0x0e;
	} else {
		wcid = mn->wcid;
	}

	m = mwx_alloc_sta_req_tlv(sizeof(struct sta_req_hdr));
	if (m == NULL)
		return ENOBUFS;

	DPRINTF("%s: %s: k_id %d, k_flags %x k_cipher %d wcid %d\n",
	    DEVNAME(sc), "add key", k->k_id, k->k_flags, k->k_cipher, wcid);
	MWX_DEV_LOG(sc,
	    "key add request id=%d flags=0x%x cipher=%d len=%d wcid=%u muar=%u\n",
	    k->k_id, k->k_flags, k->k_cipher, k->k_len, wcid, muar_idx);

	mt7921_mcu_add_key_tlv(m, &tlvnum, k, 1);
	m = mwx_fill_sta_req_hdr(m, mvif, muar_idx, wcid, tlvnum);
	if (m == NULL)
		return ENOBUFS;

	rv = mwx_mcu_send_mbuf_wait(sc, MCU_UNI_CMD_STA_REC_UPDATE, m);
	if (rv != 0)
		return rv;
	MWX_DEV_LOG(sc,
	    "key add done id=%d flags=0x%x cipher=%d wcid=%u node_flags_before=0x%x\n",
	    k->k_id, k->k_flags, k->k_cipher, wcid, mn->flags);

	/* Keep track of added keys to trigger link up when all are done. */
	if ((k->k_flags & IEEE80211_KEY_GROUP) != 0)
		mn->flags |= MWX_NODE_FLAG_HAVE_GROUP_KEY;
	else
		mn->flags |= MWX_NODE_FLAG_HAVE_PAIRWISE_KEY;

	if ((mn->flags & want_keymask) == want_keymask) {
		DPRINTF("marking port %s valid\n", ether_sprintf(ni->ni_macaddr));
		MWX_DEV_LOG(sc,
		    "key add complete port valid mac=%s node_flags=0x%x\n",
		    ether_sprintf(ni->ni_macaddr), mn->flags);
		if (ni->ni_flags & IEEE80211_NODE_MFP)
			ni->ni_flags |= IEEE80211_NODE_TXMGMTPROT;
		ni->ni_port_valid = 1;
		ieee80211_set_link_state(&sc->sc_ic, LINK_STATE_UP);
	}

	return 0;
}

void
mt7921_mcu_sta_key_delete(struct mwx_softc *sc, struct ieee80211_node *ni,
    struct ieee80211_key *k)
{
	struct mwx_vif *mvif = &sc->sc_vif;
	struct mwx_node *mn = (struct mwx_node *)ni;
	struct mbuf *m;
	uint16_t tlvnum = 0, wcid = 0;
	uint8_t muar_idx = 0;

	if (ni == NULL) {
		ni = sc->sc_ic.ic_bss;
		mn = (struct mwx_node *)ni;
	}
	if (ni == NULL)
		return;

	if ((k->k_flags & IEEE80211_KEY_GROUP) != 0) {
		wcid = mvif->vif_mn.wcid;
		muar_idx = 0x0e;
		mn->flags &= ~MWX_NODE_FLAG_HAVE_GROUP_KEY;
	} else {
		wcid = mn->wcid;
		mn->flags &= ~MWX_NODE_FLAG_HAVE_PAIRWISE_KEY;
	}

	m = mwx_alloc_sta_req_tlv(sizeof(struct sta_req_hdr));
	if (m == NULL)
		return;

	DPRINTF("%s: %s: k_id %d, k_flags %x k_cipher %d wcid %d\n",
	    DEVNAME(sc), "delete key", k->k_id, k->k_flags, k->k_cipher,
	    wcid);

	mt7921_mcu_add_key_tlv(m, &tlvnum, k, 0);
	m = mwx_fill_sta_req_hdr(m, mvif, muar_idx, wcid, tlvnum);
	if (m == NULL)
		return;

	mwx_mcu_send_mbuf(sc, MCU_UNI_CMD_STA_REC_UPDATE, m, NULL);
}

#undef free

bool
MtkMwx::mwx_match(IOPCIDevice *device)
{
	if (device == NULL) {
		MWX_LOG("probe skipped: null PCI device\n");
		return false;
	}
	uint16_t vendor = device->configRead16(kIOPCIConfigVendorID);
	uint16_t product = device->configRead16(kIOPCIConfigDeviceID);
	bool matched = pci_matchbyid(vendor, product, mwx_devices,
	    nitems(mwx_devices)) != 0;
	MWX_LOG("probe vendor=0x%04x product=0x%04x matched=%d\n",
	    vendor, product, matched);
	return matched;
}

bool
MtkMwx::attach(IOPCIDevice *device)
{
	MWX_LOG("HAL attach begin\n");
	if (com == NULL)
		com = (struct mwx_softc *)malloc(sizeof(*com), M_DEVBUF, M_WAITOK | M_ZERO);
	if (pci == NULL)
		pci = (struct pci_attach_args *)malloc(sizeof(*pci), M_DEVBUF, M_WAITOK | M_ZERO);
	if (com == NULL || pci == NULL) {
		MWX_LOG("HAL attach allocation failed\n");
		releaseAll();
		return false;
	}
	if (com->sc_rx_amsdu_lock == NULL)
		com->sc_rx_amsdu_lock = IOLockAlloc();
	if (com->sc_rx_amsdu_lock == NULL) {
		releaseAll();
		return false;
	}
	if (com->sc_rx_pool_lock == NULL)
		com->sc_rx_pool_lock = IOSimpleLockAlloc();
	if (com->sc_rx_pool_lock == NULL) {
		releaseAll();
		return false;
	}
	SLIST_INIT(&com->sc_rx_pool_free);
	com->sc_rx_amsdu_gate = getMainCommandGate();
	if (com->sc_rx_amsdu_gate == NULL) {
		releaseAll();
		return false;
	}
	MWX_LOG("HAL attach allocated\n");

	if (!taskq_init()) {
		releaseAll();
		return false;
	}
	taskQueueOwned = true;
	gMwxHal = this;
	pci->pa_tag = device;
	pci->workloop = getMainWorkLoop();
	pci->pa_id = ((pcireg_t)device->configRead16(kIOPCIConfigDeviceID) << 16) |
	    device->configRead16(kIOPCIConfigVendorID);
	strlcpy(com->sc_dev.dv_xname, "mwx0", sizeof(com->sc_dev.dv_xname));
	MWX_LOG("HAL attach pci_id=0x%08x\n",
	    pci->pa_id);
	if (!mwx_attach(NULL, (struct device *)com, pci)) {
		MWX_LOG("HAL attach failed inside mwx_attach\n");
		detach(device);
		return false;
	}
#ifdef MTK_AWDL
	mwx_awdl_attach(com);
#endif
	MWX_LOG("HAL attach complete\n");
	return true;
}

void
MtkMwx::detach(IOPCIDevice *device)
{
    MWX_LOG("HAL detach begin\n");
    if (com == NULL) {
        if (taskQueueOwned) { taskq_destroy(systq); taskQueueOwned = false; }
        releaseAll();
        return;
    }
    struct ifnet *ifp = &com->sc_ic.ic_if;
#ifdef MTK_AWDL
    /* Before sc_detaching: a remain-on-channel period is given back while the firmware still answers. */
    if (com->sc_awdl != NULL)
        mwx_awdl_quiesce(com);
#endif
    com->sc_detaching = true;
    mwx_rx_amsdu_begin_teardown(com);
    timeout_del(&com->sc_reset_to);
    /* Keep interrupts alive while in-flight MCU requests finish. */
    taskq_quiesce(com->sc_nswq);
    if (taskQueueOwned) taskq_quiesce(systq);
    if (ifp->if_flags & IFF_RUNNING) mwx_stop_internal(ifp, true);
    timeout_free(&com->sc_reset_to);
    /* No interrupt callback may access buffers during their destruction. */
    if (com->sc_ih) {
        pci_intr_disestablish(pci->pa_pc, com->sc_ih);
        com->sc_ih = NULL;
    }
    if (com->sc_rx_amsdu_lock) {
        IOLockLock(com->sc_rx_amsdu_lock);
        com->sc_rx_amsdu_irq_quiesced = true;
        IOLockUnlock(com->sc_rx_amsdu_lock);
    }
    mwx_rx_amsdu_purge(com);
#ifdef MTK_AWDL
    /* No interrupt and no RX any more: the radio's thread and memory can go. */
    mwx_awdl_free(com);
#endif
    if (com->sc_net_attached && com->sc_newstate != NULL)
        com->sc_newstate(&com->sc_ic, IEEE80211_S_INIT, -1);
    if (com->sc_memh) {
        mwx_write(com, MT_WFDMA0_HOST_INT_ENA, 0);
        mwx_dma_disable(com, 1);
    }
    if (com->sc_net_attached) {
        ieee80211_ifdetach(ifp);
        com->sc_net_attached = false;
    }
    taskq_destroy(com->sc_nswq);
    com->sc_nswq = NULL;
    if (taskQueueOwned) { taskq_destroy(systq); taskQueueOwned = false; }
    mwx_txwi_free(com);
    mwx_dma_free(com);
    if (com->sc_memh) {
        bus_space_unmap(com->sc_st, com->sc_memh, com->sc_mems);
        com->sc_memh = 0;
    }
    releaseAll();
    MWX_LOG("HAL detach complete\n");
}

IOReturn
MtkMwx::enable(IONetworkInterface *interface)
{
	MWX_LOG("HAL enable begin\n");
	if (com == NULL) {
		MWX_LOG("HAL enable failed: not attached\n");
		return kIOReturnNotAttached;
	}
	struct ifnet *ifp = &com->sc_ic.ic_if;
	if (ifp->if_flags & IFF_UP) {
		MWX_DEV_LOG(com, "HAL enable skipped: already up flags=0x%x\n",
		    ifp->if_flags);
		return kIOReturnSuccess;
	}
	ifp->if_flags |= IFF_UP;
	int rv = mwx_init(ifp);
	MWX_DEV_LOG(com, "HAL enable done mwx_init=%d flags=0x%x\n",
	    rv, ifp->if_flags);
	return rv == 0 ? kIOReturnSuccess : kIOReturnError;
}

IOReturn
MtkMwx::disable(IONetworkInterface *interface)
{
	MWX_LOG("HAL disable begin\n");
	if (com == NULL)
		return kIOReturnSuccess;
	struct ifnet *ifp = &com->sc_ic.ic_if;
	if (!(ifp->if_flags & IFF_UP)) {
		MWX_DEV_LOG(com, "HAL disable skipped: already down flags=0x%x\n",
		    ifp->if_flags);
		return kIOReturnSuccess;
	}
	ifp->if_flags &= ~IFF_UP;
	if (ifp->if_flags & IFF_RUNNING)
		mwx_stop(ifp);
	MWX_DEV_LOG(com, "HAL disable complete flags=0x%x\n", ifp->if_flags);
	return kIOReturnSuccess;
}

struct ieee80211com *
MtkMwx::get80211Controller()
{
	if (com == NULL)
		com = (struct mwx_softc *)malloc(sizeof(*com), M_DEVBUF, M_WAITOK | M_ZERO);
	return com == NULL ? NULL : &com->sc_ic;
}

MtkDriverInfo *
MtkMwx::getDriverInfo()
{
	return this;
}

MtkDriverController *
MtkMwx::getDriverController()
{
	return this;
}

IOEthernetController *
MtkMwx::mwxController()
{
	return getController();
}

int
MtkMwx::mwxSleepNsec(void *ident, int priority, const char *wmesg,
    uint64_t timo)
{
	return MtkHalService::tsleep_nsec(ident, priority, wmesg, timo);
}

void
MtkMwx::mwxWakeup(void *ident)
{
	wakeupOn(ident);
}

const char *
MtkMwx::getFirmwareVersion()
{
	if (com == NULL)
		return "MT792x";
	return mwx_hw_name(com->sc_hwtype);
}

/*
 * Noise floor as a positive number of -dBm: both front ends negate it.
 * The firmware reports no noise measurement, and the 0 returned here made
 * macOS see 0 dBm of noise, so an SNR equal to the RSSI: negative on every
 * real link. Report the thermal noise floor of the operating width instead:
 * -174 dBm/Hz + 10*log10(bandwidth) plus a typical 7 dB receiver noise
 * figure.
 */
int16_t
MtkMwx::getBSSNoise()
{
	struct ieee80211com *ic;
	uint8_t bw = CMD_CBW_20MHZ, center;

	if (com == NULL)
		return 94;
	ic = &com->sc_ic;
	/* The width is the one negotiated with the AP: only known in RUN. */
	if (ic->ic_state == IEEE80211_S_RUN && ic->ic_bss != NULL &&
	    ic->ic_bss->ni_chan != IEEE80211_CHAN_ANYC)
		mwx_node_chan_width(ic->ic_bss, &bw, &center);
	switch (bw) {
	case CMD_CBW_160MHZ:
		return 85;
	case CMD_CBW_80MHZ:
		return 88;
	case CMD_CBW_40MHZ:
		return 91;
	default:
		return 94;
	}
}

bool
MtkMwx::is5GBandSupport()
{
	return com != NULL && com->sc_capa.has_5ghz;
}

int
MtkMwx::getTxNSS()
{
	return (com != NULL && com->sc_capa.num_streams > 0) ? com->sc_capa.num_streams : 1;
}

const char *
MtkMwx::getFirmwareName()
{
	const char *rompatch, *fw;

	if (com == NULL)
		return MT7922_FIRMWARE_WM;
	mwx_hw_firmware(com->sc_hwtype, &rompatch, &fw);
	return fw;
}

UInt32
MtkMwx::supportedFeatures()
{
	return kIONetworkFeatureMultiPages;
}

const char *
MtkMwx::getFirmwareCountryCode()
{
	static char alpha2[3] = "00";
	if (com != NULL) {
		alpha2[0] = com->sc_alpha2[0];
		alpha2[1] = com->sc_alpha2[1];
		alpha2[2] = '\0';
	}
	return alpha2;
}

uint32_t
MtkMwx::getTxQueueSize()
{
	return 256;
}

uint32_t
MtkMwx::getRxRateKbps()
{
	return mwx_rxr_last_kbps;
}

int
MtkMwx::getRxRateMcs()
{
	return mwx_rxr_last_mcs;
}

void
MtkMwx::accountOutputStart(uint32_t reason)
{
	if (com == NULL)
		return;
	com->sc_output_start_cnt++;
	switch (reason) {
	case 1: com->sc_output_noqueue_cnt++; break;
	case 2: com->sc_output_active_cnt++; break;
	case 3: com->sc_output_capacity_cnt++; break;
	case 4: com->sc_output_dequeue_fail_cnt++; break;
	case 5: com->sc_output_post_active_cnt++; break;
	default: com->sc_output_kick_fail_cnt++; break;
	}
}

void
MtkMwx::clearScanningFlags()
{
	if (com)
		com->sc_flags &= ~(MWX_FLAG_SCANNING | MWX_FLAG_BGSCAN);
}

IOReturn
MtkMwx::setMulticastList(IOEthernetAddress *addr, int count)
{
	return kIOReturnSuccess;
}

void
MtkMwx::releaseAll()
{
	MWX_LOG("HAL releaseAll\n");
	if (com) {
		mwx_rx_amsdu_purge(com);
		mwx_rx_pool_free(com);
		if (com->sc_rx_pool_lock) {
			IOSimpleLockFree(com->sc_rx_pool_lock);
			com->sc_rx_pool_lock = NULL;
		}
		if (com->sc_rx_amsdu_lock) {
			IOLockFree(com->sc_rx_amsdu_lock);
			com->sc_rx_amsdu_lock = NULL;
		}
		com->sc_rx_amsdu_gate = NULL;
		mwx_free_mem(com);
		com = NULL;
	}
	if (pci) {
		mwx_free_mem(pci);
		pci = NULL;
	}
	if (gMwxHal == this)
		gMwxHal = NULL;
}

void
MtkMwx::free()
	{
		MWX_LOG("HAL free\n");
		releaseAll();
		MtkHalService::free();
	}

static int mwx_send_assoc_status(struct mwx_softc *sc, uint32_t cmd, void *data, size_t len)
{
    int seq, rv;
    uint32_t status = 0;
    rv = mwx_mcu_send_msg(sc, cmd, data, len, &seq);
    if (rv) return rv;
    rv = mwx_mcu_wait_resp_int(sc, cmd, seq, &status);
    MWX_DEV_LOG(sc, "assoc_status cmd=%x transport=%d firmware=%u\n", cmd, rv, status);
    return rv ? rv : (status ? EIO : 0);
}

static void
mwx_add_sta_he_tlvs(struct mbuf *m, uint16_t *tlvnum,
    struct ieee80211_node *ni)
{
	const struct ieee80211_he_cap_elem *elem = &ni->ni_he_cap_elem;
	const struct ieee80211_he_mcs_nss_supp *mcs = &ni->ni_he_mcs_nss_supp;
	struct sta_rec_he *he;
	struct sta_rec_he_v2 *v2;
	uint32_t cap = 0;

	he = (struct sta_rec_he *)mwx_append_tlv(m, tlvnum,
	    STA_REC_HE, sizeof(*he));
	if (elem->mac_cap_info[0] & IEEE80211_HE_MAC_CAP0_HTC_HE)
		cap |= STA_REC_HE_CAP_HTC;
	if (elem->mac_cap_info[3] & IEEE80211_HE_MAC_CAP3_OMI_CONTROL)
		cap |= STA_REC_HE_CAP_OM;
	if (elem->mac_cap_info[4] & IEEE80211_HE_MAC_CAP4_AMDSU_IN_AMPDU)
		cap |= STA_REC_HE_CAP_AMSDU_IN_AMPDU;
	if (elem->phy_cap_info[0] &
	    (IEEE80211_HE_PHY_CAP0_CHANNEL_WIDTH_SET_RU_MAPPING_IN_2G |
		 IEEE80211_HE_PHY_CAP0_CHANNEL_WIDTH_SET_RU_MAPPING_IN_5G))
		cap |= STA_REC_HE_CAP_BW20_RU242_SUPPORT;
	if (elem->phy_cap_info[1] &
	    IEEE80211_HE_PHY_CAP1_LDPC_CODING_IN_PAYLOAD)
		cap |= STA_REC_HE_CAP_LDPC;
	if (elem->phy_cap_info[1] &
	    IEEE80211_HE_PHY_CAP1_HE_LTF_AND_GI_FOR_HE_PPDUS_0_8US)
		cap |= STA_REC_HE_CAP_SU_PPDU_1LTF_8US_GI;
	if (elem->phy_cap_info[6] &
	    IEEE80211_HE_PHY_CAP6_PARTIAL_BW_EXT_RANGE)
		cap |= STA_REC_HE_CAP_PARTIAL_BW_EXT_RANGE;
	if (elem->phy_cap_info[7] &
	    IEEE80211_HE_PHY_CAP7_HE_SU_MU_PPDU_4XLTF_AND_08_US_GI)
		cap |= STA_REC_HE_CAP_SU_MU_PPDU_4LTF_8US_GI;
	if (elem->phy_cap_info[9] &
	    IEEE80211_HE_PHY_CAP9_NON_TRIGGERED_CQI_FEEDBACK)
		cap |= STA_REC_HE_CAP_TRIG_CQI_FK;
	if (elem->phy_cap_info[9] &
	    IEEE80211_HE_PHY_CAP9_TX_1024_QAM_LESS_THAN_242_TONE_RU)
		cap |= STA_REC_HE_CAP_TX_1024QAM_UNDER_RU242;
	if (elem->phy_cap_info[9] &
	    IEEE80211_HE_PHY_CAP9_RX_1024_QAM_LESS_THAN_242_TONE_RU)
		cap |= STA_REC_HE_CAP_RX_1024QAM_UNDER_RU242;
	he->he_cap = htole32(cap);
	he->t_frame_dur = (elem->mac_cap_info[1] &
	    IEEE80211_HE_MAC_CAP1_TF_MAC_PAD_DUR_MASK) >> 2;
	he->max_ampdu_exp = (elem->mac_cap_info[3] &
	    IEEE80211_HE_MAC_CAP3_MAX_AMPDU_LEN_EXP_MASK) >> 3;
	he->bw_set = (elem->phy_cap_info[0] &
	    IEEE80211_HE_PHY_CAP0_CHANNEL_WIDTH_SET_MASK) >> 1;
	he->device_class = (elem->phy_cap_info[1] &
	    IEEE80211_HE_PHY_CAP1_DEVICE_CLASS_A) ? 1 : 0;
	he->punc_pream_rx = elem->phy_cap_info[1] &
	    IEEE80211_HE_PHY_CAP1_PREAMBLE_PUNC_RX_MASK;
	he->pkt_ext = 2;
	he->max_nss_mcs[CMD_HE_MCS_BW80] = mcs->rx_mcs_80;
	he->max_nss_mcs[CMD_HE_MCS_BW160] = mcs->rx_mcs_160;
	he->max_nss_mcs[CMD_HE_MCS_BW8080] = mcs->rx_mcs_80p80;

	v2 = (struct sta_rec_he_v2 *)mwx_append_tlv(m, tlvnum,
	    STA_REC_HE_V2, sizeof(*v2));
	memcpy(v2->he_mac_cap, elem->mac_cap_info, sizeof(v2->he_mac_cap));
	memcpy(v2->he_phy_cap, elem->phy_cap_info, sizeof(v2->he_phy_cap));
	v2->pkt_ext = 2;
	v2->max_nss_mcs[CMD_HE_MCS_BW80] = mcs->rx_mcs_80;
	v2->max_nss_mcs[CMD_HE_MCS_BW160] = mcs->rx_mcs_160;
	v2->max_nss_mcs[CMD_HE_MCS_BW8080] = mcs->rx_mcs_80p80;
}

/* Publish the associated peer without resetting WTBL or key state. */
int mt7921_mcu_set_associated_peer(struct mwx_softc *sc)
{
    struct ieee80211_node *ni = sc->sc_ic.ic_bss;
    if (!ni || IEEE80211_AID(ni->ni_associd) == 0) return EINVAL;
    struct mbuf *m = mwx_alloc_sta_req_tlv(sizeof(struct sta_req_hdr));
    if (!m) return ENOBUFS;
    uint16_t tlvnum = 0;
    mt7921_mcu_add_basic_tlv(m, &tlvnum, sc, ni, 1, 0);
    if (ni->ni_flags & IEEE80211_NODE_HT) {
        /* 802.11n: peer HT caps, PHY type/A-MPDU params, RA MCS mask. */
        struct sta_rec_ht_v1 *ht = (struct sta_rec_ht_v1 *)mwx_append_tlv(m,
            &tlvnum, STA_REC_HT, sizeof(*ht));
        ht->ht_cap = htole16(ni->ni_htcaps);
        struct sta_rec_phy *phy = (struct sta_rec_phy *)mwx_append_tlv(m,
            &tlvnum, STA_REC_PHY, sizeof(*phy));
        phy->basic_rate = htole16(0x0150);
        phy->phy_type = mt7921_get_phy_mode_v2(sc, ni);
        phy->ampdu = ni->ni_ampdu_param & 0x1f;
        phy->rcpi = 0xdc;
        struct sta_rec_ra_info *ra = (struct sta_rec_ra_info *)mwx_append_tlv(m,
            &tlvnum, STA_REC_RA, sizeof(*ra));
        ra->legacy = htole16(RA_LEGACY_OFDM);
        for (int i = 0; i < HT_MCS_MASK_NUM; i++)
            ra->rx_mcs_bitmask[i] = ni->ni_rxmcs[i] &
                (i < 2 ? sc->sc_ic.ic_sup_mcs[i] : 0);
        MWX_DEV_LOG(sc, "assoc_peer HT htcaps=0x%04x ampdu=0x%02x phy=0x%02x mcs=%02x%02x\n",
            ni->ni_htcaps, ni->ni_ampdu_param, phy->phy_type,
            ra->rx_mcs_bitmask[1], ra->rx_mcs_bitmask[0]);
    }
    if ((ni->ni_flags & IEEE80211_NODE_HT) && sc->sc_tx_amsdu) {
        /* Hardware TX A-MSDU (Linux mt76_connac_mcu_sta_amsdu_tlv). */
        struct sta_rec_amsdu *amsdu = (struct sta_rec_amsdu *)mwx_append_tlv(m,
            &tlvnum, STA_REC_HW_AMSDU, sizeof(*amsdu));
        amsdu->max_amsdu_num = 8;
        amsdu->amsdu_en = 1;
        amsdu->max_mpdu_size = 0;	/* A-MSDUs up to 3839 bytes */
    }
    if (ni->ni_flags & IEEE80211_NODE_VHT) {
        struct sta_rec_vht_v1 *vht = (struct sta_rec_vht_v1 *)mwx_append_tlv(m,
            &tlvnum, STA_REC_VHT, sizeof(*vht));
        /* net80211 already intersects peer and local VHT capabilities. */
        vht->vht_cap = htole32(ni->ni_vhtcaps);
        vht->vht_rx_mcs_map = htole16(ni->ni_vht_mcsinfo.rx_mcs_map);
        vht->vht_tx_mcs_map = htole16(ni->ni_vht_mcsinfo.tx_mcs_map);
    }
    if ((ni->ni_flags & IEEE80211_NODE_HE) && sc->sc_he_enabled)
        mwx_add_sta_he_tlvs(m, &tlvnum, ni);
    struct sta_rec_state *state = (struct sta_rec_state *)mwx_append_tlv(m, &tlvnum, STA_REC_STATE, sizeof(*state));
    state->state = 2; /* MT76_STA_INFO_STATE_ASSOC: NONE=0, AUTH=1, ASSOC=2. */
    if (ni->ni_flags & IEEE80211_NODE_VHT) {
        /* bandwidth (0=20, 2=80) | (rx nss - 1) << 4 */
        uint8_t bw, center;
        mwx_node_chan_width(ni, &bw, &center);
        state->vht_opmode = (bw == CMD_CBW_160MHZ ? 3 :
            bw == CMD_CBW_80MHZ ? 2 :
            bw == CMD_CBW_40MHZ ? 1 : 0) |
            ((sc->sc_capa.num_streams > 1 ? 1 : 0) << 4);
        MWX_DEV_LOG(sc, "assoc_peer VHT vhtcaps=0x%08x rxmcs=0x%04x txmcs=0x%04x chw=%d center=%u opmode=0x%02x\n",
            ni->ni_vhtcaps, ni->ni_vht_mcsinfo.rx_mcs_map,
            ni->ni_vht_mcsinfo.tx_mcs_map, ni->ni_chw, center,
            state->vht_opmode);
    }
    if (ni->ni_flags & IEEE80211_NODE_HT) {
        /* Refresh after association, without resetting the installed keys. */
        struct sta_rec_wtbl *wtbl = (struct sta_rec_wtbl *)mwx_append_tlv(
            m, &tlvnum, STA_REC_WTBL, sizeof(*wtbl));
        uint16_t wnum = 0;
        const uint16_t wcid = ((struct mwx_node *)ni)->wcid;
        wtbl->wlan_idx_lo = wcid & 0xff;
        wtbl->wlan_idx_hi = wcid >> 8;
        wtbl->operation = WTBL_SET;
        int wlen = mt7921_mcu_wtbl_ht_tlv(m, &wnum, sc, ni);
        wtbl->tlv_num = htole16(wnum);
        wtbl->len = htole16(sizeof(*wtbl) + wlen);
    }
    m = mwx_fill_sta_req_hdr(m, &sc->sc_vif, sc->sc_vif.omac_idx, ((struct mwx_node *)ni)->wcid, tlvnum);
    if (!m) return ENOBUFS;
    int seq, rv = mwx_mcu_send_mbuf(sc, MCU_UNI_CMD_STA_REC_UPDATE, m, &seq);
    uint32_t status = 0;
    if (!rv) rv = mwx_mcu_wait_resp_int(sc, MCU_UNI_CMD_STA_REC_UPDATE, seq, &status);
    MWX_DEV_LOG(sc, "assoc_peer aid=%u state=2 transport=%d firmware=%u\n", IEEE80211_AID(ni->ni_associd), rv, status);
    return rv ? rv : (status ? EIO : 0);
}

/*
 * RX Block Ack session (we are the recipient). The firmware must know about
 * it to answer the AP's A-MPDUs with BlockAck; net80211 reorders in software.
 * Called from RX processing, so do not wait for the MCU response here.
 */
static int
mwx_mcu_wtbl_ba(struct mwx_softc *sc, struct ieee80211_node *ni, uint8_t tid,
    uint16_t ssn, uint16_t winsize, int enable, int tx)
{
	struct mwx_node *mn = (struct mwx_node *)ni;
	struct mbuf *m = mwx_alloc_sta_req_tlv(sizeof(struct sta_req_hdr));
	uint16_t tlvnum = 0, wnum = 0;
	if (m == NULL)
		return ENOBUFS;
	struct sta_rec_wtbl *wtbl = (struct sta_rec_wtbl *)mwx_append_tlv(m,
	    &tlvnum, STA_REC_WTBL, sizeof(*wtbl));
	wtbl->wlan_idx_lo = mn->wcid & 0xff;
	wtbl->wlan_idx_hi = mn->wcid >> 8;
	wtbl->operation = WTBL_SET;
	struct wtbl_ba *ba = (struct wtbl_ba *)mwx_append_tlv(m, &wnum,
	    WTBL_BA, sizeof(*ba));
	ba->tid = tid;
	ba->ba_type = tx ? MT_BA_TYPE_ORIGINATOR : MT_BA_TYPE_RECIPIENT;
	ba->ba_winsize = enable ? htole16(winsize) : 0;
	if (tx) {
		ba->sn = enable ? htole16(ssn) : 0;
		ba->ba_en = !!enable;
	} else {
		memcpy(ba->peer_addr, ni->ni_macaddr, IEEE80211_ADDR_LEN);
		ba->rst_ba_tid = tid;
		ba->rst_ba_sel = RST_BA_MAC_TID_MATCH;
		ba->rst_ba_sb = 1;
	}
	wtbl->tlv_num = htole16(wnum);
	wtbl->len = htole16(sizeof(*wtbl) + sizeof(*ba));
	m = mwx_fill_sta_req_hdr(m, &sc->sc_vif, sc->sc_vif.omac_idx,
	    mn->wcid, tlvnum);
	if (m == NULL)
		return ENOBUFS;
	return mwx_mcu_send_mbuf(sc, MCU_UNI_CMD_STA_REC_UPDATE, m, NULL);
}

static int
mwx_mcu_sta_ba(struct mwx_softc *sc, struct ieee80211_node *ni, uint8_t tid,
    uint16_t ssn, uint16_t winsize, int enable, int tx, int amsdu)
{
	struct mwx_node *mn = (struct mwx_node *)ni;
	struct sta_rec_ba *ba;
	struct mbuf *m;
	uint16_t tlvnum = 0;

	if (ni == NULL || tid >= 8 || (enable &&
	    (winsize == 0 || winsize > IEEE80211_BA_MAX_WINSZ)))
		return EINVAL;
	/* Program the hardware table before updating firmware session state.
	 * Do not wait here: RX processing shares the MCU completion work loop. */
	if (sc->sc_wtbl_ba) {
		int rv = mwx_mcu_wtbl_ba(sc, ni, tid, ssn, winsize, enable, tx);
		MWX_DEV_LOG(sc, "%s WTBL BA tid=%u enable=%d win=%u queued=%d\n",
		    tx ? "tx" : "rx", tid, enable, winsize, rv);
		if (rv)
			return rv;
	}
	m = mwx_alloc_sta_req_tlv(sizeof(struct sta_req_hdr));
	if (m == NULL)
		return ENOBUFS;
	ba = (struct sta_rec_ba *)mwx_append_tlv(m, &tlvnum, STA_REC_BA,
	    sizeof(*ba));
	ba->tid = tid;
	ba->ba_type = tx ? MT_BA_TYPE_ORIGINATOR : MT_BA_TYPE_RECIPIENT;
	ba->amsdu = amsdu ? 1 : 0;
	ba->ba_en = enable ? (1U << tid) : 0;
	ba->ssn = htole16(ssn);
	ba->winsize = htole16(winsize);
	m = mwx_fill_sta_req_hdr(m, &sc->sc_vif, sc->sc_vif.omac_idx,
	    mn->wcid, tlvnum);
	if (m == NULL)
		return ENOBUFS;
	return mwx_mcu_send_mbuf(sc, MCU_UNI_CMD_STA_REC_UPDATE, m, NULL);
}

int
mwx_ampdu_rx_start(struct ieee80211com *ic, struct ieee80211_node *ni,
    uint8_t tid)
{
	struct mwx_softc *sc = (struct mwx_softc *)ic->ic_softc;
	struct ieee80211_rx_ba *ba = &ni->ni_rx_ba[tid];
	int rv;

	mwx_rx_amsdu_purge(sc, tid);
	const bool crypto_supported = !(ic->ic_flags & (IEEE80211_F_RSNON | IEEE80211_F_WEPON)) ||
	    ni->ni_pairwise_key.k_cipher == IEEE80211_CIPHER_CCMP;
	const bool amsdu = sc->sc_rx_amsdu_enabled && crypto_supported &&
	    (ba->ba_params & IEEE80211_ADDBA_AMSDU);
	if (!amsdu)
		ba->ba_params &= ~IEEE80211_ADDBA_AMSDU;
	rv = mwx_mcu_sta_ba(sc, ni, tid, ba->ba_winstart, ba->ba_winsize, 1, 0, amsdu);
	MWX_DEV_LOG(sc, "rx BA start tid=%u ssn=%u win=%u amsdu=%d rv=%d\n", tid,
	    ba->ba_winstart, ba->ba_winsize, amsdu, rv);
	return rv;
}

void
mwx_ampdu_rx_stop(struct ieee80211com *ic, struct ieee80211_node *ni,
    uint8_t tid)
{
	struct mwx_softc *sc = (struct mwx_softc *)ic->ic_softc;
	int rv;

	mwx_rx_amsdu_purge(sc, tid);
	rv = mwx_mcu_sta_ba(sc, ni, tid, 0, 0, 0, 0, 0);
	MWX_DEV_LOG(sc, "rx BA stop tid=%u rv=%d\n", tid, rv);
}

/*
 * TX Block Ack session (we are the originator): the firmware aggregates the
 * TID into A-MPDUs and syncs its hardware sequence number to the SSN we sent
 * in the ADDBA request.
 */
int
mwx_ampdu_tx_start(struct ieee80211com *ic, struct ieee80211_node *ni,
    uint8_t tid)
{
	struct mwx_softc *sc = (struct mwx_softc *)ic->ic_softc;
	struct ieee80211_tx_ba *ba = &ni->ni_tx_ba[tid];
	uint16_t win = ba->ba_winsize;
	int amsdu, rv;

	if (win == 0 || win > 64)
		win = 64;
	amsdu = sc->sc_tx_amsdu && (ba->ba_params & IEEE80211_ADDBA_AMSDU);
	rv = mwx_mcu_sta_ba(sc, ni, tid, ba->ba_winstart, win, 1, 1, amsdu);
	MWX_DEV_LOG(sc, "tx BA start tid=%u ssn=%u win=%u amsdu=%d rv=%d\n",
	    tid, ba->ba_winstart, win, amsdu, rv);
	return rv;
}

void
mwx_ampdu_tx_stop(struct ieee80211com *ic, struct ieee80211_node *ni,
    uint8_t tid)
{
	struct mwx_softc *sc = (struct mwx_softc *)ic->ic_softc;
	int rv;

	rv = mwx_mcu_sta_ba(sc, ni, tid, 0, 0, 0, 1, 0);
	MWX_DEV_LOG(sc, "tx BA stop tid=%u rv=%d\n", tid, rv);
}

static int
mwx_send_bss_he_info(struct mwx_softc *sc, struct ieee80211_node *ni)
{
	struct mwx_vif *mvif = &sc->sc_vif;
	struct {
		struct {
			uint8_t bss_idx;
			uint8_t pad[3];
		} __packed hdr;
		struct bss_info_uni_he he;
		struct bss_info_uni_bss_color color;
	} __packed req = {};
	/*
	 * PE duration, RTS threshold and BSS color come from the AP's HE
	 * Operation element, as mac80211 fills bss_conf for mt76; the
	 * defaults apply when the AP leaves them at zero. The color goes into
	 * every HE PPDU the station sends, trigger-based ones included: with
	 * it off, the AP discards them as another BSS's.
	 */
	uint32_t op = ni->ni_he_oper_params;
	uint8_t pe = op & IEEE80211_HE_OPERATION_DFLT_PE_DURATION_MASK;
	uint16_t rts = (op & IEEE80211_HE_OPERATION_RTS_THRESHOLD_MASK) >>
	    IEEE80211_HE_OPERATION_RTS_THRESHOLD_OFFSET;
	int rv;

	req.hdr.bss_idx = mvif->idx;
	req.he.tag = htole16(UNI_BSS_INFO_HE_BASIC);
	req.he.len = htole16(sizeof(req.he));
	req.he.he_pe_duration = pe ? pe : 4;
	req.he.he_rts_thres = htole16(rts ? rts : 1023);
	req.he.max_nss_mcs[CMD_HE_MCS_BW80] =
	    sc->sc_ic.ic_he_mcs_nss_supp.tx_mcs_80;
	req.he.max_nss_mcs[CMD_HE_MCS_BW160] =
	    sc->sc_ic.ic_he_mcs_nss_supp.tx_mcs_160;
	req.he.max_nss_mcs[CMD_HE_MCS_BW8080] =
	    sc->sc_ic.ic_he_mcs_nss_supp.tx_mcs_80p80;
	req.color.tag = htole16(UNI_BSS_INFO_BSS_COLOR);
	req.color.len = htole16(sizeof(req.color));
	req.color.enable = !(op & IEEE80211_HE_OPERATION_BSS_COLOR_DISABLED);
	req.color.bss_color = (op & IEEE80211_HE_OPERATION_BSS_COLOR_MASK) >>
	    IEEE80211_HE_OPERATION_BSS_COLOR_OFFSET;
	rv = mwx_send_assoc_status(sc, MCU_UNI_CMD_BSS_INFO_UPDATE,
	    &req, sizeof(req));
	MWX_DEV_LOG(sc,
	    "BSS HE enable=%d pe=%u rts=%u color=%u/%u mcs80=0x%04x mcs160=0x%04x rv=%d\n",
	    1, req.he.he_pe_duration, le16toh(req.he.he_rts_thres),
	    req.color.bss_color, req.color.enable,
	    le16toh(req.he.max_nss_mcs[CMD_HE_MCS_BW80]),
	    le16toh(req.he.max_nss_mcs[CMD_HE_MCS_BW160]), rv);
	return rv;
}

int
mt7921_mcu_set_associated_bss(struct mwx_softc *sc, int associated)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_node *ni = ic->ic_bss;
	struct mwx_vif *mvif = &sc->sc_vif;
	struct {
		uint8_t bss_idx;
		uint8_t pad[3];
		struct mt76_connac_bss_basic_tlv basic;
	} __packed bss = {};
	struct {
		uint8_t bss_idx;
		uint8_t pad[3];
		struct {
			uint16_t tag, len;
			uint8_t control_channel, center_chan, center_chan2, bw;
			uint8_t tx_streams, rx_streams, short_st, ht_op_info;
			uint8_t sco, band, pad[2];
		} __packed rlm;
	} __packed channel = {};
	int rv, is_2g;

	if (ic->ic_opmode != IEEE80211_M_STA)
		return 0;
	if (ni == NULL || ni->ni_chan == NULL ||
	    ni->ni_chan == IEEE80211_CHAN_ANYC)
		return EINVAL;
	uint8_t op_bw, op_center;
	mwx_node_chan_width(ni, &op_bw, &op_center);
	is_2g = IEEE80211_IS_CHAN_2GHZ(ni->ni_chan);
	bss.bss_idx = mvif->idx;
	bss.basic.tag = htole16(UNI_BSS_INFO_BASIC);
	bss.basic.len = htole16(sizeof(bss.basic));
	bss.basic.active = 1;
	bss.basic.omac_idx = mvif->omac_idx;
	bss.basic.hw_bss_idx = mvif->omac_idx > EXT_BSSID_START ?
	    HW_BSSID_0 : mvif->omac_idx;
	bss.basic.band_idx = mvif->band_idx;
	bss.basic.conn_type = htole32(STA_TYPE_STA | NETWORK_INFRA);
	bss.basic.conn_state = associated ? 0 : 1;
	bss.basic.wmm_idx = mvif->wmm_idx;
	memcpy(bss.basic.bssid, ni->ni_bssid, ETHER_ADDR_LEN);
	bss.basic.bmc_tx_wlan_idx = htole16(mvif->vif_mn.wcid);
	bss.basic.sta_idx = htole16(mvif->vif_mn.wcid);
	bss.basic.bcn_interval = htole16(ni->ni_intval);
	bss.basic.dtim_period = ni->ni_dtimperiod;
	/* BSS phymode uses A/B/G bits; nonht_basic_phy uses PHY_TYPE bits. */
	bss.basic.phymode = is_2g ? ((1U << 1) | (1U << 2)) : (1U << 0);
	if (ni->ni_flags & IEEE80211_NODE_HT)
		bss.basic.phymode |= is_2g ? (1U << 3) /* GN */ :
		    (1U << 4) /* AN */;
	if (ni->ni_flags & IEEE80211_NODE_VHT)
		bss.basic.phymode |= (1U << 5); /* AC */
	if ((ni->ni_flags & IEEE80211_NODE_HE) && sc->sc_he_enabled)
		bss.basic.phymode |= (1U << 6); /* AX */
	bss.basic.nonht_basic_phy = htole16(is_2g ?
	    PHY_TYPE_BIT_HR_DSSS | PHY_TYPE_BIT_ERP : PHY_TYPE_BIT_OFDM);
	rv = mwx_send_assoc_status(sc, MCU_UNI_CMD_BSS_INFO_UPDATE, &bss, sizeof(bss));
	MWX_DEV_LOG(sc, "associated BSS enable=%d channel=%u beacon=%u dtim=%u rv=%d\n",
	    associated, ieee80211_mhz2ieee(ni->ni_chan->ic_freq, 0), ni->ni_intval, ni->ni_dtimperiod, rv);
	if (rv != 0 || !associated)
		return rv;
	if ((ni->ni_flags & IEEE80211_NODE_HE) && sc->sc_he_enabled) {
		rv = mwx_send_bss_he_info(sc, ni);
		if (rv != 0)
			return rv;
	}

	channel.bss_idx = mvif->idx;
	channel.rlm.tag = htole16(UNI_BSS_INFO_RLM);
	channel.rlm.len = htole16(sizeof(channel.rlm));
	channel.rlm.control_channel = ieee80211_mhz2ieee(ni->ni_chan->ic_freq, 0);
	channel.rlm.center_chan = op_center;
	channel.rlm.bw = op_bw;
	if (op_bw != CMD_CBW_20MHZ) {
		channel.rlm.ht_op_info = 4;	/* 40 MHz+ allowed */
		if (channel.rlm.control_channel < channel.rlm.center_chan)
			channel.rlm.sco = 1;	/* SCA */
		else if (channel.rlm.control_channel > channel.rlm.center_chan)
			channel.rlm.sco = 3;	/* SCB */
	}
	channel.rlm.tx_streams = sc->sc_capa.num_streams;
	channel.rlm.rx_streams = sc->sc_capa.antenna_mask;
	channel.rlm.short_st = 1;
	channel.rlm.band = is_2g ? 0 : 1;
	rv = mwx_send_assoc_status(sc, MCU_UNI_CMD_BSS_INFO_UPDATE,
	    &channel, sizeof(channel));
	MWX_DEV_LOG(sc, "associated BSS RLM channel=%u band=%u rv=%d\n",
	    channel.rlm.control_channel, channel.rlm.band, rv);
	return rv;
}

/*
 * System sleep. The M.2 card is powered off in S3, so the firmware, the
 * DMA ring registers and the MSI state are gone on wake. Leave the network
 * (deauth while the radio still works), mask interrupts and ignore the
 * hardware until resume().
 */
void
MtkMwx::suspend()
{
	struct mwx_softc *sc = com;
	struct ifnet *ifp;

	if (sc == NULL || sc->sc_suspended)
		return;
	MwxRxAmsduTransition transition(sc);
	if (!transition.ok())
		return;
	ifp = &sc->sc_ic.ic_if;
	MWX_DEV_LOG(sc, "suspend flags=0x%x fw_loaded=%d\n", ifp->if_flags,
	    sc->sc_fw_loaded);
	if (ifp->if_flags & IFF_RUNNING)
		mwx_stop(ifp);
	mwx_write(sc, MT_WFDMA0_HOST_INT_ENA, 0);
	sc->sc_suspended = 1;
}

/*
 * System wake: take the chip back from the firmware (with the same WFSYS
 * reset / PCIe FLR fallback as attach), then reload the firmware through
 * the hard reset task (the DMA stall recovery path). An interface that was
 * up is brought up again by that task. The native Wi-Fi front end takes
 * its interface down for the sleep: with up, it is marked up again here,
 * only when this queues the reset task, so that the task initializes it
 * and nothing else does (enable() skips an interface already up).
 * Otherwise it is initialized when the front end enables it again.
 */
void
MtkMwx::resume(bool up)
{
	struct mwx_softc *sc = com;

	if (sc == NULL || !sc->sc_suspended)
		return;
	sc->sc_mcu_mute = 0;	/* the card lost power: a new firmware will answer */
	MWX_DEV_LOG(sc, "resume lpctl=0x%08x\n", mwx_read(sc, MT_CONN_ON_LPCTL));
	if (mwx_mcu_fw_pmctrl(sc) != 0 || mwx_mcu_drv_pmctrl(sc) != 0) {
		MWX_DEV_LOG(sc, "resume: driver own failed, trying WFSYS reset\n");
		mwx_wfsys_reset(sc);
		if ((mwx_mcu_fw_pmctrl(sc) != 0 || mwx_mcu_drv_pmctrl(sc) != 0) &&
		    pci != NULL && pci->pa_tag != NULL) {
			MWX_DEV_LOG(sc, "resume: trying PCIe FLR\n");
			if (mwx_pci_flr(sc, pci->pa_tag) == 0) {
				mwx_mcu_fw_pmctrl(sc);
				mwx_mcu_drv_pmctrl(sc);
			}
		}
	}
	mwx_set(sc, MT_MCU2HOST_SW_INT_ENA, MT_MCU_CMD_WAKE_RX_PCIE);
	mwx_write(sc, MT_WFDMA0_HOST_INT_ENA, 0);
	mwx_write(sc, MT_PCIE_MAC_INT_ENABLE, 0xff);
	sc->sc_fw_loaded = 0;
	sc->sc_suspended = 0;
	sc->sc_hard_reset = 1;
	/* mwx_reset() queues the task unless one is pending already. */
	if (up && !sc->sc_resetting && !sc->sc_detaching)
		sc->sc_ic.ic_if.if_flags |= IFF_UP;
	mwx_reset(sc);
	MWX_DEV_LOG(sc, "resume: firmware reload scheduled flags=0x%x\n",
	    sc->sc_ic.ic_if.if_flags);
}

/*
 * System restart or power off. Nothing stopped the hardware on that path:
 * the interrupts and the DMA stayed enabled, with the rings pointing at
 * memory that belongs to the next system. A restart does not cut power to
 * an M.2 card, so whatever boots next, this driver included, finds the
 * chip as it is left here.
 *
 * Leave it the way Linux mt7921_pci_suspend() leaves a card that keeps
 * power: interface stopped, interrupts masked, DMA off, the chip handed
 * back to its firmware. Bus mastering goes off too, so the card cannot
 * write to memory at all. The firmware keeps running; the next attach
 * takes the chip back and resets it. Once the interface is stopped no MCU
 * command is sent: nothing may wait for the firmware from there on.
 */
void
MtkMwx::shutdown()
{
	struct mwx_softc *sc = com;

	/* Asleep: the card has no power and its registers are off limits. */
	if (sc == NULL || sc->sc_memh == 0 || sc->sc_suspended)
		return;
	MWX_DEV_LOG(sc, "shutdown lpctl=0x%08x\n", mwx_read(sc, MT_CONN_ON_LPCTL));
	/* As before a system sleep: leave the network, mask the interrupts. */
	suspend();
	if (!sc->sc_suspended)
		return;		/* a teardown already owns the card */
	sc->sc_mcu_mute = 1;	/* a late command must not wait for an answer */
	mwx_write(sc, MT_PCIE_MAC_INT_ENABLE, 0);
	if (mwx_mcu_drv_pmctrl(sc) != 0)
		MWX_DEV_LOG(sc, "shutdown: driver own failed\n");
	if (mwx_dma_disable(sc, 0) != 0)
		MWX_DEV_LOG(sc, "shutdown: DMA still busy\n");
	if (mwx_mcu_fw_pmctrl(sc) != 0)
		MWX_DEV_LOG(sc, "shutdown: firmware own failed\n");
	if (pci != NULL && pci->pa_tag != NULL)
		pci->pa_tag->setBusMasterEnable(false);
	sc->sc_fw_loaded = 0;
	MWX_DEV_LOG(sc, "shutdown done lpctl=0x%08x\n",
	    mwx_read(sc, MT_CONN_ON_LPCTL));
}

void
MtkMwx::debugPoll()
{
	struct mwx_softc *sc = com;
	struct ieee80211com *ic;
	static uint32_t polls, last_intr_calls;
	static uint32_t last_txq_dma = 0xffffffff, last_mcu_dma = 0xffffffff;
	static uint32_t txq_stuck, mcu_stuck, last_reset_poll;
	static int stall_dumped;
	uint32_t sta, ena, lp, mcu;
	uint32_t tq_cpu, tq_dma, mq_cpu, mq_dma;
	int verbose;

	if (sc == NULL || !sc->sc_fw_loaded || sc->sc_resetting ||
	    sc->sc_suspended)
		return;
	ic = &sc->sc_ic;
	mwx_rx_refill_retry(sc);
	if (sc->sc_rx_amsdu_enabled) {
		uint64_t now_ns = 0;
		absolutetime_to_nanoseconds(mach_absolute_time(), &now_ns);
		mwx_rx_amsdu_expire(sc, now_ns / 1000000);
	}
	polls++;

	/*
	 * net80211 sends AUTH / ASSOC_REQ once and then waits the whole 5 s
	 * management timer before rescanning every channel. A single lost
	 * frame (seen on 2.4 GHz: AUTH answered, ASSOC response never came)
	 * thus cost ~20 s at boot. Retransmit like mac80211 does, once per
	 * poll, up to three times.
	 */
	{
		static int retry_state = -1, retries;

		if (ic->ic_opmode == IEEE80211_M_STA && ic->ic_mgt_timer != 0 &&
		    (ic->ic_state == IEEE80211_S_AUTH ||
		    ic->ic_state == IEEE80211_S_ASSOC)) {
			if (retry_state != ic->ic_state) {
				retry_state = ic->ic_state;
				retries = 0;
			} else if (ic->ic_assoc_comeback) {
				/*
				 * Refused for now: net80211 asks again once
				 * the comeback time has passed, and an earlier
				 * request would be refused the same way.
				 */
			} else if (retries < 3) {
				retries++;
				MWX_DEV_LOG(sc, "%s retry %d to %s\n",
				    ic->ic_state == IEEE80211_S_AUTH ?
				    "AUTH" : "ASSOC", retries,
				    ether_sprintf(ic->ic_bss->ni_macaddr));
				if (ic->ic_state == IEEE80211_S_AUTH)
					IEEE80211_SEND_MGMT(ic, ic->ic_bss,
					    IEEE80211_FC0_SUBTYPE_AUTH, 1);
				else
					IEEE80211_SEND_MGMT(ic, ic->ic_bss,
					    IEEE80211_FC0_SUBTYPE_ASSOC_REQ, 0);
			}
		} else
			retry_state = -1;
	}

	sta = mwx_read(sc, MT_WFDMA0_HOST_INT_STA);
	ena = mwx_read(sc, MT_WFDMA0_HOST_INT_ENA);
	lp = mwx_read(sc, MT_CONN_ON_LPCTL);
	mcu = mwx_read(sc, MT_MCU_CMD);
	tq_cpu = mwx_read(sc, sc->sc_txq.mq_regbase + MT_DMA_CPU_IDX);
	tq_dma = mwx_read(sc, sc->sc_txq.mq_regbase + MT_DMA_DMA_IDX);
	mq_cpu = mwx_read(sc, sc->sc_txmcuq.mq_regbase + MT_DMA_CPU_IDX);
	mq_dma = mwx_read(sc, sc->sc_txmcuq.mq_regbase + MT_DMA_DMA_IDX);

	txq_stuck = (tq_cpu != tq_dma && tq_dma == last_txq_dma) ?
	    txq_stuck + 1 : 0;
	mcu_stuck = (mq_cpu != mq_dma && mq_dma == last_mcu_dma) ?
	    mcu_stuck + 1 : 0;
	last_txq_dma = tq_dma;
	last_mcu_dma = mq_dma;

	verbose = (sta & sc->sc_intr_mask) != 0 || polls % 30 == 0 ||
	    txq_stuck == 2 || mcu_stuck == 2;
	if (verbose) {
		struct ifnet *ifp = &ic->ic_if;
		char drops[160];
		size_t off = 0;
		int i;

		MWX_DEV_LOG(sc,
		    "poll state=%d chan=%u intr_calls=%u sta=0x%08x ena=0x%08x lpctl=0x%08x mcu_cmd=0x%08x\n",
		    ic->ic_state,
		    ic->ic_bss ? ieee80211_chan2ieee(ic, ic->ic_bss->ni_chan) : 0,
		    mwx_intr_calls, sta, ena, lp, mcu);
		MWX_DEV_LOG(sc,
		    "poll txq cpu=%u dma=%u | txmcu cpu=%u dma=%u | rxq cpu=%u dma=%u cons=%u | rxmcu cpu=%u dma=%u cons=%u\n",
		    tq_cpu, tq_dma, mq_cpu, mq_dma,
		    mwx_read(sc, sc->sc_rxq.mq_regbase + MT_DMA_CPU_IDX),
		    mwx_read(sc, sc->sc_rxq.mq_regbase + MT_DMA_DMA_IDX),
		    sc->sc_rxq.mq_cons,
		    mwx_read(sc, sc->sc_rxmcuq.mq_regbase + MT_DMA_CPU_IDX),
		    mwx_read(sc, sc->sc_rxmcuq.mq_regbase + MT_DMA_DMA_IDX),
		    sc->sc_rxmcuq.mq_cons);
		drops[0] = 0;
		for (i = 0; i < MWX_RX_DROP_REASONS &&
		    mwx_rx_drop_name[i] != NULL && off < sizeof(drops) - 24; i++)
			off += snprintf(drops + off, sizeof(drops) - off,
			    "%s=%u ", mwx_rx_drop_name[i], mwx_rx_drop_cnt[i]);
		MWX_DEV_LOG(sc,
		    "stats rx_hwdec=%u rx_swdec=%u hwdec_nopn=%u ipkts=%llu ierr=%llu opkts=%llu oerr=%llu hwdrop[%s] dup=%u wrongbss=%u wrongdir=%u notassoc=%u unenc=%u wepfail=%u decap=%u decryptcrc=%u ccmp_replay=%u ccmp_dec=%u tooshort=%u nombuf=%u ba_below=%u ba_above=%u\n",
		    mwx_rx_hwdec_cnt, mwx_rx_swdec_cnt, mwx_rx_hwdec_nopn_cnt,
		    (unsigned long long)(ifp->netStat ? ifp->netStat->inputPackets : 0),
		    (unsigned long long)(ifp->netStat ? ifp->netStat->inputErrors : 0),
		    (unsigned long long)(ifp->netStat ? ifp->netStat->outputPackets : 0),
		    (unsigned long long)(ifp->netStat ? ifp->netStat->outputErrors : 0), drops,
		    ic->ic_stats.is_rx_dup, ic->ic_stats.is_rx_wrongbss,
		    ic->ic_stats.is_rx_wrongdir, ic->ic_stats.is_rx_notassoc,
		    ic->ic_stats.is_rx_unencrypted, ic->ic_stats.is_rx_wepfail,
		    ic->ic_stats.is_rx_decap, ic->ic_stats.is_rx_decryptcrc,
		    ic->ic_stats.is_ccmp_replays, ic->ic_stats.is_ccmp_dec_errs,
		    ic->ic_stats.is_rx_tooshort, ic->ic_stats.is_rx_nombuf,
		    ic->ic_stats.is_ht_rx_frame_below_ba_winstart,
		    ic->ic_stats.is_ht_rx_frame_above_ba_winend);
	}
	if (verbose && (ic->ic_caps & IEEE80211_C_MFP)) {
		struct ieee80211_node *bss = ic->ic_bss;
		int run = bss != NULL && ic->ic_state == IEEE80211_S_RUN;
		uint32_t nf = bss != NULL ? bss->ni_flags : 0;
		/* What the access point advertises: 1 capable, 2 required. */
		int ap = !run ? 0 :
		    (bss->ni_rsncaps & IEEE80211_RSNCAP_MFPR) ? 2 :
		    (bss->ni_rsncaps & IEEE80211_RSNCAP_MFPC) ? 1 : 0;

		MWX_DEV_LOG(sc,
		    "pmf assoc=%d ap=%d negotiated=%d rx=%d tx=%d igtk=%d mgmt_ok=%u mgmt_hwdec=%u unprot_drop=%u bad_drop=%u bip_replay=%u bip_mic=%u saq_start=%u saq_answer=%u saq_timeout=%u saq_reply=%u comeback=%u reauth=%u\n",
		    run, ap,
		    !!(nf & IEEE80211_NODE_MFP),
		    !!(nf & IEEE80211_NODE_RXMGMTPROT),
		    !!(nf & IEEE80211_NODE_TXMGMTPROT),
		    (ic->ic_igtk_kid == 4 || ic->ic_igtk_kid == 5) &&
		    (ic->ic_nw_keys[ic->ic_igtk_kid].k_flags &
		    IEEE80211_KEY_IGTK) ? ic->ic_igtk_kid : 0,
		    ic->ic_stats.is_rx_mgmt_prot, mwx_rx_mgmt_hwdec_cnt,
		    ic->ic_stats.is_rx_mgmt_unprot,
		    ic->ic_stats.is_rx_mgmt_badprot,
		    ic->ic_stats.is_cmac_replays, ic->ic_stats.is_cmac_icv_errs,
		    ic->ic_stats.is_sa_query_started,
		    ic->ic_stats.is_sa_query_answered,
		    ic->ic_stats.is_sa_query_timeouts,
		    ic->ic_stats.is_sa_query_replies,
		    ic->ic_stats.is_assoc_comeback,
		    ic->ic_stats.is_assoc_reauth);
	}
	if (verbose)
		MWX_DEV_LOG(sc,
		    "ba gap_timeout=%u frame_lost=%u slide=%u jump=%u below=%u bar_rx=%u rx_agreements=%u drv_ctl=%u drv_bar=%u htp_ack=%u\n",
		    ic->ic_stats.is_ht_rx_ba_window_gap_timeout,
		    ic->ic_stats.is_ht_rx_ba_frame_lost,
		    ic->ic_stats.is_ht_rx_ba_window_slide,
		    ic->ic_stats.is_ht_rx_ba_window_jump,
		    ic->ic_stats.is_ht_rx_frame_below_ba_winstart,
		    ieee80211_bar_rx_count,
		    ic->ic_stats.is_ht_rx_ba_agreements, mwx_rx_ctl_cnt,
		    mwx_rx_bar_cnt, ic->ic_stats.is_ht_rx_ba_htp_ack);
	if (verbose)
		MWX_DEV_LOG(sc,
		    "tx output_start=%u noqueue=%u active=%u capacity=%u dequeue_fail=%u post_active=%u other=%u kick=%u txwi_free=%u ring_free=%d full=%u oactive=%d tx_ba=%u amsdu=%d\n",
		    sc->sc_output_start_cnt, sc->sc_output_noqueue_cnt,
		    sc->sc_output_active_cnt, sc->sc_output_capacity_cnt,
		    sc->sc_output_dequeue_fail_cnt, sc->sc_output_post_active_cnt,
		    sc->sc_output_kick_fail_cnt, sc->sc_tx_kick_cnt,
		    sc->sc_txwi.mt_nfree, mwx_dma_free_slots(&sc->sc_txq),
		    sc->sc_tx_full_cnt, ifq_is_oactive(&ic->ic_if.if_snd) ? 1 : 0,
		    ic->ic_stats.is_ht_tx_ba_agreements, sc->sc_tx_amsdu);
	if (polls % 30 == 0) {
		MWX_DEV_LOG(sc,
		    "rxdma cumulative bytes=%llu copy_bytes=%llu copy_saved=%llu batches=%llu desc=%llu max_batch=%u bad_len=%u refill_fail=%u refill_recover=%u pending=%u/%u/%u notdone=%llu donemissing=%llu\n",
		    (unsigned long long)sc->sc_rx_dma_bytes,
		    (unsigned long long)sc->sc_rx_dma_copy_bytes,
		    (unsigned long long)sc->sc_rx_dma_copy_saved,
		    (unsigned long long)sc->sc_rx_dma_batches,
		    (unsigned long long)sc->sc_rx_dma_desc,
		    sc->sc_rx_dma_max_batch, sc->sc_rx_dma_bad_len, sc->sc_rx_refill_fail,
		    sc->sc_rx_refill_recover, sc->sc_rxq.mq_refill_pending,
		    sc->sc_rxmcuq.mq_refill_pending, sc->sc_rxfwdlq.mq_refill_pending,
		    (unsigned long long)sc->sc_rx_dma_notdone, (unsigned long long)sc->sc_rx_dma_done_missing);
		MWX_DEV_LOG(sc,
		    "rxpool disabled=%u total=%u free=%u allocs=%u exhausted=%u fallbacks=%u returned=%u\n",
		    sc->sc_rx_pool_disabled, sc->sc_rx_pool_total,
		    sc->sc_rx_pool_free_cnt, sc->sc_rx_pool_allocs,
		    sc->sc_rx_pool_exhausted, sc->sc_rx_pool_fallbacks,
		    sc->sc_rx_pool_returned);
		MWX_DEV_LOG(sc,
		    "rxchain cumulative completed=%llu resumed=%llu dropped=%llu held_bytes=%u/%u/%u held_segs=%u/%u/%u\n",
		    (unsigned long long)sc->sc_rx_dma_chain_completed,
		    (unsigned long long)sc->sc_rx_dma_chain_resumed,
		    (unsigned long long)sc->sc_rx_dma_chain_dropped,
		    sc->sc_rxq.mq_rx_bytes, sc->sc_rxmcuq.mq_rx_bytes,
		    sc->sc_rxfwdlq.mq_rx_bytes, sc->sc_rxq.mq_rx_segments,
		    sc->sc_rxmcuq.mq_rx_segments, sc->sc_rxfwdlq.mq_rx_segments);
		MWX_DEV_LOG(sc,
		    "rxinput cumulative batches=%llu packets=%llu max_batch=%u\n",
		    (unsigned long long)sc->sc_rx_input_batches,
		    (unsigned long long)sc->sc_rx_input_packets,
		    sc->sc_rx_input_max_batch);
		const MtkRxAmsduStats as = mwx_rx_amsdu_stats(sc);
		MWX_DEV_LOG(sc,
		    "rxamsdu cumulative enabled=%d started=%llu completed=%llu subframes=%llu singleton=%llu orphan=%llu mismatch=%llu invalid=%llu timeout=%llu allocfail=%llu pending=%llu\n",
		    sc->sc_rx_amsdu_enabled, as.started, as.completed, as.subframes,
		    as.singleton, as.orphan, as.mismatch, as.invalid, as.timeout,
		    as.allocfail, as.pending);
		MtkRxAmpduStats &a = sc->sc_rx_ampdu_stats;
		MWX_DEV_LOG(sc,
		    "rxampdu observed completed=%llu mpdu=%llu max=%u bins[1=%llu 2-3=%llu 4-7=%llu 8-15=%llu 16-31=%llu 32+=%llu] non_ampdu=%llu missing_group2=%llu deagg_excluded=%llu uncertain=%llu pending=%u\n",
		    (unsigned long long)a.groups, (unsigned long long)a.mpdus,
		    a.maximum, (unsigned long long)a.bins[0],
		    (unsigned long long)a.bins[1], (unsigned long long)a.bins[2],
		    (unsigned long long)a.bins[3], (unsigned long long)a.bins[4],
		    (unsigned long long)a.bins[5], (unsigned long long)a.nonAmpdu,
		    (unsigned long long)a.missingGroup2,
		    (unsigned long long)a.deaggregated,
		    (unsigned long long)a.uncertainGroups, a.pending);
		mtkRxAmpduResetInterval(a);
	}
	if (verbose && mwx_rxr_total != 0) {
		MWX_DEV_LOG(sc,
		    "rxrate n=%u mode[cck=%u ofdm=%u ht=%u vht=%u he=%u] bw[20=%u 40=%u 80=%u 160=%u] nss[1=%u 2=%u] "
		    "mcs[0=%u 1=%u 2=%u 3=%u 4=%u 5=%u 6=%u 7=%u 8=%u 9=%u] sgi=%u stbc=%u ldpc=%u he_gi[0.8=%u 1.6=%u 3.2=%u]\n",
		    mwx_rxr_total, mwx_rxr_mode[0], mwx_rxr_mode[1],
		    mwx_rxr_mode[2] + mwx_rxr_mode[3], mwx_rxr_mode[4],
		    mwx_rxr_mode[8] + mwx_rxr_mode[9] + mwx_rxr_mode[10] +
		    mwx_rxr_mode[11],
		    mwx_rxr_bw[0], mwx_rxr_bw[1], mwx_rxr_bw[2], mwx_rxr_bw[3],
		    mwx_rxr_nss[0], mwx_rxr_nss[1],
		    mwx_rxr_mcs[0], mwx_rxr_mcs[1], mwx_rxr_mcs[2], mwx_rxr_mcs[3],
		    mwx_rxr_mcs[4], mwx_rxr_mcs[5], mwx_rxr_mcs[6], mwx_rxr_mcs[7],
		    mwx_rxr_mcs[8], mwx_rxr_mcs[9], mwx_rxr_sgi, mwx_rxr_stbc,
		    mwx_rxr_ldpc, mwx_rxr_he_gi[0], mwx_rxr_he_gi[1],
		    mwx_rxr_he_gi[2]);
		/* per-interval histogram */
		memset(mwx_rxr_mode, 0, sizeof(mwx_rxr_mode));
		memset(mwx_rxr_bw, 0, sizeof(mwx_rxr_bw));
		memset(mwx_rxr_nss, 0, sizeof(mwx_rxr_nss));
		memset(mwx_rxr_mcs, 0, sizeof(mwx_rxr_mcs));
		memset(mwx_rxr_he_gi, 0, sizeof(mwx_rxr_he_gi));
		mwx_rxr_sgi = mwx_rxr_stbc = mwx_rxr_ldpc = mwx_rxr_total = 0;
	}
	last_intr_calls = mwx_intr_calls;

	if (txq_stuck == 2 && !stall_dumped && tq_dma < sc->sc_txq.mq_count) {
		struct mwx_desc *d = &sc->sc_txq.mq_desc[tq_dma];
		struct mwx_txwi *mt = sc->sc_txq.mq_data[tq_dma].md_txwi;

		stall_dumped = 1;
		MWX_DEV_LOG(sc,
		    "TX STALL ring idx=%u desc buf0=0x%08x ctrl=0x%08x buf1=0x%08x info=0x%08x\n",
		    tq_dma, le32toh(d->buf0), le32toh(d->ctrl),
		    le32toh(d->buf1), le32toh(d->info));
		if (mt != NULL && mt->mt_desc != NULL) {
			struct mt76_txwi *t = mt->mt_desc;
			MWX_DEV_LOG(sc,
			    "TX STALL txwi idx=%u addr=0x%08x w=%08x %08x %08x %08x %08x %08x %08x %08x msdu=%04x ptr0=0x%08x/%04x/%04x/0x%08x ptr1=0x%08x/%04x/%04x/0x%08x\n",
			    mt->mt_idx, mt->mt_addr,
			    t->txwi[0], t->txwi[1], t->txwi[2], t->txwi[3],
			    t->txwi[4], t->txwi[5], t->txwi[6], t->txwi[7],
			    t->msdu_id[0],
			    t->ptr[0].buf0, t->ptr[0].len0, t->ptr[0].len1,
			    t->ptr[0].buf1,
			    t->ptr[1].buf0, t->ptr[1].len0, t->ptr[1].len1,
			    t->ptr[1].buf1);
		}
	}

	/*
	 * Auto recovery: if the TX data or MCU ring has not moved for 4
	 * seconds while work is pending, the WFDMA/firmware is wedged and
	 * only a WFSYS reset with firmware reload brings it back.
	 * Rate limited to once per 30 seconds.
	 */
	uint32_t stuck_limit = (sc->sc_flags &
	    (MWX_FLAG_SCANNING | MWX_FLAG_BGSCAN)) ? 10 : 4;
	if ((txq_stuck >= stuck_limit || mcu_stuck >= stuck_limit) &&
	    polls - last_reset_poll >= 30) {
		mtk_dbg_keep(1);
		MWX_DEV_LOG(sc,
		    "DMA stall detected (txq_stuck=%u mcu_stuck=%u), hard reset\n",
		    txq_stuck, mcu_stuck);
		mtk_dbg_keep(0);
		last_reset_poll = polls;
		txq_stuck = mcu_stuck = 0;
		last_txq_dma = last_mcu_dma = 0xffffffff;
		stall_dumped = 0;
		sc->sc_hard_reset = 1;
		mwx_reset(sc);
		return;
	}

	/* Recover from a lost MSI: process anything left pending. */
	if ((sta & sc->sc_intr_mask) != 0) {
		MWX_DEV_LOG(sc, "poll found pending intr 0x%08x, servicing\n",
		    sta & sc->sc_intr_mask);
		mwx_intr(sc);
	}

	/*
	 * Reception stall. The data RX ring can stop for good while the MCU
	 * ring and TX keep working, which the check above does not see; Wi-Fi
	 * off and on does not bring it back, a firmware reload does. Seen on
	 * an MT7920 with AWDL windows running, 14 minutes into a link on a DFS
	 * channel: every MCU command still succeeded, frames kept leaving, and
	 * nothing reached the ring again, scans included, until the next boot.
	 * Silent for 30 s while 32 frames went out, or, without a link, while
	 * two scans ended (a background scan of a few channels may rightly
	 * find nothing), is a stall: the same hard reset as a DMA stall, the
	 * next one no sooner than 1, 2, 4, then 8 minutes later while it
	 * keeps coming back. An associated station that is not scanning takes
	 * its access point's beacons from the ring ten times a second, AWDL's
	 * windows away included, so there 6 s of silence is a stall already;
	 * and since the station associated again after the last reload, the
	 * reception had come back and the stall is a new one: it is reloaded
	 * at once, three times in a row at most. With 30 s and the longer
	 * gaps, a stall that came back soon after each reload left the station
	 * without data for minutes, and with a minute between two reloads one
	 * that came back 5 s after the station did lasted 53 s. From the
	 * fourth reload in a row it waits a minute, so that a stall coming
	 * back at every return cannot reload the firmware every few seconds;
	 * gaps growing to 8 minutes there left a station without data for
	 * 7 minutes. Such a station counts as associated until its reception
	 * moves again, as it loses its link because of the stall. The second
	 * stall in a row also stops the AWDL radio's windows, where the ring
	 * stops, for ten minutes (mwx_awdl_rest()). The count starts again
	 * after ten minutes without a reload.
	 */
	{
		static uint32_t rx_last = 0xffffffff, rx_tx_last = 0xffffffff;
		static uint32_t rx_sent, rx_scans, rx_resets;
		static uint64_t rx_moved_ms, rx_reset_ms;
		static int rx_was_scanning, rx_moved_assoc;
		uint32_t rq_dma = mwx_read(sc, sc->sc_rxq.mq_regbase + MT_DMA_DMA_IDX);
		uint32_t n = sc->sc_txq.mq_count;
		int scanning = (sc->sc_flags &
		    (MWX_FLAG_SCANNING | MWX_FLAG_BGSCAN)) != 0;
		int assoc = ic->ic_state == IEEE80211_S_RUN && !scanning;
		uint64_t now_ns = 0, now_ms, gap, limit;

		absolutetime_to_nanoseconds(mach_absolute_time(), &now_ns);
		now_ms = now_ns / 1000000;
		if (rq_dma != rx_last || !(ic->ic_if.if_flags & IFF_RUNNING)) {
			if (rx_resets != 0 && now_ms - rx_reset_ms > 600000)
				rx_resets = 0;
			rx_last = rq_dma;
			rx_moved_ms = now_ms;
			rx_moved_assoc = assoc;
			rx_sent = rx_scans = 0;
		} else {
			if (n != 0 && tq_dma < n && rx_tx_last < n)
				rx_sent += (tq_dma + n - rx_tx_last) % n;
			if (rx_was_scanning && !scanning)
				rx_scans++;
		}
		rx_tx_last = tq_dma;
		rx_was_scanning = scanning;
		if (assoc || rx_moved_assoc)
			gap = rx_resets < 3 ? 0 : 60000;
		else
			gap = 60000ULL << (rx_resets == 0 ? 0 : rx_resets - 1 < 3 ? rx_resets - 1 : 3);
		limit = assoc || rx_moved_assoc ? 6000 : 30000;
#ifdef MTK_AWDL
		/*
		 * An AWDL peer was sending data when the ring stopped: while it is there its frames, at least its
		 * synchronization frames, keep coming, so 6 s of silence is a stall already, and a transfer may still
		 * survive the reload; at 30 s the other side had given up.
		 */
		if (sc->sc_awdl != NULL && mwx_awdl_rx_expected(sc, rx_moved_ms))
			limit = 6000;
#endif
		if (now_ms - rx_moved_ms >= limit && (rx_sent >= 32 ||
		    (ic->ic_state != IEEE80211_S_RUN && rx_scans >= 2)) &&
		    (rx_resets == 0 || now_ms - rx_reset_ms >= gap)) {
			/* The report also goes to MtkStallLog, which outlives the ring (mtk_dbg_keep()). */
			mtk_dbg_keep(1);
			MWX_DEV_LOG(sc,
			    "RX stall detected: nothing received for %llu s while %u frames went out and %u scans ended, hard reset "
			    "(rxq cpu=%u dma=%u cons=%u, rxmcu cpu=%u dma=%u cons=%u)\n",
			    (unsigned long long)((now_ms - rx_moved_ms) / 1000),
			    rx_sent, rx_scans, mwx_read(sc, sc->sc_rxq.mq_regbase + MT_DMA_CPU_IDX), rq_dma,
			    sc->sc_rxq.mq_cons, mwx_read(sc, sc->sc_rxmcuq.mq_regbase + MT_DMA_CPU_IDX),
			    mwx_read(sc, sc->sc_rxmcuq.mq_regbase + MT_DMA_DMA_IDX), sc->sc_rxmcuq.mq_cons);
			mwx_mcu_trace_log(sc, now_ns / 1000);
#ifdef MTK_AWDL
			if (sc->sc_awdl != NULL) {
				mwx_awdl_stall_log(sc);
				mwx_awdl_rest(sc, rx_resets != 0);
			}
#endif
			mtk_dbg_keep(0);
			rx_reset_ms = now_ms;
			rx_resets++;
			rx_last = 0xffffffff;
			rx_moved_ms = now_ms;
			rx_sent = rx_scans = 0;
			sc->sc_hard_reset = 1;
			mwx_reset(sc);
		}
	}
}

#ifdef MTK_AWDL
/*
 * AWDL radio, for the AWDL interface (AirportMtkWlan/awdl). It does nothing until awdl0 starts it, which needs
 * AWDL on (no -mtkawdloff): until then sc_awdl is NULL, every hook in the code above returns at once and the
 * firmware sees no new command.
 *
 * The firmware has no AWDL of its own. The interface keeps the AWDL timeline and asks for windows; this part
 * puts the radio where it is asked to be, opens the reception of other devices' frames there, sends the frames
 * queued for that channel and hands the AWDL frames it hears to the interface. Each mechanism is the one Linux
 * mt76 uses on the MT7921 family (drivers/net/wireless/mediatek/mt76/mt7921/mcu.c and main.c), in the order
 * thanhnndev found to hold on MediaTek firmware (mt7925-macos-wifi, kexts/itlmt7925/itlmt7925_m3.cpp):
 *  - time on another channel: a remain-on-channel request, MCU_UNI_CMD(ROC) with UNI_ROC_ACQUIRE, request type
 *    MT7921_ROC_REQ_ROC and the duration in ms (mt7921_mcu_set_roc), granted by the unsolicited
 *    MCU_UNI_EVENT_ROC (mt7921_mcu_uni_roc_event) and always ended with UNI_ROC_ABORT (mt7921_mcu_abort_roc).
 *    Neither request waits for an answer, as in Linux; the firmware tells the access point about the absence;
 *  - hearing other devices: in station mode the firmware hands the host its own BSS only. A window turns on
 *    the RX filter with FIF_OTHER_BSS, MCU_CE_CMD(SET_RX_FILTER) (mt7921_mcu_set_rxfilter from
 *    mt7921_configure_filter), and the firmware sniffer, MCU_UNI_CMD(SNIFFER) UNI_SNIFFER_ENABLE
 *    (mt7921_mcu_set_sniffer), then UNI_SNIFFER_CONFIG for the window's channel once the radio is there
 *    (mt7921_mcu_config_sniffer). Both go back to the station's settings when the window ends. On the
 *    station's channel, once a peer has its record on AWDL's BSS, whose BSSID is AWDL's, neither is opened;
 *  - acknowledging frames sent to awdl0: its address on a second own MAC, OMAC 1, with the DEV_INFO_UPDATE
 *    that mt76_connac_mcu_uni_add_dev sends for a second interface;
 *  - sending: the normal TX path with the 802.11 header kept (mt76_connac2_mac_write_txwi), on ALTX0, the
 *    queue mt76 uses for frames sent outside the station's schedule, at a fixed OFDM rate on either band, or at
 *    VHT for unicast data in a 5 GHz window: at 80 MHz on the station's channel when the link is that wide
 *    (mwx_awdl_vht_rates), at 20 MHz in the others (mwx_awdl_vht20_rates), with the frames queued for one peer
 *    gathered into A-MSDUs (mwx_awdl_amsdu());
 *  - the regulatory country: the access point's Country element, as cfg80211 hands it to
 *    mt7921_regd_notifier. With the world domain the firmware sends on the access point's own 5 GHz channel
 *    only (thanhnndev, mt7925-macos-wifi af60e21).
 *
 * Threads and locks. The interface calls the functions at the bottom from the main work loop or from the
 * Family's threads: they only fill the mailbox and the TX queue, under awdl->lock, and start the radio's
 * timer. A window runs on the radio's own work loop thread, inside the main command gate; its waits go
 * through tsleep_nsec(), which opens that gate, so the interrupt keeps being served. Lock order: the radio
 * work loop's gate (held by the radio thread for a whole run), then the main gate, then awdl->lock (a leaf).
 * The radio thread never waits for the main gate from outside (attemptAction), and a stop path, which may
 * hold the main gate, waits for the radio thread only by sleeping on that gate (mwx_awdl_quiesce) and takes
 * the radio work loop's gate (removeEventSource) only once the thread has left the gated part for good.
 * Frames heard are handed to the interface from mwx_rx(), on the main work loop inside the gate.
 */
#include <IOKit/IOTimerEventSource.h>
#include "awdl/MtkAwdl.h"

#define MWX_AWDL_FRAME_MAX	1600	/* a master indication with every service, or a full data frame */
#define MWX_AWDL_RX_MAX		12288	/* received: an A-MSDU up to VHT's 11454-byte MPDU. The Family announces AWDL
					 * capabilities of its own, and the Pixel sent frames over 4096 bytes:
					 * dropped, they came back the same way and a transfer stopped at 75% */
#define MWX_AWDL_TX_SLOTS	64
#define MWX_AWDL_PEERS		8
#define MWX_AWDL_MCU_MS		300	/* for the answer to one command */
#define MWX_AWDL_ROC_GRANT_MS	1000	/* a grant is waited for this long, as mt7921_set_roc() does, then aborted */
#define MWX_AWDL_ROC_LATE_MS	80	/* a grant later than this is released without sending in it */
#define MWX_AWDL_ROC_BACKOFF_MS	2000	/* after a request that was never granted */
#define MWX_AWDL_ROC_SPARE_MS	95	/* asked on top of the dwell: grant, drain and abort have to fit */
#define MWX_AWDL_ROC_STOP_MS	40	/* the dwell ends this long before the firmware ends the period itself */
#define MWX_AWDL_HOME_MIN_MS	20	/* on the station's channel at least this long between two windows away */
#define MWX_AWDL_HOME_SETTLE_MS	20	/* after an abort, before the reception changes: the radio gets home first */
#define MWX_AWDL_REST_MS	120000	/* no search after a reload for a reception stall (mtkAwdlRadioResting()) */
#define MWX_AWDL_SILENT_MS	600000	/* no windows at all after the second reception stall in a row */
#define MWX_AWDL_RX_REFRESH_MS	2000	/* reception settings sent again although unchanged, at most this often */
#define MWX_AWDL_CHAIN_WAIT_MS	80	/* for the next window on the station's channel, before the reception closes */
#define MWX_AWDL_TX_GUARD_MS	10	/* nothing new for the hardware this close to the end of a window */
#define MWX_AWDL_DRAIN_MS	25	/* for the frames handed to the hardware to leave */
#define MWX_AWDL_FAULT_MAX	3	/* unanswered commands in a row before the radio stays off */
#define MWX_AWDL_TAP_GRACE_MS	100	/* frames still on their way after the sniffer was switched off */
#define MWX_AWDL_FILTER_ENABLE	(1U << 31)	/* MT7921_FILTER_ENABLE */
#define MWX_AWDL_FILTER_OTHER_BSS (1U << 6)	/* MT7921_FILTER_OTHER_BSS */
#define MWX_AWDL_RATE_MIN	4	/* mt76_rates[]: OFDM 6 Mb/s */
#define MWX_AWDL_RATE_START	8	/* 24 Mb/s */
#define MWX_AWDL_RATE_MAX	11	/* 54 Mb/s */

/*
 * Block Ack agreements with AWDL peers. A peer in an agreement gets a station record in the firmware, which lets the
 * MAC answer its A-MPDUs with a Block Ack and aggregate the frames sent to it. The records live in a BSS of AWDL's own
 * on OMAC 1, awdl0's address, kept apart from the station's BSS: the station's teardown and power save never touch
 * them. The station's records use WCIDs 1-14 and 19 (MWX_WTBL_RESERVED); the peers take 15-17 and the BSS 18, the
 * entries mt76 leaves to a second interface. -mtkawdlnoba leaves all of it out.
 */
#define MWX_AWDL_BSS_IDX	1
#define MWX_AWDL_BSS_WCID	(MWX_WTBL_RESERVED - 1)
#define MWX_AWDL_STA_SLOTS	3
#define MWX_AWDL_STA_IDLE_MS	3000	/* a peer with no data either way for this long may give its record up */
#define MWX_AWDL_BA_WINSIZE	64	/* HT's largest window, also the receive window answered */
#define MWX_AWDL_BA_BURST	16	/* unicast frames to a peer in one second that call for an agreement */
#define MWX_AWDL_ADDBA_WAIT_MS	1000	/* for the peer's answer to our request */
#define MWX_AWDL_BA_PROBE	8	/* a new agreement's frames that must come back before more go out */
#define MWX_AWDL_BA_PROBE_MS	1000	/* how long they may take; then the agreement is given up */
#define MWX_AWDL_BAQ		8	/* Block Ack action frames heard, waiting for the radio thread */
#define MWX_AWDL_BAQ_BYTES	36	/* the header and every field of an ADDBA request, response or DELBA */
#define MWX_AWDL_TXI_PEER	0x1f	/* in txinfo[]: 1 group or action, 2 + the peer's entry */
#define MWX_AWDL_TXI_VHT	0x20	/* in txinfo[]: sent at a VHT rate */
#define MWX_AWDL_TXI_AGG	0x40	/* in txinfo[]: a frame of the aggregated path */
#define MWX_AWDL_TXI_5GHZ	0x80	/* in txinfo[]: sent on 5 GHz */
#define MWX_AWDL_VHT20		1	/* win_vht: a 5 GHz window away from an 80 MHz link, VHT at 20 MHz */
#define MWX_AWDL_VHT80		2	/* win_vht: on the station's channel while the link is 80 MHz wide */
#define MWX_AWDL_VHT_HOLD_MS	2000	/* after VHT failed down to 54 Mb/s, before it is tried again */
#define MWX_AWDL_VHT_SLOW	5	/* transmissions of one frame beyond which its VHT step goes down */
#define MWX_AWDL_AMSDU_MAX	3839	/* an A-MSDU we send: the largest every HT receiver takes */
#define MWX_AWDL_AMSDU_FRAME	(26 + MWX_AWDL_AMSDU_MAX)	/* with its QoS data header */

/*
 * Rates of the aggregated path, as the TXD6 rate field takes them: HT MCS 4 to 7, one stream, 20 MHz, long GI. An
 * A-MPDU needs an HT or later PPDU; the per-frame path keeps the OFDM ladder below.
 */
static const uint16_t mwx_awdl_ht_rates[] = {
	(MT_PHY_TYPE_HT << MT_TX_RATE_MODE_SHIFT) | 4, (MT_PHY_TYPE_HT << MT_TX_RATE_MODE_SHIFT) | 5,
	(MT_PHY_TYPE_HT << MT_TX_RATE_MODE_SHIFT) | 6, (MT_PHY_TYPE_HT << MT_TX_RATE_MODE_SHIFT) | 7,
};

/*
 * Above 54 Mb/s on the per-frame path, for unicast data in a window on the station's channel while the link is 80 MHz
 * wide: there the radio stays at the link's width and a peer that sends VHT at 80 MHz, as a phone does, hears it too.
 * VHT at 80 MHz, long GI: one stream MCS 4 and 7, then two streams MCS 7 and 9. A ladder of its own (vht in the peer's
 * entry, 0 = not in use), entered from 54 Mb/s as the OFDM ladder goes up. A frame that fails, or needs more than
 * MWX_AWDL_VHT_SLOW transmissions, takes its step down one, and the first one back to 54 Mb/s for MWX_AWDL_VHT_HOLD_MS;
 * a frame that needs a few only keeps the ladder from climbing. An A-MSDU of two frames at these rates, sent again
 * a few times, still takes less air than one frame at 54 Mb/s. In the other 5 GHz windows the same steps go at 20 MHz
 * (mwx_awdl_vht20_rates); on 2.4 GHz a frame keeps its OFDM rate.
 */
static const uint16_t mwx_awdl_vht_rates[] = {
	(MT_PHY_TYPE_VHT << MT_TX_RATE_MODE_SHIFT) | 4,
	(MT_PHY_TYPE_VHT << MT_TX_RATE_MODE_SHIFT) | 7,
	(MT_PHY_TYPE_VHT << MT_TX_RATE_MODE_SHIFT) | (1 << MT_TX_RATE_NSS_SHIFT) | 7,
	(MT_PHY_TYPE_VHT << MT_TX_RATE_MODE_SHIFT) | (1 << MT_TX_RATE_NSS_SHIFT) | 9,
};

/*
 * The ladder's steps at 20 MHz, for unicast data in a 5 GHz window away from an 80 MHz link: the remain-on-channel
 * period is 20 MHz wide, and a peer that listens there at a larger width still hears its primary channel.
 * Dino-VN/MtkWireless (1fa74dc9) sends VHT at 20 MHz in its 20 MHz remain-on-channel periods, two streams MCS 7 and 8;
 * its 80 MHz periods were granted but heard nothing. One stream MCS 4 and 7, then two streams MCS 7 and 8, long GI:
 * MCS 9 is not defined at 20 MHz for one or two streams. mtkawdlvht20=0 keeps OFDM there.
 */
static const uint16_t mwx_awdl_vht20_rates[] = {
	(MT_PHY_TYPE_VHT << MT_TX_RATE_MODE_SHIFT) | 4,
	(MT_PHY_TYPE_VHT << MT_TX_RATE_MODE_SHIFT) | 7,
	(MT_PHY_TYPE_VHT << MT_TX_RATE_MODE_SHIFT) | (1 << MT_TX_RATE_NSS_SHIFT) | 7,
	(MT_PHY_TYPE_VHT << MT_TX_RATE_MODE_SHIFT) | (1 << MT_TX_RATE_NSS_SHIFT) | 8,
};
static_assert(sizeof(mwx_awdl_vht20_rates) == sizeof(mwx_awdl_vht_rates), "one ladder, two widths");

static const uint8_t mwx_awdl_bssid[6] = { 0x00, 0x25, 0x00, 0xff, 0x94, 0x73 };

struct mwx_awdl_tx_slot {
	uint16_t	len;
	uint8_t		ch;
	uint64_t	deadline_us;
	uint8_t		data[MWX_AWDL_FRAME_MAX];
};

/* Our side of a transmit agreement with a peer. */
enum {
	MWX_AWDL_TXBA_NONE,
	MWX_AWDL_TXBA_ASKED,		/* our ADDBA request is out, waiting for the answer */
	MWX_AWDL_TXBA_ON,		/* in force: unicast data on the station's channel goes aggregated */
};

/*
 * One unicast address: the rate of the frames to it, moved by the hardware's report of each frame, and, while it takes
 * part in a Block Ack agreement, its station record. Radio thread and main work loop, both inside the main gate.
 */
struct mwx_awdl_peer {
	uint8_t		addr[6];
	uint8_t		rate;
	uint8_t		good;
	uint32_t	used;
	uint16_t	wcid;		/* its station record; 0 = none */
	uint8_t		ch;		/* where its last frame was heard, or our last one sent */
	uint8_t		rx_tids;	/* receive agreements in force, one bit per TID */
	uint8_t		tx_state;	/* MWX_AWDL_TXBA_* */
	uint8_t		tx_token;
	uint8_t		tx_tries;	/* requests left unanswered in a row */
	uint8_t		ba_rate;	/* index in mwx_awdl_ht_rates */
	uint8_t		ba_good;
	uint16_t	tx_sn;		/* next sequence number of TID 0 on the aggregated path */
	uint16_t	tx_ssn;
	uint16_t	tx_win;
	uint16_t	ba_out;		/* aggregated-path frames handed over and not reported yet */
	uint16_t	ba_ok, ba_fail;	/* reported, since the agreement started */
	uint32_t	data_n;		/* unicast data frames handed over in the second that began at data_us */
	uint64_t	data_us, last_us, tx_asked_us, tx_retry_us, ba_first_us;
	uint64_t	rx_us;		/* its last data frame heard (receive tap) */
	uint8_t		vht, vht_good;	/* step of mwx_awdl_vht_rates + 1, 0 = OFDM; first tries in a row there */
	uint64_t	vht_hold_us;	/* no VHT before this */
};

/* A Block Ack action frame heard, for the radio thread. */
struct mwx_awdl_baq {
	uint8_t		len;
	uint8_t		ch;
	uint8_t		frame[MWX_AWDL_BAQ_BYTES];
};

struct mwx_awdl {
	IOLock			*lock;		/* mailbox and TX queue */
	IOWorkLoop		*wl;		/* the radio's thread */
	IOTimerEventSource	*timer;
	volatile bool		armed;		/* started by the interface */
	volatile bool		paused;		/* a stop path is waiting: no window, no waiting */
	volatile bool		dying;		/* detach */
	volatile bool		running;	/* the radio thread is inside the gated part */
	bool			off;		/* too many unanswered commands: off until the firmware starts again */
	uint8_t			faults;
	uint8_t			addr[6];	/* awdl0 */
	/*
	 * What the firmware was told. Each is set before the command that turns it on is sent and cleared
	 * once the command that turns it off is out, so the receive tap can go by them at any moment.
	 */
	bool			omac_on;	/* addr is OMAC 1 */
	bool			omac_failed;
	bool			sniffer_on;
	bool			filter_open;	/* RX filter with FIF_OTHER_BSS */
	uint32_t		sniff_cfg;	/* channel, width and center the sniffer was last given; 0 none */
	uint64_t		rx_cfg_us;	/* when the filter and the sniffer were last sent although unchanged */
	bool			fullpwr;	/* deep sleep taken away from a station that had asked for it */
	bool			roc_active;
	volatile bool		tap_filter;	/* sniffer_on || filter_open */
	volatile bool		tap_grace;
	uint64_t		tap_grace_until_us;
	/* the window in progress */
	volatile bool		rx_open;
	volatile uint8_t	rx_ch;
	/* remain on channel */
	uint8_t			roc_token;
	volatile bool		roc_want;
	volatile bool		roc_granted;
	uint8_t			roc_status;
	uint64_t		off_since_us;
	uint64_t		backoff_until_us;
	uint64_t		home_until_us;	/* the access point keeps the radio until then */
	uint64_t		rest_until_us;	/* after a reload for a reception stall: no search until then */
	uint64_t		silent_until_us;	/* after the second stall in a row: no windows until then */
	uint32_t		lead_us;	/* measured request-to-grant time, the setup lead of the next window */
	/* the radio's own commands: their answers never reach sc_mcu_wait[] */
	uint32_t		seq_cmd[16];
	uint32_t		seq_gen[16];
	uint32_t		seq_status[16];
	bool			seq_done[16];
	/* mailbox: the next window */
	MtkAwdlWindow		next;
	bool			next_valid;
	int			wake;		/* sleep channel of the radio thread */
	/* frames for the windows */
	struct mwx_awdl_tx_slot	*tx;
	uint32_t		tx_head, tx_count;
	bool			tx_was_full;	/* a frame was refused: tell the interface once there is room */
	uint32_t		tx_inflight;	/* handed to the hardware, not reported yet */
	uint8_t			txinfo[MWX_TXWI_MAX];	/* per TXWI: 0 not ours, 1 group or action, 2 + peer */
	struct mwx_awdl_peer	peers[MWX_AWDL_PEERS];
	uint32_t		peer_clock;
	/* Block Ack */
	bool			ba_off;		/* -mtkawdlnoba */
	bool			bss_on;		/* AWDL's BSS and its group record are in the firmware */
	bool			bss_failed;	/* refused: no records until the firmware starts again */
	bool			win_onch;	/* the window in progress is on the station's channel */
	uint8_t			win_vht;	/* unicast data may go at VHT rates: MWX_AWDL_VHT20 or MWX_AWDL_VHT80, 0 not */
	uint8_t			sta_used;	/* bit per station record slot */
	uint16_t		mgmt_sn;
	struct mwx_awdl_baq	baq[MWX_AWDL_BAQ];	/* main work loop adds, radio thread takes; both gated */
	uint32_t		baq_head, baq_count;
	uint32_t		c_sta_add, c_sta_del, c_sta_fail, c_sta_busy, c_baq_full, c_split;
	uint32_t		c_ba_heard, c_ba_skip, c_mgmt_other;	/* for the log's first lines */
	uint32_t		c_ba_htc;	/* Block Ack actions heard with an HT Control field (mwx_awdl_rx_tap()) */
	uint32_t		c_vht_ok, c_vht_fail, c_vht20, c_amsdu, c_amsdu_sub;
	uint32_t		c_rxba_ok, c_rxba_no, c_rxba_del;
	uint32_t		c_txba_ask, c_txba_ok, c_txba_no, c_txba_late, c_txba_del, c_txba_drop;
	uint32_t		c_ba_ok, c_ba_fail, c_ba_hold;
	uint32_t		c_agg[16];	/* the MAC's A-MPDU length bins, read around windows that aggregated */
	uint8_t			txbuf[MWX_AWDL_FRAME_MAX];	/* radio thread only */
	uint8_t			amsdu[MWX_AWDL_AMSDU_FRAME];	/* radio thread only: mwx_awdl_amsdu() */
	uint8_t			rxbuf[MWX_AWDL_RX_MAX];	/* main work loop only */
	/* counters */
	uint32_t		c_win, c_onch, c_late, c_busy, c_share, c_badch, c_gone;
	uint32_t		c_chain_wait, c_chain_miss;	/* the next window waited for: came, did not */
	uint32_t		c_quiet;	/* windows on the station's channel with no sniffer or filter opened */
	uint32_t		c_hold;		/* windows after which the reception stayed open (mwx_awdl_rx_hold()) */
	uint32_t		c_silent;	/* windows not taken while the radio is silent after repeated stalls */
	uint32_t		c_rx_close;	/* times the sniffer went off and the station's filter came back */
	uint32_t		c_roc_ok, c_roc_fail, c_roc_late, c_roc_stray, c_roc_clip, c_cmd, c_cmd_err;
	uint32_t		c_tx_group, c_tx_uc_ok[2], c_tx_uc_fail[2], c_tx_err, c_tx_stale, c_tx_full;
	/* group frames and action frames as the TX free event reports them: delivered, failed, transmissions */
	uint32_t		c_grp_ok, c_grp_fail, c_grp_tries;
	uint32_t		c_rx_af, c_rx_data, c_rx_long, c_rx_echo, c_rx_other, c_drain;
	/* by social channel, 6, 44, 149, then any other: windows, AWDL frames heard, other frames heard */
	uint32_t		c_win_ch[4], c_af_ch[4], c_other_ch[4];
	uint32_t		c_rx_uc;	/* AWDL data frames addressed to awdl0 */
	uint8_t			first_af;	/* bit per bucket: the first AWDL frame there was logged */
	uint32_t		c_log;
	uint64_t		late_us, off_us, log_us;
	/* the station's data frames with its access point, sent [0] and received [1] (mwx_awdl_sta_traffic()) */
	uint64_t		sta_pkts[2], sta_bytes[2];
	/* of which addressed to one device */
	uint64_t		sta_uc_pkts[2], sta_uc_bytes[2];
};

static struct mwx_softc *gMwxAwdlSc;

/* The counters' bucket of a channel: AWDL's social channels 6, 44 and 149, then all the others. */
static inline int
mwx_awdl_bucket(uint8_t ch)
{
	return ch == 6 ? 0 : ch == 44 ? 1 : ch == 149 ? 2 : 3;
}

static bool mwx_awdl_tx_done(struct mwx_softc *, uint16_t, uint32_t);
static void mwx_awdl_close(struct mwx_softc *);

uint64_t
mtkAwdlUptimeUs()
{
	uint64_t ns = 0;

	absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
	return ns / 1000;
}

/* The firmware cannot be talked to: asleep, stopping, reloading or gone. Nothing is sent to it then. */
static bool
mwx_awdl_fw_gone(struct mwx_softc *sc)
{
	return sc->sc_suspended || sc->sc_stopping || sc->sc_mcu_mute || sc->sc_resetting || sc->sc_detaching ||
	    !sc->sc_fw_loaded || !(sc->sc_ic.ic_if.if_flags & IFF_RUNNING);
}

/* The station needs the radio for itself: a scan, a join, or the key handshake right after it. */
static bool
mwx_awdl_station_busy(struct mwx_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;

	if (sc->sc_flags & (MWX_FLAG_SCANNING | MWX_FLAG_BGSCAN))
		return true;
	if (ic->ic_state == IEEE80211_S_AUTH || ic->ic_state == IEEE80211_S_ASSOC)
		return true;
	return ic->ic_state == IEEE80211_S_RUN && (ic->ic_flags & IEEE80211_F_RSNON) && ic->ic_bss != NULL &&
	    !ic->ic_bss->ni_port_valid;
}

static bool
mwx_awdl_stop_asked(struct mwx_softc *sc)
{
	struct mwx_awdl *a = sc->sc_awdl;

	return !a->armed || a->paused || a->dying || a->off || mwx_awdl_fw_gone(sc);
}

static uint8_t
mwx_awdl_home(struct mwx_softc *sc, uint8_t *bw, uint8_t *center)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_node *ni = ic->ic_bss;
	uint8_t b = CMD_CBW_20MHZ, c = 0, ch;

	if (ic->ic_state != IEEE80211_S_RUN || ni == NULL || ni->ni_chan == NULL ||
	    ni->ni_chan == IEEE80211_CHAN_ANYC)
		return 0;
	ch = ieee80211_chan2ieee(ic, ni->ni_chan);
	mwx_node_chan_width(ni, &b, &c);
	if (bw != NULL)
		*bw = b;
	if (center != NULL)
		*center = c ? c : ch;
	return ch;
}

/*
 * Only a real 20 MHz channel without radar duty, as thanhnndev's driver ended up doing: a remain-on-channel
 * request for a DFS channel, or for a number that is no channel, is never granted and the firmware stopped
 * answering after a few of them (mt7925-macos-wifi 61cd987, c8a285f).
 */
static bool
mwx_awdl_channel_ok(struct mwx_softc *sc, uint8_t ch)
{
	const struct ieee80211_channel *c;

	if (!((ch >= 1 && ch <= 13) || (ch >= 36 && ch <= 48 && (ch % 4) == 0) ||
	    (ch >= 149 && ch <= 165 && (ch % 4) == 1)))
		return false;
	c = &sc->sc_ic.ic_channels[ch];
	return c->ic_flags != 0 && !(c->ic_flags & (IEEE80211_CHAN_PASSIVE | IEEE80211_CHAN_DFS));
}

/* An unanswered command. The station's own commands reload the firmware then; the radio only steps back. */
static void
mwx_awdl_fault(struct mwx_softc *sc, uint32_t cmd)
{
	struct mwx_awdl *a = sc->sc_awdl;

	a->c_cmd_err++;
	a->backoff_until_us = mtkAwdlUptimeUs() + 5000000ULL;
	if (++a->faults >= MWX_AWDL_FAULT_MAX && !a->off) {
		a->off = true;
		MWX_DEV_LOG(sc, "awdl: radio off, the firmware left %u commands unanswered (last 0x%08x)\n",
		    a->faults, cmd);
	} else if (a->c_cmd_err <= 8) {
		MWX_DEV_LOG(sc, "awdl: command 0x%08x not answered in %u ms\n", cmd, MWX_AWDL_MCU_MS);
	}
}

/*
 * One command of the radio. Its answer is taken by mwx_awdl_mcu_event() instead of sc_mcu_wait[], so a late
 * or missing one costs nothing and never reloads the firmware. wait_ms 0 sends without waiting, as Linux
 * does for the remain-on-channel requests.
 */
static int
mwx_awdl_mcu(struct mwx_softc *sc, uint32_t cmd, void *data, size_t len, uint32_t wait_ms, uint32_t *status)
{
	struct mwx_awdl *a = sc->sc_awdl;
	uint64_t until;
	int rv, seq = 0;

	if (mwx_awdl_fw_gone(sc))
		return ENXIO;
	rv = mwx_mcu_send_msg(sc, cmd, data, len, &seq);
	if (rv != 0) {
		a->c_cmd_err++;
		return rv;
	}
	a->c_cmd++;
	a->seq_status[seq] = 0;
	a->seq_done[seq] = false;
	a->seq_gen[seq] = sc->sc_mcu_seq;
	a->seq_cmd[seq] = cmd;
	if (wait_ms == 0 || a->paused || a->dying)
		return 0;
	until = mtkAwdlUptimeUs() + wait_ms * 1000ULL;
	while (!a->seq_done[seq] && a->seq_cmd[seq] == cmd && !a->paused && !a->dying) {
		uint64_t now = mtkAwdlUptimeUs();
		if (now >= until)
			break;
		tsleep_nsec(&a->seq_cmd[seq], 0, "mwxawdl", (until - now) * 1000);
	}
	if (!a->seq_done[seq]) {
		if (a->paused || a->dying)
			return 0;
		/* Still registered: a late answer is taken and freed by mwx_awdl_mcu_event(), never left in sc_mcu_wait[]. */
		mwx_awdl_fault(sc, cmd);
		return ETIMEDOUT;
	}
	a->faults = 0;
	if (status != NULL)
		*status = a->seq_status[seq];
	return 0;
}

/* UNI_ROC_ACQUIRE: struct roc_acquire_tlv of mt7921_mcu_set_roc. */
static int
mwx_awdl_roc(struct mwx_softc *sc, uint8_t ch, uint32_t duration_ms)
{
	struct mwx_awdl *a = sc->sc_awdl;
	struct {
		uint8_t		rsv[4];
		uint16_t	tag;
		uint16_t	len;
		uint8_t		bss_idx;
		uint8_t		tokenid;
		uint8_t		control_channel;
		uint8_t		sco;
		uint8_t		band;
		uint8_t		bw;
		uint8_t		center_chan;
		uint8_t		center_chan2;
		uint8_t		bw_from_ap;
		uint8_t		center_chan_from_ap;
		uint8_t		center_chan2_from_ap;
		uint8_t		reqtype;
		uint32_t	maxinterval;
		uint8_t		dbdcband;
		uint8_t		rsv2[3];
	} __packed req;
	int rv;

	memset(&req, 0, sizeof(req));
	req.tag = htole16(0);			/* UNI_ROC_ACQUIRE */
	req.len = htole16(sizeof(req) - 4);
	req.bss_idx = sc->sc_vif.idx;
	req.tokenid = ++a->roc_token;
	req.control_channel = ch;
	req.band = ch > 14 ? 2 : 1;
	req.bw = CMD_CBW_20MHZ;
	req.center_chan = ch;
	req.bw_from_ap = CMD_CBW_20MHZ;
	req.center_chan_from_ap = ch;
	req.reqtype = 1;			/* MT7921_ROC_REQ_ROC */
	req.maxinterval = htole32(duration_ms);
	req.dbdcband = 0xff;			/* auto */
	a->roc_granted = false;
	a->roc_status = 0xff;
	a->roc_want = true;
	rv = mwx_awdl_mcu(sc, MCU_UNI_CMD_ROC, &req, sizeof(req), 0, NULL);
	if (rv != 0)
		a->roc_want = false;
	return rv;
}

/* UNI_ROC_ABORT: struct roc_abort_tlv of mt7921_mcu_abort_roc. */
static int
mwx_awdl_roc_abort(struct mwx_softc *sc)
{
	struct mwx_awdl *a = sc->sc_awdl;
	struct {
		uint8_t		rsv[4];
		uint16_t	tag;
		uint16_t	len;
		uint8_t		bss_idx;
		uint8_t		tokenid;
		uint8_t		dbdcband;
		uint8_t		rsv2[5];
	} __packed req;

	memset(&req, 0, sizeof(req));
	req.tag = htole16(1);			/* UNI_ROC_ABORT */
	req.len = htole16(sizeof(req) - 4);
	req.bss_idx = sc->sc_vif.idx;
	req.tokenid = a->roc_token;
	req.dbdcband = 0xff;
	a->roc_want = false;
	return mwx_awdl_mcu(sc, MCU_UNI_CMD_ROC, &req, sizeof(req), 0, NULL);
}

/*
 * MCU_UNI_EVENT_ROC, unsolicited; m starts after the event header. struct mt7921_roc_grant_tlv follows a
 * 4-byte header: tag, len, bss_idx, tokenid, status, primarychannel, rfsco, rfband, channelwidth,
 * centerfreqseg1, centerfreqseg2, reqtype, dbdcband, rsv, max_interval. Only the grant of the request in
 * flight counts: an event for an older token is ignored (mt7925-macos-wifi 2545c30).
 */
static void
mwx_awdl_roc_event(struct mwx_softc *sc, struct mbuf *m)
{
	struct mwx_awdl *a = sc->sc_awdl;
	uint8_t g[20];

	if (mbuf_len(m) < 4 + sizeof(g) || mbuf_copydata(m, 4, sizeof(g), g) != 0)
		return;
	{
		uint8_t t[4] = { g[0], g[5], g[6], g[7] };	/* tag, token, status, channel, for the stall's log */

		mwx_mcu_trace_add(MWX_MCU_TRACE_ROC_EVENT, t, sizeof(t));
	}
	if ((g[0] | g[1] << 8) != 0 || !a->roc_want || g[5] != a->roc_token) {	/* UNI_EVENT_ROC_GRANT */
		a->c_roc_stray++;
		return;
	}
	a->roc_status = g[6];
	a->roc_granted = true;
	a->roc_want = false;
	wakeup((void *)&a->roc_granted);
}

/*
 * From mwx_mcu_rx_event(), main work loop; m starts after the event header. True when the event belongs to
 * the radio: the remain-on-channel grant, or the answer to one of its own commands. A sequence number is
 * the radio's only as long as fewer than 15 commands were sent after it, the distance at which the HAL
 * hands the same number out again.
 */
static bool
mwx_awdl_mcu_event(struct mwx_softc *sc, uint8_t eid, uint8_t seq, uint8_t option, struct mbuf *m)
{
	struct mwx_awdl *a = sc->sc_awdl;

	if ((option & MCU_UNI_CMD_UNSOLICITED_EVENT) && eid == MCU_UNI_EVENT_ROC) {
		mwx_awdl_roc_event(sc, m);
		return true;
	}
	if (seq == 0 || seq >= nitems(a->seq_cmd) || a->seq_cmd[seq] == 0)
		return false;
	if ((uint32_t)(sc->sc_mcu_seq - a->seq_gen[seq]) >= 15 ||
	    sc->sc_mcu_wait[seq].mcu_cmd != a->seq_cmd[seq]) {
		a->seq_cmd[seq] = 0;
		return false;
	}
	if (eid == MCU_UNI_EVENT_RESULT && mbuf_len(m) >= sizeof(struct mt7921_mcu_uni_event))
		a->seq_status[seq] = le32toh(mtod(m, struct mt7921_mcu_uni_event *)->status);
	a->seq_done[seq] = true;
	a->seq_cmd[seq] = 0;
	wakeup(&a->seq_cmd[seq]);
	return true;
}

/* UNI_SNIFFER_ENABLE: struct sniffer_enable_tlv of mt7921_mcu_set_sniffer. */
static int
mwx_awdl_sniffer(struct mwx_softc *sc, int enable, uint32_t wait_ms)
{
	struct {
		uint8_t		band_idx;
		uint8_t		pad[3];
		uint16_t	tag;
		uint16_t	len;
		uint8_t		enable;
		uint8_t		pad2[3];
	} __packed req;

	memset(&req, 0, sizeof(req));
	req.band_idx = sc->sc_vif.band_idx;
	req.tag = htole16(0);
	req.len = htole16(sizeof(req) - 4);
	req.enable = (uint8_t)enable;
	return mwx_awdl_mcu(sc, MCU_UNI_CMD_SNIFFER, &req, sizeof(req), wait_ms, NULL);
}

/* UNI_SNIFFER_CONFIG: struct config_tlv of mt7921_mcu_config_sniffer. */
static int
mwx_awdl_sniffer_config(struct mwx_softc *sc, uint8_t ch, uint8_t bw, uint8_t center)
{
	struct {
		uint8_t		band_idx;
		uint8_t		pad[3];
		uint16_t	tag;
		uint16_t	len;
		uint16_t	aid;
		uint8_t		ch_band;
		uint8_t		bw;
		uint8_t		control_ch;
		uint8_t		sco;
		uint8_t		center_ch;
		uint8_t		center_ch2;
		uint8_t		drop_err;
		uint8_t		pad2[3];
	} __packed req;

	memset(&req, 0, sizeof(req));
	req.band_idx = sc->sc_vif.band_idx;
	req.tag = htole16(1);
	req.len = htole16(sizeof(req) - 4);
	req.ch_band = ch > 14 ? 2 : 1;
	/*
	 * ch_width[] of mt7921_mcu_config_sniffer, not CMD_CBW_*: 20 and 40 MHz are 0 (40 comes from sco), 80
	 * is 1, 160 is 2. The wrong table stopped the firmware on a 160 MHz link (mt7925-macos-wifi 4568bf2).
	 */
	req.bw = bw == CMD_CBW_160MHZ ? 2 : bw == CMD_CBW_80MHZ ? 1 : 0;
	req.control_ch = ch;
	req.center_ch = center ? center : ch;
	if (req.control_ch < req.center_ch)
		req.sco = 1;			/* SCA */
	else if (req.control_ch > req.center_ch)
		req.sco = 3;			/* SCB */
	req.drop_err = 1;			/* no FCS-failed frames: their TLVs are noise */
	return mwx_awdl_mcu(sc, MCU_UNI_CMD_SNIFFER, &req, sizeof(req), 0, NULL);
}

/*
 * MCU_CE_CMD(SET_RX_FILTER), mt7921_mcu_set_rxfilter(dev, fif, 0, 0): mode 1 sets the whole filter. The
 * firmware does not answer it, and Linux does not wait either.
 */
static int
mwx_awdl_rxfilter(struct mwx_softc *sc, uint32_t fif)
{
	struct {
		uint8_t		rsv[4];
		uint8_t		mode;
		uint8_t		rsv2[3];
		uint32_t	fif;
		uint32_t	bit_map;
		uint8_t		bit_op;
		uint8_t		pad[51];
	} __packed req;

	if (mwx_awdl_fw_gone(sc))
		return ENXIO;
	memset(&req, 0, sizeof(req));
	req.mode = fif ? 1 : 2;
	req.fif = htole32(fif);
	sc->sc_awdl->c_cmd++;
	return mwx_mcu_send_msg(sc, MCU_CE_CMD_SET_RX_FILTER, &req, sizeof(req), NULL);
}

/* DEV_INFO_UPDATE for OMAC 1, the request mt76_connac_mcu_uni_add_dev makes for a second interface. */
static int
mwx_awdl_omac(struct mwx_softc *sc, int enable)
{
	struct mwx_awdl *a = sc->sc_awdl;
	struct {
		uint8_t		omac_idx;
		uint8_t		band_idx;
		uint16_t	pad;
		uint16_t	tag;
		uint16_t	len;
		uint8_t		active;
		uint8_t		pad2;
		uint8_t		omac_addr[ETHER_ADDR_LEN];
	} __packed req;
	uint32_t status = 0;
	int rv;

	memset(&req, 0, sizeof(req));
	req.omac_idx = 1;
	req.band_idx = sc->sc_vif.band_idx;
	req.tag = htole16(DEV_INFO_ACTIVE);
	req.len = htole16(sizeof(req) - 4);
	req.active = (uint8_t)enable;
	memcpy(req.omac_addr, a->addr, ETHER_ADDR_LEN);
	rv = mwx_awdl_mcu(sc, MCU_UNI_CMD_DEV_INFO_UPDATE, &req, sizeof(req), MWX_AWDL_MCU_MS, &status);
	return rv != 0 ? rv : status != 0 ? EIO : 0;
}

/*
 * "KeepFullPwr", what mwx_mcu_set_deep_sleep() sends, without touching the station's own record of it
 * (sc_awdl_ds). Linux keeps the chip out of deep sleep for as long as the sniffer is on
 * (mt7921_sniffer_interface_iter). An associated station already runs at full power here; this is for the
 * windows of a station without a network, which the HAL leaves in deep sleep after a disconnection.
 */
static void
mwx_awdl_fullpwr(struct mwx_softc *sc, bool on)
{
	struct mwx_awdl *a = sc->sc_awdl;
	struct mwx_connac_config req = { .resp_type = 0, };

	if (a->fullpwr == on || mwx_awdl_fw_gone(sc))
		return;
	if (on && !sc->sc_awdl_ds)
		return;				/* the station keeps full power itself */
	if (!on && !sc->sc_awdl_ds) {
		a->fullpwr = false;		/* the station changed its mind meanwhile: its setting stands */
		return;
	}
	snprintf((char *)req.data, sizeof(req.data), "KeepFullPwr %d", on ? 1 : 0);
	a->c_cmd++;
	if (mwx_mcu_send_msg(sc, MCU_CE_CMD_CHIP_CONFIG, &req, sizeof(req), NULL) == 0)
		a->fullpwr = on;
}

/*
 * The regulatory country from the access point's Country element, as cfg80211 does (802.11d) before
 * mt7921_regd_notifier sends the channel domain and the power limits again. The HAL starts with the world
 * domain, "00". It is read once per access point, on entering RUN before net80211 brings the link up, and the
 * front end is told of a change (IEEE80211_EVT_COUNTRY_CODE_UPDATE). The front end reports this country to the
 * Family (APPLE80211_IOC_COUNTRY_CODE), and the Family's AWDL peer manager takes its channels from it whenever
 * the station's link changes (configureValidAWDLChannels). Read only from the AWDL radio's first window, after the
 * link was up, it came too late: the Family kept the world domain, took 149 for AWDL, which most of Europe does
 * not allow and which a Pixel never visits, and dropped a Pixel on 44 every second.
 */
static void
mwx_ap_country(struct mwx_softc *sc, struct ieee80211_node *ni)
{
	struct ieee80211com *ic = &sc->sc_ic;
	const uint8_t *ie, *end;

	if (ni == NULL || ni->ni_rsnie_tlv == NULL || memcmp(sc->sc_cc_bssid, ni->ni_bssid, 6) == 0)
		return;
	memcpy(sc->sc_cc_bssid, ni->ni_bssid, 6);	/* once per access point */
	ie = ni->ni_rsnie_tlv;
	end = ie + ni->ni_rsnie_tlv_len;
	while (end - ie >= 2 && end - ie >= 2 + ie[1]) {
		if (ie[0] == IEEE80211_ELEMID_COUNTRY && ie[1] >= 3 && ie[2] >= 'A' && ie[2] <= 'Z' &&
		    ie[3] >= 'A' && ie[3] <= 'Z')
			break;
		ie += 2 + ie[1];
	}
	if (end - ie < 5 || ie[0] != IEEE80211_ELEMID_COUNTRY)
		return;
	if (sc->sc_alpha2[0] == (char)ie[2] && sc->sc_alpha2[1] == (char)ie[3])
		return;
	MWX_DEV_LOG(sc, "regulatory country %c%c from the access point (was %c%c)\n", ie[2], ie[3],
	    sc->sc_alpha2[0], sc->sc_alpha2[1]);
	sc->sc_alpha2[0] = (char)ie[2];
	sc->sc_alpha2[1] = (char)ie[3];
	if (mt7921_mcu_set_channel_domain(sc) != 0 || mt7921_mcu_set_rate_txpower(sc) != 0)
		MWX_DEV_LOG(sc, "the country was not taken by the firmware\n");
	if (ic->ic_event_handler)
		ic->ic_event_handler(ic, IEEE80211_EVT_COUNTRY_CODE_UPDATE, NULL);
}

/* Before an AWDL window: the same, once per access point, so nothing new after the read on entering RUN. */
static void
mwx_awdl_country(struct mwx_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;

	if (ic->ic_state == IEEE80211_S_RUN)
		mwx_ap_country(sc, ic->ic_bss);
}

static struct mwx_awdl_peer *
mwx_awdl_peer_find(struct mwx_awdl *a, const uint8_t *addr, int *slot)
{
	int i;

	for (i = 0; i < MWX_AWDL_PEERS; i++) {
		if (a->peers[i].rate != 0 && memcmp(a->peers[i].addr, addr, 6) == 0) {
			if (slot != NULL)
				*slot = i;
			return &a->peers[i];
		}
	}
	return NULL;
}

/*
 * The entry of an address, made when there is none. The one that goes is the least recently used among those without
 * a station record: a record is only given up by mwx_awdl_sta_free(), which tells the firmware.
 */
static struct mwx_awdl_peer *
mwx_awdl_peer(struct mwx_awdl *a, const uint8_t *addr, int *slot)
{
	struct mwx_awdl_peer *p = mwx_awdl_peer_find(a, addr, slot);
	int i, old = -1;

	if (p == NULL) {
		for (i = 0; i < MWX_AWDL_PEERS; i++) {
			if (a->peers[i].wcid == 0 && (old < 0 || a->peers[i].used < a->peers[old].used))
				old = i;
		}
		if (old < 0)
			old = 0;	/* not reached: MWX_AWDL_STA_SLOTS < MWX_AWDL_PEERS */
		p = &a->peers[old];
		memset(p, 0, sizeof(*p));
		memcpy(p->addr, addr, 6);
		p->rate = MWX_AWDL_RATE_START;
		p->ba_rate = nitems(mwx_awdl_ht_rates) - 1;
		*slot = old;
	}
	p->used = ++a->peer_clock;
	return p;
}

/*
 * A command of the Block Ack part, built in an mbuf (consumed), with its answer waited for as mwx_awdl_mcu() waits.
 * Radio thread only. 0 only when the firmware answered with status 0; a stop path that cuts the wait short gives
 * ECANCELED, a missing answer counts as a fault of the radio (mwx_awdl_fault()).
 */
static int
mwx_awdl_mcu_sync(struct mwx_softc *sc, uint32_t cmd, struct mbuf *m)
{
	struct mwx_awdl *a = sc->sc_awdl;
	uint64_t until;
	int rv, seq = 0;

	if (m == NULL)
		return ENOBUFS;
	if (mwx_awdl_fw_gone(sc) || a->paused || a->dying || a->off) {
		m_freem(m);
		return ECANCELED;
	}
	rv = mwx_mcu_send_mbuf(sc, cmd, m, &seq);
	if (rv != 0) {
		a->c_cmd_err++;
		return rv;
	}
	a->c_cmd++;
	a->seq_status[seq] = 0;
	a->seq_done[seq] = false;
	a->seq_gen[seq] = sc->sc_mcu_seq;
	a->seq_cmd[seq] = cmd;
	until = mtkAwdlUptimeUs() + MWX_AWDL_MCU_MS * 1000ULL;
	while (!a->seq_done[seq] && a->seq_cmd[seq] == cmd && !a->paused && !a->dying) {
		uint64_t now = mtkAwdlUptimeUs();

		if (now >= until)
			break;
		tsleep_nsec(&a->seq_cmd[seq], 0, "mwxawdl", (until - now) * 1000);
	}
	if (!a->seq_done[seq]) {
		if (a->paused || a->dying)
			return ECANCELED;
		mwx_awdl_fault(sc, cmd);
		return ETIMEDOUT;
	}
	a->faults = 0;
	return a->seq_status[seq] != 0 ? EIO : 0;
}

/*
 * AWDL's BSS, the BSS_INFO half of mt76_connac_mcu_uni_add_dev() for OMAC 1 (mwx_awdl_omac() is the DEV_INFO half). A
 * P2P device that is not connected, AP | P2P with conn_state 1, as mt76 builds one for NL80211_IFTYPE_P2P_DEVICE: the
 * firmware gives it no channel and no beacon, only the peers' records hang on it. MtkWireless found an IBSS refused by
 * the MT7921 firmware and this type taken. It shares the station's EDCA set, whose queues the aggregated path uses. Its
 * BSSID is AWDL's, which every AWDL frame carries as address 3.
 */
static int
mwx_awdl_bss_cmd(struct mwx_softc *sc, bool on)
{
	struct {
		struct {
			uint8_t		bss_idx;
			uint8_t		pad[3];
		} __packed hdr;
		struct mt76_connac_bss_basic_tlv basic;
	} __packed req;
	struct mbuf *m;

	memset(&req, 0, sizeof(req));
	req.hdr.bss_idx = MWX_AWDL_BSS_IDX;
	req.basic.tag = htole16(UNI_BSS_INFO_BASIC);
	req.basic.len = htole16(sizeof(req.basic));
	req.basic.active = on;
	req.basic.omac_idx = 1;
	req.basic.hw_bss_idx = 1;
	req.basic.band_idx = sc->sc_vif.band_idx;
	req.basic.conn_type = htole32(STA_TYPE_AP | NETWORK_P2P);
	req.basic.conn_state = 1;
	req.basic.wmm_idx = sc->sc_vif.wmm_idx;
	memcpy(req.basic.bssid, mwx_awdl_bssid, IEEE80211_ADDR_LEN);	/* the peers' frames are then this BSS's */
	req.basic.bmc_tx_wlan_idx = htole16(MWX_AWDL_BSS_WCID);
	req.basic.sta_idx = htole16(MWX_AWDL_BSS_WCID);
	if ((m = mwx_mcu_alloc_msg(sizeof(req))) == NULL)
		return ENOBUFS;
	memcpy(mtod(m, caddr_t), &req, sizeof(req));
	return mwx_awdl_mcu_sync(sc, MCU_UNI_CMD_BSS_INFO_UPDATE, m);
}

/*
 * A station record on AWDL's BSS, or its removal: for a peer, or with addr NULL the BSS's group record. STA_REC_BASIC
 * and the WTBL entry the MAC finds the peer by, its address against OMAC 1 (muar_idx 1), as
 * mt76_connac_mcu_sta_basic_tlv() and mt76_connac_mcu_wtbl_generic_tlv() fill them. A peer is a QoS station that takes
 * A-MPDUs of up to 64 KB with no MPDU spacing, as every AWDL device, and its frames keep their 802.11 header. Its type
 * is CONNECTION_INFRA_STA: mt76 gives this chip no P2P station types, and MtkWireless runs its AWDL peers so.
 */
static int
mwx_awdl_sta_cmd(struct mwx_softc *sc, uint16_t wcid, const uint8_t *addr, bool add)
{
	struct mbuf *m = mwx_alloc_sta_req_tlv(sizeof(struct sta_req_hdr));
	struct sta_rec_basic *basic;
	struct sta_rec_wtbl *wtbl;
	struct mwx_vif v;
	uint16_t tlvnum = 0, wnum = 0;
	int wlen = 0;

	if (m == NULL)
		return ENOBUFS;
	basic = (struct sta_rec_basic *)mwx_append_tlv(m, &tlvnum, STA_REC_BASIC, sizeof(*basic));
	basic->conn_type = htole32(addr == NULL ? STA_TYPE_BC | NETWORK_INFRA : STA_TYPE_STA | NETWORK_INFRA);
	basic->conn_state = add ? CONN_STATE_PORT_SECURE : CONN_STATE_DISCONNECT;
	basic->extra_info = htole16(EXTRA_INFO_VER | (add ? EXTRA_INFO_NEW : 0));
	basic->qos = add && addr != NULL;
	if (addr == NULL)
		memset(basic->peer_addr, 0xff, IEEE80211_ADDR_LEN);
	else
		memcpy(basic->peer_addr, addr, IEEE80211_ADDR_LEN);
	wtbl = (struct sta_rec_wtbl *)mwx_append_tlv(m, &tlvnum, STA_REC_WTBL, sizeof(*wtbl));
	wtbl->wlan_idx_lo = wcid & 0xff;
	wtbl->wlan_idx_hi = wcid >> 8;
	wtbl->operation = WTBL_RESET_AND_SET;
	if (add) {
		struct wtbl_generic *generic = (struct wtbl_generic *)mwx_append_tlv(m, &wnum, WTBL_GENERIC,
		    sizeof(*generic));
		struct wtbl_rx *rx;

		if (addr == NULL) {
			memset(generic->peer_addr, 0xff, IEEE80211_ADDR_LEN);
			generic->muar_idx = 0xe;
		} else {
			memcpy(generic->peer_addr, addr, IEEE80211_ADDR_LEN);
			generic->muar_idx = 1;
			generic->qos = 1;
		}
		rx = (struct wtbl_rx *)mwx_append_tlv(m, &wnum, WTBL_RX, sizeof(*rx));
		rx->rca1 = rx->rca2 = rx->rv = 1;
		wlen = sizeof(*generic) + sizeof(*rx);
		if (addr != NULL) {
			struct wtbl_hdr_trans *htr = (struct wtbl_hdr_trans *)mwx_append_tlv(m, &wnum, WTBL_HDR_TRANS,
			    sizeof(*htr));
			struct wtbl_ht *ht = (struct wtbl_ht *)mwx_append_tlv(m, &wnum, WTBL_HT, sizeof(*ht));

			htr->no_rx_trans = 1;
			ht->ht = 1;
			ht->af = 3;		/* 2^(13 + 3) - 1 = 65535 bytes */
			wlen += sizeof(*htr) + sizeof(*ht);
		}
	}
	wtbl->tlv_num = htole16(wnum);
	wtbl->len = htole16(sizeof(*wtbl) + wlen);
	memset(&v, 0, sizeof(v));
	v.idx = MWX_AWDL_BSS_IDX;
	return mwx_awdl_mcu_sync(sc, MCU_UNI_CMD_STA_REC_UPDATE, mwx_fill_sta_req_hdr(m, &v, 1, wcid, tlvnum));
}

/*
 * A record made anew. A record that is still in a firmware the interface stop did not reload answers the add with an
 * error (MtkWireless saw 0x10003 for a second add): it is removed and made again once.
 */
static int
mwx_awdl_sta_add(struct mwx_softc *sc, uint16_t wcid, const uint8_t *addr)
{
	int rv = mwx_awdl_sta_cmd(sc, wcid, addr, true);

	if (rv == EIO) {
		(void)mwx_awdl_sta_cmd(sc, wcid, addr, false);
		rv = mwx_awdl_sta_cmd(sc, wcid, addr, true);
	}
	if (rv == 0)
		mwx_mac_wtbl_update(sc, wcid);	/* as mt7921_mac_sta_add(): the entry's counters start at 0 */
	return rv;
}

/* AWDL's BSS and its group record, made before the first peer record. Radio thread. */
static bool
mwx_awdl_bss_up(struct mwx_softc *sc)
{
	struct mwx_awdl *a = sc->sc_awdl;
	int rv;

	if (a->bss_on)
		return true;
	if (a->bss_failed || !a->omac_on)
		return false;
	rv = mwx_awdl_bss_cmd(sc, true);
	if (rv == 0 && (rv = mwx_awdl_sta_add(sc, MWX_AWDL_BSS_WCID, NULL)) != 0 && rv != ECANCELED)
		(void)mwx_awdl_bss_cmd(sc, false);
	if (rv != 0) {
		if (rv != ECANCELED) {
			a->bss_failed = true;
			MWX_DEV_LOG(sc, "awdl: Block Ack off until the firmware starts again: AWDL's BSS refused (%d)\n",
			    rv);
		}
		return false;
	}
	a->bss_on = true;
	MWX_DEV_LOG(sc, "awdl: BSS %u on own MAC 1, group record %u\n", MWX_AWDL_BSS_IDX, MWX_AWDL_BSS_WCID);
	return true;
}

/* Gives a peer's station record back, and with it its agreements. Radio thread. */
static void
mwx_awdl_sta_free(struct mwx_softc *sc, struct mwx_awdl_peer *p)
{
	struct mwx_awdl *a = sc->sc_awdl;
	uint16_t wcid = p->wcid;

	if (wcid == 0)
		return;
	if (!mwx_awdl_fw_gone(sc))
		(void)mwx_awdl_sta_cmd(sc, wcid, p->addr, false);
	a->sta_used &= (uint8_t)~(1U << (wcid - MWX_WTBL_STA));
	p->wcid = 0;
	p->rx_tids = 0;
	p->tx_state = MWX_AWDL_TXBA_NONE;
	p->ba_out = 0;
	a->c_sta_del++;
	MWX_DEV_LOG(sc, "awdl: station record %u of %s given back\n", wcid, ether_sprintf(p->addr));
}

static bool mwx_awdl_delba(struct mwx_awdl *, const struct mwx_awdl_peer *, uint8_t, bool);

/*
 * The peer's station record, made when it has none. With every record taken, a peer that has moved no data either way
 * for MWX_AWDL_STA_IDLE_MS and has no aggregated frame out gives its own back, the least recently used of them, after
 * a DELBA for each of its agreements; a peer still moving data keeps its record, and the new one stays on the
 * single-frame path. 0; EBUSY when every record belongs to a peer moving data; another errno. Radio thread: p stays
 * valid across the waits, because only this thread makes or drops peer entries.
 */
static int
mwx_awdl_sta_get(struct mwx_softc *sc, struct mwx_awdl_peer *p)
{
	struct mwx_awdl *a = sc->sc_awdl;
	int i, rv;

	if (p->wcid != 0)
		return 0;
	if (!mwx_awdl_bss_up(sc))
		return ENODEV;
	if (a->sta_used == (1U << MWX_AWDL_STA_SLOTS) - 1) {
		struct mwx_awdl_peer *v = NULL;
		uint64_t now = mtkAwdlUptimeUs();

		for (i = 0; i < MWX_AWDL_PEERS; i++) {
			struct mwx_awdl_peer *q = &a->peers[i];
			uint64_t last = q->last_us > q->rx_us ? q->last_us : q->rx_us;	/* rx_us: the receive path's */

			if (q->wcid == 0 || q == p || q->ba_out != 0 ||
			    (last != 0 && (last > now || now - last < MWX_AWDL_STA_IDLE_MS * 1000ULL)))
				continue;
			if (v == NULL || q->used < v->used)
				v = q;
		}
		if (v == NULL) {
			a->c_sta_busy++;
			return EBUSY;
		}
		for (i = 0; i < 8; i++) {
			if (v->rx_tids & (1U << i))
				(void)mwx_awdl_delba(a, v, (uint8_t)i, false);
		}
		if (v->tx_state == MWX_AWDL_TXBA_ON)
			(void)mwx_awdl_delba(a, v, 0, true);
		mwx_awdl_sta_free(sc, v);
	}
	for (i = 0; i < MWX_AWDL_STA_SLOTS && (a->sta_used & (1U << i)); i++)
		;
	if (i == MWX_AWDL_STA_SLOTS)
		return ENOSPC;
	if (mwx_awdl_stop_asked(sc))
		return ECANCELED;
	rv = mwx_awdl_sta_add(sc, (uint16_t)(MWX_WTBL_STA + i), p->addr);
	if (rv != 0) {
		a->c_sta_fail++;
		if (rv != ECANCELED && a->c_sta_fail <= 8)
			MWX_DEV_LOG(sc, "awdl: station record %u for %s refused (%d)\n", MWX_WTBL_STA + i,
			    ether_sprintf(p->addr), rv);
		return rv;
	}
	a->sta_used |= (uint8_t)(1U << i);
	p->wcid = (uint16_t)(MWX_WTBL_STA + i);
	p->rx_tids = 0;
	p->tx_state = MWX_AWDL_TXBA_NONE;
	a->c_sta_add++;
	MWX_DEV_LOG(sc, "awdl: station record %u for %s\n", p->wcid, ether_sprintf(p->addr));
	return 0;
}

/*
 * An agreement in the firmware for a peer with a record, as mwx_mcu_sta_ba() sets one up with the station's access
 * point (mt76_connac_mcu_sta_ba()): the WTBL first, then STA_REC_BA, here on AWDL's BSS. tx: we are the originator.
 */
static int
mwx_awdl_ba_cmd(struct mwx_softc *sc, const struct mwx_awdl_peer *p, uint8_t tid, uint16_t ssn, uint16_t win,
    bool amsdu, bool tx, bool on)
{
	struct sta_rec_ba *sba;
	struct mwx_vif v;
	struct mbuf *m;
	uint16_t tlvnum = 0;
	int rv;

	memset(&v, 0, sizeof(v));
	v.idx = MWX_AWDL_BSS_IDX;
	if (sc->sc_wtbl_ba) {
		struct sta_rec_wtbl *wtbl;
		struct wtbl_ba *ba;
		uint16_t wnum = 0;

		if ((m = mwx_alloc_sta_req_tlv(sizeof(struct sta_req_hdr))) == NULL)
			return ENOBUFS;
		wtbl = (struct sta_rec_wtbl *)mwx_append_tlv(m, &tlvnum, STA_REC_WTBL, sizeof(*wtbl));
		wtbl->wlan_idx_lo = p->wcid & 0xff;
		wtbl->wlan_idx_hi = p->wcid >> 8;
		wtbl->operation = WTBL_SET;
		ba = (struct wtbl_ba *)mwx_append_tlv(m, &wnum, WTBL_BA, sizeof(*ba));
		ba->tid = tid;
		ba->ba_type = tx ? MT_BA_TYPE_ORIGINATOR : MT_BA_TYPE_RECIPIENT;
		ba->ba_winsize = on ? htole16(win) : 0;
		if (tx) {
			ba->sn = on ? htole16(ssn) : 0;
			ba->ba_en = on;
		} else {
			memcpy(ba->peer_addr, p->addr, IEEE80211_ADDR_LEN);
			ba->rst_ba_tid = tid;
			ba->rst_ba_sel = RST_BA_MAC_TID_MATCH;
			ba->rst_ba_sb = 1;
		}
		wtbl->tlv_num = htole16(wnum);
		wtbl->len = htole16(sizeof(*wtbl) + sizeof(*ba));
		rv = mwx_awdl_mcu_sync(sc, MCU_UNI_CMD_STA_REC_UPDATE, mwx_fill_sta_req_hdr(m, &v, 1, p->wcid, tlvnum));
		if (rv != 0)
			return rv;
		tlvnum = 0;
	}
	if ((m = mwx_alloc_sta_req_tlv(sizeof(struct sta_req_hdr))) == NULL)
		return ENOBUFS;
	sba = (struct sta_rec_ba *)mwx_append_tlv(m, &tlvnum, STA_REC_BA, sizeof(*sba));
	sba->tid = tid;
	sba->ba_type = tx ? MT_BA_TYPE_ORIGINATOR : MT_BA_TYPE_RECIPIENT;
	sba->amsdu = on && amsdu;
	sba->ba_en = on ? (uint8_t)(1U << tid) : 0;
	sba->ssn = htole16(on ? ssn : 0);
	sba->winsize = htole16(on ? win : 0);
	return mwx_awdl_mcu_sync(sc, MCU_UNI_CMD_STA_REC_UPDATE, mwx_fill_sta_req_hdr(m, &v, 1, p->wcid, tlvnum));
}

/* Queues a frame of ours for the next window on `ch`, as mtkAwdlRadioQueue() queues the interface's. */
static bool
mwx_awdl_queue(struct mwx_awdl *a, const uint8_t *frame, uint32_t len, uint8_t ch)
{
	bool ok = false;

	IOLockLock(a->lock);
	if (a->tx_count < MWX_AWDL_TX_SLOTS) {
		struct mwx_awdl_tx_slot *s = &a->tx[(a->tx_head + a->tx_count) % MWX_AWDL_TX_SLOTS];

		memcpy(s->data, frame, len);
		s->len = (uint16_t)len;
		s->ch = ch;
		s->deadline_us = mtkAwdlUptimeUs() + 500000ULL;
		a->tx_count++;
		ok = true;
	} else {
		a->c_tx_full++;
	}
	IOLockUnlock(a->lock);
	if (ok && a->rx_open)
		wakeup(&a->wake);
	return ok;
}

/* A Block Ack action frame of ours to a peer (IEEE80211_CATEG_BA), its fields after the action code. */
static bool
mwx_awdl_ba_action(struct mwx_awdl *a, const struct mwx_awdl_peer *p, uint8_t action, const uint8_t *fields,
    uint32_t n)
{
	uint8_t f[24 + 2 + 7];
	uint16_t sc = (uint16_t)((a->mgmt_sn++ & 0xfff) << 4);

	if (n > sizeof(f) - 26)
		return false;
	memset(f, 0, 24);
	f[0] = IEEE80211_FC0_TYPE_MGT | IEEE80211_FC0_SUBTYPE_ACTION;
	memcpy(f + 4, p->addr, IEEE80211_ADDR_LEN);
	memcpy(f + 10, a->addr, IEEE80211_ADDR_LEN);
	memcpy(f + 16, mwx_awdl_bssid, IEEE80211_ADDR_LEN);
	f[22] = sc & 0xff;
	f[23] = sc >> 8;
	f[24] = IEEE80211_CATEG_BA;
	f[25] = action;
	memcpy(f + 26, fields, n);
	return mwx_awdl_queue(a, f, 26 + n, p->ch);
}

/* ADDBA request: TID 0, immediate, our window, no A-MSDU, no timeout; the sequence numbers start at tx_sn. */
static bool
mwx_awdl_addba_req(struct mwx_awdl *a, struct mwx_awdl_peer *p)
{
	uint16_t params = 0x2 | (MWX_AWDL_BA_WINSIZE << 6), ssc = (uint16_t)(p->tx_sn << 4);
	uint8_t v[7];

	v[0] = ++p->tx_token;
	v[1] = params & 0xff;
	v[2] = params >> 8;
	v[3] = v[4] = 0;
	v[5] = ssc & 0xff;
	v[6] = ssc >> 8;
	p->tx_ssn = p->tx_sn;
	return mwx_awdl_ba_action(a, p, IEEE80211_ACTION_ADDBA_REQ, v, sizeof(v));
}

/* DELBA, reason 37 (the mechanism is not used any more); initiator: the agreement is one we send in. */
static bool
mwx_awdl_delba(struct mwx_awdl *a, const struct mwx_awdl_peer *p, uint8_t tid, bool initiator)
{
	uint16_t params = (initiator ? 0x0800 : 0) | (uint16_t)(tid << 12);
	uint8_t v[4] = { (uint8_t)(params & 0xff), (uint8_t)(params >> 8), 37, 0 };

	return mwx_awdl_ba_action(a, p, IEEE80211_ACTION_DELBA, v, sizeof(v));
}

/*
 * One frame on the air, as mt7921_mac_write_txwi() and mt7921_mac_write_txwi_80211() above describe a
 * management frame, with these differences: the queue is ALTX0; the rate is a fixed OFDM one whatever the
 * station's band (the station's basic rate is CCK on 2.4 GHz, which a 5 GHz window cannot send): 6 Mb/s for
 * action and group frames, and for unicast data the rate the peer's acknowledgements allow, VHT included: at 80 MHz
 * where the link is that wide (mwx_awdl_vht_rates), at 20 MHz in the other 5 GHz windows (mwx_awdl_vht20_rates);
 * the frame comes from OMAC 1 once that carries awdl0's address,
 * so that the MAC matches the peer's acknowledgement.
 */
/*
 * A unicast data frame to a peer we have a transmit agreement with, in a window on the station's channel: QoS data of
 * TID 0 with the agreement's own sequence numbers, in the best-effort queue of the station's EDCA set under the peer's
 * record, so that the MAC may gather the frames into A-MPDUs; a fixed HT rate, since an A-MPDU needs an HT PPDU. The
 * sequence number goes in the descriptor too (SN_VALID), as MtkWireless sends its AWDL frames. Only on the station's
 * channel: there the radio does not move, so nothing is left in that queue when a window ends elsewhere; frames for a
 * window away from it keep the single-frame path. The rest as mwx_awdl_tx().
 */
static int
mwx_awdl_tx_agg(struct mwx_softc *sc, struct mwx_awdl_peer *p, int slot, const uint8_t *frame, size_t len)
{
	struct mwx_awdl *a = sc->sc_awdl;
	struct mwx_txwi *mt;
	struct mt76_txwi *txp;
	struct mbuf *m;
	size_t qlen = len + 2;
	uint32_t val, rate = mwx_awdl_ht_rates[p->ba_rate];
	uint16_t sn = p->tx_sn;
	uint8_t *q;
	int rv;

	if (len < 24 || qlen > MCLBYTES)
		return EINVAL;
	m = MCLGETL(NULL, M_DONTWAIT, qlen);
	if (m == NULL)
		return ENOMEM;
	if (mbuf_next(m) != NULL) {
		m_freem(m);
		return ENOMEM;
	}
	mbuf_setlen(m, qlen);
	mbuf_pkthdr_setlen(m, qlen);
	q = mtod(m, uint8_t *);
	memcpy(q, frame, 24);
	q[0] = IEEE80211_FC0_TYPE_DATA | IEEE80211_FC0_SUBTYPE_QOS;
	q[22] = (uint8_t)(sn << 4);
	q[23] = (uint8_t)(sn >> 4);
	q[24] = 0;		/* TID 0, normal acknowledgement: inside an A-MPDU the implicit Block Ack request */
	q[25] = 0;
	memcpy(q + 26, frame + 24, len - 24);
	if ((mt = mwx_txwi_get(sc)) == NULL) {
		m_freem(m);
		return ENOBUFS;
	}
	txp = mt->mt_desc;
	memset(txp, 0, sizeof(*txp));
	val = ((qlen + MT_TXD_SIZE) & MT_TXD0_TX_BYTES_MASK) | MT_TX_TYPE_CT |
	    MT_TXD0_Q_IDX(sc->sc_vif.wmm_idx * MWX_MAX_WMM_SETS + MT_LMAC_AC01);
	txp->txwi[0] = htole32(val);
	val = MT_TXD1_LONG_FORMAT | (p->wcid & MT_TXD1_WLAN_IDX_MASK) |
	    ((1U << MT_TXD1_OWN_MAC_SHIFT) & MT_TXD1_OWN_MAC_MASK) |
	    MT_HDR_FORMAT_802_11 | MT_TXD1_HDR_INFO(26 / 2) | MT_TXD1_TID(0);
	if (sc->sc_vif.band_idx)
		val |= MT_TXD1_TGID;
	txp->txwi[1] = htole32(val);
	txp->txwi[2] = htole32(MT_TXD2_FRAME_TYPE(IEEE80211_FC0_TYPE_DATA >> IEEE80211_FC0_TYPE_SHIFT) |
	    MT_TXD2_SUB_TYPE(IEEE80211_FC0_SUBTYPE_QOS >> IEEE80211_FC0_SUBTYPE_SHIFT) | MT_TXD2_FIX_RATE |
	    MT_TXD2_HTC_VLD);
	txp->txwi[3] = htole32((8U << MT_TXD3_REM_TX_COUNT_SHIFT) | MT_TXD3_SN_VALID |
	    (((uint32_t)sn << 16) & MT_TXD3_SEQ));
	txp->txwi[5] = htole32(MT_PACKET_ID_NO_SKB & MT_TXD5_PID);
	txp->txwi[6] = htole32(MT_TXD6_FIXED_BW | ((rate << MT_TXD6_TX_RATE_SHIFT) & MT_TXD6_TX_RATE_MASK));
	txp->txwi[7] = htole32(MT_TXD7_TYPE(IEEE80211_FC0_TYPE_DATA >> IEEE80211_FC0_TYPE_SHIFT) |
	    MT_TXD7_SUB_TYPE(IEEE80211_FC0_SUBTYPE_QOS >> IEEE80211_FC0_SUBTYPE_SHIFT));

	/* From here on the error paths are those of mwx_tx(). */
	rv = mwx_txwi_enqueue(sc, mt, m);
	if (rv != 0) {
		m_freem(m);
		mwx_txwi_put(sc, mt);
		return rv;
	}
	if (mt->mt_idx < nitems(a->txinfo)) {
		a->txinfo[mt->mt_idx] = (uint8_t)(MWX_AWDL_TXI_AGG | (2 + slot) |
		    (a->rx_ch > 14 ? MWX_AWDL_TXI_5GHZ : 0));
		a->tx_inflight++;
	}
	rv = mwx_dma_txwi_enqueue(sc, &sc->sc_txq, mt);
	if (rv != 0) {
		if (mt->mt_idx < nitems(a->txinfo) && a->txinfo[mt->mt_idx] != 0) {
			a->txinfo[mt->mt_idx] = 0;
			a->tx_inflight--;
		}
		mwx_txwi_put(sc, mt);
		return rv;
	}
	p->tx_sn = (sn + 1) & 0xfff;
	p->ba_out++;
	if (p->ba_first_us == 0)
		p->ba_first_us = mtkAwdlUptimeUs();
	return 0;
}

/*
 * The group-addressed AWDL frames that go out without waiting for an acknowledgement (MT_TXD3_NO_ACK): bit 0 the
 * management ones (the PSF and MIF to ff:ff:ff:ff:ff:ff), bit 1 the data ones (mDNS to 33:33:...). mac80211 marks every
 * frame with a group address 1 IEEE80211_TX_CTL_NO_ACK, and mt76 sets MT_TXD3_NO_ACK from it on this chip (on the
 * MT7925 the same is MT_TXD3_BCM; bit 4 here is MT_TXD3_DAS). Without it the MAC may wait for an acknowledgement a
 * group frame never gets and send the frame again, up to its remaining count of 15: on the MT7925 each PSF and MIF
 * went out about 15 times, holding ALTX0 and the window while unicast waited behind it. mtkawdlnoack=<0-3> boot-arg,
 * 3 by default; 0 is the behaviour before.
 */
static uint8_t
mwx_awdl_noack_mask(void)
{
	static int mask = -1;

	if (mask < 0) {
		int v = 3;

		if (!PE_parse_boot_argn("mtkawdlnoack", &v, sizeof(v)))
			v = 3;
		mask = v & 3;
	}
	return (uint8_t)mask;
}

/* VHT at 20 MHz in the 5 GHz windows away from an 80 MHz link (mwx_awdl_vht20_rates); mtkawdlvht20=0 turns it off. */
static bool
mwx_awdl_vht20_on(void)
{
	static int on = -1;

	if (on < 0) {
		int v = 1;

		if (!PE_parse_boot_argn("mtkawdlvht20", &v, sizeof(v)))
			v = 1;
		on = v != 0;
	}
	return on != 0;
}

static int
mwx_awdl_tx(struct mwx_softc *sc, const uint8_t *frame, size_t len)
{
	struct mwx_awdl *a = sc->sc_awdl;
	struct mwx_txwi *mt;
	struct mt76_txwi *txp;
	struct mbuf *m;
	uint32_t val, rate, mode;
	uint16_t vht = 0;
	uint8_t type, subtype, omac, info = 1, ridx = MWX_AWDL_RATE_MIN, count = 15, bw = CMD_CBW_20MHZ;
	u_int hdrlen = 24;
	int rv, slot;

	if (len < 24 || len > MWX_AWDL_AMSDU_FRAME)
		return EINVAL;
	if (!mwx_tx_has_room(sc, 1))
		return ENOBUFS;
	type = (frame[0] & IEEE80211_FC0_TYPE_MASK) >> IEEE80211_FC0_TYPE_SHIFT;
	subtype = (frame[0] & IEEE80211_FC0_SUBTYPE_MASK) >> IEEE80211_FC0_SUBTYPE_SHIFT;
	if ((frame[0] & IEEE80211_FC0_TYPE_MASK) == IEEE80211_FC0_TYPE_DATA) {
		if (frame[0] & IEEE80211_FC0_SUBTYPE_QOS)
			hdrlen = 26;
		if (!(frame[4] & 1)) {
			struct mwx_awdl_peer *p = mwx_awdl_peer(a, frame + 4, &slot);
			uint64_t now = mtkAwdlUptimeUs();

			if (now - p->data_us > 1000000ULL) {
				p->data_us = now;
				p->data_n = 0;
			}
			p->data_n++;
			p->last_us = now;
			p->ch = a->rx_ch;
			if (p->tx_state == MWX_AWDL_TXBA_ON && a->win_onch && hdrlen == 24) {
				if (p->ba_ok >= MWX_AWDL_BA_PROBE || p->ba_out < MWX_AWDL_BA_PROBE)
					return mwx_awdl_tx_agg(sc, p, slot, frame, len);
				a->c_ba_hold++;		/* the agreement's first frames are not back yet */
			}
			ridx = p->rate;
			if (p->vht != 0 && a->win_vht == MWX_AWDL_VHT80)
				vht = mwx_awdl_vht_rates[p->vht - 1];
			else if (p->vht != 0 && a->win_vht == MWX_AWDL_VHT20)
				vht = mwx_awdl_vht20_rates[p->vht - 1];
			info = (uint8_t)(2 + slot);
			count = 8;
		}
	}
	if (len < hdrlen)
		return EINVAL;
	m = MCLGETL(NULL, M_DONTWAIT, len);
	if (m == NULL)
		return ENOMEM;
	if (mbuf_next(m) != NULL) {
		m_freem(m);
		return ENOMEM;
	}
	mbuf_setlen(m, len);
	mbuf_pkthdr_setlen(m, len);
	memcpy(mtod(m, uint8_t *), frame, len);
	if ((mt = mwx_txwi_get(sc)) == NULL) {
		m_freem(m);
		return ENOBUFS;
	}
	txp = mt->mt_desc;
	memset(txp, 0, sizeof(*txp));

	omac = a->omac_on ? 1 : sc->sc_vif.omac_idx;
	if (vht != 0) {
		rate = vht;
		if (a->win_vht == MWX_AWDL_VHT80)
			bw = CMD_CBW_80MHZ;
		else
			a->c_vht20++;
		info |= MWX_AWDL_TXI_VHT;
	} else {
		rate = mt76_rates[ridx].hw_value;
		mode = (rate >> 8) << MT_TX_RATE_MODE_SHIFT;
		rate = (rate & MT_TX_RATE_IDX_MASK) | (mode & MT_TX_RATE_MODE_MASK);
	}

	val = ((len + MT_TXD_SIZE) & MT_TXD0_TX_BYTES_MASK) | MT_TX_TYPE_CT | MT_TXD0_Q_IDX(MT_LMAC_ALTX0);
	txp->txwi[0] = htole32(val);
	val = MT_TXD1_LONG_FORMAT | (sc->sc_vif.vif_mn.wcid & MT_TXD1_WLAN_IDX_MASK) |
	    (((uint32_t)omac << MT_TXD1_OWN_MAC_SHIFT) & MT_TXD1_OWN_MAC_MASK) |
	    MT_HDR_FORMAT_802_11 | MT_TXD1_HDR_INFO(hdrlen / 2) | MT_TXD1_TID(0);
	if (sc->sc_vif.band_idx)
		val |= MT_TXD1_TGID;
	txp->txwi[1] = htole32(val);
	val = MT_TXD2_FRAME_TYPE(type) | MT_TXD2_SUB_TYPE(subtype) | MT_TXD2_FIX_RATE | MT_TXD2_HTC_VLD;
	if (frame[4] & 1)
		val |= MT_TXD2_MULTICAST;
	txp->txwi[2] = htole32(val);
	val = ((uint32_t)count << MT_TXD3_REM_TX_COUNT_SHIFT) | MT_TXD3_BA_DISABLE;
	if ((frame[4] & 1) && (mwx_awdl_noack_mask() & (type == 2 ? 2 : 1)))
		val |= MT_TXD3_NO_ACK;
	txp->txwi[3] = htole32(val);
	txp->txwi[5] = htole32(MT_PACKET_ID_NO_SKB & MT_TXD5_PID);
	txp->txwi[6] = htole32(MT_TXD6_FIXED_BW | (bw & MT_TXD6_BW) |
	    ((rate << MT_TXD6_TX_RATE_SHIFT) & MT_TXD6_TX_RATE_MASK));
	txp->txwi[7] = htole32(MT_TXD7_TYPE(type) | MT_TXD7_SUB_TYPE(subtype));

	/* From here on the error paths are those of mwx_tx(). */
	rv = mwx_txwi_enqueue(sc, mt, m);
	if (rv != 0) {
		m_freem(m);
		mwx_txwi_put(sc, mt);
		return rv;
	}
	if (mt->mt_idx < nitems(a->txinfo)) {
		a->txinfo[mt->mt_idx] = (uint8_t)(info | (a->rx_ch > 14 ? MWX_AWDL_TXI_5GHZ : 0));
		a->tx_inflight++;
	}
	rv = mwx_dma_txwi_enqueue(sc, &sc->sc_txq, mt);
	if (rv != 0) {
		if (mt->mt_idx < nitems(a->txinfo) && a->txinfo[mt->mt_idx] != 0) {
			a->txinfo[mt->mt_idx] = 0;
			a->tx_inflight--;
		}
		mwx_txwi_put(sc, mt);
	}
	return rv;
}

/*
 * From mwx_mac_tx_free(), main work loop: the hardware's report of one frame, the status and the number of
 * transmissions it took (MT_TX_FREE_STATUS and MT_TX_FREE_COUNT, as mt7921_mac_tx_free reads them). True when
 * the frame was the radio's.
 */
static bool
mwx_awdl_tx_done(struct mwx_softc *sc, uint16_t msdu, uint32_t txval)
{
	struct mwx_awdl *a = sc->sc_awdl;
	uint8_t info;
	bool failed = (txval & MT_TX_FREE_STATUS_MASK) != 0;
	uint32_t count = txval & MT_TX_FREE_COUNT_MASK;

	if (msdu >= nitems(a->txinfo) || (info = a->txinfo[msdu]) == 0)
		return false;
	a->txinfo[msdu] = 0;
	if (a->tx_inflight != 0 && --a->tx_inflight == 0)
		wakeup(&a->tx_inflight);
	if ((info & MWX_AWDL_TXI_PEER) == 1) {
		/* a group frame, or an action frame to one peer: how often the MAC sent it (mwx_awdl_noack_mask()) */
		if (failed)
			a->c_grp_fail++;
		else
			a->c_grp_ok++;
		a->c_grp_tries += count;
	}
	if ((info & MWX_AWDL_TXI_PEER) >= 2 && (info & MWX_AWDL_TXI_PEER) < 2 + MWX_AWDL_PEERS) {
		struct mwx_awdl_peer *p = &a->peers[(info & MWX_AWDL_TXI_PEER) - 2];
		int band = (info & MWX_AWDL_TXI_5GHZ) ? 1 : 0;

		if (failed)
			a->c_tx_uc_fail[band]++;
		else
			a->c_tx_uc_ok[band]++;
		if (info & MWX_AWDL_TXI_AGG) {
			/* The aggregated path: its own HT ladder, and the counts that tell if the agreement works. */
			if (p->ba_out != 0)
				p->ba_out--;
			if (failed) {
				a->c_ba_fail++;
				if (p->ba_fail < 0xffff)
					p->ba_fail++;
			} else {
				a->c_ba_ok++;
				if (p->ba_ok < 0xffff)
					p->ba_ok++;
			}
			if (failed || count > 3) {
				if (p->ba_rate > 0)
					p->ba_rate--;
				p->ba_good = 0;
			} else if (count <= 1) {
				if (++p->ba_good >= 10 && p->ba_rate + 1 < nitems(mwx_awdl_ht_rates)) {
					p->ba_rate++;
					p->ba_good = 0;
				}
			} else {
				p->ba_good = 0;
			}
		} else if (info & MWX_AWDL_TXI_VHT) {
			/* The VHT ladder: down one on a failure or a frame sent too many times, from its first step back to 54 Mb/s
			 * for a while; a frame sent again a few times only stops the climb. */
			if (failed)
				a->c_vht_fail++;
			else
				a->c_vht_ok++;
			if (failed || count > MWX_AWDL_VHT_SLOW) {
				p->vht_good = 0;
				if (p->vht > 1) {
					p->vht--;
				} else if (p->vht == 1) {
					p->vht = 0;
					p->vht_hold_us = mtkAwdlUptimeUs() + MWX_AWDL_VHT_HOLD_MS * 1000ULL;
				}
			} else if (count <= 1) {
				if (++p->vht_good >= 10 && p->vht != 0 && p->vht < nitems(mwx_awdl_vht_rates)) {
					p->vht++;
					p->vht_good = 0;
				}
			} else {
				p->vht_good = 0;
			}
		} else if (failed || count > 3) {
			if (p->rate > MWX_AWDL_RATE_MIN)
				p->rate--;
			p->good = 0;
		} else if (count <= 1) {
			if (++p->good >= 10) {
				if (p->rate < MWX_AWDL_RATE_MAX)
					p->rate++;
				else if (p->vht == 0 && mtkAwdlUptimeUs() >= p->vht_hold_us)
					p->vht = 1;	/* VHT from the next window that allows it */
				p->good = 0;
			}
		} else {
			p->good = 0;
		}
	} else if (failed) {
		a->c_tx_err++;
	}
	return true;
}

/* One A-MSDU subframe at offset `at` of the body: the frame's destination, source and MSDU length, then the MSDU, all
 * that follows its 24-byte header. Returns where the subframe ends. */
static uint32_t
mwx_awdl_amsdu_sub(uint8_t *b, uint32_t at, const uint8_t *f, uint32_t len)
{
	uint32_t msdu = len - 24;

	memcpy(b + at, f + 4, IEEE80211_ADDR_LEN);
	memcpy(b + at + 6, f + 10, IEEE80211_ADDR_LEN);
	b[at + 12] = (uint8_t)(msdu >> 8);
	b[at + 13] = (uint8_t)msdu;
	memcpy(b + at + 14, f + 24, msdu);
	return at + 14 + msdu;
}

/*
 * The unicast data frame in txbuf and those right behind it in the queue to the same peer, as one A-MSDU in amsdu: QoS
 * data of TID 0 with the A-MSDU bit, the first frame's header, each frame a subframe, 4-byte aligned. Only for a peer
 * whose frames go at VHT in this window (mwx_awdl_tx()), with no transmit agreement: an A-MSDU travels in an HT or later
 * PPDU, and at VHT the preamble, the wait and the acknowledgement take most of a single frame's air time. At most
 * MWX_AWDL_AMSDU_MAX: two frames of awdl0's MTU. Its length, or 0 when no frame joined the first. awdl->lock held: the
 * frames that join leave the queue.
 */
static uint32_t
mwx_awdl_amsdu(struct mwx_softc *sc, uint8_t ch, uint64_t now, uint32_t len)
{
	struct mwx_awdl *a = sc->sc_awdl;
	const struct mwx_awdl_peer *p;
	uint8_t *b = a->amsdu + 26;
	uint32_t at, end, n = 0;

	if (len < 32 || a->txbuf[0] != IEEE80211_FC0_TYPE_DATA || a->txbuf[1] != 0 || (a->txbuf[4] & 1) ||
	    !a->win_vht || (p = mwx_awdl_peer_find(a, a->txbuf + 4, NULL)) == NULL || p->vht == 0 ||
	    p->tx_state == MWX_AWDL_TXBA_ON)
		return 0;
	end = mwx_awdl_amsdu_sub(b, 0, a->txbuf, len);
	while (a->tx_count != 0) {
		const struct mwx_awdl_tx_slot *s = &a->tx[a->tx_head];

		at = (end + 3) & ~3U;
		if (s->deadline_us < now || (s->ch != ch && s->ch != 0) || s->len < 32 ||
		    s->data[0] != IEEE80211_FC0_TYPE_DATA || s->data[1] != 0 ||
		    memcmp(s->data + 4, a->txbuf + 4, IEEE80211_ADDR_LEN) != 0 ||
		    memcmp(s->data + 10, a->txbuf + 10, IEEE80211_ADDR_LEN) != 0 ||
		    at + 14 + (s->len - 24) > MWX_AWDL_AMSDU_MAX)
			break;
		memset(b + end, 0, at - end);
		end = mwx_awdl_amsdu_sub(b, at, s->data, s->len);
		a->tx_head = (a->tx_head + 1) % MWX_AWDL_TX_SLOTS;
		a->tx_count--;
		n++;
	}
	if (n == 0)
		return 0;
	memcpy(a->amsdu, a->txbuf, 24);
	a->amsdu[0] = IEEE80211_FC0_TYPE_DATA | IEEE80211_FC0_SUBTYPE_QOS;
	a->amsdu[24] = IEEE80211_QOS_AMSDU;	/* TID 0 */
	a->amsdu[25] = 0;
	a->c_amsdu++;
	a->c_amsdu_sub += n + 1;
	return 26 + end;
}

/* Sends the queued frames for `ch`; the ones for another channel or too old are dropped. Radio thread. */
static void
mwx_awdl_tx_flush(struct mwx_softc *sc, uint8_t ch)
{
	struct mwx_awdl *a = sc->sc_awdl;
	int budget;

	for (budget = MWX_AWDL_TX_SLOTS; budget > 0; budget--) {
		uint32_t len = 0, amsdu = 0;
		uint64_t now = mtkAwdlUptimeUs();
		bool freed = false;

		if (!mwx_tx_has_room(sc, 1))
			return;
		IOLockLock(a->lock);
		while (a->tx_count) {
			struct mwx_awdl_tx_slot *s = &a->tx[a->tx_head];
			if (s->deadline_us >= now && (s->ch == ch || s->ch == 0)) {
				len = s->len;
				memcpy(a->txbuf, s->data, len);
			} else {
				a->c_tx_stale++;
			}
			a->tx_head = (a->tx_head + 1) % MWX_AWDL_TX_SLOTS;
			a->tx_count--;
			if (len)
				break;
		}
		if (len != 0)
			amsdu = mwx_awdl_amsdu(sc, ch, now, len);
		if (a->tx_was_full && a->tx_count <= MWX_AWDL_TX_SLOTS / 2) {
			a->tx_was_full = false;
			freed = true;
		}
		IOLockUnlock(a->lock);
		if (freed)
			mtkAwdlRadioRoomFreed();
		if (!len)
			return;
		if (amsdu != 0) {
			if (mwx_awdl_tx(sc, a->amsdu, amsdu) != 0)
				a->c_tx_err++;
			continue;
		}
		mtkAwdlStampFrame(a->txbuf, len, mtkAwdlUptimeUs());
		if (mwx_awdl_tx(sc, a->txbuf, len) != 0)
			a->c_tx_err++;
		else if (a->txbuf[4] & 1 || (a->txbuf[0] & IEEE80211_FC0_TYPE_MASK) != IEEE80211_FC0_TYPE_DATA)
			a->c_tx_group++;
	}
}

/* Our transmit agreement with a peer ends; tell: a DELBA tells the peer. Radio thread. */
static void
mwx_awdl_txba_end(struct mwx_softc *sc, struct mwx_awdl_peer *p, const char *why, uint64_t retry_us, bool tell)
{
	struct mwx_awdl *a = sc->sc_awdl;

	if (p->tx_state == MWX_AWDL_TXBA_ON) {
		if (p->wcid != 0)
			(void)mwx_awdl_ba_cmd(sc, p, 0, 0, 0, false, true, false);
		if (tell)
			(void)mwx_awdl_delba(a, p, 0, true);
	}
	p->tx_state = MWX_AWDL_TXBA_NONE;
	p->tx_retry_us = mtkAwdlUptimeUs() + retry_us;
	MWX_DEV_LOG(sc, "awdl: transmit agreement with %s ended: %s (frames back %u, failed %u, out %u)\n",
	    ether_sprintf(p->addr), why, p->ba_ok, p->ba_fail, p->ba_out);
}

/*
 * One Block Ack action frame a peer sent to awdl0. An ADDBA request is answered once the firmware holds the receive
 * agreement on the peer's record, so that its A-MPDUs get their Block Ack; without a record it is declined (37). The
 * answer to our own request starts the transmit agreement; a DELBA ends either. Radio thread.
 */
static void
mwx_awdl_ba_frame(struct mwx_softc *sc, const struct mwx_awdl_baq *q)
{
	struct mwx_awdl *a = sc->sc_awdl;
	const uint8_t *f = q->frame;
	struct mwx_awdl_peer *p;
	uint16_t params, status;
	uint8_t tid, v[7];
	int slot;

	/* The first ones in the log, fields as they came: whether a peer takes part in Block Ack at all. */
	if (q->len >= 26 && a->c_ba_heard++ < 16) {
		char fields[3 * 8 + 1];
		uint32_t i, k = 0;

		for (i = 26; i < q->len && i < 26 + 8; i++)
			k += (uint32_t)snprintf(fields + k, sizeof(fields) - k, " %02x", f[i]);
		fields[k] = 0;
		MWX_DEV_LOG(sc, "awdl: Block Ack action %u from %s on %u:%s\n", f[25], ether_sprintf(f + 10), q->ch,
		    fields);
	}
	switch (q->len >= 26 ? f[25] : 0xff) {
	case IEEE80211_ACTION_ADDBA_REQ: {
		uint16_t timeout, ssn, win;
		bool amsdu;

		if (q->len < 33)
			return;
		params = (uint16_t)(f[27] | f[28] << 8);
		timeout = (uint16_t)(f[29] | f[30] << 8);
		ssn = (uint16_t)((f[31] | f[32] << 8) >> 4);
		tid = (params >> 2) & 0xf;
		win = params >> 6;
		amsdu = params & 1;
		if (win == 0 || win > MWX_AWDL_BA_WINSIZE)
			win = MWX_AWDL_BA_WINSIZE;
		p = mwx_awdl_peer(a, f + 10, &slot);
		if (q->ch != 0)
			p->ch = q->ch;
		status = IEEE80211_STATUS_REFUSED;
		/* Immediate Block Ack only; A-MSDUs inside are fine, whole or split by the hardware. */
		if (tid < 8 && (params & 0x2) && mwx_awdl_sta_get(sc, p) == 0 &&
		    mwx_awdl_ba_cmd(sc, p, tid, ssn, win, amsdu, false, true) == 0) {
			status = 0;
			p->rx_tids |= (uint8_t)(1U << tid);
			params = (uint16_t)((amsdu ? 1 : 0) | 0x2 | (tid << 2) | (win << 6));
			a->c_rxba_ok++;
		} else {
			a->c_rxba_no++;
		}
		if (a->c_rxba_ok + a->c_rxba_no <= 16)
			MWX_DEV_LOG(sc, "awdl: %s asks to send aggregated: tid %u ssn %u win %u amsdu %d -> %s\n",
			    ether_sprintf(p->addr), tid, ssn, win, amsdu, status == 0 ? "accepted" : "declined");
		if (mwx_awdl_stop_asked(sc))
			return;
		v[0] = f[26];
		v[1] = status & 0xff;
		v[2] = status >> 8;
		v[3] = params & 0xff;
		v[4] = params >> 8;
		v[5] = timeout & 0xff;
		v[6] = timeout >> 8;
		(void)mwx_awdl_ba_action(a, p, IEEE80211_ACTION_ADDBA_RESP, v, sizeof(v));
		break;
	}
	case IEEE80211_ACTION_ADDBA_RESP: {
		uint16_t win;

		if (q->len < 33)
			return;
		status = (uint16_t)(f[27] | f[28] << 8);
		params = (uint16_t)(f[29] | f[30] << 8);
		p = mwx_awdl_peer_find(a, f + 10, &slot);
		if (p == NULL || p->tx_state != MWX_AWDL_TXBA_ASKED || f[26] != p->tx_token || ((params >> 2) & 0xf) != 0) {
			/* An answer to no request of ours in progress: late, another token or another TID. */
			if (a->c_ba_skip++ < 8)
				MWX_DEV_LOG(sc, "awdl: answer from %s left aside: token %u of %u, state %u, tid %u, status %u\n",
				    ether_sprintf(f + 10), f[26], p != NULL ? p->tx_token : 0,
				    p != NULL ? p->tx_state : 0xff, (params >> 2) & 0xf, status);
			return;
		}
		win = params >> 6;
		if (win == 0 || win > MWX_AWDL_BA_WINSIZE)
			win = MWX_AWDL_BA_WINSIZE;
		p->tx_tries = 0;
		p->tx_state = MWX_AWDL_TXBA_NONE;
		if (status != 0) {
			a->c_txba_no++;
			p->tx_retry_us = mtkAwdlUptimeUs() + 60000000ULL;
			MWX_DEV_LOG(sc, "awdl: %s declined our transmit agreement (%u)\n", ether_sprintf(p->addr),
			    status);
			return;
		}
		if (p->wcid == 0 || mwx_awdl_ba_cmd(sc, p, 0, p->tx_ssn, win, false, true, true) != 0) {
			/* the peer holds the agreement: it is told it ended */
			p->tx_retry_us = mtkAwdlUptimeUs() + 60000000ULL;
			if (!mwx_awdl_stop_asked(sc))
				(void)mwx_awdl_delba(a, p, 0, true);
			a->c_txba_drop++;
			return;
		}
		p->tx_state = MWX_AWDL_TXBA_ON;
		p->tx_win = win;
		p->ba_ok = p->ba_fail = p->ba_out = 0;
		p->ba_first_us = 0;
		p->ba_good = 0;
		a->c_txba_ok++;
		MWX_DEV_LOG(sc, "awdl: transmit agreement with %s: ssn %u win %u\n", ether_sprintf(p->addr), p->tx_ssn,
		    win);
		break;
	}
	case IEEE80211_ACTION_DELBA:
		if (q->len < 30 || (p = mwx_awdl_peer_find(a, f + 10, &slot)) == NULL)
			return;
		params = (uint16_t)(f[26] | f[27] << 8);
		tid = params >> 12;
		if (params & 0x0800) {
			/* from the originator: its sending to us in that agreement ends */
			if (p->wcid != 0 && (p->rx_tids & (1U << tid))) {
				(void)mwx_awdl_ba_cmd(sc, p, tid, 0, 0, false, false, false);
				p->rx_tids &= (uint8_t)~(1U << tid);
			}
			a->c_rxba_del++;
		} else if (tid == 0 && p->tx_state != MWX_AWDL_TXBA_NONE) {
			mwx_awdl_txba_end(sc, p, "the peer ended it", 30000000ULL, false);
			a->c_txba_del++;
		}
		break;
	}
}

/*
 * The Block Ack part of the radio thread, run between windows: the frames the peers sent, our own requests to the
 * peers we send many frames to on the station's channel, and the agreements to give up. A transmit agreement ends when
 * frames handed over stay unreported MWX_AWDL_BA_PROBE_MS after its first one while fewer than MWX_AWDL_BA_PROBE came
 * back (a queue the firmware does not serve), or when it loses more than half of its first frames: they then take the
 * single-frame path again, for ten minutes. At most MWX_AWDL_BA_PROBE frames are out until the first ones are back.
 */
static void
mwx_awdl_ba_work(struct mwx_softc *sc)
{
	struct mwx_awdl *a = sc->sc_awdl;
	uint64_t now;
	uint8_t home;
	int i, rv;

	if (a->ba_off || !a->armed)
		return;
	while (a->baq_count != 0 && !mwx_awdl_stop_asked(sc)) {
		struct mwx_awdl_baq q = a->baq[a->baq_head];

		a->baq_head = (a->baq_head + 1) % MWX_AWDL_BAQ;
		a->baq_count--;
		mwx_awdl_ba_frame(sc, &q);
	}
	home = mwx_awdl_home(sc, NULL, NULL);
	for (i = 0; i < MWX_AWDL_PEERS && !mwx_awdl_stop_asked(sc); i++) {
		struct mwx_awdl_peer *p = &a->peers[i];

		if (p->rate == 0)
			continue;
		now = mtkAwdlUptimeUs();
		switch (p->tx_state) {
		case MWX_AWDL_TXBA_ASKED:
			if (now - p->tx_asked_us < MWX_AWDL_ADDBA_WAIT_MS * 1000ULL)
				break;
			a->c_txba_late++;
			p->tx_state = MWX_AWDL_TXBA_NONE;
			p->tx_retry_us = now + (++p->tx_tries >= 3 ? 30000000ULL : 1000000ULL);
			break;
		case MWX_AWDL_TXBA_ON:
			if (p->ba_out != 0 && p->ba_ok < MWX_AWDL_BA_PROBE && p->ba_first_us != 0 &&
			    now - p->ba_first_us > MWX_AWDL_BA_PROBE_MS * 1000ULL) {
				mwx_awdl_txba_end(sc, p, "its first frames did not come back", 600000000ULL, true);
				a->c_txba_drop++;
			} else if (p->ba_ok + p->ba_fail >= 16 && p->ba_ok + p->ba_fail <= 256 &&
			    p->ba_fail * 2 > p->ba_ok + p->ba_fail) {
				mwx_awdl_txba_end(sc, p, "most of its frames failed", 600000000ULL, true);
				a->c_txba_drop++;
			}
			break;
		default:
			if (p->data_n < MWX_AWDL_BA_BURST || now - p->data_us > 1000000ULL || now < p->tx_retry_us ||
			    home == 0 || p->ch != home)
				break;
			if ((rv = mwx_awdl_sta_get(sc, p)) != 0) {
				/* Every record with a peer moving data: asked again once one of them may be quiet. */
				p->tx_retry_us = mtkAwdlUptimeUs() +
				    (rv == EBUSY ? MWX_AWDL_STA_IDLE_MS * 1000ULL : 60000000ULL);
				break;
			}
			if (mwx_awdl_stop_asked(sc) || !mwx_awdl_addba_req(a, p))
				break;
			p->tx_state = MWX_AWDL_TXBA_ASKED;
			p->tx_asked_us = mtkAwdlUptimeUs();
			a->c_txba_ask++;
			break;
		}
	}
}

/*
 * Every record and agreement, and AWDL's BSS. With the firmware still there they are removed from it, before OMAC 1
 * goes; after a reload of the firmware only forgotten here. The peers keep their agreements until they find out.
 */
static void
mwx_awdl_ba_down(struct mwx_softc *sc, bool firmware)
{
	struct mwx_awdl *a = sc->sc_awdl;
	int i;

	for (i = 0; i < MWX_AWDL_PEERS; i++) {
		struct mwx_awdl_peer *p = &a->peers[i];

		if (p->wcid != 0) {
			if (firmware)
				mwx_awdl_sta_free(sc, p);
			p->wcid = 0;
		}
		p->rx_tids = 0;
		p->tx_state = MWX_AWDL_TXBA_NONE;
		p->ba_out = 0;
	}
	a->sta_used = 0;
	if (a->bss_on && firmware && !mwx_awdl_fw_gone(sc)) {
		(void)mwx_awdl_sta_cmd(sc, MWX_AWDL_BSS_WCID, NULL, false);
		(void)mwx_awdl_bss_cmd(sc, false);
		MWX_DEV_LOG(sc, "awdl: BSS %u removed\n", MWX_AWDL_BSS_IDX);
	}
	a->bss_on = false;
	a->baq_count = 0;
}

/* The MAC's A-MPDU length bins (reading clears them, mt7921_mac_reset_counters()), kept around aggregating windows. */
static void
mwx_awdl_agg_bins(struct mwx_softc *sc, bool keep)
{
	struct mwx_awdl *a = sc->sc_awdl;
	int i;

	for (i = 0; i < 4; i++) {
		uint32_t v = mwx_read(sc, MT_TX_AGG_CNT(0, i)), v2 = mwx_read(sc, MT_TX_AGG_CNT2(0, i));

		if (!keep)
			continue;
		a->c_agg[2 * i] += v & 0xffff;
		a->c_agg[2 * i + 1] += v >> 16;
		a->c_agg[8 + 2 * i] += v2 & 0xffff;
		a->c_agg[8 + 2 * i + 1] += v2 >> 16;
	}
}

/* Sleeps (gate open) until the uptime reaches `until_us`, a stop is asked for or another window arrives. */
static void
mwx_awdl_sleep_until(struct mwx_softc *sc, uint64_t until_us, bool wake_on_window)
{
	struct mwx_awdl *a = sc->sc_awdl;

	for (;;) {
		uint64_t now = mtkAwdlUptimeUs();
		uint64_t ns;

		if (now >= until_us || mwx_awdl_stop_asked(sc))
			return;
		if (wake_on_window) {
			bool newer;
			IOLockLock(a->lock);
			newer = a->next_valid;
			IOLockUnlock(a->lock);
			if (newer)
				return;
		}
		ns = (until_us - now) * 1000;
		if (ns > 5000000ULL)
			ns = 5000000ULL;
		tsleep_nsec(&a->wake, 0, "mwxawdl", ns);
	}
}

/*
 * Back to the station's own reception. The sniffer goes off first and its answer is waited for: until then
 * the receive tap keeps the station's filter in software. Then MT7921_FILTER_ENABLE alone, which is
 * mt7921_configure_filter() without FIF flags. The register is left to the firmware, which applies that
 * command on its own and does not answer it: writing MT_WF_RFCR here as well put two writers on it at the
 * moment the data RX ring was seen to stop. Nothing is sent to a firmware that is not there:
 * mwx_awdl_resume() or a reload sorts it. wait_ms 0 does not wait for the sniffer's answer: the filter
 * command still reaches the firmware after it, and the tap's grace covers the frames in between.
 */
static void
mwx_awdl_close_wait(struct mwx_softc *sc, uint32_t wait_ms)
{
	struct mwx_awdl *a = sc->sc_awdl;

	a->rx_open = false;
	if ((!a->sniffer_on && !a->filter_open) || mwx_awdl_fw_gone(sc))
		return;
	a->c_rx_close++;
	if (a->sniffer_on && mwx_awdl_sniffer(sc, 0, wait_ms) == 0) {
		a->sniffer_on = false;
		a->sniff_cfg = 0;
	}
	if (a->filter_open && !a->sniffer_on && mwx_awdl_rxfilter(sc, MWX_AWDL_FILTER_ENABLE) == 0)
		a->filter_open = false;
	if (!a->sniffer_on && !a->filter_open) {
		a->tap_grace_until_us = mtkAwdlUptimeUs() + MWX_AWDL_TAP_GRACE_MS * 1000ULL;
		a->tap_grace = true;
		a->tap_filter = false;
	}
}

static void
mwx_awdl_close(struct mwx_softc *sc)
{
	mwx_awdl_close_wait(sc, MWX_AWDL_MCU_MS);
}

/*
 * Whether the reception stays open after a window, pointed at the station's channel, until the next one. Linux sets
 * the sniffer of a monitor up once and afterwards only moves it to the next channel (mt7921_change_chanctx); here it
 * was switched off and the station's filter restored after every window, about twice a second, and every reception
 * stall whose commands were logged came at that switch-off, also with the radio given 20 ms to get home first. So
 * while the timeline runs and the station is associated, with nothing in its way, the reception is left open: it
 * closes when the timeline stops, the station scans or leaves its network (mwx_awdl_rx_release()), or a peer has
 * its record on AWDL's BSS, when on the station's channel neither the sniffer nor the filter is opened (`quiet` in
 * mwx_awdl_window()).
 */
static bool
mwx_awdl_rx_hold(struct mwx_softc *sc)
{
	struct mwx_awdl *a = sc->sc_awdl;

	return (a->sniffer_on || a->filter_open) && !mwx_awdl_stop_asked(sc) && !mwx_awdl_station_busy(sc) &&
	    mwx_awdl_home(sc, NULL, NULL) != 0 && !(a->bss_on && a->sta_used != 0);
}

/* Between windows the sniffer listens on the station's channel at the link's width, as in a window there. */
static int
mwx_awdl_rx_home(struct mwx_softc *sc)
{
	struct mwx_awdl *a = sc->sc_awdl;
	uint8_t bw = CMD_CBW_20MHZ, center = 0, home = mwx_awdl_home(sc, &bw, &center);
	uint32_t cfg;

	if (home == 0)
		return ENXIO;
	cfg = 1U << 24 | (uint32_t)center << 16 | (uint32_t)bw << 8 | home;
	if (!a->sniffer_on || cfg == a->sniff_cfg)
		return 0;
	a->sniff_cfg = 0;
	if (mwx_awdl_sniffer_config(sc, home, bw, center) != 0)
		return EIO;
	a->sniff_cfg = cfg;
	return 0;
}

/*
 * The station takes the radio for itself, a scan or the end of its association: a reception held open between
 * windows goes back to the station's settings first, without waiting, so the commands reach the firmware ahead of
 * the station's. A window in progress ends on its own, as the station is busy, and closes then.
 */
static void
mwx_awdl_rx_release(struct mwx_softc *sc)
{
	struct mwx_awdl *a = sc->sc_awdl;

	if (a->running || (!a->sniffer_on && !a->filter_open))
		return;
	mwx_awdl_close_wait(sc, 0);
}

/*
 * One window. Radio thread, inside the main command gate; every wait opens the gate, and after every wait
 * the reasons to stop are looked at again. A step that fails ends the window: the remain-on-channel period
 * is always given back with an abort and the reception goes back to the station's settings. After a window
 * that went well the reception only moves back to the station's channel while it may stay open
 * (mwx_awdl_rx_hold()).
 */
static void
mwx_awdl_window(struct mwx_softc *sc, const MtkAwdlWindow *w)
{
	struct mwx_awdl *a = sc->sc_awdl;
	uint8_t bw = CMD_CBW_20MHZ, center = 0;
	uint8_t home = mwx_awdl_home(sc, &bw, &center);
	bool onch = home != 0 && home == w->channel, agg;
	uint64_t now = mtkAwdlUptimeUs(), start = w->startUs, end, roc_deadline = 0, t;
	uint32_t lead = onch ? 5000 : a->lead_us, dwell = w->dwellMs;
	bool refresh, quiet, ok = false;
	int i;

	if (a->off || mwx_awdl_fw_gone(sc)) {
		a->c_gone++;
		return;
	}
	if (now < a->silent_until_us) {
		a->c_silent++;
		mwx_awdl_close(sc);
		return;
	}
	if (mwx_awdl_station_busy(sc) || now < a->backoff_until_us) {
		a->c_busy++;
		if (!mwx_awdl_rx_hold(sc))
			mwx_awdl_close(sc);
		return;
	}
	/*
	 * The access point keeps the radio until home_until_us, MWX_AWDL_HOME_MIN_MS after the last window away. Which
	 * EAWs are left to it is the interface's schedule (Apple's rows, by state), and this window is picked up right
	 * when the one before it ended: what counts is when its setup will begin, which waits instead.
	 */
	if (!onch && home != 0 &&
	    (start == 0 ? now : start > lead ? start - lead : 0) < a->home_until_us) {
		a->c_share++;
		mwx_awdl_sleep_until(sc, a->home_until_us, false);
		now = mtkAwdlUptimeUs();
		if (mwx_awdl_stop_asked(sc))
			return;
	}
	if (!mwx_awdl_channel_ok(sc, w->channel)) {
		a->c_badch++;
		if (!mwx_awdl_rx_hold(sc))
			mwx_awdl_close(sc);
		return;
	}
	if (start == 0)
		start = now;
	if (start > now + lead)
		mwx_awdl_sleep_until(sc, start - lead, false);
	now = mtkAwdlUptimeUs();
	if (mwx_awdl_stop_asked(sc))
		return;
	if (now > start + (uint64_t)dwell * 500 || mwx_awdl_station_busy(sc)) {
		a->c_late++;
		if (!mwx_awdl_rx_hold(sc))
			mwx_awdl_close(sc);
		return;
	}

	mwx_awdl_country(sc);
	if (!a->omac_on && !a->omac_failed) {
		if (mwx_awdl_omac(sc, 1) == 0)
			a->omac_on = true;
		else
			a->omac_failed = true;	/* frames then leave from the station's own MAC, unacknowledged */
		MWX_DEV_LOG(sc, "awdl: own MAC 1 %s for %s\n", a->omac_on ? "set" : "refused",
		    ether_sprintf(a->addr));
		if (mwx_awdl_stop_asked(sc))
			return;
	}
	if (home == 0)
		mwx_awdl_fullpwr(sc, true);

	/*
	 * Other devices' frames, from before the radio moves. A window right after one on the station's channel finds the
	 * reception as that one left it, the dense schedule's case: the filter and the sniffer's channel go to the firmware
	 * again only when they change, or every MWX_AWDL_RX_REFRESH_MS. Before, every window sent both, ten identical
	 * commands a second, and the data RX ring has been seen to stop for good after some hundreds of windows.
	 */
	refresh = now >= a->rx_cfg_us + MWX_AWDL_RX_REFRESH_MS * 1000ULL;
	/*
	 * On the station's channel, once a peer has its station record on AWDL's BSS, whose BSSID is AWDL's, the MAC takes
	 * the peers' frames as frames of a BSS of ours: neither the sniffer nor the filter for other BSSs is opened there, as
	 * ForbiddenItlwm keeps its listener out of promiscuous mode while the phone has a firmware station (their
	 * maintainer saw the firmware stop acknowledging the phone with promiscuous reception on). Here the data RX ring
	 * stopped right after the sniffer and the filter changed under load. The first such window closes what an earlier
	 * one opened; elsewhere, and before any record, the reception opens as before.
	 */
	quiet = onch && a->bss_on && a->sta_used != 0;
	if (quiet) {
		a->c_quiet++;
		if (a->sniffer_on || a->filter_open)
			mwx_awdl_close(sc);
	} else {
		a->tap_filter = true;
		if (!a->filter_open || refresh) {
			a->filter_open = true;
			if (mwx_awdl_rxfilter(sc, MWX_AWDL_FILTER_ENABLE | MWX_AWDL_FILTER_OTHER_BSS) != 0)
				goto fail;
		}
		if (!a->sniffer_on) {
			a->sniffer_on = true;
			if (mwx_awdl_sniffer(sc, 1, 0) != 0)
				goto fail;
		}
	}
	if (!onch) {
		/*
		 * A timed request, long enough for the grant, the wait for the start, the dwell, the drain and
		 * the abort, which has to reach the firmware before the period ends on its own (mt7925-macos-wifi
		 * 0b3a4f0, 2ab8329).
		 */
		uint32_t ahead = start > now ? (uint32_t)((start - now) / 1000) : 0;
		uint32_t duration = ahead + dwell + MWX_AWDL_ROC_SPARE_MS, took;
		uint64_t asked = mtkAwdlUptimeUs();

		if (mwx_awdl_roc(sc, w->channel, duration) != 0)
			goto fail;
		a->roc_active = true;
		a->off_since_us = asked;
		roc_deadline = asked + (uint64_t)duration * 1000;
		/*
		 * The grant is waited for, never given up early: an abort sent before a grant that was still
		 * coming is what the firmware did not survive (mt7925-macos-wifi 0f00c9a). Linux waits a second
		 * (mt7921_set_roc); the first grant for channel 149 after the firmware starts takes over 300 ms,
		 * and one that comes after the abort shows only as a stray event.
		 */
		while (a->roc_want && !a->roc_granted &&
		    mtkAwdlUptimeUs() < asked + MWX_AWDL_ROC_GRANT_MS * 1000ULL && !mwx_awdl_fw_gone(sc) &&
		    !a->dying)
			tsleep_nsec((void *)&a->roc_granted, 0, "mwxroc", 2000000ULL);
		took = (uint32_t)((mtkAwdlUptimeUs() - asked) / 1000);
		if (!a->roc_granted || a->roc_status != 0 || took >= MWX_AWDL_ROC_LATE_MS) {
			if (a->roc_granted && a->roc_status == 0) {
				a->c_roc_late++;
				ok = true;	/* only the time is gone: the reception may stay as it is */
			} else
				a->c_roc_fail++;
			if (a->c_roc_fail + a->c_roc_late <= 8)
				MWX_DEV_LOG(sc, "awdl: no window on %u: grant %d status %u after %u ms\n",
				    w->channel, a->roc_granted, a->roc_status, took);
			if (!a->roc_granted)
				a->backoff_until_us = mtkAwdlUptimeUs() + MWX_AWDL_ROC_BACKOFF_MS * 1000ULL;
			goto abort;
		}
		a->c_roc_ok++;
		/* The next window starts its setup this much earlier (an average, kept between 4 and 45 ms). */
		took = (uint32_t)(mtkAwdlUptimeUs() - now) + 3000;
		took = (7 * a->lead_us + took) / 8;
		a->lead_us = took < 4000 ? 4000 : took > 45000 ? 45000 : took;
	} else {
		a->c_onch++;
	}
	/*
	 * The sniffer follows the radio. On the station's channel it listens at the link's own width, or the link
	 * would be narrowed for the window (mt7925-macos-wifi 15a6659). Off it, on the 5 GHz social channels, at 80
	 * MHz: Apple devices run AWDL there at 80 MHz (44: 36-48, 149: 149-161), and a peer that sends its unicast
	 * data that wide is not heard by a radio listening at 20 (mt7925-macos-wifi completed its first transfer
	 * with the station at 80 MHz on 149; ForbiddenItlwm listens at 80 on 44 and 149). The remain-on-channel
	 * request stays at 20 MHz, as Linux makes it; our frames leave at 20 MHz on the primary channel.
	 */
	{
		uint8_t sbw = CMD_CBW_20MHZ, sctr = w->channel;

		uint32_t cfg;

		if (onch) {
			sbw = bw;
			sctr = center;
		} else if (w->channel == 44 || w->channel == 149) {
			sbw = CMD_CBW_80MHZ;
			sctr = w->channel == 44 ? 42 : 155;
		}
		cfg = 1U << 24 | (uint32_t)sctr << 16 | (uint32_t)sbw << 8 | w->channel;
		if (!quiet && (cfg != a->sniff_cfg || refresh)) {
			a->sniff_cfg = 0;
			if (mwx_awdl_sniffer_config(sc, w->channel, sbw, sctr) != 0)
				goto abort;
			a->sniff_cfg = cfg;
		}
		if (refresh && !quiet)
			a->rx_cfg_us = mtkAwdlUptimeUs();
	}
	if (mtkAwdlUptimeUs() < start)
		mwx_awdl_sleep_until(sc, start, false);
	now = mtkAwdlUptimeUs();
	if (now > start + 2000) {
		a->c_late++;
		a->late_us += now - start;
	}
	end = start + (uint64_t)dwell * 1000;
	if (roc_deadline != 0 && end + MWX_AWDL_ROC_STOP_MS * 1000ULL > roc_deadline) {
		end = roc_deadline - MWX_AWDL_ROC_STOP_MS * 1000ULL;
		a->c_roc_clip++;
	}
	a->rx_ch = w->channel;
	a->rx_open = true;
	a->c_win++;
	a->c_win_ch[mwx_awdl_bucket(w->channel)]++;
	/* Aggregated frames only where the radio stays: see mwx_awdl_tx_agg(). VHT at the link's 80 MHz there, at 20 MHz
	 * in the other 5 GHz windows. */
	a->win_onch = onch && !a->ba_off;
	if (onch && bw == CMD_CBW_80MHZ)
		a->win_vht = MWX_AWDL_VHT80;
	else if (w->channel > 14 && mwx_awdl_vht20_on())
		a->win_vht = MWX_AWDL_VHT20;
	else
		a->win_vht = 0;
	agg = false;
	for (i = 0; a->win_onch && i < MWX_AWDL_PEERS; i++)
		agg |= a->peers[i].tx_state == MWX_AWDL_TXBA_ON;
	if (agg)
		mwx_awdl_agg_bins(sc, false);

	for (;;) {
		now = mtkAwdlUptimeUs();
		if (now >= end || mwx_awdl_stop_asked(sc) || mwx_awdl_station_busy(sc))
			break;
		/* Nothing new for the hardware near the end: it would leave after the radio went back. */
		if (now + MWX_AWDL_TX_GUARD_MS * 1000ULL < end)
			mwx_awdl_tx_flush(sc, w->channel);
		tsleep_nsec(&a->wake, 0, "mwxawdl", end - now > 1000 ? 1000000ULL : (end - now) * 1000);
	}
	/* What the hardware still holds leaves on this channel, as far as the period allows. */
	t = mtkAwdlUptimeUs();
	while (a->tx_inflight != 0 && !mwx_awdl_fw_gone(sc) && !a->dying) {
		now = mtkAwdlUptimeUs();
		if (now >= t + MWX_AWDL_DRAIN_MS * 1000ULL ||
		    (roc_deadline != 0 && now + 15000 >= roc_deadline))
			break;
		tsleep_nsec(&a->tx_inflight, 0, "mwxawdl", 1000000ULL);
	}
	if (a->tx_inflight != 0) {
		a->c_drain++;
		a->tx_inflight = 0;		/* not carried into the next window */
	}
	if (agg && !mwx_awdl_fw_gone(sc))
		mwx_awdl_agg_bins(sc, true);
	a->win_onch = false;
	a->win_vht = false;
	a->rx_open = false;

	/*
	 * The next window on the station's channel right after this one keeps the reception open. The interface posts it
	 * one EAW ahead from the controller's work loop, which received data can hold up: then it is waited for, up to
	 * MWX_AWDL_CHAIN_WAIT_MS, rather than turning the sniffer and the filter off and on again while frames come in.
	 * Both reception stalls seen with their commands came right after such a change under load.
	 */
	if (onch && !mwx_awdl_stop_asked(sc) && !mwx_awdl_station_busy(sc)) {
		uint64_t until = mtkAwdlUptimeUs() + MWX_AWDL_CHAIN_WAIT_MS * 1000ULL;
		bool chained, posted, waited = false;

		for (;;) {
			IOLockLock(a->lock);
			posted = a->next_valid;
			chained = posted && a->next.channel == w->channel && a->next.startUs <= end + 10000;
			IOLockUnlock(a->lock);
			if (chained) {
				if (waited)
					a->c_chain_wait++;
				return;
			}
			if (posted || mtkAwdlUptimeUs() >= until || mwx_awdl_stop_asked(sc) ||
			    mwx_awdl_station_busy(sc))
				break;
			waited = true;
			mwx_awdl_sleep_until(sc, until, true);
		}
		if (waited)
			a->c_chain_miss++;
	}
	ok = true;
abort:
	if (a->roc_active) {
		/*
		 * Always, late or not: the firmware does not leave the channel by itself when the time is up
		 * (mt7925-macos-wifi b4ec2f0). The abort goes before the sniffer is switched off, whose answer can
		 * take long (67c0c45).
		 */
		mwx_awdl_roc_abort(sc);
		a->roc_active = false;
		now = mtkAwdlUptimeUs();
		a->off_us += now - a->off_since_us;
		/* The access point gets the radio back, at least for what it kept while the radio was away. */
		a->home_until_us = now + MWX_AWDL_HOME_MIN_MS * 1000ULL;
		/*
		 * The reception changes once the radio is home. The data RX ring stopped for good at the end of
		 * windows whose abort and sniffer switch-off had gone out in the same millisecond, the last frame
		 * taken as the filter went back to the station's, while the firmware was still bringing the radio
		 * back.
		 */
		if (a->sniffer_on || a->filter_open)
			mwx_awdl_sleep_until(sc, now + MWX_AWDL_HOME_SETTLE_MS * 1000ULL, false);
	}
fail:
	if (ok && mwx_awdl_rx_hold(sc) && mwx_awdl_rx_home(sc) == 0) {
		a->c_hold++;
		return;
	}
	mwx_awdl_close(sc);
}

static IOReturn
mwx_awdl_run_gated(OSObject *, void *arg, void *, void *, void *)
{
	struct mwx_softc *sc = (struct mwx_softc *)arg;
	struct mwx_awdl *a = sc->sc_awdl;
	MtkAwdlWindow w;
	uint64_t now;
	bool have;

	if (a == NULL || a->dying)
		return kIOReturnSuccess;
	a->running = true;
	for (;;) {
		if (!mwx_awdl_stop_asked(sc))
			mwx_awdl_ba_work(sc);
		IOLockLock(a->lock);
		have = a->armed && !a->paused && !a->dying && a->next_valid;
		if (have) {
			w = a->next;
			a->next_valid = false;
		}
		IOLockUnlock(a->lock);
		if (!have)
			break;
		mwx_awdl_window(sc, &w);
	}
	if (!a->dying) {
		/* While the timeline runs the reception waits open for the next window (mwx_awdl_rx_hold()). */
		if (!mwx_awdl_rx_hold(sc))
			mwx_awdl_close(sc);
		if (!a->armed && !a->paused) {
			mwx_awdl_fullpwr(sc, false);
			/*
			 * AWDL is off: the peers' records and AWDL's BSS go, then the chip stops acknowledging
			 * frames for awdl0's address.
			 */
			if (!mwx_awdl_fw_gone(sc))
				mwx_awdl_ba_down(sc, true);
			if (a->omac_on && !mwx_awdl_fw_gone(sc) && mwx_awdl_omac(sc, 0) == 0)
				a->omac_on = false;
		}
	}
	now = mtkAwdlUptimeUs();
	if (a->c_log < 4 || now - a->log_us > 30000000ULL) {
		char line[480];

		a->c_log++;
		a->log_us = now;
		mtkAwdlRadioSummary(line, sizeof(line));
		MWX_DEV_LOG(sc, "awdl: radio %s\n", line);
		mtkAwdlRadioChannels(line, sizeof(line));
		MWX_DEV_LOG(sc, "awdl: radio %s\n", line);
		mtkAwdlRadioBlockAck(line, sizeof(line));
		MWX_DEV_LOG(sc, "awdl: radio %s\n", line);
	}
	a->running = false;
	wakeup((void *)&a->running);
	return kIOReturnSuccess;
}

/* The radio thread. It only tries the main gate: a stop path that holds it may be waiting for this thread. */
static void
mwx_awdl_timer(OSObject *, IOTimerEventSource *)
{
	struct mwx_softc *sc = gMwxAwdlSc;
	struct mwx_awdl *a;
	IOCommandGate *gate;
	int tries;

	if (sc == NULL || (a = sc->sc_awdl) == NULL || (gate = sc->sc_rx_amsdu_gate) == NULL)
		return;
	for (tries = 0; tries < 5000 && !a->dying && !a->paused; tries++) {
		if (gate->attemptAction(mwx_awdl_run_gated, sc) != kIOReturnCannotLock)
			break;
		IOSleep(1);
	}
}

/*
 * From mwx_rx() before mt7921_mac_fill_rx(): the receive descriptor's payload format (kMtkAwdl*Part) when the frame is
 * QoS data of AWDL (no DS bits, the AWDL BSSID) that the hardware split out of an A-MSDU, else 0. The hardware splits
 * the A-MSDUs of peers that have a station record. hdr_gap: where the 802.11 header starts, 0 to work it out from the
 * descriptor's groups as mtkRxAmsduReadDescriptor() does.
 */
static uint8_t
mwx_awdl_rx_subframe(struct mbuf *m, uint16_t hdr_gap)
{
	const uint8_t *d = mtod(m, const uint8_t *);
	size_t off = hdr_gap;
	uint32_t rxd1, rxd2, rxd4;

	if (mbuf_len(m) < 24)
		return 0;
	rxd1 = mtkRxAmsduLe32(d + 4);
	rxd2 = mtkRxAmsduLe32(d + 8);
	rxd4 = mtkRxAmsduLe32(d + 16);
	if ((rxd4 & MT_RXD4_NORMAL_PAYLOAD_FORMAT) == 0)
		return 0;
	if (off == 0) {
		off = 24;
		if (rxd1 & MT_RXD1_NORMAL_GROUP_4)
			off += 16;
		if (rxd1 & MT_RXD1_NORMAL_GROUP_1)
			off += 16;
		if (rxd1 & MT_RXD1_NORMAL_GROUP_2)
			off += 8;
		if (rxd1 & MT_RXD1_NORMAL_GROUP_3)
			off += (rxd1 & MT_RXD1_NORMAL_GROUP_5) ? 80 : 8;
		off += 2 * ((rxd2 & MT_RXD2_NORMAL_HDR_OFFSET_MASK) >> MT_RXD2_NORMAL_HDR_OFFSET_SHIFT);
	}
	if (mbuf_len(m) < off + 26 + 2)
		return 0;
	d += off;
	if ((d[0] & (IEEE80211_FC0_TYPE_MASK | IEEE80211_FC0_SUBTYPE_QOS)) !=
	    (IEEE80211_FC0_TYPE_DATA | IEEE80211_FC0_SUBTYPE_QOS) || (d[1] & IEEE80211_FC1_DIR_MASK) != 0 ||
	    memcmp(d + 16, mwx_awdl_bssid, 6) != 0)
		return 0;
	return (uint8_t)(rxd4 & MT_RXD4_NORMAL_PAYLOAD_FORMAT);
}

/*
 * For the watchdog's reception check (MtkMwx::debugPoll()): the radio runs and a peer's data frame came less than 3 s
 * before `since_ms`, the last time the data RX ring moved.
 */
static bool
mwx_awdl_rx_expected(struct mwx_softc *sc, uint64_t since_ms)
{
	struct mwx_awdl *a = sc->sc_awdl;
	uint64_t since_us = since_ms * 1000ULL;
	int i;

	if (!a->armed)
		return false;
	for (i = 0; i < MWX_AWDL_PEERS; i++) {
		uint64_t rx = a->peers[i].rx_us;

		if (rx != 0 && rx + 3000000ULL >= since_us)
			return true;
	}
	return false;
}

/*
 * For the report of a reception stall (MtkMwx::debugPoll()): what the radio was doing when it was noticed, then its
 * counters, as in its periodic line.
 */
static void
mwx_awdl_stall_log(struct mwx_softc *sc)
{
	struct mwx_awdl *a = sc->sc_awdl;
	char line[480];

	MWX_DEV_LOG(sc, "awdl at the stall: radio %s, window %s, last on %u, remain-on-channel %s, sniffer %s, "
	    "filter %s\n", a->armed ? "started" : "stopped", a->rx_open ? "open" : "closed", a->rx_ch,
	    a->roc_active ? "held" : "none", a->sniffer_on ? "on" : "off", a->filter_open ? "open" : "station");
	mtkAwdlRadioSummary(line, sizeof(line));
	MWX_DEV_LOG(sc, "awdl: radio %s\n", line);
}

/*
 * After the reload that ends a reception stall, the search waits MWX_AWDL_REST_MS (mtkAwdlRadioResting()). The
 * Family turns AWDL on again after the reload, at boot or at an unlock with a denser sequence, and the ring stopped
 * again within the windows that followed: at the 17th, in one boot. Real use still runs the timeline.
 * Real use is also what kept the windows coming while the ring stopped again and again, its traffic or the
 * Family's search: after the second stall in a row (`again`) the radio takes no window at all for
 * MWX_AWDL_SILENT_MS. AWDL is away for that time; the station keeps its data.
 */
static void
mwx_awdl_rest(struct mwx_softc *sc, bool again)
{
	struct mwx_awdl *a = sc->sc_awdl;
	uint64_t now = mtkAwdlUptimeUs();

	a->rest_until_us = now + MWX_AWDL_REST_MS * 1000ULL;
	MWX_DEV_LOG(sc, "awdl: no search for %u s: the Family's sequence alone does not start the timeline\n",
	    MWX_AWDL_REST_MS / 1000);
	if (again) {
		a->silent_until_us = now + MWX_AWDL_SILENT_MS * 1000ULL;
		MWX_DEV_LOG(sc, "awdl: no windows for %u s: the reception stopped again within ten minutes\n",
		    MWX_AWDL_SILENT_MS / 1000);
	}
}

/*
 * One data frame of the station with its access point, for the interface's rating of the access point's traffic
 * (mtkAwdlRadioStationTraffic()): from mwx_tx() before the frame is handed over, and from mwx_rx() for what goes to
 * net80211, of which only what the access point sent counts. Each direction has its own counters, written by its own
 * path only. The frames addressed to one device are counted apart as well (mtkAwdlRadioStationUnicast()): the final
 * destination is the third address of a frame to the access point, the first one otherwise.
 */
static void
mwx_awdl_sta_traffic(struct mwx_softc *sc, struct mbuf *m, bool rx)
{
	struct mwx_awdl *a = sc->sc_awdl;
	struct ieee80211com *ic = &sc->sc_ic;
	const struct ieee80211_frame *wh;
	const uint8_t *da;

	if (mbuf_len(m) < sizeof(*wh))
		return;
	wh = mtod(m, const struct ieee80211_frame *);
	if ((wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) != IEEE80211_FC0_TYPE_DATA ||
	    (wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_NODATA))
		return;
	if (rx && (ic->ic_bss == NULL || !IEEE80211_ADDR_EQ(wh->i_addr2, ic->ic_bss->ni_bssid)))
		return;
	a->sta_pkts[rx]++;
	a->sta_bytes[rx] += mbuf_pkthdr_len(m);
	da = (wh->i_fc[1] & IEEE80211_FC1_DIR_MASK) == IEEE80211_FC1_DIR_TODS ? wh->i_addr3 : wh->i_addr1;
	if (!IEEE80211_IS_MULTICAST(da)) {
		a->sta_uc_pkts[rx]++;
		a->sta_uc_bytes[rx] += mbuf_pkthdr_len(m);
	}
}

/*
 * Receive tap, from mwx_rx() after mt7921_mac_fill_rx(): m starts with the 802.11 header. True when the
 * frame is not for net80211.
 *
 * AWDL frames go to the interface. For everything else, as long as the firmware may be handing over other
 * devices' frames (the sniffer or the wider filter is on, or was a moment ago), the station's own filter is
 * applied here: net80211 gets a frame only if it is addressed to the station, or group addressed in the
 * station's BSS. net80211 does not look at addr1 of a unicast frame, so a frame of our access point for
 * another station would otherwise run through this station's Block Ack window with that station's sequence
 * numbers.
 */
static bool
mwx_awdl_rx_tap(struct mwx_softc *sc, struct mbuf *m, const struct ieee80211_rxinfo *rxi, uint8_t part)
{
	struct mwx_awdl *a = sc->sc_awdl;
	struct ieee80211com *ic = &sc->sc_ic;
	size_t len = mbuf_pkthdr_len(m), n = len < 30 ? len : 30;
	uint8_t h[30];
	const uint8_t *bssid;
	bool filter = a->tap_filter;
	struct mwx_awdl_peer *p;

	if (!filter && a->tap_grace) {
		if (mtkAwdlUptimeUs() < a->tap_grace_until_us)
			filter = true;
		else
			a->tap_grace = false;
	}
	if (!filter && !a->armed)
		return false;
	if (len < 10 || mbuf_copydata(m, 0, n, h) != 0)
		return filter;

	if (n >= 24 && memcmp(h + 16, mwx_awdl_bssid, 6) == 0 &&
	    ((h[0] == 0xd0 && n >= 29 && h[24] == 0x7f && h[25] == 0x00 && h[26] == 0x17 && h[27] == 0xf2 &&
	    h[28] == 0x08) ||
	    ((h[0] & IEEE80211_FC0_TYPE_MASK) == IEEE80211_FC0_TYPE_DATA && (h[1] & IEEE80211_FC1_DIR_MASK) == 0))) {
		int rssi = rxi->rxi_rssi;

		if (!a->armed)
			return true;
		if (memcmp(h + 10, a->addr, 6) == 0) {
			a->c_rx_echo++;		/* our own frame */
			return true;
		}
		if (len > sizeof(a->rxbuf) || mbuf_copydata(m, 0, len, a->rxbuf) != 0) {
			a->c_rx_long++;
			return true;
		}
		if (part != kMtkAwdlWhole && h[0] != 0xd0) {
			/*
			 * A subframe of an A-MSDU the hardware split: the two alignment bytes it leaves after the
			 * 802.11 header go, as mt76 removes them (mt7921_mac_fill_rx()), and the frame reads as an
			 * A-MSDU of one subframe.
			 */
			size_t hl = 26 + ((h[1] & IEEE80211_FC1_ORDER) ? 4 : 0);

			if (len < hl + 2 + 14) {
				a->c_rx_long++;
				return true;
			}
			memmove(a->rxbuf + hl, a->rxbuf + hl + 2, len - hl - 2);
			len -= 2;
			a->c_split++;
		}
		if ((p = mwx_awdl_peer_find(a, h + 10, NULL)) != NULL) {
			if (rxi->rxi_chan || a->rx_ch)
				p->ch = (uint8_t)(rxi->rxi_chan ? rxi->rxi_chan : a->rx_ch);
			if (h[0] != 0xd0)
				p->rx_us = mtkAwdlUptimeUs();
		}
		if (h[0] == 0xd0) {
			uint8_t ch = rxi->rxi_chan ? rxi->rxi_chan : a->rx_ch;
			int b = mwx_awdl_bucket(ch);

			a->c_rx_af++;
			a->c_af_ch[b]++;
			if (!(a->first_af & (1U << b))) {
				a->first_af |= (uint8_t)(1U << b);
				MWX_DEV_LOG(sc, "awdl: first AWDL frame heard on channel %u, from %s\n", ch,
				    ether_sprintf(h + 10));
			}
		} else {
			a->c_rx_data++;
			if (!(h[4] & 1) && memcmp(h + 4, a->addr, 6) == 0)
				a->c_rx_uc++;
		}
		mtkAwdlInput(a->rxbuf, (uint32_t)len, rxi->rxi_chan ? rxi->rxi_chan : a->rx_ch,
		    rssi < 0 && rssi > -128 ? rssi : 0, mtkAwdlUptimeUs(), h[0] == 0xd0 ? kMtkAwdlWhole : part);
		return true;
	}
	/*
	 * A Block Ack action frame a peer sent to awdl0, for the radio thread, which answers it (mwx_awdl_ba_frame()). With
	 * the Order bit set a 4-byte HT Control field follows the sequence control and moves the category from byte 24 to
	 * 28: the queued copy leaves the field out and clears the bit, so it reads as the plain frame (wh0ami7/ForbiddenItlwm
	 * c65f795, a peer that answers at an HE rate with +HTC). htc= in the Block Ack line counts them.
	 */
	bool htc = n >= 30 && (h[1] & IEEE80211_FC1_ORDER) && h[28] == IEEE80211_CATEG_BA && h[29] <= 2;
	bool plain = n >= 25 && !(h[1] & IEEE80211_FC1_ORDER) && h[24] == IEEE80211_CATEG_BA;
	if (!a->ba_off && h[0] == (IEEE80211_FC0_TYPE_MGT | IEEE80211_FC0_SUBTYPE_ACTION) && (plain || htc) &&
	    memcmp(h + 16, mwx_awdl_bssid, 6) == 0 && memcmp(h + 4, a->addr, 6) == 0) {
		if (!a->armed)
			return true;
		if (a->baq_count < MWX_AWDL_BAQ) {
			struct mwx_awdl_baq *q = &a->baq[(a->baq_head + a->baq_count) % MWX_AWDL_BAQ];
			size_t skip = htc ? 4 : 0;

			q->len = (uint8_t)(len - skip < sizeof(q->frame) ? len - skip : sizeof(q->frame));
			if (mbuf_copydata(m, 0, 24, q->frame) == 0 &&
			    mbuf_copydata(m, 24 + skip, q->len - 24, q->frame + 24) == 0) {
				if (htc) {
					q->frame[1] &= ~IEEE80211_FC1_ORDER;
					a->c_ba_htc++;
				}
				q->ch = (uint8_t)(rxi->rxi_chan ? rxi->rxi_chan : a->rx_ch);
				a->baq_count++;
				if (a->timer != NULL && !a->dying)
					a->timer->setTimeoutUS(1);
			}
		} else {
			a->c_baq_full++;
		}
		return true;
	}
	/*
	 * Any other management frame sent to awdl0 is taken by nothing here; the first ones go to the log. A Block Ack
	 * frame of a peer with a BSSID other than AWDL's would end up here.
	 */
	if (n >= 26 && (h[0] & IEEE80211_FC0_TYPE_MASK) == IEEE80211_FC0_TYPE_MGT && memcmp(h + 4, a->addr, 6) == 0 &&
	    a->c_mgmt_other++ < 16)
		MWX_DEV_LOG(sc, "awdl: management frame %02x for awdl0 not taken: category %u action %u, BSSID "
		    "%02x:%02x:%02x:%02x:%02x:%02x, from %s\n", h[0], h[24], h[25], h[16], h[17], h[18], h[19], h[20],
		    h[21], ether_sprintf(h + 10));
	if (!filter)
		return false;

	/* The station's filter. */
	if (memcmp(h + 4, ic->ic_myaddr, 6) == 0)
		return false;
	if ((h[0] & IEEE80211_FC0_TYPE_MASK) != IEEE80211_FC0_TYPE_CTL && n >= 24 && (h[4] & 1)) {
		if ((h[0] & IEEE80211_FC0_TYPE_MASK) == IEEE80211_FC0_TYPE_MGT &&
		    (sc->sc_flags & (MWX_FLAG_SCANNING | MWX_FLAG_BGSCAN)) &&
		    ((h[0] & IEEE80211_FC0_SUBTYPE_MASK) == IEEE80211_FC0_SUBTYPE_BEACON ||
		    (h[0] & IEEE80211_FC0_SUBTYPE_MASK) == IEEE80211_FC0_SUBTYPE_PROBE_RESP))
			return false;		/* a scan wants every network */
		bssid = (h[0] & IEEE80211_FC0_TYPE_MASK) == IEEE80211_FC0_TYPE_DATA &&
		    (h[1] & IEEE80211_FC1_DIR_MASK) == IEEE80211_FC1_DIR_FROMDS ? h + 10 : h + 16;
		if (ic->ic_state >= IEEE80211_S_AUTH && ic->ic_bss != NULL &&
		    memcmp(bssid, ic->ic_bss->ni_bssid, 6) == 0)
			return false;
	}
	a->c_rx_other++;
	a->c_other_ch[mwx_awdl_bucket(rxi->rxi_chan ? rxi->rxi_chan : a->rx_ch)]++;
	return true;
}

/*
 * A stop path (interface down, system sleep, shutdown, reset, detach) is about to stop the chip: no new
 * window, and the one in progress ends now, giving its remain-on-channel period back if the firmware is
 * still there. Returns when the radio thread is out of the gated part. Called with or without the main gate
 * held; the wait opens it. mwx_awdl_resume() lets the radio work again.
 */
static void
mwx_awdl_quiesce(struct mwx_softc *sc)
{
	struct mwx_awdl *a = sc->sc_awdl;
	int tries;

	if (a == NULL)
		return;
	a->paused = true;
	wakeup(&a->wake);
	wakeup((void *)&a->roc_granted);
	for (tries = 0; a->running && tries < 400; tries++)
		tsleep_nsec((void *)&a->running, 0, "mwxawq", 5000000ULL);
	if (a->running)
		MWX_DEV_LOG(sc, "awdl: the radio thread did not stop in 2 s\n");
	a->rx_open = false;
}

/* The firmware was loaded again: it has none of the radio's settings. From mwx_init_hw(). */
static void
mwx_awdl_firmware_reset(struct mwx_softc *sc)
{
	struct mwx_awdl *a = sc->sc_awdl;

	if (a == NULL)
		return;
	a->omac_on = a->omac_failed = false;
	a->sniffer_on = a->filter_open = a->fullpwr = a->roc_active = false;
	a->sniff_cfg = 0;
	a->rx_open = false;
	a->roc_want = a->roc_granted = false;
	a->tap_filter = false;
	a->tx_inflight = 0;
	memset(a->txinfo, 0, sizeof(a->txinfo));
	memset(a->seq_cmd, 0, sizeof(a->seq_cmd));
	a->faults = 0;
	a->off = false;
	mwx_awdl_ba_down(sc, false);
	a->bss_failed = false;
	a->win_onch = false;
	a->win_vht = false;
}

/*
 * The interface runs again, at the end of mwx_init_hw(), which has just switched the sniffer off and written
 * the station's RX filter register. What a window left behind in a firmware that was not reloaded is taken
 * back, and windows are allowed again.
 */
static void
mwx_awdl_resume(struct mwx_softc *sc)
{
	struct mwx_awdl *a = sc->sc_awdl;

	if (a == NULL)
		return;
	if (a->filter_open)
		mwx_awdl_rxfilter(sc, MWX_AWDL_FILTER_ENABLE);
	a->sniffer_on = a->filter_open = a->roc_active = false;
	a->sniff_cfg = 0;
	a->omac_on = a->omac_failed = false;	/* the interface was registered again */
	/*
	 * Records and the BSS are made again when needed; one the firmware still holds is removed and made anew then
	 * (mwx_awdl_sta_add()).
	 */
	mwx_awdl_ba_down(sc, false);
	a->win_onch = false;
	a->win_vht = false;
	a->rx_open = false;
	a->roc_want = a->roc_granted = false;
	a->tap_grace_until_us = mtkAwdlUptimeUs() + MWX_AWDL_TAP_GRACE_MS * 1000ULL;
	a->tap_grace = true;
	a->tap_filter = false;
	a->faults = 0;
	a->off = false;
	a->paused = false;
	if (a->armed && a->timer != NULL)
		a->timer->setTimeoutUS(1000);
}

/* Attach: the functions below find the hardware layer through this. */
static void
mwx_awdl_attach(struct mwx_softc *sc)
{
	gMwxAwdlSc = sc;
}

/* Detach. The radio thread is waited for before its timer is taken off its work loop (see the top). */
static void
mwx_awdl_free(struct mwx_softc *sc)
{
	struct mwx_awdl *a = sc->sc_awdl;
	int tries;

	if (gMwxAwdlSc == sc)
		gMwxAwdlSc = NULL;
	if (a == NULL)
		return;
	a->armed = false;
	a->dying = true;
	wakeup(&a->wake);
	wakeup((void *)&a->roc_granted);
	if (a->timer != NULL)
		a->timer->cancelTimeout();
	for (tries = 0; a->running && tries < 400; tries++)
		tsleep_nsec((void *)&a->running, 0, "mwxawf", 5000000ULL);
	if (a->timer != NULL) {
		if (a->wl != NULL)
			a->wl->removeEventSource(a->timer);
		a->timer->release();
		a->timer = NULL;
	}
	if (a->wl != NULL) {
		a->wl->release();
		a->wl = NULL;
	}
	sc->sc_awdl = NULL;
	if (a->lock != NULL) {
		IOLockLock(a->lock);		/* a caller of the functions below that is still inside */
		IOLockUnlock(a->lock);
	}
	if (a->tx != NULL)
		IOFree(a->tx, sizeof(*a->tx) * MWX_AWDL_TX_SLOTS);
	if (a->lock != NULL)
		IOLockFree(a->lock);
	IOFree(a, sizeof(*a));
}

int
mtkAwdlRadioStart(const uint8_t *address)
{
	struct mwx_softc *sc = gMwxAwdlSc;
	struct mwx_awdl *a;

	if (sc == NULL || gMwxHal == NULL || address == NULL || sc->sc_detaching)
		return ENXIO;
	a = sc->sc_awdl;
	if (a == NULL) {
		a = (struct mwx_awdl *)IOMalloc(sizeof(*a));
		if (a == NULL)
			return ENOMEM;
		memset(a, 0, sizeof(*a));
		a->lock = IOLockAlloc();
		a->tx = (struct mwx_awdl_tx_slot *)IOMalloc(sizeof(*a->tx) * MWX_AWDL_TX_SLOTS);
		a->wl = IOWorkLoop::workLoop();
		a->timer = IOTimerEventSource::timerEventSource(gMwxHal, mwx_awdl_timer);
		a->lead_us = 30000;
		{
			int off = 0;

			a->ba_off = PE_parse_boot_argn("-mtkawdlnoba", &off, sizeof(off));
		}
		if (a->lock == NULL || a->tx == NULL || a->wl == NULL || a->timer == NULL ||
		    a->wl->addEventSource(a->timer) != kIOReturnSuccess) {
			if (a->timer != NULL)
				a->timer->release();
			if (a->wl != NULL)
				a->wl->release();
			if (a->tx != NULL)
				IOFree(a->tx, sizeof(*a->tx) * MWX_AWDL_TX_SLOTS);
			if (a->lock != NULL)
				IOLockFree(a->lock);
			IOFree(a, sizeof(*a));
			return ENOMEM;
		}
		sc->sc_awdl = a;
	}
	IOLockLock(a->lock);
	if (memcmp(a->addr, address, 6) != 0) {
		memcpy(a->addr, address, 6);
		a->omac_on = a->omac_failed = false;	/* the next window registers the new address */
	}
	a->next_valid = false;
	a->tx_count = 0;
	IOLockUnlock(a->lock);
	a->armed = true;
	MWX_DEV_LOG(sc, "awdl: radio started for %s%s\n", ether_sprintf((uint8_t *)a->addr),
	    a->ba_off ? ", Block Ack off (-mtkawdlnoba)" : "");
	return 0;
}

void
mtkAwdlRadioStop()
{
	struct mwx_softc *sc = gMwxAwdlSc;
	struct mwx_awdl *a = sc != NULL ? sc->sc_awdl : NULL;

	if (a == NULL)
		return;
	a->armed = false;
	IOLockLock(a->lock);
	a->next_valid = false;
	a->tx_count = 0;
	IOLockUnlock(a->lock);
	/* The radio thread ends the window in progress and closes the reception. */
	wakeup(&a->wake);
	if (a->timer != NULL && !a->dying)
		a->timer->setTimeoutUS(1);
}

int
mtkAwdlRadioWindow(const MtkAwdlWindow *window)
{
	struct mwx_softc *sc = gMwxAwdlSc;
	struct mwx_awdl *a = sc != NULL ? sc->sc_awdl : NULL;

	if (a == NULL || !a->armed || a->dying || window == NULL || !window->channel || !window->dwellMs)
		return ENXIO;
	if (a->off)
		return EIO;
	IOLockLock(a->lock);
	a->next = *window;
	a->next_valid = true;
	IOLockUnlock(a->lock);
	wakeup(&a->wake);
	if (a->timer != NULL)
		a->timer->setTimeoutUS(1);
	return 0;
}

int
mtkAwdlRadioQueue(const uint8_t *frame, uint32_t length, uint8_t channel, uint32_t maxAgeMs)
{
	struct mwx_softc *sc = gMwxAwdlSc;
	struct mwx_awdl *a = sc != NULL ? sc->sc_awdl : NULL;
	int rv = 0;

	if (a == NULL || !a->armed || a->dying)
		return ENXIO;
	if (frame == NULL || length < 24 || length > MWX_AWDL_FRAME_MAX)
		return EINVAL;
	IOLockLock(a->lock);
	if (a->tx_count >= MWX_AWDL_TX_SLOTS) {
		a->c_tx_full++;
		a->tx_was_full = true;
		rv = ENOBUFS;
	} else {
		struct mwx_awdl_tx_slot *s = &a->tx[(a->tx_head + a->tx_count) % MWX_AWDL_TX_SLOTS];
		memcpy(s->data, frame, length);
		s->len = (uint16_t)length;
		s->ch = channel;
		s->deadline_us = mtkAwdlUptimeUs() + (uint64_t)maxAgeMs * 1000;
		a->tx_count++;
	}
	IOLockUnlock(a->lock);
	if (rv == 0 && a->rx_open)
		wakeup(&a->wake);		/* a window is open: the frame leaves now */
	return rv;
}

uint32_t
mtkAwdlRadioRoom()
{
	struct mwx_softc *sc = gMwxAwdlSc;
	struct mwx_awdl *a = sc != NULL ? sc->sc_awdl : NULL;
	uint32_t room;

	if (a == NULL || !a->armed || a->dying)
		return 0;
	IOLockLock(a->lock);
	room = MWX_AWDL_TX_SLOTS - a->tx_count;
	IOLockUnlock(a->lock);
	return room;
}

bool
mtkAwdlRadioResting()
{
	struct mwx_softc *sc = gMwxAwdlSc;
	struct mwx_awdl *a = sc != NULL ? sc->sc_awdl : NULL;

	return a != NULL && mtkAwdlUptimeUs() < a->rest_until_us;
}

uint8_t
mtkAwdlRadioHomeChannel()
{
	struct mwx_softc *sc = gMwxAwdlSc;

	return sc != NULL ? mwx_awdl_home(sc, NULL, NULL) : 0;
}

bool
mtkAwdlRadioChannelAllowed(uint8_t channel)
{
	struct mwx_softc *sc = gMwxAwdlSc;

	return sc != NULL && mwx_awdl_channel_ok(sc, channel);
}

void
mtkAwdlRadioStationTraffic(uint64_t *packets, uint64_t *bytes)
{
	struct mwx_softc *sc = gMwxAwdlSc;
	struct mwx_awdl *a = sc != NULL ? sc->sc_awdl : NULL;

	*packets = a != NULL ? a->sta_pkts[0] + a->sta_pkts[1] : 0;
	*bytes = a != NULL ? a->sta_bytes[0] + a->sta_bytes[1] : 0;
}

void
mtkAwdlRadioStationUnicast(uint64_t *inPackets, uint64_t *inBytes, uint64_t *outPackets, uint64_t *outBytes)
{
	struct mwx_softc *sc = gMwxAwdlSc;
	struct mwx_awdl *a = sc != NULL ? sc->sc_awdl : NULL;

	*inPackets = a != NULL ? a->sta_uc_pkts[1] : 0;
	*inBytes = a != NULL ? a->sta_uc_bytes[1] : 0;
	*outPackets = a != NULL ? a->sta_uc_pkts[0] : 0;
	*outBytes = a != NULL ? a->sta_uc_bytes[0] : 0;
}

void
mtkAwdlRadioSummary(char *buffer, size_t length)
{
	struct mwx_softc *sc = gMwxAwdlSc;
	struct mwx_awdl *a = sc != NULL ? sc->sc_awdl : NULL;

	if (buffer == NULL || length == 0)
		return;
	if (a == NULL) {
		snprintf(buffer, length, "not started");
		return;
	}
	snprintf(buffer, length,
	    "win=%u onch=%u quiet=%u chain=%u/%u hold=%u/%u roc=%u/%u/%u stray=%u clip=%u lead=%uus late=%u/%llums away=%llums busy=%u share=%u "
	    "badch=%u gone=%u cmd=%u/%u tx grp=%u uc2=%u/%u uc5=%u/%u err=%u stale=%u full=%u drain=%u "
	    "rx af=%u data=%u uc=%u echo=%u other=%u long=%u omac1=%d cc=%c%c off=%d silent=%u grpst=%u/%u tries=%u "
	    "noack=%u",
	    a->c_win, a->c_onch, a->c_quiet, a->c_chain_wait, a->c_chain_miss, a->c_hold, a->c_rx_close, a->c_roc_ok,
	    a->c_roc_fail, a->c_roc_late,
	    a->c_roc_stray, a->c_roc_clip, a->lead_us,
	    a->c_late, a->late_us / 1000, a->off_us / 1000, a->c_busy, a->c_share, a->c_badch, a->c_gone, a->c_cmd,
	    a->c_cmd_err, a->c_tx_group, a->c_tx_uc_ok[0], a->c_tx_uc_fail[0], a->c_tx_uc_ok[1], a->c_tx_uc_fail[1],
	    a->c_tx_err, a->c_tx_stale, a->c_tx_full, a->c_drain, a->c_rx_af, a->c_rx_data, a->c_rx_uc, a->c_rx_echo,
	    a->c_rx_other, a->c_rx_long, a->omac_on, sc->sc_alpha2[0], sc->sc_alpha2[1], a->off, a->c_silent,
	    a->c_grp_ok, a->c_grp_fail, a->c_grp_tries, mwx_awdl_noack_mask());
}

/* Windows, AWDL frames and other frames heard, by social channel: a line of its own, the summary is long already. */
void
mtkAwdlRadioChannels(char *buffer, size_t length)
{
	struct mwx_softc *sc = gMwxAwdlSc;
	struct mwx_awdl *a = sc != NULL ? sc->sc_awdl : NULL;

	if (buffer == NULL || length == 0)
		return;
	if (a == NULL) {
		snprintf(buffer, length, "not started");
		return;
	}
	snprintf(buffer, length, "win/af/other ch6=%u/%u/%u ch44=%u/%u/%u ch149=%u/%u/%u chx=%u/%u/%u",
	    a->c_win_ch[0], a->c_af_ch[0], a->c_other_ch[0], a->c_win_ch[1], a->c_af_ch[1], a->c_other_ch[1],
	    a->c_win_ch[2], a->c_af_ch[2], a->c_other_ch[2], a->c_win_ch[3], a->c_af_ch[3], a->c_other_ch[3]);
}

/*
 * Block Ack, a line of its own: records made/given back/refused; receive agreements accepted/declined/ended by the
 * peer; transmit agreements asked/accepted/declined/unanswered/ended by the peer/given up; aggregated-path frames
 * reported sent/failed and held back while an agreement was young; A-MSDU subframes split by the hardware; then each
 * peer with a record (WCID, receive TIDs, transmit state, HT rung) and the A-MPDU length bins that are not zero.
 */
void
mtkAwdlRadioBlockAck(char *buffer, size_t length)
{
	struct mwx_softc *sc = gMwxAwdlSc;
	struct mwx_awdl *a = sc != NULL ? sc->sc_awdl : NULL;
	size_t n;
	int i;

	if (buffer == NULL || length == 0)
		return;
	if (a == NULL) {
		snprintf(buffer, length, "not started");
		return;
	}
	n = (size_t)snprintf(buffer, length,
	    "ba off=%d bss=%d%s sta=%u/%u/%u/%u rx=%u/%u/%u tx=%u/%u/%u/%u/%u/%u frames=%u/%u hold=%u split=%u qfull=%u "
	    "htc=%u",
	    a->ba_off, a->bss_on, a->bss_failed ? "!" : "", a->c_sta_add, a->c_sta_del, a->c_sta_fail, a->c_sta_busy,
	    a->c_rxba_ok,
	    a->c_rxba_no, a->c_rxba_del, a->c_txba_ask, a->c_txba_ok, a->c_txba_no, a->c_txba_late, a->c_txba_del,
	    a->c_txba_drop, a->c_ba_ok, a->c_ba_fail, a->c_ba_hold, a->c_split, a->c_baq_full, a->c_ba_htc);
	for (i = 0; i < MWX_AWDL_PEERS && n < length; i++) {
		const struct mwx_awdl_peer *p = &a->peers[i];

		if (p->wcid != 0)
			n += (size_t)snprintf(buffer + n, length - n, " %u:rx%02x/tx%u/ht%u/r%u/v%u", p->wcid, p->rx_tids,
			    p->tx_state, p->ba_rate, p->rate, p->vht);
	}
	if (n < length)
		n += (size_t)snprintf(buffer + n, length - n, " vht=%u/%u vht20=%u%s amsdu=%u/%u", a->c_vht_ok,
		    a->c_vht_fail, a->c_vht20, mwx_awdl_vht20_on() ? "" : "(off)", a->c_amsdu, a->c_amsdu_sub);
	if (n < length)
		n += (size_t)snprintf(buffer + n, length - n, " agg");
	for (i = 0; i < 16 && n < length; i++) {
		if (a->c_agg[i] != 0)
			n += (size_t)snprintf(buffer + n, length - n, " %d:%u", i, a->c_agg[i]);
	}
}
#endif /* MTK_AWDL */
