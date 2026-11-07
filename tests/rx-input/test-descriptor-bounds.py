#!/usr/bin/env python3
"""Run the real RX parser under ASan/UBSan with truncated optional RX groups
and check the packet number it reads from GROUP_1 of decrypted frames."""
from pathlib import Path
import re, subprocess, tempfile, sys
root = Path(__file__).resolve().parents[2]
s = (root / 'SimpleMtkWlan/hal_mwx/MtkMwx.cpp').read_text()
f = s[s.index('int\nmt7921_mac_fill_rx('):s.index('\nuint32_t\nmt7921_mac_tx_rate_val(')]
sec = re.search(r'struct mwx_rx_sec \{.*?\n\};', s, re.S).group()
if '--test-original' in sys.argv:
 start=f.index('\n\t/* Validate the entire optional descriptor')
 end=f.index('\n\tif (rxd1 & MT_RXD1_NORMAL_BAND_IDX)', start)
 f=f[:start]+f[end:]
h = (root / 'SimpleMtkWlan/hal_mwx/if_mwxreg.h').read_text()
defines = '\n'.join(line for line in h.splitlines() if re.match(r'#define\s+MT_(RXD[0-9]|PRXV_RCPI)', line) and not line.endswith('\\'))
stub = r'''
#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <cerrno>
#include <vector>
#include "RxAmpduStats.hpp"
#include <sys/mman.h>
#include <unistd.h>
#define MWX_DEV_LOG(...) ((void)0)
#define MWX_RX_DROP_LOG(...) ((void)0)
#define le32toh(v) (v)
#define MAX(a,b) ((a)>(b)?(a):(b))
#define DEVNAME(sc) "test"
#define IEEE80211_S_RUN 5
#define IEEE80211_RXI_HWDEC 1
#define IEEE80211_FC1_PROTECTED 0x40
#define IEEE80211_FC0_TYPE_MASK 0x0c
#define IEEE80211_FC0_TYPE_DATA 0x08
#define IEEE80211_FC0_SUBTYPE_NODATA 0x40
struct mbuf { uint8_t *data; size_t len; };
size_t mbuf_len(mbuf *m) { return m->len; }
#define mtod(m,t) reinterpret_cast<t>((m)->data)
void m_adj(mbuf *m, size_t n) { m->data += n; m->len -= n; }
struct ieee80211_frame_min { uint8_t data[10]; };
struct ieee80211_frame { uint8_t i_fc[2]; uint8_t data[22]; };
struct ieee80211_rxinfo { unsigned rxi_flags=0; int rxi_rssi=0; unsigned rxi_chan=0; };
struct ieee80211com { int ic_state=0; struct { unsigned is_rx_decryptcrc=0, is_rx_locmicfail=0; } ic_stats; };
struct mwx_softc { ieee80211com sc_ic; MtkRxAmpduStats sc_rx_ampdu_stats={}; unsigned sc_rx_fill_fail=0; struct { int num_streams=2; unsigned antenna_mask=3; } sc_capa; };
void ieee80211_michael_mic_failure(ieee80211com*, int) {}
int rcpi_to_rssi(unsigned shift, uint32_t value) { return int((value >> shift) & 255) / 2 - 110; }
void mwx_rx_rate_account(uint32_t) {}
'''+'\n'+sec+'\n'+r'''
static uint8_t *put32(uint8_t *p, uint32_t v) { memcpy(p, &v, 4); return p + 4; }
'''
main = r'''
int main() {
 unsigned flags[] = {MT_RXD1_NORMAL_GROUP_1, MT_RXD1_NORMAL_GROUP_2,
  MT_RXD1_NORMAL_GROUP_3, MT_RXD1_NORMAL_GROUP_4, MT_RXD1_NORMAL_GROUP_5};
 unsigned rejected=0, accepted=0;
 for (unsigned mask=0; mask<32; ++mask) {
  unsigned rxd1=0, words=6;
  for (unsigned i=0;i<5;++i) if(mask & (1u<<i)) rxd1 |= flags[i];
  if(mask&1)words+=4;if(mask&2)words+=2;if(mask&4)words+=2;if(mask&8)words+=4;
  if((mask&4)&&(mask&16))words+=18;
  for (unsigned len=24;len<words*4+sizeof(ieee80211_frame_min);len+=4) {
   size_t page=static_cast<size_t>(getpagesize());
   auto region=static_cast<uint8_t*>(mmap(nullptr,page*2,PROT_READ|PROT_WRITE,MAP_PRIVATE|MAP_ANON,-1,0));
   assert(region!=MAP_FAILED);assert(mprotect(region+page,page,PROT_NONE)==0);
   auto raw=region+page-len;memset(raw,0,len);
   memcpy(raw+4,&rxd1,4);mbuf m{raw,len};mwx_softc sc;ieee80211_rxinfo rx;mwx_rx_sec sec;
   assert(mt7921_mac_fill_rx(&sc,&m,&rx,&sec)!=0);++rejected;assert(munmap(region,page*2)==0);
  }
  unsigned len=words*4+sizeof(ieee80211_frame_min);
  auto raw=static_cast<uint8_t*>(malloc(len));memset(raw,0,len);memcpy(raw+4,&rxd1,4);
  mbuf m{raw,len};mwx_softc sc;ieee80211_rxinfo rx;mwx_rx_sec sec;
  assert(mt7921_mac_fill_rx(&sc,&m,&rx,&sec)==0);++accepted;assert(!sec.pn_valid&&sec.sec_mode==0);free(raw);
 }
 // GROUP_1 of a decrypted frame carries the packet number, PN0 first, after
 // GROUP_4 when present; CM/CLM or a missing group leave no packet number.
 const uint8_t pn[6]={0x11,0x22,0x33,0x44,0x55,0x66};unsigned checked=0;
 for(unsigned variant=0;variant<5;++variant){
  bool g4=variant==1,cm=variant==2,nog1=variant==3,clm=variant==4;
  uint32_t rxd1=(nog1?0:MT_RXD1_NORMAL_GROUP_1)|(g4?MT_RXD1_NORMAL_GROUP_4:0)|(4u<<16)|(2u<<21)|(cm?MT_RXD1_NORMAL_CM:0)|(clm?MT_RXD1_NORMAL_CLM:0);
  size_t words=6+(g4?4:0)+(nog1?0:4),len=words*4+26;std::vector<uint8_t>b(len,0);
  put32(b.data()+4,rxd1);put32(b.data()+12,MT_RXD3_NORMAL_U2M);if(!nog1)memcpy(b.data()+24+(g4?16:0),pn,6);
  b[words*4]=0x88;b[words*4+1]=0x42;b[words*4+24]=3;
  mbuf m{b.data(),len};mwx_softc sc;ieee80211_rxinfo rx;mwx_rx_sec sec;memset(&sec,0x5a,sizeof sec);
  assert(mt7921_mac_fill_rx(&sc,&m,&rx,&sec)==0);assert(m.len==26&&m.data[1]==0x42); /* Protected untouched here */
  assert(sec.sec_mode==4&&sec.key_id==2);
  if(cm||clm){assert(!(rx.rxi_flags&IEEE80211_RXI_HWDEC)&&!sec.pn_valid);}
  else{assert(rx.rxi_flags&IEEE80211_RXI_HWDEC);assert(sec.pn_valid==!nog1);if(!nog1)assert(!memcmp(sec.pn,pn,6));}
  ++checked;
 }
 assert(checked==5);
 printf("PASS: %u truncated descriptors rejected; %u complete group layouts accepted; GROUP_1 packet number read for decrypted frames only, after GROUP_4; descriptor bounds checks passed\n",rejected,accepted);
}
'''
with tempfile.TemporaryDirectory(prefix='mtk-rx-bounds-') as d:
 p=Path(d);(p/'test.cpp').write_text(stub+defines+'\n'+f+main)
 sanitizers = ([] if '--no-sanitizers' in sys.argv else
  ['-fsanitize=undefined'] if '--ubsan' in sys.argv else ['-fsanitize=undefined'])
 subprocess.run(['clang++','-std=c++14','-I',str(root/'SimpleMtkWlan/hal_mwx')]+sanitizers+['-g',str(p/'test.cpp'),'-o',str(p/'test')],check=True)
 r=subprocess.run([str(p/'test')])
 if '--test-original' in sys.argv:
  assert r.returncode < 0, 'Original parser should fault against a protected guard page'
  print('PASS: original parser faults on truncated optional RX group as expected')
 else:
  r.check_returncode()
