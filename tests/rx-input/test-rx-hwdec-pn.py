#!/usr/bin/env python3
"""Actual RX parser, CCMP header restore and net80211 hardware-decrypt check.

The packet number of a hardware-decrypted frame travels from GROUP_1 of the
RX descriptor into the restored CCMP header, and the production
ieee80211_input_hwdecrypt() then accepts the frame once and rejects a replay.
"""
from pathlib import Path
import ast,re,subprocess,tempfile
root=Path(__file__).resolve().parents[2]
fixture=ast.parse((root/'tests/rx-input/test-rx-amsdu-metadata.py').read_text())
pre=next(ast.literal_eval(n.value) for n in fixture.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='pre' for t in n.targets)).replace('Packet','mbuf')
hal=(root/'SimpleMtkWlan/hal_mwx/MtkMwx.cpp').read_text()
regs=(root/'SimpleMtkWlan/hal_mwx/if_mwxreg.h').read_text()
net=root/'mtk80211/openbsd/net80211'
defines='\n'.join(l for l in regs.splitlines() if re.match(r'#define\s+MT_(RXD[0-9]|PRXV_RCPI)',l) and not l.endswith('\\'))
cipher=re.search(r'enum mt76_cipher_type \{.*?\};',regs,re.S).group()
sec=re.search(r'struct mwx_rx_sec \{.*?\n\};',hal,re.S).group()
fill=hal[hal.index('int\nmt7921_mac_fill_rx('):hal.index('\nuint32_t\nmt7921_mac_tx_rate_val(')]
def extract(s,name):
 m=re.search(r'(?m)^(?:static\s+)?(?:struct ieee80211_key \*|struct mbuf \*|mbuf_t|int)\n'+name+r'\(',s);assert m,name
 a=s.index('{',m.start());pos=a+1;depth=1
 while depth:depth+=(s[pos]=='{')-(s[pos]=='}');pos+=1
 return s[m.start():pos]
restore=extract(hal,'mwx_rx_hwdec_restore')
crypto='\n'.join([extract((net/'ieee80211_crypto.c').read_text(),'ieee80211_get_rxkey'),extract((net/'ieee80211_crypto_ccmp.c').read_text(),'ieee80211_ccmp_get_pn'),extract((net/'ieee80211_input.c').read_text(),'ieee80211_input_hwdecrypt')])
stubs=r'''
#include "RxAmpduStats.hpp"
using u_int8_t=uint8_t;using u_int16_t=uint16_t;using u_int64_t=uint64_t;using u_int=unsigned;using caddr_t=char*;
#define mtod(m,t) ((t)mbuf_data(m))
#define m_freem mbuf_freem
#define M_DONTWAIT 0
#define MWX_DEV_LOG(...) ((void)0)
#define MWX_RX_DROP_LOG(...) ((void)0)
#define le32toh(v) (v)
#define MAX(a,b) ((a)>(b)?(a):(b))
#define DEVNAME(sc) "test"
#define IEEE80211_S_RUN 5
#define IEEE80211_RXI_HWDEC 1
#define IEEE80211_RXI_HWDEC_SAME_PN 4
#define IEEE80211_FC1_PROTECTED 0x40
#define IEEE80211_FC0_TYPE_MASK 0x0c
#define IEEE80211_FC0_TYPE_DATA 0x08
#define IEEE80211_FC0_TYPE_MGT 0x00
#define IEEE80211_FC0_SUBTYPE_NODATA 0x40
#define IEEE80211_F_RSNON 1
#define IEEE80211_CIPHER_USEGROUP 0
#define IEEE80211_CIPHER_TKIP 2
#define IEEE80211_CIPHER_CCMP 4
#define IEEE80211_CCMP_HDRLEN 8
#define IEEE80211_TKIP_HDRLEN 8
#define IEEE80211_WEP_EXTIV 0x20
#define IEEE80211_QOS_TID 15
#define IEEE80211_MMIE_LEN 18
#define IEEE80211_ELEMID_MMIE 76
#define IEEE80211_IS_MULTICAST(p) ((p)[0]&1)
#define LE_READ_2(p) ((uint16_t)(p)[0]|((uint16_t)(p)[1]<<8))
struct ieee80211_frame_min {uint8_t data[16];};
struct ieee80211_frame {uint8_t i_fc[2],dur[2],i_addr1[6],i_addr2[6],i_addr3[6],i_seq[2];};
struct ieee80211_key {unsigned k_cipher=0;uint64_t k_rsc[16]={},k_mgmt_rsc=0;};
struct ieee80211_node {unsigned ni_rsncipher=IEEE80211_CIPHER_CCMP;ieee80211_key ni_pairwise_key;};
struct ieee80211_rxinfo {unsigned rxi_flags=0;int rxi_rssi=0;unsigned rxi_chan=0;};
struct ieee80211com {int ic_state=IEEE80211_S_RUN;unsigned ic_flags=IEEE80211_F_RSNON;ieee80211_key ic_nw_keys[6];struct{unsigned is_rx_decryptcrc=0,is_rx_locmicfail=0,is_ccmp_replays=0,is_tkip_replays=0;}ic_stats;};
struct mwx_softc {ieee80211com sc_ic;MtkRxAmpduStats sc_rx_ampdu_stats={};unsigned sc_rx_fill_fail=0;struct{int num_streams=2;unsigned antenna_mask=3;}sc_capa;};
static void ieee80211_michael_mic_failure(ieee80211com*,int){}
static int rcpi_to_rssi(unsigned shift,uint32_t v){return int((v>>shift)&255)/2-110;}
static void mwx_rx_rate_account(uint32_t){}
static void m_adj(mbuf_t m,int n){mbuf_adj(m,n);}
static bool ieee80211_has_qos(const ieee80211_frame*w){return (w->i_fc[0]&0x8c)==0x88;}
static unsigned ieee80211_get_hdrlen(const ieee80211_frame*w){return 24+(ieee80211_has_qos(w)?2:0);}
static unsigned ieee80211_get_qos(const ieee80211_frame*w){return ((const uint8_t*)w)[24];}
static int ieee80211_tkip_get_tsc(uint64_t*,uint64_t**,mbuf_t,ieee80211_key*){assert(false);return EINVAL;}
static size_t m_leadingspace(mbuf_t m){return mbuf_leadingspace(m);}
static unsigned prepend_calls=0;static bool fail_prepend=false;
static mbuf_t m_prepend(mbuf_t m,size_t n,int){++prepend_calls;if(fail_prepend){mbuf_freem(m);return nullptr;}if(m->off>=n){m->off-=n;m->len+=n;m->pklen+=n;return m;}std::vector<uint8_t>s(m->storage.size()+n);std::copy(m->storage.begin(),m->storage.end(),s.begin()+n);m->storage.swap(s);m->len+=n;m->pklen+=n;return m;}
static mbuf_t m_pullup(mbuf_t m,size_t n){if(m->len>=n)return m;if(n>m->pklen){mbuf_freem(m);return nullptr;}mbuf_t r=packet({},m->pklen);r->len=r->pklen=m->pklen;assert(mbuf_copydata(m,0,m->pklen,mbuf_data(r))==0);mbuf_freem(m);return r;}
static uint32_t mwx_rx_hwdec_nopn_cnt=0;
'''
main=r'''
// QoS (or plain, or management) data frame as the hardware delivers it after
// decryption: header kept, Protected still set, CCMP header and MIC gone.
static std::vector<uint8_t> mkframe(bool qos=true,bool prot=true,unsigned tid=3,bool mcast=false,unsigned type=0x08,size_t payload=20){
 size_t h=qos?26:24;std::vector<uint8_t>v(h+payload);v[0]=qos?0x88:type;v[1]=(prot?0x40:0)|0x02;for(size_t i=4;i<22;++i)v[i]=0x10+i;if(mcast)v[4]=0x01;v[22]=0x40;v[23]=0x12;if(qos)v[24]=tid;for(size_t i=h;i<v.size();++i)v[i]=0xc0+(i&31);return v;}
static std::vector<uint8_t> descriptor(const std::vector<uint8_t>&f,unsigned cipher=4,unsigned kid=0,bool group1=true,uint64_t pn=0x060504030201ULL,bool cm=false,bool group4=false){
 size_t gap=24+(group4?16:0)+(group1?16:0);std::vector<uint8_t>b(gap+f.size(),0);
 put32(b,4,(group1?MT_RXD1_NORMAL_GROUP_1:0)|(group4?MT_RXD1_NORMAL_GROUP_4:0)|(cipher<<16)|(kid<<21)|(cm?MT_RXD1_NORMAL_CM:0));put32(b,12,MT_RXD3_NORMAL_U2M);
 if(group1)for(unsigned i=0;i<6;++i)b[24+(group4?16:0)+i]=pn>>(8*i);memcpy(b.data()+gap,f.data(),f.size());return b;}
struct Rx{mwx_softc sc;ieee80211_rxinfo rxi;mwx_rx_sec sec;mbuf_t m=nullptr;size_t gap=0;};
static mbuf_t receive(Rx&r,const std::vector<uint8_t>&d){r.rxi={};r.m=packet(d);assert(mt7921_mac_fill_rx(&r.sc,r.m,&r.rxi,&r.sec)==0);r.gap=r.m->off;return mwx_rx_hwdec_restore(&r.sc,r.m,&r.rxi,&r.sec);}
static std::vector<uint8_t> flat(mbuf_t m){std::vector<uint8_t>v(m->pklen);assert(mbuf_copydata(m,0,m->pklen,v.data())==0);return v;}
// What net80211 hands on: the frame as sent, header stripped and Protected clear.
static std::vector<uint8_t> plain(std::vector<uint8_t> f){f[1]&=~0x40;return f;}
// Header unchanged but Protected, then exactly one CCMP header with the packet
// number and key id, ExtIV set, then the payload unchanged.
static void expect_iv(mbuf_t m,const std::vector<uint8_t>&f,unsigned h,uint64_t pn,unsigned kid){
 auto v=flat(m);assert(v.size()==f.size()+8&&m->len>=h+8);assert(!memcmp(v.data(),f.data(),h)&&(v[1]&0x40));
 uint8_t iv[8]={uint8_t(pn),uint8_t(pn>>8),0,uint8_t(0x20|kid<<6),uint8_t(pn>>16),uint8_t(pn>>24),uint8_t(pn>>32),uint8_t(pn>>40)};
 assert(!memcmp(v.data()+h,iv,8)&&!memcmp(v.data()+h+8,f.data()+h,f.size()-h));}
int main(){
 ieee80211_node ni;ni.ni_pairwise_key.k_cipher=IEEE80211_CIPHER_CCMP;Rx r;auto f=mkframe();const uint64_t pn=0x060504030201ULL;
 // The descriptor's packet number becomes the CCMP header, written in the
 // headroom the descriptor left, and net80211 accepts the frame once.
 auto m=receive(r,descriptor(f));assert(m==r.m&&prepend_calls==0&&m->off==r.gap-8&&r.gap==40);assert(r.sec.pn_valid&&r.sec.sec_mode==MT_CIPHER_AES_CCMP&&r.sec.key_id==0);expect_iv(m,f,26,pn,0);
 assert(ieee80211_input_hwdecrypt(&r.sc.sc_ic,&ni,m,&r.rxi)==m);assert(ni.ni_pairwise_key.k_rsc[3]==pn);assert(flat(m)==plain(f));mbuf_freem(m);
 // The same frame received again is a replay and is consumed.
 m=receive(r,descriptor(f));expect_iv(m,f,26,pn,0);assert(!ieee80211_input_hwdecrypt(&r.sc.sc_ic,&ni,m,&r.rxi));assert(r.sc.sc_ic.ic_stats.is_ccmp_replays==1&&live==0);
 // A lower packet number is rejected, a higher one accepted and remembered.
 m=receive(r,descriptor(f,4,0,true,pn-1));assert(!ieee80211_input_hwdecrypt(&r.sc.sc_ic,&ni,m,&r.rxi)&&r.sc.sc_ic.ic_stats.is_ccmp_replays==2);
 m=receive(r,descriptor(f,4,0,true,pn+1));assert(ieee80211_input_hwdecrypt(&r.sc.sc_ic,&ni,m,&r.rxi)==m&&ni.ni_pairwise_key.k_rsc[3]==pn+1);mbuf_freem(m);
 // Each TID keeps its own counter; non-QoS data uses TID 0 and a 24-byte header.
 auto f5=mkframe(true,true,5);m=receive(r,descriptor(f5,4,0,true,9));expect_iv(m,f5,26,9,0);assert(ieee80211_input_hwdecrypt(&r.sc.sc_ic,&ni,m,&r.rxi)==m&&ni.ni_pairwise_key.k_rsc[5]==9&&ni.ni_pairwise_key.k_rsc[3]==pn+1);mbuf_freem(m);
 auto fn=mkframe(false);m=receive(r,descriptor(fn,4,0,true,7));expect_iv(m,fn,24,7,0);assert(ieee80211_input_hwdecrypt(&r.sc.sc_ic,&ni,m,&r.rxi)==m&&ni.ni_pairwise_key.k_rsc[0]==7&&flat(m)==plain(fn));mbuf_freem(m);
 // GROUP_4 sits before GROUP_1 and the key id reaches the header.
 m=receive(r,descriptor(f,4,3,true,pn+2,false,true));assert(r.gap==56);expect_iv(m,f,26,pn+2,3);assert(ieee80211_input_hwdecrypt(&r.sc.sc_ic,&ni,m,&r.rxi)==m&&ni.ni_pairwise_key.k_rsc[3]==pn+2);mbuf_freem(m);
 // A group-addressed frame is checked against the group key the key id names.
 r.sc.sc_ic.ic_nw_keys[1].k_cipher=IEEE80211_CIPHER_CCMP;auto fg=mkframe(true,true,0,true);m=receive(r,descriptor(fg,4,1,true,3));expect_iv(m,fg,26,3,1);
 assert(ieee80211_input_hwdecrypt(&r.sc.sc_ic,&ni,m,&r.rxi)==m&&r.sc.sc_ic.ic_nw_keys[1].k_rsc[0]==3&&ni.ni_pairwise_key.k_rsc[0]==7);mbuf_freem(m);
 m=receive(r,descriptor(fg,4,1,true,3));assert(!ieee80211_input_hwdecrypt(&r.sc.sc_ic,&ni,m,&r.rxi)&&r.sc.sc_ic.ic_stats.is_ccmp_replays==3);
 // TKIP keeps the previous behaviour: no header, Protected cleared, net80211 leaves it alone.
 ni.ni_pairwise_key.k_cipher=IEEE80211_CIPHER_TKIP;m=receive(r,descriptor(f,MT_CIPHER_TKIP));{auto v=flat(m);assert(m==r.m&&v.size()==f.size()&&!(v[1]&0x40)&&!memcmp(v.data()+2,f.data()+2,f.size()-2));}
 assert(ieee80211_input_hwdecrypt(&r.sc.sc_ic,&ni,m,&r.rxi)==m&&m->pklen==f.size()&&ni.ni_pairwise_key.k_rsc[3]==pn+2);mbuf_freem(m);ni.ni_pairwise_key.k_cipher=IEEE80211_CIPHER_CCMP;
 // A CCMP frame decrypted without a reported packet number is dropped and counted.
 m=receive(r,descriptor(f,4,0,false));assert(m==nullptr&&live==0&&mwx_rx_hwdec_nopn_cnt==1);
 // Frames the hardware did not decrypt (CM set, or no cipher) are untouched.
 m=receive(r,descriptor(f,4,0,true,pn,true));assert(m==r.m&&!(r.rxi.rxi_flags&IEEE80211_RXI_HWDEC)&&!r.sec.pn_valid&&flat(m)==f);mbuf_freem(m);
 auto fo=mkframe(true,false);m=receive(r,descriptor(fo,0,0,false));assert(m==r.m&&r.rxi.rxi_flags==0&&flat(m)==fo);mbuf_freem(m);
 // A decrypted management frame gets no header, only Protected cleared.
 auto fm=mkframe(false,true,0,false,0x00);m=receive(r,descriptor(fm));{auto v=flat(m);assert(m==r.m&&v.size()==fm.size()&&!(v[1]&0x40)&&!memcmp(v.data()+2,fm.data()+2,fm.size()-2));}mbuf_freem(m);
 // A decrypted frame shorter than its own header is dropped, not restored.
 auto fs=mkframe();fs.resize(20);m=receive(r,descriptor(fs));assert(m==nullptr&&live==0&&mwx_rx_hwdec_nopn_cnt==2);
 // Without headroom, or with a chained buffer, the header is restored through
 // prepend and pullup; an allocation failure consumes the frame.
 {mwx_rx_sec s{};s.sec_mode=MT_CIPHER_AES_CCMP;s.pn_valid=1;for(unsigned i=0;i<6;++i)s.pn[i]=pn>>(8*i);ieee80211_rxinfo ri;ri.rxi_flags=IEEE80211_RXI_HWDEC;
  ni.ni_pairwise_key.k_rsc[3]=0;mbuf_t n=packet(f);prepend_calls=0;auto*o=mwx_rx_hwdec_restore(&r.sc,n,&ri,&s);assert(o&&prepend_calls==1);expect_iv(o,f,26,pn,0);assert(ieee80211_input_hwdecrypt(&r.sc.sc_ic,&ni,o,&ri)==o&&ni.ni_pairwise_key.k_rsc[3]==pn&&flat(o)==plain(f));mbuf_freem(o);
  std::vector<uint8_t>first(14,0);first.insert(first.end(),f.begin(),f.begin()+26);n=packet(first);mbuf_adj(n,14);n->next=packet(std::vector<uint8_t>(f.begin()+26,f.end()));n->pklen=f.size();
  o=mwx_rx_hwdec_restore(&r.sc,n,&ri,&s);assert(o&&prepend_calls==2);expect_iv(o,f,26,pn,0);mbuf_freem(o);
  n=packet(f);fail_prepend=true;assert(!mwx_rx_hwdec_restore(&r.sc,n,&ri,&s)&&live==0);fail_prepend=false;}
 assert(live==0);
 puts("PASS actual fill_rx + CCMP header restore + net80211 hwdecrypt: descriptor packet number to header in place, production accept/replay/lower/higher, per-TID and group-key counters, GROUP_4 offset and key id, TKIP/no-PN/not-decrypted/management/short paths, prepend+pullup fallback and its failure");
}
'''
src=pre+'\n'+defines+'\n'+cipher+'\n'+sec+'\n'+stubs+'\n'+fill+'\n'+restore+'\n'+crypto+'\n'+main
with tempfile.TemporaryDirectory(prefix='mtk-rx-hwdec-pn-') as d:
 p=Path(d);(p/'test.cpp').write_text(src)
 subprocess.run(['clang++','-std=c++14','-I',str(root/'SimpleMtkWlan/hal_mwx'),'-fsanitize=undefined','-fno-sanitize-recover=all','-g',str(p/'test.cpp'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
