#!/usr/bin/env python3
"""Actual VHT element parsers against a protected page, and the length guards
of both callers (probe response and association response) in the source."""
from pathlib import Path
import os,re,subprocess,tempfile
root=Path(__file__).resolve().parents[2]; net=root/'mtk80211/openbsd/net80211'
h=(net/'ieee80211.h').read_text(); var=(net/'ieee80211_var.h').read_text(); nodeh=(net/'ieee80211_node.h').read_text()
node=(net/'ieee80211_node.c').read_text(); inp=(net/'ieee80211_input.c').read_text()
def fn(s,name):
 m=re.search(r'(?m)^'+name+r'\([^;]*?\)\s*\n\{',s);assert m,name
 return 'void\n'+s[m.start():s.index('\n}',m.end())+2]+'\n'
def struct(name):
 m=re.search(r'(?ms)^struct '+name+r' \{.*?^\} __packed;',h);assert m,name
 return m.group()+'\n'
functions=fn(node,'ieee80211_setup_vhtopmode')+fn(node,'ieee80211_setup_vhtcaps')
# Every IEEE80211_* macro the parsers use, the _S shift twins the MS/SM
# macros paste, and whatever those definitions use in turn.
headers=h+'\n'+var+'\n'+nodeh
def define(name):
 m=re.search(r'(?m)^#define\s+'+name+r'\b((?:[^\n]*\\\n)*[^\n]*)',headers);return m.group() if m else None
defs={};todo=set(re.findall(r'\bIEEE80211_\w+',functions))
while todo:
 t=todo.pop()
 for name in (t,t+'_S'):
  if name in defs:continue
  d=define(name)
  if d is None:continue
  defs[name]=d;todo|=set(re.findall(r'\bIEEE80211_\w+',d))-set(defs)
assert 'IEEE80211_VHTCAP_MAX_MPDU_MASK_S' in defs and 'IEEE80211_VHTCAP_BEAMFORMEE_STS_SHIFT' in defs
defines='\n'.join(defs[k] for k in sorted(defs))
# The two callers guard the element length the same way before parsing.
def body(name):
 a=inp.index('void\n'+name+'(');return inp[a:inp.index('\n}\n',a)]
guard=r'if \(vhtcap != NULL && vhtcap\[1\] < sizeof\(struct ieee80211_ie_vhtcap\) - 2\) \{.*?vhtcap = NULL;\n    \}\n    if \(vhtopmode != NULL && vhtopmode\[1\] < sizeof\(struct ieee80211_ie_vht_operation\) - 2\) \{.*?vhtopmode = NULL;\n    \}'
guards=[]
for caller in ['ieee80211_recv_probe_resp','ieee80211_recv_assoc_resp']:
 b=body(caller);m=re.search(guard,b,re.S);assert m,caller
 use=b.index('ieee80211_setup_vhtcaps(ic, ni, vhtcap);');assert m.end()<use and b.index('ieee80211_setup_vhtopmode(ni, vhtopmode);')>use,caller
 assert b.count('ieee80211_setup_vhtcaps(')==1 and b.count('ieee80211_setup_vhtopmode(')==1,caller
 guards.append(m.group())
assert guards[0]==guards[1]
pre=r'''
#include <cassert>
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>
#define __packed __attribute__((packed))
#define DPRINTF(x) do {} while (0)
#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))
#define le16toh(x) uint16_t(x)
#define htole16(x) uint16_t(x)
#define cpu_to_le16(x) uint16_t(x)
static uint32_t le32dec(const void *p){const uint8_t *b=(const uint8_t*)p;return uint32_t(b[0])|uint32_t(b[1])<<8|uint32_t(b[2])<<16|uint32_t(b[3])<<24;}
static uint16_t le16dec(const void *p){const uint8_t *b=(const uint8_t*)p;return uint16_t(b[0]|b[1]<<8);}
'''
stubs=r'''
struct ieee80211_node {uint32_t ni_vhtcaps;uint16_t ni_vht_basicmcs;ieee80211_vht_mcs_info ni_vht_mcsinfo;uint8_t ni_vht_chan1,ni_vht_chan2,ni_vht_chanwidth;uint32_t ni_flags;};
struct ieee80211com {uint32_t ic_caps;uint16_t ic_htcaps;uint32_t ic_vhtcaps;uint16_t ic_vht_tx_mcs_map,ic_vht_rx_mcs_map;};
'''
guardfn='static void vht_guards(const uint8_t *&vhtcap,const uint8_t *&vhtopmode){\n'+guards[0]+'\n}\n'
post=r'''
struct Guard{uint8_t *base;size_t page;
 Guard(){page=size_t(sysconf(_SC_PAGESIZE));base=static_cast<uint8_t*>(mmap(nullptr,page*2,PROT_READ|PROT_WRITE,MAP_ANON|MAP_PRIVATE,-1,0));assert(base!=MAP_FAILED);assert(mprotect(base+page,page,PROT_NONE)==0);}
 ~Guard(){assert(munmap(base,page*2)==0);}
 uint8_t *place(const std::vector<uint8_t>&v){auto*p=base+page-v.size();memcpy(p,v.data(),v.size());return p;}};
int main(){const uint16_t little=1;assert(*reinterpret_cast<const uint8_t*>(&little)==1);Guard g;unsigned cases=0;
 static_assert(sizeof(ieee80211_ie_vhtcap)==14&&sizeof(ieee80211_ie_vht_operation)==7,"VHT element wire sizes");
 // Elements of exactly the guarded minimum length end at the protected page:
 // the parsers read nothing past them.
 for(unsigned fill=0;fill<256;fill+=51){
  std::vector<uint8_t> cap(14,uint8_t(fill));cap[0]=191;cap[1]=12;cap[8]=0x34;cap[9]=0x12;cap[12]=0x78;cap[13]=0x56;
  ieee80211com ic{};ic.ic_caps=IEEE80211_C_SUPPORTS_VHT_EXT_NSS_BW;ic.ic_htcaps=IEEE80211_HTCAP_TXSTBC|IEEE80211_HTCAP_RXSTBC_MASK;ic.ic_vhtcaps=0xffffffff;ic.ic_vht_tx_mcs_map=ic.ic_vht_rx_mcs_map=0xffaa;
  ieee80211_node n;memset(&n,0,sizeof n);ieee80211_setup_vhtcaps(&ic,&n,g.place(cap));
  assert((n.ni_flags&IEEE80211_NODE_VHTCAP)&&n.ni_vht_mcsinfo.rx_highest==0x1234&&n.ni_vht_mcsinfo.tx_highest==0x5678);
  assert(((n.ni_vhtcaps^le32dec(cap.data()+2))&IEEE80211_VHTCAP_MAX_MPDU_MASK)==0);++cases;
  std::vector<uint8_t> op(7,uint8_t(fill));op[0]=192;op[1]=5;op[2]=1;op[3]=42;op[4]=0;op[5]=0xfa;op[6]=0xff;
  memset(&n,0,sizeof n);ieee80211_setup_vhtopmode(&n,g.place(op));
  assert(n.ni_vht_chanwidth==1&&n.ni_vht_chan1==42&&n.ni_vht_chan2==0&&n.ni_vht_basicmcs==0xfffa);++cases;
 }
 // The callers' guards drop a shorter element and keep one of the minimum length or longer.
 for(unsigned len=0;len<16;++len){
  uint8_t cap[16]={191,uint8_t(len)},op[16]={192,uint8_t(len)};const uint8_t *vhtcap=cap,*vhtopmode=op;vht_guards(vhtcap,vhtopmode);
  assert((vhtcap!=nullptr)==(len>=12)&&(vhtopmode!=nullptr)==(len>=5));++cases;
 }
 {const uint8_t *vhtcap=nullptr,*vhtopmode=nullptr;vht_guards(vhtcap,vhtopmode);assert(!vhtcap&&!vhtopmode);++cases;}
 printf("PASS actual VHT capability/operation parsers: %u protected-page and guard cases; both probe and association response parsers check the element length first\n",cases);}
'''
source=pre+defines+'\n'+''.join(struct(n) for n in ['ieee80211_vht_mcs_info','ieee80211_ie_vhtcap','ieee80211_ie_vht_operation'])+stubs+functions+guardfn+post
with tempfile.TemporaryDirectory(prefix='mtk-vht-bounds-') as td:
 p=Path(td);cpp=p/'test.cpp';cpp.write_text(source)
 subprocess.run(['clang++','-std=gnu++14','-fsanitize=undefined','-fno-sanitize-recover=all','-g',str(cpp),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True,env={**os.environ,'UBSAN_OPTIONS':'halt_on_error=1:print_stacktrace=1'})
