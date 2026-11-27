"""Continuous read-only CAL capture. Runs until interrupted; sends no commands."""
import argparse,json,time,sys
from pathlib import Path

def parse(s):
 if not s.startswith(("M,","C,","S,","L,","U,","F,")):return None
 out={"type":s[0]}
 for part in s.strip().split(',')[1:]:
  if '=' not in part:return None
  k,v=part.split('=',1)
  try:out[k]=int(v)
  except ValueError:out[k]=v
 return out if out.get('fw') in ('BOX_ALIGN_PD_V1','BOX_ALIGN_PD_V2','BOX_ALIGN_PD_V3','BOX_ALIGN_PD_V4','BOX_ALIGN_PD_V5','BOX_ALIGN_PD_V6','BOX_ALIGN_PD_V7','BOX_ALIGN_PD_V8','BOX_ALIGN_PD_V9','BOX_ALIGN_PD_V10','BOX_APPROACH_V1','BOX_APPROACH_V2','BOX_SCAN4_V3','BOX_CENTER_V4','BOX_FINE20_V5','BOX_REV25_V6','BOX_DIAG_V7','BOX_TRANSIT30_V8','BOX_SINGLEPASS_V9','BOX_EDGES3_V10','BOX_KP04_V11','BOX_KP02_V12','BOX_PD5_V13','BOX_ASSIST_V14','BOX_PDASSIST3_V15','BOX_OLDLCD_V16','BOX_OLDLCD_V17','BOX_OLDLCD_V18','BOX_FOUR_V19','BOX_FOUR450_V20','BOX_EXIT10_V21') else None

def main():
 import serial
 if hasattr(sys.stdout,'reconfigure'):sys.stdout.reconfigure(encoding='utf-8')
 a=argparse.ArgumentParser();a.add_argument('--port',default='COM4');a.add_argument('--output');opt=a.parse_args()
 path=Path(opt.output or Path(__file__).resolve().parents[1]/'work'/time.strftime('box_stage1_%Y%m%d_%H%M%S.log'))
 path.parent.mkdir(parents=True,exist_ok=True)
 conn=serial.Serial(port=None,baudrate=115200,timeout=.2);conn.port=opt.port;conn.dtr=False;conn.rts=False
 pending=bytearray();confirmed=False;seen={};previous=None;last_print=0;last_rx=time.monotonic();warned=False
 try:
  conn.open();print(f'PORT_OPEN {opt.port}; awaiting BOX_EXIT10_V21 (accepting previous versions for identification); log={path}',flush=True)
  with path.open('x',encoding='utf-8') as log,path.with_suffix('.jsonl').open('x',encoding='utf-8') as js,path.with_suffix('.bin').open('xb') as raw:
   while True:
    data=conn.read(1024)
    if not data:
     if time.monotonic()-last_rx>3 and not warned:print('NO_VALID_DATA >3s; do not start / check power and firmware',flush=True);warned=True
     continue
    raw.write(data);raw.flush();pending.extend(data)
    if len(pending)>32768:pending.clear();print('RX_BUFFER_RESET',flush=True)
    while b'\n' in pending:
     line,_,tail=pending.partition(b'\n');pending[:]=tail
     s=line.decode('utf-8',errors='replace').rstrip('\r');log.write(s+'\n');log.flush()
     f=parse(s)
     if not f:
      if s.startswith(('M,','PROBE,')):print('UNEXPECTED_FIRMWARE '+s[:90],flush=True)
      continue
     js.write(json.dumps(f)+'\n');js.flush();last_rx=time.monotonic();warned=False
     if confirmed!=f['fw']:print('CONFIRMED '+f['fw'],flush=True);confirmed=f['fw']
     if f['type']=='U':
      trig=f.get('trig');prior=getattr(main,'us_trigger',None)
      delta=trig-prior if trig is not None and prior is not None else None
      main.us_trigger=trig
      state=(f.get('why'),f.get('wait'),f.get('hits'),f.get('boostL'),f.get('boostR'))
      if state!=getattr(main,'us_state',None) or (delta is not None and delta>65 and f.get('wait')!='SCAN'):
       print('SONAR '+json.dumps(f),flush=True)
      main.us_state=state
     elif f['type']=='F':
      print('FOUR_FACES '+json.dumps(f),flush=True)
     elif f['type']=='L':
      print('FACE_SAMPLE '+json.dumps(f),flush=True)
     elif f['type']=='S':
      print('SCAN_AVERAGES '+json.dumps(f),flush=True)
     elif f['type']=='C':
      key=(f.get('id'),f.get('group'),f.get('step'),f.get('mean10'))
      if key not in seen:print('RESULT '+json.dumps(f),flush=True);seen[key]=True
     else:
      seq=f.get('seq')
      if previous is not None and seq is not None and seq!=previous+1:
       print(f'SEQ_GAP previous={previous} current={seq}',flush=True)
       if seq<previous:seen.clear();print('FIRMWARE_RESET',flush=True)
      previous=seq
      state=(f.get('phase'),f.get('action'),f.get('step'),f.get('error'))
      if state!=getattr(main,'state',None) or last_rx-last_print>=1:
       print('STATUS '+json.dumps(f),flush=True);last_print=last_rx;main.state=state
 except KeyboardInterrupt:print('CAPTURE_STOPPED',flush=True)
 finally:
  if conn.is_open:conn.close()
if __name__=='__main__':main()
