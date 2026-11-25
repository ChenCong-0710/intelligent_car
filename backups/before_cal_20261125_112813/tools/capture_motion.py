"""Read-only PD65 telemetry. No commands are sent to the car."""
import argparse,json,time,sys
from pathlib import Path

def parse(s):
 if not s.startswith('M,'):return None
 out={}
 for field in s.strip().split(',')[1:]:
  if '=' not in field:return None
  k,v=field.split('=',1)
  try:out[k]=int(v)
  except ValueError:out[k]=v
 return out if str(out.get('fw','')).startswith('PD65_V') and all(k in out for k in ('ms','phase','action','error')) else None

def main():
 if hasattr(sys.stdout,'reconfigure'):sys.stdout.reconfigure(encoding='utf-8')
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--port',default='COM4');p.add_argument('--seconds',type=float,default=90);p.add_argument('--output',default='work/pd65_run.log');p.add_argument('--selftest',action='store_true');a=p.parse_args()
 if a.selftest:
  f=parse('M,fw=PD65_V1,ms=1,phase=RUN,action=LEFT_30,error=OK,refL10=-650,countL=-215')
  assert f and f['refL10']==-650 and f['countL']==-215 and parse('garbage') is None
  print('PASS: PD65 telemetry parser');return 0
 import serial
 path=Path(a.output);path.parent.mkdir(parents=True,exist_ok=True)
 conn=serial.Serial();conn.port=a.port;conn.baudrate=115200;conn.timeout=.2;conn.dtr=False;conn.rts=False
 count=0;pending=bytearray();old=None
 try:
  conn.open();print('LISTENING '+a.port+'; waiting for PD65 telemetry. Do not start until telemetry is confirmed.',flush=True)
  started=time.monotonic()
  with path.open('w',encoding='utf-8') as text,path.with_suffix('.jsonl').open('w',encoding='utf-8') as js,path.with_suffix('.bin').open('wb') as raw:
   while a.seconds<=0 or time.monotonic()-started<a.seconds:
    b=conn.read(1024)
    if not b:continue
    raw.write(b);raw.flush();pending.extend(b)
    if len(pending)>16384:pending.clear()
    while b'\n' in pending:
     line,_,tail=pending.partition(b'\n');pending[:]=tail;s=line.decode('utf-8',errors='replace').rstrip('\r');text.write(s+'\n');text.flush()
     f=parse(s)
     if not f:continue
     count+=1;js.write(json.dumps(f)+'\n');js.flush()
     state=tuple(f.get(k) for k in ('phase','action','cycle','error','trim','mode'))
     if state!=old:
      print(f"{f['ms']}ms {f.get('mode')} {f['phase']} {f['action']} cycle={f.get('cycle')} error={f['error']} rpm={f.get('rpmL10',0)/10:.1f},{f.get('rpmR10',0)/10:.1f} counts={f.get('countL')},{f.get('countR')} diff={f.get('diff')} trim={f.get('trim')} IRQ={f.get('irqUs')}us",flush=True);old=state
  print(f'FINISHED valid_frames={count} log={path}',flush=True)
  return 0 if count else 2
 finally:
  if conn.is_open:conn.close()
if __name__=='__main__':
 try:raise SystemExit(main())
 except KeyboardInterrupt:print('Capture stopped.')
