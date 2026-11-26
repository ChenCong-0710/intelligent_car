#include "initial_align.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#define RAD 0.0174532925199433f
static float scan_cos[ALIGN_POINTS], scan_sin[ALIGN_POINTS];
static float angle(unsigned i) { return (90.0f-5.0f*(float)i)*RAD; }
static uint16_t pulse(unsigned i) {
    /* Calibrated piecewise mapping: +90=2400us, 0=1460us, -90=500us.
       One straight line between endpoints would give an incorrect 1450us centre. */
    if (i <= 18U)
        return (uint16_t)((float)ALIGN_LEFT_US +
            ((float)ALIGN_CENTRE_US-ALIGN_LEFT_US)*(float)i/18.0f+0.5f);
    return (uint16_t)((float)ALIGN_CENTRE_US +
        ((float)ALIGN_RIGHT_US-ALIGN_CENTRE_US)*(float)(i-18U)/18.0f+0.5f);
}
static void fail(InitialAlign *a, AlignError e) {
    a->error=e; a->phase=ALIGN_FAILED;
}
void align_start(InitialAlign *a, uint32_t now) {
    memset(a,0,sizeof(*a)); a->phase=ALIGN_HOME; a->since=a->started=now;
    a->servo_us=ALIGN_CENTRE_US;
}
/* Paper faces and ultrasound have small local errors. Fit the dominant plane
   with bounded consensus, then refit its inliers using the original inverse-
   range least squares. Boundaries locate the box, not the inlier subwindow. */
#define FACE_BAND_MM 8.0f
#define FACE_BAND_RATIO 0.05f
#define FACE_SUPPORT_PERCENT 70U
static bool model(float x,float y,float *phi,float *rho){
    *rho=1.0f/hypotf(x,y);*phi=atan2f(y,x);
    return isfinite(*rho)&&*rho>=40&&*rho<=400&&fabsf(*phi)<=80*RAD;
}
static bool inlier(const InitialAlign *a,unsigned i,float phi,float rho,float *e){
    if(!a->range[i])return false;
    float c=cosf(phi)*scan_cos[i]+sinf(phi)*scan_sin[i];
    *e=a->range[i]*c-rho; /* Perpendicular mm, not amplified grazing range error. */
    return c>=0.35f&&fabsf(*e)<=fmaxf(FACE_BAND_MM,FACE_BAND_RATIO*rho);
}
static unsigned score(const InitialAlign *a,unsigned lo,unsigned hi,
                      float phi,float rho,float *sum){
    unsigned n=0;*sum=0;
    /* Compute trig once per hypothesis, not once per echo. */
    float nx=cosf(phi),ny=sinf(phi),band=fmaxf(FACE_BAND_MM,FACE_BAND_RATIO*rho);
    for(unsigned i=lo;i<=hi;++i){
        if(!a->range[i])continue;
        float c=nx*scan_cos[i]+ny*scan_sin[i],e=a->range[i]*c-rho;
        if(c>=0.35f&&fabsf(e)<=band){++n;*sum+=e*e;}
    }
    return n;
}
static bool patch(const InitialAlign *a,unsigned lo,unsigned hi,
                  float *phi,float *rho,float *rms,unsigned *support){
    unsigned total=0,best=0;float best_sum=1e30f,bp=0,br=0;
    for(unsigned i=lo;i<=hi;++i)if(a->range[i])++total;
    if(total<4)return false;
    for(unsigned i=lo;i+3<=hi;++i){
        if(!a->range[i])continue;
        for(unsigned j=i+3;j<=hi;++j){
            if(!a->range[j])continue;
            float det=scan_cos[i]*scan_sin[j]-scan_cos[j]*scan_sin[i];
            if(fabsf(det)<0.1f)continue;
            float u=1.0f/a->range[i],v=1.0f/a->range[j];
            float x=(u*scan_sin[j]-v*scan_sin[i])/det;
            float y=(scan_cos[i]*v-scan_cos[j]*u)/det,p,r,sum;
            if(!model(x,y,&p,&r))continue;
            unsigned n=score(a,lo,hi,p,r,&sum);
            if(n>best||(n==best&&sum<best_sum)){best=n;best_sum=sum;bp=p;br=r;}
        }
    }
    unsigned required=(total*FACE_SUPPORT_PERCENT+99U)/100U;
    if(required<4)required=4;
    if(best<required)return false;
    for(unsigned pass=0;pass<2;++pass){
        float cc=0,ss=0,cs=0,cy=0,sy=0;
        for(unsigned i=lo;i<=hi;++i){
            float e;if(!inlier(a,i,bp,br,&e))continue;
            float c=scan_cos[i],s=scan_sin[i],y=1.0f/a->range[i];
            cc+=c*c;ss+=s*s;cs+=c*s;cy+=c*y;sy+=s*y;
        }
        float det=cc*ss-cs*cs;
        if(det<0.0001f)return false;
        float p,r,sum;
        if(!model((cy*ss-sy*cs)/det,(sy*cc-cy*cs)/det,&p,&r))return false;
        unsigned n=score(a,lo,hi,p,r,&sum);
        /* Keep a valid consensus if refitting would pull it toward outliers. */
        if(n<required)break;
        bp=p;br=r;best=n;best_sum=sum;
    }
    /* A tiny clean cluster cannot stand in for the whole bounded face. */
    bool first=false,last=false;
    for(unsigned i=lo;i<=hi;++i){float e;if(inlier(a,i,bp,br,&e)){
        if(i<=lo+(hi-lo)/3)first=true;
        if(i>=hi-(hi-lo)/3)last=true;
    }}
    if(!first||!last)return false;
    *phi=bp;*rho=br;*rms=sqrtf(best_sum/best);*support=best;
    return true;
}
/* If the paper face misses consensus quality, two real boundaries still
   permit the original inverse-range least-squares normal calculation.
   Only missing/degenerate coordinates prevent a mathematical result. */
static bool boundary_patch(const InitialAlign *a,unsigned lo,unsigned hi,
                          float *phi,float *rho,float *rms,unsigned *support){
 float cc=0,ss=0,cs=0,cy=0,sy=0;unsigned n=0;
 for(unsigned i=lo;i<=hi;++i){if(!a->range[i])continue;
  float c=scan_cos[i],v=scan_sin[i],y=1.f/a->range[i];
  cc+=c*c;ss+=v*v;cs+=c*v;cy+=c*y;sy+=v*y;++n;
 }
 float det=cc*ss-cs*cs;if(n<2||fabsf(det)<1e-8f)return false;
 float x=(cy*ss-sy*cs)/det,y=(sy*cc-cy*cs)/det,length=hypotf(x,y);
 if(!isfinite(length)||length<=0)return false;
 *rho=1.f/length;*phi=atan2f(y,x);float sum=0;
 for(unsigned i=lo;i<=hi;++i)if(a->range[i]){
  float e=a->range[i]*(x*scan_cos[i]+y*scan_sin[i])/length-*rho;sum+=e*e;
 }
 *rms=sqrtf(sum/n);*support=n;return isfinite(*rho)&&isfinite(*phi);
}
/* Confirm a jump toward a farther background on two outside samples.
   One isolated high echo inside a wrinkled face is not a box boundary. */
static bool boundary(uint16_t inside,uint16_t outside,uint16_t next){
 if(!inside)return false;
 if(!outside)return !next&&inside<350U;
 float band=fmaxf(30.f,.25f*inside);
 if((float)outside-inside<=band)return false;
 return next?(float)next-inside>band*.5f:inside<350U;
}
static bool left_edge(const InitialAlign *a,unsigned i){
 return i>=2&&boundary(a->range[i],a->range[i-1],a->range[i-2]);
}
static bool right_edge(const InitialAlign *a,unsigned i){
 return i+2<ALIGN_POINTS&&boundary(a->range[i],a->range[i+1],a->range[i+2]);
}
bool align_fit(InitialAlign *a){
 for(unsigned i=0;i<ALIGN_POINTS;++i){scan_cos[i]=cosf(angle(i));scan_sin[i]=sinf(angle(i));}
 unsigned best_n=0,blo=0,bhi=0;float best_rho=1e9f,bphi=0,brms=0,bedge=0;
 for(unsigned lo=0;lo+1<ALIGN_POINTS;++lo){
  if(!left_edge(a,lo))continue;
  for(unsigned hi=lo;hi<ALIGN_POINTS;++hi){
   if(!a->range[hi]){if(hi+1>=ALIGN_POINTS||!a->range[hi+1])break;else continue;}
   /* Mild wrinkles / one noisy echo must not split the face. Large jumps
      still separate the foreground box from another surface. */
   if(hi>lo&&a->range[hi-1]&&fabsf((float)a->range[hi]-a->range[hi-1])>
       fmaxf(60.f,.5f*fminf(a->range[hi],a->range[hi-1])))break;
   if(hi<=lo||!right_edge(a,hi))continue;
   float phi,rho,rms;unsigned n;bool relaxed=false;
   if(!patch(a,lo,hi,&phi,&rho,&rms,&n)){
    if(!boundary_patch(a,lo,hi,&phi,&rho,&rms,&n))continue;
    relaxed=true;
   }
   float xl=a->range[lo]*scan_cos[lo],yl=a->range[lo]*scan_sin[lo];
   float xr=a->range[hi]*scan_cos[hi],yr=a->range[hi]*scan_sin[hi];
   float edge_phi=atan2f(-(xl-xr),yl-yr);
   if(fabsf(edge_phi-phi)>20.f*RAD)relaxed=true;
   if(!best_n||rho<best_rho-5||(fabsf(rho-best_rho)<=5&&n>best_n)){
    best_n=n;best_rho=rho;blo=lo;bhi=hi;bphi=phi;brms=rms;bedge=edge_phi;a->line_warning=relaxed;
   }
  }
 }
 if(!best_n)return false;
 a->normal_deg=bphi/RAD;a->rho_mm=best_rho;a->rms_mm=brms;
 a->side_mm=(uint16_t)lroundf(best_rho); /* Fallback reference; no post-turn echo gate. */
 a->fit_first=(uint8_t)blo;a->fit_last=(uint8_t)bhi;a->fit_support=(uint8_t)best_n;
 a->edge_left_seen=a->edge_right_seen=true;
 a->fit_total=0;for(unsigned i=blo;i<=bhi;++i)if(a->range[i])++a->fit_total;
 a->edge_left_mm=a->range[blo];a->edge_right_mm=a->range[bhi];a->edge_normal_deg=bedge/RAD;
 a->centre_y_mm=.5f*(a->range[blo]*scan_sin[blo]+a->range[bhi]*scan_sin[bhi]);
 a->side_left=fabsf(a->centre_y_mm)<5.f?a->normal_deg>=0:a->centre_y_mm>0;
 a->target_deg=a->normal_deg+(a->side_left?-90.f:90.f);
 a->moving_wheel=1U;a->motor_direction=a->target_deg>=0?1:-1;
 return true;
}
/* Side probe remains at calibrated +/-90 throughout the traverse.
   x uses physical A-both-edge scale (43mm diameter, 420 edges/revolution).
   One scan only: start moving unconditionally, accept a start inside the face. */
#define MM_PER_EDGE (43.0f*3.14159265359f/420.0f)
bool align_fit_line(InitialAlign *a){
 /* Exclude endpoints and the 10mm band next to each physical edge. Select
    a majority line, then refit its inliers; never fit background or edges. */
 unsigned lo=1,hi=a->line_n>1?a->line_n-2:0;
 if(!a->entry_seen||!a->exit_seen||a->line_n<2)return false;
 if(a->line_n<7){lo=0;hi=a->line_n-1;a->line_warning=true;}
 while(lo<hi&&a->line_x[lo]<(a->edge_entry*MM_PER_EDGE+10))++lo;
 while(hi>lo&&a->line_x[hi]>(a->edge_exit*MM_PER_EDGE-10))--hi;
 if(hi<lo+1){lo=0;hi=a->line_n-1;a->line_warning=true;}
 if(a->line_x[hi]<=a->line_x[lo])return false;
 unsigned best=0;float bm=0,bb=0,bs=1e30f;
 for(unsigned i=lo;i<hi;++i)for(unsigned j=i+1;j<=hi;++j){
  float dx=a->line_x[j]-a->line_x[i];if(dx<20)continue;
  float m=(a->line_y[j]-a->line_y[i])/dx,b=a->line_y[i]-m*a->line_x[i];
  unsigned n=0;float sum=0;
  for(unsigned k=lo;k<=hi;++k){float e=a->line_y[k]-m*a->line_x[k]-b;
   if(fabsf(e)<=8){++n;sum+=e*e;}}
  if(n>best||(n==best&&sum<bs)){best=n;bm=m;bb=b;bs=sum;}
 }
 unsigned required=((hi-lo+1)*70+99)/100;if(required<5)required=5;
 if(best<required){a->line_warning=true;bm=0;bb=0;}
 bool use_all=best<required;
 float sx=0,sy=0;unsigned n=0;
 for(unsigned k=lo;k<=hi;++k)if(use_all||fabsf(a->line_y[k]-bm*a->line_x[k]-bb)<=8){sx+=a->line_x[k];sy+=a->line_y[k];++n;}
 float mx=sx/n,my=sy/n,sxx=0,sxy=0;
 for(unsigned k=lo;k<=hi;++k)if(use_all||fabsf(a->line_y[k]-bm*a->line_x[k]-bb)<=8){
  float dx=a->line_x[k]-mx;sxx+=dx*dx;sxy+=dx*(a->line_y[k]-my);}
 if(sxx<=0)return false;
 float m=sxy/sxx,b=my-m*mx,sum=0;unsigned used=0;
 for(unsigned k=lo;k<=hi;++k)if(use_all||fabsf(a->line_y[k]-bm*a->line_x[k]-bb)<=8){
  float e=a->line_y[k]-m*a->line_x[k]-b;sum+=e*e;++used;}
 a->line_rms=sqrtf(sum/used);a->line_slope=m;a->line_support=(uint8_t)used;
 /* Same normal-to-side-probe formula as coarse alignment. For left
    probe face is y=b+m*x; for right probe y=-b-m*x. */
 a->fine_normal_deg=atan2f(a->side_left?1.f:-1.f,-m)/RAD;
 a->fine_deg=a->fine_normal_deg+(a->side_left?-90.f:90.f);
 /* A large correction or poor residual means coarse pose/data is unreliable.
    Do not clamp a large requested turn and pretend the body is parallel. */
 if(fabsf(a->fine_deg)>20||a->line_rms>8)a->line_warning=true;
 return isfinite(a->fine_deg);
}
void align_cancel(InitialAlign *a){a->phase=ALIGN_CANCELLED;}
static void plan_approach(InitialAlign *a,uint16_t mm){
 a->approach_range_mm=mm;
 if(mm<=ALIGN_APPROACH_TARGET_MM){a->approach_hard_stop=true;return;}
 if(mm<=ALIGN_APPROACH_TRIGGER_MM&&!a->approach_plan_ready){
  a->approach_plan_mm=mm;
  a->approach_remaining_counts=(int32_t)lroundf(((float)mm-ALIGN_APPROACH_TARGET_MM)/MM_PER_EDGE);
  a->approach_plan_ready=true;
 }
}
void align_step(InitialAlign *a,uint32_t now,uint32_t seq,uint32_t trigger_ms,
 uint32_t echo_ms,bool valid,uint16_t mm,const AlignMotionFeedback *motion){
 if(a->phase==ALIGN_DONE||a->phase==ALIGN_FAILED||a->phase==ALIGN_CANCELLED)return;
 if(motion&&motion->fault){a->motor_error=motion->error;fail(a,ALIGN_MOTOR);return;}
 bool fresh=seq!=a->last_seq;if(fresh)a->last_seq=seq;
 uint32_t elapsed=now-a->since;
 bool range_ok=fresh&&valid&&mm>=40&&mm<=400&&now-echo_ms<=100U;
 if(a->phase==ALIGN_HOME){
  a->servo_us=ALIGN_CENTRE_US;
  if(elapsed>=ALIGN_SIDE_SETTLE_MS){a->phase=ALIGN_SEEK;a->since=now;}
 }else if(a->phase==ALIGN_SEEK){
  a->servo_us=ALIGN_CENTRE_US;
  if(range_ok&&mm<=ALIGN_START_RANGE_MM){
   a->approach_range_mm=mm;
   a->approach_requested=a->approach_plan_ready=a->approach_plan_applied=false;
   a->approach_hard_stop=a->approach_hard_stop_applied=a->approach_echo_lost=false;
   a->approach_invalid=0;
   if(mm<=ALIGN_APPROACH_TARGET_MM){a->phase=ALIGN_PRE_SCAN_WAIT;a->since=now;
   a->approach_stop_mm=0;a->approach_stop_sum=0;a->approach_stop_hits=0;}
   else {plan_approach(a,mm);a->phase=ALIGN_APPROACH;a->since=now;}
  }
 }else if(a->phase==ALIGN_APPROACH||a->phase==ALIGN_APPROACH_STOP){
  a->servo_us=ALIGN_CENTRE_US;
  if(a->approach_requested&&fresh&&now-echo_ms<=100U){
   if(range_ok){a->approach_invalid=0;plan_approach(a,mm);}
   else if(++a->approach_invalid>=2){
    /* Actual loss of range: brake and reacquire at centre, do not scan blind. */
    a->approach_echo_lost=true;a->approach_hard_stop=true;
   }
  }
  if(a->approach_plan_applied||a->approach_hard_stop_applied)a->phase=ALIGN_APPROACH_STOP;
  if(a->phase==ALIGN_APPROACH_STOP&&motion&&motion->done){
   a->phase=ALIGN_PRE_SCAN_WAIT;a->since=now;
   a->approach_stop_mm=0;a->approach_stop_sum=0;a->approach_stop_hits=0;
  }
 }else if(a->phase==ALIGN_PRE_SCAN_WAIT){
  a->servo_us=ALIGN_CENTRE_US;
  /* Latch stationary front distance before turning the servo away. */
  if(range_ok&&(uint32_t)(trigger_ms-a->since)<=elapsed&&a->approach_stop_hits<250){
   a->approach_stop_sum+=mm;++a->approach_stop_hits;
   a->approach_stop_mm=(uint16_t)(a->approach_stop_sum/a->approach_stop_hits);
  }
  if(elapsed>=ALIGN_SEGMENT_WAIT_MS){
   if(a->approach_echo_lost){a->phase=ALIGN_SEEK;a->since=now;}
   else {a->phase=ALIGN_SCAN;a->index=0;a->sum=0;a->hits=0;
    memset(a->range,0,sizeof(a->range));a->servo_us=ALIGN_LEFT_US;a->since=now;}
  }
 }else if(a->phase==ALIGN_SCAN){
  uint32_t settle=a->index==0?1200U:120U;
  if(range_ok&&elapsed>=settle&&(uint32_t)(trigger_ms-a->since)>=settle&&
     (uint32_t)(trigger_ms-a->since)<=elapsed){
   a->sum+=mm;if(a->hits<250)++a->hits;
  }
  if(elapsed>=settle+180U){
   a->range[a->index]=a->hits>=2?(uint16_t)(a->sum/a->hits):0;a->sum=0;a->hits=0;
   if(++a->index==ALIGN_POINTS)a->phase=ALIGN_FIT;
   else {a->servo_us=pulse(a->index);a->since=now;}
  }
 }else if(a->phase==ALIGN_FIT){
  if(!align_fit(a)){fail(a,ALIGN_NO_FACE);return;}
  a->servo_us=a->side_left?ALIGN_LEFT_US:ALIGN_RIGHT_US;
  a->since=now;a->phase=ALIGN_SIDE_SETTLE;a->turn_requested=false;
  a->left_turn_requested=false;a->wheel_degrees[0]=a->wheel_degrees[1]=0;
  a->first_turn_degrees[0]=a->first_turn_degrees[1]=0;
 }else if(a->phase==ALIGN_SIDE_SETTLE){
  if(elapsed>=ALIGN_SIDE_SETTLE_MS){
   a->since=a->turn_since=now;a->phase=fabsf(a->target_deg)<=2.f?ALIGN_BRAKE:ALIGN_TURN;
  }
 }else if(a->phase==ALIGN_TURN){
  if(a->turn_requested&&motion){
   for(unsigned w=0;w<2;++w)a->wheel_degrees[w]=motion->wheel_degrees[w];
   a->actual_deg=a->wheel_degrees[1]-a->wheel_degrees[0];
   if(motion->done){
    for(unsigned w=0;w<2;++w)a->first_turn_degrees[w]=a->wheel_degrees[w];
    a->phase=ALIGN_TURN_HALF_WAIT;a->since=now;
   }
  }
 }else if(a->phase==ALIGN_TURN_HALF_WAIT){
  if(elapsed>=ALIGN_SEGMENT_WAIT_MS){
   a->phase=ALIGN_TURN_LEFT;a->since=now;a->moving_wheel=0;
   a->motor_direction=a->target_deg>0?-1:1;
  }
 }else if(a->phase==ALIGN_TURN_LEFT){
  if(a->left_turn_requested&&motion){
   for(unsigned w=0;w<2;++w)a->wheel_degrees[w]=a->first_turn_degrees[w]+motion->wheel_degrees[w];
   a->actual_deg=a->wheel_degrees[1]-a->wheel_degrees[0];
   if(motion->done){a->phase=ALIGN_BRAKE;a->since=now;}
  }
 }else if(a->phase==ALIGN_BRAKE){
  if(elapsed>=ALIGN_SEGMENT_WAIT_MS)a->phase=ALIGN_DONE;
 }
}
const char *align_phase_name(AlignPhase p) {
    static const char *const n[]={"IDLE","SCAN","FIT","SIDE_SETTLE","TURN","BRAKE","LOCK","RETREAT","WAIT_FORWARD","FORWARD","SCAN_BRAKE","FINE_TURN","WAIT_FINAL","FINAL_BACK","DONE","FAILED","CANCELLED","HOME","SEEK","APPROACH","APPROACH_STOP","PRE_SCAN_WAIT","TURN_HALF_WAIT","TURN_LEFT"};
    return (unsigned)p<sizeof(n)/sizeof(n[0])?n[p]:"UNKNOWN";
}
const char *align_error_name(AlignError e) {
    static const char *const n[]={"OK","NO_FACE","TURN_TIMEOUT","ENCODER","STALL","SIDE_NO_ECHO","HARDWARE","MOTOR","RETREAT_TIMEOUT","NO_BOUNDARIES","BAD_LINE","STAGE_TIMEOUT"};
    return (unsigned)e<sizeof(n)/sizeof(n[0])?n[e]:"UNKNOWN";
}
