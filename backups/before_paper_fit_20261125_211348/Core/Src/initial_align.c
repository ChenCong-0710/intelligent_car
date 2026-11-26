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
    memset(a,0,sizeof(*a)); a->phase=ALIGN_SCAN; a->since=a->started=now;
    a->servo_us=ALIGN_LEFT_US;
}
/* Fit inverse range over a candidate continuous patch. Reject ill-conditioned,
   grazing and mixed-plane patches using individual range residuals. */
static bool patch(const InitialAlign *a, unsigned lo, unsigned hi,
                  float *phi, float *rho, float *rms) {
    float cc=0,ss=0,cs=0,cy=0,sy=0;unsigned support=0;
    for (unsigned i=lo;i<=hi;++i) {
        if (!a->range[i]) continue;
        ++support;
        float c=scan_cos[i],s=scan_sin[i],y=1.0f/a->range[i];
        cc+=c*c;ss+=s*s;cs+=c*s;cy+=c*y;sy+=s*y;
    }
    if(support<4)return false;
    float det=cc*ss-cs*cs;
    if (det<0.0001f) return false;
    float x=(cy*ss-sy*cs)/det,y=(sy*cc-cy*cs)/det;
    *rho=1.0f/hypotf(x,y); *phi=atan2f(y,x);
    if (!isfinite(*rho) || *rho<40 || *rho>400 || fabsf(*phi)>80*RAD) return false;
    float sum=0,band=fmaxf(3.0f,0.025f*(*rho));
    for (unsigned i=lo;i<=hi;++i) {
        if(!a->range[i])continue;
        float c=(*rho)*(x*scan_cos[i]+y*scan_sin[i]);
        if (c<0.35f) return false;
        float e=(float)a->range[i]-*rho/c;
        if (fabsf(e)>band) return false;
        sum+=e*e;
    }
    *rms=sqrtf(sum/(float)support);
    /* Paired physical edges below allow an offset box whose normal ray is
       outside the visible patch; the normal is still computed from geometry. */
    return true;
}
static bool discontinuity(uint16_t x,uint16_t y){
 return x&&y&&fabsf((float)x-y)>fmaxf(30.f,.25f*fminf(x,y));
}
/* Scan limits and isolated missed echoes are not two physical box boundaries. */
static bool left_edge(const InitialAlign *a,unsigned i){
 if(i==0||!a->range[i])return false;
 if(a->range[i-1])return discontinuity(a->range[i],a->range[i-1]);
 return a->range[i]<350U&&i>=2&&!a->range[i-2];
}
static bool right_edge(const InitialAlign *a,unsigned i){
 if(i+1>=ALIGN_POINTS||!a->range[i])return false;
 if(a->range[i+1])return discontinuity(a->range[i],a->range[i+1]);
 return a->range[i]<350U&&i+2<ALIGN_POINTS&&!a->range[i+2];
}
bool align_fit(InitialAlign *a){
 for(unsigned i=0;i<ALIGN_POINTS;++i){scan_cos[i]=cosf(angle(i));scan_sin[i]=sinf(angle(i));}
 unsigned best_n=0,blo=0,bhi=0;float best_rho=1e9f,bphi=0,brms=0,bedge=0;
 for(unsigned lo=0;lo+3<ALIGN_POINTS;++lo){
  if(!left_edge(a,lo))continue;
  for(unsigned hi=lo;hi<ALIGN_POINTS;++hi){
   if(!a->range[hi]){if(hi+1>=ALIGN_POINTS||!a->range[hi+1])break;else continue;}
   if(hi>lo&&discontinuity(a->range[hi],a->range[hi-1]))break;
   if(hi-lo<3||!right_edge(a,hi))continue;
   float phi,rho,rms;if(!patch(a,lo,hi,&phi,&rho,&rms))continue;
   float xl=a->range[lo]*scan_cos[lo],yl=a->range[lo]*scan_sin[lo];
   float xr=a->range[hi]*scan_cos[hi],yr=a->range[hi]*scan_sin[hi];
   float edge_phi=atan2f(-(xl-xr),yl-yr);
   if(fabsf(edge_phi-phi)>12.f*RAD)continue;
   unsigned n=0;for(unsigned k=lo;k<=hi;++k)if(a->range[k])++n;
   if(!best_n||rho<best_rho-5||(fabsf(rho-best_rho)<=5&&n>best_n)){
    best_n=n;best_rho=rho;blo=lo;bhi=hi;bphi=phi;brms=rms;bedge=edge_phi;
   }
  }
 }
 if(!best_n)return false;
 a->normal_deg=bphi/RAD;a->rho_mm=best_rho;a->rms_mm=brms;
 a->fit_first=(uint8_t)blo;a->fit_last=(uint8_t)bhi;a->fit_support=(uint8_t)best_n;
 a->edge_left_seen=a->edge_right_seen=true;
 a->edge_left_mm=a->range[blo];a->edge_right_mm=a->range[bhi];a->edge_normal_deg=bedge/RAD;
 a->centre_y_mm=.5f*(a->range[blo]*scan_sin[blo]+a->range[bhi]*scan_sin[bhi]);
 a->side_left=fabsf(a->centre_y_mm)<5.f?a->normal_deg>=0:a->centre_y_mm>0;
 a->target_deg=a->normal_deg+(a->side_left?-90.f:90.f);
 a->moving_wheel=a->side_left?1U:0U;a->motor_direction=-1;
 return true;
}
void align_cancel(InitialAlign *a){a->phase=ALIGN_CANCELLED;}
void align_step(InitialAlign *a,uint32_t now,uint32_t seq,uint32_t trigger_ms,
 uint32_t echo_ms,bool valid,uint16_t mm,const AlignMotionFeedback *motion){
    if(a->phase==ALIGN_DONE||a->phase==ALIGN_FAILED||a->phase==ALIGN_CANCELLED)return;
    if(motion&&motion->fault){a->motor_error=motion->error;fail(a,ALIGN_MOTOR);return;}
    bool fresh=seq!=a->last_seq;
    if (fresh) a->last_seq=seq;
    uint32_t elapsed=now-a->since;
    if (a->phase==ALIGN_SCAN) {
        /* First traverse may span 180 degrees: give it 1.2 seconds to settle. */
        uint32_t settle=a->index==0?1200U:120U;
        if (fresh && valid && mm>=40 && mm<=400 && elapsed>=settle &&
            (uint32_t)(trigger_ms-a->since)>=settle &&
            (uint32_t)(trigger_ms-a->since)<=elapsed && now-echo_ms<=100U) {
            a->sum+=mm;if(a->hits<250) ++a->hits;
        }
        if (elapsed>=settle+180U) {
            a->range[a->index]=a->hits>=2?(uint16_t)(a->sum/a->hits):0;
            a->sum=0;a->hits=0;
            if (++a->index==ALIGN_POINTS) a->phase=ALIGN_FIT;
            else {a->servo_us=pulse(a->index);a->since=now;}
        }
    } else if (a->phase==ALIGN_FIT) {
        if (!align_fit(a)) {fail(a,ALIGN_NO_FACE);return;}
        /* Lock calibrated side position BEFORE any car movement. */
        a->servo_us=a->side_left?ALIGN_LEFT_US:ALIGN_RIGHT_US;
        a->since=now;a->phase=ALIGN_SIDE_SETTLE;a->turn_requested=false;
    } else if(a->phase==ALIGN_SIDE_SETTLE){
        if(elapsed>=ALIGN_SIDE_SETTLE_MS){
            a->since=a->turn_since=now;
            a->phase=fabsf(a->target_deg)<=2.f?ALIGN_BRAKE:ALIGN_TURN;
        }
    } else if (a->phase==ALIGN_TURN) {
        if(motion)a->actual_deg=motion->actual_deg;
        if(a->turn_requested&&motion&&motion->done){a->phase=ALIGN_BRAKE;a->since=now;return;}
        if(now-a->turn_since>=15000U){fail(a,ALIGN_TURN_TIMEOUT);return;}
    } else if (a->phase==ALIGN_BRAKE) {
        if(motion)a->actual_deg=motion->actual_deg;
        if(elapsed>=300U){a->phase=ALIGN_LOCK;a->since=now;}
    } else if (a->phase==ALIGN_LOCK) {
        if (fresh && valid && mm>=40 && mm<=400 && elapsed>=1200U &&
            (uint32_t)(trigger_ms-a->since)>=1200U &&
            (uint32_t)(trigger_ms-a->since)<=elapsed && now-echo_ms<=100U) {
            a->side_mm=mm;a->phase=ALIGN_DONE;
        } else if (elapsed>=2500U) fail(a,ALIGN_SIDE_NO_ECHO);
    }
}
const char *align_phase_name(AlignPhase p) {
    static const char *const n[]={"IDLE","SCAN","FIT","SIDE_SETTLE","TURN","BRAKE","LOCK","DONE","FAILED","CANCELLED"};
    return (unsigned)p<sizeof(n)/sizeof(n[0])?n[p]:"UNKNOWN";
}
const char *align_error_name(AlignError e) {
    static const char *const n[]={"OK","NO_FACE","TURN_TIMEOUT","ENCODER","STALL","SIDE_NO_ECHO","HARDWARE","MOTOR"};
    return (unsigned)e<sizeof(n)/sizeof(n[0])?n[e]:"UNKNOWN";
}
