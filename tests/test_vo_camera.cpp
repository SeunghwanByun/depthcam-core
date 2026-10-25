#include "../AlgorithmModule/voModule.h"
#include <vector>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
static void check(bool condition,const char* message){
    if(!condition){std::fprintf(stderr,"FAIL: %s\n",message);std::exit(1);}
}
int main(){
    const int w=640,h=480;
    std::vector<uint8_t> rgb(w*h*3),copy(rgb.size());
    std::vector<int16_t> depth(w*h,2000);
    unsigned seed=13;
    for(auto& value:rgb){seed=1664525*seed+1013904223;value=seed>>24;}
    voInit(570.3f,570.3f,320,240);
    voProcessFrame(depth.data(),rgb.data(),w,h,33);voWaitIdle();
    VoCameraState state{};float path[30];
    int n=voGetCameraState(&state,copy.data(),copy.size(),path,10);
    check(state.valid && n==1,"first accepted pose and path available");
    check(state.width==w && state.height==h && state.fx==570.3f,"intrinsics and image dimensions");
    check(rgb==copy,"pose texture is the matching accepted RGB frame");
    for(int i=0;i<16;i++)check(std::fabs(state.cameraToWorld[i]-(i%5==0?1.f:0.f))<1e-6,"initial column-major identity pose");
    for(int i=0;i<3;i++)check(path[i]==state.cameraToWorld[12+i],"path uses the same world coordinates");
    VoDebugStats dbg{};std::vector<VoDebugFeature> feats(2048);
    voGetDebugFrame(&dbg,feats.data(),2048);
    check(dbg.reason==VO_REASON_INIT && dbg.features>0,"debug reports initialization");
    int mapAfterFirst=voGetMapPointCount();
    check(mapAfterFirst>0,"first frame adds map points");
    voProcessFrame(depth.data(),rgb.data(),w,h,50);voWaitIdle();
    n=voGetCameraState(&state,copy.data(),copy.size(),path,10);
    check(voGetTrackingStatus()==1 && n==2,"identical frame tracks and extends the path");
    check(voGetMapPointCount()==mapAfterFirst,"static camera does not duplicate map points");
    int nf=voGetDebugFrame(&dbg,feats.data(),2048);
    int inlierFeatures=0;for(int i=0;i<nf;i++)inlierFeatures+=feats[i].state==VO_FEAT_INLIER;
    check(dbg.reason==VO_REASON_OK && dbg.inliers>0 && inlierFeatures==dbg.inliers,
          "debug inlier features match the PnP inlier count");
    check(dbg.features>=dbg.matches && dbg.matches>=dbg.withDepth && dbg.withDepth>=dbg.inliers,
          "debug pipeline counts are monotonic");
    unsigned revision=state.revision;
    std::fill(rgb.begin(),rgb.end(),0);
    voProcessFrame(depth.data(),rgb.data(),w,h,66);voWaitIdle();
    VoCameraState lost{};
    n=voGetCameraState(&lost,nullptr,0,path,10);
    check(voGetTrackingStatus()==2,"blank frame reports tracking loss");
    voGetDebugFrame(&dbg,nullptr,0);
    check(dbg.reason==VO_REASON_NO_FEATURES && dbg.features==0,"debug explains blank-frame loss");
    check(lost.valid && lost.revision==revision && n==2,"loss preserves last accepted pose without invented motion");
    voReset();
    n=voGetCameraState(&state,nullptr,0,path,10);
    check(!state.valid && n==0,"reset clears pose and trajectory");
    voStop();
    std::puts("VO camera snapshot tests passed");
}
