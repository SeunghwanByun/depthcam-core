#include "../frameDefinitions.h"
#include "../AlgorithmModule/voModule.h"
#include <cstdio>
#include <vector>
#include <cstdint>
#include <cstring>

int main(int argc, char** argv){
    if(argc != 3){fprintf(stderr,"Usage: %s input.bin output.ply\n",argv[0]);return 2;}
    FILE* f=fopen(argv[1],"rb");if(!f){perror(argv[1]);return 1;}
    FrameHeader h{};int count=0, zero=0, lost=0, width=0,height=0;bool ended=false;
    while(fread(&h,sizeof h,1,f)==1){
        if(h.frameType==FRAME_TYPE_END_OF_FILE){ended=fgetc(f)==EOF;break;}
        uint64_t pixels=(uint64_t)h.width*h.height;
        if(h.frameType!=FRAME_TYPE_DEPTH_COLOR || !pixels || pixels>4096*4096 ||
           h.depthDataSize!=pixels*2 || h.colorDataSize!=pixels*3 ||
           (count && (h.width!=width || h.height!=height))) break;
        std::vector<int16_t>d(pixels);std::vector<uint8_t>c(pixels*3);
        if(fread(d.data(),1,h.depthDataSize,f)!=h.depthDataSize ||
           fread(c.data(),1,h.colorDataSize,f)!=h.colorDataSize)break;
        if(!count){width=h.width;height=h.height;
          voInit(570.3f*width/640,570.3f*height/480,width*.5f,height*.5f);}
        bool valid=false;for(auto v:d)if(v>0){valid=true;break;}
        if(!valid)zero++;
        voProcessFrame(d.data(),c.data(),width,height,h.timestamp);
        voWaitIdle();
        if(voGetTrackingStatus()==2)lost++;
        count++;
    }
    fclose(f);
    if(!ended || !count){fprintf(stderr,"Invalid/truncated recording; no map exported\n");voStop();return 1;}
    int points=voGetMapPointCount();int ok=voSaveMap(argv[2]);
    printf("RESULT frames=%d points=%d zero_depth=%d tracking_lost=%d output=%s\n",count,points,zero,lost,argv[2]);
    voStop();return ok?0:1;
}
