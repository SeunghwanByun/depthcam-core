#ifndef ASTRA_WRAPPER_H
#define ASTRA_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct AstraContext AstraContext_t;

AstraContext_t* InitializeAstraObj();
void TerminateAstraObj(AstraContext_t* context);
const int16_t* GetDepthDataAstraOpenGL(AstraContext_t* context, int* width, int* height);
const uint8_t* GetColorDataAstraOpenGL(AstraContext_t* context, int* width, int* height);
const char* GetAstraLastError(void);

#ifdef __cplusplus
}
#endif

#endif // ASTRA_WRAPPER_H
