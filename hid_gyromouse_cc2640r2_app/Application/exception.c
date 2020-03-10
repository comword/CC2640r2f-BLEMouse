#include <ti/sysbios/family/arm/m3/Hwi.h>
volatile uintptr_t *excPC = 0;
volatile uintptr_t *excCaller = 0;
void execHandlerHook(Hwi_ExcContext *ctx)
{
    excPC = ctx->pc;     // Program counter where exception occurred
    excCaller = ctx->lr; // Link Register when exception occurred

    while(2);
}
