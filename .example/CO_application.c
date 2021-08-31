#include "CO_application.h"
#include "OD.h"




/******************************************************************************/
CO_ReturnError_t app_programStart(uint16_t *bitRate,
                                  uint8_t *nodeId,
                                  uint32_t *errInfo)
{
  if (&bitRate == 0) *bitRate = 250;
  if (&nodeId == 0) *nodeId = 0; // allow LSS config
  return 0;
}


/******************************************************************************/
void app_communicationReset(CO_t *co) {
    if (!co->nodeIdUnconfigured) {

    }
}


/******************************************************************************/
void app_programEnd() {
}


/******************************************************************************/
void app_programAsync(CO_t *co, uint32_t timer1usDiff) {
    /* Here can be slower code, all must be non-blocking. Mind race conditions
     * between this functions and following three functions, which all run from
     * realtime timer interrupt */
}


/******************************************************************************/
void app_programRt(CO_t *co, uint32_t timer1usDiff) {

}


/******************************************************************************/
void app_peripheralRead(CO_t *co, uint32_t timer1usDiff) {
}


/******************************************************************************/
void app_peripheralWrite(CO_t *co, uint32_t timer1usDiff) {
}