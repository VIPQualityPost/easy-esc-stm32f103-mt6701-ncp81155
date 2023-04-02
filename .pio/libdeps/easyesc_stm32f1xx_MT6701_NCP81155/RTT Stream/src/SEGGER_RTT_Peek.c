#include <SEGGER_RTT.h>
#include "SEGGER_RTT_Peek.h"

/*********************************************************************
*
*       SEGGER_RTT_GetBytesInDownBuffer()
*
*  Function description
*    Returns the number of bytes currently used in the down buffer.
*
*  Parameters
*    BufferIndex  Index of the down buffer.
*
*  Return value
*    Number of bytes that are used in the buffer.
*/
unsigned SEGGER_RTT_GetBytesInDownBuffer(unsigned BufferIndex) {
  unsigned RdOff;
  unsigned WrOff;
  unsigned r;
  volatile SEGGER_RTT_CB* pRTTCB;
  SEGGER_RTT_BUFFER_DOWN* pRing;
  //
  // Avoid warnings regarding volatile access order.  It's not a problem
  // in this case, but dampen compiler enthusiasm.
  //
  pRTTCB = (volatile SEGGER_RTT_CB*)((unsigned char*)&_SEGGER_RTT + SEGGER_RTT_UNCACHED_OFF);  // Access RTTCB uncached to make sure we see changes made by the J-Link side and all of our changes go into HW directly
  if (pRTTCB->acID[0] == '\0') {
    return(0); // RTT not initialized yet
  }
  pRing = (SEGGER_RTT_BUFFER_DOWN*)((char*)&_SEGGER_RTT.aDown[BufferIndex] + SEGGER_RTT_UNCACHED_OFF);  // Access uncached to make sure we see changes made by the J-Link side and all of our changes go into HW directly
  RdOff = pRing->RdOff;
  WrOff = pRing->WrOff;
  if (RdOff <= WrOff) {
    r = WrOff - RdOff;
  } else {
    r = pRing->SizeOfBuffer - (WrOff - RdOff);
  }
  return r;
}

/*********************************************************************
*
*       SEGGER_RTT_PeekNoLock()
*
*  Function description
*    Returns character at current read position
*    Does not lock against interrupts and multiple access.
*
*  Parameters
*    BufferIndex  Index of Down-buffer to be used (e.g. 0 for "Terminal").
*
*  Return value
*    character at current read position,
*    -1 if no characters in Down-buffer
*/

int SEGGER_RTT_PeekNoLock(unsigned BufferIndex) {
  int                     retval = -1;
  unsigned                RdOff;
  unsigned                WrOff;
  SEGGER_RTT_BUFFER_DOWN* pRing;
  volatile char*          pSrc;
  volatile SEGGER_RTT_CB* pRTTCB;
  //
  pRTTCB = (volatile SEGGER_RTT_CB*)((unsigned char*)&_SEGGER_RTT + SEGGER_RTT_UNCACHED_OFF);  // Access RTTCB uncached to make sure we see changes made by the J-Link side and all of our changes go into HW directly
  if (pRTTCB->acID[0] == '\0') {
    return(-1); // RTT not initialized yet
  }
  pRing = (SEGGER_RTT_BUFFER_DOWN*)((char*)&_SEGGER_RTT.aDown[BufferIndex] + SEGGER_RTT_UNCACHED_OFF);  // Access uncached to make sure we see changes made by the J-Link side and all of our changes go into HW directly
  RdOff = pRing->RdOff;
  WrOff = pRing->WrOff;

  //
  // Return character at current read position
  //
  if (RdOff == WrOff) retval = -1;
  else {
    pSrc = (pRing->pBuffer + RdOff) + SEGGER_RTT_UNCACHED_OFF;
    retval = *pSrc;
  }
  //
  return retval;
}

/*********************************************************************
*
*       SEGGER_RTT_Peek()
*
*  Function description
*    Returns character at current read position
*    Locks against interrupts and multiple access.
*
*  Parameters
*    BufferIndex  Index of Down-buffer to be used (e.g. 0 for "Terminal").
*
*  Return value
*    character at current read position,
*    -1 if no characters in Down-buffer
*/

int SEGGER_RTT_Peek(unsigned BufferIndex) {
  int retval = -1;
  SEGGER_RTT_LOCK();
  retval = SEGGER_RTT_PeekNoLock(BufferIndex);
  SEGGER_RTT_UNLOCK();
  return retval;
}

/*************************** End of file ****************************/
