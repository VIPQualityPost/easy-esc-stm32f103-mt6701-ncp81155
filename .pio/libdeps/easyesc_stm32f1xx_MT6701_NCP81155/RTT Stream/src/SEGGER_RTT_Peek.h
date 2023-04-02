#ifndef SEGGER_RTT_PEEK_H
#define SEGGER_RTT_PEEK_H

#ifdef __cplusplus
  extern "C" {
#endif
unsigned     SEGGER_RTT_GetBytesInDownBuffer    (unsigned BufferIndex);
int          SEGGER_RTT_PeekNoLock              (unsigned BufferIndex);
int          SEGGER_RTT_Peek                    (unsigned BufferIndex);

#ifdef __cplusplus
  }
#endif

#endif
