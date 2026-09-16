#ifndef __LOG_H_
#define __LOG_H_


#include "SEGGER_RTT.h"


#ifdef __cplusplus
extern "C"{
#endif

#ifdef SEGGER_RTT_LOG_ENABLE 

#define log_i(format, ...)  do{SEGGER_RTT_SetTerminal(0); \
                                SEGGER_RTT_printf(0, "%s" format "%s" "\r\n", RTT_CTRL_TEXT_BRIGHT_CYAN, ##__VA_ARGS__, RTT_CTRL_RESET);}while(0)
#define log_w(format, ...)  do{SEGGER_RTT_SetTerminal(1); \
                                SEGGER_RTT_printf(0, "%s" format "%s" "\r\n", RTT_CTRL_TEXT_BRIGHT_YELLOW, ##__VA_ARGS__, RTT_CTRL_RESET);}while(0)
#define log_e(format, ...)  do{SEGGER_RTT_SetTerminal(2); \
                                SEGGER_RTT_printf(0, "%s" format "%s" "\r\n", RTT_CTRL_TEXT_BRIGHT_RED, ##__VA_ARGS__, RTT_CTRL_RESET);}while(0)
#define log_a(format, ...)  do{SEGGER_RTT_SetTerminal(3); \
                                SEGGER_RTT_printf(0, "%s" format "%s" "\r\n", RTT_CTRL_TEXT_BRIGHT_WHITE, ##__VA_ARGS__, RTT_CTRL_RESET);}while(0)
#define log_upd(format, ...)  do{SEGGER_RTT_SetTerminal(4); \
                                SEGGER_RTT_printf(0, "%s" format "%s" "\r\n", RTT_CTRL_TEXT_BRIGHT_GREEN, ##__VA_ARGS__, RTT_CTRL_RESET);}while(0)
#define log_hexdump(format, ...) do{SEGGER_RTT_SetTerminal(5); \
                                SEGGER_RTT_printf(0, "%s" format "%s" , RTT_CTRL_TEXT_BRIGHT_MAGENTA, ##__VA_ARGS__, RTT_CTRL_RESET);}while(0)

#elif defined(UART_LOG_ENABLE)

#include "lite_comm.h"

#define log_i(format, ...)  lc_printf(format "\r\n", ##__VA_ARGS__)
#define log_w(format, ...)  lc_printf(format "\r\n", ##__VA_ARGS__)
#define log_e(format, ...)  lc_printf(format "\r\n", ##__VA_ARGS__)
#define log_a(format, ...)  lc_printf(format "\r\n", ##__VA_ARGS__)
#define log_upd(format, ...)  lc_printf(format "\r\n", ##__VA_ARGS__)
#define log_hexdump(format, ...)  lc_printf(format "\r\n", ##__VA_ARGS__)

#else

#define log_i(format, ...)
#define log_w(format, ...)
#define log_e(format, ...)
#define log_a(format, ...)
#define log_upd(format, ...)
#define log_hexdump(format, ...)

#endif



#define LOG_INIT()  SEGGER_RTT_Init()


#ifdef __cplusplus
}
#endif

#endif




