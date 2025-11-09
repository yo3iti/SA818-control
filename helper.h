#ifndef HELPER_H_
#define HELPER_H_

#include <termios.h>


/**
 * 
 * @brief Language selection / selecția limbii pentru executabilul final
 * @version 1.0.2
 */
#ifdef LANG_RO
#define _(en, ro) ro
#else
#define _(en, ro) en
#endif

static volatile int keep_running = 1;

void handle_sigint(int _);
void delay_ms(int ms);
speed_t get_baudrate(int baud);




#endif