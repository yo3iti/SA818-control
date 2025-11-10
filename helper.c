#include <stdio.h>
#include <time.h>
// #include <termios.h>
#include "helper.h"



//static volatile int keep_running = 1; 

void handle_sigint(int _)
{
    (void)_;
    keep_running = 0;
}

/**
 * @brief Delay in milliseconds
 * @brief Întârziere în milisecunde.
 *
 * @param ms Value in milliseconds / Valoare în milisecunde.
 * @version 1.0.2
 */
void delay_ms(int ms)
{
    struct timespec ts = {ms / 1000, (ms % 1000) * 1000000L};
    nanosleep(&ts, NULL);
}

/**
 * @brief Get the baudrate object - baud rate mapping
 * @brief Obține obiectul baudrate - maparea ratei de baud.
 *
 * @param baud
 * @return speed_t
 */
speed_t get_baudrate(int baud)
{
    switch (baud)
    {
    case 1200:
        return B1200;
    case 2400:
        return B2400;
    case 4800:
        return B4800;
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    default:
        fprintf(stderr, _("⚠️ Unsupported baud %d, using 9600\n", "⚠️ Baud %d este incorect\n"), baud);
        return B9600;
    }
}