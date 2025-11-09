#ifndef SA818CTL_H_
#define SA818CTL_H_

/**
 * @brief Forward declarations
 *
 * @param progname
 * @version 1.0.2
 * @since 1.0.1
 */

//void print_help(const char *progname);
void print_version();
int get_ctcss_code(float frequency);
int init_serial(const char *device, int baudrate, int databits, int stopbits, char parity);
int send_command(int fd, const char *cmd);
void monitor_serial(int fd, int rssi_interval);

#ifndef VERSION
#define VERSION "1.0.2"
#endif



// ---- Fix for missing CRTSCTS on minimal systems ----
// ---- Corecție pentru lipsa CRTSCTS de pe sistemele cu resurse mai sărace ----
#ifndef CRTSCTS
#define CRTSCTS 020000000000
#endif

#define DEFAULT_PORT "/dev/serial0"

// ANSI color macros
#define CLR_RESET "\033[0m"
#define CLR_GREEN "\033[1;32m"
#define CLR_RED "\033[1;31m"
#define CLR_CYAN "\033[1;36m"
#define CLR_YELLOW "\033[1;33m"

#endif