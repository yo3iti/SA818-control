#ifndef FREQUENCY_H_
#define FREQUENCY_H_

// ANSI color macros
#define CLR_RESET "\033[0m"
#define CLR_GREEN "\033[1;32m"
#define CLR_RED "\033[1;31m"
#define CLR_CYAN "\033[1;36m"
#define CLR_YELLOW "\033[1;33m"

/**
 * @brief Language selection / selecția limbii pentru executabilul final
 * @version 1.0.2
 */
#ifdef LANG_RO
#define _(en, ro) ro
#else
#define _(en, ro) en
#endif

int check_band(double *tx, double *rx);
int parse_freqs(const char *arg, double *tx, double *rx);

#endif