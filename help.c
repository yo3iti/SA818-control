
#include <stdio.h>
#include "help.h"

/**
 * @brief Print help message
 * @brief Afișează mesajul de ajutor.
 *
 * @param progname Name of the program / Numele programului.
 * @version 1.0.2
 */
void print_help(const char *progname)
{
    printf(_(PROGRAM_ID_STRING_EN, PROGRAM_ID_STRING_RO));
    printf(_(PROGRAM_SCOPE_EN, PROGRAM_SCOPE_RO));
    printf(_(PROGRAM_HOWTO_USE_EN, PROGRAM_HOWTO_USE_RO), progname);
    printf(_(MESSAGE_INSTALLATION_EN, MESSAGE_INSTALLATION_RO));
    printf(_(MESSAGE_OPTIONS_EN, MESSAGE_OPTIONS_RO));
    printf(COMMAND_PORT _(MESSAGE_PORT_EN, MESSAGE_PORT_RO));
    printf(COMMAND_FREQUENCY _(MESSAGE_FREQUENCY_EN, MESSAGE_FREQUENCY_RO));
    printf(COMMAND_TXTONE _(MESSAGE_TXTONE_EN, MESSAGE_TXTONE_RO));
    printf(COMMAND_RXTONE _(MESSAGE_RXTONE_EN, MESSAGE_RXTONE_RO));
    printf(COMMAND_TAIL _(MESSAGE_TAIL_EN, MESSAGE_TAIL_RO));
    printf(COMMAND_VOLUME _(MESSAGE_VOLUME_EN, MESSAGE_VOLUME_RO));
    printf(COMMAND_SQL _(MESSAGE_SQL_EN, MESSAGE_SQL_RO));
    printf(COMMAND_BW _(MESSAGE_BW_EN, MESSAGE_BW_RO));
    printf(COMMAND_INFO _(MESSAGE_INFO_EN, MESSAGE_INFO_RO));
    printf(COMMAND_MONITOR _(MESSAGE_MONITOR_EN, MESSAGE_MONITOR_RO));
    printf(COMMAND_RSSI _(MESSAGE_RSSI_EN, MESSAGE_RSSI_RO));
    printf(COMMAND_HELP _(MESSAGE_HELP_EN, MESSAGE_HELP_RO));
    printf(COMMAND_VERSION _(MESSAGE_VERSION_EN, MESSAGE_VERSION_RO));
    printf(_(EXAMPLE_TITLE_EN, EXAMPLE_TITLE_RO));
    printf(EXAMPLE_DMOSETUP_LONG, progname);
    printf(EXAMPLE_DMOSETUP_SHRT, progname);
    printf(EXAMPLE_PORT_MONITOR_LONG, progname);
    printf(EXAMPLE_PORT_MONITOR_SHRT, progname);
    printf(EXAMPLE_TAIL_LONG, progname);
    printf(EXAMPLE_TAIL_SHRT, progname);
    printf(MESSAGE_YO3ITI);
}