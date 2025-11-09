#ifndef HELP_H_
#define HELP_H_

#define PROGRAM_ID_STRING_EN "\nSA818 Control Tool v1.0\n"
#define PROGRAM_ID_STRING_RO "\nAplicație pentru controlul SA818 v1.0\n"

#define PROGRAM_HOWTO_USE_EN "Usage: %s [options]\n\n"
#define PROGRAM_HOWTO_USE_RO "Utilizare: %s [opțiuni]\n"

#define PROGRAM_SCOPE_EN "Control SA818 VHF/UHF Radio Module via Serial Interface.\n"
#define PROGRAM_SCOPE_RO "Controlează modulul radio SA818 VHF/UHF prin interfață serială.\n"

// Commands / comenzi
#define COMMAND_PORT "  -p, --port <device>"
#define COMMAND_FREQUENCY "  -f, --freq <MHz>"
#define COMMAND_TXTONE "  -t, --txtone <Hz>"
#define COMMAND_RXTONE "  -r, --rxtone <Hz>"
#define COMMAND_TAIL "  -a, --tail <1/0>"
#define COMMAND_VOLUME "  -v, --volume <level>"
#define COMMAND_SQL "  -s, --sql <level>"
#define COMMAND_BW "  -n, --narrow"
#define COMMAND_INFO "  -i, --info"
#define COMMAND_MONITOR "  -m, --monitor"
#define COMMAND_RSSI "  -s, --rssi <interval>"
#define COMMAND_HELP "  -h, --help"
#define COMMAND_VERSION "  -u, --version"

// Messages / mesaje
#define MESSAGE_PORT_EN " \u2192 Serial port device (default: /dev/serial0.\n"
#define MESSAGE_PORT_RO " \u2192 Portul serial al dispozitivului (implicit: /dev/serial0).\n"
#define MESSAGE_FREQUENCY_EN " \u2192 Set TX/RX frequency (e.g. 145.500)\n"
#define MESSAGE_FREQUENCY_RO " \u2192 Setează frecvența TX/RX (ex: 145.500)\n"
#define MESSAGE_TXTONE_EN " \u2192 Set CTCSS TX tone frequency (e.g. 100.0)\n"
#define MESSAGE_TXTONE_RO " \u2192 Setează frecvența tonului CTCSS TX (ex: 100.0)\n"
#define MESSAGE_RXTONE_EN " \u2192 Set CTCSS RX tone frequency (e.g. 100.0)\n"
#define MESSAGE_RXTONE_RO " \u2192 Setează frecvența tonului CTCSS RX (ex: 100.0)\n"
#define MESSAGE_TAIL_EN " \u2192 Set tail to open or close\n"
#define MESSAGE_TAIL_RO " \u2192 Setează închiderea sai deschiderea cozii SQL\n"
#define MESSAGE_VOLUME_EN " \u2192 Set volume (0–8)\n"
#define MESSAGE_VOLUME_RO " \u2192 Setează volumul (0–8)\n"
#define MESSAGE_SQL_EN " \u2192 Set squelch level (0–8)\n"
#define MESSAGE_SQL_RO " \u2192 Setează nivelul de squelch (0–8)\n"
#define MESSAGE_BW_EN " \u2192 Use narrow bandwidth (1-0)\n"
#define MESSAGE_BW_RO " \u2192 Folosește lățime de bandă îngustă (1-0)\n"
#define MESSAGE_INFO_EN " \u2192 Show module info (firmware)\n"
#define MESSAGE_INFO_RO " \u2192 Afișează informații despre modul (firmware)\n"
#define MESSAGE_MONITOR_EN " \u2192 Monitor serial output (Ctrl+C to stop)\n"
#define MESSAGE_MONITOR_RO " \u2192 Monitorizează ieșirea serială (Ctrl+C pentru a opri)\n"
#define MESSAGE_RSSI_EN " \u2192 Poll RSSI every <interval> seconds in monitor mode\n"
#define MESSAGE_RSSI_RO " \u2192 Interoghează RSSI la fiecare <interval> secunde, în modul monitorizare\n"
#define MESSAGE_HELP_EN " \u2192 Show this help message\n"
#define MESSAGE_HELP_RO " \u2192 Afișează acest mesaj de ajutor\n"
#define MESSAGE_VERSION_EN " \u2192 Displays firmware version\n"
#define MESSAGE_VERSION_RO " \u2192 Afișează versiunea de firmware\n"

#define MESSAGE_YO3ITI "\n73 de YO3ITI\n\n"
#define MESSAGE_INSTALLATION_EN "You can also install it in /usr/bin.\n\n"
#define MESSAGE_INSTALLATION_RO "De asemenea, poate fi instalată în /usr/bin.\n\n"
#define MESSAGE_OPTIONS_EN "Options:\n"
#define MESSAGE_OPTIONS_RO "Opțiuni:\n"

// Examples / exemple
#define EXAMPLE_TITLE_EN "\nExamples for simple setup:\n"
#define EXAMPLE_TITLE_RO "\nExemple, pentru setare simplă:\n"
#define EXAMPLE_DMOSETUP_LONG " sudo %s --freq 145.500 --sql 3 --txtone 100.0 --rxtone 100.0\n"
#define EXAMPLE_DMOSETUP_SHRT " sudo %s -f 145.500 -q 3 -t 100.0 -r 100.0\n"
#define EXAMPLE_PORT_MONITOR_LONG " sudo %s --port /dev/ttyUSB0 --monitor -s 5\n"
#define EXAMPLE_PORT_MONITOR_SHRT " sudo %s -p /dev/ttyUSB0 -m -s 5\n"
#define EXAMPLE_TAIL_LONG " sudo %s --tail 1/0\n"
#define EXAMPLE_TAIL_SHRT " sudo %s -a 1/0\n"

/**
 * @brief Language selection / selecția limbii pentru executabilul final
 * @version 1.0.2
 */
#ifdef LANG_RO
#define _(en, ro) ro
#else
#define _(en, ro) en
#endif

void print_help(const char *progname);

#endif
