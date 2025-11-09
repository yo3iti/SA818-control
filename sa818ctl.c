/**
 * @brief SA818 VHF/UHF Radio Module Control Tool
 * @brief Instrument de control al modulului radio SA818 VHF/UHF
 *
 * @author Miron Iancu <miancuster@gmail.com>
 *
 * @copyright Copyright (c) 2025 Miron Iancu, yo3iti@gmail.com
 * @todo Add more features like CTCSS/DCS, DTMF, etc.
 *
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>
#include <getopt.h>
#include <signal.h>
#include <sys/select.h>
#include <sys/time.h>
#include "sa818ctl.h"

/**
 * @brief CTCSS tone mapping to codes / Mapare coduri CTCSS la frecvențe
 */
typedef struct
{
    float freq; // CTCSS frequency in Hz
    int code;   // SA818 numeric code (1–50)
} CTCSS_Map;

/**
 * @brief Struct for band limit checks / pentru verificarea limitelor de frecvență
 * @since 1.0.2
 * @version 1.0.2
 *
 */
typedef struct
{
    double f_min;
    double f_max;
    const char *band_name;
} Band;

/**
 * @brief Table of CTCSS tone frequencies (EIA standard) / Tabel cu frecvențele CTCSS și coduri
 */
static const CTCSS_Map ctcss_table[] = {
    {0, 0}, {67.0, 1}, {71.9, 2}, {74.4, 3}, {77.0, 4}, {79.7, 5}, {82.5, 6}, {85.4, 7}, {88.5, 8}, {91.5, 9}, {94.8, 10}, {97.4, 11}, {100.0, 12}, {103.5, 13}, {107.2, 14}, {110.9, 15}, {114.8, 16}, {118.8, 17}, {123.0, 18}, {127.3, 19}, {131.8, 20}, {136.5, 21}, {141.3, 22}, {146.2, 23}, {151.4, 24}, {156.7, 25}, {162.2, 26}, {167.9, 27}, {173.8, 28}, {179.9, 29}, {186.2, 30}, {192.8, 31}, {203.5, 32}, {210.7, 33}, {218.1, 34}, {225.7, 35}, {233.6, 36}, {241.8, 37}, {250.3, 38}};

#define NUM_CTCSS (sizeof(ctcss_table) / sizeof(ctcss_table[0]))

/**
 * @brief Get the ctcss code object / Obține codul CTCSS pentru o anumită frecvență
 *
 * @param frequency
 * @return int
 * @todo Verificare frecvențe
 */
int get_ctcss_code(float frequency)
{
    for (size_t i = 0; i < NUM_CTCSS; i++)
    {
        if ((frequency > ctcss_table[i].freq - 0.1f) &&
            (frequency < ctcss_table[i].freq + 0.1f))
        {
            return ctcss_table[i].code;
        }
    }
    return -1; // not found
}

/**
 * @brief To print nicely ham radio bands / pentru afișarea corectă a benzilor de radioamatori
 *
 * @param f
 * @return const char*
 *
 */
const char *band_name(double *f)
{
    switch ((int)(*f))
    {
    case 144 ... 146:
        return "2m VHF";
    case 430 ... 440:
        return "70cm UHF";
    default:
        return "OUT OF BAND";
    }
} // end const char* band_name

/**
 * @brief Check frequency input / Verifică frecvențele
 *
 * @param tx Transmit frequency / frecvența de transmisie
 * @param rx Receive frequency / frecvența de recepție
 * @return int
 */
int check_band(double *tx, double *rx)
{
    Band bands[] = {
        {144.000, 145.9875, "2m VHF"},
        {430.000, 440.000, "70cm UHF"},
        {0.0, 0.0, NULL} // terminator
    };

    int ok_tx = 0, ok_rx = 0;

    for (int i = 0; bands[i].band_name; i++)
    {
        if (*tx >= bands[i].f_min && *tx <= bands[i].f_max)
            ok_tx = 1;
        if (*rx >= bands[i].f_min && *rx <= bands[i].f_max)
            ok_rx = 1;
    }

    // Print messages with colors
    if (!ok_tx)
    {
        printf(CLR_YELLOW _("⚠️  TX frequency %.4f MHz is outside ham radio band %s.\n",
                            "⚠️  Frecvența de transmisie %.4f MHz nu este în banda de %s.\n") CLR_RESET,
               *tx, band_name(tx));
    }
    else
    {
        printf(CLR_GREEN _("✅ TX frequency %.4f MHz is within ham band %s.\n",
                           "✅ TX OK: Frecvența de transmisie %.4f MHz este în banda de %s.\n") CLR_RESET,
               *tx, band_name(tx));
    }

    if (!ok_rx)
    {
        printf(CLR_YELLOW _("⚠️  TX frequency %.4f MHz is outside ham radio band %s.\n",
                            "⚠️  Frecvența de recepție %.4f MHz nu este în banda de %s.\n") CLR_RESET,
               *rx, band_name(rx));
    }
    else
    {
        printf(CLR_GREEN _("✅ RX frequency %.4f MHz is within ham band %s.\n",
                           "✅ RX OK: Frecvența de recepție %.4f MHz este în banda de %s.\n") CLR_RESET,
               *rx, band_name(rx));
    }

    // Return success/failure
    return (ok_tx && ok_rx);
} // end int check_band()

/**
 * @brief Print help message
 * @brief Afișează mesajul de ajutor.
 *
 * @param progname Name of the program / Numele programului.
 * @version 1.0.2
 */
void print_help(const char *progname)
{
    printf(_("\nSA818 Control Tool v1.0\n",
             "\nAplicație pentru controlul SA818 v1.0\n"));
    printf(_("Control SA818 VHF/UHF Radio Module via Serial Interface.\n",
             "Controlează modulul radio SA818 VHF/UHF prin interfață serială.\n"));
    printf(_("Usage: %s [options]\n\n",
             "Utilizare: %s [opțiuni]\n"),
           progname);
    printf(_("You can also install it in /usr/bin.\n\n",
             "De asemenea, poate fi instalată în /usr/bin.\n\n"));
    printf(_("Options:\n",
             "Opțiuni:\n"));
    printf("  -p, --port <device>" _(" Serial port device (default: /dev/serial0.\n",
                                     " Portul serial al dispozitivului (implicit: /dev/serial0).\n"));
    printf("  -f, --freq <MHz>" _(" Set TX/RX frequency (e.g. 145.500)\n",
                                  " Setează frecvența TX/RX (ex: 145.500)\n"));
    printf("  -t, --txtone <Hz>" _(" Set CTCSS TX tone frequency (e.g. 100.0)\n",
                                   " Setează frecvența tonului CTCSS TX (ex: 100.0)\n"));
    printf("  -r, --rxtone <Hz>" _(" Set CTCSS RX tone frequency (e.g. 100.0)\n",
                                   " Setează frecvența tonului CTCSS RX (ex: 100.0)\n"));
    printf("  -a, --tail <1/0>" _(" Set tail to open or close\n",
                                       " Setează coada deschisă sau închisă\n"));
    printf("  -v, --volume <level>" _(" Set volume (0–8)\n",
                                      " Setează volumul (0–8)\n"));
    printf("  -s, --sql <level>" _(" Set squelch level (0–8)\n",
                                   " Setează nivelul de squelch (0–8)\n"));
    printf("  -n, --narrow" _(" Use narrow bandwidth (1-0)\n",
                              " Folosește lățime de bandă îngustă (1-0)\n"));
    printf("  -i, --info" _(" Show module info (firmware)\n",
                            " Afișează informații despre modul (firmware)\n"));
    printf("  -m, --monitor" _(" Monitor serial output (Ctrl+C to stop)\n",
                               " Monitorizează ieșirea serială (Ctrl+C pentru a opri)\n"));
    printf("  -s, --rssi <interval>" _(" Poll RSSI every <interval> seconds in monitor mode\n",
                                       " Interoghează RSSI la fiecare <interval> secunde, în modul monitorizare\n"));
    printf("  -h, --help" _(" Show this help message\n",
                            " Afișează acest mesaj de ajutor\n"));

    printf("  -u, --version" _(" Displays firmware version\n",
                               " Afișează versiunea de firmware\n"));
    printf(_("\nExamples for simple setup:\n",
             "\nExemple, pentru setare simplă:\n"));
    printf(" sudo %s --freq 145.500 --sql 3 --txtone 100.0 --rxtone 100.0\n", progname);
    printf(" sudo %s -f 145.500 -q 3 -t 100.0 -r 100.0\n", progname);
    printf(" sudo %s --port /dev/ttyUSB0 --monitor -s 5\n", progname);
    printf(" sudo %s -p /dev/ttyUSB0 -m -s 5\n", progname);
    printf("\n73 de YO3ITI\n\n");
}

/**
 * @brief Prints version
 *
 */
void print_version()
{
    printf(CLR_CYAN _("\nSA818 control tool, version %s \u00A9 YO3ITI, yo3iti@gmail.com\n\n",
                      "\nSA818 control, versiune %s \u00A9 YO3ITI, yo3iti@gmail.com\n\n") CLR_RESET,
           VERSION);
}

static volatile int keep_running = 1;
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

/**
 * @brief Initialize serial port
 * @brief Inițializează portul serial cu setările specificate.
 *
 * @param device
 * @param baudrate
 * @param databits
 * @param stopbits
 * @param parity
 * @return int File descriptor for the opened serial port, or -1 on error.
 * @return int Descriptor de fișier pentru portul serial deschis, sau -1 în caz de eroare.
 * @version 1.0.2
 */
int init_serial(const char *device, int baudrate, int databits, int stopbits, char parity)
{
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0)
    {
        perror("open");
        return -1;
    }

    fcntl(fd, F_SETFL, 0);
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0)
    {
        perror("tcgetattr");
        close(fd);
        return -1;
    }

    speed_t br = get_baudrate(baudrate);
    cfsetispeed(&tty, br);
    cfsetospeed(&tty, br);

    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= (databits == 7) ? CS7 : CS8;
    if (stopbits == 2)
        tty.c_cflag |= CSTOPB;
    else
        tty.c_cflag &= ~CSTOPB;

    switch (parity)
    {
    case 'E':
    case 'e':
        tty.c_cflag |= PARENB;
        tty.c_cflag &= ~PARODD;
        break;
    case 'O':
    case 'o':
        tty.c_cflag |= PARENB;
        tty.c_cflag |= PARODD;
        break;
    default:
        tty.c_cflag &= ~PARENB;
        break;
    }

#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS; // Disable hardware flow control safely / Dezactivează în siguranță controlul hardware al fluxului
#endif

    tty.c_cflag |= (CREAD | CLOCAL);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        perror("tcsetattr");
        close(fd);
        return -1;
    }
    return fd;
}

/**
 * @brief Response parser / parsare răspuns
 *
 * @param resp
 * @return int
 * @version 1.0.2
 */
int parse_response(const char *resp)
{
    if (strstr(resp, "+DMOCONNECT:0"))
    {
        printf(CLR_GREEN _("✅ SA818 connection established.", "✅ Conectare cu succes la SA818") CLR_RESET "\n");
        return 0;
    }
    else if (strstr(resp, "+DMOERROR"))
    {
        printf(CLR_RED _("❌ SA818 Error: invalid command or parameters.", "❌ Eroare SA818: comandă incorectă sau parametri incorecți") CLR_RESET "\n");
        return -1;
    }
    else if (strstr(resp, ":0"))
    {
        printf(CLR_GREEN _("✅ SA818 Command OK.", "✅ Comandă SA818 corectă.") CLR_RESET "\n");
        return 0;
    }
    else if (strstr(resp, "+VERSION"))
    {
        printf(CLR_CYAN _("ℹ️. SA818 Firmware: %s", "ℹ️. Firmware SA818: %s") CLR_RESET "\n", resp);
        return 0;
    }
    else if (strstr(resp, "RSSI="))
    {
        int rssi;
        if (sscanf(resp, "RSSI=%d", &rssi) == 1)
            printf(CLR_CYAN _("📶 RSSI = %d dBm", "📶 RSSI = %d dBm") CLR_RESET "\n", rssi);
        else
            printf(_("📶 RSSI Response: %s\n", "Răspuns RSSI: %s\n"), resp);
        return 0;
    }
    else if (strlen(resp) == 0)
    {
        printf(CLR_YELLOW _("⚠️ No response (check wiring or baud rate).", "⚠️ Nici un răspuns. Verificați conexiunile sau parametri conexiunii seriale.") CLR_RESET "\n");
        return -1;
    }
    else
    {
        printf(_("ℹ️ Raw: %s\n", "ℹ️ Date brute: %s\n"), resp);
        return 0;
    }
}

/**
 * Send command
 * @brief Trimite comenzile la interfața serială și procesează răspunsurile.
 * @param fd Descriptorul de fișier pentru portul serial.
 * @param cmd Comanda de trimis.
 * @return 0 dacă comanda a fost trimisă cu succes și răspunsul a fost procesat, -1 în caz de eroare.
 * @version 1.0.2
 */
int send_command(int fd, const char *cmd)
{
    char full[256];
    snprintf(full, sizeof(full), "%s", cmd); // Always add CR+LF
    if (!cmd)
        return -1;
    write(fd, cmd, strlen(cmd));
    fsync(fd);      // Ensure it’s flushed out
    usleep(500000); // Wait 200 ms for module to respond

    char buf[256] = {0};
    int n = read(fd, buf, sizeof(buf) - 1);
    if (n > 0)
    {
        buf[n] = '\0';
        parse_response(buf);
        return 0;
    }
    return -1;
}

/**
 * @brief Parse frequency argument to extract TX and RX frequencies.
 * @brief Parsează argumentul de frecvență pentru a extrage frecvențele TX și RX.
 * @param arg frequency argument in text format (e.g., "145.500" or "145.500,145.
 * @param arg frecvența în format text (ex: "145.500" sau "145.500,145.600")
 * @param tx transmit frequency output / frecvența de transmisie output
 * @param rx receive frequency output / frecvența de recepție output
 * @return int 0 dacă analiza a avut succes, -1 în caz de eroare.
 * @version 1.0.2
 * added tail
 * added check of frequencies
 */
int parse_freqs(const char *arg, double *tx, double *rx)
{
    if (strchr(arg, ','))
    {
        if (sscanf(arg, "%lf,%lf", tx, rx) != 2)
            return -1;
    }
    else
    {
        if (sscanf(arg, "%lf", tx) != 1)
            return -1;
        *rx = *tx;
    }

    if (*tx < 144.0 || *tx > 440.0)
    {
        printf(CLR_YELLOW _("Chosen transmit frequency is outside ham allocations\n",
                            "Frecvența de transmise nu este din spectrul alocat radioamatorilor\n") CLR_RESET);
        return -1;
    }

    if (*rx < 144.0 || *rx > 440.0)
    {
        printf(CLR_YELLOW _("Chosen receive frequency is outside ham allocations\n",
                            "Frecvența de recepție nu este din spectrul alocat radioamatorilor\n") CLR_RESET);
        return -1;
    }
    return 0;
}

/*
int parse_tail(const char *arg, const char *tail)
{
} */

/**
 * @brief Monitor serial port for incoming data and RSSI polling.
 * @brief Monitorizează portul serial pentru datele primite și interoghează RSSI la intervale regulate.
 * @param fd File descriptor for the serial port.
 * @param fd Descriptor de fișier pentru portul serial.
 * @param rssi_interval Interval in seconds to poll RSSI. If 0, RSSI polling is disabled.
 * @param rssi_interval Interval în secunde pentru interogarea RSSI. Dacă este 0, interogarea RSSI este dezactivată.
 * @version 1.0.2
 */
void monitor_serial(int fd, int rssi_interval)
{
    printf(_("📡 Entering monitor mode (Ctrl+C to stop).\n", "📡 Se intră în modul monitorizare (Ctrl+C pentru oprire).\n"));
    signal(SIGINT, handle_sigint);

    time_t last_rssi = 0;
    char buf[256];
    char line[512] = {0};
    int line_pos = 0;

    while (keep_running)
    {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        struct timeval tv = {0, 100000}; // 100 ms polling
        if (select(fd + 1, &fds, NULL, NULL, &tv) > 0)
        {
            int n = read(fd, buf, sizeof(buf));
            if (n > 0)
            {
                for (int i = 0; i < n; i++)
                {
                    char c = buf[i];

                    // Filter out non-printable bytes except CR/LF
                    // Filtrează octeții neimprimabili, cu excepția CR/LF
                    if ((c >= 32 && c <= 126) || c == '\r' || c == '\n')
                    {
                        if (c == '\r' || c == '\n')
                        {
                            if (line_pos > 0)
                            {
                                line[line_pos] = '\0';
                                printf("📨 %s\n", line);
                                parse_response(line);
                                line_pos = 0;
                            }
                        }
                        else if (line_pos < (int)sizeof(line) - 1)
                        {
                            line[line_pos++] = c;
                        }
                    }
                }
            }
        }

        // RSSI polling at intervals via parameter rssi_interval
        // polling RSSI la intervale regulate, date prin parametrul rssi_interval
        if (rssi_interval > 0)
        {
            time_t now = time(NULL);
            if (now - last_rssi >= rssi_interval)
            {
                last_rssi = now;
                send_command(fd, "RSSI?\r\n");
            }
        }
    }

    printf(CLR_YELLOW _("\n🛑 Monitor stopped", "\n🛑 Monitorizarea a fost oprită.") CLR_RESET "\n");
}

/**
 * @brief Main function
 * @brief Funcția principală
 * @param argc
 * @param argv
 * @return int 0 on success, 1 on failure
 * @return int 0 la succes, 1 la eșec
 * @version 1.0.2
 */
int main(int argc, char *argv[])
{
    const char *device = DEFAULT_PORT;
    int baudrate = 9600, databits = 8, stopbits = 1, sql = 3;
    char parity = 'N';
    int narrow = 0, monitor = 0, info = 0, rssi_interval = 0, tail = 1;
    double tx = 0, rx = 0;
    float tx_tone = 0.0;
    float rx_tone = 0.0;
    char command[256] = {0};

    static struct option long_opts[] = {
        {"port", required_argument, 0, 'p'},
        {"device", required_argument, 0, 'd'},
        {"sql", required_argument, 0, 'q'},
        {"freq", required_argument, 0, 'f'},
        {"txtone", required_argument, 0, 't'},
        {"rxtone", required_argument, 0, 'r'},
        {"tail", required_argument, 0, 'a'},
        {"narrow", no_argument, 0, 'n'},
        {"volume", required_argument, 0, 'v'},
        {"monitor", no_argument, 0, 'm'},
        {"info", no_argument, 0, 'i'},
        {"rssi", required_argument, 0, 's'}, // changed from r
        {"version", no_argument, 0, 'u'},    // changed from v
        {"help", no_argument, 0, 'h'},
        {0, 0, 0, 0}};

    int opt;
    while ((opt = getopt_long(argc, argv, "p:d:q:f:t:r:a:nv:mis:u:h", long_opts, NULL)) != -1)
    {
        switch (opt)
        {
        case 'p':
            device = optarg;
            break;
        case 'd':
            device = optarg;
            break;
        case 'q':
            // Squelch level
            // int 0-8
            sql = atoi(optarg);
            break;
        case 'f':
            if (parse_freqs(optarg, &tx, &rx) != 0)
            {
                fprintf(stderr, CLR_RED _("Invalid frequency format. Use 145.500 or 145.500,145.600.",
                                          "Formatul frecvenței este incorect. Folosiți 145.500 sau 145.500,145.600 etc.") CLR_RESET "\n");
                return 1;
            }
            break;
        case 'a':
            snprintf(command, sizeof(command), "AT+SETTAIL=%d", atoi(optarg));
            break;
        case 'n':
            narrow = 1;
            break;
        case 'v':
            snprintf(command, sizeof(command), "AT+DMOSETVOLUME=%d\r\n", atoi(optarg));
            break;
        case 'm':
            monitor = 1;
            break;
        case 'i':
            info = 1;
            break;
        case 's':
            rssi_interval = atoi(optarg);
            break;
        case 'u':
            print_version();
            return 0;
        case 't':
            // CTCSS TX tone
            // float
            tx_tone = atof(optarg);
            break;
        case 'r':
            // CTCSS RX tone
            // float
            rx_tone = atof(optarg);
            break;
        case 'h':
            print_help(argv[0]);
            return 0;
        default:
            print_help(argv[0]);
            return 1;
        }
    }

    printf(_("🔧 Opening %s @ %d baud (8N1)\n", "🔧 Deschid %s cu baud de %d (8N1)\n"), device, baudrate);
    int fd = init_serial(device, baudrate, databits, stopbits, parity);
    if (fd < 0)
        return 1;

    delay_ms(500);

    printf(_("🔌 Sending AT+DMOCONNECT...\n", "🔌 Trimit AT+DMOCONNECT...\n"));
    if (send_command(fd, "AT+DMOCONNECT\r\n") != 0)
    {
        printf(CLR_RED _("❌ Could not connect to SA818. Check TX/RX wiring.",
                         "❌ Nu m-am putut conecta la SA818. Verifică conexiunea TX/RX.") CLR_RESET "\n");
        close(fd);
        return 1;
    }

    delay_ms(500);

    if (info)
        snprintf(command, sizeof(command), "AT+VERSION\r\n");

    if (tx > 0)
    {
        double tx_freq = tx;
        double rx_freq = tx;

        if (!check_band(&tx_freq, &rx_freq))
        {
            printf(CLR_RED _("❌ Aborting configuration — out-of-band frequencies.\n",
                                "❌ Frecvențe din afara benzii de radioamatori. Configurare anulată.\n") CLR_RESET);
            close(fd);
            return 1;
        }

        int tx_code = get_ctcss_code(tx_tone); // Convert 100.0 Hz -> 1000
        int rx_code = get_ctcss_code(rx_tone);
        snprintf(command, sizeof(command),
                 "AT+DMOSETGROUP=%d,%.4f,%.4f,%04d,%d,%04d\r\n",
                 narrow ? 0 : 1, tx, rx, tx_code, sql, rx_code);
    }

    if (command[0])
        send_command(fd, command);

    if (monitor)
        monitor_serial(fd, rssi_interval);

    if (tail)
        snprintf(command, sizeof(command), "AT+SETTAIL=1\r\n");

    

    close(fd);
    return 0;
} // end of main
// end of file
