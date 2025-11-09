#include <stdio.h>
#include <string.h>
#include "frequency.h"

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