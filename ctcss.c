#include <stdio.h>
#include "ctcss.h"

/**
 * @brief CTCSS tone mapping to codes / Mapare coduri CTCSS la frecvențe
 */
typedef struct
{
    float freq; // CTCSS frequency in Hz
    int code;   // SA818 numeric code (1–50)
} CTCSS_Map;

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