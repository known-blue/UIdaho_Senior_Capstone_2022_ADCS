#include "comm.h"

/**
 * @brief
 * Zeroes out a packet that will receive a command from the satellite. Used to
 * ensure the program doesn't attempt to read uninitialized data.
 * 
 * @param[out] tes  Command packet - passed by reference
 */
void clearTEScommand(TEScommand *tes)
{
    int i;
    for (i = 0; i < COMMAND_LEN; i++)
    {
        tes->data[i] = 0;
    }
}

/**
 * @brief
 * Zeroes out a packet that will be filled with data collected by the ADCS. Used
 * to ensure the program doesn't attempt to read uninitialized data.
 * 
 * @param[out] adcs  Data packet - passed by reference
 */
void clearADCSdata(ADCSdata *adcs)
{
    int i;
    for (i = 0; i < PACKET_LEN; i++)
    {
        adcs->data[i] = 0;
    }
}

/**
 * @brief
 * Converts a floating-point number to a fixed-point number with 5 bits for the
 * integer part and 3 bits for the fraction part. Resulting data cannot be
 * properly interpreted until converted back into a float.
 * 
 * @param[in] f  Float to convert
 * 
 * @return Fixed-point conversion
 */
fixed5_3_t floatToFixed(float f)
{
    fixed5_3_t fix;
    fix = (uint8_t)(f * (1 << 3));
    return fix;
}

/**
 * @brief
 * Converts a fixed-point number with 5 integer bits and 3 fraction bits to a
 * floating-point number. Resulting data can be properly interpreted with no
 * extra conversions.
 * 
 * @param[in] fix  Fixed-point number to convert
 * 
 * @return Float conversion
 */
float fixedToFloat(fixed5_3_t fix)
{
    float f;
    f = ((float)fix) / (1 << 3);
    return f;
}