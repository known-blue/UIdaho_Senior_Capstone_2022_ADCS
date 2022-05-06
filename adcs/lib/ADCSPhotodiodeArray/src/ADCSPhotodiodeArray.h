#ifndef ADCSPHOTODIODEARRAY_H
#define ADCSPHOTODIODEARRAY_H

#include <Arduino.h>

/*
 * Truth table for the multiplexer that the 6 photodiodes will be hooked up to.
 *  
 * INPUTS
 * | A | B | C | CHANNEL | 
 * -----------------------
 * | L | L | L |    0    | X+
 * | H | L | L |    1    | X-
 * | L | H | L |    2    | Y+
 * | H | H | L |    3    | Y-
 * | L | L | H |    4    | Z+
 * | H | L | H |    5    | Z-
 * 
 */
enum PhotoCoordinate {X_POS=0, X_NEG=1, Y_POS=2, Y_NEG=3, Z_POS=4, Z_NEG=5};

class ADCSPhotodiodeArray {
private:
	uint8_t _input, _a, _b, _c;

public:
	ADCSPhotodiodeArray(uint8_t analog_input, uint8_t a, uint8_t b, uint8_t c);
	void init(void);
	float read(uint8_t channel); // read a scaled voltage from channel
};

#endif
