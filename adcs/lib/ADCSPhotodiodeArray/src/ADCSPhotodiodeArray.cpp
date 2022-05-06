#include "ADCSPhotodiodeArray.h"

/*
 * Configure the pins attached to the multiplexer (a-c) as outputs and the pin on analog input as an input.
 */
ADCSPhotodiodeArray::ADCSPhotodiodeArray(uint8_t analog_input, uint8_t a, uint8_t b, uint8_t c)
{
	_input = analog_input;
	_a = a;
	_b = b;
	_c = c;
}

void ADCSPhotodiodeArray::init(void)
{
	// configure digital pins
	pinMode(_a, OUTPUT);
	digitalWrite(_a, LOW);

	pinMode(_b, OUTPUT);
	digitalWrite(_b, LOW);

	pinMode(_c, OUTPUT);
	digitalWrite(_c, LOW);

	// set ADC resolution to 12 bits
	analogReadResolution(12);

	// configure analog pins
	pinMode(_input, INPUT);
}

/*
 * Read the value measured on one of the 6 multiplexer channels.
 *
 * @param[in]  channel  The channel/coordinate related sensor. Can be PhotodiodeCoordinate from ADCSPhotodiodeArray.h.
 *
 * @return     Scaled voltage between 0 and 3.3V
 */
float ADCSPhotodiodeArray::read(uint8_t channel)
{
	int result;
	float converted_result;

	switch (channel)
	{
	case 0:
		digitalWrite(_a, LOW);
		digitalWrite(_b, LOW);
		digitalWrite(_c, LOW);
		break;

	case 1:
		digitalWrite(_a, HIGH);
		digitalWrite(_b, LOW);
		digitalWrite(_c, LOW);
		break;

	case 2:
		digitalWrite(_a, LOW);
		digitalWrite(_b, HIGH);
		digitalWrite(_c, LOW);
		break;

	case 3:
		digitalWrite(_a, HIGH);
		digitalWrite(_b, HIGH);
		digitalWrite(_c, LOW);
		break;

	case 4:
		digitalWrite(_a, LOW);
		digitalWrite(_b, LOW);
		digitalWrite(_c, HIGH);
		break;

	case 5:
		digitalWrite(_a, LOW);
		digitalWrite(_b, LOW);
		digitalWrite(_c, HIGH);
		break;

	default:
		digitalWrite(_a, LOW);
		digitalWrite(_b, LOW);
		digitalWrite(_c, LOW);
	}

	result = analogRead(_input);
	converted_result = (float)result * (3.3f / 4096.0f);
	
	return converted_result;
}