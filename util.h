/*
Part of prius3charger_buck
Copyright (c) 2020 Perttu "celeron55" Ahola

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <Arduino.h>

#define NUM_ELEMS(x) (sizeof (x) / sizeof (x)[0])

static unsigned long timestamp_age(unsigned long timestamp_ms)
{
	return millis() - timestamp_ms;
}

static bool timestamp_younger_than(unsigned long timestamp_ms, unsigned long max_age)
{
	// Timestamp is assumed to be initialized to 0. This means that a timestamp
	// of 0 is infinitely old.
	if(timestamp_ms == 0)
		return false;
	return timestamp_age(timestamp_ms) < max_age;
}

static int8_t fahrenheit_to_celsius(uint16_t fahrenheit)
{
	int16_t result = ((int16_t)fahrenheit - 32) * 5 / 9;
	if(result < -128)
		return -128;
	if(result > 127)
		return 127;
	return result;
}

static bool ENM_compare_and_update(unsigned long &t0, const unsigned long &interval)
{
	bool trigger_now = timestamp_age(t0) >= interval;
	if(trigger_now)
		t0 = millis();
	return trigger_now;
}

#define EVERY_N_MILLISECONDS(ms) for(static unsigned long t0 = 0; ENM_compare_and_update(t0, ms); )

#define REPORT_BOOL(var) \
	{\
		static bool reported_value = false;\
		if(var != reported_value || console_report_all_values){\
			log_print_timestamp();\
			CONSOLE.print(F(">> "#var" = "));\
			if(var)\
				CONSOLE.println(F("TRUE"));\
			else\
				CONSOLE.println(F("FALSE"));\
			reported_value = var;\
		}\
	}

#define REPORT_UINT8(var) \
	{\
		static uint8_t reported_value = 0;\
		if(var != reported_value){\
			log_print_timestamp();\
			CONSOLE.print(F(">> "#var" = "));\
			CONSOLE.println(var);\
			reported_value = var;\
		}\
	}

#define REPORT_UINT16(var) \
	{\
		static uint16_t reported_value = 0;\
		if(var != reported_value || console_report_all_values){\
			log_print_timestamp();\
			CONSOLE.print(F(">> "#var" = "));\
			CONSOLE.println(var);\
			reported_value = var;\
		}\
	}

#define REPORT_INT16(var) \
	{\
		static int16_t reported_value = 0;\
		if(var != reported_value || console_report_all_values){\
			log_print_timestamp();\
			CONSOLE.print(F(">> "#var" = "));\
			CONSOLE.println(var);\
			reported_value = var;\
		}\
	}

#define REPORT_UINT8_HYS(var, hys) \
	{\
		static uint8_t reported_value = 0;\
		if(abs(((int16_t)var - (int16_t)reported_value)) > (hys) || console_report_all_values){\
			log_print_timestamp();\
			CONSOLE.print(F(">> "#var" = "));\
			CONSOLE.println(var);\
			reported_value = var;\
		}\
	}

#define REPORT_UINT16_HYS(var, hys) \
	{\
		static uint16_t reported_value = 0;\
		if(abs(((int32_t)var - (int32_t)reported_value)) > (hys) || console_report_all_values){\
			log_print_timestamp();\
			CONSOLE.print(F(">> "#var" = "));\
			CONSOLE.println(var);\
			reported_value = var;\
		}\
	}

#define REPORT_INT16_HYS(var, hys) \
	{\
		static int16_t reported_value = 0;\
		if(abs((int16_t)((int32_t)var - (int32_t)reported_value)) > (hys) || console_report_all_values){\
			log_print_timestamp();\
			CONSOLE.print(F(">> "#var" = "));\
			CONSOLE.println(var);\
			reported_value = var;\
		}\
	}

#define REPORT_UINT16_FORMAT(var, hys, mul, unit) \
	{\
		static uint16_t reported_value = 0;\
		if(abs((int16_t)((int32_t)var - (int32_t)reported_value)) > (hys) || console_report_all_values){\
			log_print_timestamp();\
			CONSOLE.print(F(">> "#var" = "));\
			CONSOLE.print((float)var * mul);\
			CONSOLE.println(F(unit));\
			reported_value = var;\
		}\
	}

#define REPORT_INT16_FORMAT(var, hys, mul, unit) \
	{\
		static int16_t reported_value = 0;\
		if(abs(var - reported_value) > (hys) || console_report_all_values){\
			log_print_timestamp();\
			CONSOLE.print(F(">> "#var" = "));\
			CONSOLE.print((float)var * mul);\
			CONSOLE.println(F(unit));\
			reported_value = var;\
		}\
	}

#define REPORT_ENUM(var, names) \
	{\
		static uint8_t reported_value = 0;\
		if(var != reported_value || console_report_all_values){\
			log_print_timestamp();\
			CONSOLE.print(F(">> "#var" = "));\
			CONSOLE.println(names[var]);\
			reported_value = var;\
		}\
	}

#define REPORT_ENUM_PROGMEM(var, names) \
	{\
		static uint8_t reported_value = 0;\
		if(var != reported_value || console_report_all_values){\
			log_print_timestamp();\
			CONSOLE.print(F(">> "#var" = "));\
			CONSOLE.println((__FlashStringHelper*)pgm_read_word(&names[var]));\
			reported_value = var;\
		}\
	}

#define REPORT_UINT16_BITMAP(var, num_bits, names) \
	{\
		static uint16_t reported_value = 0;\
		if(var != reported_value){\
			log_print_timestamp();\
			CONSOLE.print(F(">> "#var" = "));\
			for(uint16_t i=0; i<num_bits; i++){\
				if(var & bit(i)){\
					CONSOLE.print(names[i]);\
					CONSOLE.print(" ");\
				}\
			}\
			CONSOLE.println();\
			reported_value = var;\
		}\
	}


static uint16_t limit_uint16(uint16_t v, uint16_t min, uint16_t max)
{
	if(v < min)
		return min;
	if(v > max)
		return max;
	return v;
}

static int16_t limit_int16(int16_t v, int16_t min, int16_t max)
{
	if(v < min)
		return min;
	if(v > max)
		return max;
	return v;
}

static int32_t limit_int32(int32_t v, int32_t min, int32_t max)
{
	if(v < min)
		return min;
	if(v > max)
		return max;
	return v;
}
