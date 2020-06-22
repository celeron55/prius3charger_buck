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

#pragma once
#include "filtering.h"

template<const int PIN, const int NUM_SAMPLES, const int TRIGGER_LIMIT>
struct SoftwareDebouncePin
{
	DigitalNoiseFilter<TRIGGER_LIMIT, TRIGGER_LIMIT> filter;

	SoftwareDebouncePin()
	{
		(void)read();
	}

	// Reads value from hardware and returns it filtered
	bool read()
	{
		for(uint8_t i=0; i<NUM_SAMPLES; i++){
			filter.feed(digitalRead(PIN));
		}
		return filter.get();
	}

	// Gets last value without reading it from hardware
	bool get()
	{
		return filter.get();
	}
};

template<const int PIN, const int NUM_SAMPLES, const int TRIGGER_LIMIT>
struct SoftwareDebounceAnalogPin
{
	DigitalNoiseFilter<TRIGGER_LIMIT, TRIGGER_LIMIT> filter;

	SoftwareDebounceAnalogPin()
	{
		(void)read();
	}

	// Reads value from hardware and returns it filtered
	bool read()
	{
		for(uint8_t i=0; i<NUM_SAMPLES; i++){
			filter.feed(analogRead(PIN) >= 300);
		}
		return filter.get();
	}

	// Gets last value without reading it from hardware
	bool get()
	{
		return filter.get();
	}
};
