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

static char log_format_buf[17];

static void log_print_timestamp()
{
	uint32_t t = millis();
	int ms = t % 1000;
	t /= 1000;
	int s = t % 60;
	t /= 60;
	int m = t % 60;
	t /= 60;
	int h = t;
	if(h == 0 && m == 0)
		snprintf(log_format_buf, sizeof log_format_buf, "%02i.%03is: ", s, ms);
	else if(h == 0)
		snprintf(log_format_buf, sizeof log_format_buf, "%02im%02i.%03is: ", m, s, ms);
	else
		snprintf(log_format_buf, sizeof log_format_buf, "%02ih%02im%02i.%03is: ", h, m, s, ms);
	CONSOLE.print(log_format_buf);
}

static void log_println(const char *line)
{
	log_print_timestamp();
	CONSOLE.println(line);
}

static void log_println_P(const char *line)
{
	log_print_timestamp();
	CONSOLE.println((__FlashStringHelper*)line);
}

#define log_println_f(x) log_println_P(PSTR(x))

