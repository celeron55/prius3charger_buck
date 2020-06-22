/*
MIT License

Copyright (c) 2017 Collin Kidder

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#pragma once
#include <Arduino.h>

//This structure presupposes little endian mode. If you use it on a big endian processor you're going to have a bad time.
typedef union {
  uint64_t value;
	struct {
		uint32_t low;
		uint32_t high;
	};
	struct {
		uint16_t s0;
		uint16_t s1;
		uint16_t s2;
		uint16_t s3;
  };
	uint8_t bytes[8];
	uint8_t byte[8]; //alternate name so you can omit the s if you feel it makes more sense
} BytesUnion;

typedef struct
{
  uint32_t id = 0;		// EID if ide set, SID otherwise
  uint32_t fid = 0;		// family ID
  uint8_t rtr = 0;		// Remote Transmission Request
  uint8_t priority = 0;	// Priority but only important for TX frames and then only for special uses.
  uint8_t extended = 0;	// Extended ID flag
  uint16_t time = 0;      // CAN timer value when mailbox message was received.
  uint8_t length = 0;		// Number of data bytes
  BytesUnion data;	// 64 bits - lots of ways to access it.
} CAN_FRAME;
