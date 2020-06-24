/*
prius3charger_buck
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

Usage:

Prius Gen 3 buck/boost converter charger controller - buck mode charging
- For atmega328 on PriusG3_V1c
- In Arduino IDE:
	1) Board selection: Arduino Nano
	2) Program bootloader using an ISP programmer like USBasp
	3) Connect USB-serial adapter to "FTDI 5V" connector (GND -- 5V RX TX RST)
	4) Upload and monitor using the serial connection at 57600 baud.
- Alternatively you can use MiniCore for atmega328 with the "no bootloader"
  option and program using any in-circuit programmer supported by avrdude

This program is designed to give excellent information on the serial console
about what it is doing and why it is doing it.

Notes about connections:
- AC_CON_CTRL / AC_CONTACTOR_SWITCH:
	- This activates a contactor to connect AC into MG1 terminals after
	  precharging. This is always required.
	- This can activate the battery side main contactor.
	  ALTERNATIVELY the battery side main contactor can be controlled by an
	  external controller based on CANbus messages or other signals.
	- This can activate a relay to feed 12V to the charger in parallel to the
	  ignition key separated by diodes. This way once charging has started the
	  ignition key can be turned off and the charger will shut down by itself
	  when charging is complete.
	- This can activate an indicator LED to show that the charger is active
- AC_PRECH_CTRL / AC_PRECHARGE_SWITCH:
	- This can activate a contactor to connect AC into MG1 terminals via
	  precharge resistors.
	  ALTERNATIVELY this software can precharge the MG/input side by boosting
	  from the battery side.
	- This can activate the battery side precharge contactor.
	  ALTERNATIVELY the battery side precharge can be initiated by an external
	  controller based on CANbus messages or other signals.
	- This can activate an indicator LED to show that the charger is precharging
- HV_CON_CTRL / CONVERTER_SHORT_SWITCH:
	- This can activate a contactor to short out the buck/boost converter to
	  enable feeding more current to the motors than the converter inductor and
	  IGBT can carry. This is switched on when the AC contactor and AC precharge
	  switch outputs are inactive.

TODO: Maybe make output current and voltage values configurable in EEPROM. It's
	  mostly a waste of program space though and a risky if you accidentally
	  change them while monitoring.

Connections:
- A0: DCBUS1 (output side, lower voltage, toyota's battery voltage)
- A1: DCBUS2 (input side, higher voltage, MG1/2 voltage)
- A2: BOOST T1
- A3: BOOST T2
- A4: MG1 L1 current (10k/10k resistor divider)
- A5: MG1 L2 current (10k/10k resistor divider)
- A6: ACS758LCB-100U current sensor (not used in this version)
- A7: EVSE_PP
- 2/PD2: EVSE CP
- 3/PD3: MCP2515 INT
- 4/PD4: EVSE SW
- 5/PD5: AC_CON_CTRL / AC_CONTACTOR_SWITCH
- 6/PD6: HV_CON_CTRL / CONVERTER_SHORT_SWITCH
- 7/PD7: AC_PRECH_CTRL / AC_PRECHARGE_SWITCH
- 8/PB0: MCP2515 CS
- 9/PB1/OC1A: Boost low side switch (active high)
- 10/PB2/OC1B: Boost high side switch (active high)

*/
#include "config.h"
#include "log.h"
#include "util.h"
#include "command_accumulator.h"
#include "avgbuffer.h"
#include "software_debounce.h"
// NOTE: These library portions are included with the project for ease of use.
// NOTE: They are licensed separately under different open source licenses.
// NOTE: See the files for details.
#include "mcp_can.h"
#include "can_common.h"

// Charging parameters
#define OUTPUT_CURRENT_MAX_A 80
#define BATTERY_CHARGE_VOLTAGE 300

// Other behavior
#define EVSE_FORCE_INPUT_AMPS 0  // If 0, EVSE CP PWM is followed
#define EVSE_PWM_TIMEOUT_MS 50
#define MAX_PRECHARGE_MS 30000
#define PRECHARGE_BOOST_ENABLED true
#define PRECHARGE_BOOST_START_MS 0
#define PRECHARGE_BOOST_VOLTAGE 600  // European 3-phase rectifies to 600V
//#define PWM_FREQ 3900  // Works for sure, but is very loud
#define PWM_FREQ 10000  // Not much tested, but is much quieter

// CANbus
// Edit send_canbus_frames(), handle_canbus_frame() and init_system_can_filters() to do what you need
#define CANBUS_ENABLE true
#define CANBUS_SEND_INTERVAL_MS 500
#define CANBUS_TIMEOUT_MS 2000

// Advanced
// Prints diagnostics about whether PWM output is limited by voltage or current
#define REPORT_PWM_LIMITING_VALUE false
// European 3-phase rectifies to 600V. Our measured voltage likes to sag a lot.
#define RECTIFIED_AC_MINIMUM_VOLTAGE 350

// Absolute maximums
#define INPUT_CURRENT_MAX_A 32
#define INPUT_VOLTAGE_MAX_V 650  // Maximum of all Toyota inverters
#define OUTPUT_VOLTAGE_MAX_V 305  // Yaris inverter has 300V and 350V capacitors
#define BMS_MIN_CELL_MV_FOR_FAIL 2500
#define BMS_MAX_CELL_MV_FOR_FAIL 4220
#define BMS_MAX_TEMPREATURE_C_FOR_FAIL 45
#define BOOST_MAX_TEMPERATURE_C 70

// Some secondary values that can be automatically set
#define BATTERY_MINIMUM_VOLTAGE (BATTERY_CHARGE_VOLTAGE / 2)
#define AC_PRECHARGE_MINIMUM_VOLTAGE RECTIFIED_AC_MINIMUM_VOLTAGE

// Hardcoded tests
#define TEST_CONTACTORS false
#define TEST_BOOST false
#define TEST_BUCK false

// Scaling of analog inputs
// NOTE: DCBUS2 is buck high side = MG side = 3-phase AC input side
// NOTE: DCBUS1 is buck low side = battery side
#define DCBUS2_OFFSET_BITS 0
#define DCBUS2_V_PER_BIT 1.234
/*#define DCBUS1_OFFSET_BITS 0
#define DCBUS1_V_PER_BIT 0.438*/
// Not sure what's up with this, maybe my Prius gen3 inverter is a bit wonky
#define DCBUS1_OFFSET_BITS 74
#define DCBUS1_V_PER_BIT 0.560

// This is what it should be
#define MG1_CURRENT_A_PER_BIT 1.0
// This is what you need with wrong resistors on Yaris inverter
//#define MG1_CURRENT_A_PER_BIT 0.735

#define PWM_HANDLER_INTERVAL 2

#define DCBUS1_PIN               A0
#define DCBUS2_PIN               A1
#define BOOST_T1_PIN             A2
#define BOOST_T2_PIN             A3
#define MG1_L1_CURRENT_PIN       A4
#define MG1_L2_CURRENT_PIN       A5
#define EXTRA_CURRENT_SENSOR_PIN A6
#define EVSE_PP_PIN              A7
#define EVSE_CP_PIN               2
#define MCP2515_INT_PIN           3
#define EVSE_SW_PIN               4
#define AC_CONTACTOR_SWITCH_PIN   5
#define CONVERTER_SHORT_SWITCH_PIN 6
#define AC_PRECHARGE_SWITCH_PIN   7
#define MCP2515_CS_PIN            8
#define BOOST_LOW_SWITCH_PIN      9
#define BOOST_HIGH_SWITCH_PIN    10

// If non-zero, overrides everything except INPUT_CURRENT_MAX_A.
// If zero, EVSE limit is followed.
uint8_t force_ac_input_amps = EVSE_FORCE_INPUT_AMPS;

// TODO: when reading these in the main thread, disable interrupts, maybe?
volatile int16_t dcbus1_raw = 0;
volatile int16_t dcbus2_raw = 0;
volatile int16_t mg2l1_current_raw_calibrated_zero = 530;
volatile int16_t mg2l2_current_raw_calibrated_zero = 530;
volatile int16_t mg2l1_current_raw = 0;
volatile int16_t mg2l2_current_raw = 0;
volatile int16_t boost_t1_raw = 0;
volatile int16_t boost_t2_raw = 0;
volatile int16_t evse_pp_raw = 0;
volatile int16_t input_voltage_V = 0;
volatile int16_t output_voltage_V = 0;
volatile int16_t input_dc_current_Ax10 = 0;
volatile int16_t output_dc_current_Ax10 = 0;

volatile enum DisablePwmReason {
	DPR_PWM_ENABLED,
	DPR_DCBUS1_OVERVOLTAGE,
	DPR_DCBUS2_OVERVOLTAGE,
	DPR_WANTED_PWM_IS_ZERO,
	DPR_BOOST_OVER_TEMPERATURE,
	DPR_LOST_EVSE_PROXIMITY_PILOT,
	DPR_PULSE_DONE,

	DPR_COUNT
} disable_pwm = DPR_PWM_ENABLED;
const char *DisablePwmReason_STRINGS[DPR_COUNT] = {
	"PWM_ENABLED",
	"DCBUS1_OVERVOLTAGE",
	"DCBUS2_OVERVOLTAGE",
	"WANTED_PWM_IS_ZERO",
	"DPR_BOOST_OVER_TEMPERATURE",
	"DPR_LOST_EVSE_PROXIMITY_PILOT",
	"DPR_PULSE_DONE",
};

// Once succesfully calibrated, PWM interrupt will be enabled
bool current_sensor_zero_offsets_calibrated = false;

// Current is measured at the MG current sensors where AC is coming in. This is
// used to get a somewhat usable DC value from that.
// 10 extra samples
//AvgBuffer<int16_t, int32_t, PWM_FREQ/PWM_HANDLER_INTERVAL/7/50+10> input_current_avgbuf;
// 2 full waves
AvgBuffer<int16_t, int32_t, PWM_FREQ/PWM_HANDLER_INTERVAL/7/25+1> input_current_avgbuf;

int8_t boost_t1_c = 0;
int8_t boost_t2_c = 0;

volatile uint8_t interrupt_counter_for_mainloop = 0;
volatile bool mainloop_running = false;
volatile bool console_report_all_values = false;
CommandAccumulator<24> command_accumulator;

enum SwitchingMode {
	SM_NONE,
	SM_BUCK,
	SM_BOOST,
	SM_BOOST_SINGLE_PULSE,
};

// Switching control is based on these, each is a maximum.
volatile int16_t wanted_output_voltage = 0;
volatile int16_t wanted_output_current = 0;
volatile int16_t wanted_pwm = 0;
SwitchingMode wanted_switching_mode = SM_BUCK;

// Switching control state (don't touch from main program)
volatile int16_t current_pwm = 0;
volatile enum {BSPS_INIT, BSPS_PULSING, BSPS_PULSED} boost_single_pulse_state = BSPS_INIT;
SwitchingMode current_switching_mode = SM_BUCK;

// CANbus

MCP_CAN system_can(MCP2515_CS_PIN);

unsigned long canbus_last_receive_timestamp = 0;

struct CanbusStatus {
	bool permit_charge = false;
	bool main_contactor_closed = false;
	uint16_t pack_voltage_V = 0;
	uint16_t cell_voltage_min_mV = 0;
	uint16_t cell_voltage_max_mV = 5000;
	int8_t cell_temperature_min = -128;
	int8_t cell_temperature_max = 127;
	bool charge_completed = false;
	uint16_t max_charge_current_A = 0;
} canbus_status;

// Charger state machine and related stuff

enum ChargerState {
	CS_WAITING_START_TRIGGER,
	CS_WAITING_CANBUS,
	CS_WAITING_CHARGE_PERMISSION,
	CS_PRECHARGING,
	CS_CHARGING=8,
	CS_STOPPING_CHARGE,
	CS_DONE_CHARGING,
	CS_FAILED, // Keep output shut down and wait state reset by user action
	CS_ALLOW_MEASUREMENT_THEN_FAIL, // Wait 15s, then fail (DEBUG UTILITY)
	CS_COUNT
} charger_state = CS_WAITING_START_TRIGGER;

const char* const ChargerState_STRINGS[CS_COUNT] = {
	"CS_WAITING_START_TRIGGER",
	"CS_WAITING_CANBUS",
	"CS_WAITING_CHARGE_PERMISSION",
	"CS_PRECHARGING",
	"4", // Some room for future development
	"5",
	"6",
	"7",
	"CS_CHARGING",
	"CS_STOPPING_CHARGE",
	"CS_DONE_CHARGING",
	"CS_FAILED",
	"CS_ALLOW_MEASUREMENT_THEN_FAIL",
};

enum ChargerFailReason {
	CFR_NOT_FAILED,
	CFR_CANBUS_DEAD,
	CFR_BMS_NO_CHARGE_PERMIT,
	CFR_BMS_OVER_TEMPERATURE,
	CFR_BMS_OVER_VOLTAGE,
	CFR_BMS_UNDER_VOLTAGE,
	CFR_AC_PRECHARGE_FAILED,
	CFR_CUSTOM_MEASUREMENT_DELAY_ENDED,
	CFR_UNHANDLED_STATE,
	CFR_PRECHARGE_MINIMUM_VOLTAGE_NOT_SET,
	CFR_INPUT_VOLTAGE_TOO_LOW,
	CFR_INPUT_VOLTAGE_TOO_HIGH,
	CFR_LOST_EVSE_PROXIMITY_PILOT,
	CFR_PRECHARGE_VOLTAGE_THROUGH_THE_ROOF,

	CFR_COUNT
};

const char* const ChargerFailReason_STRINGS[CFR_COUNT] = {
	"CFR_NOT_FAILED",
	"CFR_CANBUS_DEAD",
	"CFR_BMS_NO_CHARGE_PERMIT",
	"CFR_BMS_OVER_TEMPERATURE",
	"CFR_BMS_OVER_VOLTAGE",
	"CFR_BMS_UNDER_VOLTAGE",
	"CFR_AC_PRECHARGE_FAILED",
	"CFR_CUSTOM_MEASUREMENT_DELAY_ENDED",
	"CFR_UNHANDLED_STATE",
	"CFR_PRECHARGE_MINIMUM_VOLTAGE_NOT_SET",
	"CFR_INPUT_VOLTAGE_TOO_LOW",
	"CFR_INPUT_VOLTAGE_TOO_HIGH",
	"CFR_LOST_EVSE_PROXIMITY_PILOT",
	"CFR_PRECHARGE_VOLTAGE_THROUGH_THE_ROOF",
};

struct ChargerStatus
{
	ChargerFailReason fail_reason = CFR_NOT_FAILED;
	unsigned long no_start_condition_timestamp = 0;
	unsigned long start_timestamp = 0;
	unsigned long precharge_start_timestamp = 0;
	unsigned long stopping_charge_start_timestamp = 0;
	unsigned long fail_timestamp = 0;
	unsigned long allow_measurement_start_timestamp = 0;
	int16_t precharge_last_input_voltage = 0;
	int16_t precharge_last_battery_voltage = 0;
	bool battery_side_looks_precharged = false;
} charger;


// EVSE CP PWM measurement
bool evse_pwm_enough_pulses_received = false;
volatile unsigned long evse_pwm_last_rise_timestamp_us = 0;
volatile unsigned long evse_pwm_last_valid_value_timestamp = 0;
AvgBuffer<int8_t, int16_t, 50> evse_allowed_amps_avgbuf;
volatile uint16_t evse_pwm_pulse_counter = 0;

// EVSE final converted value from CP PWM
uint8_t evse_allowed_amps = 0;

// EVSE final converted value from PP resistance (0 = no cable)
uint8_t evse_pp_cable_rating_a = 0;

unsigned long inductor_short_switch_closed_timestamp = 0;
unsigned long ac_contactor_closed_timestamp = 0;

volatile enum LimitingValue {
	LV_NOTHING,
	LV_SOMETHING,
	LV_OUTPUT_CURRENT_SLOW,
	LV_INPUT_CURRENT_SLOW,
	LV_OUTPUT_VOLTAGE_SLOW,
	LV_OUTPUT_CURRENT,
	LV_INPUT_CURRENT,
	LV_OUTPUT_VOLTAGE,
	LV_OUTPUT_CURRENT_FAST,
	LV_INPUT_CURRENT_FAST,
	LV_OUTPUT_VOLTAGE_FAST,

	LV_COUNT,
} limiting_value = LV_NOTHING;

const char* const LimitingValue_STRINGS[LV_COUNT] = {
	"LV_NOTHING",
	"LV_SOMETHING",
	"LV_OUTPUT_CURRENT_SLOW",
	"LV_INPUT_CURRENT_SLOW",
	"LV_OUTPUT_VOLTAGE_SLOW",
	"LV_OUTPUT_CURRENT",
	"LV_INPUT_CURRENT",
	"LV_OUTPUT_VOLTAGE",
	"LV_OUTPUT_CURRENT_FAST",
	"LV_INPUT_CURRENT_FAST",
	"LV_OUTPUT_VOLTAGE_FAST",
};


void setup()
{
	pinMode(DCBUS1_PIN, INPUT);
	pinMode(DCBUS2_PIN, INPUT);
	pinMode(BOOST_T1_PIN, INPUT);
	pinMode(BOOST_T2_PIN, INPUT);
	pinMode(MG1_L1_CURRENT_PIN, INPUT);
	pinMode(MG1_L2_CURRENT_PIN, INPUT);
	pinMode(EXTRA_CURRENT_SENSOR_PIN, INPUT);
	pinMode(EVSE_PP_PIN, INPUT);
	pinMode(EVSE_CP_PIN, INPUT);
	pinMode(MCP2515_INT_PIN, INPUT);
	pinMode(EVSE_SW_PIN, OUTPUT);
	pinMode(AC_CONTACTOR_SWITCH_PIN, OUTPUT);
	pinMode(CONVERTER_SHORT_SWITCH_PIN, OUTPUT);
	pinMode(AC_PRECHARGE_SWITCH_PIN, OUTPUT);
	pinMode(MCP2515_CS_PIN, OUTPUT);
	pinMode(BOOST_LOW_SWITCH_PIN, OUTPUT);
	pinMode(BOOST_HIGH_SWITCH_PIN, OUTPUT);

	// Wait for programming
	delay(2000);

	// We're using 57600 baud instead of 115200 because we can't process serial
	// data that fast with our monster interrupt routine running in sync with
	// PWM generation.
	Serial.begin(57600);

	log_println_f("-!- prius3charger_buck");

#if TEST_CONTACTORS == true
	for(;;){
		log_println_f("DEBUG: AC_PRECHARGE_SWITCH_PIN on");
		digitalWrite(AC_PRECHARGE_SWITCH_PIN, HIGH);
		delay(1900);
		log_println_f("DEBUG: AC_CONTACTOR_SWITCH_PIN on");
		digitalWrite(AC_CONTACTOR_SWITCH_PIN, HIGH);
		delay(100);
		log_println_f("DEBUG: AC_PRECHARGE_SWITCH_PIN off");
		digitalWrite(AC_PRECHARGE_SWITCH_PIN, LOW);
		delay(2000);
		log_println_f("DEBUG: AC_CONTACTOR_SWITCH_PIN off");
		digitalWrite(AC_CONTACTOR_SWITCH_PIN, LOW);
		delay(2000);
	}
#endif

	for(uint8_t i=0; i<10 && !current_sensor_zero_offsets_calibrated; i++){
		delay(100);
		calibrate_current_sensor_zero_offsets_if_needed();
	}

	SPI.begin();

	init_system_can();

	// EVSE PWM interrupt
	attachInterrupt(digitalPinToInterrupt(EVSE_CP_PIN), evse_cp_pwm_handler, CHANGE);

	mainloop_running = true;
}

void calibrate_current_sensor_zero_offsets_if_needed()
{
	if(current_sensor_zero_offsets_calibrated)
		return;

	// Calibrate current sensor zero offsets before PWM output is initialized
	Serial.println(F("Calibrating current sensor zero offsets..."));
	set_pwm_inactive();

	int16_t il1_0 = analogRead(MG1_L1_CURRENT_PIN);
	int16_t il2_0 = analogRead(MG1_L2_CURRENT_PIN);
	int16_t il1_1 = 0;
	int16_t il2_1 = 0;
	const uint8_t require_stable_count = 20;
	uint8_t stable_count = 0;
	while(stable_count < require_stable_count){
		delay(20);
		il1_1 = analogRead(MG1_L1_CURRENT_PIN);
		il2_1 = analogRead(MG1_L2_CURRENT_PIN);
		if(abs(il1_1 - il1_0) >= 3 || abs(il2_1 - il2_0) >= 3){
			break;
		}
		stable_count++;
	}
	if(il1_1 < 460 || il1_1 > 570 || il2_1 < 460 || il2_1 > 570){
		Serial.print(F("Not accepting current calibration values: "));
		Serial.print(il1_1);
		Serial.print(F(", "));
		Serial.print(il2_1);
		Serial.print(" (out of range)");
		Serial.println();
		return;
	}
	if(stable_count < require_stable_count){
		Serial.print(F("Not accepting current calibration values: "));
		Serial.print(il1_1);
		Serial.print(F(", "));
		Serial.print(il2_1);
		Serial.print(" (unstable)");
		Serial.println();
		return;
	}
	mg2l1_current_raw_calibrated_zero = il1_1;
	mg2l2_current_raw_calibrated_zero = il2_1;
	log_print_timestamp();
	Serial.print(F("Calibrated zero offsets: IL1,2: "));
	Serial.print(mg2l1_current_raw_calibrated_zero);
	Serial.print(F(", "));
	Serial.print(mg2l2_current_raw_calibrated_zero);
	Serial.println();

	// PWM output
	// A: Set on compare match, phase and frequency correct, clk/1
	// B: Clear on compare match, phase and frequency correct, clk/1
	TCCR1A = _BV(COM1A1) | _BV(COM1A0) | _BV(COM1B1);
	TCCR1B = _BV(WGM13) | _BV(CS10);
#define PWM_MAX (uint16_t)((F_CPU / 2.0) / (float)PWM_FREQ)
	ICR1 = PWM_MAX;
	// Interrupt at TOP
	TIMSK1 |= _BV(ICIE1);

	// Set initial PWM to zero
	set_pwm_inactive();

	current_sensor_zero_offsets_calibrated = true;
}

void loop()
{
#if TEST_BOOST == true
	dcbus2_raw = analogRead(DCBUS2_PIN) - DCBUS2_OFFSET_BITS;
	input_voltage_V = ((int32_t)dcbus2_raw * (int32_t)(DCBUS2_V_PER_BIT*1000)) / 1000;
	dcbus1_raw = analogRead(DCBUS1_PIN) - DCBUS1_OFFSET_BITS;
	output_voltage_V = ((int32_t)dcbus1_raw * (int32_t)(DCBUS1_V_PER_BIT*1000)) / 1000;
	EVERY_N_MILLISECONDS(200){
		Serial.print(output_voltage_V);
		Serial.print(" ");
		Serial.println(input_voltage_V);
	}
	if(input_voltage_V < 400 && ((millis()/1000)&1)==0){
		set_pwm_boost_active(ICR1 * 0.01);
	} else {
		set_pwm_inactive();
	}
	return;
#endif
#if TEST_BUCK == true
	dcbus2_raw = analogRead(DCBUS2_PIN) - DCBUS2_OFFSET_BITS;
	input_voltage_V = ((int32_t)dcbus2_raw * (int32_t)(DCBUS2_V_PER_BIT*1000)) / 1000;
	dcbus1_raw = analogRead(DCBUS1_PIN) - DCBUS1_OFFSET_BITS;
	output_voltage_V = ((int32_t)dcbus1_raw * (int32_t)(DCBUS1_V_PER_BIT*1000)) / 1000;
	EVERY_N_MILLISECONDS(200){
		Serial.print(output_voltage_V);
		Serial.print(" ");
		Serial.println(input_voltage_V);
	}
	if(output_voltage_V < 20 && ((millis()/1000)&1)==0){
		set_pwm_buck_active(ICR1 * 0.01);
	} else {
		set_pwm_inactive();
	}
	return;
#endif

	EVERY_N_MILLISECONDS(1000){
		calibrate_current_sensor_zero_offsets_if_needed();
	}

	read_console_serial();

	read_canbus_frames();

	EVERY_N_MILLISECONDS(100){
		apply_canbus_timeouts();
	}

	convert_evse();

	handle_evse_pwm_timeout();

	handle_charger_state();

	control_inductor_short_switch();

	avoid_explosions();

	EVERY_N_MILLISECONDS(CANBUS_SEND_INTERVAL_MS){
		send_canbus_frames();
	}

	EVERY_N_MILLISECONDS(1000){
		convert_temperatures();
	}
	EVERY_N_MILLISECONDS(200){
		report_status_on_console();
	}
	EVERY_N_MILLISECONDS(5){
		interrupt_counter_for_mainloop = 0;
	}
}

void charger_fail(ChargerFailReason fail_reason)
{
	charger_state = CS_FAILED;

	if(charger.fail_reason == CFR_NOT_FAILED){
		charger.fail_reason = fail_reason;
		charger.fail_timestamp = millis();
	}

	set_wanted_output_V_A_pwm(0, 0, 0, SM_NONE);
	digitalWrite(AC_CONTACTOR_SWITCH_PIN, LOW);
	digitalWrite(AC_PRECHARGE_SWITCH_PIN, LOW);
	digitalWrite(EVSE_SW_PIN, LOW);

	log_print_timestamp();
	CONSOLE.print("Charger failed: \"");
	CONSOLE.print(ChargerFailReason_STRINGS[charger.fail_reason]);
	CONSOLE.println("\"");
}

// Returns true if failed
bool fail_if_charging_unsafe()
{
	if(CANBUS_ENABLE){
		if(!canbus_alive()){
			charger_fail(CFR_CANBUS_DEAD);
			return true;
		}
		if(!canbus_status.permit_charge){
			charger_fail(CFR_BMS_NO_CHARGE_PERMIT);
			return true;
		}
		if(canbus_status.cell_temperature_max > BMS_MAX_TEMPREATURE_C_FOR_FAIL){
			charger_fail(CFR_BMS_OVER_TEMPERATURE);
			return true;
		}
		if(canbus_status.cell_voltage_min_mV < BMS_MIN_CELL_MV_FOR_FAIL){
			charger_fail(CFR_BMS_UNDER_VOLTAGE);
			return true;
		}
		if(canbus_status.cell_voltage_max_mV > BMS_MAX_CELL_MV_FOR_FAIL){
			charger_fail(CFR_BMS_OVER_VOLTAGE);
			return true;
		}
	}
	return false;
}

void restore_initial_state()
{
	log_println_f("restore_initial_state()");
	if(charger_state != CS_FAILED && charger_state != CS_DONE_CHARGING){
		log_println_f("restore_initial_state(): Can't restore: Not failed or ended");
		return;
	}
	charger_state = CS_WAITING_START_TRIGGER;
	charger = ChargerStatus(); // Reset
}

void start_charging()
{
	log_println_f("start_charging()");

	if(charger_state != CS_WAITING_START_TRIGGER &&
			charger_state != CS_DONE_CHARGING){
		log_println_f("Can't start charge: Not at charge cycle start or done charging");
		return;
	}

	charger = ChargerStatus(); // Reset

	charger.start_timestamp = millis();

	set_wanted_output_V_A_pwm(0, 0, 0, SM_NONE);

	// Request power from charger
	digitalWrite(EVSE_SW_PIN, HIGH);

	// Open converter short switch to let us have different voltages between the
	// MG and battery rails
	digitalWrite(CONVERTER_SHORT_SWITCH_PIN, LOW);
	// Wait for a little bit to make sure AC contactor doesn't switch before
	// converter short switch releases
	delay(100);

	charger_state = CS_WAITING_CANBUS;
}

void stop_charging()
{
	if(charger_state == CS_FAILED){
		// Don't change failed state to anything else here as that can be unsafe.
		// Failed state can only be restored by power cycling or by
		// restore_initial_state().
		log_println_f("stop_charging(): Charger is in failed state.");
		return;
	}
	if(charger_state == CS_STOPPING_CHARGE){
		log_println_f("stop_charging(): Already stopped");
		return;
	}

	log_println_f("Stopping charger");

	charger_state = CS_STOPPING_CHARGE;
	charger.stopping_charge_start_timestamp = millis();
}

#define HANDLE_CHARGER_STATE(state, handler) \
		if(charger_state == (CS_##state)){ handler(); return; }

void handle_charger_state()
{
	// State modifiers
	if(charger_state != CS_FAILED){
		if(charger_state > CS_CHARGING && charger_state < CS_STOPPING_CHARGE){
			fail_if_charging_unsafe();
		}
	}

	// State handlers
	HANDLE_CHARGER_STATE(WAITING_START_TRIGGER, [&](){
		if(evse_pp_cable_rating_a > 0 && get_max_input_a() > 0 && current_sensor_zero_offsets_calibrated){
			// Wait until 2 seconds of continuous start conditions
			if(timestamp_age(charger.no_start_condition_timestamp) >= 2000){
				log_println_f("Charger start triggered");
				start_charging();
			} else {
				EVERY_N_MILLISECONDS(500){
					log_println_f("... Waiting stable starting condition for 2000ms");
				}
			}
			return;
		} else {
			charger.no_start_condition_timestamp = millis();
		}
		EVERY_N_MILLISECONDS(5000){
			if(evse_pp_cable_rating_a == 0){
				log_println_f("... Waiting for EVSE PP connection"
						" (pull it to ground if you don't have EVSE)");
			}
			if(get_max_input_a() == 0 && force_ac_input_amps == 0){
				log_println_f("... Waiting for EVSE CP PWM"
						" (set force_ac_input_amps if you don't have EVSE)");
			}
		}
	});
	HANDLE_CHARGER_STATE(WAITING_CANBUS, [&](){
		if(CANBUS_ENABLE){
			if(canbus_alive()){
				log_println_f("CANbus detected");
				charger_state = CS_WAITING_CHARGE_PERMISSION;
			}
			EVERY_N_MILLISECONDS(5000){
				log_println_f("... Waiting for CANbus");
			}
		} else {
			// Skip ahead, we don't need no CANbus!
			charger_state = CS_WAITING_CHARGE_PERMISSION;
		}
	});
	HANDLE_CHARGER_STATE(WAITING_CHARGE_PERMISSION, [&](){
		if((canbus_status.permit_charge || !CANBUS_ENABLE) && evse_pp_cable_rating_a > 0){
			if(CANBUS_ENABLE){
				log_println_f("BMS gives charge permission and cable proximity pilot is connected. Starting AC side precharge");
			} else {
				log_println_f("Cable proximity pilot is connected. Starting AC side precharge");
			}
			charger_state = CS_PRECHARGING;


			charger.precharge_start_timestamp = millis();
			charger.precharge_last_input_voltage = dcbus2_raw * DCBUS2_V_PER_BIT;

			report_status_on_console();

			log_print_timestamp();
			CONSOLE.print(F("Precharge starting at "));
			CONSOLE.print(charger.precharge_last_input_voltage);
			CONSOLE.println(" V");
			return;
		}
		EVERY_N_MILLISECONDS(5000){
			if(CANBUS_ENABLE){
				if(!canbus_status.permit_charge){
					log_println_f("... Waiting for BMS charge permission");
				}
			}
			if(evse_pp_cable_rating_a == 0){
				log_println_f("... Waiting for EVSE proximity pilot connection");
			}
		}
	});
	HANDLE_CHARGER_STATE(PRECHARGING, [&](){
		if(evse_pp_cable_rating_a == 0){
			charger_fail(CFR_LOST_EVSE_PROXIMITY_PILOT);
			return;
		}

		// Stop and cancel charging if any voltage goes through the roof
		if(input_voltage_V > INPUT_VOLTAGE_MAX_V ||
				output_voltage_V > OUTPUT_VOLTAGE_MAX_V){
			charger_fail(CFR_PRECHARGE_VOLTAGE_THROUGH_THE_ROOF);
			return;
		}

		// Delay closing the AC side precharge switch a bit.
		// This delay is useful if your precharge contactor doesn't quite work
		// at 12V but works at 14V, and your DC-DC converter is enabled at the
		// start of CS_PRECHARGING.
		// Ok yeah that's a bit silly for sure, but it happened to me!
		if(timestamp_age(charger.precharge_start_timestamp) >= 500){
			if(!digitalRead(AC_CONTACTOR_SWITCH_PIN)){
				digitalWrite(AC_PRECHARGE_SWITCH_PIN, HIGH);
			}
		}

		// Follow battery side precharge
		EVERY_N_MILLISECONDS(1000){
			if(!charger.battery_side_looks_precharged){
				if(
					(abs(output_voltage_V - charger.precharge_last_battery_voltage) <= 2 ||
							PRECHARGE_BOOST_ENABLED)
					&&
					output_voltage_V >= BATTERY_MINIMUM_VOLTAGE
				){
					log_print_timestamp();
					CONSOLE.print(F("-> Battery side precharge looks FINISHED at "));
					CONSOLE.print(output_voltage_V);
					CONSOLE.println("V");

					charger.battery_side_looks_precharged = true;
				} else {
					log_print_timestamp();
					CONSOLE.print(F("... Battery side precharging at "));
					CONSOLE.print(output_voltage_V);
					CONSOLE.println("V");

					charger.precharge_last_battery_voltage = output_voltage_V;
				}
			}

			if(CANBUS_ENABLE){
				if(!canbus_status.main_contactor_closed){
					charger.battery_side_looks_precharged = false;
				}
			}
		}

		// Follow AC side precharge
		EVERY_N_MILLISECONDS(1000){
			if(!digitalRead(AC_CONTACTOR_SWITCH_PIN)){
				if(
					abs(input_voltage_V - charger.precharge_last_input_voltage) <= 2 &&
					input_voltage_V >= AC_PRECHARGE_MINIMUM_VOLTAGE
				){
					uint16_t finish_voltage = input_voltage_V;

					digitalWrite(AC_CONTACTOR_SWITCH_PIN, HIGH);
					// This delay allows an NO AUX contact on the precharge
					// contactor to be paralleled with a resistor to form an
					// economizer for the main contactor.
					delay(100);
					digitalWrite(AC_PRECHARGE_SWITCH_PIN, LOW);

					log_print_timestamp();
					CONSOLE.print(F("-> AC side precharge FINISHED at "));
					CONSOLE.print(finish_voltage);
					CONSOLE.println("V, AC contactor CLOSED");
				} else {
					log_print_timestamp();
					CONSOLE.print(F("... AC side precharging at "));
					CONSOLE.print(input_voltage_V);
					CONSOLE.println("V");

					charger.precharge_last_input_voltage = input_voltage_V;
				}
			}
		}

		// Precharge finish condition
		if(digitalRead(AC_CONTACTOR_SWITCH_PIN) && charger.battery_side_looks_precharged){
			log_println_f("Input and battery side precharge done, now charging.");
			charger_state = CS_CHARGING;
			return;
		}

		// Precharge timeout
		if(timestamp_age(charger.precharge_start_timestamp) > MAX_PRECHARGE_MS){
			// Open precharge contactor
			digitalWrite(AC_PRECHARGE_SWITCH_PIN, LOW);
			// Stop boosting
			set_wanted_output_V_A_pwm(0, 0, 0, SM_NONE);
			set_pwm_inactive();
			// Report
			log_print_timestamp();
			CONSOLE.print(F("-> Precharge FAILED at "));
			CONSOLE.print(input_voltage_V);
			CONSOLE.print(F("V: Voltage not reaching target "));
			CONSOLE.print(AC_PRECHARGE_MINIMUM_VOLTAGE);
			CONSOLE.println("V");
			// Fail
			charger_fail(CFR_AC_PRECHARGE_FAILED);
			return;
		}

		// Use boosting to precharge in case there are no precharge resistors
		// configured.
		// This can be done only after the battery side has been precharged
		// first.
		if(PRECHARGE_BOOST_ENABLED &&
				timestamp_age(charger.precharge_start_timestamp) > PRECHARGE_BOOST_START_MS){
			if(!charger.battery_side_looks_precharged){
				EVERY_N_MILLISECONDS(5000){
					log_println_f("... Waiting battery side to be precharged before boosting");
				}
			}
			if(charger.battery_side_looks_precharged &&
					input_voltage_V < PRECHARGE_BOOST_VOLTAGE &&
					input_voltage_V < INPUT_VOLTAGE_MAX_V - 20){
				EVERY_N_MILLISECONDS(500){
					log_println_f("... Doing AC side precharge boost pulses");
				}
				// Make one boost pulse at a time
				set_wanted_output_V_A_pwm(PRECHARGE_BOOST_VOLTAGE, 1, ICR1*0.02,
						SM_BOOST_SINGLE_PULSE);
			} else {
				set_wanted_output_V_A_pwm(0, 0, 0, SM_NONE);
				set_pwm_inactive();
			}
		}

		// Report what we're waiting for
		if(!digitalRead(AC_CONTACTOR_SWITCH_PIN)){
			EVERY_N_MILLISECONDS(5000){
				log_println_f("... Doing AC side precharge");
			}
		}
		if(!charger.battery_side_looks_precharged){
			EVERY_N_MILLISECONDS(5000){
				log_println_f("... Doing battery side precharge");
			}
		}
	});
	HANDLE_CHARGER_STATE(CHARGING, [&](){
		if(evse_pp_cable_rating_a == 0){
			charger_fail(CFR_LOST_EVSE_PROXIMITY_PILOT);
			return;
		}

		check_and_react_if_high_power_input_failed();

		if(CANBUS_ENABLE){
			if(!canbus_status.permit_charge){
				report_status_on_console();
				log_println_f("BMS does not permit charging");
				charger_state = CS_STOPPING_CHARGE;
				charger.stopping_charge_start_timestamp = millis();
				return;
			}
		}

		// Stop charging if finished

		if(CANBUS_ENABLE){
			if(canbus_status.charge_completed){
				report_status_on_console();
				log_println_f("BMS reports charge completion");
				charger_state = CS_STOPPING_CHARGE;
				charger.stopping_charge_start_timestamp = millis();
				return;
			}
		}

		if(output_voltage_V >= BATTERY_CHARGE_VOLTAGE - 2 &&
				output_dc_current_Ax10 < 5){
			if(timestamp_younger_than(charger.start_timestamp, 30000)){
				EVERY_N_MILLISECONDS(5000){
					log_println_f("... Charging looks complete but continuing (ignoring BMS)");
				}
			} else {
				report_status_on_console();
				log_println_f("Charging looks complete, stopping (ignoring BMS)");
				charger_state = CS_STOPPING_CHARGE;
				charger.stopping_charge_start_timestamp = millis();
				return;
			}
		}

		// Update output according to vehicle requirements

		if(CANBUS_ENABLE){
			set_wanted_output_V_A_pwm(
					BATTERY_CHARGE_VOLTAGE,
					canbus_status.max_charge_current_A,
					PWM_MAX,
					SM_BUCK);
		} else {
			set_wanted_output_V_A_pwm(
					BATTERY_CHARGE_VOLTAGE,
					OUTPUT_CURRENT_MAX_A,
					PWM_MAX,
					SM_BUCK);
		}

		if(fail_if_charging_unsafe())
			return;
	});
	HANDLE_CHARGER_STATE(STOPPING_CHARGE, [&](){
		set_wanted_output_V_A_pwm(0, 0, 0, SM_NONE);

		if(timestamp_age(charger.stopping_charge_start_timestamp) >= 1000){
			charger_state = CS_DONE_CHARGING;
			return;
		}

		EVERY_N_MILLISECONDS(5000){
			log_println_f("... Stopping charge");
		}
	});
	HANDLE_CHARGER_STATE(DONE_CHARGING, [&](){
		set_wanted_output_V_A_pwm(0, 0, 0, SM_NONE);

		digitalWrite(AC_PRECHARGE_SWITCH_PIN, LOW);
		digitalWrite(AC_CONTACTOR_SWITCH_PIN, LOW);
		digitalWrite(EVSE_SW_PIN, LOW);

		EVERY_N_MILLISECONDS(5000){
			log_println_f("... Done charging");
		}
	});
	HANDLE_CHARGER_STATE(FAILED, [&](){
		set_wanted_output_V_A_pwm(0, 0, 0, SM_NONE);

		digitalWrite(AC_PRECHARGE_SWITCH_PIN, LOW);
		digitalWrite(AC_CONTACTOR_SWITCH_PIN, LOW);
		digitalWrite(EVSE_SW_PIN, LOW);

		EVERY_N_MILLISECONDS(5000){
			log_print_timestamp();
			CONSOLE.print("... In fault state: \"");
			CONSOLE.print(ChargerFailReason_STRINGS[charger.fail_reason]);
			CONSOLE.println("\"");
		}

		// Reset automatically from some states that don't seem too dangerous
		if(charger.fail_reason == CFR_LOST_EVSE_PROXIMITY_PILOT){
			if(timestamp_age(charger.fail_timestamp) >= 5000){
				log_println_f("Automatically resetting EVSE PP lost failure");
				restore_initial_state();
			}
		}
	});
	HANDLE_CHARGER_STATE(ALLOW_MEASUREMENT_THEN_FAIL, [&](){
		if(timestamp_age(charger.allow_measurement_start_timestamp) > 15000){
			charger_fail(CFR_CUSTOM_MEASUREMENT_DELAY_ENDED);
		}
		EVERY_N_MILLISECONDS(2000){
			log_println_f("... Allowing measurement for 15s");
		}
	});

	log_println_f("Unhandled charger state; falling back to stopping charge");
	charger_fail(CFR_UNHANDLED_STATE);
}

void control_inductor_short_switch()
{
	// Maintain some timestamps
	if(digitalRead(CONVERTER_SHORT_SWITCH_PIN)){
		inductor_short_switch_closed_timestamp = millis();
	}
	if(digitalRead(AC_CONTACTOR_SWITCH_PIN)){
		ac_contactor_closed_timestamp = millis();
	}

	// Close converter short switch if charging isn't happening, AC contactor has
	// been open for some time and the voltage difference is reasonable
	if((charger_state == CS_WAITING_START_TRIGGER || charger_state == CS_DONE_CHARGING ||
				charger_state == CS_FAILED) &&
			timestamp_age(ac_contactor_closed_timestamp) >= 500){
		if(!digitalRead(CONVERTER_SHORT_SWITCH_PIN)){
			if(abs(input_voltage_V - output_voltage_V) < 20){
				log_println_f("Closing converter short switch");
				digitalWrite(CONVERTER_SHORT_SWITCH_PIN, HIGH);
			} else {
				EVERY_N_MILLISECONDS(5000){
					log_println_f("Can't close converter short switch due to voltage "
							"difference. If you're using the feature, make sure to "
							"have a bleed down resistor in parallel with the "
							"converter short switch.");
				}
			}
		}
	} else {
		// Always if the AC main contactor is active, deactive the inductor
		// short switch, and tell the programmer they messed up. You can look up
		// if you find this in logs if your 3 phase breaker opened.
		if(digitalRead(CONVERTER_SHORT_SWITCH_PIN)){
			log_println_f("WARNING: CONVERTER_SHORT_SWITCH was active while AC "
					"was active. The short switch has now been deactivated.");
			digitalWrite(CONVERTER_SHORT_SWITCH_PIN, LOW);
		}
	}
}

void avoid_explosions()
{
	// Nah
}

void set_wanted_output_V_A_pwm(int16_t voltage_V, int16_t current_A, int16_t pwm,
		SwitchingMode switching_mode)
{
	cli();
	wanted_switching_mode = switching_mode;
	wanted_output_voltage = voltage_V;
	wanted_output_current = current_A;
	wanted_pwm = pwm;
	boost_single_pulse_state = BSPS_INIT;
	sei();
}

void check_and_react_if_high_power_input_failed()
{
	cli();
	int16_t voltage_now = dcbus2_raw * DCBUS2_V_PER_BIT;
	sei();

	EVERY_N_MILLISECONDS(1000){
		if(voltage_now < RECTIFIED_AC_MINIMUM_VOLTAGE){
			log_print_timestamp();
			CONSOLE.print(F("Input voltage dropped below the minimum voltage "));
			CONSOLE.print(RECTIFIED_AC_MINIMUM_VOLTAGE);
			CONSOLE.println(F(" V"));

			charger_fail(CFR_INPUT_VOLTAGE_TOO_LOW);
			return;
		}

		if(voltage_now > INPUT_VOLTAGE_MAX_V){
			log_print_timestamp();
			CONSOLE.print(F("Input voltage rose above the maximum voltage "));
			CONSOLE.print(INPUT_VOLTAGE_MAX_V);
			CONSOLE.println(F(" V"));

			charger_fail(CFR_INPUT_VOLTAGE_TOO_HIGH);
			return;
		}
	}
}

// Times are referenced to ICR1 value (ICR1 = 100%, 0 = 0%)
static void set_ontimes(uint16_t lowswitch_offtime, uint16_t highswitch_ontime)
{
	// OC1A: Low side
	OCR1A = lowswitch_offtime;
	// OC1B: High side
	OCR1B = highswitch_ontime;
}

// Sets high side switch PWM, and low side switch PWM based on it
static void set_pwm_buck_active(uint16_t highswitch_ontime)
{
	if(!current_sensor_zero_offsets_calibrated)
		return;
#if 0
	const float deadtime_ns = 2500;
	const uint16_t deadtime = (uint16_t)((float)deadtime_ns/(1.0/16e6*1e9));
	set_ontimes(highswitch_ontime + deadtime, highswitch_ontime);
#else
	set_ontimes(ICR1+1, highswitch_ontime);
#endif
}

// Sets low side switch PWM, and high side switch PWM based on it
static void set_pwm_boost_active(uint16_t lowswitch_ontime)
{
	if(!current_sensor_zero_offsets_calibrated)
		return;
	if(lowswitch_ontime > ICR1)
		lowswitch_ontime = ICR1;
	uint16_t lowswitch_offtime = ICR1 - lowswitch_ontime;
#if 0
	const float deadtime_ns = 10000;
	const uint16_t deadtime = (uint16_t)((float)deadtime_ns/(1.0/16e6*1e9));
	set_ontimes(lowswitch_offtime, lowswitch_offtime - deadtime);
#else
	set_ontimes(lowswitch_offtime, 0);
#endif
}

static void set_pwm_inactive()
{
	set_ontimes(ICR1, 0);
}

static int16_t get_max_input_a()
{
	int16_t max_input_a = [&]() -> int16_t {
		// If force_ac_input_amps is set, it overrides everything except
		// INPUT_CURRENT_MAX_A.
		if(force_ac_input_amps != 0)
			return force_ac_input_amps;
		// Otherwise use EVSE CP PWM limit
		// Derate slightly just to be sure to not potentially pull too much
		// current from a public charge point. That would be rude!
		return evse_allowed_amps - evse_allowed_amps / 8;
	}();

	// Cable limit (EVSE PP resistor)
	// This is fairly reliable and fairly important so there is no need to
	// support overriding it.
	if(max_input_a >= evse_pp_cable_rating_a){
		max_input_a = evse_pp_cable_rating_a;
	}

	// Final limit
	if(max_input_a >= INPUT_CURRENT_MAX_A){
		max_input_a = INPUT_CURRENT_MAX_A;
	}

	return max_input_a;
}

static void control_buck()
{
	int16_t max_input_a = get_max_input_a();

	// Calibrated input peak A = 2.0 * input RMS A (at around 10kW)
	int16_t max_input_dc_current_Ax10 = max_input_a * 20;

	if(
		output_dc_current_Ax10 >= wanted_output_current * 20
	){
		// More than 200% current or more than 120% output voltage
		if(current_pwm > 10)
			current_pwm -= current_pwm / 8;
		else
			current_pwm = 0;
		limiting_value = LV_OUTPUT_CURRENT_FAST;
	} else if(
		input_dc_current_Ax10 > max_input_dc_current_Ax10 * 2
	){
		// More than 200% current or more than 120% output voltage
		if(current_pwm > 10)
			current_pwm -= current_pwm / 8;
		else
			current_pwm = 0;
		limiting_value = LV_INPUT_CURRENT_FAST;
	} else if(
		output_voltage_V >= wanted_output_voltage * 6 / 5
	){
		// More than 200% current or more than 120% output voltage
		if(current_pwm > 10)
			current_pwm -= current_pwm / 8;
		else
			current_pwm = 0;
		limiting_value = LV_OUTPUT_VOLTAGE_FAST;
	} else if(
		output_dc_current_Ax10 > wanted_output_current * 12
	){
		// More than 120% current or more than 100% output voltage
		current_pwm--;
		limiting_value = LV_OUTPUT_CURRENT;
	} else if(
		input_dc_current_Ax10 > max_input_dc_current_Ax10 * 6 / 5
	){
		// More than 120% current or more than 100% output voltage
		current_pwm--;
		limiting_value = LV_INPUT_CURRENT;
	} else if(
		output_voltage_V > wanted_output_voltage
	){
		// More than 120% current or more than 100% output voltage
		current_pwm--;
		limiting_value = LV_OUTPUT_VOLTAGE;
	} else if(
		output_dc_current_Ax10 < wanted_output_current * 5 &&
		input_dc_current_Ax10 < max_input_dc_current_Ax10 / 2 &&
		output_voltage_V < wanted_output_voltage
	){
		// Less than 50% current and less than 100% output voltage
		current_pwm++;
		//limiting_value = LV_NOTHING; // Reset by report_status_on_console()
	} else {
		// Close to wanted values; adjust slower
		static uint8_t counter = 0;
		counter++;
		if(counter >= 4){
			counter = 0;

			// More than 100% current or more than 100% output voltage
			if(output_dc_current_Ax10 > wanted_output_current * 10){
				current_pwm--;
				limiting_value = LV_OUTPUT_CURRENT_SLOW;
			} else if(output_voltage_V > wanted_output_voltage){
				current_pwm--;
				limiting_value = LV_OUTPUT_VOLTAGE_SLOW;
			} else if(input_dc_current_Ax10 > max_input_dc_current_Ax10){
				current_pwm--;
				limiting_value = LV_INPUT_CURRENT_SLOW;
			} else {
				// Less than 100% current and less than 100% output voltage
				current_pwm++;
				//limiting_value = LV_NOTHING; // Reset by report_status_on_console()
			}
		}
	}

	if(current_pwm > wanted_pwm){
		current_pwm = wanted_pwm;
	}

	// Limit PWM
	if(current_pwm < 0)
		current_pwm = 0;
	else if(current_pwm > ICR1)
		current_pwm = ICR1;

	disable_pwm = [&](){
		if(dcbus1_raw >= (int16_t)(OUTPUT_VOLTAGE_MAX_V / DCBUS1_V_PER_BIT)){
			return DPR_DCBUS1_OVERVOLTAGE;
		}
		if(dcbus2_raw >= (int16_t)(INPUT_VOLTAGE_MAX_V / DCBUS2_V_PER_BIT)){
			return DPR_DCBUS2_OVERVOLTAGE;
		}
		if(wanted_pwm == 0){
			return DPR_WANTED_PWM_IS_ZERO;
		}
		if(boost_t1_c > BOOST_MAX_TEMPERATURE_C ||
				boost_t2_c > BOOST_MAX_TEMPERATURE_C){
			return DPR_BOOST_OVER_TEMPERATURE;
		}
		if(evse_pp_cable_rating_a == 0){
			return DPR_LOST_EVSE_PROXIMITY_PILOT;
		}
		return DPR_PWM_ENABLED;
	}();

	if(disable_pwm != DPR_PWM_ENABLED){
		set_pwm_inactive();
		current_pwm = 0;
	} else {
		set_pwm_buck_active(current_pwm);
	}
}

static void control_boost()
{
	if(wanted_switching_mode == SM_BOOST_SINGLE_PULSE){
		disable_pwm = DPR_PWM_ENABLED;
		if(boost_single_pulse_state == BSPS_INIT){
			// Start a pulse if it's needed
			if(input_voltage_V >= INPUT_VOLTAGE_MAX_V ||
					input_voltage_V >= wanted_output_voltage){
				// Enough voltage, not creating pulse
				current_pwm = 0;
				set_pwm_inactive();
				boost_single_pulse_state = BSPS_PULSED;
			} else {
				current_pwm = wanted_pwm;
				set_pwm_boost_active(current_pwm);
				boost_single_pulse_state = BSPS_PULSING;
			}
		} else if(boost_single_pulse_state == BSPS_PULSING){
			// Pulse was started, now we end it
			current_pwm = 0;
			set_pwm_inactive();
			boost_single_pulse_state = BSPS_PULSED;
		} else {
			// Pulse started and ended, nothing to do
			disable_pwm = DPR_PULSE_DONE;
		}
	} else if(wanted_switching_mode == SM_BOOST){
		disable_pwm = DPR_PWM_ENABLED;
		if(input_voltage_V >= INPUT_VOLTAGE_MAX_V ||
				input_voltage_V >= wanted_output_voltage){
			// Enough voltage
			current_pwm = 0;
			set_pwm_inactive();
		} else {
			current_pwm = wanted_pwm;
			set_pwm_boost_active(current_pwm);
		}
	}
}

SIGNAL(TIMER1_CAPT_vect) 
{
#if TEST_BOOST == true || TEST_BUCK == true
	return;
#endif

	static uint8_t pwm_handler_interval_counter = 0;
	pwm_handler_interval_counter++;
	if(pwm_handler_interval_counter < PWM_HANDLER_INTERVAL)
		return;
	pwm_handler_interval_counter = 0;

	static uint8_t interrupt_counter = 0;

	// NOTE: We have time for one analogRead and some calculation per PWM cycle

	static uint8_t ch_i = 255;
	ch_i++;
	switch(ch_i){
	case 0:
		dcbus1_raw = analogRead(DCBUS1_PIN) - DCBUS1_OFFSET_BITS;
		output_voltage_V = ((int32_t)dcbus1_raw * (int32_t)(DCBUS1_V_PER_BIT*1000)) / 1000;
		break;
	case 1:
		dcbus2_raw = analogRead(DCBUS2_PIN) - DCBUS2_OFFSET_BITS;
		input_voltage_V = ((int32_t)dcbus2_raw * (int32_t)(DCBUS2_V_PER_BIT*1000)) / 1000;
		break;
	case 2:
		mg2l1_current_raw = analogRead(MG1_L1_CURRENT_PIN) -
				mg2l1_current_raw_calibrated_zero;
		break;
	case 3:
		mg2l2_current_raw = analogRead(MG1_L2_CURRENT_PIN) -
				mg2l2_current_raw_calibrated_zero;
		break;
	case 4:
		boost_t1_raw = analogRead(BOOST_T1_PIN);
		break;
	case 5:
		boost_t2_raw = analogRead(BOOST_T2_PIN);
		break;
	case 6:
		evse_pp_raw = analogRead(EVSE_PP_PIN);
		break;
	default:
		ch_i = 255;
		break;
	}

	if(ch_i == 3){
		// Control PWM

		int16_t l1_current_A =
				abs(((int32_t)mg2l1_current_raw * (int32_t)(MG1_CURRENT_A_PER_BIT * 1000)) / 1000);
		int16_t l2_current_A =
				abs(((int32_t)mg2l2_current_raw * (int32_t)(MG1_CURRENT_A_PER_BIT * 1000)) / 1000);
		int16_t input_now_A = l1_current_A > l2_current_A ? l1_current_A : l2_current_A;
		input_current_avgbuf.push(input_now_A);
		input_dc_current_Ax10 = limit_int32(input_current_avgbuf.avg(10), 0, 32767);
		int32_t input_power_Wx10 = (int32_t)input_dc_current_Ax10 * input_voltage_V;
		int16_t input_to_output_V_factor_x10 = input_voltage_V * 10 / output_voltage_V;
		// Limit so that value is not too inaccurate
		if(input_to_output_V_factor_x10 > 50) // 50 = 5x
			input_to_output_V_factor_x10 = 50;
		// With correction according to measurements in practice (0.74x)
		output_dc_current_Ax10 = limit_int32((int32_t)input_dc_current_Ax10 *
				input_to_output_V_factor_x10 * 74 / 1000, 0, 32767);
		// Actually this is more accurate at around 10-15kW
		// NOTE: Accurate as input power, but output after efficiency is
		//       probably more like above?
		/*output_dc_current_Ax10 = limit_int32((int32_t)input_dc_current_Ax10 *
				input_to_output_V_factor_x10 * 93 / 1000, 0, 32767);*/

		if(current_switching_mode != wanted_switching_mode){
			current_pwm = 0;
			current_switching_mode = wanted_switching_mode;
		}

		if(current_switching_mode == SM_BUCK){
			control_buck();
		} else if(current_switching_mode == SM_BOOST_SINGLE_PULSE ||
				current_switching_mode == SM_BOOST){
			control_boost();
		} else {
			set_pwm_inactive();
			current_pwm = 0;
		}
	}

	if(mainloop_running){
		if(interrupt_counter == 0){ // 29Hz, 34ms
			// If main loop doesn't reset counter before 340ms, issue warning
			if(interrupt_counter_for_mainloop > 10){
				Serial.println(F("WARNING: Too little time for main loop detected"));
				interrupt_counter_for_mainloop = 0;
			}
			interrupt_counter_for_mainloop++;
		}
	}

	interrupt_counter++;
}

void report_status_on_console()
{
	static unsigned long last_accurate_report_timestamp = 0;
	bool accurate = false;
	if(timestamp_age(last_accurate_report_timestamp) >= 2000){
		last_accurate_report_timestamp = millis();
		accurate = true;
	}

	// Configuration
	REPORT_INT16(BATTERY_CHARGE_VOLTAGE);
	REPORT_INT16(OUTPUT_CURRENT_MAX_A);

	// PWM control
	REPORT_ENUM(disable_pwm, DisablePwmReason_STRINGS);
	REPORT_INT16_FORMAT(wanted_output_voltage, 1, 1, " V")
	REPORT_INT16_FORMAT(wanted_output_current, 1, 1, " A")
	REPORT_UINT16_FORMAT(wanted_pwm, accurate ? 1 : 5, 100.0/PWM_MAX, " %")
	//REPORT_INT16_FORMAT(dcbus1_raw, accurate ? 2 : 4, DCBUS1_V_PER_BIT, " V")
	//REPORT_INT16_FORMAT(dcbus2_raw, accurate ? 2 : 4, DCBUS2_V_PER_BIT, " V")
	//REPORT_INT16_FORMAT(mg2l1_current_raw, accurate ? 1 : 2, MG1_CURRENT_A_PER_BIT, " A")
	//REPORT_INT16_FORMAT(mg2l2_current_raw, accurate ? 1 : 2, MG1_CURRENT_A_PER_BIT, " A")
#if REPORT_PWM_LIMITING_VALUE == true
	REPORT_ENUM(limiting_value, LimitingValue_STRINGS);
	limiting_value = LV_NOTHING;
#endif

	// Digital outputs
	REPORT_BOOL(digitalRead(AC_CONTACTOR_SWITCH_PIN))
	REPORT_BOOL(digitalRead(AC_PRECHARGE_SWITCH_PIN))
	REPORT_BOOL(digitalRead(CONVERTER_SHORT_SWITCH_PIN))

	// Temperatures
	//REPORT_INT16_FORMAT(boost_t1_raw, accurate ? 2 : 50, 1, " raw");
	REPORT_INT16_FORMAT(boost_t1_c, accurate ? 2 : 5, 1, " C");
	//REPORT_INT16_FORMAT(boost_t2_raw, accurate ? 2 : 50, 1, " raw");
	REPORT_INT16_FORMAT(boost_t2_c, accurate ? 2 : 5, 1, " C");

	// EVSE
	REPORT_INT16_FORMAT(evse_pp_raw, accurate ? 2 : 50, 1, " raw");
	REPORT_UINT8(evse_allowed_amps);
	REPORT_UINT8(evse_pp_cable_rating_a);
	REPORT_INT16(get_max_input_a());

	// Charger status
	REPORT_ENUM(charger_state, ChargerState_STRINGS);
	REPORT_ENUM(charger.fail_reason, ChargerFailReason_STRINGS);

	// BMS
	if(CANBUS_ENABLE){
		REPORT_BOOL(canbus_alive())
		REPORT_INT16_FORMAT(canbus_status.max_charge_current_A, accurate ? 1 : 2, 1, "A")
		REPORT_INT16_FORMAT(canbus_status.cell_voltage_max_mV, accurate ? 10 : 100, 0.001, "V")
		REPORT_BOOL(canbus_status.permit_charge)
		REPORT_BOOL(canbus_status.main_contactor_closed)
	}

	// Input and output voltage, current and PWM %
	/*REPORT_INT16_FORMAT(input_voltage_V, accurate ? 2 : 4, 1, " V")
	REPORT_INT16_FORMAT(output_voltage_V, accurate ? 2 : 4, 1, " V")
	REPORT_INT16_FORMAT(input_dc_current_Ax10, accurate ? 1 : 2, 0.1, " A")
	REPORT_INT16_FORMAT(output_dc_current_Ax10, accurate ? 1 : 2, 0.1, " A")
	REPORT_UINT16_FORMAT(current_pwm, accurate ? 1 : 5, PWM_MAX/100.0/1024, " %")*/
	{
		static int16_t reported_input_voltage_V = 0;
		static int16_t reported_output_voltage_V = 0;
		static int16_t reported_input_dc_current_Ax10 = 0;
		static int16_t reported_output_dc_current_Ax10 = 0;
		static int16_t reported_current_pwm = 0;
		if(
			abs(input_voltage_V - reported_input_voltage_V) > (accurate ? 2 : 10) ||
			abs(output_voltage_V - reported_output_voltage_V) > (accurate ? 2 : 4) ||
			abs(input_dc_current_Ax10 - reported_input_dc_current_Ax10) > (accurate ? 1 : 5) ||
			((abs(output_dc_current_Ax10 - reported_output_dc_current_Ax10) > (accurate ? 1 : 8)) && current_pwm > 0) ||
			abs(current_pwm - reported_current_pwm) > (accurate ? 1 : (PWM_MAX/100+1)) ||
			console_report_all_values
		){
			log_print_timestamp();
			CONSOLE.print(F(">> PWM "));
			CONSOLE.print((float)current_pwm / (float)(PWM_MAX / 100.0));
			CONSOLE.print(F("%, in "));
			CONSOLE.print(input_dc_current_Ax10 * 0.1);
			CONSOLE.print(F("A @ "));
			CONSOLE.print(input_voltage_V);
			CONSOLE.print(F("V, out "));
			CONSOLE.print(output_dc_current_Ax10 * 0.1);
			CONSOLE.print(F("A @ "));
			CONSOLE.print(output_voltage_V);
			CONSOLE.print(F("V, "));
			CONSOLE.print((int32_t)input_dc_current_Ax10 * input_voltage_V / 10);
			CONSOLE.println(F("W"));

			reported_input_voltage_V = input_voltage_V;
			reported_output_voltage_V = output_voltage_V;
			reported_input_dc_current_Ax10 = input_dc_current_Ax10;
			reported_output_dc_current_Ax10 = output_dc_current_Ax10;
			reported_current_pwm = current_pwm;
		}
	}
}

void console_help()
{
	CONSOLE.println(F("Useful commands:"));
	CONSOLE.println(F("  r (report)"));
	CONSOLE.println(F("  chp  (charger stop)"));
	CONSOLE.println(F("  chr  (charger restore)"));
	CONSOLE.println(F("  aca <A>  (force_ac_input_amps)"));
}

void handle_command(const char *command, size_t command_len)
{
	if(command[0] == 'h' || command[0] == '?'){
		console_help();
		return;
	}
	if(strcmp(command, "report") == 0 || strcmp(command, "r") == 0){
		console_report_all_values = true;
		report_status_on_console();
		console_report_all_values = false;
		return;
	}
	if(strcmp(command, "charger stop") == 0 || strcmp(command, "chp") == 0){
		stop_charging();
		return;
	}
	if(strcmp(command, "charger restore") == 0 || strcmp(command, "chr") == 0){
		restore_initial_state();
		return;
	}
	if(strncmp(command, "aca ", 4) == 0){
		force_ac_input_amps = strtol(&command[4], NULL, 10);
		log_print_timestamp();
		CONSOLE.print(F("force_ac_input_amps set: "));
		CONSOLE.print(force_ac_input_amps);
		CONSOLE.println(F(" A"));
		return;
	}

	CONSOLE.print(F("Unknown command: "));
	CONSOLE.println(command);
	console_help();
}

void read_console_serial()
{
	while(CONSOLE.available()){
		if(command_accumulator.put_char(CONSOLE.read())){
			const char *command = command_accumulator.command();
			size_t len = command_accumulator.next_i;
			log_print_timestamp();
			CONSOLE.print(F("Command: "));
			CONSOLE.println(command);
			handle_command(command, len);
		}
	}
}

void evse_cp_pwm_handler()
{
	static bool state_before = false;

	int8_t state_sum = 0;
	for(uint8_t i=0; i<10; i++){
		state_sum += digitalRead(EVSE_CP_PIN) ? 1 : -1;
	}
	bool state_now = state_sum >= 0;

	if(state_now == state_before)
		return;

	if(state_now){
		evse_pwm_last_rise_timestamp_us = micros();
	} else {
		unsigned long t1 = micros();
		if(t1 >= evse_pwm_last_rise_timestamp_us){
			unsigned long t = t1 - evse_pwm_last_rise_timestamp_us;
			uint32_t amps = t / 16;
			if(amps >= 3 || amps <= 80){
				evse_pwm_pulse_counter++;
				evse_pwm_last_valid_value_timestamp = millis();
				evse_allowed_amps_avgbuf.push(amps);
				evse_allowed_amps = evse_allowed_amps_avgbuf.avg();
			}
		}
	}
	state_before = state_now;
}

void handle_evse_pwm_timeout()
{
	cli();
	unsigned long t = evse_pwm_last_valid_value_timestamp;
	sei();

	EVERY_N_MILLISECONDS(EVSE_PWM_TIMEOUT_MS){
		cli();
		uint16_t v = evse_pwm_pulse_counter;
		evse_pwm_pulse_counter = 0;
		sei();
		if(v >= EVSE_PWM_TIMEOUT_MS / 2){ // 50% of pulses at 1ms/pulse
			evse_pwm_enough_pulses_received = true;
		} else {
			evse_pwm_enough_pulses_received = false;
			evse_allowed_amps = 0;
			cli();
			evse_allowed_amps_avgbuf.reset();
			sei();
		}
	}
}

void convert_evse()
{
	evse_pp_cable_rating_a = [&](){
		const uint16_t a = evse_pp_raw;
		if(a < (uint16_t)(1024/5 * 0.94))
			return 63;
		if(a < (uint16_t)(1024/5 * 2.29))
			return 32;
		if(a < (uint16_t)(1024/5 * 2.70))
			return 13;
		if(a < (uint16_t)(1024/5 * 3.03))
			return 6;
		return 0;
	}();
}

void convert_temperatures()
{
	// TODO: Fix this mess
	{
		int16_t t = boost_t1_raw;
		// This is weird
		if(t < 450)
			t += (879 - 75);
		// Not calibrated
		const int16_t bits_per_c = (867 - 792) / (20 - 35);
		boost_t1_c = 20 + (boost_t1_raw - 867) / bits_per_c;
	}
	{
		int16_t t = boost_t2_raw;
		// This is weird
		if(t < 450)
			t += (879 - 75);
		// Not calibrated
		const int16_t bits_per_c = (867 - 792) / (20 - 35);
		boost_t2_c = 20 + (boost_t2_raw - 867) / bits_per_c;
	}
}

static void canbus_detected() {
	canbus_last_receive_timestamp = millis();
}

void init_system_can()
{
	// Init MCP2515
	for(uint8_t i=0; i<10; i++){
		if(system_can.begin(MCP_STDEXT, CAN_500KBPS, MCP_16MHZ) == CAN_OK){
			log_println_f("can_init: MCP2515 init ok");
			// Allow messages to be transmitted
			system_can.setMode(MCP_NORMAL);
			// One-shot TX
			//system_can.enOneShotTX();
			break;
		} else {
			log_println_f("can_init: MCP2515 init failed");
		}
		delay(500);
	}

	init_system_can_filters();
}

void init_system_can_filters()
{
	// Any 0x6** (input message) or 0x1** (BMS)
	if(system_can.init_Mask(0, false, 0x0f00L << 16) == MCP2515_FAIL) goto filter_fail;
	if(system_can.init_Filt(0, false, 0x0600L << 16) == MCP2515_FAIL) goto filter_fail;
	if(system_can.init_Filt(1, false, 0x0100L << 16) == MCP2515_FAIL) goto filter_fail;

	// Any 0x1** (BMS)
	if(system_can.init_Mask(1, false, 0x0f00L << 16) == MCP2515_FAIL) goto filter_fail;
	if(system_can.init_Filt(2, false, 0x0100L << 16) == MCP2515_FAIL) goto filter_fail;
	if(system_can.init_Filt(3, false, 0x0100L << 16) == MCP2515_FAIL) goto filter_fail;
	if(system_can.init_Filt(4, false, 0x0100L << 16) == MCP2515_FAIL) goto filter_fail;
	if(system_can.init_Filt(5, false, 0x0100L << 16) == MCP2515_FAIL) goto filter_fail;
	return;

filter_fail:
	log_println_f("FAILED to set MCP2515 filters");
	log_println_f("WARNING: CANbus communication will not work correctly.");
	//for(;;);
}

uint8_t pack_32A_into_4bits(uint8_t v)
{
	uint8_t r = v / 2;
	if(r > 0x0f)
		r = 0x0f;
	return r;
}

void send_canbus_frames()
{
	// Put our status info on the bus
	{
		CAN_FRAME frame;
		frame.id = 0x600;
		frame.length = 8;
		bool request_inverter_disable = (evse_pp_cable_rating_a > 0 ||
				evse_allowed_amps > 0 || (charger_state >= CS_WAITING_CANBUS &&
						charger_state <= CS_STOPPING_CHARGE));
		// Request main contactor, in addition to when it's actually needed,
		// also when the inverter is requested to be disabled, so that the DC-DC
		// can maintain charge in the 12V system.
		bool request_main_contactor =
				digitalRead(AC_PRECHARGE_SWITCH_PIN) || digitalRead(AC_CONTACTOR_SWITCH_PIN) ||
				charger_state == CS_PRECHARGING || charger_state == CS_CHARGING ||
				charger_state == CS_STOPPING_CHARGE || request_inverter_disable;
		uint16_t pack_voltage_Vx10 = output_voltage_V * 10;
		uint16_t charge_current_Ax10 = current_pwm == 0 ? 0 : output_dc_current_Ax10;
		int8_t charger_temperature_c = boost_t1_c;
		if(charger_temperature_c < boost_t2_c)
			charger_temperature_c = boost_t2_c;
		bool request_cooling = (charger_temperature_c > 40);

		frame.data.bytes[0] =
				(request_main_contactor ? (1<<0) : 0) |
				(false ? (1<<1) : 0) |
				(false ? (1<<2) : 0) |
				(request_inverter_disable ? (1<<3) : 0) |
				(request_cooling ? (1<<4) : 0);
		frame.data.bytes[1] = pack_voltage_Vx10 >> 8;
		frame.data.bytes[2] = pack_voltage_Vx10 & 0xff;
		frame.data.bytes[3] = output_dc_current_Ax10 / 10;
		frame.data.bytes[4] = (pack_32A_into_4bits(evse_allowed_amps) << 4) |
				pack_32A_into_4bits(evse_pp_cable_rating_a);
		frame.data.bytes[5] = charger_temperature_c;
		frame.data.bytes[6] = ((charger.fail_reason & 0x0f) << 4) | (charger_state & 0x0f);
		frame.data.bytes[7] = force_ac_input_amps;

		if(!canc_send(system_can, frame)){
			EVERY_N_MILLISECONDS(10000){
				log_println_f("Failed to send CAN frame 0x600");
			}
		}
	}

	// Put a separate message on the bus to disable the inverter when a cable is
	// connected or we are charging
	if(evse_pp_cable_rating_a > 0 || evse_allowed_amps > 0 ||
			(charger_state >= CS_WAITING_CANBUS &&
					charger_state <= CS_STOPPING_CHARGE)){
		CAN_FRAME frame;
		frame.id = 0x320;
		frame.length = 8;

		frame.data.bytes[0] = 1;
		frame.data.bytes[1] = 0;
		frame.data.bytes[2] = 0;
		frame.data.bytes[3] = 0;
		frame.data.bytes[4] = 0;
		frame.data.bytes[5] = 0;
		frame.data.bytes[6] = 0;
		frame.data.bytes[7] = 0;

		if(!canc_send(system_can, frame)){
			EVERY_N_MILLISECONDS(10000){
				log_println_f("Failed to send CAN frame 0x320");
			}
		}
	}
}

static void handle_canbus_frame(const CAN_FRAME &frame)
{
	// BMS
	if(frame.id == 0x100){
		canbus_detected();

		canbus_status.permit_charge = frame.data.bytes[0] & (1<<0);
		canbus_status.main_contactor_closed = frame.data.bytes[0] & (1<<2);
		canbus_status.pack_voltage_V =
				((frame.data.bytes[1] << 8) | frame.data.bytes[2]) / 10;
		return;
	}
	if(frame.id == 0x101){
		canbus_detected();

		canbus_status.cell_voltage_min_mV = (uint16_t)((frame.data.bytes[0] << 4) |
				(frame.data.bytes[1] >> 4)) * 10;
		canbus_status.cell_voltage_max_mV = (uint16_t)(((frame.data.bytes[1] & 0x0f) << 8) |
				frame.data.bytes[2]) * 10;
		canbus_status.cell_temperature_min = frame.data.bytes[3];
		canbus_status.cell_temperature_max = frame.data.bytes[4];
		canbus_status.charge_completed = frame.data.bytes[5] & (1<<1);
		return;
	}
	if(frame.id == 0x102){
		canbus_detected();

		canbus_status.max_charge_current_A =
				((frame.data.bytes[2] << 8) | frame.data.bytes[3]) / 10;
		return;
	}

	// Configuration messages for ourselves
	if(frame.id == 0x620){
		// Set setting
		uint8_t setting_id = frame.data.bytes[0];
		uint16_t old_value = (frame.data.bytes[1] << 8) | frame.data.bytes[2];
		uint16_t new_value = (frame.data.bytes[3] << 8) | frame.data.bytes[4];
		if(setting_id == 0){ // AC input A limit setting
			if(force_ac_input_amps == old_value){
				force_ac_input_amps = new_value;
				send_canbus_frames();

				log_print_timestamp();
				CONSOLE.print(F("CANbus setting force_ac_input_amps = "));
				CONSOLE.println(force_ac_input_amps);
			}
		}
		if(setting_id == 1){
			log_println_f("CANbus start charge");
			start_charging();
		}
		if(setting_id == 2){
			log_println_f("CANbus stop charge");
			stop_charging();
		}
		if(setting_id == 3){
			log_println_f("CANbus restore initial state");
			restore_initial_state();
		}
		return;
	}
}

static void apply_canbus_timeouts()
{
	if(CANBUS_ENABLE){
		static bool canbus_reported_alive = false;
		if(canbus_alive()){
			if(!canbus_reported_alive &&
					canbus_last_receive_timestamp != 0){
				canbus_reported_alive = true;
				log_println_f("CANbus up");
				canbus_status = CanbusStatus(); // Reset status
			}
		} else {
			if(canbus_reported_alive){
				canbus_reported_alive = false;
				log_println_f("CANbus lost");
				canbus_status = CanbusStatus(); // Reset status
			}
		}
	}
}

static void read_canbus_frames()
{
	if(CANBUS_ENABLE){
		for(uint8_t i=0; i<10 && !digitalRead(MCP2515_INT_PIN); i++){
			CAN_FRAME frame;
			canc_read(system_can, frame);

			uint32_t &id = frame.id;
			uint8_t &len = frame.length;
			uint8_t *data = frame.data.bytes;

			handle_canbus_frame(frame);
		}
	}
}

static bool canc_send(MCP_CAN &mcp_can, const CAN_FRAME &frame)
{
	return (mcp_can.sendMsgBuf(frame.id, 0, frame.length,
			frame.data.bytes) == CAN_OK);
}

static bool canc_read(MCP_CAN &mcp_can, CAN_FRAME &frame)
{
	memset(&frame, 0, sizeof frame);
	uint8_t r = mcp_can.readMsgBuf(&frame.id, &frame.length, frame.data.bytes);
	return (r == CAN_OK);
}

static bool canbus_alive()
{
	return timestamp_younger_than(canbus_last_receive_timestamp, CANBUS_TIMEOUT_MS);
}
