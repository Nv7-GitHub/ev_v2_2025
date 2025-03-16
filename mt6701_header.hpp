#include "mt6701.h"
#pragma once 

class MT6701 {
public:
	MT6701( void );
	// Only hardware I2C and SPI support
	bool initializeI2C( bool wire0 );
	bool initializeSSI( int cs_pin );

	// SPI & I2C functions
	float angleRead( void );

	// SPI only functions
	mt6701_status_t fieldStatusRead( void );

	// I2C only functions
	void uvwModeSet( uint8_t pole_pairs );
	void abzModeSet( uint16_t pulses_per_round, mt6701_pulse_width_t z_pulse_width = MT6701_PULSE_WIDTH_180, mt6701_hyst_t hysteresis = MT6701_HYST_1 );
	void nanbnzEnable( bool nanbnz_enable ); // Only for QFN version
	// Analog/PWM pin mode select
	void analogModeSet( float start = 0.0f, float stop = 360.0f );
	void pwmModeSet( mt6701_pwm_freq_t frequency = MT6701_PWM_FREQ_497_2, mt6701_pwm_pol_t polarity = MT6701_PWM_POL_HIGH );
	// General tracking options
	void offsetSet( float offset );
	void directionSet( mt6701_direction_t direction );
	// Save settings as default
	void programmEEPROM( void );

private:
	int cs_pin;
	mt6701_handle_t handle;
  bool wire0;

	// Internal c function for driver
	static uint8_t ssi_read( uint8_t* data, uint8_t len );
	static uint8_t i2c_write_wire0( uint8_t reg, uint8_t data );
	static uint8_t i2c_read_wire0( uint8_t reg, uint8_t *data );
  static uint8_t i2c_write_wire1( uint8_t reg, uint8_t data );
	static uint8_t i2c_read_wire1( uint8_t reg, uint8_t *data );
};
