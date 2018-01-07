#include <omower-pwmservo.h>
#include <omower-defs.h>
#include <pwm_lib.h>

using namespace arduino_due::pwm_lib;

#ifndef NO_PWM_CHA
pwm<pwm_pin::CH_PWM_A> pwm_a;
#endif
#ifndef NO_PWM_CHB
pwm<pwm_pin::CH_PWM_B> pwm_b;
#endif
pwm<pwm_pin::CH_PWM_C> pwm_c;
pwm<pwm_pin::CH_PWM_D> pwm_d;
pwm<pwm_pin::CH_PWM_E> pwm_e;
#ifndef NO_FAN_CONTROL
pwm<pwm_pin::CH_PWM_FAN> pwm_fan;
#endif
#ifndef NO_CHARGE_CONTROL
pwm<pwm_pin::CH_PWM_BOOST> pwm_boost;
pwm<pwm_pin::CH_PWM_CC> pwm_cc;
#endif

// Start/set RC servo with duty 0..999 (1..2 ms) with refresh rate (hertz)
void pwmServo::servoStart(numThing n, uint16_t duty, uint16_t refresh) {
  pwmStart(n, 100000000L / (uint32_t) refresh, (1000L + (uint32_t) duty) * 100L);
} // void pwmServo::servoStart(numThing n, uint16_t duty, uint16_t refresh)

// Set duty cycle 0..999 (1..2 ms)
void pwmServo::servoSet(numThing n, uint16_t duty) {
  pwmSet(n, (1000L + (uint32_t) duty) * 100L);
} // Set duty cycle 0..999 (1..2 ms)

inline uint32_t pwmServo::husToCycles(uint32_t hus) {
  return (hus * 42L) / 100L;
} // inline uint32_t pwmServo::husToCycles(uint32_t hus)

// Start/set PWM output with duty/period in hundreths of microsecond
void pwmServo::pwmStart(numThing n, uint32_t period, uint32_t duty) {
  debug(L_DEBUG, (char *) F("pwmServo::pwmStart: thing %hu, period %lu, duty %lu\n"),
        n, period, duty);
  switch (n) {
    case PWM_A:
#ifndef NO_PWM_CHA
      pwm_a.start(period, duty);
#endif
      break;
    case PWM_B:
#ifndef NO_PWM_CHB
      pwm_b.start(period, duty);
#endif
      break;
    case PWM_C:
      pwm_c.start(period, duty);
      break;
    case PWM_D:
      pwm_d.start(period, duty);
      break;
    case PWM_E:
      pwm_e.start(period, duty);
      break;
    case PWM_F:
      // TC0/B0 - TIOB0
      REG_PMC_PCER0 |= PMC_PCER0_PID27;     // Enable TC0/0
      REG_PIOB_ABSR |= PIO_ABSR_P27;        // Multiplexer to TIOB0
      REG_PIOB_PDR |= PIO_PDR_P27;          // Disable GPIO for PB27
      REG_TC0_CMR0 = TC_CMR_BCPC_CLEAR |    // Set TIOB on counter match with RC0
                     TC_CMR_BCPB_SET |      // Clear TIOB on counter match with RB0
                     TC_CMR_WAVE |          // Enable wave mode
                     TC_CMR_WAVSEL_UP_RC |  // Count up with automatic trigger on RC compare
                     TC_CMR_EEVT_XC0 |      // Set event selection to XC0 to make TIOB an output
                     TC_CMR_TCCLKS_TIMER_CLOCK1; // CPU Freq / 2

      REG_TC0_RC0 = husToCycles(period);
      REG_TC0_RB0 = REG_TC0_RC0 - husToCycles(duty);

      REG_TC0_CCR0 = TC_CCR_SWTRG | TC_CCR_CLKEN;
      break;
    case PWM_G:
      // TC2/B2 - TIOB8
      REG_PMC_PCER1 |= PMC_PCER1_PID35;     // Enable TC2/2
      REG_PIOD_ABSR |= PIO_ABSR_P8;         // Multiplexer to TIOB8
      REG_PIOD_PDR |= PIO_PDR_P8;           // Disable GPIO for PD8
      REG_TC2_CMR2 = TC_CMR_BCPC_CLEAR |    // Set TIOB on counter match with RC0
                     TC_CMR_BCPB_SET |      // Clear TIOB on counter match with RB0
                     TC_CMR_WAVE |          // Enable wave mode
                     TC_CMR_WAVSEL_UP_RC |  // Count up with automatic trigger on RC compare
                     TC_CMR_EEVT_XC0 |      // Set event selection to XC0 to make TIOB an output
                     TC_CMR_TCCLKS_TIMER_CLOCK1; // CPU Freq / 2

      REG_TC2_RC2 = husToCycles(period);
      REG_TC2_RB2 = REG_TC2_RC2 - husToCycles(duty);

      REG_TC2_CCR2 = TC_CCR_SWTRG | TC_CCR_CLKEN;
      break;
    case PWM_H:
      // TC2/A0 - TIOA6
      REG_PMC_PCER1 |= PMC_PCER1_PID33;     // Enable TC2/0
      REG_PIOC_ABSR |= PIO_ABSR_P25;        // Multiplexer to TIOA6
      REG_PIOC_PDR |= PIO_PDR_P25;          // Disable GPIO for PC25
      REG_TC2_CMR0 = TC_CMR_ACPC_CLEAR |    // Set TIOB on counter match with RC0
                     TC_CMR_ACPA_SET |      // Clear TIOB on counter match with RB0
                     TC_CMR_WAVE |          // Enable wave mode
                     TC_CMR_WAVSEL_UP_RC |  // Count up with automatic trigger on RC compare
                     TC_CMR_EEVT_XC0 |      // Set event selection to XC0 to make TIOB an output
                     TC_CMR_TCCLKS_TIMER_CLOCK1; // CPU Freq / 2

      REG_TC2_RC0 = husToCycles(period);
      REG_TC2_RA0 = REG_TC2_RC0 - husToCycles(duty);

      REG_TC2_CCR0 = TC_CCR_SWTRG | TC_CCR_CLKEN;
      break;
    case PWM_FAN:
#ifndef NO_FAN_CONTROL
      pwm_fan.start(period, duty);
#endif
      break;
    case PWM_BOOST:
#ifndef NO_CHARGE_CONTROL
      pwm_boost.start(period, duty);
#endif
      break;
    case PWM_CC:
#ifndef NO_CHARGE_CONTROL
      pwm_cc.start(period, duty);
#endif
      break;
  }
} // void pwmServo::pwmStart(numThing n, uint32_t period, uint32_t duty)

// Set PWM output duty in microseconds
void pwmServo::pwmSet(numThing n, uint32_t duty) {
  switch (n) {
    case PWM_A:
#ifndef NO_PWM_CHA
      pwm_a.set_duty(duty);
#endif
      break;
    case PWM_B:
#ifndef NO_PWM_CHB
      pwm_b.set_duty(duty);
#endif
      break;
    case PWM_C:
      pwm_c.set_duty(duty);
      break;
    case PWM_D:
      pwm_d.set_duty(duty);
      break;
    case PWM_E:
      pwm_e.set_duty(duty);
      break;
    case PWM_F:
      // TC0/B0 - TIOB0
      REG_TC0_RB0 = REG_TC0_RC0 - husToCycles(duty);
      break;
    case PWM_G:
      // TC2/B2 - TIOB8
      REG_TC2_RB2 = REG_TC0_RC2 - husToCycles(duty);
      break;
    case PWM_H:
      // TC2/A0 - TIOA6
      REG_TC2_RA0 = REG_TC0_RC2 - husToCycles(duty);
      break;
    case PWM_FAN:
#ifndef NO_FAN_CONTROL
      pwm_fan.set_duty(duty);
#endif
      break;
    case PWM_BOOST:
#ifndef NO_CHARGE_CONTROL
      pwm_boost.set_duty(duty);
#endif
      break;
    case PWM_CC:
#ifndef NO_CHARGE_CONTROL
      pwm_cc.set_duty(duty);
#endif
      break;
  }
} // void pwmServo::pwmSet(numThing n, uint32_t duty)

numThing pwmServo::numThings() {
  return 11;
} // numThing pwmServo::numThings()

_locationThings pwmServo::locThings() {
  return _locationThings::SPECIAL;
} // _locationThings pwmServo::locThings()
