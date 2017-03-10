#ifndef __CPWM_H__
#define __CPWM_H__

#include <sam3s4b.h>

#define PWM_INVALID_ARGUMENT  0xFFFF

/** Definitions for PWM channel number */
typedef enum _pwm_ch_t {
	PWM_CHANNEL_0 = 0,
	PWM_CHANNEL_1 = 1,
	PWM_CHANNEL_2 = 2,
	PWM_CHANNEL_3 = 3
} pwm_ch_t;

/** Input parameters when initializing PWM */
typedef struct {
	/** Frequency of clock A in Hz (set 0 to turn it off) */
	uint32_t ul_clka;
	/** Frequency of clock B in Hz (set 0 to turn it off) */
	uint32_t ul_clkb;
	/** Frequency of master clock in Hz */
	uint32_t ul_mck;
} pwm_clock_t;


/** Definitions for PWM channel alignment */
typedef enum {
	PWM_ALIGN_LEFT = (0 << 8),   /* The period is left aligned. */
	PWM_ALIGN_CENTER = (1 << 8)  /* The period is center aligned. */
} pwm_align_t;

/** Definitions for PWM level */
typedef enum {
	PWM_LOW = 0,     /* Low level */
	PWM_HIGH = 1,  /* High level */
} pwm_level_t;

/** Definitions for PWM event */
typedef enum {
	PWM_EVENT_PERIOD_END = (0 << 10),      /* The channel counter event occurs at the end of the PWM period. */
	PWM_EVENT_PERIOD_HALF_END = (1 << 10)  /* The channel counter event occurs at the half of the PWM period. */
} pwm_counter_event_t;

/** Configurations of a PWM channel output */
typedef struct {
	/** Boolean of using override output as PWMH */
	bool b_override_pwmh;
	/** Boolean of using override output as PWML */
	bool b_override_pwml;
	/** Level of override output for PWMH */
	pwm_level_t override_level_pwmh;
	/** Level of override output for PWML */
	pwm_level_t override_level_pwml;
} pwm_output_t;

// Definitions for PWM fault input ID
typedef enum {
	PWM_FAULT_PWMFI1 = (1 << 0),
	PWM_FAULT_MAINOSC = (1 << 1),
	PWM_FAULT_ADC = (1 << 2),
	PWM_FAULT_ACC = (1 << 3),
	PWM_FAULT_TIMER_0 = (1 << 4),
	PWM_FAULT_TIMER_1 = (1 << 5)
} pwm_fault_id_t;

/** Input parameters when configuring a PWM channel mode */
typedef struct {
	/** Channel number */
	uint32_t channel;
	/** Channel prescaler */
	uint32_t ul_prescaler;
	/** Channel alignment */
	pwm_align_t alignment;
	/** Channel initial polarity */
	pwm_level_t polarity;
	/** Duty Cycle Value */
	uint32_t ul_duty;
	/** Period Cycle Value */
	uint32_t ul_period;

	/** Channel counter event */
	pwm_counter_event_t counter_event;
	/** Boolean of channel dead-time generator */
	bool b_deadtime_generator;
	/** Boolean of channel dead-time PWMH output inverted */
	bool b_pwmh_output_inverted;
	/** Boolean of channel dead-time PWML output inverted */
	bool b_pwml_output_inverted;
	/** Dead-time Value for PWMH Output */
	uint16_t us_deadtime_pwmh;
	/** Dead-time Value for PWML Output */
	uint16_t us_deadtime_pwml;
	/** Channel output */
	pwm_output_t output_selection;
	/** Boolean of Synchronous Channel */
	bool b_sync_ch;
	/** Fault ID of the channel */
	pwm_fault_id_t fault_id;
	/** Channel PWMH output level in fault protection */
	pwm_level_t ul_fault_output_pwmh;
	/** Channel PWML output level in fault protection */
	pwm_level_t ul_fault_output_pwml;

} pwm_channel_t;


class cPWM
{
public:
	cPWM(Pwm *pwm_base);
	~cPWM();
	uint32_t init(pwm_clock_t *clock_config);
	uint32_t channelInit(pwm_channel_t *pwm_channel);
	void channelEnableInterrupt(uint32_t ul_event, uint32_t ul_fault);
	void channelEnable(uint32_t ul_channel);
	void channelDisable(uint32_t ul_channel);
	uint32_t updateDuty(pwm_channel_t *p_channel, uint32_t ul_duty);
	
	uint32_t channelInterruptStatus();

private:
	Pwm *mPWM;
};

#endif
