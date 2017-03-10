#include "cPWM.h"

#define PWM_CLOCK_DIV_MAX  256
#define PWM_CLOCK_PRE_MAX  11

/**
 * \brief Find a prescaler/divisor couple to generate the desired ul_frequency
 * from ul_mck.
 *
 * \param ul_frequency Desired frequency in Hz.
 * \param ul_mck Master clock frequency in Hz.
 *
 * \retval Return the value to be set in the PWM Clock Register (PWM Mode Register for
 * SAM3N/SAM4N/SAM4C/SAM4CP/SAM4CM) or PWM_INVALID_ARGUMENT if the configuration cannot be met.
 */
static uint32_t pwm_clocks_generate(uint32_t ul_frequency, uint32_t ul_mck)
{
	uint32_t ul_divisors[PWM_CLOCK_PRE_MAX] =
			{1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024 };
	uint32_t ul_pre = 0;
	uint32_t ul_div;

	/* Find prescaler and divisor values */
	do {
		ul_div = (ul_mck / ul_divisors[ul_pre]) / ul_frequency;
		if (ul_div <= PWM_CLOCK_DIV_MAX) {
			break;
		}
		ul_pre++;
	} while (ul_pre < PWM_CLOCK_PRE_MAX);

	/* Return result */
	if (ul_pre < PWM_CLOCK_PRE_MAX) {
		return ul_div | (ul_pre << 8);
	} else {
		return PWM_INVALID_ARGUMENT;
	}
}


cPWM::cPWM(Pwm *pwm_base) : mPWM(pwm_base)
{
	
}

cPWM::~cPWM()
{
}


uint32_t cPWM::init(pwm_clock_t *clock_config)
{
	uint32_t clock = 0;
	uint32_t result;

	/* Clock A */
	if (clock_config->ul_clka != 0) {
		result = pwm_clocks_generate(clock_config->ul_clka, clock_config->ul_mck);
		if (result == PWM_INVALID_ARGUMENT) {
			return result;
		}

		clock = result;
	}

	/* Clock B */
	if (clock_config->ul_clkb != 0) {
		result = pwm_clocks_generate(clock_config->ul_clkb, clock_config->ul_mck);

		if (result == PWM_INVALID_ARGUMENT) {
			return result;
		}

		clock |= (result << 16);
	}
	mPWM->PWM_CLK = clock;
	return 0;
}


uint32_t cPWM::channelInit(pwm_channel_t *p_channel)
{
	uint32_t tmp_reg = 0;
	uint32_t ch_num = p_channel->channel;

	/* Channel Mode/Clock Register */
	tmp_reg = (p_channel->ul_prescaler & 0xF) |
			(p_channel->polarity << 9) |
			(p_channel->counter_event) |
			(p_channel->b_deadtime_generator << 16) |
			(p_channel->b_pwmh_output_inverted << 17) |
			(p_channel->b_pwml_output_inverted << 18) |
			(p_channel->alignment);
	mPWM->PWM_CH_NUM[ch_num].PWM_CMR = tmp_reg;

	/* Channel Duty Cycle Register */
	mPWM->PWM_CH_NUM[ch_num].PWM_CDTY = p_channel->ul_duty;

	/* Channel Period Register */
	mPWM->PWM_CH_NUM[ch_num].PWM_CPRD = p_channel->ul_period;
	
	/* Channel Dead Time Register */
	if (p_channel->b_deadtime_generator) {
		mPWM->PWM_CH_NUM[ch_num].PWM_DT =
				PWM_DT_DTL(p_channel->
				us_deadtime_pwml) | PWM_DT_DTH(p_channel->
				us_deadtime_pwmh);
	}

	/* Output Selection Register */
	tmp_reg  = mPWM->PWM_OS & (~((PWM_OS_OSH0 | PWM_OS_OSL0) << ch_num));
	tmp_reg |= ((p_channel->output_selection.b_override_pwmh) << ch_num) |
			(((p_channel->output_selection.b_override_pwml) << ch_num)
					<< 16);
	mPWM->PWM_OS = tmp_reg;

	/* Output Override Value Register */
	tmp_reg  = mPWM->PWM_OOV & (~((PWM_OOV_OOVH0 | PWM_OOV_OOVL0) << ch_num));
	tmp_reg |= ((p_channel->output_selection.override_level_pwmh) << ch_num) |
			(((p_channel->output_selection.override_level_pwml) << ch_num)
					<< 16);
	mPWM->PWM_OOV = tmp_reg;

	/* Sync Channels Mode Register */
	uint32_t channel = (1 << ch_num);
	if (p_channel->b_sync_ch) {
		mPWM->PWM_SCM |= channel;
	} else {
		mPWM->PWM_SCM &= ~((uint32_t) channel);
	}

	if (p_channel->ul_fault_output_pwmh == PWM_HIGH) {
		mPWM->PWM_FPV |= (0x01 << ch_num);
	} else {
		mPWM->PWM_FPV &= (~(0x01 << ch_num));
	}
	if (p_channel->ul_fault_output_pwml == PWM_HIGH) {
		mPWM->PWM_FPV |= ((0x01 << ch_num) << 16);
	} else {
		mPWM->PWM_FPV &= (~((0x01 << ch_num) << 16));
	}

	/* Fault Protection Enable Register */
	uint32_t fault_enable_reg = 0;

	ch_num *= 8;
	fault_enable_reg = mPWM->PWM_FPE;
	fault_enable_reg &= ~(0xFF << ch_num);
	fault_enable_reg |= ((p_channel->fault_id) << ch_num);
	mPWM->PWM_FPE = fault_enable_reg;

	return 0;
}

void cPWM::channelEnableInterrupt(uint32_t ul_event, uint32_t ul_fault)
{
	mPWM->PWM_IER1 = (1 << ul_event) | (1 << (ul_fault + 16));
}

void cPWM::channelEnable(uint32_t ul_channel)
{
	mPWM->PWM_ENA = (1 << ul_channel);
}

uint32_t cPWM::updateDuty(pwm_channel_t *p_channel, uint32_t ul_duty)
{
	uint32_t ch_num = p_channel->channel;

		/** Check parameter */
	if (p_channel->ul_period < ul_duty) {
		return PWM_INVALID_ARGUMENT;
	} else {
		/* Save new duty cycle value */
		p_channel->ul_duty = ul_duty;

		mPWM->PWM_CH_NUM[ch_num].PWM_CDTYUPD = ul_duty;
	}

	return 0;
}

uint32_t cPWM::channelInterruptStatus()
{
	return mPWM->PWM_ISR1;
}

void cPWM::channelDisable(uint32_t ul_channel)
{
	mPWM->PWM_DIS = (1 << ul_channel);
}
