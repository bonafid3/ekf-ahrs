#include <sam3s4b.h>
#include "cPMC.h" // power management controller
#include "cTWI.h"
#include "cPIO.h"
#include "cTC.h"
#include "cUART.h"
#include "cBuffer.h"
#include "cPWM.h"
#include "cUART.h"
#include "cPDC.h" // peripheral dma controller
#include "cByteArray.h"
#include "cBuffer.h"

#include "cEKF.h"

#include "cMPU6050.h"
#include "cHMC5883.h"

#include "utils.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static cBuffer gBuffer;

static cEKF gEKF;

tStateData gState;

volatile uint32_t gCurrentTime = 0;
volatile uint32_t gDiffTime = 0;

volatile uint32_t gTickCount = 0;
volatile uint32_t gDelayCount = 0;

volatile uint32_t gSensorTime = 0;

volatile bool gSensorReady = false;

//PWM frequency in Hz
#define PWM_FREQUENCY		50
// Period value of PWM output waveform
//#define PERIOD_VALUE		60000
#define PERIOD_VALUE		40000

#define PWM_CHANNEL_MOT1	PWM_CHANNEL_0
#define PWM_CHANNEL_MOT2	PWM_CHANNEL_1
#define PWM_CHANNEL_MOT3	PWM_CHANNEL_2
#define PWM_CHANNEL_MOT4	PWM_CHANNEL_3

#define GEAR		PIO_PA19
#define THROTTLE	PIO_PA5
#define AILERON		PIO_PA6
#define ELEVATOR	PIO_PA7
#define RUDDER		PIO_PA8

#define INTA		PIO_PA24

#define LED			PIO_PA1

#define SERVO1		PIO_PA11
#define SERVO2		PIO_PA12
#define SERVO3		PIO_PA13
#define SERVO4		PIO_PA14

#define SERVO_NEUTRAL	3000

enum {eFM_UNDEFINED, eFM_MANUAL, eFM_AUTOPILOT};

volatile uint32_t gFlightMode = eFM_MANUAL;
volatile uint32_t gPrevFlightMode = eFM_UNDEFINED;

pwm_channel_t gServo1;
pwm_channel_t gServo2;
pwm_channel_t gServo3;
pwm_channel_t gServo4;

void enableAutopilot();
void disableAutopilot();

#define sampleFreq	72.7f		// sample frequency in Hz
#define betaDef		0.02f		// 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float beta = betaDef;								// 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

void MadgwickAHRSupdate(sSensorData &sd);
void MadgwickAHRSupdateIMU(sSensorData &sd);

float invSqrt(float x);

struct sSettings
{
	sSettings(){ pkt[0]='p'; pkt[1]='k'; pkt[2]='t'; }
	char pkt[3];

	cMatrix<3,3> mMagCal;
	
	float mMagXoffset;
	float mMagYoffset;
	float mMagZoffset;
};

// Timer0 channel1 used to forward pwm signals when running in manual mode
void TC1_Handler(void)
{
	cTC tc1(TC0, 1);
	if((tc1.status() & TC_SR_CPCS) == TC_SR_CPCS) {
	}
}

void UART0_Handler()
{
	cUART uart(UART0);
	uint32_t status = uart.status();
	
	//cPIO pioa(PIOA);
	
	if((status & UART_SR_RXRDY) == UART_SR_RXRDY) {
		uint8_t c;
		if(uart.read(c)) {
			gBuffer.append(c);
		}
	}
}

void PIOA_Handler( void )
{
	uint32_t status = 0;
	cPIO pioa(PIOA);
	
	cTC tc0(TC0, 0);
	
	status = pioa.interruptStatus();
	status &= pioa.interruptMask();

	if((status & INTA) == INTA) {
		gSensorReady = true;
	}

	if((status & GEAR) == GEAR) {
		if(pioa.status(GEAR)) {
			tc0.start(); // swtrg resets counter value
			pioa.configureFallingEdgeInterrupt(GEAR);
			gCurrentTime = tc0.counterValue();
		} else {
			gDiffTime = tc0.counterValue() - gCurrentTime;
			tc0.stop();
			pioa.configureRisingEdgeInterrupt(GEAR);
		}
	}
	
	if(gFlightMode == eFM_MANUAL) {

#undef FORWARD_PIN
#define FORWARD_PIN(p1, p2)							\
if((status & p1) == p1) {							\
	if(pioa.status(p1)) {							\
		pioa.set(p2);								\
		pioa.configureFallingEdgeInterrupt(p1);		\
	} else {										\
		pioa.clear(p2);								\
		pioa.configureRisingEdgeInterrupt(p1);		\
	}												\
}													\

		FORWARD_PIN(THROTTLE, SERVO1);
		FORWARD_PIN( AILERON, SERVO2);
		FORWARD_PIN(ELEVATOR, SERVO3);
		FORWARD_PIN(  RUDDER, SERVO4);

	}	
}

void SysTick_Handler()
{
	gTickCount++;
	gDelayCount++;
	gSensorTime++;
}


void delayms(uint32_t ms)
{
	gDelayCount = 0;
	while(gDelayCount < ms)
	{}
}

void delay(uint32_t s)
{
	delayms(s * 1000);
}


int main()
{
	WDT->WDT_MR = WDT_MR_WDDIS;
		
	SystemInit();
	
	SysTick_Config(SystemCoreClock / 1000 ); // 1ms resolution 60000 works
	
	//delay(1);

	cPMC pmc; // peripheral master clock
	pmc.enablePeriphClk(ID_PIOA);
	pmc.enablePeriphClk(ID_TWI0);
	pmc.enablePeriphClk(ID_TC0); // sam3s4b has only one 16bit timer with 3 channels, so this is channel 0
	pmc.enablePeriphClk(ID_TC1); // and this is channel 1 on Timer0
	pmc.enablePeriphClk(ID_UART0);
	pmc.enablePeriphClk(ID_UART1);
	pmc.enablePeriphClk(ID_PWM);
		
	cPIO pioa(PIOA);
	pioa.setPeripheralA(PIO_PA4A_TWCK0);
	pioa.setPeripheralA(PIO_PA3A_TWD0);
	
	// to uart 0 raspberry pi / pc
	pioa.setPeripheralA(PIO_PA9A_URXD0);
	pioa.setPeripheralA(PIO_PA10A_UTXD0);



	// uart1 to gps
	pioa.setPeripheralA(PIO_PA21A_RXD1);
	pioa.setPeripheralA(PIO_PA22A_TXD1);
			
	pioa.disableInterrupt(0xFFFFFFFF);
			
	pioa.setOutput(LED);
	pioa.setOutput(SERVO1);
	pioa.setOutput(SERVO2);
	pioa.setOutput(SERVO3);
	pioa.setOutput(SERVO4);
	
	pioa.setInput(GEAR);
	pioa.setInput(THROTTLE);
	pioa.setInput(AILERON);
	pioa.setInput(ELEVATOR);
	pioa.setInput(RUDDER);
	pioa.setInput(INTA);
	
	pioa.clear(LED);

	NVIC_DisableIRQ(PIOA_IRQn);
	NVIC_ClearPendingIRQ(PIOA_IRQn);
	NVIC_SetPriority(PIOA_IRQn, 0);
	NVIC_EnableIRQ(PIOA_IRQn);
	
	pioa.configureRisingEdgeInterrupt(GEAR); // rising edge
	pioa.enableInterrupt(GEAR);

	pioa.configureRisingEdgeInterrupt(THROTTLE); // rising edge
	pioa.enableInterrupt(THROTTLE);

	pioa.configureRisingEdgeInterrupt(AILERON); // rising edge
	pioa.enableInterrupt(AILERON);

	pioa.configureRisingEdgeInterrupt(ELEVATOR); // rising edge
	pioa.enableInterrupt(ELEVATOR);

	pioa.configureRisingEdgeInterrupt(RUDDER); // rising edge
	pioa.enableInterrupt(RUDDER);

	pioa.configureRisingEdgeInterrupt(INTA); // rising edge
	pioa.enableInterrupt(INTA);
		

	// channel 0
	cTC tc0(TC0, 0);
	tc0.init(TC_CMR_TCCLKS_TIMER_CLOCK2); // TIMER_CLOCK2 = SYSCLOCK/8 this is ok to have 50Hz signal within 0 - 0xffff range
	//tc0.writeRC(0xffff);
	//tc0.start();
	//NVIC_EnableIRQ(TC0_IRQn); //channel 0 irq
	//tc0.enableInterrupt(TC_IER_CPCS | TC_IER_COVFS);
	
	cTC tc1(TC0, 1);
	tc1.init(TC_CMR_TCCLKS_TIMER_CLOCK1); // TIMER_CLOCK1 = SYSCLOCK/2
	tc1.writeRC(0xffff);
	//tc1.start();

	//NVIC_DisableIRQ(TC1_IRQn);
	//NVIC_ClearPendingIRQ(TC1_IRQn);
	//NVIC_SetPriority(TC1_IRQn, 0);
	//NVIC_EnableIRQ(TC1_IRQn);
	//tc1.enableInterrupt(TC_IER_CPCS);







	cUART uart(UART0);
	
	sam_uart_opt_t opts;
	opts.ul_baudrate = 230400;
//	opts.ul_baudrate = 9600;
	opts.ul_mck = SystemCoreClock;
	opts.ul_mode = (UART_MR_PAR_Msk & UART_MR_PAR_NO) | (UART_MR_CHMODE_Msk & UART_MR_CHMODE_NORMAL);
	
	uart.init(&opts);
	
	uart.enableTX();
	uart.enableRX();
	
	cPDC pdc(PDC_UART0);
	pdc.enableTransfer(PERIPH_PTCR_TXTEN);
	
	uart.enableInterrupt(UART_IER_RXRDY);
	NVIC_EnableIRQ(UART0_IRQn);

	cTWI twi(TWI0); // two wire
	
	twi_options_t topts = {0};
	topts.chip = 0;
	topts.master_clk = CHIP_FREQ_CPU_MAX;
	topts.speed = 100000;
	
	//twi.enableMasterMode();
	if(twi.init_master(&topts) != TWI_SUCCESS)
	{
		pioa.set(PIO_PA20);
	}
	
	for(int i=0x00; i<0x7f; i++)
	{
		if(twi.probe(i) == TWI_SUCCESS)
		{
			pioa.set(PIO_PA20);
		}
	}
	
	cMPU6050 mpu(twi);
	cHMC5883 hmc(twi);

	bool resu = mpu.testConnection();
	mpu.initialize(hmc);
	
	delayms(100);
	
	char *ptr=0;
	sSensorData sensorData;
	
	//tRawSensorData sd;
	
    gState.accel_ref_x = 0;
    gState.accel_ref_y = 0;
    gState.accel_ref_z = 1.0;

    gState.accel_var = 0.01; // smaller means more sensitive

    gState.mag_ref_x = 0;
    gState.mag_ref_y = 0;
    gState.mag_ref_z = 0;

    gState.mag_var = 0.001;

    gState.process_var = 0.5; // 0.1	
		
	tc1.start();
		
	gSensorTime = 0;
	while(1) {
		if(gSensorReady) {
			gSensorReady = false;
			
			bool resMPU = mpu.readData(sensorData);

			if(mpu.mSamples == 128) {
				float length = sqrt(sensorData.mMagX*sensorData.mMagX + sensorData.mMagY*sensorData.mMagY + sensorData.mMagZ*sensorData.mMagZ);
				gState.mag_ref_x = sensorData.mMagX / length;
				gState.mag_ref_y = sensorData.mMagY / length;
				gState.mag_ref_z = sensorData.mMagZ / length;
			}

			gEKF.estimateStates(gState, sensorData);


			sensorData.mQ0 = gState.qib.a;
			sensorData.mQ1 = gState.qib.b;
			sensorData.mQ2 = gState.qib.c;
			sensorData.mQ3 = gState.qib.d;
			
			//MadgwickAHRSupdate(sensorData);
			//MadgwickAHRSupdateIMU(sensorData);
			//sensorData.mQ0 = q0;
			//sensorData.mQ1 = q1;
			//sensorData.mQ2 = q2;
			//sensorData.mQ3 = q3;
		}
		
		if(gSensorTime >= 30) {		
			gSensorTime=0;
			
			//char mystr[30]={0};
			//sprintf(mystr, "%d,%d,%d\n", sensorData.mMagX, sensorData.mMagY, sensorData.mMagZ);
			//pdc.sendUART(mystr, strlen(mystr));
			
			ptr = (char*)&sensorData;
			pdc.sendUART(ptr, sizeof(sSensorData)); // length is important here!
		}
		
		if(gBuffer.size() == sizeof(sSettings)) {
			uint8_t data[sizeof(sSettings)] = {0};
			int p=0;
			while(gBuffer.get(data[p])) {
				p++;
			}
			if(data[0] == 'a' && data[1] == 'b' && data[2] == 'c') {
				sSettings *s = (sSettings*)data;
				gState.mag_cal = s->mMagCal;
				gState.beta_mag_x = s->mMagXoffset;
				gState.beta_mag_y = s->mMagYoffset;
				gState.beta_mag_z = s->mMagZoffset;
				
				
			}
		}
		
	}
	
	// dmp stuff... dmp may be able to work with the HMC5883, but now I use Madgwick's algo
	//mpu.reset();
	//delayms(50);
	//mpu.dmpInitialize();
	
	while(1) {
		if(gDiffTime > 0x2F00) { // 3c2b, 21d7

			gFlightMode = eFM_AUTOPILOT;
			if(gPrevFlightMode != gFlightMode) {
				gPrevFlightMode = gFlightMode;
				pioa.clear(LED);
				enableAutopilot();
			}
			
		} else {

			gFlightMode = eFM_MANUAL;
			if(gPrevFlightMode != gFlightMode) {
				gPrevFlightMode = gFlightMode;
				pioa.set(LED);
				disableAutopilot();
			}

		}
	}




}


// enables PWM signals on output pins
void enableAutopilot()
{
	cPWM pwm(PWM); // PWM base
	
	cPIO pioa(PIOA);
	
	// pwm for the motor and servos
	pioa.setPeripheralB(PIO_PA11B_PWMH0);
	pioa.setPeripheralB(PIO_PA12B_PWMH1);
	pioa.setPeripheralB(PIO_PA13B_PWMH2);
	pioa.setPeripheralB(PIO_PA14B_PWMH3);	
	
	pwm.channelDisable(PWM_CHANNEL_MOT1);
	pwm.channelDisable(PWM_CHANNEL_MOT2);
	pwm.channelDisable(PWM_CHANNEL_MOT3);
	pwm.channelDisable(PWM_CHANNEL_MOT4);
	delayms(10);

	pwm_clock_t clock_setting = {
		.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
		.ul_clkb = 0,
		.ul_mck = SystemCoreClock
	};

	pwm.init(&clock_setting);

#undef INIT_SERVO
#define INIT_SERVO(servo, pwmChannel)		\	
	servo.alignment = PWM_ALIGN_LEFT;		\
	servo.polarity = PWM_HIGH;				\
	servo.ul_prescaler = PWM_CMR_CPRE_CLKA;	\
	servo.ul_period = PERIOD_VALUE;			\
	servo.ul_duty = SERVO_NEUTRAL;			\
	servo.channel = pwmChannel;				\
	pwm.channelInit(&servo);				\
	pwm.channelEnable(pwmChannel);			\

	INIT_SERVO(gServo1, PWM_CHANNEL_MOT1);
	INIT_SERVO(gServo2, PWM_CHANNEL_MOT2);
	INIT_SERVO(gServo3, PWM_CHANNEL_MOT3);
	INIT_SERVO(gServo4, PWM_CHANNEL_MOT4);
	
	delayms(10);
	
	if(pwm.updateDuty(&gServo1, SERVO_NEUTRAL) != PWM_INVALID_ARGUMENT) {
		pioa.set(PIO_PA20);
	}
	if(pwm.updateDuty(&gServo2, SERVO_NEUTRAL) != PWM_INVALID_ARGUMENT) {
		pioa.set(PIO_PA20);
	}
	if(pwm.updateDuty(&gServo3, SERVO_NEUTRAL) != PWM_INVALID_ARGUMENT) {
		pioa.set(PIO_PA20);
	}
	if(pwm.updateDuty(&gServo4, SERVO_NEUTRAL) != PWM_INVALID_ARGUMENT) {
		pioa.set(PIO_PA20);
	}

}

void disableAutopilot()
{
	cPIO pioa(PIOA);
	cPWM pwm(PWM); // PWM base
	pwm.channelDisable(PWM_CHANNEL_MOT1);
	pwm.channelDisable(PWM_CHANNEL_MOT2);
	pwm.channelDisable(PWM_CHANNEL_MOT3);
	pwm.channelDisable(PWM_CHANNEL_MOT4);
	delayms(10);
	pioa.setOutput(SERVO1);
	pioa.setOutput(SERVO2);
	pioa.setOutput(SERVO3);
	pioa.setOutput(SERVO4);	
}

void MadgwickAHRSupdate(sSensorData &sd) {
	float gx = sd.mGyroX;
	float gy = sd.mGyroY;
	float gz = sd.mGyroZ;

	float ax = sd.mAccX;
	float ay = sd.mAccY;
	float az = sd.mAccZ;

	float mx = sd.mMagX;
	float my = sd.mMagY;
	float mz = sd.mMagZ;

	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		//MadgwickAHRSupdateIMU(sd);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

void MadgwickAHRSupdateIMU(sSensorData &sd) {
	
	float gx = sd.mGyroX;
	float gy = sd.mGyroY;
	float gz = sd.mGyroZ;

	float ax = sd.mAccX;
	float ay = sd.mAccY;
	float az = sd.mAccZ;	
	
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

/*
float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
*/

float invSqrt(float x){
   uint32_t i = 0x5F1F1412 - (*(uint32_t*)&x >> 1);
   float tmp = *(float*)&i;
   return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
}
