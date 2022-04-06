/*
 * GccApplication1.c
 *
 * Created: 20.07.2020 03:12:59
 * Author : Oleg
 */ 

#include <avr/io.h>
#include <inttypes.h>
#include <stdbool.h>
#include <avr/interrupt.h>

#define F_CPU 8000000UL

/* Port's defines */
	#define R_KEY 0
	#define L_KEY 1

	#define R_LAMP 0
	#define L_LAMP 1
	
	#define KeyPinReg PINA
	#define LampPinReg PINB
	#define LampPortReg PORTB

/* TC0 Presets */
	#define TC0_PRSC_VAL 1024
	
	#if TC0_PRSC_VAL == 1
		#define TC0_PRSC_MODE 1
	#elif TC0_PRSC_VAL == 8
		#define TC0_PRSC_MODE 2
	#elif TC0_PRSC_VAL == 64
		#define TC0_PRSC_MODE 3
	#elif TC0_PRSC_VAL == 256
		#define TC0_PRSC_MODE 4
	#elif TC0_PRSC_VAL == 1024
		#define TC0_PRSC_MODE 5
	#elif TC0_PRSC_VAL == 0
		#define TC0_PRSC_MODE 0
	#else
		#define TC0_PRSC_VAL 1
	#endif

	#define TC0_INT_FREQ 50  /* TC0 overflow Interrupt frequency, Hz */
	
	#define TC0_CNT_FREQ (F_CPU/TC0_PRSC_VAL)
	#define TC0_STRT_VAL (256-(TC0_CNT_FREQ/TC0_INT_FREQ)-1)
	
/* Software Timers Presets */
	#define LightingTimerTimeoutSeconds 3600UL
	#define LightingTimerStartValue (LightingTimerTimeoutSeconds*TC0_INT_FREQ)

/* User flags */
	#define LightingTimerFlagEnable 0
	//#define __ 2
	//#define __ 3
	//#define __ 4
	//#define __ 5
	//#define __ 6
	//#define __ 7

	volatile uint8_t UserFlags = 0;
	
/* Global Variables */
	volatile uint32_t LightingTimer = 0;		// Software timer for automatic switching off of lamps
	
	

EMPTY_INTERRUPT (EXT_INT0_vect)		/* External Interrupt Request 0 */
EMPTY_INTERRUPT (PCINT0_vect)		/* Pin Change Interrupt Request 0 */
EMPTY_INTERRUPT (PCINT1_vect)		/* Pin Change Interrupt Request 1 */
EMPTY_INTERRUPT (WATCHDOG_vect)		/* Watchdog Time-out */
EMPTY_INTERRUPT (TIM1_CAPT_vect)	/* Timer/Counter1 Capture Event */
EMPTY_INTERRUPT (TIM1_COMPA_vect)	/* Timer/Counter1 Compare Match A */
EMPTY_INTERRUPT (TIM1_COMPB_vect)	/* Timer/Counter1 Compare Match B */
EMPTY_INTERRUPT (TIM1_OVF_vect)		/* Timer/Counter1 Overflow */
EMPTY_INTERRUPT (TIM0_COMPA_vect)	/* Timer/Counter0 Compare Match A */
EMPTY_INTERRUPT (TIM0_COMPB_vect)	/* Timer/Counter0 Compare Match B */
//EMPTY_INTERRUPT (TIM0_OVF_vect)		/* Timer/Counter0 Overflow */
EMPTY_INTERRUPT (ANA_COMP_vect)		/* Analog Comparator */
EMPTY_INTERRUPT (ADC_vect)			/* ADC Conversion Complete */
EMPTY_INTERRUPT (EE_RDY_vect)		/* EEPROM Ready */
EMPTY_INTERRUPT (USI_STR_vect)		/* USI START */
EMPTY_INTERRUPT (USI_OVF_vect)		/* USI Overflow */

void InternalModulesInitialization (void) 
{
	/* Configure Ports */
		DDRA	&=	~(1<<R_KEY)	| ~(1<<L_KEY) ;
		
		PORTA	|= (1<<R_KEY)	| (1<<L_KEY) ;
		
		DDRB	|= (1<<R_LAMP)	| (1<<L_LAMP) ;
				
		PORTB	&= ~(1<<R_LAMP) | ~(1<<L_LAMP) ;
		
	// PRR – Power Reduction Register
	/*
		Bit 3 – PRTIM1: Power Reduction Timer/Counter1. Writing a logic one to this bit shuts down the Timer/Counter1 module.
		Bit 2 – PRTIM0: Power Reduction Timer/Counter0. Writing a logic one to this bit shuts down the Timer/Counter0 module.
		Bit 1 – PRUSI: Power Reduction USI. Writing a logic one to this bit shuts down the USI by stopping the clock to the module.
		Bit 0 – PRADC: Power Reduction ADC. Writing a logic one to this bit shuts down the ADC.
	*/
		PRR |= 1<<PRTIM1 | 0<<PRTIM0 | 1<<PRUSI | 1<<PRADC;

	// MCUCR – MCU Control Register:
		/*
		BODS = 1 - Disable BOD in POwer-Down and Stand-By
		PUD = 1 - Pull-Up Resistors Disable.
		SE = 1 - Sleep Enable.
		SM(1,0) = 00;01;10 - Idle; ADC Noice Reduction; Power-down.
		ISC0(1,0) = 00...11 - Low level | Any logical change | Falling edge | Rising edge
		*/
		//MCUCR |= 0<<BODS | 0<<PUD | 0<<SE | 0<<SM1 | 0<<SM0 | 0<<ISC01 | 0<<ISC00;

	// GIMSK - General Interrupt Mask Register:
		/*
		INT0 = 1 - External Interrupt Request 0 Enable
		PCIE1 = 1 - Pin Change Interrupt 1 Enable
		PCIE0 = 1 - Pin Change Interrupt 0 Enable
		*/
		//GIMSK |= 0<<INT0 | 0<<PCIE1 | 1<<PCIE0
	
	// PCMSK(1,0) – Pin Change Mask Registers:
		/*
		PCMSK1:
			3. PCINT11 - 1 = Enable interrupts on individual pin(s)
			2. PCINT10 - 1 = Enable interrupts on individual pin(s)
			1. PCINT9 - 1 = Enable interrupts on individual pin(s)
			0. PCINT8 - 1 = Enable interrupts on individual pin(s)
			
		PCMSK0:
			PCINT7...PCINT0 - 1 = Enable interrupts on individual pin(s)
		*/
		//PCMSK0 |= 1<<PCINT0 | 1<<PCINT1 | 1<<PCINT2 | 1<<PCINT3 | 0<<PCINT4 | 0<<PCINT5 | 0<<PCINT6 | 0<<PCINT7;
	
	// TCC0 initialization:
		/*
		TCC0 registers description:
		
			TCCR0A:
				COM0A(1,0) = 00...11 - Compare Output Mode if Compare A is match
				COM0B(1,0) = 00...11 - Compare Output Mode if Compare B is match
				WGM0(1,0) = 00...11 - Waveform Generation Mode
			
			TCCR0B:
				WGM0(2) = 0...1 - Waveform Generation Mode
				CS0(2,1,0) = 000...111 - T/C0 Clock Prescaler Mode - T/C0 stopped, 1, 8, 64, 256, 1024, ?0 fall, ?0 rise
				FOC0A - Force Output Compare A
				FOC0B - Force Output Compare B
			
			TCNT0 - Timer/Counter count register
			OCR0A - Output Compare Register A
			OCR0B - Output Compare Register B

			TIMSK0 - Timer/Counter Interrupt Mask Register:
				OCIE0A = 1 - Timer/Counter Compare Match A interrupt is enabled.
				OCIE0B = 1 - Timer/Counter Compare Match B interrupt is enabled.
				TOIE0 = 1 - Timer/Counter0 Overflow interrupt is enabled.

			TIFR0 - Timer/Counter 0 Interrupt Flag Register:
				OCF0A - Timer/Counter Compare Match A interrupt Flag
				OCF0B - Timer/Counter Compare Match B interrupt Flag
				TOV0 - Timer/Counter0 Overflow interrupt Flag

			GTCCR - General Timer/Counter Control Register:
				TSM = 1 - TC0 in sync. mode
				PSR10 = 1 - Timer/Counter0 prescaler is Reset.
		*/
		TCCR0A |= 0<<COM0A1 | 0<<COM0A0 | 0<<COM0B1 | 0<<COM0B0 | 0<<WGM01 | 0<<WGM00;
		TIMSK0 |= 0<<OCIE0A | 0<<OCIE0B | 1<<TOIE0;
		TCNT0 = TC0_STRT_VAL;
		GTCCR |= (0<<TSM)|(1<<PSR10); // Reset prescaler
			
		TCCR0B |=	((TC0_PRSC_MODE & 0b00000100)>>CS02) << CS02 |
					((TC0_PRSC_MODE & 0b00000010)>>CS01) << CS01 |
					((TC0_PRSC_MODE & 0b00000001)>>CS00) << CS00 ;


	// ADC initialization.
		/* 
		ADC registers description:
		
			ADMUX:
				REFS(1,0) = 00 - Vcc | 01 - AREF | 10 - Internal 1V1 voltage reference
				MUX(5,4,3,2,1,0) = 000000...111111 - input select:
								000000...000111 - adc0...adc7
								001000...011111 - reserved for differential channels
								100000 - AGND
								100001 - 1V1 voltage reference
								100010 - temperature sensor
								100011...100111 - reserved for offset calibration
								101000...111111 - reserved for reversal differential channels

			ADCSRA:
				ADEN = 1 - ADC Enable
				ADSC = 1 - Start Single/First Conversation
				ADATE = 1 - Auto Triggering of the ADC is enabled
				ADIF - ADC Interrupt Flag
				ADIE = 1 - ADC Conversation Complete Interrupt is activated
				Clock prescaler: ADPS(2,1,0) = 000...111 - 2, 2, 4, 8, 16, 32, 64, 128

			ADCSRB:
				BIN = 1 - Enable Bipolar Input Mode
				ACME = 1 - When the ADC is switched off (ADEN in ADCSRA is zero), the ADC multiplexer selects the negative input to the Analog Comparator.
				ADLAR = 0 - Right Adjust Result; 1 - Left Adjust Result
				ADC Auto Trigger Source: ADTS(2,1,0) = 000...111 - Free Running Mode | An. Comp. | INT0 | T/C0 comp. match A | T/C0 overflow | T/C1 comp. match B | T/C1 overflow| T/C1 capture event

			DIDR0:
				ADC7D..ADC0D: ADC7..0 Digital Input Disable
	*/
		//ADCSRA |= 1<<ADEN | 0<<ADSC | 0<<ADATE | 1<<ADIE | 1<<ADPS2 | 1<<ADPS1 | 0<<ADPS0;
		//ADCSRB |= 0<<BIN | 0<<ADLAR | 0<<ACME | 0<<ADTS2 | 0<<ADTS1 | 0<<ADTS0;

	// Analog Comparator initialization:
		/*	
		Analog Comparator registers description:
		
			ACSR:
				ACD = 1 - Analog Comparator Disable. When this bit is written logic one, the power to the Analog Comparator is switched off.
				ACBG = 1 - A fixed bandgap reference voltage is connected to the positive comparator input; 0 - The Positive Comparator Input connected to a IO Pin.
				ACO - Analog Comparator Output.
				ACI - Analog Comparator Interrupt Flag.
				ACIE - Analog Comparator Interrupt Enable.
				ACIC - 1 - enable the input capture function in Timer/Counter1 to be triggered by the analog comparator.
				ACIS(1,0) - Analog Comparator Interrupt Mode Select: 00 - when output is toggle | 10 - on Falling Output Edge | 11 - on Rising Output Edge.
		*/			
		ACSR |= (1<<ACD)|
				(1<<ACBG)|
				(0<<ACIE)|
				(0<<ACIC)|
				(0<<ACIS1)|
				(0<<ACIS0);	// Comparator disabled.
				
	sei();
}

void StartLightingTimer(void)
{
	if (LightingTimer <= LightingTimerStartValue/2)
	{
		LightingTimer = LightingTimerStartValue;
		UserFlags |= 1<<LightingTimerFlagEnable;
	}
}

void StopLightingTimer(void)
{
	UserFlags &= ~(1<<LightingTimerFlagEnable);
	LightingTimer = 0;
}


ISR (TIM0_OVF_vect)		/* Timer/Counter0 Overflow */
{
	TCNT0 = TC0_STRT_VAL;
	
	static uint_least8_t PriorKeyState = 0b00000011;
	bool KeyPressed = false;
	
	uint8_t KeyState = KeyPinReg; //& ((1<<L_KEY) | (1<<R_KEY));	// Get key state
	
	if (!(KeyState & (1<<R_KEY)))
	{
		if (PriorKeyState & (1<<R_KEY))
		{
			LampPinReg |= (1<<R_KEY);
			KeyPressed = true;
		}
	}
	
	if (!(KeyState & (1<<L_KEY)))
	{
		if (PriorKeyState & (1<<L_KEY))
		{
			LampPinReg |= (1<<L_KEY);
			KeyPressed = true;
		}
	}

	
	PriorKeyState = KeyState;		// Save key state for future checks
	
	if (KeyPressed == true)
	{
		asm("nop" : : );
		if ( (LampPortReg & (1<<L_LAMP)) | (LampPortReg & (1<<R_LAMP)) ) StartLightingTimer(); // Start the timer if a button was pressed and a lamp turned on
	}
	
	if ( (UserFlags & (1<<LightingTimerFlagEnable)) & (!(LampPortReg & (1<<L_LAMP))) & (!(LampPortReg & (1<<R_LAMP))) ) StopLightingTimer();

	if (UserFlags & 1<<LightingTimerFlagEnable)
	{
		if (LightingTimer > 0) LightingTimer--;
		else
		{
			LampPortReg &= ~(1<<L_LAMP) & ~(1<<R_LAMP);
			//LampPortReg &= ~(1<<R_LAMP);	// Turn off both lamps
			StopLightingTimer();
		}
	}
	
}

void main(void)
{
	InternalModulesInitialization();

    while (1) 
    {
    }
}

