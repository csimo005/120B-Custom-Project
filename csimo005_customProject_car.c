/*
 * Author: Cody Simons
 * Email: csimo005@ucr.edu
 * Lab Section: 
 *
 * File: csimo005_customProject_cor.c
 * Description: This the code that will run on the car. It will connect to the 
 * remote. It will periodically recieve data packets from the remote and control
 * the car accordingly.
 *
 * Data Packet: 5 bytes,
 *                byte0: Servo Postion, High Byte
 *                byte1: Servo Postion, Low Byte
 *                byte2: Motor Speed, High Byte
 *                byte3: Motor Speed, Low Byte
 *                byte4: bit0 & bit1 used for turn signals
 *
 * Hardware Description: 
 *
 * I acknowledge all content contained herein, excluding template or example
 * code, is my own original work.
 */ 
 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <stdlib.h>

#define R_REGISTER(R) (R&0x1F)
#define W_REGISTER(R) ((R&0x1f)|(1<<5))
#define R_RX_PAYLOAD 0x61
#define W_TX_PAYLOAD 0xA0
#define FLUSH_TX 0xE1
#define FLUSH_RX 0xE2
#define REUSE_TX_PL 0xE3
#define R_RX_PL_WID 0x60
#define W_ACK_PAYLOAD(P) ((0xA8)|(P&0x07))
#define W_TX_PAYLOAD_NOACK 0xB0
#define NOP 0xFF

#define MOSI 5
#define MISO 6
#define SCK 7
#define CSN 0
#define CE 1
#define IRQ 2

// TimerISR() sets this to 1. C programmer should clear to 0.
volatile unsigned char TimerFlag = 0;

// Internal variables for mapping AVR's ISR to our cleaner TimerISR model.
unsigned long _avr_timer_M = 1; // Start count from here, down to 0. Default 1 ms.
unsigned long _avr_timer_cntcurr = 0; // Current internal count of 1ms ticks

void TimerOn() {
	// AVR timer/counter controller register TCCR1
	// bit3 = 1: CTC mode (clear timer on compare)
	// bit2bit1bit0=011: pre-scaler /64
	// 00001011: 0x0B
	// SO, 8 MHz clock or 8,000,000 /64 = 125,000 ticks/s
	// Thus, TCNT1 register will count at 125,000 ticks/s
	TCCR3B = 0x0B;
	// AVR output compare register OCR1A.
	// Timer interrupt will be generated when TCNT1==OCR1A
	// We want a 1 ms tick. 0.001 s * 125,000 ticks/s = 125
	// So when TCNT1 register equals 125,
	// 1 ms has passed. Thus, we compare to 125.
	OCR3A = 125;
	// AVR timer interrupt mask register
	// bit1: OCIE1A -- enables compare match interrupt
	TIMSK3 = 1<<OCIE3A;
	//Initialize avr counter
	TCNT1=0;
	// TimerISR will be called every _avr_timer_cntcurr milliseconds
	_avr_timer_cntcurr = _avr_timer_M;
	//Enable global interrupts: 0x80: 1000000
	SREG |= 0x80;
}
void TimerOff() {
	// bit3bit1bit0=000: timer off
	TCCR3B = 0x00;
}
void TimerISR() {
	TimerFlag = 1;
}
// In our approach, the C programmer does not touch this ISR, but rather TimerISR()
ISR(TIMER3_COMPA_vect) {
	// CPU automatically calls when TCNT1 == OCR1
	// (every 1 ms per TimerOn settings)
	// Count down to 0 rather than up to TOP (results in a more efficient comparison)
	_avr_timer_cntcurr--;
	if (_avr_timer_cntcurr == 0) {
		// Call the ISR that the user uses
		TimerISR();
		_avr_timer_cntcurr = _avr_timer_M;
	}
}
void delay_ms(int miliSec) //for 8 Mhz crystal
{
	int i,j;
	for(i=0;i<miliSec;i++)
	for(j=0;j<775;j++)
	{
		asm("nop");
	}
}

// Set TimerISR() to tick every M ms
void TimerSet(unsigned long M) {
	_avr_timer_M = M;
	_avr_timer_cntcurr = _avr_timer_M;
}
// Master code
void SPI_MasterInit(void) {
	// Set DDRB to have MOSI, SCK, and SS as output and MISO as input
	DDRB|=(1<<SCK)|(1<<MOSI)|(1<<4);
	// Set SPCR register to enable SPI, enable master, and use SCK frequency
	//   of fosc/16  (pg. 168)
	SPCR|=(1<<SPE)|(1<<MSTR)|(1<<SPR0);
	// Make sure global interrupts are enabled on SREG register (pg. 9)
	SREG|=0x80;
}

void SPI_MasterTransmit(unsigned char *txData, unsigned char *rxData, volatile uint8_t *port, unsigned char pin) {
	// data in SPDR will be transmitted, e.g. SPDR = cData;
	unsigned char bytes = sizeof(txData);
	*port&=~(1<<pin);
	for(unsigned char i=0;i<bytes;++i) {
			SPDR=txData[i];
			while(!(SPSR & (1<<SPIF)));
			rxData[i]=SPDR;
	}
	*port|=(1<<pin);
}

unsigned char writeRegister(unsigned char regAddr, unsigned char regValue[], unsigned char bytes) {
	PORTB&=~(1<<CSN);
	SPDR=W_REGISTER(regAddr);
	while(!(SPSR & (1<<SPIF)));
	unsigned char status = SPDR;
	for(unsigned char i=0;i<bytes;++i) {
		SPDR=regValue[i];
		while(!(SPSR & (1<<SPIF)));
	}
	PORTB|=(1<<CSN);
	return status;
}
unsigned char readRegister(unsigned char regAddr, unsigned char regValue[], unsigned char bytes) {
	PORTB&=~(1<<CSN);
	SPDR=R_REGISTER(regAddr);
	while(!(SPSR & (1<<SPIF)));
	unsigned char status = SPDR;
	for(unsigned char i=0;i<bytes;++i) {
		SPDR=regValue[i];
		while(!(SPSR & (1<<SPIF)));
		regValue[i]=SPDR;
	}
	PORTB|=(1<<CSN);
	return status;
}
unsigned char writePayloadTx(unsigned char msg[], unsigned char bytes) {
	PORTB&=~(1<<CSN);
	SPDR=W_TX_PAYLOAD;
	while(!(SPSR & (1<<SPIF)));
	unsigned char status = SPDR;
	for(unsigned char i=0;i<bytes;++i) {
		SPDR=msg[i];
		while(!(SPSR & (1<<SPIF)));
	}
	PORTB|=(1<<CSN);
	return status;
}
unsigned char writePayloadTxNoack(unsigned char msg[], unsigned char bytes) {
	PORTB&=~(1<<CSN);
	SPDR=W_TX_PAYLOAD_NOACK;
	while(!(SPSR & (1<<SPIF)));
	unsigned char status = SPDR;
	for(unsigned char i=0;i<bytes;++i) {
		SPDR=msg[i];
		while(!(SPSR & (1<<SPIF)));
	}
	PORTB|=(1<<CSN);
	return status;
}
unsigned char readPayloadRx(unsigned char msg[], unsigned char bytes) {
	PORTB&=~(1<<CSN);
	SPDR=R_RX_PAYLOAD;
	while(!(SPSR & (1<<SPIF)));
	unsigned char status = SPDR;
	for(unsigned char i=0;i<bytes;++i) {
		SPDR=msg[i];
		while(!(SPSR & (1<<SPIF)));
		msg[i]=SPDR;
	}
	PORTB|=(1<<CSN);
	return status;
}
unsigned char flushTx() {
	unsigned char status;
	PORTB&=~(1<<CSN);
	SPDR=FLUSH_TX;
	while(!(SPSR & (1<<SPIF)));
	status=SPDR;
	PORTB|=(1<<CSN);
	return status;
}
unsigned char flushRx() {
	unsigned char status;
	PORTB&=~(1<<CSN);
	SPDR=FLUSH_RX;
	while(!(SPSR & (1<<SPIF)));
	status=SPDR;
	PORTB|=(1<<CSN);
	return status;
}

void Radio_TxInit() {
	SPI_MasterInit();
	DDRB|=(1<<CSN)|(1<<CE);
	PORTB|=(1<<IRQ)|(1<<CSN);
	delay_ms(5); //settling time specified in the data sheet
	unsigned char oneByte[1]={0x72};
	unsigned char threeByte[3]={0x18, 0x04, 0x96};
	writeRegister(0x00, oneByte, 1); //set config reg to power up into tx mode
	oneByte[0]=0x01;
	writeRegister(0x03, oneByte, 1); //set address length to 3 bytes
	writeRegister(0x10, threeByte, 3); //set address of transmit to random number, in this case my birthday
	oneByte[0]=76;
	writeRegister(0x05, oneByte, 1); //set to channel 76
	oneByte[0]=0x00;
	writeRegister(0x01, oneByte, 1); //turn off Auto Acknowledge, one way communication okay if a few packets get dropped
	writeRegister(0x04, oneByte, 1); //turn off retransmit, not ideal for remote control
	oneByte[0]=0x01;
	writeRegister(0x1D, oneByte, 1); //enable W_TX_PAYLOAD_NOACK
	flushTx();
	PORTB|=(1<<CE);
}

void Radio_TxTransmitt(unsigned char msg[], unsigned char bytes) {
	writePayloadTxNoack(msg, bytes);
}

void Radio_RxInit() {
	SPI_MasterInit();
	DDRB|=(1<<CSN)|(1<<CE);
	PORTB|=(1<<IRQ)|(1<<CSN);
	delay_ms(5);
	unsigned char oneByte[1]={0x33};
	unsigned char threeByte[3]={0x18, 0x04, 0x96};
	writeRegister(0x00, oneByte, 1); //set config reg to power up into rx mode
	oneByte[0]=0x01;
	writeRegister(0x03, oneByte, 1); //set address length to 3 bytes
	writeRegister(0x0A, threeByte, 3); //set address of pipe 0 to random number, in this case my birthday
	oneByte[0]=76;
	writeRegister(0x05, oneByte, 1); //set channel to 76
	oneByte[0]=0x00;
	writeRegister(0x01, oneByte, 1); //turn off auto acknowledge
	oneByte[0]=5;
	writeRegister(0x11, oneByte, 1); //set packet length for pipe 0
	flushRx();
	PORTB|=(1<<CE);
}

void Radio_RxReset() {
	PORTB&=~0x08;
	delay_ms(10);
	PORTB|=0x08;
	Radio_RxInit();
}

unsigned char Radio_RxReady() {
	unsigned char FIFOStatus[1];
	readRegister(0x17, FIFOStatus, 1);
	if((FIFOStatus[0]&0x01)==0) {
		return(1);
	}
	return(0);
}

void Radio_RxRead(unsigned char data[], unsigned char bytes) {
	readPayloadRx(data, bytes);
}

void servoInit(){
	DDRD|=0x20; //servo is on port d pin 5
	//WGM13 WGM12 WGM11 WGM10: 1110, fast pwm, reset at TCMNT1==ICR1
	//COM1A1 COM1A0: 11, fast pwm inverted output, low->high at ICR1==OCR1A
	//CS11 CS10: 10, for a prescaler of /8
	TCCR1A |= 1<<WGM11 | 1<<COM1A1 | 1<<COM1A0;
	TCCR1B |= 1<<WGM13 | 1<<WGM12 | 0<<CS12 | 1<<CS11 | 0<<CS10;
	//base clock = 8Mhz, after prescaller clock is 1Mhz
	//Servo expects a period of 20ms, or 0.02s
	//1Mhz = 0.0000001 ticks/sec
	//0.02/0.0000001 = 20000 ticks 
	//subtract 1 becuase you start at 0, 19999 ticks
	ICR1 = 19999;
	//defined by servo data sheet that center of servo corresponds to 1.52ms
	//becuase it is in invering mode go high with 1.52ms left
	OCR1A = ICR1 - 1520;
}

void servoSetPos(unsigned short position) {
    //sanatize value, max is 1023 or 10 bits
    position&=0x03FF;
	if(position <= 522 && position >= 502) {//deadzone center of range
	        //center of the servo, value from datasheet
	        position = 512;
	}
	if(position<170) {
		position=170;
	}
	if(position>850) {
		position=850;
	}
	//max 2000, min 1000, so to determine value based input
	//min+(max-min)*(position/1023)
	//change order of operation to avoid fractions and use left
	//shift instead of divide
	//frac is a long to avoid overflow
	unsigned long frac = (((unsigned long)position-170)*1400) / 850;
	OCR1A = ICR1 - (frac + 800);
}

void motorInit() {
	DDRD|=0xC0;
	TCCR2A|=(1<<COM2A1)|(1<<COM2B1)|(1<<WGM21)|(1<<WGM20);
	TCCR2B|=(1<<CS20);
	OCR2A=0xFF;
	OCR2B=0xFF;
}

void setMotorSpeed(unsigned short speed) {
	if(speed>850) {
		OCR2A=0xFF;
		OCR2B=0x00;
	} else if(speed>517) {
		OCR2A=0xFF;
		unsigned char duty=((850-(unsigned long)speed)*255)/(850-517);
		duty=((long)duty*50)/255+175;
		OCR2B=duty;
	} else if(speed>506) {
		OCR2A=0xFF;
		OCR2B=0xFF;
	} else if(speed>169) {
		unsigned char duty=255-((506-(unsigned long)speed)*255)/(506-170);
		duty=((long)duty*50)/255+175;
		OCR2A=duty;
		OCR2B=0xFF;
	} else {
		OCR2A=0x00;
		OCR2B=0xFF;
	}
}

typedef struct task_{
	signed char state;
	unsigned long int period;
	unsigned long int elapsedTime;
	int (*TickFct)(int);	
} task;

enum lightStates{lightInit, lightOff, lightLeft, lightRight,lightHazard};
	
unsigned char turnSignals;
unsigned char lights;	
	
int lightsTick(int state) {
	switch(state) {
		case lightInit: {
			lights=0;
			if(turnSignals==0x01) {
			        return(lightLeft);
			} else if(turnSignals==0x02) {
			        return(lightRight);
			} else if(turnSignals==0x03) {
			        return(lightHazard);
			} else {
			        return(lightOff);
			}
			break;
		}
		case lightOff: {
			if(turnSignals==0x01) {
			        return(lightLeft);
			} else if(turnSignals==0x02) {
			        return(lightRight);
			} else if(turnSignals==0x03) {
			        return(lightHazard);
			} else {
			        return(lightOff);
			}
			break;
		}
		case lightLeft: {
			if(turnSignals==0x01) {
			        lights=(lights==0xF0)?0x00:((lights<<1)|0x10);
			        return(lightLeft);
			} else if(turnSignals==0x02) {
			        lights=0x00;
			        return(lightRight);
			} else if(turnSignals==0x03) {
			        lights=0x00;
			        return(lightHazard);
			} else {
			        lights=0x00;
			        return(lightOff);
			}
			break;
		}
		case lightRight: {
			if(turnSignals==0x01) {
			        lights=0x00;
			        return(lightLeft);
			} else if(turnSignals==0x02) {
			        lights=(lights==0x0F)?0:((lights>>1)|0x08);
			        return(lightRight);
			} else if(turnSignals==0x03) {
			        lights=0x00;
			        return(lightHazard);
			} else {
			        lights=0x00;
			        return(lightOff);
			}
			break;
		}
		case lightHazard: {
			if(turnSignals==0x01) {
			        lights=0x00;
			        return(lightLeft);
			} else if(turnSignals==0x02) {
			        lights=0x00;
			        return(lightRight);
			} else if(turnSignals==0x03) {
			        lights=~lights;
			        return(lightHazard);
			} else {
              lights=0x00;
              return(lightOff);
      }
      break;
    }
    default: {
		return(lightInit);
      break;
    }
  }
}

//setLights is used to interface with to turn signals
//input is bit masked to isolate bit0 and bit1
void setLights(unsigned char signal) {
        signal&=0x03;
        turnSignals=signal;
}

enum radioStates{radioInit, radioRead, radioLost};

unsigned char droppedPackets;
unsigned char recievedPackets;

int radioTick(int state) {
  switch(state) {
    case radioInit: {
      droppedPackets=0;
	  recievedPackets=0;
      Radio_RxInit();
	  motorInit();
      servoInit();
      return(radioRead);
      break;
    }
    case radioRead: {
      if(Radio_RxReady()) {
        droppedPackets=0;
        unsigned char state[5];
        Radio_RxRead(state, 5);
		if(recievedPackets==4) {
			unsigned short servoPosition=(state[0]<<8)|state[1];
			servoSetPos(servoPosition);
			unsigned short motorSpeed=(state[2]<<8)|state[3];
			setMotorSpeed(motorSpeed);
			setLights(state[4]);
		} else {
			recievedPackets++;
		}
        return(radioRead);
      } else {
        droppedPackets++;
        if(droppedPackets==40) {
		  setMotorSpeed(512);
          servoSetPos(512);
          setLights(0x03);
		  droppedPackets=0;
		  recievedPackets=0;
          return(radioLost);
        }
        return(radioRead);
      }
      break;
    }
    case radioLost: {
      if(Radio_RxReady()) {
        droppedPackets=0;
        unsigned char state[5];
        Radio_RxRead(state, 5);
		if(recievedPackets==4) {
			unsigned short servoPosition=(state[0]<<8)|state[1];
			servoSetPos(servoPosition);
			PORTA=servoPosition>>2;
			unsigned short motorSpeed=(state[2]<<8)|state[3];
			setMotorSpeed(motorSpeed);
			setLights(state[4]);
			return(radioRead);
		} else {
			recievedPackets++;
		}
        return(radioLost);
      } else {
		  droppedPackets++;
		  if(droppedPackets==40) {
			  Radio_RxReset();
			  droppedPackets=0;
		  }
		  return(radioLost);
	  }
      break;
    }
    default: {
      return(radioInit);
      break;
    }
  }
}

int main(void)
{
  DDRA=0xFF; PORTA=0x00;
  DDRB=0x08; PORTB=0x08;
  task tasks[2];
  tasks[0].state=radioInit;
  tasks[0].period=25;
  tasks[0].elapsedTime=tasks[0].period;
  tasks[0].TickFct=&radioTick;
  tasks[1].state=lightInit;
  tasks[1].period=500;
  tasks[1].elapsedTime=tasks[1].period;
  tasks[1].TickFct=&lightsTick;
  TimerSet(25);
  TimerOn();
  servoInit();
  while(1){
    for(unsigned char i=0;i<2;++i) {
      if(tasks[i].elapsedTime>=tasks[i].period) {
        tasks[i].state=tasks[i].TickFct(tasks[i].state);
        tasks[i].elapsedTime=0;
      }
      tasks[i].elapsedTime+=25;
    }
	PORTA=lights;
    while(!TimerFlag);
    TimerFlag=0;
  }
  return 0;
}
