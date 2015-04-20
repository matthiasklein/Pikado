/*******************************************************************************
*  Pikado - Interface electronic and GUI for a PC based e-dart machine         *
*  Copyright (C) 2006 - 2010 Matthias Klein <matthias.klein@linux.com>         *
*                                                                              *
*  This file is part of Pikado.                                                *
*                                                                              *
*  Pikado is free software: you can redistribute it and/or modify              *
*  it under the terms of the GNU General Public License as published by        *
*  the Free Software Foundation, either version 3 of the License, or           *
*  (at your option) any later version.                                         *
*                                                                              *
*  Pikado is distributed in the hope that it will be useful,                   *
*  but WITHOUT ANY WARRANTY; without even the implied warranty of              *
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the               *
*  GNU General Public License for more details.                                *
*                                                                              *
*  You should have received a copy of the GNU General Public License           *
*  along with Pikado.  If not, see <http://www.gnu.org/licenses/>.             *
*******************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>

typedef unsigned char u08;

u08 SoftwareVersion[] = "Pikado_2010_02_13";

#define TIMER0_RELOAD		205	   // 200us @ 16 MHz
#define PC_BAUDRATE     51     // 19200 Baud @ 16MHz

#define REC_CNT         25     // Number of recordings

// Memory for the recordings
volatile u08 MxData[REC_CNT][4][2];
volatile u08 MxRecStart, MxRecFinish;

// Memory for the analysis
u08 MxCount[4][2][8];          // [output][input-byte][input-bit]

void init(void)
{
  memset(MxData, 0, sizeof(MxData));
  MxRecStart   = 0;
  MxRecFinish  = 0;

  // Outputs 1..8 of the matrix
	DDRA  =  0x00;					        // Inputs
	PORTA =  0xFF;					        // Internal pull-up enable

  // Outputs 9..16 of the matrix
	DDRC  =  0x00;					        // Inputs
	PORTC =  0xFF;					        // Internal pull-up enable

  // ISP / IO
	DDRB  =  0x00;					        // Inputs
	PORTB =  0x00;					        // Tri-state (Hi-Z)

  // Inputs 1..4 of the matrix / RS232 / LED3
	DDRD  =  0xF9;					        // PD7..3 + PD0 Outputs; PD1..PD2 Inputs
	PORTD =  0xF9;					        // PD7..3 + PD0 HIGH;    PD1..PD2 Tri-state (Hi-Z)

  // ISP / push-buttons / LED1..2
	DDRE  =  0xCD;					        // Outputs: PE7, PE6, PE3, PE2, PE0; inputs:  the rest
	PORTE =  0xFF;					        // HIGH:    PE7, PE6, PE3, PE2, PE0; pull-up: the rest

  // JTAG / IO
	DDRF  =  0x00;					        // Inputs
	PORTF =  0x00;					        // Tri-state (Hi-Z)

  // USART: Baudrate
  UBRR1H =  (u08) (PC_BAUDRATE >> 8);
  UBRR1L =  (u08) PC_BAUDRATE;
  
  // USART: Sender, receiver und receiver-interrupt enable
  UCSR1B = (1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1);

  // Set frame format: 8data, 1stop bit
  UCSR1C = (1<<UCSZ10) | (1<<UCSZ11);

  TIFR  |= 1<<TOV0;		  	        // Clear timer 0 overflow flag
  TIMSK |= 1<<TOIE0;		          // Timer 0 overflow interrupt enable
	TCNT0 =  TIMER0_RELOAD;
	TCCR0 =  (1<<CS02);             // Timer 0 run: prescaler 64

	TIFR  |= 1<<TOV2;		  	        // Clear timer 2 overflow flag
	TCNT2 =  0;
	TCCR2 =  (1<<CS22) | (1<<CS20); // Timer 2 run: prescaler 1024

	sei();									        // Global interrupt enable
}

ISR(TIMER0_OVF_vect)
{
  u08 nOut, i, In[2];
  static u08 RecIndex = 0;

	TCNT0 = TIMER0_RELOAD;

  if(MxRecFinish != 0)
    return;
  
  for(nOut=0; nOut<4; nOut++)
  {
    // Readout of the matrix
    switch(nOut)
		{
		  case 0: PORTD &= ~(1<<PD4); break;
		  case 1: PORTD &= ~(1<<PD5); break;
		  case 2: PORTD &= ~(1<<PD6); break;
		  case 3: PORTD &= ~(1<<PD7); break;
		}

    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    
    In[0] = ~PINA;
    In[1] = ~PINC;

    PORTD |= 0xF0;

    if(MxRecStart == 0)
    {
      // Search for a hit
      for(i=0; i<2; i++)
      {
        if(In[i] != 0)
          MxRecStart = 0xFF;
      }
    }
    else
    {
      // Recording ...
      for(i=0; i<2; i++)
        MxData[RecIndex][nOut][i] = In[i];
    }
  }

  if(MxRecStart != 0)
  {
    RecIndex++;

    if(RecIndex == REC_CNT)
    {
      RecIndex    = 0;
      MxRecStart  = 0;
      MxRecFinish = 0xFF;
    }
  }
}

void pc_send(u08 *data, u08 len)
{
  u08 i;
  
  for(i=0; i<len; i++)
  {
    while (!( UCSR1A & (1<<UDRE1))); // wait if the dataregister is ready
    UDR1 = data[i];
  }
}

ISR(USART1_RX_vect)
{
  u08 c = UDR1;

  if(c == 0x00)    // Connection test
  {
    c = 0xFF;
    pc_send(&c, 1);
  }
  else if((c >= 0x04) && (c <= 0x08))    // Set the output count
  {
	  c = 0x00;
    pc_send(&c, 1);
  }
  else if(c == 0x10)    // Get the output count
  {
    c = 4;
    pc_send(&c, 1);
  }
  else if(c == 0x11)    // Send the software version
  {
    pc_send(SoftwareVersion, strlen((char *) SoftwareVersion) + 1);
  }
}

void delay_ms(unsigned long ms) // max. 4 seconds
{
  
	TIFR   |= 1<<TOV1;		  	            // Clear timer 1 overflow flag
  ms = 0xFFFF - ((ms * 1000) / 64);
	TCNT1  =  (unsigned int) ms;
	TCCR1B =  (1<<CS12) | (1<<CS10);      // Timer 1 run: prescaler 1024

  while(!(TIFR & 1<<TOV1));             // Wait until the timer overflows

	TCCR1B =  0;                          // Timer 1 stop
}

int main(void)
{
  // For the analysis of the matrix recordings
  u08 nRec, nOut, nIn, BitMask, BitPos;
  u08 HitVal, nOutHit, nInHit, nBitHit;
  u08 SegmentNr;

  // Debouncing of the push-buttons
  u08 tmp;
  u08 key_state = 0;
  u08 key_press = 0;
  u08 ct0       = 0;
  u08 ct1       = 0;

	init();
  
	while(1)
  {
    // Push-button debouncing: timer2 overflows all 16,384ms
    if(TIFR & 1<<TOV2)
    {
      TIFR |= 1<<TOV2;              // Clear timer 2 overflow flag

      tmp = key_state ^ ~PINE;      // Key changed ?
      ct0 = ~( ct0 & tmp );		      // Reset or count ct0
      ct1 = (ct0 ^ ct1) & tmp;	    // Reset or count ct1
      tmp &= ct0 & ct1;		          // Count until roll over 
      key_state ^= tmp;		          // Then toggle debounced state
      key_press |= key_state & tmp;	// 0->1: key pressing detect

      // 1. Push-button
      if(key_press & 0x04)
      {
        SegmentNr = 0xFE;
        pc_send(&SegmentNr, 1);

        key_press &= 0xFB;
      }

      // 2. Push-button
      if(key_press & 0x08)
      {
        SegmentNr = 0xFF;
        pc_send(&SegmentNr, 1);

        key_press &= 0xF7;
      }
    }

    // Dart recognized and data recorded
    if(MxRecFinish != 0)
    {
      memset(MxCount, 0, sizeof(MxCount));

      // Counting of the high-pulses
      for(nRec=0; nRec<REC_CNT; nRec++)
      {
        for(nOut=0; nOut<4; nOut++)
        {
          for(nIn=0; nIn<2; nIn++)
          {
            BitPos = 0;
            for(BitMask=1; BitMask!=0; BitMask<<=1)
            {
              
              // Contact was closed
              if(MxData[nRec][nOut][nIn] & BitMask)
              {
                MxCount[nOut][nIn][BitPos]++;
              }

              BitPos++;
            }
          }
        }
      }

      // Search the highest count
      HitVal  = 0;
      nOutHit = 0;
      nInHit  = 0;
      nBitHit = 0;
      for(nOut=0; nOut<4; nOut++)
      {
        for(nIn=0; nIn<2; nIn++)
        {
          for(BitPos=0; BitPos<8; BitPos++)
          {
            if(MxCount[nOut][nIn][BitPos] > HitVal)
            {
              HitVal  = MxCount[nOut][nIn][BitPos];
              
              nOutHit = nOut;
              nInHit  = nIn;
              nBitHit = BitPos;
            }
          }
        }
      }
     
      if(HitVal >= 2)
      {
        // Send the SegmentNr to the PC
        SegmentNr = (nInHit * 8) + nBitHit + (nOutHit * 16) + 1;
        pc_send(&SegmentNr, 1);

        // Wait for 200ms
        delay_ms(200);
      }

      // Enable for the next recording
      MxRecFinish = 0x0;
    }
  }
}

