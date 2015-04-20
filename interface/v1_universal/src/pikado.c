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

u08 SoftwareVersion[] = "Pikado_2006_07_15";

#define TIMER0_RELOAD   205    // 200us @ 16 MHz
#define PC_BAUDRATE     51     // 19200 Baud @ 16MHz

#define MEM_ADR_DDRC    (u08 *) 0x14 + 0x20
#define MEM_ADR_DDRD    (u08 *) 0x11 + 0x20

#define REC_CNT         25     // Number of recordings

// Configuration
volatile u08 OutputCount;     // 1..8

// Data from the Matrix
volatile u08 *MxOutDDR[8];     // Addresses of the DDR-register of the outputs
volatile u08 MxOutVal[8];      // Output values for the outputs

// Memory for the recordings
volatile u08 MxData[REC_CNT][8][3];
volatile u08 MxRecStart, MxRecFinish;

// Memory for the analysis
u08 MxCount[8][3][8];          // [output][input-byte][input-bit]

void config_outputs(void)
{
  switch(OutputCount)
  {
  case 4:
    DDRD  &= 0x0F;        // Inputs:                  PD4..PD7
    PORTD |= 0xF0;        // Internal pull-up enable: PD4..PD7
    break;
  case 5:
    DDRD  |= 0x80;        // Output:                  PD7
    DDRD  &= 0x8F;        // Inputs:                  PD4..PD6
    PORTD &= 0x7F;        // Tri-state (Hi-Z):        PD7
    PORTD |= 0x70;        // Internal pull-up enable: PD4..PD6
    break;
  case 6:
    DDRD  |= 0xC0;        // Output:                  PD6, PD7
    DDRD  &= 0xCF;        // Inputs:                  PD4, PD5
    PORTD &= 0x3F;        // Tri-state (Hi-Z):        PD6, PD7
    PORTD |= 0x30;        // Internal pull-up enable: PD4, PD5
    break;
  case 7:
    DDRD  |= 0xE0;        // Output:                  PD5..PD7
    DDRD  &= 0xEF;        // Inputs:                  PD4
    PORTD &= 0x1F;        // Tri-state (Hi-Z):        PD5..PD7
    PORTD |= 0x10;        // Internal pull-up enable: PD4
    break;
  case 8:
    DDRD  |= 0xF0;        // Output:                  PD4..PD7
    PORTD &= 0x0F;        // Tri-state (Hi-Z):        PD4..PD7
    break;
  }
}

void init(void)
{
  OutputCount = 4;

  memset(MxData, 0, sizeof(MxData));
  MxRecStart   = 0;
  MxRecFinish  = 0;

  MxOutDDR[0] = MEM_ADR_DDRC;
  MxOutVal[0] = 1<<PC7;

  MxOutDDR[1] = MEM_ADR_DDRC;
  MxOutVal[1] = 1<<PC6;

  MxOutDDR[2] = MEM_ADR_DDRC;
  MxOutVal[2] = 1<<PC1;

  MxOutDDR[3] = MEM_ADR_DDRC;
  MxOutVal[3] = 1<<PC0;

  MxOutDDR[4] = MEM_ADR_DDRD;
  MxOutVal[4] = 1<<PD7;

  MxOutDDR[5] = MEM_ADR_DDRD;
  MxOutVal[5] = 1<<PD6;

  MxOutDDR[6] = MEM_ADR_DDRD;
  MxOutVal[6] = 1<<PD5;

  MxOutDDR[7] = MEM_ADR_DDRD;
  MxOutVal[7] = 1<<PD4;

  // USART: Baudrate
  UBRRH =  (u08) (PC_BAUDRATE >> 8);
  UBRRL =  (u08) PC_BAUDRATE;
  
  // USART: Sender, receiver und receiver-interrupt enable
  UCSRB =  (1<<RXEN) | (1<<TXEN) | (1<<RXCIE);

  // USART: Dataformat: 8data, 1stop bit
  UCSRC =  (1<<URSEL) | (1<<UCSZ1) | (1<<UCSZ0);

  TIFR  |= 1<<TOV0;               // Clear timer 0 overflow flag
  TIMSK |= 1<<TOIE0;              // Timer 0 overflow interrupt enable
  TCNT0 =  TIMER0_RELOAD;
  TCCR0 =  (1<<CS01) | (1<<CS00); // Timer 0 run: prescaler 64

  TIFR  |= 1<<TOV2;               // Clear timer 2 overflow flag
  TCNT2 =  0;
  TCCR2 =  0x07;                  // Timer 2 run: prescaler 1024

  DDRA  =  0x00;                  // Inputs
  PORTA =  0xFF;                  // Internal pull-up enable

  DDRB  =  0x00;                  // Inputs
  PORTB =  0xFF;                  // Internal pull-up enable

  DDRC  &= 0x3C;                  // Outputs:          PC0, PC1, PC6, PC7
  PORTC &= 0x3C;                  // Tri-state (Hi-Z): PC0, PC1, PC6, PC7

  // For the two push-buttons
  DDRD  &= 0xF3;                  // Inputs:                  PD2..PD3
  PORTD |= 0x0C;                  // Internal pull-up enable: PD2..PD3

  config_outputs();
  sei();                          // Global interrupt enable
}

ISR(TIMER0_OVF_vect)
{
  TCNT0 = TIMER0_RELOAD;

  u08 nOut, i, In[3];
  static u08 RecIndex = 0;

  if(MxRecFinish != 0)
    return;
  
  for(nOut=0; nOut<OutputCount; nOut++)
  {
    // Readout of the matrix
    *MxOutDDR[nOut] |= MxOutVal[nOut];
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    
    In[0] = ~PINA;
    In[1] = ~PINB;
    In[2] = ~PIND;
    In[2] >>= 4;

    *MxOutDDR[nOut] &= ~MxOutVal[nOut];

    // Mask-out of the unused inputs
    if      (OutputCount == 5)
      In[2] &= 0x07;
    else if (OutputCount == 6)
      In[2] &= 0x03;    
    else if (OutputCount == 7)
      In[2] &= 0x01;    
    else if (OutputCount == 8)
      In[2] &= 0x00;

    if(MxRecStart == 0)
    {
      // Search for a hit
      for(i=0; i<3; i++)
      {
        if(In[i] != 0)
          MxRecStart = 0xFF;
      }
    }
    else
    {
      // Recording ...
      for(i=0; i<3; i++)
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
    while (!(UCSRA & (1<<UDRE))); // wait until the dataregister is ready
    UDR = data[i];
  }
}

ISR(USART_RXC_vect)
{
  u08 c = UDR;

  if(c == 0x00)    // Connection test
  {
    c = 0xFF;
    pc_send(&c, 1);
  }
  else if((c >= 0x04) && (c <= 0x08))    // Set the output count
  {
    OutputCount = c;
    config_outputs();
    c = 0x00;
    pc_send(&c, 1);
  }
  else if(c == 0x10)    // Get the output count
  {
    c = OutputCount;
    pc_send(&c, 1);
  }
  else if(c == 0x11)    // Send the software version
  {
    pc_send(SoftwareVersion, strlen((char *) SoftwareVersion) + 1);
  }
}

void delay_ms(unsigned long ms) // max. 4 seconds
{
  
  TIFR   |= 1<<TOV1;                    // Clear timer 1 overflow flag
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

      tmp = key_state ^ ~PIND;      // Key changed ?
      ct0 = ~( ct0 & tmp );         // Reset or count ct0
      ct1 = (ct0 ^ ct1) & tmp;      // Reset or count ct1
      tmp &= ct0 & ct1;             // Count until roll over 
      key_state ^= tmp;             // Then toggle debounced state
      key_press |= key_state & tmp; // 0->1: key pressing detect

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
        for(nOut=0; nOut<OutputCount; nOut++)
        {
          for(nIn=0; nIn<3; nIn++)
          {
            BitPos = 0;
            for(BitMask=1; BitMask!=0; BitMask<<=1)
            {
              BitPos++;
              
              // Contact was closed
              if(MxData[nRec][nOut][nIn] & BitMask)
              {
                MxCount[nOut][nIn][BitPos]++;
              }
            }
          }
        }
      }

      // Search the highest count
      HitVal  = 0;
      nOutHit = 0;
      nInHit  = 0;
      nBitHit = 0;
      for(nOut=0; nOut<OutputCount; nOut++)
      {
        for(nIn=0; nIn<3; nIn++)
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

      // Special version for Claus with "HitVal >= 4"
      if(HitVal >= 2)
      {
        // Send the SegmentNr to the PC
        SegmentNr = nBitHit + (nInHit * 8) + (nOutHit * 20);
        pc_send(&SegmentNr, 1);

        // Wait for 200ms
        delay_ms(200);
      }

      // Enable for the next recording
      MxRecFinish = 0x0;
    }
  }
}

