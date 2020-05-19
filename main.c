//-----------------------------------------------------------------------------------------------------
//                                     AMPLIFIER FIRMARE TOF MPD 
//                                          CREATED BY BMG
//                                           EDITED BY BAS
// 06.02.2020 BMG 115200 baud at 16 MHz, GT ON, PT ON
//-----------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------
//Includes
//-----------------------------------------------------------------------------------------------------

#include <iostm8l152c8.h>                    // c8
#include <stm8l.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

//#include <intrinsics.h>


//-----------------------------------------------------------------------------------------------------
//Global constant deinitions
//-----------------------------------------------------------------------------------------------------

#define dselfaddr 1;

#define revision 4116; // rev PCB, PT, GT, rev FW

#define RBSIZE	32
#define cmdbufsz 32  
#define sbufsz 80 


//-----------------------------------------------------------------------------------------------------
//Global variables
//-----------------------------------------------------------------------------------------------------

char cmdl[cmdbufsz];                         // input buffer used to get full command line from PC until '\n'
                                             // sprintf buffer size 
char sbuf[sbufsz];                           // sprintf buffer (to usart1)

float vtpmv, vtnmv, vtamv, tptn;

unsigned int vtp, vtn, vta, PT, GT;

volatile char inbuf[RBSIZE];		     // circular reception buffer
volatile char *rdptr;			     // read pointer for inbuf (used in interrup and main)
volatile char *wrptr;			     // write pointer for inbuf (used in interrup and main)
                                             // command line buffer size - used to get full command line from PC/ctl until '\n' 


volatile unsigned char cmd, star;            // cmd - command, one of "r w < >"
volatile unsigned char sl, s;                // lenghth to print and cycle var for sprintf buffer 
//volatile uint8_t dacmsb,daclsb;            // first and second DAC bytes to transfer
//volatile uint8_t *cbf; 


volatile int vdac = 85;
volatile int vdaccolib, vdaclast;
volatile int argsread;                       // number of arguments (values) got after sscanf
		                             // receiver buffer size 


volatile unsigned int ca, sa;                // vars coming from external command - ca - Cell Address, sa - SubAddress inside the cell
volatile unsigned int cd;                    // vars coming from external command - cd - data to be written to cell
volatile unsigned int oa, sad, rev;          // Our Address, Self Address Delay

//volatile char buff;  
//char addr = 'a';

//-----------------------------------------------------------------------------------------------------
//Function prototypes
//-----------------------------------------------------------------------------------------------------


void delay(unsigned int n);
void sadelay();
void init_adc();
void dac_colibrator();

char rch(void);
char getData(char address);

int MyLowLevelPutchar(int sendchar);

unsigned int adc_single_measureP(void);
unsigned int adc_single_measureN(void);
unsigned int adc_single_measureA(void);

__interrupt void interrupt(void);



//-----------------------------------------------------------------------------------------------------
// MAIN ROUTINE
//-----------------------------------------------------------------------------------------------------


int main() 
{
  
unsigned char c;                             // general purpose cycle/index var
char inch;  
  
  CLK_PCKENR2_bit.PCKEN20 = 1;               // ADC clock enable
// I2C init
  CLK_PCKENR1_bit.PCKEN13 = 1;               // I2C clock enable
  I2C1_FREQR = 0x01;            
  I2C1_CCRL = 0x32;             
  I2C1_TRISER = 0x02;           
  I2C1_CR1_bit.PE = 1;   
  I2C1_CR2_bit.ACK = 1;
  I2C1_CR2_bit.POS = 0;
//  I2C1_OARL = 0xA0;
//  I2C1_OARH_bit.ADDCONF = 1;
// USART init
   CLK_PCKENR1_bit.PCKEN15 = 1;              // USART1 clock enable 
   CLK_CKDIVR = 0x00;                        // System clock source /16 == 16Mhz
   CLK_ICKCR_bit.HSION = 1;
   USART1_BRR2 = 0x0A;
   USART1_BRR1 = 0x08;                       // 115200 baud
   USART1_CR2_bit.REN = 1;                   // rx en
   USART1_CR2_bit.TEN = 1;                   // tx en
   USART1_CR2_bit.RIEN = 1;                  // interrupt r i e n

// DAC init
   CLK_PCKENR1_bit.PCKEN17 = 1;              // give clocking in DAC
  //CLK_PCKENR2_bit.PCKEN25 = 1;             // for acces to RI registers (clocking comparator)
  //RI_IOSR3_bit.CH15E = 1;                  // turn on I/O switch for PB4
  
  //DAC_CH1CR1_bit.TEN = 0;                  // DAC transfer data through the trigger
  //DAC_CH1CR1_bit.TSEL = 3;                 // set means through the softwear (softwear trigger)
   DAC_CH1CR1_bit.BOFF = 0;                  // set internal buffer for reduce output impedance
                                             // and derive external loads directly, without OA 
                                             // (op. amplifier)
  //DAC_SWTRIG_bit.SWTRIG1 = 1;
   DAC_CH1CR1_bit.EN = 1;                    // powered DAC
  
//Port init   
  //i2c
   PC_DDR_bit.DDR1 = 0;
   PC_DDR_bit.DDR0 = 0;
   PC_ODR_bit.ODR0 = 1;                      // SDA
   PC_ODR_bit.ODR1 = 1;                      // SCL

   PC_CR1_bit.C11 = 0;
   PC_CR1_bit.C10 = 0;

   PC_CR2_bit.C21 = 0;
   PC_CR2_bit.C20 = 0;
  //+2.5
        PA_DDR_bit.DDR5 = 1;
        PA_CR1_bit.C15 = 1;                  // 0 1-vix 0-vx
        PA_CR2_bit.C25 = 1;
        PA_ODR_bit.ODR5 = 0;                 // defaul 0 is ON
  //usart      
        PC_DDR_bit.DDR3 = 1;                 // OutPut TX line on
        PC_CR1_bit.C13 = 1;
        PC_CR2_bit.C23 = 0;
        
        PC_DDR_bit.DDR2 = 0;                 // InPut RX line on
        PC_CR1_bit.C12 = 0;                  // ppl
        PC_CR2_bit.C22 = 0;                  // 2Mhz
        
        //PD3 Drive Enable
        // PD_DDR_bit.DDR3 = 1;
        // PD_CR1_bit.C13 = 1;
        // PD_CR2_bit.C23 = 1;
        //  PD_ODR_bit.ODR3 = 1;
        //PB0 Recive Enable
        PB_DDR_bit.DDR0 = 1;
        PB_CR1_bit.C10 = 1;
        PB_CR2_bit.C20 = 1;
        PB_ODR_bit.ODR0 = 0;                 // 0 - low is enable
       
////PD7 LED INIT                             // ony on test board
//        PD_DDR_bit.DDR7 = 1;
//        PD_CR1_bit.C17 = 0;                // 0 1-vix 0-vx
//        PD_CR2_bit.C27 = 1;
//        PD_ODR_bit.ODR7 = 1;

oa = dselfaddr;  
rev = revision;

init_adc();
  delay(1000);
  
dac_colibrator();
DAC_CH1DHR8 = vdaccolib;
wrptr = rdptr = inbuf;

sadelay();
PB_ODR_bit.ODR0 = !PB_ODR_bit.ODR0;
  delay(100);
  
printf("%2d ready\n\r",oa);
  delay(100);
  
PB_ODR_bit.ODR0 = !PB_ODR_bit.ODR0;

asm("RIM");


cmdl[0] = 0; 
       
  while(1)			             // loop forever waiting for a command and servicing interrupts
    {
      c = 0;                                 // point to beginning of cmdl
      
      do 
         {
          inch = rch(); 
          cmdl[c++] = inch;
         } while(inch!='\n'&&inch!='\r'&&c!=cmdbufsz);                         // read cmd line from PC/ctl to buffer while not CR or LF or buffer overflow 
      
            cmdl[--c] = 0;                                                     // terminate buffer by 0
            argsread = sscanf(cmdl, "%c%2x%2x", &cmd, &ca, &sa);

      if(cmdl[0]!='*'&&cmdl[0]!='@'&&cmdl[0]!='&') continue;
      
      //  if(cmdl[1]=='$')                                                     // $ only legal commands allowed, otherwise wait for next command
      //    {argsread=sscanf(cmdl,"%c%c",&star,&cmd);                          // get command
      //    if(c!=3 || argsread!=2)continue;}    
     
     
      if(cmdl[0] == '*')                                                       // = only legal commands allowed, otherwise wait for next command
        
        {argsread = sscanf(cmdl, "%c%c", &star, &cmd);                         // get command
        
      if(c!=3 || argsread!=2)continue;} 
      
      
      
      if(cmdl[0] == '@')                                                       // & only legal commands allowed, otherwise wait for next command
        
        {argsread = sscanf(cmdl, "%c%2d%3d", &cmd, &ca, &cd);                  // get command, addr. and 8bit data from cmdline
        
      if(c!=7||argsread!=3||ca!=oa) continue;
         
        //response @
        
          vdac = cd;
          DAC_CH1DHR8 = vdac;
        
          PB_ODR_bit.ODR0 = !PB_ODR_bit.ODR0;
             delay(100);
             
          printf ("%d\n\r", vdac);
             delay(100);
             
          PB_ODR_bit.ODR0 = !PB_ODR_bit.ODR0;
          
        }
      
      if(cmdl[0]=='&')                                                         // & only legal commands allowed, otherwise wait for next command
        
          {
            argsread = sscanf(cmdl, "%c%2d%c", &cmd ,&ca, &cmd);               // get command and addr.
        
          if(c!=5 || argsread!=3 || ca!=oa) continue;
          
          }
      
      
      switch (cmd) 

        {

          case '$': 
          
            vtp = adc_single_measureP(); 
            vtn = adc_single_measureN();
            vta = adc_single_measureA();
            
            PT = getData(0x94);
            //PT = 01;
            GT = getData(0x92);
            //GT = 00;
        
            PB_ODR_bit.ODR0 = !PB_ODR_bit.ODR0;
              delay(100);
            
            printf ("%d %d %d %d %d %d \n\r", vtp, vtn, vta, vdac, PT, GT);
              delay(100);
              
            PB_ODR_bit.ODR0 = !PB_ODR_bit.ODR0;
            
          break;
          
        
          case 'S': 
            
            vtp = adc_single_measureP(); 
            vtn = adc_single_measureN();
            vta = adc_single_measureA();
            
            PT = getData(0x94);
          //PT = 01;
            GT = getData(0x92);
          //GT = 00;
            
            sadelay();                                                         // here needed delay by addr    
            
            PB_ODR_bit.ODR0 = !PB_ODR_bit.ODR0;
              delay(100);
              
            printf ("%d %d %d %d %d %d %d \n\r", oa, vtp, vtn, vta, vdac, PT, GT);
              delay(100);
            
            PB_ODR_bit.ODR0 = !PB_ODR_bit.ODR0;
            
          break;
          
          
          case 'R': 
            
            sadelay();
            
            PB_ODR_bit.ODR0 = !PB_ODR_bit.ODR0;
              delay(100);
              
            printf ("amp addr %d rev: %d \n\r", oa, rev);
              delay(100);
              
            PB_ODR_bit.ODR0 = !PB_ODR_bit.ODR0;
            
          break;
          
          
          case '=': 
            
            vdac = vdaccolib;
            DAC_CH1DHR8 = vdac;
            
            PB_ODR_bit.ODR0 = !PB_ODR_bit.ODR0;
              delay(100);
              
            printf ("%d\n\r", vdac);
              delay(100);
              
            PB_ODR_bit.ODR0 = !PB_ODR_bit.ODR0;
            
          break;
          
          
          case 'n': 
            
            PA_ODR_bit.ODR5 = 0;
            vdac = vdaclast;
            
            DAC_CH1DHR8 = vdac;
            
            PB_ODR_bit.ODR0 = !PB_ODR_bit.ODR0;
              delay(100);
              
            printf ("on\n\r");
              delay(100);
              
            PB_ODR_bit.ODR0 = !PB_ODR_bit.ODR0;
            
          break;
          
          
          case 'f': 
            
            vdaclast = vdac;
            DAC_CH1DHR8 = 0;
            PA_ODR_bit.ODR5 = 1;
            
            PB_ODR_bit.ODR0 = !PB_ODR_bit.ODR0;
              delay(100);
              
            printf ("off\n\r");
              delay(100);
              
            PB_ODR_bit.ODR0 = !PB_ODR_bit.ODR0;
            
          break;
          
          
          case '#': 
          
            PB_ODR_bit.ODR0 = !PB_ODR_bit.ODR0;
              delay(100);
              
            printf ("%d\n\r", oa);
              delay(100);
              
            PB_ODR_bit.ODR0 = !PB_ODR_bit.ODR0;
            
          break;

        } //end of switch
  
     } //end of while

} //end of MAIN

//-----------------------------------------------------------------------------------------------------
// END OF MAIN
//-----------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------------------------------

void delay(unsigned int n)
{
   while (n-- > 0);
}

//-----------------------------------------------------------------------------------------------------

void sadelay()                               // Self Address Delay
{   
  sad = oa;
  
   while (sad-- > 0)
    {
      delay(10000);
    }                                        // delay
}

//-----------------------------------------------------------------------------------------------------

void init_adc()
{
  ADC1_CR1_bit.ADON = 0;                     // adc off
  ADC1_CR1_bit.RES = 0x00;                   // 12 bit
  ADC1_CR2 = 0x87;                           // 384 ADC clock cycles
  ADC1_SQR1_bit.DMAOFF;                      // DMA OFF
  //ADC1_SQR2_bit.CHSEL_S16 = 1;             // ch16 PB2
  //ADC1_SQR3_bit.CHSEL_S14 = 1;             // ch14 PB4
  ADC1_TRIGR2_bit.TRIG23 = 1;                // trigger off
  ADC1_TRIGR1_bit.VREFINTON = 1;             // trigger off
  ADC1_TRIGR3_bit.TRIG9 = 1;                 // trigger off
  ADC1_CR1_bit.ADON = 1;                     // adc on
  
}

//-----------------------------------------------------------------------------------------------------

void dac_colibrator()
{
   while (tptn <= 162 || tptn >= 170)
    {
          vtp = adc_single_measureP(); 
            delay(3000);
            
          vtn = adc_single_measureN();
            delay(3000);
            
          tptn = (vtp - vtn) * 0.805664;
            delay(50);
            
     if (tptn <= 162)
      {
          vdac = vdac - 1;
          DAC_CH1DHR8 = vdac;
          vdaccolib = vdac;
      }
   
     if (tptn >= 170)
      {
          vdac = vdac + 1;
          DAC_CH1DHR8 = vdac;
          vdaccolib = vdac;
      }
     
    }
}

//-----------------------------------------------------------------------------------------------------

char rch(void)                               // Get a char 
{ 
  char c;				     // character to be returned 
 
    while (rdptr == wrptr)               
      {
      };  
    
    c = *rdptr++;
  
    if (rdptr >= &inbuf[RBSIZE]) rdptr = inbuf;
  
  return (c);
}

//-----------------------------------------------------------------------------------------------------

char getData(char address) 
{
  unsigned char result;
  
    I2C1_CR2_bit.START = 1;                  // sending START bit
    while(!(I2C1_SR1_bit.SB));               // waiting tx complite
  
    I2C1_DR = address;                       // sends address with last bit for write
    while(!(I2C1_SR1_bit.ADDR));  
    while(!(I2C1_SR1_bit.TXE));   
    while(!(I2C1_SR3_bit.TRA));  
  
    I2C1_DR = 0x00;                          // address for read
    while(!(I2C1_SR1_bit.TXE));
    while(!(I2C1_SR1_bit.BTF));
    
    I2C1_CR2_bit.START = 1;                  // repeat START or reset
    while(!(I2C1_SR1_bit.SB));
    
    I2C1_DR = address + 1;                   // sends address with bit for read
    while(!(I2C1_SR1_bit.ADDR)); 
    while(I2C1_SR3_bit.TRA);
    while(!(I2C1_SR1_bit.RXNE));  
  
      result = (I2C1_DR << 2);               // get result
  
    while(!(I2C1_SR1_bit.BTF));
      
      result += (I2C1_DR >> 6);              // get result
      
    while(!(I2C1_SR1_bit.BTF));
    I2C1_CR2_bit.STOP = 1;                   // STOP bit
  
  return result/4;
  
}

//-----------------------------------------------------------------------------------------------------

int MyLowLevelPutchar(int sendchar)
{
  USART1_DR = sendchar;                      // Put data in send register
  
    while(!USART1_SR_TC && !USART1_SR_TXE);  // Wait until sent (TXE or TC ??)
  
  return sendchar;
  
}

//-----------------------------------------------------------------------------------------------------


//ADC threashold + PB2
unsigned int adc_single_measureP(void)
{ 
  ADC1_SQR2_bit.CHSEL_S16 = 1;
  
  unsigned int adc_res;
  unsigned int value = 0;
  
  ADC1_CR1_bit.START = 1;                    // adc Conversion start
  
  while (!(ADC1_SR & MASK_ADC1_SR_EOC))   
    {
    };                                           // set by hardware at the end of conversion
  
      adc_res = ADC1_DRH << 8;   
      adc_res |= ADC1_DRL;
      value = adc_res;
  
  ADC1_SQR2_bit.CHSEL_S16 = 0;
  
  return value;
}

//-----------------------------------------------------------------------------------------------------

//ADC threashold - PB4
unsigned int adc_single_measureN(void)
{ 
  ADC1_SQR3_bit.CHSEL_S14 = 1;
  
  unsigned int adc_res;
  unsigned int value = 0;
 
  ADC1_CR1_bit.START = 1;                    // adc Conversion start
  
  while (!(ADC1_SR & MASK_ADC1_SR_EOC))   
     {
     };                                      // set by hardware at the end of conversion
  
      adc_res = ADC1_DRH << 8;   
      adc_res |= ADC1_DRL;
      value = adc_res;
  
  ADC1_SQR3_bit.CHSEL_S14 = 0;
  
  return value;
}

//-----------------------------------------------------------------------------------------------------

//ADC Amp Voltage PD7
unsigned int adc_single_measureA(void)
{ 
  ADC1_SQR4_bit.CHSEL_S7 = 1;
 
  unsigned int adc_res;
  unsigned int value = 0;
  
  ADC1_CR1_bit.START = 1;                    // adc Conversion start
  
  while (!(ADC1_SR & MASK_ADC1_SR_EOC))   
     {
     };                                      // set by hardware at the end of conversion
  
      adc_res = ADC1_DRH << 8;   
      adc_res |= ADC1_DRL;
      value = adc_res;
  
  ADC1_SQR4_bit.CHSEL_S7 = 0;
  
  return value;
}

//-----------------------------------------------------------------------------------------------------

// interrupt handler on RX USART1

#pragma vector = USART1_R_OR_vector          // (OR or RXNE???)

__interrupt void interrupt(void)
{
  *wrptr++ = USART1_DR;
  
if (wrptr >= &inbuf[RBSIZE]) wrptr = inbuf;

////  PD_ODR_bit.ODR7 = !PD_ODR_bit.ODR7;
//  buff = USART1_DR;
//  switch (buff) {
//  case 'n': PA_ODR_bit.ODR5 = 0; printf (" amp on \n\r");
//            break;
//  case 'f': PA_ODR_bit.ODR5 = 1; printf (" amp off \n\r");
//            break;
//  case '$': vtp = adc_single_measureP(); vtn = adc_single_measureN();vta = adc_single_measureA();
//            PT = getData(0x94);
//            GT = getData(0x92);
//            //delay(60000);
//            PD_ODR_bit.ODR3 = !PD_ODR_bit.ODR3;
//            delay(100);
//            printf (" %c %i %i %i %i %d %d \n\r",addr,vtp,vtn,vta,vdac,PT,GT);
//            delay(100);
//            PD_ODR_bit.ODR3 = !PD_ODR_bit.ODR3;
//            break;
//  case '+': vdac=vdac+1;DAC_CH1DHR8 = vdac;
//            break;
//  case '-': vdac=vdac-1;DAC_CH1DHR8 = vdac;
//            break;
//  case '=': vdac = 85;DAC_CH1DHR8 = vdac;
//            break;
//    }
  
}
