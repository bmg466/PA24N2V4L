
#include <iostm8l152c8.h>//c8
#include <stm8l.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
//#include <intrinsics.h>
///////////////////////////////////////////////////////////////////////////////
//volatile char buff;  
//char addr = 'a';
volatile int vdac = 85;
volatile int vdaccolib;
uint16_t vtp,vtn,vta,PT,GT;
float vtpmv,vtnmv,vtamv,tptn;

#define dselfaddr 6;

volatile uint16_t ca,sa;                // vars coming from external command - ca - Cell Address, sa - SubAddress inside the cell
volatile uint16_t cd;                   // vars coming from external command - cd - data to be written to cell
volatile uint16_t oa,sad;                   // Our Address, Self Address Delay
volatile unsigned char cmd,star;             // cmd - command, one of "r w < >"
volatile int argsread;                  // number of arguments (values) got after sscanf
#define RBSIZE	32		                  // receiver buffer size 
volatile char inbuf[RBSIZE];		        // circular reception buffer
volatile char *rdptr;			              // read pointer for inbuf (used in interrup and main)
volatile char *wrptr;			              // write pointer for inbuf (used in interrup and main)
#define cmdbufsz 32                     // command line buffer size - used to get full command line from PC/ctl until '\n' 
char cmdl[cmdbufsz];           // input buffer used to get full command line from PC until '\n'
#define sbufsz 80                       // sprintf buffer size 
char sbuf[sbufsz];             // sprintf buffer (to usart1)
volatile uint8_t sl,s;                  // lenghth to print and cycle var for sprintf buffer 
//volatile uint8_t dacmsb,daclsb;         // first and second DAC bytes to transfer
//volatile uint8_t *cbf; 


char rch(void)                          // Get a char 
{ char c;				                        // character to be returned 
  while (rdptr == wrptr) {};            
  c=*rdptr++;			                      
  if(rdptr >= &inbuf[RBSIZE]) rdptr=inbuf;
  return (c);
}

void delay(unsigned int n)
{
    while (n-- > 0);
}
void sadelay() //Self Address Delay
{   sad = oa;
while ( sad-- >0){delay(60000);};
}
void init_adc()
{
  //ADC1_CR1_bit.ADON = 1; //adc on
  ADC1_CR1_bit.RES = 0x00; //12 bit
  ADC1_CR2_bit.SMTP1 = 0x03; //24 ADC clock cycles
  ADC1_SQR1_bit.DMAOFF;   //DMA OFF
  //ADC1_SQR2_bit.CHSEL_S16 = 1;   //ch16 PB2
  //ADC1_SQR3_bit.CHSEL_S14 = 1;   //ch14 PB4
  ADC1_TRIGR2_bit.TRIG23 = 1; //trigger off
  ADC1_TRIGR1_bit.VREFINTON = 1; //trigger off
  ADC1_TRIGR3_bit.TRIG9 = 1; //trigger off
  ADC1_CR1_bit.ADON = 1;//adc on
  
}
//ADC threashold + PB2
uint16_t adc_single_measureP(void)
{ ADC1_SQR2_bit.CHSEL_S16 = 1;
  uint16_t adc_res;
  uint16_t value = 0;
  ADC1_CR1_bit.START = 1;   //adc Conversion start
  while (!(ADC1_SR & MASK_ADC1_SR_EOC))   {};   // set by hardware at the end of conversion
  adc_res = ADC1_DRH << 8;   adc_res |= ADC1_DRL;
  value = adc_res;
  ADC1_SQR2_bit.CHSEL_S16 = 0;
  return value;
}
//ADC threashold - PB4
uint16_t adc_single_measureN(void)
{ ADC1_SQR3_bit.CHSEL_S14 = 1;
  uint16_t adc_res;
  uint16_t value = 0;
  ADC1_CR1_bit.START = 1;   //adc Conversion start
  while (!(ADC1_SR & MASK_ADC1_SR_EOC))   {};   // set by hardware at the end of conversion
  adc_res = ADC1_DRH << 8;   adc_res |= ADC1_DRL;
  value = adc_res;
  ADC1_SQR3_bit.CHSEL_S14 = 0;
  return value;
}
//ADC Amp Voltage PD7
uint16_t adc_single_measureA(void)
{ ADC1_SQR4_bit.CHSEL_S7 = 1;
  uint16_t adc_res;
  uint16_t value = 0;
  ADC1_CR1_bit.START = 1;   //adc Conversion start
  while (!(ADC1_SR & MASK_ADC1_SR_EOC))   {};   // set by hardware at the end of conversion
  adc_res = ADC1_DRH << 8;   adc_res |= ADC1_DRL;
  value = adc_res;
  ADC1_SQR4_bit.CHSEL_S7 = 0;
  return value;
}
void dac_colibrator()
{
  while (tptn<=162 || tptn>=170){
    vtp = adc_single_measureP(); 
    delay(50);
    vtn = adc_single_measureN();
    delay(50);
    tptn=(vtp-vtn)*0.805664;
    delay(50);
    if (tptn<=162){
    vdac=vdac-1;DAC_CH1DHR8 = vdac;
    vdaccolib=vdac;}
  if (tptn>=170){
    vdac=vdac+1;DAC_CH1DHR8 = vdac;
    vdaccolib=vdac;}}
}
int MyLowLevelPutchar(int sendchar)
{
  USART1_DR= sendchar;     // Put data in send register
  while(!USART1_SR_TC && !USART1_SR_TXE);  // Wait until sent (TXE or TC ??)
  return sendchar;
}

char getData(char address) {
  unsigned char result;
  
  I2C1_CR2_bit.START = 1; // sending START bit
  while(!(I2C1_SR1_bit.SB)); // waiting tx complite
  
  I2C1_DR = address; // sends address with last bit for write
  while(!(I2C1_SR1_bit.ADDR));  
  while(!(I2C1_SR1_bit.TXE));   
  while(!(I2C1_SR3_bit.TRA));  
  
  I2C1_DR = 0x00;  // address for read
  while(!(I2C1_SR1_bit.TXE));
  while(!(I2C1_SR1_bit.BTF));
    
  I2C1_CR2_bit.START = 1; // repeat START or reset
  while(!(I2C1_SR1_bit.SB));
  
  I2C1_DR = address+1; // sends address with bit for read
  while(!(I2C1_SR1_bit.ADDR)); 
  while(I2C1_SR3_bit.TRA);
  while(!(I2C1_SR1_bit.RXNE));  
  
  result = (I2C1_DR<<2);  // get result
  while(!(I2C1_SR1_bit.BTF));
  result += (I2C1_DR>>6);  // get result
  while(!(I2C1_SR1_bit.BTF));
  
  
  I2C1_CR2_bit.STOP = 1; // STOP bit
  
  return result/4;
}

int main() {
uint8_t c;                                             // general purpose cycle/index var
char inch;  
  
  CLK_PCKENR2_bit.PCKEN20 = 1; // ADC clock enable

// I2C init
  CLK_PCKENR1_bit.PCKEN13 = 1; // I2C clock enable
  I2C1_FREQR = 0x01;            
  I2C1_CCRL = 0x32;             
  I2C1_TRISER = 0x02;           
  I2C1_CR1_bit.PE = 1;   
  I2C1_CR2_bit.ACK = 1;
  I2C1_CR2_bit.POS = 0;
//  I2C1_OARL = 0xA0;
//  I2C1_OARH_bit.ADDCONF = 1;
// USART init
   CLK_PCKENR1_bit.PCKEN15 = 1; // USART1 clock enable 
   CLK_CKDIVR = 0x04;           // System clock source /16 == 16Mhz
   CLK_ICKCR_bit.HSION = 1;
   USART1_BRR2 = 0x08;
   USART1_BRR1 = 0x06; // 9600 baud
   USART1_CR2_bit.REN = 1; //rx en
   USART1_CR2_bit.TEN = 1; //tx en
   USART1_CR2_bit.RIEN = 1; // interrupt r i e n

// DAC init
  CLK_PCKENR1_bit.PCKEN17 = 1; // give clocking in DAC
  //CLK_PCKENR2_bit.PCKEN25 = 1; // for acces to RI registers (clocking comparator)
  //RI_IOSR3_bit.CH15E = 1; // turn on I/O switch for PB4
  
  //DAC_CH1CR1_bit.TEN = 0; // DAC transfer data through the trigger
  //DAC_CH1CR1_bit.TSEL = 3; // set means through the softwear (softwear trigger)
  DAC_CH1CR1_bit.BOFF = 0; // set internal buffer for reduce output impedance
                        // and derive external loads directly, without OA 
                        // (op. amplifier)
  //DAC_SWTRIG_bit.SWTRIG1 = 1;
  DAC_CH1CR1_bit.EN = 1; // powered DAC
  
//Port init   
  //i2c
  PC_DDR_bit.DDR1 = 0;
  PC_DDR_bit.DDR0 = 0;
  PC_ODR_bit.ODR0 = 1;  //SDA
  PC_ODR_bit.ODR1 = 1;  //SCL

  PC_CR1_bit.C11 = 0;
  PC_CR1_bit.C10 = 0;

  PC_CR2_bit.C21 = 0;
  PC_CR2_bit.C20 = 0;
  //+2.5
        PA_DDR_bit.DDR5 = 1;
        PA_CR1_bit.C15 = 1;//0 1-vix 0-vx
        PA_CR2_bit.C25 = 1;
        PA_ODR_bit.ODR5 = 0;// defaul 0 is ON
  //usart      
        PC_DDR_bit.DDR3 = 1; //OutPut TX line on
        PC_CR1_bit.C13 = 1;
        PC_CR2_bit.C23 = 0;
        
        PC_DDR_bit.DDR2 = 0; //InPut RX line on
        PC_CR1_bit.C12 = 0; //ppl
        PC_CR2_bit.C22 = 0; //2Mhz
        
        //PD3 Drive Enable
        PD_DDR_bit.DDR3 = 1;
        PD_CR1_bit.C13 = 1;
        PD_CR2_bit.C23 = 1;
        PD_ODR_bit.ODR3 = 0;
        //PB0 Recive Enable
        PB_DDR_bit.DDR0 = 1;
        PB_CR1_bit.C10 = 1;
        PB_CR2_bit.C20 = 1;
        PB_ODR_bit.ODR0 = 0; // 0 - low is enable
       
////PD7 LED INIT //ony on test board
//        PD_DDR_bit.DDR7 = 1;
//        PD_CR1_bit.C17 = 0;//0 1-vix 0-vx
//        PD_CR2_bit.C27 = 1;
//        PD_ODR_bit.ODR7 = 1;

oa=dselfaddr;  
init_adc();
dac_colibrator();
DAC_CH1DHR8 = vdaccolib;
wrptr=rdptr=inbuf;
sadelay();
PD_ODR_bit.ODR3 = !PD_ODR_bit.ODR3;
delay(100);
printf("%2d ready_\n\r",oa);
delay(100);
PD_ODR_bit.ODR3 = !PD_ODR_bit.ODR3;
asm("RIM");
cmdl[0]=0; 
       while(1)			                                          // loop forever waiting for a command and servicing interrupts
  {
  c=0;                                                                              // point to beginning of cmdl
  do {inch=rch(); cmdl[c++]=inch;} while(inch!='\n' && inch!='\r' && c!=cmdbufsz);  // read cmd line from PC/ctl to buffer while not CR or LF or buffer overflow 
  cmdl[--c]=0;                                                                      // terminate buffer by 0
 argsread=sscanf(cmdl,"%c%2x%2x",&cmd,&ca,&sa);

 
  if(cmdl[0]!='*' && cmdl[0]!='@') continue;
  
//  if(cmdl[1]=='$')                                             // $ only legal commands allowed, otherwise wait for next command
//    {argsread=sscanf(cmdl,"%c%c",&star,&cmd);                   // get command
//    if(c!=3 || argsread!=2)continue;}    
  if(cmdl[0]=='*')                                             // = only legal commands allowed, otherwise wait for next command
    {argsread=sscanf(cmdl,"%c%c",&star,&cmd);                   // get command
    if(c!=3 || argsread!=2)continue;} 
                                                                  
  if(cmdl[0]=='@')                                             // & only legal commands allowed, otherwise wait for next command
    {argsread=sscanf(cmdl,"%c%2d%3d",&cmd,&ca,&cd);     // get command, addr. and 8bit data from cmdline
    if(c!=7 || argsread!=3 || ca!=oa) continue;}
  switch (cmd) {
    case '$': 
            vtp = adc_single_measureP(); vtn = adc_single_measureN();vta = adc_single_measureA();
            PT = getData(0x94);
            GT = getData(0x92);
//here needed delay by addr
      sadelay();      
      PD_ODR_bit.ODR3 = !PD_ODR_bit.ODR3;
      delay(100);
      printf ("%d %d %d %d %d %d %d \n\r",oa,vtp,vtn,vta,vdac,PT,GT);
      delay(100);
      PD_ODR_bit.ODR3 = !PD_ODR_bit.ODR3;
    break;
    case '@': 
      vdac=cd;DAC_CH1DHR8 = vdac;
    break;
    case '=': 
      vdac=vdaccolib;DAC_CH1DHR8 = vdac;
    break;
    case 'n': 
      PA_ODR_bit.ODR5 = 0;
    break;
    case 'f': 
      PA_ODR_bit.ODR5 = 1;
    break;

  }
  }}

// interrupt handler on RX USART1
#pragma vector=USART1_R_OR_vector //( OR or RXNE???)
__interrupt void interrupt(void){

*wrptr++=USART1_DR;
if(wrptr >= &inbuf[RBSIZE]) wrptr=inbuf;
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

