#include  "msp430g2231.h"

#define     LED0                  BIT0
#define     LED1                  BIT6
#define     LED_DIR               P1DIR
#define     LED_OUT               P1OUT
 
#define     BUTTON                BIT3
#define     BUTTON_OUT            P1OUT
#define     BUTTON_DIR            P1DIR
#define     BUTTON_IN             P1IN
#define     BUTTON_IE             P1IE
#define     BUTTON_IES            P1IES
#define     BUTTON_IFG            P1IFG
#define     BUTTON_REN            P1REN

#define     TXD                   BIT1                      // TXD on P1.1
#define     RXD                   BIT2                      // RXD on P1.2

#define     TIMER_PWM_MODE        0   
#define     TIMER_UART_MODE       1
#define     TIMER_PWM_PERIOD      2000  
#define     TIMER_PWM_OFFSET      20

//   Conditions for 9600/4=2400 Baud SW UART, SMCLK = 1MHz
#define     Bitime_5              0x05*4                      // ~ 0.5 bit length + small adjustment
#define     Bitime                13*4//0x0D    
 
#define     UART_UPDATE_INTERVAL  1000

/* Using a moving average filter on sampled WDT count values */
#define MOTION_DEPTH  20  // 60sec ?
unsigned int motMeasured[MOTION_DEPTH];
unsigned int motPtr;
unsigned int motNow;  // ++ for each motion detected (on or off, either count)
unsigned int motAverage;
unsigned int wdt_count;
#define WDT_COUNT_MAX 3   // 3 seconds




void set_clk(void)
{
  if (CALBC1_1MHZ ==0xFF || CALDCO_1MHZ == 0xFF)                                     
  {  
    while(1);                               // If calibration constants erased
                                            // do not load, trap CPU!!
  } 
 //1Mhz
  BCSCTL1 = CALBC1_1MHZ;                    // Set range
  DCOCTL = CALDCO_1MHZ;                     // Set DCO step + modulation */
/* //8Mhz
  BCSCTL1 = CALBC1_8MHZ;                    // Set range
  DCOCTL = CALDCO_8MHZ;                     // Set DCO step + modulation */
/* //12Mhz
  BCSCTL1 = CALBC1_12MHZ;                   // Set range
  DCOCTL = CALDCO_12MHZ;                    // Set DCO step + modulation*/
/* //16Mhz
  BCSCTL1 = CALBC1_16MHZ;                   // Set range
  DCOCTL = CALDCO_16MHZ;                    // Set DCO step + modulation*/
  
#if 0
  BCSCTL2 &= ~(DIVS_3);                         // SMCLK = DCO / 8 = 1MHz 
  BCSCTL1 |= DIVA_1;                        // ACLK/2
  BCSCTL3 |= LFXT1S_2;                      // ACLK = VLO
  
  TACCR0 = 1200;                             //   
  TACTL = TASSEL_1 | MC_1;                  // TACLK = SMCLK, Up mode.  
  TACCTL1 = CCIE + OUTMOD_3;                // TACCTL1 Capture Compare
  TACCR1 = 600;
#endif
}
void hw_init(void)
{
  P1DIR |= BIT0 | BIT6;     // output and
  P1OUT &= ~(BIT0 | BIT6);  // turn off P1.0 and P1.6  
}

void set_intlvl_debounce()
{
  if (P1IN & BIT5)
    P1IES |= BIT5;  // 0=low-2-high 1=high-2-low
  else
    P1IES &= ~BIT5;
}

void avg_init(void)
{
  for (int i=0; i<MOTION_DEPTH;i++)
    motMeasured[i] = 0;
  motPtr = 0;
  motNow = 0;
  motAverage = 0;
  wdt_count = 0;
}

void proc_and_light(void)
{
  unsigned long sum = 0;
  for (int i=0; i<MOTION_DEPTH;i++)
    sum += motMeasured[i];
  
#if 1
  if (sum > 20) // 20=0.33 * ( 20 * 3)
    P1OUT |= BIT0;
  else
    P1OUT &= ~BIT0;
#else
        sum <<= 3;
      sum += TIMER_PWM_OFFSET;      
      TACCR1 = ( (sum) < (TIMER_PWM_PERIOD-1) ? (sum) : (TIMER_PWM_PERIOD-1) );
      TACCTL0 |= CCIE;
      TACCTL1 |= CCIE; 
#endif
}
void ConfigureTimerPwm(void)
{  
  TACCR0 = TIMER_PWM_PERIOD;                              //   
  TACTL = TASSEL_2 | MC_1;                  // TACLK = SMCLK, Up mode.
  TACCTL0 = CCIE;
  TACCTL1 = CCIE + OUTMOD_3;                // TACCTL1 Capture Compare
  TACCR1 = 1;
}
void mirror_motInput_as_led(void)
{
    if (P1IN & BIT5)
    P1OUT |= BIT0;
  else
    P1OUT &= ~BIT0;
}

int main( void )
{
  set_clk();
  hw_init();
  //ConfigureTimerPwm();
  avg_init();
  
  // WDT ints used as basic system timer (screw reliability!)
  WDTCTL = WDT_ADLY_1000;
  IE1 |= WDTIE;                             // Enable WDT interrupt
  
  // motion sensor is connected here as I/O
  P1DIR &= ~BIT5;    // input
  //mirror_motInput_as_led();
  set_intlvl_debounce();
  P1IE |= BIT5;
  P1IFG &= ~BIT5;
  
  while(1)
  {    
    _BIS_SR(LPM3_bits + GIE);                 // Enter LPM3 w/interrupt
    
    // now go do processing of new data
    proc_and_light();
    
    // done
    P1OUT &= ~BIT6;
  }
  return 0;
}

// Port 1 interrupt service routine
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
  //mirror_motInput_as_led();
  set_intlvl_debounce();
  P1IFG &= ~BIT5;                           // IFG cleared
  motNow++; // TODO: need to use compare instead for more accurate count
  //__low_power_mode_off_on_exit();           // exit low-power mode
}

#pragma vector=WDT_VECTOR
__interrupt void WDT_ISR(void)
{
    IE1 &= ~WDTIE;                   /* disable interrupt */
    IFG1 &= ~WDTIFG;                 /* clear interrupt flag */
    
    // inc. if we see continuous motion
    if (P1IN & BIT5)
      motNow++;
    
    // WDT fires at 1sec
    if (++wdt_count > WDT_COUNT_MAX)
    {
      wdt_count =0;
      P1OUT |= BIT6;
      motMeasured[motPtr++] = motNow;
      if (motPtr > MOTION_DEPTH)
        motPtr=0;
      motNow = 0;
      __low_power_mode_off_on_exit();           // exit low-power mode
    }
    IE1 |= WDTIE;
}

#pragma vector=TIMERA1_VECTOR
__interrupt void ta1_isr(void)
{
  TACCTL1 &= ~CCIFG;
  P1OUT &= ~BIT0;
   
}

// Timer A0 interrupt service routine
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A (void)
{
#if 0
  if (timerMode == TIMER_UART_MODE)
  {
    CCR0 += Bitime;                           // Add Offset to CCR0  
    if (CCTL0 & CCIS0)                        // TX on CCI0B?
    {
      if ( BitCnt == 0)
        CCTL0 &= ~ CCIE;                        // All bits TXed, disable interrupt
      else
      {
        CCTL0 |=  OUTMOD2;                    // TX Space
        if (TXByte & 0x01)
        CCTL0 &= ~ OUTMOD2;                   // TX Mark
        TXByte = TXByte >> 1;
        BitCnt --;
      }
    }
  }
  else
#endif
  {
    P1OUT |= BIT0;
    TACCTL0 &= ~CCIFG;              
  }
}







#if 0 




/******************************************************************************
 *                  MSP-EXP430G2-LaunchPad User Experience Application
 * 
 * 1. Device starts up in LPM3 + blinking LED to indicate device is alive    
 *    + Upon first button press, device transitions to application mode
 * 2. Application Mode
 *    + Continuously sample ADC Temp Sensor channel, compare result against 
 *      initial value        
 *    + Set PWM based on measured ADC offset: Red LED for positive offset, Green
 *      LED for negative offset
 *    + Transmit temperature value via TimerA UART to PC  
 *    + Button Press --> Calibrate using current temperature  
 *                       Send character '°' via UART, notifying PC 
 ******************************************************************************/
  

#define     APP_STANDBY_MODE      0
#define     APP_APPLICATION_MODE  1


#define     TEMP_SAME             0
#define     TEMP_HOT              1
#define     TEMP_COLD             2

#define     TEMP_THRESHOLD        5  

//   Conditions for 9600/4=2400 Baud SW UART, SMCLK = 1MHz
#define     Bitime_5              0x05*4                      // ~ 0.5 bit length + small adjustment
#define     Bitime                13*4//0x0D    
 
#define     UART_UPDATE_INTERVAL  1000


unsigned char BitCnt;


unsigned char applicationMode = APP_STANDBY_MODE;
unsigned char timerMode = TIMER_PWM_MODE;

unsigned char tempMode;
unsigned char calibrateUpdate = 0;
unsigned char tempPolarity = TEMP_SAME;
unsigned int TXByte;
                               
/* Using an 8-value moving average filter on sampled ADC values */  
long tempMeasured[8];
unsigned char tempMeasuredPosition=0;
long tempAverage;

long tempCalibrated, tempDifference;


  
void InitializeLeds(void);
void InitializeButton(void);
void PreApplicationMode(void);                     // Blinks LED, waits for button press
void ConfigureAdcTempSensor(void);
void ConfigureTimerPwm(void);
void ConfigureTimerUart(void);  
void Transmit(void);
void InitializeClocks(void);

void main(void)
{
  unsigned int uartUpdateTimer = UART_UPDATE_INTERVAL;
  unsigned char i;
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
  
  InitializeClocks();
  InitializeButton();
  InitializeLeds();
  PreApplicationMode();                     // Blinks LEDs, waits for button press
  
  /* Application Mode begins */
  applicationMode = APP_APPLICATION_MODE;
  ConfigureAdcTempSensor();
  ConfigureTimerPwm();
    
  BCSCTL1 = CALBC1_1MHZ; // Set range
  DCOCTL = CALDCO_1MHZ;   // SMCLK = DCO = 1MHz
  
  __enable_interrupt();                     // Enable interrupts.
  
  
  /* Main Application Loop */
  while(1)
  {    
    ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
    __bis_SR_register(CPUOFF + GIE);        // LPM0 with interrupts enabled
    
    
    /* Moving average filter out of 8 values to somewhat stabilize sampled ADC */
    tempMeasured[tempMeasuredPosition++] = ADC10MEM;
    if (tempMeasuredPosition == 8)
      tempMeasuredPosition = 0;
    tempAverage = 0;
    for (i = 0; i < 8; i++)
      tempAverage += tempMeasured[i];
    tempAverage >>= 3;                      // Divide by 8 to get average
    
    if ((--uartUpdateTimer == 0) || calibrateUpdate )
    {
      ConfigureTimerUart();
      if (calibrateUpdate)
      { 
        TXByte = 248;                       // A character with high value, outside of temp range 
        Transmit();
        calibrateUpdate = 0;
      }   
      TXByte = (unsigned char)( ((tempAverage - 630) * 761) / 1024 );      
      Transmit(); 
      uartUpdateTimer = UART_UPDATE_INTERVAL;
      ConfigureTimerPwm();
    }
    
    
    tempDifference = tempAverage - tempCalibrated;
    if (tempDifference < -TEMP_THRESHOLD)
    {
      tempDifference = -tempDifference;
      tempPolarity = TEMP_COLD;
      LED_OUT &= ~ LED1;
    }
    else
    if (tempDifference > TEMP_THRESHOLD)
    {
      tempPolarity = TEMP_HOT;
      LED_OUT &= ~ LED0;
    }
    else
    {
      tempPolarity = TEMP_SAME;
      TACCTL0 &= ~CCIE;
      TACCTL1 &= ~CCIE;
      LED_OUT &= ~(LED0 + LED1);        
    } 
    
    if (tempPolarity != TEMP_SAME)    
    {      
      tempDifference <<= 3;
      tempDifference += TIMER_PWM_OFFSET;      
      TACCR1 = ( (tempDifference) < (TIMER_PWM_PERIOD-1) ? (tempDifference) : (TIMER_PWM_PERIOD-1) );
      TACCTL0 |= CCIE;
      TACCTL1 |= CCIE;      
    }   
  }  
}

void PreApplicationMode(void)
{    
  LED_DIR |= LED0 + LED1;
  LED_OUT |= LED0;                          // To enable the LED toggling effect
  LED_OUT &= ~LED1;
    
  BCSCTL1 |= DIVA_1;                        // ACLK/2
  BCSCTL3 |= LFXT1S_2;                      // ACLK = VLO
  
  TACCR0 = 1200;                             //   
  TACTL = TASSEL_1 | MC_1;                  // TACLK = SMCLK, Up mode.  
  TACCTL1 = CCIE + OUTMOD_3;                // TACCTL1 Capture Compare
  TACCR1 = 600;  
  __bis_SR_register(LPM3_bits + GIE);          // LPM0 with interrupts enabled
}

void ConfigureAdcTempSensor(void)
{
  unsigned char i;
  /* Configure ADC Temp Sensor Channel */
  ADC10CTL1 = INCH_10 + ADC10DIV_3;         // Temp Sensor ADC10CLK/4
  ADC10CTL0 = SREF_1 + ADC10SHT_3 + REFON + ADC10ON + ADC10IE;
  __delay_cycles(1000);                     // Wait for ADC Ref to settle  
  ADC10CTL0 |= ENC + ADC10SC;               // Sampling and conversion start
  __bis_SR_register(CPUOFF + GIE);          // LPM0 with interrupts enabled
  tempCalibrated = ADC10MEM;
  for (i=0; i < 8; i++)
    tempMeasured[i] = tempCalibrated;
  tempAverage = tempCalibrated;  
}


void ConfigureTimerPwm(void)
{
  timerMode = TIMER_PWM_MODE;
  
  TACCR0 = TIMER_PWM_PERIOD;                              //   
  TACTL = TASSEL_2 | MC_1;                  // TACLK = SMCLK, Up mode.
  TACCTL0 = CCIE;
  TACCTL1 = CCIE + OUTMOD_3;                // TACCTL1 Capture Compare
  TACCR1 = 1;
}

void ConfigureTimerUart(void)
{
  timerMode = TIMER_UART_MODE;               // Configure TimerA0 UART TX 
                           
  CCTL0 = OUT;                               // TXD Idle as Mark
  TACTL = TASSEL_2 + MC_2 + ID_3;            // SMCLK/8, continuous mode
  P1SEL |= TXD + RXD;                        //
  P1DIR |= TXD;                              //  
}

// Function Transmits Character from TXByte 
void Transmit()
{ 
  BitCnt = 0xA;                             // Load Bit counter, 8data + ST/SP
  while (CCR0 != TAR)                       // Prevent async capture
    CCR0 = TAR;                             // Current state of TA counter
  CCR0 += Bitime;                     // Some time till first bit
  TXByte |= 0x100;                        // Add mark stop bit to TXByte
  TXByte = TXByte << 1;                 // Add space start bit
  CCTL0 =  CCIS0 + OUTMOD0 + CCIE;          // TXD = mark = idle
  while ( CCTL0 & CCIE );                   // Wait for TX completion
}



// Timer A0 interrupt service routine
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A (void)
{
  if (timerMode == TIMER_UART_MODE)
  {
    CCR0 += Bitime;                           // Add Offset to CCR0  
    if (CCTL0 & CCIS0)                        // TX on CCI0B?
    {
      if ( BitCnt == 0)
        CCTL0 &= ~ CCIE;                        // All bits TXed, disable interrupt
      else
      {
        CCTL0 |=  OUTMOD2;                    // TX Space
        if (TXByte & 0x01)
        CCTL0 &= ~ OUTMOD2;                   // TX Mark
        TXByte = TXByte >> 1;
        BitCnt --;
      }
    }
  }
  else
  {
    if (tempPolarity == TEMP_HOT)
      LED_OUT |= LED1;   
    if (tempPolarity == TEMP_COLD)      
      LED_OUT |= LED0;
    TACCTL0 &= ~CCIFG;              
  }
}

#pragma vector=TIMERA1_VECTOR
__interrupt void ta1_isr(void)
{
  TACCTL1 &= ~CCIFG;
  if (applicationMode == APP_APPLICATION_MODE)
    LED_OUT &= ~(LED0 + LED1);
  else
    LED_OUT ^= (LED0 + LED1);
    
}

void InitializeClocks(void)
{

  BCSCTL1 = CALBC1_1MHZ;                    // Set range
  DCOCTL = CALDCO_1MHZ;
  BCSCTL2 &= ~(DIVS_3);                         // SMCLK = DCO / 8 = 1MHz  
}

void InitializeButton(void)                 // Configure Push Button 
{
  BUTTON_DIR &= ~BUTTON;
  BUTTON_OUT |= BUTTON;
  BUTTON_REN |= BUTTON;
  BUTTON_IES |= BUTTON;
  BUTTON_IFG &= ~BUTTON;
  BUTTON_IE |= BUTTON;
}


void InitializeLeds(void)
{
  LED_DIR |= LED0 + LED1;                          
  LED_OUT &= ~(LED0 + LED1);  
}

/* *************************************************************
 * Port Interrupt for Button Press 
 * 1. During standby mode: to exit and enter application mode
 * 2. During application mode: to recalibrate temp sensor 
 * *********************************************************** */
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{   
  BUTTON_IFG = 0;  
  BUTTON_IE &= ~BUTTON;            /* Debounce */
  WDTCTL = WDT_ADLY_250;
  IFG1 &= ~WDTIFG;                 /* clear interrupt flag */
  IE1 |= WDTIE;  
    
  if (applicationMode == APP_APPLICATION_MODE)
  {
    tempCalibrated = tempAverage;
    calibrateUpdate  = 1;
  }
  else
  {
    applicationMode = APP_APPLICATION_MODE; // Switch from STANDBY to APPLICATION MODE
    __bic_SR_register_on_exit(LPM3_bits);        
  }   
}

#pragma vector=WDT_VECTOR
__interrupt void WDT_ISR(void)
{
    IE1 &= ~WDTIE;                   /* disable interrupt */
    IFG1 &= ~WDTIFG;                 /* clear interrupt flag */
    WDTCTL = WDTPW + WDTHOLD;        /* put WDT back in hold state */
    BUTTON_IE |= BUTTON;             /* Debouncing complete */
}

// ADC10 interrupt service routine
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR (void)
{
  __bic_SR_register_on_exit(CPUOFF);        // Return to active mode
}


#endif  // if 0
