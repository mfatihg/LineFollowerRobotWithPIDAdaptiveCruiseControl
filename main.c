#include "stm32f10x.h"                // Device header
#include "clock.h"        // Custom clock config
#include "delay.h"           // Custom delay
#include "stdio.h"                    // For prints if needed (not used here)

/* --- Global Variables for Line Sensor PID --- */
static int8_t  g_lineData[8]     = {0};    // Store raw line sensor reads
static int     g_linePos         = 0;      // Weighted line position
static int     g_lineLastError   = 0;      // Last line error for derivative
static int     g_lineErrorHist[10] = {0};  // Past errors for integral
static float   g_lineKp  = 0.007f;         // Proportional gain (line)
static float   g_lineKi  = 0.003f;         // Integral gain (line)
static float   g_lineKd  = 100.0f;         // Derivative gain (line)

/* --- Global Variables for Distance PID --- */
static float   g_distanceMeasured    = 0.0f;   // Measured distance (cm)
static float   g_distanceSetpoint    = 60.0f;  // Desired distance (cm)
static float   g_distanceError       = 0.0f;   // Current error
static float   g_distanceLastError   = 0.0f;   // Last error for derivative
static float   g_distanceErrorHist[10] = {0};  // Past distance errors for integral
static float   g_distKp = 0.9f;               // Proportional gain (distance)
static float   g_distKi = 0.05f;              // Integral gain (distance)
static float   g_distKd = 0.2f;               // Derivative gain (distance)

/* --- Motor Speeds & Limits --- */
static int     g_baseSpeed      = 30;     // Base forward speed
static int     g_motorMaxSpeed  = 100;    // Maximum motor speed (absolute value)

/* --- Misc Variables --- */
static int     g_lineNoDetectCount = 0;   // Count cycles with no line
static int     g_lineEndedLeft     = 0;   // 1 if line ended on left, 0 if ended on right

/* --- Function Prototypes --- */
void initGPIO(void);                        
void initPWM_Timer2(void);                 
void initPWM_Timer3(void);                 
void driveMotors(int left, int right);     
int  readLineSensors(void);                
void shiftLineErrorHistory(int error);     
int  sumLineErrors(int count);             
void updateLinePID(int *pLeftOut, int *pRightOut);

void shiftDistErrorHistory(float error);   
float sumDistErrors(int count);            
float updateDistancePID(void);

void handleNoLine(void);
void initUltrasonic(void);
float measureDistance(void);
void buzzerControl(float dist);

/* --- MAIN FUNCTION --- */
int main(void)
{
  initSystemClock();                 // Configure system clock to ~72 MHz
  initGPIO();                        // Configure all GPIO pins
  initPWM_Timer2();                  // Configure PWM on Timer2 (PB0, PB1)
  initPWM_Timer3();                  // Configure PWM on Timer3 (PB10, PB11)
  initTimer2ForDelay();             // Re-init Timer2 for microsecond delay (educational example)
  initUltrasonic();                  // Configure ultrasonic pins
  
  while(1)
  {
    /* --- 1) Measure distance and compute distance PID output --- */
    g_distanceMeasured = measureDistance();         // Ultrasonic measurement
    g_distanceError    = g_distanceSetpoint - g_distanceMeasured; // Distance error
    float forwardPID   = updateDistancePID();       // Update distance PID => modifies forward speed
    
    int forwardSpeed = g_baseSpeed + (int)forwardPID; // Combine base speed + distance correction
    if(forwardSpeed < -g_motorMaxSpeed) forwardSpeed = -g_motorMaxSpeed;
    if(forwardSpeed >  g_motorMaxSpeed) forwardSpeed =  g_motorMaxSpeed;
    
    /* --- 2) Compute line sensor PID output (steering) --- */
    // 'steering' will be subtracted from right motor and added to left motor
    int steeringLeft  = 0;
    int steeringRight = 0;
    updateLinePID(&steeringLeft, &steeringRight);  // Compute line-based correction for left, right
    
    /* 
       Combine forwardSpeed and line steering. 
       
    */
    int finalLeftSpeed  = forwardSpeed + steeringLeft;
    int finalRightSpeed = forwardSpeed + steeringRight;
    
    // Clamp final speeds
    if(finalLeftSpeed  < -g_motorMaxSpeed) finalLeftSpeed  = -g_motorMaxSpeed;
    if(finalLeftSpeed  >  g_motorMaxSpeed) finalLeftSpeed  =  g_motorMaxSpeed;
    if(finalRightSpeed < -g_motorMaxSpeed) finalRightSpeed = -g_motorMaxSpeed;
    if(finalRightSpeed >  g_motorMaxSpeed) finalRightSpeed =  g_motorMaxSpeed;
    
    /* --- 3) Drive motors and buzzer feedback --- */
    driveMotors(finalLeftSpeed, finalRightSpeed);  // Send final commands to motors
    buzzerControl(g_distanceMeasured);             // Buzzer logic
  }
  return 0; 
}

/* --- GPIO INIT --- */
void initGPIO(void)
{
  RCC->APB2ENR |= (1<<2) | (1<<3) | (1<<0); // Enable clock for GPIOA, GPIOB, AFIO

  /* --- PB0, PB1 => Timer2 CH3, CH4 for PWM --- */
  GPIOB->CRL &= ~((0xF<<(0*4)) | (0xF<<(1*4))); // Clear bits for PB0, PB1
  GPIOB->CRL |=  ((0xB<<(0*4)) | (0xB<<(1*4))); // 0xB => 1011 => 50 MHz, AF PP

  /* --- PB10, PB11 => Timer3 CH3, CH4 for PWM --- */
  GPIOB->CRH &= ~((0xF<<(2*4)) | (0xF<<(3*4))); // Clear bits for PB10, PB11
  GPIOB->CRH |=  ((0xB<<(2*4)) | (0xB<<(3*4))); // 0xB => 1011 => 50 MHz, AF PP

  /* --- PB8 => Buzzer output --- */
  // PB8 in CRH => (8-8)*4=0 in CRH
  GPIOB->CRH &= ~(0xF << (0*4));    
  GPIOB->CRH |=  (0x3 << (0*4));    // 0x3 => 0011 => 50 MHz, GP Push-Pull

  /* --- PB12 => TRIG (Output), PB13 => ECHO (Input PullUp) --- */
  // PB12 => output push-pull
  GPIOB->CRH &= ~(0xF << (4*4));   
  GPIOB->CRH |=  (0x3 << (4*4));   // 0x3 => 0011 => 50 MHz, GP Push-Pull
  // PB13 => input with pull-up => 0x8
  GPIOB->CRH &= ~(0xF << (5*4));  
  GPIOB->CRH |=  (0x8 << (5*4));   
  GPIOB->ODR |=  (1<<13);         // Activate pull-up

  /* --- PA0..PA7 => QTR8 line sensors => input pull-up --- */
  for(int pin=0; pin<8; pin++)
  {
    GPIOA->CRL &= ~(0xF << (pin*4));   // Clear bits
    GPIOA->CRL |=  (0x8 << (pin*4));   // 0x8 => 1000 => Input w/ pull-up
  }
  GPIOA->ODR |= 0xFF; // Pull all PA0..PA7 up
}

/* --- PWM INIT ON TIMER2 (PB0=CH3, PB1=CH4) --- */
void initPWM_Timer2(void)
{
  RCC->APB1ENR |= (1<<0);        // Enable TIM2 clock
  AFIO->MAPR   |= (3<<8);        // Full remap of TIM2 => PB0, PB1 => CH3, CH4
  TIM2->PSC     = 72 - 1;        // Prescaler => 72 => Timer freq=1 MHz
  TIM2->ARR     = 50000;         // Period => 50000 => ~20 Hz (example)
  
  // CH3 => OC3M=110 (PWM1 mode), OC3PE=1
  TIM2->CCMR2 |= (6<<4) | (1<<3);  
  // CH4 => OC4M=110, OC4PE=1
  TIM2->CCMR2 |= (6<<12) | (1<<11);
  // Enable CH3, CH4 outputs
  TIM2->CCER  |= (1<<8) | (1<<12);
  // ARPE=1
  TIM2->CR1   |= (1<<7);
  // Enable TIM2
  TIM2->CR1   |= (1<<0);
}

/* --- PWM INIT ON TIMER3 (PB10=CH3, PB11=CH4) --- */
void initPWM_Timer3(void)
{
  RCC->APB1ENR |= (1<<1);     // Enable TIM3 clock
  TIM3->PSC     = 72 - 1;     // Prescaler => 72 => 1 MHz
  TIM3->ARR     = 2000;       // Period => 2000 => 500 Hz (example)
  
  // CH3 => OC3M=110 (PWM1), OC3PE=1
  TIM3->CCMR2  |= (6<<4) | (1<<3);
  // CH4 => OC4M=110 (PWM1), OC4PE=1
  TIM3->CCMR2  |= (6<<12) | (1<<11);
  // Enable CH3, CH4
  TIM3->CCER   |= (1<<8) | (1<<12);
  // ARPE=1
  TIM3->CR1    |= (1<<7);
  // Enable TIM3
  TIM3->CR1    |= (1<<0);
}

/* --- DRIVE MOTORS --- */
void driveMotors(int left, int right)
{
  /* 
     PB0=TIM2_CH3, PB1=TIM2_CH4 => Left motor 
     PB10=TIM3_CH3, PB11=TIM3_CH4 => Right motor
     
  */
  if(left >= 0)
  {
    TIM2->CCR3 = (left * 10);
    TIM2->CCR4 = 0;
  }
  else
  {
    TIM2->CCR3 = 0;
    TIM2->CCR4 = (-left * 10);
  }
  
  if(right >= 0)
  {
    TIM3->CCR3 = (right * 10);
    TIM3->CCR4 = 0;
  }
  else
  {
    TIM3->CCR3 = 0;
    TIM3->CCR4 = (-right * 10);
  }
}

/* --- READ LINE SENSORS --- */
int readLineSensors(void)
{
  int sumPos      = 0;               // Weighted sum
  int activeCount = 0;               // Number of sensors active
  int weights[8]  = {1000,2000,3000,4000,5000,6000,7000,8000};
  
  for(int i=0; i<8; i++)
  {
    g_lineData[i] = ( (GPIOA->IDR & (1<<i)) ? 1 : 0 );
    if(g_lineData[i])
    {
      sumPos += weights[i];
      activeCount++;
      if(i==0) g_lineEndedLeft = 1;  // If leftmost sensor sees line
      if(i==7) g_lineEndedLeft = 0;  // If rightmost sensor sees line
    }
  }
  if(activeCount == 0)
  {
    g_lineNoDetectCount++;
    return -1; // No line
  }
  g_lineNoDetectCount=0;
  return (sumPos / activeCount);
}

/* --- SHIFT LINE ERROR HISTORY --- */
void shiftLineErrorHistory(int error)
{
  for(int i=9; i>0; i--)
    g_lineErrorHist[i] = g_lineErrorHist[i-1];
  g_lineErrorHist[0] = error;
}

/* --- SUM LINE ERRORS --- */
int sumLineErrors(int count)
{
  int sum=0;
  for(int i=0; i<count; i++)
    sum += g_lineErrorHist[i];
  return sum;
}

/* --- HANDLE NO LINE DETECTED (SHARP TURN) --- */
void handleNoLine(void)
{
  // If not too long, do smaller spin; else bigger spin
  if(g_lineNoDetectCount < 25)
  {
    if(g_lineEndedLeft == 1) 
      driveMotors(15, -25);   // Spin left
    else
      driveMotors(-25, 15);   // Spin right
  }
  else
  {
    if(g_lineEndedLeft == 1)
      driveMotors(70, -53);   // Larger spin left
    else
      driveMotors(-53, 70);   // Larger spin right
  }
}

/* --- UPDATE LINE PID => OUTPUTS STEERING FOR LEFT AND RIGHT --- */
void updateLinePID(int *pLeftOut, int *pRightOut)
{
  int sensorPos = readLineSensors();        // Read QTR8
  if(sensorPos < 0)
  {
    handleNoLine();                        // If no line, do special maneuver
    *pLeftOut  = 0;                        // No normal PID in that scenario
    *pRightOut = 0;
    return;
  }
  // Normal line PID
  int error = 4500 - sensorPos;            // 4500 is midpoint
  shiftLineErrorHistory(error);            // Shift array
  int P = error;                           // Proportional
  int I = sumLineErrors(5);                // Sum last 5 errors for integral
  int D = error - g_lineLastError;         // Derivative
  g_lineLastError = error;                 // Update last error
  
  float pidOut = (P*g_lineKp) + (I*g_lineKi) + (D*g_lineKd);
  
  *pLeftOut  = (int) pidOut;      // steering for left
  *pRightOut = -(int) pidOut;     // steering for right
}

/* --- SHIFT DISTANCE ERROR HISTORY --- */
void shiftDistErrorHistory(float error)
{
  for(int i=9; i>0; i--)
    g_distanceErrorHist[i] = g_distanceErrorHist[i-1];
    g_distanceErrorHist[0] = error;
}

/* --- SUM DISTANCE ERRORS --- */
float sumDistErrors(int count)
{
  float sum=0.0f;
  for(int i=0; i<count; i++)
    sum += g_distanceErrorHist[i];
  return sum;
}

/* --- UPDATE DISTANCE PID --- */
float updateDistancePID(void)
{
  // Shift history
  shiftDistErrorHistory(g_distanceError);

  // P, I, D
  float P = g_distanceError;
  float I = sumDistErrors(5);  // Sum last 5
  float D = g_distanceError - g_distanceLastError;

  g_distanceLastError = g_distanceError; // Update last error

  float pidVal = (P * g_distKp) + (I * g_distKi) + (D * g_distKd);
  return pidVal; 
}

/* --- ULTRASONIC INIT --- */
void initUltrasonic(void)
{
  GPIOB->ODR &= ~(1<<12); // TRIG=0 initially
}

/* --- MEASURE DISTANCE (HC-SR04) --- */
float measureDistance(void)
{
  GPIOB->ODR &= ~(1<<12);   // TRIG=0
  delay_us(2);              // Wait 2 us
  GPIOB->ODR |= (1<<12);    // TRIG=1
  delay_us(10);             // High for 10 us
  GPIOB->ODR &= ~(1<<12);   // TRIG=0
  
  while((GPIOB->IDR & (1<<13)) == 0);  // Wait ECHO=1
  uint32_t start = TIM2->CNT;          // Record start
  while((GPIOB->IDR & (1<<13)) != 0);  // Wait ECHO=0
  uint32_t end   = TIM2->CNT;          // Record end
  
  uint32_t pulseWidth = (end > start) ? (end - start) : 0;
  float dist = (float)pulseWidth / 58.0f; // approximate => 1us ~ 58 => 1 cm
  return dist;
}

/* --- BUZZER CONTROL --- */
void buzzerControl(float dist)
{
  // Example: beep if near setpoint or if out of range
  if(dist > 58.0f && dist < 62.0f)
  {
    GPIOB->BSRR = (1<<8);     // Buzzer ON
    delay_ms(100);            // 100 ms beep
    GPIOB->BSRR = (1<<(8+16));// Buzzer OFF
  }
  else if(dist < 40.0f || dist > 80.0f)
  {
    GPIOB->BSRR = (1<<8);     // Continuous beep ON
  }
  else
  {
    GPIOB->BSRR = (1<<(8+16));// Buzzer OFF
  }
}