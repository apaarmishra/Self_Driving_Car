// RTOS Framework - Fall 2016
// J Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 4 Pushbuttons and 4 LEDs, UART

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "hw_nvic.h"

//-----------------------------------------------------------------------------
// Bitbanding references for LEDs' and Push buttons.
//-----------------------------------------------------------------------------

#define RED_LEDR   (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 0*4)))  //PB0 --> Right
#define RED_LEDL   (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 1*4)))  //PB1 --> Left
#define RED_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define MAX_CHARS 20 // Defining the maximum number of characters a user can enter as a command
#define SENSOR_NUM 8
#define FORWARD 0
#define BACK 1
#define LEFT 2
#define RIGHT 3
#define STOP 4

// task
#define STATE_INVALID    0 // no task
#define STATE_READY      1 // ready to run
#define STATE_DELAYED    2 // has run, but now awaiting timer
#define MAX_TASKS 3       // maximum number of valid tasks


// Global variables

bool ok;
char userCommand[MAX_CHARS];
uint8_t i = 0;
int position[10] = {0};           // To get the position number of each useful string in the complete command
char type[10] = {'\0'};           // To determine whether the strings in the command are numbers or characters
int parseCount = 0;               // To keep count on the number of strings in the command
bool comEnter = false;
bool dirFlag = false;
uint32_t ADCValue[SENSOR_NUM] = {0};
bool initial = true;
uint8_t magout_flag = 0;
float magVolt[SENSOR_NUM] = {0.0};
char str[10];
int bleStart = 0;
uint8_t previousMag = 8;
uint16_t sysCount = 0;
uint32_t tempDist = 0;
uint32_t distance = 0;
uint32_t speed = 0;
uint32_t rpm = 0;
uint32_t rotationCount = 0;
uint32_t circumference = 20;
bool countFlag = true;
uint8_t tailLights = 4;
uint32_t temp_rotationCount = 0;
uint8_t nPoleCount = 0;

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();

void yield();
void updateStackPointer(void *);
void storeStackPointer(uint32_t *);
static uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks

struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    void *pid;                     // used to uniquely identify thread
    void *sp;                      // location of stack pointer for thread
    uint8_t priority;              // 0=highest, 15=lowest
    uint8_t currentPriority;       // used for priority inheritance
    uint8_t skipCounter;           // used for prioritisation
} tcb[MAX_TASKS];

uint32_t stack[MAX_TASKS][256];

//-----------------------------------------------------------------------------
// RTOS Kernel
//-----------------------------------------------------------------------------

// Function to save the stack pointer in memory
void storeStackPointer(uint32_t *p)
{
    __asm(" STR R13, [R0]");
}

// Function to load the stack pointer to point to the stack of the particular task
void updateStackPointer(void *o)
{
    __asm(" LDR R13, [R0]");
}

void rtosInit()
{
    uint8_t i;

    // no tasks running
    taskCount = 0;

    // clear out tcb records
    for (i = 0; i < MAX_TASKS; i++)
    {
        tcb[i].state = STATE_INVALID;
        tcb[i].pid = 0;
    }

    // Initialisation of systick for 1ms system timer
    NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
    NVIC_ST_RELOAD_R = 39999;   // reload value
    NVIC_ST_CURRENT_R = 1;      // any write to current clears it
    NVIC_ST_CTRL_R = NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_INTEN; // enable SysTick with core clock and interrupt
}

int rtosScheduler()
{
    // Implementing prioritization to 16 levels
    bool ok;
    static uint8_t task = 0xFF;
    ok = false;
    while (!ok)
    {
        task++;
        if (task >= MAX_TASKS)
            task = 0;
        ok = (tcb[task].state == STATE_READY);
        if(initial)
            initial = false;
        else
        {
            if(tcb[task].skipCounter != 0)
            {
                ok = false;
                tcb[task].skipCounter--;
            }
            else
                tcb[task].skipCounter = tcb[task].priority;
        }
    }
    return task;
}

bool createThread(_fn fn, char name[], int priority)
{
    bool ok = false;
    uint8_t i = 0;
    uint8_t j = 0;
    bool found = false;
    if (taskCount < MAX_TASKS)
    {
        // make sure fn not already in list (prevent reentrancy)
        while (!found && (i < MAX_TASKS))
        {
            found = (tcb[i++].pid ==  fn);
        }
        if (!found)
        {
            // find first available tcb record
            i = 0;
            while (tcb[i].state != STATE_INVALID) {i++;}
            tcb[i].state = STATE_READY;
            tcb[i].pid = fn;
            // Preloading the stack to look like the task had run before
            stack[i][255] = fn;
            for(j=254;j>245;j--)
                stack[i][j] = (255-j);
            tcb[i].sp = &stack[i][j-1];
            tcb[i].priority = priority;
            tcb[i].currentPriority = priority;
            tcb[i].skipCounter = priority;
            // increment task count
            taskCount++;
            ok = true;
        }
    }
    return ok;
}

void rtosStart()
{
    NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE ;
    // Code to call the first task to be run, restoring the preloaded context
    taskCurrent = rtosScheduler();
    updateStackPointer(&tcb[taskCurrent].sp);// Initialize the SP with tcb[task_current].sp;
    __asm(" POP {R4-R11}");// Restore the stack to run the first process
    __asm(" POP {R3, PC}");
}

// Function to yield execution back to scheduler
void yield()
{
    // push registers, call scheduler, pop registers, return to new function
    __asm("  PUSH {R4-R11}");
    storeStackPointer(&tcb[taskCurrent].sp);
    taskCurrent = rtosScheduler();
    updateStackPointer(&tcb[taskCurrent].sp);
    __asm("  POP {R4-R11}");
    __asm(" POP {R3, PC}");
}

// Function to add support for the system timer
void systickIsr()
{
    sysCount++;
    if((sysCount == 500) || (sysCount == 1000))
    {
        switch(tailLights)
        {
            case FORWARD:
                    RED_LEDR = 0;
                    RED_LEDL = 0;
                    break;
            case BACK:
                    RED_LEDR ^= 1;
                    RED_LEDL ^= 1;
                    break;
            case LEFT:
                    RED_LEDR = 0;
                    RED_LEDL ^= 1;
                    break;
            case RIGHT:
                    RED_LEDR ^= 1;
                    RED_LEDL = 0;
                    break;
            case STOP:
                    RED_LEDR = 1;
                    RED_LEDL = 1;
                    break;
            default:
                    ;
        }
    }
    if(sysCount == 1000)
    {
        sysCount = 0;
        speed = distance - tempDist;
        rpm = temp_rotationCount * 60;
        tempDist = distance;
        temp_rotationCount = 0;
    }
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    // PWM is system clock / 2
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S)
            | SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_2;

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port B and E peripherals
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOF;

    // Enable back LEDs at Port B0,B1 and onboard RED LED PF1, GREEN PF3
    GPIO_PORTF_DIR_R |= 0x0A;  // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R |= 0x0A; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= 0x0A;  // enable LEDs and pushbuttons

    GPIO_PORTB_DIR_R |= 0x03;   // make bit 0 and 1 an output
    GPIO_PORTB_DR8R_R |= 0x03;  // set drive 0 and 1 strength to 2mA
    GPIO_PORTB_DEN_R |= 0x03;   // enable bit 0 and 1 for digital

    // Configure PWM on Port B
    GPIO_PORTB_DIR_R |= 0x30;   // make bit 4 and 5 an output
    GPIO_PORTB_DR2R_R |= 0x30;  // set drive 4 and 5 strength to 2mA
    GPIO_PORTB_DEN_R |= 0x30;   // enable bit 4 and 5 for digital
    GPIO_PORTB_AFSEL_R |= 0x30; // select auxilary function for bit 4
    GPIO_PORTB_PCTL_R = GPIO_PCTL_PB4_M0PWM2 | GPIO_PCTL_PB5_M0PWM3; // enable PWM on bit 4

    // Configure PWM on Port E
    GPIO_PORTE_DIR_R |= 0x30;   // make bits 4 and 5 outputs
    GPIO_PORTE_DR2R_R |= 0x30;  // set drive strength to 2mA
    GPIO_PORTE_DEN_R |= 0x30;   // enable bits 4 and 5 for digital
    GPIO_PORTE_AFSEL_R |= 0x30; // select auxilary function for bits 4 and 5
    GPIO_PORTE_PCTL_R = GPIO_PCTL_PE4_M0PWM4 | GPIO_PCTL_PE5_M0PWM5; // enable PWM on bits 4 and 5



    // Configure PWM module0 to drive RGB backlight
    // M0PWM2 (PB4), M0PWM1a
    // M0PWM3 (PB5), M0PWM1b
    // M0PWM4 (PE4), M0PWM2a
    // M0PWM5 (PE5), M0PWM2b
    SYSCTL_RCGC0_R |= SYSCTL_RCGC0_PWM0;             // turn-on PWM0 module
    __asm(" NOP");                                   // wait 3 clocks
    __asm(" NOP");
    __asm(" NOP");
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;                // reset PWM0 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM0_1_CTL_R = 0;                                // turn-off PWM0 generator 1
    PWM0_2_CTL_R = 0;                                // turn-off PWM0 generator 2
    PWM0_1_GENA_R = PWM_0_GENA_ACTCMPAD_ZERO | PWM_0_GENA_ACTLOAD_ONE;
                                                    // output 2 on PWM0, gen 1a, cmpb
    PWM0_1_GENB_R = PWM_0_GENB_ACTCMPBD_ZERO | PWM_0_GENB_ACTLOAD_ONE;
                                                    // output 3 on PWM0, gen 1a, cmpb
    PWM0_2_GENA_R = PWM_0_GENA_ACTCMPAD_ZERO | PWM_0_GENA_ACTLOAD_ONE;
                                                     // output 4 on PWM0, gen 2a, cmpa
    PWM0_2_GENB_R = PWM_0_GENB_ACTCMPBD_ZERO | PWM_0_GENB_ACTLOAD_ONE;
                                                     // output 5 on PWM0, gen 2b, cmpb
    PWM0_1_LOAD_R = 1024;                            // set period to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
    PWM0_2_LOAD_R = 1024;
    // invert outputs for duty cycle increases with increasing compare values
    PWM0_INVERT_R = PWM_INVERT_PWM2INV | PWM_INVERT_PWM3INV |  PWM_INVERT_PWM4INV | PWM_INVERT_PWM5INV; // PB4 and PB5

    PWM0_1_CMPA_R = 0;                               // PB4 (0=always low, 1023=always high)
    PWM0_1_CMPB_R = 0;                               // PB5
    PWM0_2_CMPA_R = 0;                               // PE4
    PWM0_2_CMPB_R = 0;                              // PE5

    PWM0_1_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM0 generator 1
    PWM0_2_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM0 generator 2
    PWM0_ENABLE_R = PWM_ENABLE_PWM2EN |PWM_ENABLE_PWM3EN | PWM_ENABLE_PWM4EN |PWM_ENABLE_PWM5EN;

    SYSCTL_RCGCADC_R |= 1;                           // turn on ADC module 0 clocking
    GPIO_PORTD_AFSEL_R |= 0x0F;                      // select alternative functions for AIN4,5,6,7 (PD3,2,1,0)
    GPIO_PORTD_DEN_R &= ~0x0F;                       // turn off digital operation on pin PD3,2,1,0
    GPIO_PORTD_AMSEL_R |= 0x0F;
    GPIO_PORTE_AFSEL_R |= 0x0F;                      // select alternative functions for AIN3,2,1,0(PE0,1,2,3)
    GPIO_PORTE_DEN_R &= ~0x0F;                       // turn off digital operation on pin PE0,PE1,PE2,PE3
    GPIO_PORTE_AMSEL_R |= 0x0F;
    ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN0;                // disable sample sequencer 0 (SS0) for programming
    ADC0_EMUX_R = ADC_EMUX_EM0_PROCESSOR;            // select SS0 bit in ADCPSSI as trigger
    ADC0_SSMUX0_R |= 0x01237654;                         // set first sample to AIN3, AIN4, AIN5, AIN6 and AIN7
    ADC0_SSCTL0_R = ADC_SSCTL0_END7;                 // mark seventh sample as the end
    ADC0_SAC_R |= ADC_SAC_AVG_4X;                    // 4x hardware oversampling
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN0;                 // enable SS0 for operation

}

void configUart1Addr()
{
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;         // turn-on UART1, leave other uarts in same status
    GPIO_PORTC_DEN_R |= 0x30;                        // default, added for clarity
    GPIO_PORTC_AFSEL_R |= 0x30;                      // default, added for clarity
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_U1TX | GPIO_PCTL_PC4_U1RX;

    // Configure UART1 to 38400 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART1_CTL_R = 0;                                 // turn-off UART1 to allow safe programming
    UART1_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART1_IBRD_R = 260;                              // r = 40 MHz / (Nx9.6kHz), set floor(r)=260, where N=16
    UART1_FBRD_R = 27;                               // round(fract(r)*64)=27
    UART1_LCRH_R |= UART_LCRH_WLEN_8 | UART_LCRH_SPS | UART_LCRH_PEN;// configure for 8N1 and parity enabled w/o 16-level FIFO
    UART1_IM_R |= UART_IM_RXIM;                      // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_UART1-16);               // turn-on interrupt 22
    UART1_CTL_R |= UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module
}

void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*3
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             B    WMS_LOOP0");       // 1*3
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

// read ADC
void readAdc0Ss3()
{
    ADC0_PSSI_R |= ADC_PSSI_SS0;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY)
        yield();           // wait until SS3 is not busy
    int i=0;
    while(!(ADC0_SSFSTAT0_R & ADC_SSFSTAT0_EMPTY))
    {
        ADCValue[i]= ADC0_SSFIFO0_R;                    // get single result from the FIFO
        magVolt[i] = ((float)ADCValue[i] * 3.3)/4095;
        i++;
    }
}

void putcUart1(char c)
{
    while (UART1_FR_R & UART_FR_TXFF);
    UART1_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart1(char* str)
{
    int i;
    for (i = 0; i < strlen(str); i++)
      putcUart1(str[i]);
}

void U1RxIsr()
{
    while (UART1_FR_R & UART_FR_RXFE);
    int data = UART1_DR_R;
    char data1 = data & 0xFF;
    if((data1 != '\n') && (data1 != '.'))
    {
        userCommand[i] = data1;
        dirFlag = true;
        i++;
    }
    else
    {
        userCommand[i] = '\0';
        dirFlag = false;
        i = 0;
        comEnter = true;
    }
    UART1_ICR_R |= UART_ICR_RXIC;           //clear the interrupt
}

// STEP 3(a)
// Function name: upperToLower()
// The function upperToLower() is used to convert the uppercase characters in the entered command if any to lowercase characters.

void upperToLower(char *userCommand)
{
    int commandIndex = 0;
    while(userCommand[commandIndex])
    {
        if(userCommand[commandIndex] >= 'A' && userCommand[commandIndex] <= 'Z')
            userCommand[commandIndex] = userCommand[commandIndex]+32;
        commandIndex++;
    }
}

// STEP 3(b)
// Function name: parseCommand()
// The function parseCommand() is used to determine the position of the important stuff in the string

void parseCommand()
{
    int positionIndex = 0;
    int commandIndex = 0;
    bool flag = false;
    while(userCommand[commandIndex])
    {
        if(userCommand[commandIndex] >= 'a' && userCommand[commandIndex] <= 'z')
        {
            if(flag == false)
            {
                position[positionIndex] = commandIndex;
                type[positionIndex++] = 'a';
                flag = true;
            }
        }
        else if(userCommand[commandIndex] >= '0' && userCommand[commandIndex] <= '9')
        {
            if(flag == false)
            {
                position[positionIndex] = commandIndex;
                type[positionIndex++] = 'n';
                flag = true;
            }
        }
        else if(userCommand[commandIndex] == '&')
        {
                position[positionIndex] = commandIndex;
                type[positionIndex++] = 's';
        }
        else if(userCommand[commandIndex] == '_')
        {
            if(flag == false)
            {
                position[positionIndex] = commandIndex;
                type[positionIndex++] = 'o';
                flag = true;
            }
        }
        else
            flag = false;
        commandIndex++;
        parseCount = positionIndex;
    }
}

// STEP 3(c)
// Function name: parseCommand()
// The function parseCommand() is used to determine the position of the important stuff in the string

void delimiterToNull()
{
    int commandIndex = 0;
    while(userCommand[commandIndex])
    {
        if(userCommand[commandIndex] >= '0' && userCommand[commandIndex] <= '9');
        else if(userCommand[commandIndex] >= 'a' && userCommand[commandIndex] <= 'z');
        else if(userCommand[commandIndex] >= '!');
        else
            userCommand[commandIndex] = '\0';
        commandIndex++;
    }
}

// STEP 4(a)
// Function name: isCommand()
// The function isCommand() is used to check whether the user has entered the proper command or not.

bool isCommand(char *command, int argNumber)
{
    char *dummy = &userCommand[position[0]];
    if ((strcmp(dummy,command) == 0) && ((parseCount-1) == argNumber))
        return true;
    else
        return false;
}

void intToString(int value)
{
    uint8_t i = 0;
    uint8_t j = 0;

    if(value == 0)
    {
        str[0] = '0';
        str[1] = '\0';
    }
    else
    {
        while(value!=0)
        {
            str[i++] = value%10 + '0';
            value = value/10;
        }
        for(j = 0; j < i/2; j++)
        {
            str[j] ^= str[i-j-1];
            str[i-j-1] ^= str[j];
            str[j] ^= str[i-j-1];
        }
        str[i] = '\0';
    }
}

// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose

void magnet()
{
    while (true)
    {
        // REQUIRED: add code to read the Hall effect sensors that maintain lane centering and report the lane position relative to center
        // Read sensor
        //ADCValue[0] ----> Right most sensor
        //ADCValue[1] ----> Right sensor
        //ADCValue[2] ----> Center sensor
        //ADCValue[3] ----> Left sensor
        //ADCValue[4] ----> Left most sensor
        readAdc0Ss3();
        int16_t diff03 = 0;
        int16_t diff13 = 0;
        int16_t diff23 = 0;
        int16_t diff43 = 0;
        int16_t diff53 = 0;
        int16_t diff63 = 0;
        diff03 = ADCValue[0] - ADCValue[3];
        diff13 = ADCValue[1] - ADCValue[3];
        diff23 = ADCValue[2] - ADCValue[3];
        diff43 = ADCValue[4] - ADCValue[3];
        diff53 = ADCValue[5] - ADCValue[3];
        diff63 = ADCValue[6] - ADCValue[3];
        if((ADCValue[0] < 950) || (ADCValue[1] < 950) || (ADCValue[2] < 950) || (ADCValue[3] < 950) || (ADCValue[4] < 950) || (ADCValue[5] < 950) || (ADCValue[6] < 950))
        {
            GREEN_LED = 1;
            bleStart = 0;
        }
        else if((ADCValue[0] < 1300) && (ADCValue[1] < 1300) && (ADCValue[2] < 1300) && (ADCValue[3] < 1300) && (ADCValue[4] < 1300) && (ADCValue[5] < 1300) && (ADCValue[6] < 1300))
        {
            magout_flag = previousMag;
            RED_LED = 0;
            GREEN_LED = 0;
        }
        else if((ADCValue[0] > 1400) || ((ADCValue[0] > 1301) && (diff03 > diff13)))
        {
            magout_flag = previousMag = 0;//rightmost
            RED_LED = 1;
        }
        else if((ADCValue[1] > 1400) || ((1301 < ADCValue[1]) && ((diff13 > 0) || (diff13 < diff23))))
        {
           magout_flag = previousMag = 1;//right
           RED_LED = 1;
        }
        else if((ADCValue[2] > 1400) || ((1301 < ADCValue[2]) && (diff23 > 0)))
        {
           magout_flag = previousMag = 2;//slight right
           RED_LED = 1;
        }
        else if((ADCValue[3] > 1400) || ((1301 < ADCValue[3]) && ((diff23 < 0) || (diff43 < 0))))
        {
            magout_flag = previousMag = 3;//center
            RED_LED = 1;
        }
        else if((ADCValue[4] > 1400) || ((1301 < ADCValue[4]) && ((diff43 > 0) || (diff43 > diff53))))
        {
            magout_flag = previousMag = 4;//slight left
            RED_LED = 1;

        }
        else if((ADCValue[5] > 1400) || ((1301 < ADCValue[5]) && ((diff53 > 0) || (diff53 > diff63))))
        {
            magout_flag = previousMag = 5;//left
            RED_LED = 1;

        }
        else if((ADCValue[6] > 1400) || ((1301 < ADCValue[6]) && ((diff63 > 0))))
        {
            magout_flag = previousMag = 6;//leftmost
            RED_LED = 1;
        }
        if (bleStart == 0)
        {
            PWM0_1_CMPA_R = 0;     //PB4
            PWM0_2_CMPA_R = 0; //PE4
            //Reverse motor 2
            PWM0_1_CMPB_R = 0;    //PB5
            PWM0_2_CMPB_R = 0; //PE5
            tailLights = 4;
            GREEN_LED = 0;
            yield();
        }
        else if ((magout_flag == 3) && (bleStart == 1))   //forward - Center
        {
            PWM0_1_CMPA_R = 0;    //PB4
            PWM0_2_CMPA_R = 780; //PE4 //900 //PB4 ip to IN1 left motor
            //forward motor 2
            PWM0_1_CMPB_R = 0;     //PB5 //900 PB5 ti IN3 right motor
            PWM0_2_CMPB_R = 705;
            tailLights = 0;
            yield();
        }
        else if ((magout_flag == 4) && (bleStart == 1))  // slight left
        {
            PWM0_1_CMPA_R = 0;     //PB4
            PWM0_2_CMPA_R = 800; //PE4 //700
            //Reverse motor 2
            PWM0_1_CMPB_R = 0;    //PB5 //900
            PWM0_2_CMPB_R = 720; //PE5
            tailLights = 2;
            yield();
        }
        else if ((magout_flag == 5) && (bleStart == 1))  //left
        {
            PWM0_1_CMPA_R = 0;     //PB4
            PWM0_2_CMPA_R = 758; //PE4 //800
            //Reverse motor 2
            PWM0_1_CMPB_R = 0;    //PB5 //900
            PWM0_2_CMPB_R = 850; //PE5
            tailLights = 2;
            yield();
        }
        else if ((magout_flag == 6) && (bleStart == 1))  //extreme left
        {
            PWM0_1_CMPA_R = 0;     //PB4
            PWM0_2_CMPA_R = 680; //PE4 //800
            //Reverse motor 2
            PWM0_1_CMPB_R = 0;    //PB5 //900
            PWM0_2_CMPB_R = 920; //PE5
            tailLights = 2;
            yield();
        }
        else if ((magout_flag == 2) && (bleStart == 1))  // slight right
        {
            PWM0_1_CMPA_R = 0;     //PB4
            PWM0_2_CMPA_R = 800; //PE4 //900
            //Reverse motor 2
            PWM0_1_CMPB_R = 0;    //PB5 //800
            PWM0_2_CMPB_R = 700; //PE5
            tailLights = 3;
            yield();
        }
        else if ((magout_flag == 1) && (bleStart == 1))  //  right
        {
            PWM0_1_CMPA_R = 0;     //PB4
            PWM0_2_CMPA_R = 900; //PE4 //900
            //Reverse motor 2
            PWM0_1_CMPB_R = 0;    //PB5 //700
            PWM0_2_CMPB_R = 640; //PE5
            tailLights = 3;
            yield();
        }
        else if ((magout_flag == 0) && (bleStart == 1))   //extreme right
        {
            PWM0_1_CMPA_R = 0;     //PB4
            PWM0_2_CMPA_R = 950; //PE4 //900
            //Reverse motor 2
            PWM0_1_CMPB_R = 0;    //PB5 //900
            PWM0_2_CMPB_R = 600; //PE5
            tailLights = 3;
            yield();
        }
        yield();
    }
}

void bluetooth()
{
    while(true)
    {
        if(comEnter)
        {
            upperToLower(&userCommand);    // Function to convert all the characters in the command to lowercase
            parseCommand();              // Function to divide the entire user command into substrings
            delimiterToNull();           // Function to convert characters other than alphabets,numbers and some special characters to null characters
            putsUart1("\r\n");
            if((isCommand("f",0)))
            {
                PWM0_1_CMPA_R = 0;    //PB4 ip to IN1 left motor
                PWM0_2_CMPA_R = 892; //PE4
                //forward motor 2
                PWM0_1_CMPB_R = 0;     //PB5 ti IN3 right motor
                PWM0_2_CMPB_R = 824; //PE5
                waitMicrosecond(100000);
                previousMag = 3;
                tailLights = 0;
                bleStart = 1;
            }
            else if(isCommand("b",0))
            {
                PWM0_1_CMPA_R = 892;     //PB4
                PWM0_2_CMPA_R = 0; //PE4
                //Reverse motor 2
                PWM0_1_CMPB_R = 824;    //PB5
                PWM0_2_CMPB_R = 0; //PE5
                waitMicrosecond(100000);
                tailLights = 1;
                bleStart = 1;
            }
            else if(isCommand("l",0))
            {
                PWM0_1_CMPA_R = 0;     //PB4
                PWM0_2_CMPA_R = 758; //PE4 //800
                //Reverse motor 2
                PWM0_1_CMPB_R = 0;    //PB5 //900
                PWM0_2_CMPB_R = 850; //PE5
                waitMicrosecond(100000);
                previousMag = 5;
                tailLights = 2;
                bleStart = 1;
            }
            else if(isCommand("r",0))
            {
                PWM0_1_CMPA_R = 0;     //PB4
                PWM0_2_CMPA_R = 900; //PE4 //900
                //Reverse motor 2
                PWM0_1_CMPB_R = 0;    //PB5 //700
                PWM0_2_CMPB_R = 640; //PE5
                waitMicrosecond(100000);
                previousMag = 1;
                tailLights = 3;
                bleStart = 1;
            }
            else if(isCommand("s",0))
            {
                PWM0_1_CMPA_R = 0;     //PB4
                PWM0_2_CMPA_R = 0; //PE4
                //Reverse motor 2
                PWM0_1_CMPB_R = 0;    //PB5
                PWM0_2_CMPB_R = 0; //PE5
                waitMicrosecond(100000);
                tailLights = 4;
                bleStart = 0;
            }
            else if((isCommand("i",0)))
            {
                intToString(distance);
                putsUart1("\r\nDistance = ");
                putsUart1(str);
                intToString(speed);
                putsUart1("\r\nSpeed = ");
                putsUart1(str);
                intToString(rpm);
                putsUart1("\r\nRPM = ");
                putsUart1(str);
            }
            comEnter = false;
            yield();
        }
        else
            yield();
    }
}

void carStats()
{
    while(true)
    {
        if ((ADCValue[7] > 1500) && (countFlag == true))
        {
            rotationCount++;
            temp_rotationCount++;
            countFlag = false;
        }
        if (ADCValue[7] < 1300)
            countFlag = true;
        distance = rotationCount * circumference;
        yield();
    }
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    // Initialize hardware
    initHw();
    configUart1Addr();
    rtosInit();

    // Power-up flash
    RED_LED = 1;
    waitMicrosecond(250000);
    RED_LED = 0;
    waitMicrosecond(250000);

    // Add required idle process
    ok = createThread(magnet, "Magnet", 0);
    // Add other processes
    ok &= createThread(bluetooth, "Bluetooth", 0);
    ok &= createThread(carStats, "CarStats",0);

    // Start up RTOS
    if (ok)
        rtosStart(); // never returns
    else
        RED_LED = 1;
    return 0;
    // don't delete this unreachable code
    // if a function is only called once in your code, it will be
    // accessed with two goto instructions instead of call-return,
    // so any stack-based code will not function correctly
    yield();
}
