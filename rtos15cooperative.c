// RTOS Framework - Spring 2016
// J Losh

// Student Name: Rohan Narula.

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
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include <string.h>

// REQUIRED: correct these bitbanding references for green and yellow LEDs (temporary to guarantee compilation)
#define RED_LED       (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*4)))
#define ORANGE_LED    (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4)))
#define GREEN_LED     (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))
#define YELLOW_LED    (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4)))
#define PUSH_BUTTON1  (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 0*4)))
#define PUSH_BUTTON2  (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 1*4)))
#define PUSH_BUTTON3  (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 2*4)))
#define PUSH_BUTTON4  (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 3*4)))


//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_QUEUE_SIZE 10
struct semaphore
{
  unsigned int count;
  unsigned int queueSize;
  unsigned int processQueue[MAX_QUEUE_SIZE]; // store task index here
} *s, keyPressed, keyReleased, flashReq;


// task
#define STATE_INVALID    0 // no task
#define STATE_READY      1 // ready to run
#define STATE_BLOCKED    2 // has run, but now blocked by semaphore
#define STATE_DELAYED    3 // has run, but now awaiting timer

#define MAX_TASKS 20       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks

char input[30],new_input[15],stringBuffer[40];

uint8_t i = 0;
uint32_t lastTime = 0x1312D00;
double  totalTime = 0;
struct _tcb
{
  uint8_t state;                 // see STATE_ values above
  void *pid;                     // used to uniquely identify process
  void *sp;                      // location of stack pointer for process
  void *taskSemaphore;           // location of the semaphore
  uint8_t priority;              // 0=highest, 7=lowest
  uint8_t currentPriority;       // used for priority inheritance
  uint32_t ticks;                // ticks until sleep complete
  uint32_t skipcount;
  char *label;
  double processtime;
  double avgTime;
  void *restorePID;
} tcb[MAX_TASKS];

uint32_t stack[MAX_TASKS][256];

//-----------------------------------------------------------------------------
// RTOS Kernel
//-----------------------------------------------------------------------------

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
  // REQUIRED: systick for 1ms system timer
}

int rtosScheduler() //Change the rtosScheduler making it more simpler and see what happens. remove ok.
{
  // REQUIRED: Implement prioritization to 8 levels
  TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
  tcb[taskCurrent].processtime += lastTime - TIMER1_TAV_R;
  lastTime = TIMER1_TAV_R;
  bool ok;
  static uint8_t task = 0xFF;
  ok = false;
  while (!ok)
  {
    task++;
    if (task >= MAX_TASKS)
      task = 0;
    if(tcb[task].state == STATE_READY){
        if (tcb[task].skipcount == 0)
        {
            ok = 1;
            tcb[task].skipcount = tcb[task].priority;
        }
        else{
            tcb[task].skipcount--;
        }
    }
//    ok = (tcb[task].state == STATE_READY);
  }
  TIMER1_CTL_R |= TIMER_CTL_TAEN;                 // turn-on timer
  return task;
}

bool createProcess(_fn fn, int priority, char *label)
{
    bool ok = false;
       uint8_t i = 0;
       bool found = false;
       // REQUIRED: take steps to ensure a task switch cannot occur
       // save starting address if room in task list
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
           tcb[i].skipcount = priority;
           tcb[i].label = label;
           tcb[i].processtime = 0;
           tcb[i].restorePID = fn;
           stack[i][240] = 0;
           stack[i][241] = 1;
           stack[i][242] = 2;
           stack[i][243] = 3;
           stack[i][244] = 4;
           stack[i][245] = 5;
           stack[i][246] = 6;
           stack[i][247] = 7;
           stack[i][248] = 8;
           stack[i][249] = 9;
           stack[i][250] = 10;
           stack[i][251] = 11;
           stack[i][252] = 12;
           stack[i][253] = (uint32_t) fn;


           // REQUIRED: preload stack to look like the task had run before
           tcb[i].sp = &stack[i][240]; // REQUIRED: + offset as needed for the pre-loaded stack
           tcb[i].priority = priority;
           tcb[i].currentPriority = priority;
           // increment task count
           taskCount++;
           ok = true;
         }
       }
       // REQUIRED: allow tasks switches again
       return ok;
}

// REQUIRED: modify this function to destroy a process
void destroyProcess(_fn fn)
{
    uint8_t i,j;
    struct semaphore *semaphorePointer;
    for(i = 0;i < MAX_TASKS; i++)
    {

        if(tcb[i].pid == fn)
        {
            tcb[i].state = STATE_INVALID;
            tcb[i].avgTime = 0;
            tcb[i].processtime = 0;
            tcb[i].pid = 0;
            semaphorePointer = tcb[i].taskSemaphore;
            for(j = 0;j<MAX_QUEUE_SIZE;j++){
                if(semaphorePointer->processQueue[j] == tcb[i].pid)
                {

                    while(j<MAX_QUEUE_SIZE)
                    {
                        semaphorePointer->processQueue[j-1] = semaphorePointer->processQueue[j];
                        j++;
                    }
                    semaphorePointer->queueSize--;
                    break;
                }
            }
        }
    }
}

void stackvalue(uint32_t a){
    __asm("   mov sp, r0    ");
    __asm("   sub sp,#8     ");
}

void rtosStart()
{

  // REQUIRED: add code to call the first task to be run, restoring the preloaded context
  _fn fn;
  taskCurrent = rtosScheduler();
  stackvalue((uint32_t) tcb[taskCurrent].sp);
  // Add code to initialize the SP with tcb[task_current].sp;
  // Restore the stack to run the first process

  __asm("   pop {r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12}    ");
  __asm("   pop {pc}    ");
}

void init(void* p, int count)
{
  s = p;
  s->count = count;
  s->queueSize = 0;
}
void* stack2tcb(){
    __asm("    mov r0, sp   ");
}
// REQUIRED: modify this function to yield execution back to scheduler
void yield()
{
    // push registers, call scheduler, pop registers, return to new function
    __asm("    pop {r3,lr}  ");
    __asm("    push {lr}    ");
    __asm("    push {r0,r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12}");
    tcb[taskCurrent].sp = stack2tcb();
    taskCurrent = rtosScheduler();
    stackvalue((uint32_t) tcb[taskCurrent].sp);
    __asm("    pop {r0,r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12}");
    __asm("    pop {pc}");


}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses
void sleep(uint32_t tick)
{
    // push registers, set state to delayed, store timeout, call scheduler, pop registers,
    // return to new function (separate unrun or ready processing)
    __asm("    pop {r3,lr}  ");
    __asm("    push {lr}    ");
    __asm("    push {r0,r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12}     ");
    tcb[taskCurrent].sp = stack2tcb();
    tcb[taskCurrent].ticks = tick;
    tcb[taskCurrent].state = STATE_DELAYED;
    taskCurrent = rtosScheduler();
    stackvalue((uint32_t) tcb[taskCurrent].sp);
    __asm("    pop {r0,r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12}      ");
    __asm("    pop {pc}     ");

}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler
void wait(struct semaphore *pSemaphore)
{
    if (pSemaphore->count > 0)
    {
        pSemaphore->count--;
    }
    else{
        pSemaphore->processQueue[pSemaphore->queueSize] = tcb[taskCurrent].pid;
        pSemaphore->queueSize++;
        tcb[taskCurrent].taskSemaphore = pSemaphore;
        __asm("    pop {r3,lr}  ");
        __asm("    push {lr}    ");
        __asm("    push {r0,r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12}     ");
        tcb[taskCurrent].sp = stack2tcb();
        tcb[taskCurrent].state = STATE_BLOCKED;
        taskCurrent = rtosScheduler();
        stackvalue((uint32_t) tcb[taskCurrent].sp);
        __asm("    pop {r0,r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12}      ");
        __asm("    pop {pc}     ");
    }


}

// REQUIRED: modify this function to signal a semaphore is available
void post(struct semaphore* pSemaphore)
{
    uint8_t i,j;
    pSemaphore->count++;
    if(pSemaphore->queueSize > 0){
        for(j = 0; j < MAX_TASKS; j++){
            if(tcb[j].pid ==  pSemaphore->processQueue[0]){
                tcb[j].state = STATE_READY;
            }
        }
        for(i = 1; i <= pSemaphore->queueSize;i++)
        {
            pSemaphore->processQueue[i-1] = pSemaphore->processQueue[i];
        }
        pSemaphore->queueSize--;
        pSemaphore->count--;
    }


}

// REQUIRED: modify this function to add support for the system timer
void systickIsr()
{
    uint8_t i = 0;
    for(i=0;i<MAX_TASKS;i++){
        if(tcb[i].state == STATE_DELAYED){
            tcb[i].ticks--;

        if(tcb[i].ticks == 0)
        {
            tcb[i].state = STATE_READY;
        }
    }
    }
}


//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOD|SYSCTL_RCGC2_GPIOC|SYSCTL_RCGC2_GPIOA;

    // REQUIRED: Add initialization for orange, red, green, and yellow LEDs
    //           4 pushbuttons, and uart

    // Off-Boards PUSH BUTTON
    GPIO_PORTD_DIR_R = 0x00;
    GPIO_PORTD_DR2R_R = 0x00;
    GPIO_PORTD_DEN_R = 0x0F;
    GPIO_PORTD_PUR_R = 0x0F;
    // Off-Boards LED
    GPIO_PORTC_DIR_R = 0xF0;
    GPIO_PORTC_DR2R_R = 0xF0;
    GPIO_PORTC_DEN_R = 0xF0;

    // Systick Configuration for 1ms
    NVIC_ST_CTRL_R = 0;
    NVIC_ST_RELOAD_R = 39999;
    NVIC_ST_CURRENT_R = 0;
    NVIC_ST_CTRL_R = 0x07;

    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0; // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                         // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                       // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                 // use system clock (40 MHz)
    UART0_IBRD_R = 21; // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    // Configure Timer 1 as the time base
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 0x1312D00;                      // set load value to 40e6 for 1 Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)


}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
                                                // Approx clocks per us
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

// REQUIRED: add code to return a value from 0-15 indicating which of 4 PBs are pressed
uint8_t readPbs()
{

    return ((!PUSH_BUTTON1)+(!PUSH_BUTTON2)*2+(!PUSH_BUTTON3)*4+(!PUSH_BUTTON4)*8);
}

void putcUart0(char c) {
    while (UART0_FR_R & UART_FR_TXFF){
        yield();
    }
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str) {
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
        putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0() {
    while (UART0_FR_R & UART_FR_RXFE){
        yield();
    }
    return UART0_DR_R & 0xFF;
}


void getString(){
    char ch;
    ch = getcUart0();
    putcUart0(ch);
    if (ch == 13){
        uint8_t j = 0;
        input[i] = ch;
        i++;
        for (i = 0; i < strlen(input); i++) {
            if(input[i] == 32){
                if(input[i-1] == 32 || i == 0){
                }
                else{
                    new_input[j] = input[i];
                    j++;
                }
            }
            else{
                new_input[j] = input[i];
                j++;
            }

        }
        for (i = 0; i <= 30; i++) {
            input[i] = '\0';
        }
        i = 0;

    }
    else if (ch == 8) {
        i--;
        putcUart0(32);
        putcUart0(8);
        input[i] = 0;
    }  else {
        input[i] = ch;
        i++;
    }
}

void TeratermInit(){
    uint8_t j;
    for (j = 0; j <= 30; j++) {
        input[j] = 0;
    }
    putsUart0("Welcome to EE6314 RTOS Project..\r\n");
    putsUart0("The following are the kernal functions that have been provided:\r\n");
    putsUart0("\r\nps: Displays the Process name, PID, state and CPU time percentage of the process\r\n");
    putsUart0("\r\nipcs: Displays the resources available and the processes waiting for it.\r\n");
    putsUart0("\r\nkill: Destroys the process using its PID\r\n");
    putsUart0("\r\nreboot: Restarts the controller\r\n");
    putsUart0("\r\npidof: Returns the PID of the process by using its name\r\n");
    putsUart0("\r\n<ProcessName>&: creates a process using its name\r\n");
    putsUart0("\r\nhelp: Display the commands available and their description\r\n");
    putsUart0("\r\n");
    putsUart0("Enter the command\r\n");
}

void Timer1Isr()
{
    uint8_t index;
    totalTime = 0;
    for(index = 0; index < MAX_TASKS; index++){
        if(tcb[index].state != STATE_INVALID){
            tcb[index].avgTime = tcb[index].processtime;
            tcb[index].processtime = 0;
            totalTime += tcb[index].avgTime;
        }
    }
    lastTime = 0x1312D00;

    TIMER1_ICR_R = TIMER_ICR_TATOCINT;
}

int CheckOpcodeTwo(char *name) {
    int i = 0;
    while(name[i] != 0) {
        if (!(48 <= name[i] && name[i] <= 57)) {
            return 1;
        }
        i++;
    }
    return 0;

}
// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
  while(true)
  {
    ORANGE_LED = 1;
    waitMicrosecond(1000);
    ORANGE_LED = 0;
    waitMicrosecond(1000);
    yield();

  }
}

void flash4Hz()
{
  while(true)
  {
    GREEN_LED ^= 1;
    sleep(125);
  }
}


void oneshot()
{
  while(true)
  {
    wait(&flashReq);
    YELLOW_LED = 1;
    sleep(1000);
    YELLOW_LED = 0;
  }
}

void partOfLengthyFn()
{
  // represent some lengthy operation
  waitMicrosecond(1000);
  // give another process a chance
  yield();
}

void lengthyFn()
{
  uint16_t i;
  while(true)
  {
    for (i = 0; i < 4000; i++)
    {
      partOfLengthyFn();
    }
    RED_LED ^= 1;
  }
}

void readKeys()
{
  uint8_t buttons;
  while(true)
  {
    wait(&keyReleased);
    buttons = 0;
    while (buttons == 0)
    {
      buttons = readPbs();
      yield();
    }
    post(&keyPressed);
    if ((buttons & 1) != 0)
    {
      YELLOW_LED ^= 1;
      RED_LED = 1;
    }
    if ((buttons & 2) != 0)
    {
      post(&flashReq);
      RED_LED = 0;
    }
    if ((buttons & 4) != 0)
    {
      createProcess(flash4Hz, 0, "flash4Hz");
    }
    if ((buttons & 8) != 0)
    {
      destroyProcess(flash4Hz);
    }
    yield();
  }
}

void debounce()
{
  uint8_t count;
  while(true)
  {
    wait(&keyPressed);
    count = 3;
    while (count != 0)
    {
      sleep(3);
      if (readPbs() == 0)
        count--;
      else
        count = 3;
    }
    post(&keyReleased);
  }
}

void uncooperative()
{
  while(true)
  {
    while (readPbs() == 8)
    {
    }
    yield();
  }
}

void CommandWindow(){
    uint8_t task, index, i = 0, j = 0, length, length2;
    char OpcodeOne[20], OpcodeTwo[15];
    double cpuTime;
    TeratermInit();
    while(1){
    getString();
    length2 = strlen(new_input);
    if(new_input[length2-1] == 13){
        new_input[length2-1] = '\0';
        for(index = 0; index < 20; index++){
                OpcodeOne[index] = '\0';
            }
            for(index = 0; index < 15; index++){
                    OpcodeTwo[index] = '\0';
                }
            i = 0;
            while(new_input[i]!=32 && new_input[i]!=0){
                OpcodeOne[i] = new_input[i];
                i++;
            }
            OpcodeOne[i] = '\0';
            i++;
            j = 0;
            while(new_input[i]!=0){
                OpcodeTwo[j] = new_input[i];
                i++;
                j++;
            }
            OpcodeTwo[j] = '\0';
            length = strlen(OpcodeOne);
            if(strcmp("reboot", OpcodeOne) == 0){
                if(OpcodeTwo[0] != '\0'){
                    putsUart0("\r\nThis command does not accept any parameters/arguments.\r\n");
                    putsUart0("\r\nPlease write reboot only.\r\n");
                }
                else{
                    ResetISR();
                }
                putsUart0("\r\n");
                putsUart0("Enter the command\r\n");
                for(index = 0; index < 15; index++){
                        new_input[index] = '\0';
                        }
            }
            else if(strcmp("ps", OpcodeOne) == 0){
                if(OpcodeTwo[0] != '\0')
                {
                    putsUart0("\r\nThis command does not accept any parameters/arguments.\r\n");
                    putsUart0("\r\nPlease write ps only.\r\n");

                }
                else{
                    putsUart0("\r\n");
                    for(task = 0; task < MAX_TASKS; task++){
                        if(tcb[task].pid != 0 && tcb[task].pid != tcb[7].pid){
                            sprintf(stringBuffer, "PID ID:--> %u\r\n", tcb[task].pid);
                            putsUart0(stringBuffer);
                            sprintf(stringBuffer, "Process Name:--> %s\r\n", tcb[task].label);
                            putsUart0(stringBuffer);
                            cpuTime = (tcb[task].avgTime*100)/totalTime;
                            sprintf(stringBuffer, "CPU time percentage:--> %f\r\n", cpuTime);
                            putsUart0(stringBuffer);
                            if(tcb[task].state == 1){
                                sprintf(stringBuffer, "Status:--> Ready\r\n");

                            }
                            else if(tcb[task].state == 2){
                                sprintf(stringBuffer, "Status:--> Blocked\r\n");

                            }
                            else if(tcb[task].state == 3){
                                sprintf(stringBuffer, "Status:--> Delayed\r\n");

                            }
                            putsUart0(stringBuffer);
                            putcUart0('\n');
                        }

                }

                    }
                for(index = 0; index < 15; index++){
                new_input[index] = '\0';
                }
                putsUart0("\r\n");
                putsUart0("Enter the command\r\n");
                }
            else if(strcmp("ipcs", OpcodeOne) == 0){
                uint8_t wait;
                putsUart0("\r\n");
                putsUart0("\r\nFlashReq:\r\n");
                for(index = 0; index < MAX_TASKS; index++){
                    if(OpcodeTwo[0] != '\0'){
                        putsUart0("\r\nThis command does not accept any parameters/arguments.\r\n");
                        putsUart0("\r\nPlease write ipcs only.\r\n");
                    }
                    else{
                        for(index = 0; index < MAX_TASKS; index++){
                        if(tcb[index].pid != 0){
                                                if(flashReq.processQueue[0] == tcb[index].pid){
                                                                        sprintf(stringBuffer, "Task: %s with PID: %u is waiting\r\n", tcb[index].label, tcb[index].pid);
                                                                        putsUart0(stringBuffer);
                                                                        wait++;
                                                                    }
                                                            }
                        }

                                        if(wait == 0){
                                            sprintf(stringBuffer, "No Task is waiting\r\n");
                                            putsUart0(stringBuffer);
                                        }
                                        sprintf(stringBuffer,"Resources available: %d\r\n",flashReq.count);
                                        putsUart0(stringBuffer);
                                        putsUart0("\r\nKeyPressed:\r\n");
                                        wait = 0;
                                        for(index = 0; index < MAX_TASKS; index++){
                                            if(tcb[index].pid != 0){
                                                if(keyPressed.processQueue[0] == tcb[index].pid){
                                                                        sprintf(stringBuffer, "Task: %s with PID: %u is waiting\r\n", tcb[index].label, tcb[index].pid);
                                                                        putsUart0(stringBuffer);
                                                                        wait++;
                                                                    }
                                                            }

                                        }
                                        if(wait == 0){
                                            sprintf(stringBuffer, "No Task is waiting\r\n");
                                            putsUart0(stringBuffer);
                                        }
                                        sprintf(stringBuffer,"Resources available: %d\r\n",keyPressed.count);
                                        putsUart0(stringBuffer);
                                        putsUart0("\r\nkeyReleased:\r\n");
                                        wait = 0;
                                        for(index = 0; index < MAX_TASKS; index++){
                                            if(tcb[index].pid != 0){
                                                if(keyReleased.processQueue[0] == tcb[index].pid){
                                                                        sprintf(stringBuffer, "Task: %s with PID: %u is waiting\r\n", tcb[index].label, tcb[index].pid);
                                                                        putsUart0(stringBuffer);
                                                                        wait++;
                                                                    }
                                            }

                                        }
                                        if(wait == 0){
                                            sprintf(stringBuffer, "No Task is waiting\r\n");
                                            putsUart0(stringBuffer);
                                        }
                                        sprintf(stringBuffer,"Resources available: %d\r\n",keyReleased.count);
                                        putsUart0(stringBuffer);
                                        wait = 0;

                    }
                }
                for(index = 0; index < 15; index++){
                new_input[index] = '\0';
                }
                putsUart0("\r\n");
                putsUart0("\r\nEnter the command\r\n");

            }
            else if(strcmp("kill", OpcodeOne) == 0){
                uint32_t processID;
                if(CheckOpcodeTwo(OpcodeTwo) == 0){
                    processID = atoi(&OpcodeTwo);
                                    if(processID == tcb[0].pid){
                                        sprintf(stringBuffer,"\r\nIdle Process cannot be destroyed");
                                        putsUart0(stringBuffer);
                                    }
                                    else{
                                        for(index = 1; index < MAX_TASKS; index++){
                                            if(processID == tcb[index].pid){
                                                destroyProcess(tcb[index].pid);
                                                sprintf(stringBuffer,"\r\nProcess Destroyed --> %s\r\n",tcb[index].label);
                                                putsUart0(stringBuffer);
                                                break;
                                            }
                                            else if(processID == tcb[index].restorePID && tcb[index].pid == 0){
                                                sprintf(stringBuffer,"\r\nProcess with PID --> %d does not exist. Please enter a valid PID\r\n",processID);
                                                putsUart0(stringBuffer);
                                            }
                                        }

                                    }
                }
                else
                {
                    sprintf(stringBuffer,"\r\nPlease enter a valid numeric PID\r\n");
                    putsUart0(stringBuffer);
                }

                for(index = 0; index < 15; index++){
                new_input[index] = '\0';
                }
                putsUart0("\r\n");
                putsUart0("Enter the command\r\n");
            }
            else if(strcmp("pidof", OpcodeOne) == 0){
                uint8_t found = 0;
                if(OpcodeTwo[0] == '\0'){
                    putsUart0("\r\nPlease enter a process name\r\n");
                }
                else{
                    for(index = 0; index < MAX_TASKS; index++){
                                        if(strcmp(OpcodeTwo,tcb[index].label) == 0){
                                            sprintf(stringBuffer, "\r\nThe PID of this task is %u", tcb[index].pid);
                                            putsUart0(stringBuffer);
                                            found++;
                                            break;
                                        }
                                    }
                                    if(found == 0){
                                        putsUart0("\r\nNo process with such a name exists");
                                    }
                }

                for(index = 0; index < 15; index++){
                new_input[index] = '\0';
                }
                putsUart0("\r\n");
                putsUart0("Enter the command\r\n");
            }
            else if(OpcodeOne[length-1] == '&'){
                bool createStatus;
                  OpcodeOne[length-1] = '\0';
                  for(index = 0; index < MAX_TASKS; index++){
                      if(strcmp(OpcodeOne,tcb[index].label)==0){
                          createStatus = createProcess(tcb[index].restorePID,tcb[index].priority,tcb[index].label);
                          if(createStatus == false){
                              putsUart0("\r\nProcess is already been created\r\n");
                          }
                          else{
                              sprintf(stringBuffer,"\r\nProcess Created --> %s\r\n",tcb[index].label);
                              putsUart0(stringBuffer);
                          }
                      }
                  }
                  for(index = 0; index < 15; index++){
                          new_input[index] = '\0';
                          }
                          putsUart0("\r\n");
                          putsUart0("Enter the command\r\n");
            }
            else if(strcmp("help", OpcodeOne) == 0){
                if(OpcodeTwo[0] != '\0')
                               {
                                   putsUart0("\r\nThis command does not accept any parameters/arguments.\r\n");
                                   putsUart0("\r\nPlease write help only.\r\n");

                               }
                else{
                    putsUart0("The following are the kernal functions that have been provided:\r\n");
                    putsUart0("\r\nps: Displays the Process name, PID, state and CPU time percentage of the process\r\n");
                    putsUart0("\r\nipcs: Displays the resources available and the processes waiting for it.\r\n");
                    putsUart0("\r\nkill: Destroys the process using its PID\r\n");
                    putsUart0("\r\nreboot: Restarts the controller\r\n");
                    putsUart0("\r\npidof: Returns the PID of the process by using its name\r\n");
                    putsUart0("\r\n<ProcessName>&: creates a process using its name\r\n");
                    putsUart0("\r\nhelp: Display the commands available and their description\r\n");
                }
                putsUart0("\r\n");
                putsUart0("Enter the command\r\n");
                for(index = 0; index < 15; index++){
                        new_input[index] = '\0';
                        }
            }
            else{
                if(OpcodeOne[0] =='\0'){
                    putsUart0("\r\n");
                    putsUart0("Enter the command\r\n");
                }
                else{
                    for(index = 0; index < 15; index++){
                    new_input[index] = '\0';
                    }
                    putsUart0("\r\n");
                    putsUart0("Wrong Command\r\n");
                    putsUart0("Enter again\r\n");
                }

            }
    }

    yield();
    }
}


//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    bool ok;

    // Initialize hardware
    initHw();
    rtosInit();

    // Power-up flash
    RED_LED = 1;
    waitMicrosecond(250000);
    RED_LED = 0;
    waitMicrosecond(250000);

    // Initialize semaphores
    init(&keyPressed, 0);
    init(&keyReleased, 1);
    init(&flashReq, 5);

    // Add required idle process
    ok =  createProcess(idle, 7, "idle"); // Task 0
    // Add other processes
    ok &= createProcess(flash4Hz, 0, "flash4Hz");   // Task 1
    ok &= createProcess(lengthyFn, 6, "lengthyFn"); // Task 2
    ok &= createProcess(oneshot, 3, "oneshot");    // Task 3
    ok &= createProcess(readKeys, 1, "readKeys");   // Task 4
    ok &= createProcess(debounce, 3, "debounce");   // Task 5
    ok &= createProcess(uncooperative, 5, "uncooperative"); // Task 6
    ok &= createProcess(CommandWindow, 4, "CommandWindow");   // Task 7

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
    yield(); sleep(0); wait(0); post(0);
}
