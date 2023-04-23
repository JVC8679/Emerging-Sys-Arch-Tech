/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Problem - Create a thermostat using the cc3220sf utilizing the GPIO, I2C, and UART.
 * TMP006 temperature sensor to read the room temperature (via I2C), an LED to indicate
 * the output to the thermostat where LED on = heat on (via GPIO), two buttons to increase
 * and decrease the set temperature (via GPIO interrupt), and the UART to simulate the
 * data being sent to the server.TMP006 temperature sensor to read the room temperature (via I2C),
 * an LED to indicate the output to the thermostat where LED on = heat on (via GPIO), two buttons
 * to increase and decrease the set temperature (via GPIO interrupt), and the UART
 * to simulate the data being sent to the server.
 */

/*
 * Solution - Created code that initializes the timer and uses it to drive specified actions.
 * Created code that uses interrupt to detect button presses.
 * Created code to initialize the I2C peripheral and use it to read the temperature sensor.
 * Created code to initialize the GPIO peripheral and use it.
 * Created code to initialize the UART peripheral and output specified data.
 * Implement a task scheduler functionality.
 */


/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>

/* Driver configuration */
#include "ti_drivers_config.h"
#include <ti/drivers/Timer.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>

// Driver Handles - Global variables
UART_Handle uart;
Timer_Handle timer0;
I2C_Handle i2c;

#define TRUE   1
#define FALSE  0
#define NUMBER_OF_TASKS 3
#define GLOBAL_PERIOD 100
#define DISPLAY(x) UART_write(uart, &output, x);

// Global period to be used initTimer()
int global_period = GLOBAL_PERIOD;

// UART Global Variables
char output[64];
int  bytesToSend;

// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
char *id;
} sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};

uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// Task Scheduler Variables
int seconds = 0;
int setpoint = 28;
int heat = 0;
int temperature = 0;
int firstButtonWasPressed = FALSE;  // It is initially false that the button was pressed
int secondButtonWasPressed = FALSE; // It is initially false that the button was pressed
volatile char timerFlag = TRUE;     // It is initially true to run once through the main function.


/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    firstButtonWasPressed = TRUE;  // It is true that the button was pressed
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    secondButtonWasPressed = TRUE;
}


void initUART(void)
{
    UART_Params uartParams;

    // Init the driver
    UART_init();

    // Configure the driver
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    // Open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);

    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}


// Make sure you call initUART() before calling this function.
void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;

    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "))

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);

    if (i2c == NULL)
    {
        DISPLAY(snprintf(output, 64, "Failed\n\r"))
        while (1);
    }
    DISPLAY(snprintf(output, 32, "Passed\n\r"))

    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses

    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    found = false;
    for (i=0; i<3; ++i)
    {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))
        if (I2C_transfer(i2c, &i2cTransaction))
        {
            DISPLAY(snprintf(output, 64, "Found\n\r"))
            found = true;
            break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"))
    }

    if(found)
    {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address:%x\n\r", sensors[i].id, i2cTransaction.slaveAddress))
    }

    else
    {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found,contact professor\n\r"))
    }
}
int16_t readTemp(void)
{

    int16_t temperature = 0;

    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
        /*
         * Extract degrees C from the received data;
         * see TMP sensor datasheet
         */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;
        /*
         * If the MSB is set '1', then we have a 2's complement
         * negative value which needs to be sign extended
         */
        if (rxBuffer[0] & 0x80)
        {
            temperature |= 0xF000;
        }
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor(%d)\n\r",i2cTransaction.status))
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"))
    }
    return temperature;
}



// A single task in the task list.
struct task_entry
{
    void (*f)();    // Function to call to perform the task
    int elapsed_time;   // Amount of time since last triggered
    int period;     // Period of the task in ms
    volatile char triggered; // Whether or not the task was triggered
};

// Forward declaration
void task_one();
void task_two();
void task_three();

// The task list
struct task_entry tasks[NUMBER_OF_TASKS] =
{
     {&task_one,    200,  200, TRUE},
     {&task_two,    500,  500, TRUE},
     {&task_three, 1000, 1000, TRUE}
};


void task_one()
{
    // Processing for task_one takes place
    // Every 200ms, check button presses

       if (firstButtonWasPressed == TRUE) // Button on one side raises setpoint (thermostat setting) by 1
       {
           setpoint += 1;                               // Increment thermostat
           firstButtonWasPressed = FALSE;              // Reset button to FALSE
       }

       if (secondButtonWasPressed == TRUE) // Button on the other side lowers setpoint (thermostat setting) by 1
       {
           setpoint -= 1;                               // Decrement thermostat
           secondButtonWasPressed = FALSE;             // Reset button to FALSE
       }


}
void task_two()
{
    // Processing for task_two takes place
    // Every 500ms, read temp
       temperature = readTemp();
       if (temperature >= setpoint) {
           GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
           heat = 0;
       }
       else {
           GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
           heat = 1;
       }


}
void task_three()
{
    // Processing for task_three takes place
    // Every 1000ms, print display
       DISPLAY(snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", temperature, setpoint, heat, seconds));
       seconds += 1;


}


/* Timercallback checks the elapsed time against the period of each task. If time has expired,
 * it sets the triggered flag of the task manager to true and global timer flag to true for
 * further processing in the main function.
 */
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
        int x = 0;
        // Walk through each task.
        for (x = 0; x < NUMBER_OF_TASKS; x++)
        {
            // Check if task's interval has expire
            if (tasks[x].elapsed_time >= tasks[x].period)
            {
                // Bing! This task's timer is up
                // Set it's flag, and the global flag
                tasks[x].triggered = TRUE;
                timerFlag = TRUE;
                // Reset the elapsed_time
                tasks[x].elapsed_time = 0;
            }
            else
            {
                tasks[x].elapsed_time += global_period;
            }
        }
}

void initTimer(void)
{
    Timer_Params params;
    // Init the driver
    Timer_init();

    // Configure the driver
    Timer_Params_init(&params);
    params.period = 100000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
    }

    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        DISPLAY(snprintf(output, 64, "Failed to start timer.\n\r"))
        while (1) {}
    }
}


/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);


    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_1);


    // Call driver init functions
    initUART(); // The UART must be initialized before calling initI2C()
    initI2C();
    initTimer();

    // Loop Forever
    while (TRUE)
    {

        // Wait for task intervals (periods) to elapse
        while (!timerFlag){}

        // Wait for timer period
        // Process the tasks for which the interval has expired
        int x = 0;
        for (x = 0; x < NUMBER_OF_TASKS; x++)
        {
            if (tasks[x].triggered)
            {
                // Executes the task function
                tasks[x].f();
                // reset
                tasks[x].triggered = FALSE;
                tasks[x].elapsed_time = 0;
            }

        }
        // Reset everything (e.g. flags) and go back to the beginning
        timerFlag = FALSE;  // Lower flag raised by timer

    }

}
