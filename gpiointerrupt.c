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
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART2.h>

/* Driver configuration */
#include "ti_drivers_config.h"

//Prototypes
void LEDOn();
void LEDOff();

//GLOBALS
char temperature = 0;
char setPoint = 0;
char heatOn = 0;
uint32_t secondsElapsed = 0;

char DEFAULT_SET_POINT = 20;

// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
}
sensors[3] = {
      { 0x48, 0x0000, "11X" },
      { 0x49, 0x0000, "116" },
      { 0x41, 0x0001, "006" }
};
uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// Driver Handles - Global variables
I2C_Handle i2c;
UART2_Handle uart;
Timer_Handle timer0;

// UART Global Variables
char output[64];
int bytesToSend;

//Timer
volatile unsigned char TimerFlag = 0;

/*========================
 * I2C
 *========================*/

// Make sure you call initUART() before calling this function.
void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;

    UART2_write(uart, "Initializing I2C Driver - ", sizeof("Initializing I2C Driver - ") - 1, NULL);  // -1 to exclude the null terminator

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL)
    {
        UART2_write(uart, "Found\n\r", sizeof("Found\n\r") - 1, NULL);
        while (1);
    }

    UART2_write(uart, "No\n\r", sizeof("No\n\r") - 1, NULL);

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
        i2cTransaction.targetAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;

        UART2_write(uart, "Is this ", sizeof("Is this ")-1, NULL);
        UART2_write(uart, sensors[i].id, sizeof(sensors[i].id)-1, NULL);
        UART2_write(uart, "?", sizeof("?")-1, NULL);

        if (I2C_transfer(i2c, &i2cTransaction))
        {
            UART2_write(uart, "Found\n\r", sizeof("Found\n\r")-1, NULL);
            found = true;
            break;
        }

        UART2_write(uart, "No\n\r", sizeof("No\n\r") - 1, NULL);
    }

    if(found)
    {
        UART2_write(uart, "Detected TMP ", sizeof("Detected TMP ")-1, NULL);
        UART2_write(uart, sensors[i].id, sizeof(sensors[i].id)-1, NULL);
        UART2_write(uart, " I2C address: ", sizeof(" I2C address: ")-1, NULL);

        snprintf(output, sizeof(output), "%x\n\r", i2cTransaction.targetAddress);
        UART2_write(uart, output, sizeof(output), NULL);
    }
    else
    {
        UART2_write(uart, "Temperature sensor not found, contact professor\n\r", sizeof("Temperature sensor not found, contact professor\n\r")-1, NULL);
    }
}

//Reads in the temperature and returns the value.
int16_t readTemp(void)
{
    int j;
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
        snprintf(output, 64, "Error reading temperature sensor (%d)\n\r", i2cTransaction.status);
        snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r");
    }
    return temperature;
}

/*========================
 * I2C
 *========================*/

//Start the UART
void initUART(void)
{
    UART2_Params uartParams;

    // Init the driver
    //UART2_init();

    // Configure the driver
    UART2_Params_init(&uartParams);
    uartParams.writeMode = UART2_Mode_BLOCKING;
    uartParams.readMode = UART2_Mode_BLOCKING;
    uartParams.readReturnMode = UART2_ReadReturnMode_FULL;
    uartParams.baudRate = 115200;

    // Open the driver
    uart = UART2_open(CONFIG_UART2_0, &uartParams);
    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}

//Init stuff...

//Callback
char counterButtons = 0; //Check every 200ms
char counterTemp = 0;   //Check every 500ms
char counterServer = 0;    //Check every second

char checkButtons = 0;
char checkTemp = 0;
char checkServerUpdate = 0;

//The timer callback, this will be called every time the timer ticks over
void TimerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    //Increment our counters
    counterButtons = counterButtons + 1;
    counterTemp = counterTemp + 1;
    counterServer = counterServer + 1;

    //Set flags based on what actions need taken
    if(counterButtons >= 2){ //Check every 200ms
        checkButtons = 1;
        counterButtons = 0;
    }

    if(counterTemp >= 5){ //Check every 500ms
        checkTemp = 1;
        counterTemp = 0;
    }

    if(counterServer >= 10){ //Check every 1000ms
        secondsElapsed = secondsElapsed + 1; //This always occurs at 1 second, so going to use it to increment the seconds.
        checkServerUpdate = 1;
        counterServer = 0;
    }

}

char changeTemp = 0;

//Button Callback, will call any time the button is pressed
void TempUp(uint_least8_t index){
    //Turn up the setPoint
    //UART2_write(uart, "TempUp called\n", sizeof("TempUp called\n"), NULL);
    changeTemp = 1;
}

void TempDown(uint_least8_t index){
    //Turn down the setPoint
    //UART2_write(uart, "TempDown called\n", sizeof("TempDown called\n"), NULL);
    changeTemp = -1;
}


//Initialize the timer
void initTimer(void)
{
    Timer_Params params;

    // Init the driver
    Timer_init();

    // Configure the driver
    Timer_Params_init(&params);
    params.period = 100000; //100 ms
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = TimerCallback;

    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
    }

    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }
}

//Methods
//LED Methods
void LEDOn(){
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
}

void LEDOff(){
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
}

enum SM_ThermostatStates{
    IDLE,
    CHECK_BUTTONS,
    CHECK_TEMP,
    UPDATE_SERVER
} thermostatState;

//Resets the state as well as all check flags and counters
void Reset(){
    thermostatState = IDLE;
    counterButtons = 0;
    counterTemp = 0;
    counterServer = 0;
    changeTemp = 0;
    checkButtons = 0;
    checkTemp = 0;
    checkServerUpdate = 0;
    setPoint = DEFAULT_SET_POINT;
}

//Adjusts the setPoint and then resets the changeTemp value
void AdjustSetPoint(){
    setPoint = setPoint + changeTemp;

    //Limit setPoint between 0 - 99
    if(setPoint <= 0){
        setPoint = 0;
    }
    else if (setPoint >= 99){
        setPoint = 99;
    }

    changeTemp = 0;
}

//Reads the temp, and compares to determine if HVAC should be enabled
void CheckTemp(){
    temperature = readTemp();

    //Limit temp between 0 - 99
    if(temperature <= 0){
        temperature = 0;
    }
    else if (temperature >= 99){
        temperature = 99;
    }

    //Turn on LED if less than the set point
    //Turn off LED if greater than the set point
    //Set the heatOn variable accordingly
    if(temperature > setPoint){
        heatOn = 0;
        LEDOff();
    }
    else if (temperature < setPoint){
        heatOn = 1;
        LEDOn();
    }

    //No specification for exact, so going to let it stay whatever it was until we surpass.
    //This is usually handled via a threshold
}

//Updates the display with the current information
void UpdateDisplay(){
    snprintf(output, sizeof(output), "<%02d, %02d, %d, %04d>\n\r", temperature, setPoint, heatOn, secondsElapsed);
    UART2_write(uart, output, sizeof(output), NULL);
}

//Updates the server with the Thermostat data
void UpdateServer(){
    //TODO: IMPLEMENT SERVER COMMUNICATION
    return;
}



//State Machine
void SmartThermostat(){

    //checkButtons
    //checkServerUpdate

    switch(thermostatState){

    //This is the check if flags are set state
    case(IDLE):
        if(checkButtons){ thermostatState = CHECK_BUTTONS; } //Every 200ms
        if(checkTemp){ thermostatState = CHECK_TEMP; } //Every 500ms
        if(checkServerUpdate){ thermostatState = UPDATE_SERVER; }; //Every Second
        break;

    //Every 200ms - Read if there was input on the buttons. Adjust the setPoint accordingly
    case(CHECK_BUTTONS):
        if(changeTemp != 0){
            AdjustSetPoint();
        }
        checkButtons = 0;
        thermostatState = IDLE;
        break;

    //Every 500ms - Read the temp, and check if we need to turn on the heat
    case(CHECK_TEMP):
        CheckTemp();
        checkTemp = 0;
        thermostatState = IDLE;
        break;

    //Every Second - Update Display and Update the Server
    case(UPDATE_SERVER):
        UpdateDisplay();
        UpdateServer();
        checkServerUpdate = 0;
        thermostatState = IDLE; //Return back to idle
        break;

    default:
        //Something happened. Reset.
        Reset();
        break;
    }

}



//=============================================================
//BEGIN PROCESSING
//=============================================================

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();
    initUART();
    initI2C();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW); //LED

    //Buttons
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING); //left - if USB up
    GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING); //right - if USB up

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    /* Install Button callback, adding to both buttons */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, TempDown);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_1, TempUp);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_1);

    //Initialize the timer
    initTimer();

    //Set the state machine
    thermostatState = IDLE;

    //Set the default setPoint
    setPoint = DEFAULT_SET_POINT;

    while(1){

        SmartThermostat();

    }
}
