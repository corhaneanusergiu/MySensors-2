/*
* Copyright (C) 
* Version 1 - 2015 Dirk H. R. 
* Version 2 - 2016 Rik J; improved handling of contact bounce
* 
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*
* DESCRIPTION
* This Program will implement four child sensors, which present one Push Button each.
*
* Frequency is 1 MHz to ensure operation at low Voltages
*
*/


#include <MySensor.h>
#include <SPI.h>
#include "utility/PinChangeInt.h" //include PinChange lib from MySensors utilities
#include <readVcc.h>


#define SKETCH_NAME "Wall Remote"
#define SKETCH_MAJOR_VER "1"
#define SKETCH_MINOR_VER "0"

#define NodeID 2

#define BotLeft_CHILD_ID 0			//Child ID for Bottom Left 
#define TopCornerLeft_CHILD_ID 1	//Child ID for Top Left (Corner Switch)

#define BotRight_CHILD_ID 2			//Child ID for Bottom Left
#define TopCornerRight_CHILD_ID 3	//Child ID for Top Left (Corner Switch)

#define BotLeft_PIN A2 // pc2 A1 //21			//Digital Input for Bottom Left Pin			PCInt 11
#define TopLeft_PIN A1 // pc1 A2//19			//Digital Input for Top Left (Corner Switch) Pin	PCInt 9

#define BotRight_PIN 4 //pd4 3			//Digital Input for Bottom Right Pin			
#define TopRight_PIN 3 //pd3 4			//Digital Input for Top Right (Corner Switch) Pin	

#define BotLeft_PinState BIT_TST(PINC, 2, 1)
#define TopLeft_PinState BIT_TST(PINC, 1, 1)

#define BotRight_PinState BIT_TST(PIND, 4, 1)
#define TopRight_PinState BIT_TST(PIND, 3, 1)

#define Btn_BotLeft_ID 0
#define Btn_TopLeft_ID 1

#define Btn_BotRight_ID 2
#define Btn_TopRight_ID 3

#define DEBOUNCE_TIME   10


#define BIT_SET(REG, bit)                    ( REG |= (1UL << (bit) ) )

#define BIT_CLR(REG, bit)                    ( REG &= ~(1UL << (bit) ) )

#define BIT_TST(REG, bit, val)              ( ( (REG & (1UL << (bit) ) ) == ( (val) << (bit) ) ) )

int oldBatteryPcnt = 0;

// Define voltages

int MIN_V = 2400; // empty voltage (0%)
int MAX_V = 3200; // full voltage (100%)

MySensor sensor_node;

// Change V_TRIPPED if you don't use S_DOOR in presentation below
MyMessage msg_BotLeft(BotLeft_CHILD_ID, V_TRIPPED);
MyMessage msg_TopLeft(TopCornerLeft_CHILD_ID, V_TRIPPED);

MyMessage msg_BotRight(BotRight_CHILD_ID, V_TRIPPED);
MyMessage msg_TopRight(TopCornerRight_CHILD_ID, V_TRIPPED);

#define BTN_BUFFER_SIZE 10
//The BtnVal and the Btn_xx_ID are used to store the current state of the four buttons into a single uint8
uint8_t BtnVal = 0;	//idle/start value is 255 because Buttons get pulled up in idle state
uint8_t BtnVal_last = 0;
uint8_t BtnCurrent = 0;
uint8_t BtnPending = false;
uint8_t BtnBuffer[BTN_BUFFER_SIZE], BtnReadIdx, BtnProcessIdx;

unsigned long debounceTimer;

void setup()
{

  Serial.begin(9600);
  //Serial.println("starting");
  sensor_node.begin(NULL, NodeID);

  // Setup the buttons
  pinMode(BotLeft_PIN, INPUT);
  pinMode(TopLeft_PIN, INPUT);

  pinMode(BotRight_PIN, INPUT);
  pinMode(TopRight_PIN, INPUT);

  // Activate internal pull-ups
  digitalWrite(BotLeft_PIN, HIGH);
  digitalWrite(TopLeft_PIN, HIGH);

  digitalWrite(BotRight_PIN, HIGH);
  digitalWrite(TopRight_PIN, HIGH);

  //Use internal 1.1V Reference (for Battery Measurement)
  analogReference(INTERNAL);

  // Send the sketch version information to the gateway and Controller
  sensor_node.sendSketchInfo(SKETCH_NAME, SKETCH_MAJOR_VER"."SKETCH_MINOR_VER);

  // Register Wall Remote buttons (they will be created as child devices)
  // I decided to take the S_DOOR parameter as i inteded the WallRemote to switch lights,
  // however you can change if you think this is more apropriate, hovwever
  // If S_DOOR is not used, remember to update variable type you send in. See "msg" above.
  sensor_node.present(BotLeft_CHILD_ID, S_DOOR);
  sensor_node.present(TopCornerLeft_CHILD_ID, S_DOOR);

  sensor_node.present(BotRight_CHILD_ID, S_DOOR);
  sensor_node.present(TopCornerRight_CHILD_ID, S_DOOR);

  /*Setup PinChange Interrupt. Unfortunately you must ensure that the "MyGateway.cpp" will not be compiled for this,
  i.e. look into your Arduino libraries\MySensors folder and
  ---> RENAME THE "MyGateway.cpp" to e.g. "MyGateway.cpp.nolib" <---.
  Of cause you must undo the renameing when you want to compile a MySensors Gateway!
  The reason for this renaming is that the compilter of Arduino compiles also the MyGateway where the PinChange interrupt
  Vector is also defined, which will lead to an error because it is only allowed to have it one time in the program.
  */
  PCattachInterrupt(BotLeft_PIN, BotLeft_ISR, CHANGE);
  PCattachInterrupt(TopLeft_PIN, TopLeft_ISR, CHANGE);

  PCattachInterrupt(BotRight_PIN, BotRight_ISR, CHANGE);
  PCattachInterrupt(TopRight_PIN, TopRight_ISR, CHANGE);
  interrupts();
  //Serial.println("init complete");
}

void WallRemote_Sleep() {
  //Custom Function that puts ATMega and NRF24L01 into sleep mode.
  //This was necessary because PinChange Interrupts have been used.
  sensor_node.RF24::powerDown();	//Power Down NRF24
  Serial.flush();	// although there should be nothing in the Serial cue, Let serial prints finish (debug, log etc)
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); //power everything down, wake up only by (PinChange) interrupts
}

void inline BtnPressHandler(uint8_t Button, uint8_t ValPos) {
  //Handler that just saves state of pressed button into BtnVal variable

  uint8_t inpValue = !digitalRead(Button);  
  bitWrite(BtnCurrent, ValPos, inpValue);

  if (bitRead(BtnPending, ValPos))  // if a value is already pending that one will be processed only
    return;   // so in case of a bouncing contact the first edge is used (which is the correct value)
 
  bitWrite(BtnVal, ValPos, inpValue); // save current state 
  bitSet(BtnPending, ValPos); // set pending bit
}

//The six following ISRs are used to Wakeup the device and to call the Button Handler routine
void BotLeft_ISR() {
  BtnPressHandler(BotLeft_PIN, Btn_BotLeft_ID);
}

void TopLeft_ISR() {
  BtnPressHandler(TopLeft_PIN, Btn_TopLeft_ID);
}

void BotRight_ISR() {
  BtnPressHandler(BotRight_PIN, Btn_BotRight_ID);
}

void TopRight_ISR() {
  BtnPressHandler(TopRight_PIN, Btn_TopRight_ID);
}

void BtnSender(MyMessage Btn_Msg, uint8_t Btn_ID) 
{ 
  if (bitRead(BtnPending, Btn_ID))
  {/*
    Serial.print("B ");
    Serial.print(Btn_ID);
    Serial.print(" to: ");
    Serial.println(bitRead(BtnVal, Btn_ID));*/
    if (bitRead(BtnVal, Btn_ID))
      sensor_node.send(Btn_Msg.set(HIGH));
    else
      sensor_node.send(Btn_Msg.set(LOW));
    bitWrite(BtnVal_last, Btn_ID, bitRead(BtnVal, Btn_ID)); // save the current processed state
    bitClear(BtnPending, Btn_ID);  // this request was handled
  }
}

//loop only checks the ButtonState Variable and transmits changes if necessary
void loop()
{
// Measure battery
  float batteryV = readVcc();
  int batteryPcnt = (((batteryV - MIN_V) / (MAX_V - MIN_V)) * 100 );
  if (batteryPcnt > 100) {
    batteryPcnt = 100;
  }

 if (batteryPcnt != oldBatteryPcnt) {
    sensor_node.sendBatteryLevel(batteryPcnt); // Send battery percentage
    oldBatteryPcnt = batteryPcnt;
  }

 // Handle sending the buttons
  BtnSender(msg_BotLeft, Btn_BotLeft_ID);
  BtnSender(msg_TopLeft, Btn_TopLeft_ID);
  BtnSender(msg_BotRight, Btn_BotRight_ID);
  BtnSender(msg_TopRight, Btn_TopRight_ID);
 
  // read current state of inputs
  bitWrite(BtnCurrent, Btn_BotLeft_ID, !digitalRead(BotLeft_PIN));
  bitWrite(BtnCurrent, Btn_TopLeft_ID, !digitalRead(TopLeft_PIN));
  bitWrite(BtnCurrent, Btn_BotRight_ID, !digitalRead(BotRight_PIN));
  bitWrite(BtnCurrent, Btn_TopRight_ID, !digitalRead(TopRight_PIN));
 
  
  if (!BtnPending) // did none of the buttons change after we've handled that button?
  {
  // now check if buttons changed state compared to what we've send
    if (BtnCurrent ^ BtnVal_last)
    {
      BtnPending = (BtnCurrent ^ BtnVal_last);
      BtnVal = BtnCurrent;
      /*
      Serial.print("P ");
      Serial.print(BtnPending);
      Serial.print(", C ");
      Serial.print(BtnCurrent);
      Serial.print(", V ");
      Serial.println(BtnVal_last);*/
    }
  }
  
  
  
  if (!BtnPending) // all buttons processed?
  {
    WallRemote_Sleep();// Go back to sleep (custom function, see above)
  }
  else  // do loop once again and process the buttons
  {
  /*
    Serial.print("Pending: ");
    Serial.println(BtnPending);*/
  }    
}
