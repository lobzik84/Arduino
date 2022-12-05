#include <EEPROM.h>
#include <Ethernet.h>
#include <Time.h>
#include <DS1302.h>

//recharge modes
#define MANUAL_RECHARGE 0
#define DELAYED_ON_DEMAND 1
#define TIMER_ONLY 2
#define TIMER_AND_DEMAND 3

//valve states
#define ERROR_SWITCH_FAIL_STATE -3
#define ERROR_MOTOR_FAIL_STATE -2
#define NOT_INITIALIZED_STATE -1
#define READY_STATE 0
#define WAIT_FOR_RECHARGE_STATE 1
#define RECHARGE_STATE 2
#define STANDBY_STATE 3

#define VALVE_CYCLES 6

//constant time intervals
#define FULL_MOTOR_CYCLE_MAX_TIME 50 //sec
#define FULL_MOTOR_CYCLE_MIN_TIME 10 //sec
#define MAX_MOTOR_TIME 20
#define MIN_MOTOR_SW_TIME 100
#define MIN_COUNTER_TIME 500
#define POWER_TIMEOUT 6 //sec

#define MILLIS_IN_MIN 60000 //milliseconds in minute. must be 60000, useful to set 1000 during debug :)

// pins definition
#define MOTOR_POS_MAIN_SWITCH_PIN 2
#define MOTOR_POS_MAIN_SWITCH_INT 0 //pin2
#define COUNTER_PIN 3
#define COUNTER_INT 1 //pin 3
#define RTC_CE_PIN 5
#define RTC_IO_PIN 6
#define RTC_SCLK_PIN 7
#define WATCHDOG_PIN 8
#define POWER_ON_PIN 9
#define MOTOR_POS_AUX_SWITCH_PIN 21
#define MOTOR_DRIVE_PIN 23
#define CHEM_PUMP_PIN 25
#define AIR_PUMP_PIN 27
#define IN_VALVE_PIN 29
#define OUT_VALVE_PIN 31

// control values to check memory is ok
#define RAM_CONTROL_VALUE 0xEB
#define EEPROM_CONTROL_VALUE 0xEB

//DS1302 RAM data map
#define RAM_CONTROL_VALUE_ADDR 0x00  //1 byte
#define RAM_COUNTER_START_ADDR 0x01  //4 bytes
#define RAM_CNTBASE_START_ADDR 0x05  //4 bytes

//EEPROM data map
#define EEPROM_MAC_START_ADDR 0x00   //6 bytes
#define EEPROM_IP_START_ADDR 0x06    //4 bytes
#define EEPROM_MASK_START_ADDR 0x0a  //4 bytes
#define EEPROM_GW_START_ADDR 0x0e    //4 bytes
#define EEPROM_CONFIG_BYTE_ADDR 0x12 //1 byte
#define EEPROM_STATE_START_ADDR 0x20 //2 bytes
#define EEPROM_DATE_START_ADDR 0x22  //4 bytes
#define EEPROM_RECHARGE_MODE_ADDR 0x30
#define EEPROM_RECHARGE_TIME_H_ADDR 0x31
#define EEPROM_RECHARGE_TIME_M_ADDR 0x32
#define EEPROM_CHARGE_VOLUME_ADDR 0x33
#define EEPROM_RECHARGE_DAY_ADDR 0x34
#define EEPROM_BACKWASH_TIME_ADDR 0x35
#define EEPROM_BRINE_TIME_ADDR 0x36
#define EEPROM_RINSE_TIME_ADDR 0x37
#define EEPROM_REFILL_TIME_ADDR 0x38
#define EEPROM_CHEM_PUMP_PULSE_ADDR 0x39
#define EEPROM_COUNTS_PER_CHEM_PULSE_ADDR 0x3A
#define EEPROM_AIR_PUMP_WORK_TIME_ADDR 0x3B
#define EEPROM_VALVE_CONFIG_ADDR 0x3C
#define EEPROM_LITERS_PER_COUNT_ADDR 0x3D
#define EEPROM_CONTROL_VALUE_ADDR 0xEE

#define SERVER_PORT 23 //telnet server port
#define TEXT_BUFFER_SIZE 39 //length of longest command string plus two spaces for CR + LF

// this is default settings for first run on non-inited EEPROM; 
byte RECHARGE_MODE = DELAYED_ON_DEMAND;
byte RECHARGE_TIME_H = 2;
byte RECHARGE_TIME_M = 0;
byte CHARGE_VOLUME = 20; //thousands of pulses for recharge (m3)
byte RECHARGE_DAY = 2; //period
byte BACKWASH_TIME = 5; //minutes
byte BRINE_TIME = 5;
byte RINSE_TIME = 5;
byte REFILL_TIME = 5;
byte CHEM_PUMP_PULSE = 30; //*10 = 300 ms
boolean CHEM_DURING_BRINE = true; //CHEM pump on during brining
byte COUNTS_PER_CHEM_PULSE = 6;//counter pulses to make one chem pump pulse 
byte AIR_PUMP_WORK_TIME = 4; //sec
byte LITERS_PER_COUNT = 5; //decimal parts of liter in one count
boolean CLOSE_OUT_VALVE_TILL_RECHARGE = false;
boolean CLOSE_IN_VALVE_TILL_STANDBY = false;
boolean CLOSE_IN_VALVE_TILL_ERROR = false;
boolean WATHDOG_ENABLED = true;
byte MAC[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
byte IP[4] = {192, 168, 0, 10};
byte MASK[4] = {255, 255, 255, 0};
byte GW[4] = {192, 168, 0, 1};
boolean DHCP = true;

//valve variables;
int state = 0;
byte cycle = 0;
byte prevCycle = 0;
boolean wasDefaultBeforeInit = false;
unsigned long cycleStartTime = 0;
unsigned long motorOnTime = 0;
unsigned long counter = 0;
unsigned long counterTime = 0;
unsigned long lastMainSwitchTime = 0;
unsigned long lastAuxSwitchTime = 0;
long counterBase = 0;
unsigned long chemPumpOnTime = 0;
boolean mainSwitchTriggered = false;
boolean airPumpManual = false;
boolean chemPumpManual = false;
unsigned long lastRecharge = 0;
unsigned long lastPowerXfer = 0;
boolean lastPowerState = false;
boolean lastCounterState = false;
boolean counterState = false;
boolean lastMainSwitchState = false;
boolean mainSwitchState = false;
boolean lastAuxSwitchState = false;
//system global devices
DS1302 rtc(RTC_CE_PIN, RTC_IO_PIN, RTC_SCLK_PIN);
EthernetServer server(SERVER_PORT); // Telnet listens on port 23
EthernetClient client = 0; // Client needs to have global scope so it can be called

//buffers
String textBuffer = "";  
byte parsedBytesBuffer[12];

union mytype
{
  long l;
  byte b[4];
} tmp; //for long to byte[4] conversion and back

void setup() 
{                
  pinMode(WATCHDOG_PIN, INPUT);//disable watchdog
    
  pinMode(MOTOR_DRIVE_PIN, OUTPUT);  
  pinMode(CHEM_PUMP_PIN, OUTPUT); 
  pinMode(AIR_PUMP_PIN, OUTPUT); 
  pinMode(IN_VALVE_PIN, OUTPUT); 
  pinMode(OUT_VALVE_PIN, OUTPUT); 
 
  pinMode(COUNTER_PIN, INPUT_PULLUP);    
  pinMode(MOTOR_POS_MAIN_SWITCH_PIN, INPUT_PULLUP);    
  pinMode(MOTOR_POS_AUX_SWITCH_PIN, INPUT_PULLUP);    
  pinMode(POWER_ON_PIN, INPUT_PULLUP);    
  
  Serial.begin(9600);
  while (!Serial) {;} // wait for serial port to connect. Needed for Leonardo
  if (checkEEPROM())
  {
    readValveSettings();    
    readNetworkSettings();
  }
  
  initNetwork();

  checkRTC();
  setSyncProvider(getRTCtime);   // the function to get the time from the RTC
  if(timeStatus()!= timeSet) responseln ("ERROR: Unable to sync with the RTC");
  
  if (WATHDOG_ENABLED) pinMode(WATCHDOG_PIN, OUTPUT);//enable watchdog
  resetWatchDog();
  
  initCycles();
  
  lastPowerState = digitalRead(POWER_ON_PIN);
  lastCounterState = digitalRead(COUNTER_PIN);
  counterState = lastCounterState;
  lastMainSwitchState = digitalRead(MOTOR_POS_MAIN_SWITCH_PIN);
  mainSwitchState = lastMainSwitchState;
  lastAuxSwitchState = digitalRead(MOTOR_POS_AUX_SWITCH_PIN);
  lastPowerXfer = millis();
}

void loop() 
{
  resetWatchDog();
  checkPumps();
  checkPins();
  checkMotor();
  checkCycle();

  
  //if (server.available()) client = server.available();
  
  //if (client.connected() && client.available()) readCommand(&client);
  
  for (int sock=0; sock < 4; sock++) 
  {
    EthernetClient cli = server.available_(sock);
    if (cli) {
        if (cli.connected()) readCommand(&cli);
    }  
  }
  
  if (Serial.available()) readCommand(&Serial);
  
  if (cycle != prevCycle) saveState();
  prevCycle = cycle;
  boolean powerState = digitalRead(POWER_ON_PIN);
  if (lastPowerState != powerState) 
  {
    lastPowerXfer = millis();
    lastPowerState = powerState;
  }

  if (millis() - lastPowerXfer > POWER_TIMEOUT * 1000)
  {
    if (state != STANDBY_STATE && state >= 0 && !powerState) 
    {
      if (cycle > 0) cycleStartTime = millis() - cycleStartTime;//store cycle elapsed time
      state = STANDBY_STATE;
      responseln("main power: OFF");
      if (CLOSE_IN_VALVE_TILL_STANDBY) closeWaterSupply();
    }
    
    if (state == STANDBY_STATE && powerState) 
    {
      responseln("main power: ON");
      if (cycle == 0) 
      {  
        state = READY_STATE;
      }
      else 
      {
        cycleStartTime = millis() - cycleStartTime; //restoring cycle start time
        state = RECHARGE_STATE;
        responseln("WARN: Restoring recharge!");
      }
      if (CLOSE_IN_VALVE_TILL_STANDBY) openWaterSupply();
    }
  

  }
  
  if (state == READY_STATE)
  {
    switch (RECHARGE_MODE)
    {
      case (DELAYED_ON_DEMAND):
        if (counter > 1000 * CHARGE_VOLUME) state = WAIT_FOR_RECHARGE_STATE;
      break;
      
      case (TIMER_ONLY):
        if (((now() - lastRecharge) / SECS_PER_DAY) >= RECHARGE_DAY) state = WAIT_FOR_RECHARGE_STATE;
      break;
      
      case (TIMER_AND_DEMAND):
        if (counter > 1000 * CHARGE_VOLUME || ((now() - lastRecharge) / SECS_PER_DAY) >= RECHARGE_DAY) state = WAIT_FOR_RECHARGE_STATE;
      break;
      
      default:
        //manual recharge
      break;
    }
  }
  if (state == WAIT_FOR_RECHARGE_STATE && hour() == RECHARGE_TIME_H && minute() == RECHARGE_TIME_M) startRecharge();
  
}

void initNetwork()
{
  if (DHCP) //if DHCP is enabled
  {
     // start the Ethernet connection using just MAC, with DHCP:
    if (Ethernet.begin(MAC) == 0)  
      Serial.println("Failed to configure Ethernet using DHCP");
    else
    {
      // print your local IP address:
      Serial.print("My IP address: ");
      for (byte thisByte = 0; thisByte < 4; thisByte++) 
      {
        // print the value of each byte of the IP address:
        Serial.print(Ethernet.localIP()[thisByte], DEC);
        Serial.print("."); 
      }
      Serial.println(); 
    }
  }
  else //if DHCP disabled
  {
    //configuring static IP and start the Ethernet connection
    Ethernet.begin(MAC, IP, GW, MASK);     
  }
  server.begin();
}



void initCycles()
{
  //check if there was no recharge before shutdown
  cycle = EEPROM.read(EEPROM_STATE_START_ADDR);
  if (cycle == 0xFF) cycle = 0; //new eeprom
  if (cycle > 0 && cycle < VALVE_CYCLES) state = RECHARGE_STATE;
  
  for (int i = 0; i < 4; i++) tmp.b[i] = EEPROM.read(EEPROM_DATE_START_ADDR + i);
  lastRecharge = tmp.l; // get long value of last recharge date (in secs)
  
  for (int i = 0; i < 4; i++) tmp.b[i] = rtc.peek(RAM_COUNTER_START_ADDR + i);
  counter = tmp.l; //get long value of counter
  
  for (int i = 0; i < 4; i++) tmp.b[i] = rtc.peek(RAM_CNTBASE_START_ADDR + i);
  counterBase = tmp.l; //get long value of counterBase
}

void startRecharge()
{
  if (cycle == 0 && (state == READY_STATE || state == WAIT_FOR_RECHARGE_STATE))
  {
    responseln("INFO: **** Starting recharge ****");
    if (CLOSE_OUT_VALVE_TILL_RECHARGE) closeWaterSupply();
    state = RECHARGE_STATE;
    lastRecharge = now();

    tmp.l = lastRecharge;
    for (int i = 0; i < 4; i++) EEPROM.write(EEPROM_DATE_START_ADDR + i, tmp.b[i]);
    nextCycle();
  }
  else
  {
    response("WARN: Could not start recharge because ");
    if (cycle > 0) responseln("already recharging...");
    if (state < 0) responseln("valve is in error state...");
    if (state == STANDBY_STATE) responseln("stand-by mode");
  }
}

boolean checkRTC()
{
  byte controlByte = rtc.peek(RAM_CONTROL_VALUE_ADDR);
  if (controlByte != RAM_CONTROL_VALUE)
  {
    responseln ("ERROR: RTC Control byte check failed! New or unitialized RTC detected. Initializing... Please set Time!");
    rtc.halt(false);
    rtc.writeProtect(false);
    for (int i = 0; i < 31; i++) rtc.poke(i, 0x00);
    rtc.poke(RAM_CONTROL_VALUE_ADDR, RAM_CONTROL_VALUE);
    return false;
  }
  else return true;
}

boolean checkEEPROM()
{
  byte controlByte = EEPROM.read(EEPROM_CONTROL_VALUE_ADDR);
  if (controlByte != EEPROM_CONTROL_VALUE)
  {
    Serial.println("ERROR: EEPROM Control byte check failed! New EEPROM detected. Default values loaded... Check your settings!");
    writeValveSettings();
    writeNetworkSettings();
    EEPROM.write(EEPROM_CONTROL_VALUE_ADDR, EEPROM_CONTROL_VALUE);
    return false;
  }
  else return true;
}

void checkPumps()
{
  if (cycle == 0 && chemPumpOnTime > 0 && !chemPumpManual)
  {
    if (digitalRead(CHEM_PUMP_PIN) && ((millis() - chemPumpOnTime) > CHEM_PUMP_PULSE * 10 ))
    {
      digitalWrite(CHEM_PUMP_PIN, LOW);
      chemPumpOnTime = 0;
    }
  }
  if (digitalRead(AIR_PUMP_PIN) && ((millis() - counterTime) > (AIR_PUMP_WORK_TIME * 1000)) && !airPumpManual) digitalWrite(AIR_PUMP_PIN, LOW);
}



void checkPins()
{
  if (lastCounterState != digitalRead(COUNTER_PIN))
  {
    counterTime = millis(); 
    lastCounterState = digitalRead(COUNTER_PIN);
  }
  if (lastCounterState != counterState && millis() - counterTime > MIN_COUNTER_TIME)
  {
    counterTime = millis(); 
    counterState = lastCounterState;
    responseln("DEBUG: counter!");
    counter++;
    union mytype
    {
      long l;
      byte b[4];
    } tmpC; 
    tmpC.l = counter;
    for (int i = 0; i < 4; i++) rtc.poke(RAM_COUNTER_START_ADDR + i, tmpC.b[i]);
    
    if (COUNTS_PER_CHEM_PULSE > 0 & (counter % COUNTS_PER_CHEM_PULSE == 0) && cycle == 0 && state >= 0)
    {
      chemPumpOnTime = millis();
      digitalWrite(CHEM_PUMP_PIN, HIGH);
    }
    if (AIR_PUMP_WORK_TIME > 0 && cycle == 0 && state >= 0)
     digitalWrite(AIR_PUMP_PIN, HIGH);
  }
    
  if (lastMainSwitchState != digitalRead(MOTOR_POS_MAIN_SWITCH_PIN))
  {
    lastMainSwitchTime = millis();
    lastMainSwitchState = digitalRead(MOTOR_POS_MAIN_SWITCH_PIN);
  }
  if (lastMainSwitchState != mainSwitchState && millis() - lastMainSwitchTime > MIN_MOTOR_SW_TIME)
  {
    mainSwitchState = lastMainSwitchState;
    lastMainSwitchTime = millis();
    cycleStartTime = millis();
    responseln ("DEBUG: Main sw!");
    cycle++;
    if (state < 0) return; //if error, do nothing
    digitalWrite(MOTOR_DRIVE_PIN, LOW);
    mainSwitchTriggered = true; 
  }
}


void checkMotor()
{
  if (lastAuxSwitchState != digitalRead(MOTOR_POS_AUX_SWITCH_PIN))
  {
    lastAuxSwitchState = digitalRead(MOTOR_POS_AUX_SWITCH_PIN);
    lastAuxSwitchTime = millis(); 
    responseln ("DEBUG: Aux sw!");
  }
  
  if (state == NOT_INITIALIZED_STATE)
  {
    if (digitalRead(MOTOR_DRIVE_PIN))
    {
      //if motor is running, check work time
      int workTime = (millis() - motorOnTime) / 1000;
      if (workTime > FULL_MOTOR_CYCLE_MAX_TIME)
      {
        digitalWrite(MOTOR_DRIVE_PIN, LOW);
        if (cycle < VALVE_CYCLES)
        {
          state = ERROR_MOTOR_FAIL_STATE;
          responseln("ERROR: could not initialize valve - motor timeout error!");
        }
        else
        {
          state = ERROR_SWITCH_FAIL_STATE;// looks like bug - if reinit done after non-finished cycle, error will pop up
          responseln("ERROR: could not initialize valve - aux switch not working!");
        }
      }
      else if (workTime > FULL_MOTOR_CYCLE_MIN_TIME) //if motor runs for more than 10 secs, check for aux switch
      {
        if(lastAuxSwitchState && millis() - lastAuxSwitchTime > MIN_MOTOR_SW_TIME)
        {
          digitalWrite(MOTOR_DRIVE_PIN, LOW);
          state = READY_STATE;
          responseln("INFO: Motor re-init done; motor work time " + String(millis() - motorOnTime) + " time from aux sw " + String(millis() - lastAuxSwitchTime));
          if (wasDefaultBeforeInit && cycle != VALVE_CYCLES) 
            responseln("ERROR: WARNING!! Incorrect cycles count! Got " + String(cycle) + " cycles instead of " + String (VALVE_CYCLES) + "! Check main switch.");
          cycle = 0;
        }
      }
      else if (workTime > (FULL_MOTOR_CYCLE_MIN_TIME - 1) && lastAuxSwitchState && millis() - lastAuxSwitchTime > MIN_MOTOR_SW_TIME) //if motor runs for more than 10 secs, but aux switch is off, then it's a fault
      {
        digitalWrite(MOTOR_DRIVE_PIN, LOW);
        if (cycle == 0)
        {
          state = ERROR_MOTOR_FAIL_STATE;
          responseln("ERROR: could not initialize valve - motor timeout error!");
        }
        else
        {
          state = ERROR_SWITCH_FAIL_STATE;
          responseln("ERROR: could not initialize valve - aux switch not working!");
        }
      }
      
    }
    else
    {
      motorOnTime = millis(); //if not initialized, but motor is off - enable motor
      digitalWrite(MOTOR_DRIVE_PIN, HIGH);
      wasDefaultBeforeInit = lastAuxSwitchState;
      cycle = 0;
    } 
  }
  else if (digitalRead(MOTOR_DRIVE_PIN) && ((millis() - motorOnTime) > MAX_MOTOR_TIME * 1000))
  {
    digitalWrite(MOTOR_DRIVE_PIN, LOW);
    state = ERROR_MOTOR_FAIL_STATE;
    responseln("ERROR: motor timeout error!");
  }
  if (cycle == 0 && (state == READY_STATE || state == WAIT_FOR_RECHARGE_STATE) && !digitalRead(MOTOR_DRIVE_PIN) && !lastAuxSwitchState && millis() - lastAuxSwitchTime > MIN_MOTOR_SW_TIME)
  {
    responseln("WARN: Cycle is 0, but valve is not in default position! Re-init motor...");
    initMotor();
  }
}

void initMotor()
{
  state = NOT_INITIALIZED_STATE;
}

void checkCycle()
{
  if (state == READY_STATE && cycle != 0) cycle = 0; //in ready state cycle must be zero!
  if (cycle > 0 & state == RECHARGE_STATE & !digitalRead(MOTOR_DRIVE_PIN)) //if recharge is in progress and motor not running and no error
  {
    switch (cycle)
    {
      case 1:
        if (mainSwitchTriggered)
        {
          responseln("INFO: backwash started. motor work time " + String(cycleStartTime - motorOnTime));
          mainSwitchTriggered = false;
        }
        if ((millis() - cycleStartTime) > BACKWASH_TIME * MILLIS_IN_MIN)  
        {
          responseln("INFO: backwash finished");
          nextCycle();
        }
      break;
      case 2:
        if (mainSwitchTriggered)
        {
          responseln("INFO: brine started. motor work time " + String(cycleStartTime - motorOnTime));
          mainSwitchTriggered = false;
        }
        if (CHEM_DURING_BRINE) digitalWrite(CHEM_PUMP_PIN, HIGH);
        if ((millis() - cycleStartTime) > BRINE_TIME * MILLIS_IN_MIN) 
        {
          responseln("INFO: brine finished");
          if (CHEM_DURING_BRINE) digitalWrite(CHEM_PUMP_PIN, LOW);
          nextCycle();
        }
      break;
      case 3:
        if (mainSwitchTriggered)
        {
          responseln("INFO: rinse started. motor work time " + String(cycleStartTime - motorOnTime));
          mainSwitchTriggered = false;
        }
        if ((millis() - cycleStartTime) > RINSE_TIME * MILLIS_IN_MIN) 
        {
          responseln("INFO: rinse finished");
          nextCycle();
        }
      break;
      case 4:
        if (mainSwitchTriggered)
        {
          responseln("INFO: refill started. motor work time " + String(cycleStartTime - motorOnTime));
          mainSwitchTriggered = false;
        }
        if ((millis() - cycleStartTime) > REFILL_TIME * MILLIS_IN_MIN)  
        {
          responseln("INFO: refill finished");
          nextCycle();
        }
      break;
      case 5:
        {         
          nextCycle();
        }
      break;
      default:
        if (!lastAuxSwitchState && millis() - lastMainSwitchTime > 2 * MIN_MOTOR_SW_TIME )
        {
          responseln("WARN: Cycle finished, but valve is not in default position! Re-init motor...");
          initMotor();
          resetCounter();
        }
        if (lastAuxSwitchState && millis() - lastAuxSwitchTime > MIN_MOTOR_SW_TIME)
        {
          response("INFO: **** Recharge finished! ****"); 
          if (mainSwitchTriggered)
          {
            responseln(" motor work time " + String(cycleStartTime - motorOnTime));
            mainSwitchTriggered = false;
          }
          else
           responseln("");
          cycle = 0; //recharge finished
          resetCounter();
          state = READY_STATE;
          if (CLOSE_OUT_VALVE_TILL_RECHARGE) openWaterSupply();
        }
      break;
    }
  }
}

void resetCounter()
{
  counterBase = counterBase - counter;
  tmp.l = counterBase;
  for (int i = 0; i < 4; i++) rtc.poke(RAM_CNTBASE_START_ADDR + i, tmp.b[i]);
  counter = 0;
  for (int i = 0; i < 4; i++) rtc.poke(RAM_COUNTER_START_ADDR + i, 0x00);
}

void nextCycle()
{
  if (digitalRead(MOTOR_DRIVE_PIN)) return;
  motorOnTime = millis();
  digitalWrite(MOTOR_DRIVE_PIN, HIGH);
}

void openWaterSupply()
{
  if (digitalRead(OUT_VALVE_PIN))
  {
    responseln("INFO: opening water supply to house");
    digitalWrite(OUT_VALVE_PIN, LOW);
  }
}

void closeWaterSupply()
{
  if (!digitalRead(OUT_VALVE_PIN))
  {
    responseln("INFO: closing water supply to house");
    digitalWrite(OUT_VALVE_PIN, HIGH);
  }
}

void openWaterSource()
{
  if (digitalRead(IN_VALVE_PIN))
  {
    responseln("INFO: opening water source");
    digitalWrite(IN_VALVE_PIN, LOW);
  }
}

void closeWaterSource()
{
  if (!digitalRead(IN_VALVE_PIN))
  {
    responseln("INFO: closing water source");
    digitalWrite(IN_VALVE_PIN, HIGH);
  }
}

void saveState()
{
  EEPROM.write(EEPROM_STATE_START_ADDR, cycle);
}

void readNetworkSettings()
{
  for (int i = 0; i < 6; i++) MAC[i] = EEPROM.read(EEPROM_MAC_START_ADDR + i); //reading 6 bytes of MAC from EEPROM
  for (int i = 0; i < 4; i++) IP[i] = EEPROM.read(EEPROM_IP_START_ADDR + i);
  for (int i = 0; i < 4; i++) MASK[i] = EEPROM.read(EEPROM_MASK_START_ADDR + i);
  for (int i = 0; i < 4; i++) GW[i] = EEPROM.read(EEPROM_GW_START_ADDR + i);  
  DHCP = bitRead(EEPROM.read(EEPROM_CONFIG_BYTE_ADDR), 0);
}


void writeNetworkSettings()
{
  for (int i = 0; i < 6; i++) EEPROM.write(EEPROM_MAC_START_ADDR + i, MAC[i]); //reading 6 bytes of MAC from EEPROM
  for (int i = 0; i < 4; i++) EEPROM.write(EEPROM_IP_START_ADDR + i, IP[i]);
  for (int i = 0; i < 4; i++) EEPROM.write(EEPROM_MASK_START_ADDR + i, MASK[i]);
  for (int i = 0; i < 4; i++) EEPROM.write(EEPROM_GW_START_ADDR + i, GW[i]); 
  byte romByte = EEPROM.read(EEPROM_CONFIG_BYTE_ADDR);
  bitWrite(romByte, 0, DHCP);
  EEPROM.write(EEPROM_CONFIG_BYTE_ADDR, romByte);

}

void readValveSettings()
{
  RECHARGE_MODE = EEPROM.read(EEPROM_RECHARGE_MODE_ADDR);
  RECHARGE_TIME_H = EEPROM.read(EEPROM_RECHARGE_TIME_H_ADDR);
  RECHARGE_TIME_M = EEPROM.read(EEPROM_RECHARGE_TIME_M_ADDR);
  CHARGE_VOLUME = EEPROM.read(EEPROM_CHARGE_VOLUME_ADDR);
  RECHARGE_DAY = EEPROM.read(EEPROM_RECHARGE_DAY_ADDR);
  BACKWASH_TIME = EEPROM.read(EEPROM_BACKWASH_TIME_ADDR);
  BRINE_TIME = EEPROM.read(EEPROM_BRINE_TIME_ADDR);
  RINSE_TIME = EEPROM.read(EEPROM_RINSE_TIME_ADDR);
  REFILL_TIME = EEPROM.read(EEPROM_REFILL_TIME_ADDR);
  CHEM_PUMP_PULSE = EEPROM.read(EEPROM_CHEM_PUMP_PULSE_ADDR);
  COUNTS_PER_CHEM_PULSE = EEPROM.read(EEPROM_COUNTS_PER_CHEM_PULSE_ADDR);
  AIR_PUMP_WORK_TIME = EEPROM.read(EEPROM_AIR_PUMP_WORK_TIME_ADDR);
  LITERS_PER_COUNT = EEPROM.read(EEPROM_LITERS_PER_COUNT_ADDR);
  
  byte romByte = EEPROM.read(EEPROM_VALVE_CONFIG_ADDR);
  if (romByte != 0xFF) //TODO по-ммоему это здесь зря
  {
   CHEM_DURING_BRINE = bitRead(romByte, 1);
   CLOSE_OUT_VALVE_TILL_RECHARGE = bitRead(romByte, 2);
   CLOSE_IN_VALVE_TILL_STANDBY = bitRead(romByte, 3);
   CLOSE_IN_VALVE_TILL_ERROR = bitRead(romByte, 4);
  }
}

void writeValveSettings()
{
  EEPROM.write(EEPROM_RECHARGE_MODE_ADDR, RECHARGE_MODE);
  EEPROM.write(EEPROM_RECHARGE_TIME_H_ADDR, RECHARGE_TIME_H);
  EEPROM.write(EEPROM_RECHARGE_TIME_M_ADDR, RECHARGE_TIME_M);
  EEPROM.write(EEPROM_CHARGE_VOLUME_ADDR, CHARGE_VOLUME);
  EEPROM.write(EEPROM_RECHARGE_DAY_ADDR, RECHARGE_DAY);
  EEPROM.write(EEPROM_BACKWASH_TIME_ADDR, BACKWASH_TIME);
  EEPROM.write(EEPROM_BRINE_TIME_ADDR, BRINE_TIME);
  EEPROM.write(EEPROM_RINSE_TIME_ADDR, RINSE_TIME);
  EEPROM.write(EEPROM_REFILL_TIME_ADDR, REFILL_TIME);
  EEPROM.write(EEPROM_CHEM_PUMP_PULSE_ADDR, CHEM_PUMP_PULSE);
  EEPROM.write(EEPROM_COUNTS_PER_CHEM_PULSE_ADDR, COUNTS_PER_CHEM_PULSE);
  EEPROM.write(EEPROM_AIR_PUMP_WORK_TIME_ADDR, AIR_PUMP_WORK_TIME);
  EEPROM.write(EEPROM_LITERS_PER_COUNT_ADDR, LITERS_PER_COUNT);
  
  byte romByte = EEPROM.read(EEPROM_VALVE_CONFIG_ADDR);
  bitWrite(romByte, 1, CHEM_DURING_BRINE);
  bitWrite(romByte, 2, CLOSE_OUT_VALVE_TILL_RECHARGE);
  bitWrite(romByte, 3, CLOSE_IN_VALVE_TILL_STANDBY);
  bitWrite(romByte, 4, CLOSE_IN_VALVE_TILL_ERROR);
  bitWrite(romByte, 7, 0);
  EEPROM.write(EEPROM_VALVE_CONFIG_ADDR, romByte);
}

void resetWatchDog()
{
  if (WATHDOG_ENABLED) digitalWrite(WATCHDOG_PIN, !digitalRead(WATCHDOG_PIN));//trigger watchog
}


unsigned long getRTCtime()
{
  Time t = rtc.getTime();
  
  tmElements_t rtcTm;
  rtcTm.Year = t.year -1970;
  rtcTm.Month = t.mon;
  rtcTm.Day = t.date;
  rtcTm.Hour = t.hour;
  rtcTm.Minute = t.min;
  rtcTm.Second = t.sec;
  
  unsigned long tmil = makeTime(rtcTm);
 
  return tmil;
}

void readCommand(Stream *str)
{
  char c;
  while (str->available())
  {
    c = str->read();
    if (c > 31 && c < 126) //filtering bullshit out
      textBuffer += String(c);
    if(c == 0x0d) 
    {
     if (textBuffer.length() > 0) 
     {
       printEcho();
       parseReceivedText();
     }
      textBuffer="";
      str->flush(); //flushing everything else
    }
    if (textBuffer.length() > TEXT_BUFFER_SIZE)
    {
      printErrorMessage();
      textBuffer="";
      str->flush(); //flushing everything else
      break;
    }
  }
}

void parseReceivedText()
{
  textBuffer.trim();
  textBuffer.toLowerCase();
  if (textBuffer.substring(0,2) == "sr")
  {
    printOK();
    startRecharge();
  }
  else if (textBuffer.substring(0,2) == "fn")
  { //force next cycle
    printOK();
    if (cycle < VALVE_CYCLES)
      nextCycle();
    else     
    {
      cycle = 0;
      responseln("INFO: returned to default position");
    }
  }
  else if (textBuffer.substring(0,2) == "rv")
  {
    if (textBuffer.substring(0,3) == "rv=")
    {
       textBuffer = textBuffer.substring(3);
       long vol = parseLongDigits();
       counterBase = counter - (vol * 10 / LITERS_PER_COUNT);
    }
    else
    {
      printOK();
      counterBase = counter;
    }
    tmp.l = counterBase;
    for (int i = 0; i < 4; i++) rtc.poke(RAM_CNTBASE_START_ADDR + i, tmp.b[i]);
  }
  else if (textBuffer.substring(0,2) == "rc")
  {
    resetCounter();
    printOK();
  }
  else if (textBuffer.substring(0,2) == "wr")
  {
    if (state == READY_STATE) state = WAIT_FOR_RECHARGE_STATE;
    printOK();
  }
  else if (textBuffer.substring(0,2) == "cr")
  {
    if (state == WAIT_FOR_RECHARGE_STATE) state = READY_STATE;
    printOK();
  }
   
   
  else if (textBuffer.substring(0,2) == "gv")
  {
    responseln("volume: "+ String((counter - counterBase) * LITERS_PER_COUNT/10) + " l");
    printOK();
  }  
  else if (textBuffer.substring(0,2) == "ps")
  {//print state
    responseln("counter: "+ String(counter));
    responseln("last recharge: " + String(year(lastRecharge)) + "." + String(month(lastRecharge)) + "." + String(day(lastRecharge)) + " " + String(hour(lastRecharge)) + ":" + String(minute(lastRecharge)) + ":" + String(second(lastRecharge)));
    responseln("date: "+ String(year()) + "." + String(month()) + "." + String(day()) + " " + String(hour()) + ":" + String(minute()) + ":" + String(second()));
    responseln("cycle: "+ String(cycle));
    responseln("state: "+ String(state));
    responseln("volume: "+ String((counter - counterBase) * LITERS_PER_COUNT/10)+ " l");
    response("motor: "); if (digitalRead(MOTOR_DRIVE_PIN)) responseln("ON"); else responseln("OFF");
    response("chem pump: "); if (digitalRead(CHEM_PUMP_PIN)) responseln("ON"); else responseln("OFF");
    response("air pump: "); if (digitalRead(AIR_PUMP_PIN)) responseln("ON"); else responseln("OFF");
    response("main switch: "); if (digitalRead(MOTOR_POS_MAIN_SWITCH_PIN)) responseln("OFF"); else responseln("ON");
    response("aux switch: "); if (digitalRead(MOTOR_POS_AUX_SWITCH_PIN)) responseln("OFF"); else responseln("ON");
    response("in valve: "); if (digitalRead(IN_VALVE_PIN)) responseln("CLOSE"); else responseln("OPEN");
    response("out valve: "); if (digitalRead(OUT_VALVE_PIN)) responseln("CLOSE"); else responseln("OPEN");
    response("main power: "); if (digitalRead(POWER_ON_PIN)) responseln("ON"); else responseln("OFF");
    printOK();
  }

  else if (textBuffer.substring(0,2) == "in")
  {
    initNetwork();
    printOK();
  }

  else if (textBuffer.substring(0,2) == "im")
  {
    printOK();
    initMotor();
  }
  else if (textBuffer.substring(0,2) == "oi")
  {
    openWaterSource();
    printOK();
  }
  else if (textBuffer.substring(0,2) == "ci")
  {
    closeWaterSource();
    printOK();
  }
  else if (textBuffer.substring(0,2) == "oo")
  {
    openWaterSupply();
    printOK();
  }
  else if (textBuffer.substring(0,2) == "co")
  {
    closeWaterSupply();
    printOK();
  }
  else if (textBuffer.substring(0,3) == "ap0")
  {
    printOK();
    airPumpManual = false;
    digitalWrite(AIR_PUMP_PIN, LOW);
  }
  else if (textBuffer.substring(0,3) == "ap1")
  {
    printOK();
    airPumpManual = true;
    digitalWrite(AIR_PUMP_PIN, HIGH);
  }
  else if (textBuffer.substring(0,3) == "cp0")
  {
    printOK();   
    chemPumpManual = false;
    digitalWrite(CHEM_PUMP_PIN, LOW);
  }
  else if (textBuffer.substring(0,3) == "cp1")
  {
    printOK();
    chemPumpManual  = true;
    digitalWrite(CHEM_PUMP_PIN, HIGH);
  }
  else if (textBuffer.substring(0,3) == "pns")
  {//print 
    printNetworkSettings();
    printOK();
  }
  else if (textBuffer.substring(0,3) == "snp")
  {//set network parameters
    textBuffer = textBuffer.substring(3);
    doNetworkSettings();
    writeNetworkSettings();
  }
  else if (textBuffer.substring(0,3) == "pvs")
  {//print 
    readValveSettings();
    printValveSettings();
    printOK();
  }
  else if (textBuffer.substring(0,3) == "svp")
  {//set valve parameters
    textBuffer = textBuffer.substring(3);
    doValveSettings();
    writeValveSettings();
  }
  else if (textBuffer.substring(0,2) == "st")
  {
    textBuffer = textBuffer.substring(2);
    doTimeSettings();
  }
  else if (textBuffer.substring(0,2) == "cl")
  {
    closeConnection();
  }
  else if (textBuffer.substring(0,1) == "?")
  {
    printHelpMessage();
  }
  else printErrorMessage();
 }

void printNetworkSettings()
{
  response("MAC: ");
  for (int i = 0; i < 6; i++) response(String(MAC[i], HEX) + ":");
  responseln("");
  response("DHCP: ");
  if (DHCP)
    responseln("ON");
  else
    responseln("OFF");
  response("IP: ");
  for (int i = 0; i < 4; i++) response(String(IP[i], DEC) + ".");
  responseln("");
  response("MASK: ");
  for (int i = 0; i < 4; i++) response(String(MASK[i], DEC) + ".");
  responseln("");
  response("GW: ");
  for (int i = 0; i < 4; i++) response(String(GW[i], DEC) + ".");
  responseln("");
}

void printValveSettings()
{
  responseln("mode: " + String(RECHARGE_MODE));
  responseln("time_h: " + String(RECHARGE_TIME_H));
  responseln("time_m: " + String(RECHARGE_TIME_M));
  responseln("vol: " + String(CHARGE_VOLUME));
  responseln("days: " + String(RECHARGE_DAY));
  responseln("backwash: " + String(BACKWASH_TIME));
  responseln("brine: " + String(BRINE_TIME));
  responseln("rinse: " + String(RINSE_TIME));
  responseln("refill: " + String(REFILL_TIME));
  responseln("pulse: " + String(CHEM_PUMP_PULSE * 10));
  response("chem_brine: "); if (CHEM_DURING_BRINE) responseln("ON"); else responseln("OFF");
  responseln("counts: " + String(COUNTS_PER_CHEM_PULSE));
  responseln("aero: " + String(AIR_PUMP_WORK_TIME));
  responseln("onepulse: " + String(LITERS_PER_COUNT/10) + "." + String(LITERS_PER_COUNT%10) + " //liters volume per one counter tick. sets in decimal parts of liter!");
  response("close_recharge: "); if (CLOSE_OUT_VALVE_TILL_RECHARGE) responseln("ON"); else responseln("OFF");
  response("close_standby: "); if (CLOSE_IN_VALVE_TILL_STANDBY) responseln("ON"); else responseln("OFF");
  response("close_on_err: "); if (CLOSE_IN_VALVE_TILL_ERROR) responseln("ON"); else responseln("OFF");
		 
}

void doValveSettings()
{
  int value = 0;
  if (textBuffer.substring(0,5) == "mode=")
  {
    textBuffer = textBuffer.substring(5);
    value = parseDigits();
    if (value >= 0 && value <= 3)
    {
      RECHARGE_MODE = value;
      printOK();
    }
    else
     printErrorMessage();
  }
  else if (textBuffer.substring(0,7) == "time_h=")
  {
    textBuffer = textBuffer.substring(7);
    value = parseDigits();
    if (value >= 0 && value <= 23)
    {
      RECHARGE_TIME_H = value;
      printOK();
    }
    else
     printErrorMessage();
  }
  else if (textBuffer.substring(0,7) == "time_m=")
  {
    textBuffer = textBuffer.substring(7);
    value = parseDigits();
    if (value >= 0 && value <= 59)
    {
      RECHARGE_TIME_M = value;
      printOK();
    }
    else
     printErrorMessage();
  }
  else if (textBuffer.substring(0,4) == "vol=")
  {
    textBuffer = textBuffer.substring(4);
    value = parseDigits();
    if (value >= 0 && value <= 254)
    {
      CHARGE_VOLUME = value;
      printOK();
    }
    else
     printErrorMessage();
  }
  else if (textBuffer.substring(0,5) == "days=")
  {
    textBuffer = textBuffer.substring(5);
    value = parseDigits();
    if (value >= 0 && value <= 254)
    {
      RECHARGE_DAY = value;
      printOK();
    }
    else
     printErrorMessage();
  }
  else if (textBuffer.substring(0,9) == "backwash=")
  {
    textBuffer = textBuffer.substring(9);
    value = parseDigits();
    if (value >= 0 && value <= 254)
    {
      BACKWASH_TIME = value;
      printOK();
    }
    else
     printErrorMessage();
  }
  else if (textBuffer.substring(0,6) == "brine=")
  {
    textBuffer = textBuffer.substring(6);
    value = parseDigits();
    if (value >= 0 && value <= 254)
    {
      BRINE_TIME = value;
      printOK();
    }
    else
     printErrorMessage();
  }
  else if (textBuffer.substring(0,6) == "rinse=")
  {
    textBuffer = textBuffer.substring(6);
    value = parseDigits();
    if (value >= 0 && value <= 254)
    {
      RINSE_TIME = value;
      printOK();
    }
    else
     printErrorMessage();
  }
  else if (textBuffer.substring(0,7) == "refill=")
  {
    textBuffer = textBuffer.substring(7);
    value = parseDigits();
    if (value >= 0 && value <= 254)
    {
      REFILL_TIME = value;
      printOK();
    }
    else
     printErrorMessage();
  }
  else if (textBuffer.substring(0,6) == "pulse=")
  {
    textBuffer = textBuffer.substring(6);
    value = parseDigits();
    if (value >= 0 && value <= 2540)
    {
      CHEM_PUMP_PULSE = value / 10;
      printOK();
    }
    else
     printErrorMessage();
  }
  else if (textBuffer.substring(0,11) == "chem_brine=")
  {
    textBuffer = textBuffer.substring(11);
    if (textBuffer == "on")
    {
      CHEM_DURING_BRINE = true;
      printOK();
    }
    else if (textBuffer == "off")
    {
      CHEM_DURING_BRINE = false;
      printOK();
    }
    else
     printErrorMessage();
  }
  
  else if (textBuffer.substring(0,15) == "close_recharge=")
  {
    textBuffer = textBuffer.substring(15);
    if (textBuffer == "on")
    {
      CLOSE_OUT_VALVE_TILL_RECHARGE = true;
      printOK();
    }
    else if (textBuffer == "off")
    {
      CLOSE_OUT_VALVE_TILL_RECHARGE = false;
      printOK();
    }
    else
     printErrorMessage();
  }
    else if (textBuffer.substring(0,14) == "close_standby=")
  {
    textBuffer = textBuffer.substring(14);
    if (textBuffer == "on")
    {
      CLOSE_IN_VALVE_TILL_STANDBY = true;
      printOK();
    }
    else if (textBuffer == "off")
    {
      CLOSE_IN_VALVE_TILL_STANDBY = false;
      printOK();
    }
    else
     printErrorMessage();
  }
    else if (textBuffer.substring(0,13) == "close_on_err=")
  {
    textBuffer = textBuffer.substring(13);
    if (textBuffer == "on")
    {
      CLOSE_IN_VALVE_TILL_ERROR = true;
      printOK();
    }
    else if (textBuffer == "off")
    {
      CLOSE_IN_VALVE_TILL_ERROR = false;
      printOK();
    }
    else
     printErrorMessage();
  }
  
  
  else if (textBuffer.substring(0,7) == "counts=")
  {
    textBuffer = textBuffer.substring(7);
    value = parseDigits();
    if (value >= 0 && value <= 254)
    {
      COUNTS_PER_CHEM_PULSE = value;
      printOK();
    }
    else
     printErrorMessage();
  }
  else if (textBuffer.substring(0,5) == "aero=")
  {
    textBuffer = textBuffer.substring(5);
    value = parseDigits();
    if (value >= 0 && value <= 254)
    {
      AIR_PUMP_WORK_TIME = value;
      printOK();
    }
    else
     printErrorMessage();
  }
  else if (textBuffer.substring(0,9) == "onepulse=")
  {
    textBuffer = textBuffer.substring(9);
    value = parseDigits();
    if (value >= 0 && value <= 254)
    {
      LITERS_PER_COUNT = value;
      printOK();
    }
    else
     printErrorMessage();
  }
  else printErrorMessage();
  
}

void doTimeSettings()
{
  int value = 0;
  tmElements_t setTm;
  breakTime(now(), setTm);
  if (textBuffer.substring(0,5) == "year=")
  {
    textBuffer = textBuffer.substring(5);
    value = parseDigits();
    if (value >= 2000 && value <= 2099)
    {
      setTm.Year = value - 1970;
      printOK();
    }
    else
     printErrorMessage();
  }
  else if (textBuffer.substring(0,6) == "month=")
  {
    textBuffer = textBuffer.substring(6);
    value = parseDigits();
    if (value >= 1 && value <= 12)
    {
      setTm.Month = value;
      printOK();
    }
    else
     printErrorMessage();
  }
  else if (textBuffer.substring(0,4) == "day=")
  {
    textBuffer = textBuffer.substring(4);
    value = parseDigits();
    if (value >= 1 && value <= 31)
    {
      setTm.Day = value;
      printOK();
    }
    else
     printErrorMessage();
  }
  else if (textBuffer.substring(0,5) == "hour=")
  {
    textBuffer = textBuffer.substring(5);
    value = parseDigits();
    if (value >= 0 && value <= 23)
    {
      setTm.Hour = value;
      printOK();
    }
    else
     printErrorMessage();
  }
  else if (textBuffer.substring(0,4) == "min=")
  {
    textBuffer = textBuffer.substring(4);
    value = parseDigits();
    if (value >= 0 && value <= 59)
    {
      setTm.Minute = value;
      printOK();
    }
    else
     printErrorMessage();
  }
  else if (textBuffer.substring(0,4) == "sec=")
  {
    textBuffer = textBuffer.substring(4);
    value = parseDigits();
    if (value >= 0 && value <= 59)
    {
      setTm.Second = value;
      printOK();
    }
    else
     printErrorMessage();
  }
   else if (textBuffer.substring(0,9) == "unixsecs=")
  {
    textBuffer = textBuffer.substring(9);
    long unixsecs = parseLongDigits();
    if (unixsecs > 0)
    {
      breakTime(unixsecs, setTm);
      printOK();
    }
    else
     printErrorMessage();
  }
  else
     printErrorMessage();
     
  rtc.setDOW(setTm.Wday);       
  rtc.setTime(setTm.Hour, setTm.Minute, setTm.Second);
  rtc.setDate(setTm.Day, setTm.Month, tmYearToCalendar(setTm.Year)); //adjust time in RTC first 

  setTime(getRTCtime()); //then update local 
}

void doNetworkSettings()
{
  if (textBuffer.substring(0,4) == "mac=")
  {
    //set mac
    textBuffer = textBuffer.substring(4);
    if (parseBytes(6))
    {
      for(int i = 0; i < 6; i++) MAC[i] = parsedBytesBuffer[i];
      printOK();
    }
    else
      printErrorMessage();
  }
  else if (textBuffer.substring(0,5) == "dhcp=")
  {
    //set dhcp
    textBuffer = textBuffer.substring(5);
    if (textBuffer == "on")
    {
      DHCP = true;
      printOK();
    }
    else if (textBuffer == "off")
    {
      DHCP = false;
      printOK();
    }
    else
      printErrorMessage();
  }
  else if (textBuffer.substring(0,3) == "ip=")
  {
    //set ip
    textBuffer = textBuffer.substring(3);
    if (parseDecAddr())
    {
      for(int i = 0; i < 4; i++) IP[i] = parsedBytesBuffer[i];
      printOK();
    }
    else
      printErrorMessage();
  }
  else if (textBuffer.substring(0,5) == "mask=")
  {
    //set mask
    textBuffer = textBuffer.substring(5);
    if (parseDecAddr())
    {
      for(int i = 0; i < 4; i++) MASK[i] = parsedBytesBuffer[i];
      printOK();
    }
    else
      printErrorMessage();
  }
  else if (textBuffer.substring(0,3) == "gw=")
  {
    //set gateway
    textBuffer = textBuffer.substring(3);
    if (parseDecAddr())
    {
      for(int i = 0; i < 4; i++) GW[i] = parsedBytesBuffer[i];
      printOK();
    }
    else
      printErrorMessage();
  }
  
  else
      printErrorMessage();

}

boolean parseBytes(int numBytes)
{
  String strByte;
  int bytes = 0;
  while (textBuffer.indexOf(":") > 0 && bytes < numBytes)
  {
    int pos = textBuffer.indexOf(":");
    strByte = textBuffer.substring(0,pos);
    textBuffer = textBuffer.substring(pos+1);
    int oneByte = parseByte(strByte);
    if (oneByte < 0) return false;
    parsedBytesBuffer[bytes] = oneByte;
    bytes++;
  }
  if (bytes != numBytes || textBuffer.indexOf(":") > 0) return false;
  return true;
}

boolean parseDecAddr()
{
  String strByte;
  int bytes = 0;
  for (int i = 0; i < 4; i++)
  {
    int oneByte = parseDigits();
    if (oneByte < 0 || oneByte > 255) return false;
    parsedBytesBuffer[i] = oneByte;
    textBuffer = textBuffer.substring(1);
  }
  return true;
}


int parseDigit(char c)
{
  int digit = -1;
  digit = (int) c - 0x30; // subtracting 0x30 from ASCII code gives value
  if(digit < 0 || digit > 9) digit = -1;
  return digit;
}

long parseLongDigits() //parses buffer from begin, shrinks it till first non-digit, returns number;
{
  long num = -1;
  int digit = 0;
  for (int i=0; i < textBuffer.length(); i++)
  {
    digit = parseDigit(textBuffer.charAt(i));
    if (digit < 0) 
    {
      textBuffer = textBuffer.substring(i);
      break;
    }
    if (num == -1) num = 0;
    num = num*10 + digit;
  }
  return num;
}

int parseDigits() //parses buffer from begin, shrinks it till first non-digit, returns number;
{
  int num = -1;
  int digit = 0;
  for (int i=0; i < textBuffer.length(); i++)
  {
    digit = parseDigit(textBuffer.charAt(i));
    if (digit < 0) 
    {
      textBuffer = textBuffer.substring(i);
      break;
    }
    if (num == -1) num = 0;
    num = num*10 + digit;
  }
  return num;
}

int parseByte(String byteStr)
{
  int oneByte = 0;
  if (byteStr.length() == 1)
  {
    oneByte = (int)byteStr.charAt(0) - 0x30;
    if (oneByte < 0)
      return -1;
    if (oneByte > 9)
    {
      oneByte = (int)byteStr.charAt(0) - 0x57;
      if (oneByte < 0x0A || oneByte > 0x0F) return -1;
    }
    return oneByte;
  }
  else if (byteStr.length() == 2)
  {
    oneByte = (int)byteStr.charAt(0) - 0x30;
    if (oneByte < 0)
      return -1;
    if (oneByte > 9)
    {
      oneByte = (int)byteStr.charAt(0) - 0x57;
      if (oneByte < 0x0A || oneByte > 0x0F) return -1;
    }
    int secondByte = (int)byteStr.charAt(1) - 0x30;
    if (secondByte < 0)
      return -1;
    if (secondByte > 9)
    {
      secondByte = (int)byteStr.charAt(1) - 0x57;
      if (secondByte < 0x0A || secondByte > 0x0F) return -1;
    }
    return (oneByte*16 + secondByte);
  }
  return -1;
}

void printEcho()
{
  responseln(textBuffer); //printing command back to all clients
}

void printOK()
{
  responseln(" OK");
}

void printErrorMessage()
{
  responseln("ERROR: unrecognized command");
}

void closeConnection()
{
  responseln("\nBye.\n");
  client.stop();
}

void response(String text)
{
  server.print(text);
  Serial.print(text);
}

void responseln(String text)
{
  server.println(text);
  Serial.println(text);
}

void printHelpMessage()
{
responseln("Supported commands:");
responseln("");
responseln(" sr                        - start recharge");
responseln("");
responseln(" sp                        - set configuration parameters. ");
responseln("                             Device reboot needed for settings to take effect.");
responseln(" spMAC=0A:0B:0B:0D:0E:0F   - set 6 bytes of MAC address");
responseln(" spIP=192.168.0.3");
responseln(" spMASK=255.255.255.0");
responseln(" spGW=192.168.0.1          - set ip, subnet mask, default gateway");
responseln(" spDHCP=ON                 - set DHCP ON. If DHCP is ON, IP ");
responseln("                             obtained from DHCP server automatically on");
responseln("                             power-on, and IP, GW, MASK settings are ignored.");
responseln("");
responseln(" pc                        - print configuration parameters");
responseln(" cl                        - close connection");
responseln(" ?                         - print this help message");
responseln("");
responseln("  Commands are not case sensitive.");
responseln("");

}

