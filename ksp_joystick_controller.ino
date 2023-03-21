#include <Wire.h>
//#include <Arduino.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
# include <string.h>


#include <KerbalSimpit.h>
 

#define I2C_ADDRESS 0x3c
 
#define RST_PIN -1

SSD1306AsciiAvrI2c oled;


// throttle 
const int throttle = A3;
const int numReadings = 5;
int readings[numReadings];
int index = 0;
int total = 0;

int throttle_average = 0;

const int joystick_x = A1;
const int joystick_y = A2;
const int joystick_z = A0;

const int flight_switch = 12;
const int eva_switch = 13;
const int sas_led = 11;


const int antitarget_switch = 10;
const int sas_switch = 9;
const int maneuver_switch = 8;
const int prograde_switch = 7;
const int retrograde_switch = 6;
const int normal_switch = 5;
const int antinormal_switch = 4;
const int target_switch = 3;



int antitarget_switch_state = 0;
int sas_switch_state = 0;
int maneuver_switch_state = 0;
int prograde_switch_state = 0;
int retrograde_switch_state = 0;
int normal_switch_state = 0;
int antinormal_switch_state = 0;
int target_switch_state = 0;







const int joystick_button = 2;
int joystick_button_state;
int joystick_button_last_state = LOW;

int joystick_button_mode = 0;


unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 20;



String joystickButtonMode = "J-Button Mode: ";


int throttle_state = 0;
int joystick_x_state = 0;
int joystick_y_state = 0;
int joystick_z_state = 0;

int flight_switch_state = 0;
int eva_switch_state = 0;
int rcs_switch_state = 0;






int rcs_state = 0;
int sas_state = 0;
int throttle_display = 0;

int autopilot_current_mode = 0;

int joystick_current_mode = 0;

bool sas_last_state = false;

bool rcs_current_mode = false;

bool display_refresh = false;


int throttle_last_value = 0;

bool isFlying;

byte currentActionStatus = true;

String resultAltitude;
String resultPeriapsis;
String resultApoapsis;
String resultOrbitalVelocity;

String resultDeltaV;
String resultLOX;
String resultLNG;
String resultSolidFuel;


String kilo_metres = " km";
String metres = " m";
String metres_per_second = " m/s";
String mega_metres = " Mm";
String litres = " l";

String throttle_string = "Throttle: ";
int throttle_display_state;



String autopilotMode = "Autopilot: ";
String autopilotMode_state;
String rcsMode = "RCS: ";
String rcsMode_state;
String joystickMode = "J-Mode: ";
String joystickMode_state;

KerbalSimpit mySimpit(Serial);


void setup() {
  Serial.begin(115200);


  #if RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
  #else // RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  #endif // RST_PIN >= 0

  pinMode(throttle, INPUT);
  pinMode(joystick_x, INPUT);
  pinMode(joystick_y, INPUT);
  pinMode(joystick_z, INPUT);

  pinMode(antitarget_switch, INPUT);
  pinMode(sas_switch, INPUT);
  pinMode(maneuver_switch, INPUT);
  pinMode(prograde_switch, INPUT);
  pinMode(retrograde_switch, INPUT);
  pinMode(normal_switch, INPUT);
  pinMode(antinormal_switch, INPUT);
  pinMode(target_switch, INPUT);

  pinMode(joystick_button, INPUT);


  pinMode(flight_switch, INPUT);
  pinMode(eva_switch, INPUT);
  pinMode(sas_led, OUTPUT);


  
  while (!mySimpit.init()) {
    delay(100);
  }
  
  mySimpit.printToKSP("Connected", PRINT_TO_SCREEN);
  // Sets our callback function. The KerbalSimpit library will
  // call this function every time a packet is received.
  mySimpit.inboundHandler(messageHandler);
  // Send a message to the plugin registering for the Action status channel.
  // The plugin will now regularly send Action status  messages while the
  // flight scene is active in-game.
  mySimpit.registerChannel(ACTIONSTATUS_MESSAGE);
  mySimpit.registerChannel(ALTITUDE_MESSAGE);
  mySimpit.registerChannel(VELOCITY_MESSAGE);
  mySimpit.registerChannel(APSIDES_MESSAGE);
  mySimpit.registerChannel(DELTAV_MESSAGE);
  mySimpit.registerChannel(LF_MESSAGE);
  mySimpit.registerChannel(OX_MESSAGE);
  mySimpit.registerChannel(SF_MESSAGE);
  mySimpit.registerChannel(SCENE_CHANGE_MESSAGE);

  oled.setFont(Adafruit5x7);
  

}
 
void loop() {
 


  mySimpit.update();

  
  int reading_joystick_button = digitalRead(joystick_button);

  if (reading_joystick_button != joystick_button_last_state)
  {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay)
  {
    if (reading_joystick_button != joystick_button_state)
    {
      joystick_button_state = reading_joystick_button;
      if (joystick_button_state == HIGH)
      {
        if (joystick_button_mode < 2)
        {
          joystick_button_mode++;
        }
        else
        {
          joystick_button_mode = 0;
        }
        mySimpit.printToKSP("Joystick Button pressed!");
        //delay(50);
      }
    }
  }
  joystick_button_last_state = reading_joystick_button;

  joystick_x_state = analogRead(joystick_x);
  joystick_y_state = analogRead(joystick_y);
  joystick_z_state = analogRead(joystick_z);

  
  eva_switch_state = digitalRead(eva_switch);
  flight_switch_state = digitalRead(flight_switch);

  
  if ((eva_switch_state == LOW) && (flight_switch_state == LOW))
  { 
    joystickMode_state = "RCS";
      

    translationMessage trans_msg;

    if ((joystick_x_state > 450) && (joystick_x_state < 541)) 
    {
      joystick_x_state = 0;
      trans_msg.setX(joystick_x_state);
      
    }
    else 
    { 
      
      joystick_x_state = map(joystick_x_state, 0,1023, INT16_MIN,INT16_MAX);
      trans_msg.setX(joystick_x_state);
    }
    if ((joystick_y_state > 475) && (joystick_y_state < 541)) 
    {
      joystick_y_state = 0;
      trans_msg.setY(joystick_y_state);
    }
    else {
    {
      joystick_y_state = map(joystick_y_state, 0,1023, INT16_MIN, INT16_MAX);
      trans_msg.setY(joystick_y_state);
    }
    } 
    if ((joystick_z_state > 450) && (joystick_z_state < 541)) 
    {
      joystick_z_state = 0;
      trans_msg.setZ(joystick_z_state);
    }
    else
    {
      joystick_z_state = map(joystick_z_state, 0,1023, INT16_MIN,INT16_MAX);
      trans_msg.setZ(joystick_z_state);

    } 


    mySimpit.send(TRANSLATION_MESSAGE, trans_msg);
    
  }
  else if (eva_switch_state == HIGH)
  {
    joystickMode_state = "EVA";

  } 
  else if (flight_switch_state == HIGH)
  {
    joystickMode_state = "Flight";
   

    rotationMessage rot_msg; 
    
    if ((joystick_x_state > 450) && (joystick_x_state < 541)) 
    {
      joystick_x_state = 0;
      rot_msg.setYaw(joystick_x_state);
      
    }
    else 
    { 
      
      joystick_x_state = map(joystick_x_state, 0,1023, INT16_MIN,INT16_MAX);
      rot_msg.setYaw(joystick_x_state);
    }
    if ((joystick_y_state > 475) && (joystick_y_state < 541)) 
    {
      joystick_y_state = 0;
      rot_msg.setPitch(joystick_y_state);
    }
    else {
    {
      joystick_y_state = map(joystick_y_state, 0,1023, INT16_MIN, INT16_MAX);
      rot_msg.setPitch(joystick_y_state);
    }
    } 
    if ((joystick_z_state > 450) && (joystick_z_state < 541)) 
    {
      joystick_z_state = 0;
      rot_msg.setRoll(joystick_z_state);
    }
    else
    {
      joystick_z_state = map(joystick_z_state, 0,1023, INT16_MIN,INT16_MAX);
      rot_msg.setRoll(joystick_z_state);

    } 
    /*
    String x = "X-Axis";
    String y = "Y-Axis";
    String z = "Z-Axis";
    
    mySimpit.printToKSP(x + joystick_x_state);
    mySimpit.printToKSP(y + joystick_y_state);
    mySimpit.printToKSP(z + joystick_z_state);
    */
    mySimpit.send(ROTATION_MESSAGE, rot_msg);
    
  }
 


  total = total- readings[index];
  readings[index] = analogRead(throttle);

  total = total + readings[index];

  index = index + 1;

  if (index >= numReadings)
  {
    index = 0;
  }

  throttle_average = total / numReadings;

  throttleMessage throttle_msg;
  if ((throttle_average <= 6))
  {
    throttle_display_state = 0;
    throttle_msg.throttle = 0;
    mySimpit.send(THROTTLE_MESSAGE, throttle_msg);
  }
  else if ((throttle_average >= 1017))
  {
    throttle_display_state = 100;

    throttle_msg.throttle = INT16_MAX;
    mySimpit.send(THROTTLE_MESSAGE, throttle_msg);
  }
  else
  {
    throttle_display_state = (throttle_average/1023.0) * 100;

    throttle_msg.throttle = map(throttle_average, 0, 1023, 0, INT16_MAX);
    mySimpit.send(THROTTLE_MESSAGE, throttle_msg); 
  }
  rcs_state = (currentActionStatus & RCS_ACTION) != 0;


  if ((rcs_state == true) && (joystick_button_mode == 0))
  {
    rcsMode_state = "Engaged";
  }
  else if ((rcs_state == false) && (joystick_button_mode ==0)) 
  {
    rcsMode_state = "Disengaged";
  }
 
  sas_state = (currentActionStatus & SAS_ACTION) != 0;

  if (sas_state == true)
  {
    antitarget_switch_state = digitalRead(antitarget_switch);
    sas_switch_state = digitalRead(sas_switch);
    maneuver_switch_state = digitalRead(maneuver_switch);
    prograde_switch_state = digitalRead(prograde_switch);
    retrograde_switch_state = digitalRead(retrograde_switch);
    normal_switch_state = digitalRead(normal_switch);
    antinormal_switch_state = digitalRead(antinormal_switch);
    target_switch_state = digitalRead(target_switch);
    
    digitalWrite(sas_led, HIGH);
     
    if (antitarget_switch_state == HIGH) 
    {
      mySimpit.setSASMode(AP_ANTITARGET);
      autopilotMode_state = "Antitarget";
     

    }
    else if (sas_switch_state == HIGH)
    {
      mySimpit.setSASMode(AP_STABILITYASSIST);
      autopilotMode_state = "SAS";
      
    }
    else if (maneuver_switch_state == HIGH )
    {
      mySimpit.setSASMode(AP_MANEUVER);
      autopilotMode_state = "Maneuver";
      
    }

    else if (prograde_switch_state == HIGH)
    {
      mySimpit.setSASMode(AP_PROGRADE);
      autopilotMode_state = "Prograde";
     
      
    }
    else if (retrograde_switch_state == HIGH)
    {
      mySimpit.setSASMode(AP_RETROGRADE);
      autopilotMode_state = "Retrograde";
    
      
      
    }

    else if (normal_switch_state == HIGH)
    {
      mySimpit.setSASMode(AP_NORMAL);
      autopilotMode_state = "Normal";
     
    }
    else if (antinormal_switch_state == HIGH)
    {
      mySimpit.setSASMode(AP_ANTINORMAL);
      autopilotMode_state = "Antinormal";

      
    }

    else if (target_switch_state == HIGH)
    {
      mySimpit.setSASMode(AP_TARGET);
      autopilotMode_state = "Target";
     
      
    }      
  }
  else if ((sas_state == false ) && (joystick_button_mode == 0)) 
  {
    autopilotMode_state = "Disengaged";
    digitalWrite(sas_led, LOW);
  }
  else if (sas_state == false) 
  {
    digitalWrite(sas_led, LOW);
  }
  



  if ((joystick_button_mode == 0) && (isFlying == true))
  {
    draw_firstRow(joystickMode + joystickMode_state);
    draw_secondRow(autopilotMode + autopilotMode_state);
    draw_thirdRow(rcsMode + rcsMode_state);
    draw_fourthRow(throttle_string + throttle_display_state + "%");
  }
  else if ((joystick_button_mode == 1 ) && (isFlying == true))
  {
    draw_firstRow(resultAltitude);
    draw_secondRow(resultPeriapsis);
    draw_thirdRow(resultApoapsis);
    draw_fourthRow(resultOrbitalVelocity);
  }
  else if ((joystick_button_mode ==2 ) && (isFlying == true))
  {
    draw_firstRow(resultDeltaV);
    draw_secondRow(resultLNG);
    draw_thirdRow(resultLOX);
    draw_fourthRow(resultSolidFuel);
  }
  else 
  {
    oled.clear();
  }

}


void draw_firstRow(String firstRowString)
{
  oled.setCursor(0, 0);
  oled.print(firstRowString);
  oled.clearToEOL();
}
void draw_secondRow(String secondRowString)
{
  oled.setCursor(0, 2);
  oled.print(secondRowString);
  oled.clearToEOL();
}
void draw_thirdRow(String thirdRowString)
{
  oled.setCursor(0, 4);
  oled.print(thirdRowString);
  oled.clearToEOL();
}
void draw_fourthRow(String fourthRowString)
{
  oled.setCursor(0, 6);
  oled.print(fourthRowString);
  oled.clearToEOL();
}



void messageHandler(byte messageType, byte msg[], byte msgSize) {
  switch(messageType) 
  {
    case ACTIONSTATUS_MESSAGE:
      // Checking if the message is the size we expect is a very basic
      // way to confirm if the message was received properly.
      if (msgSize == 1) {
        currentActionStatus = msg[0];

        //Let the LED_BUILIN match the current SAS state        
      }
      break;
    case ALTITUDE_MESSAGE:
      if (msgSize == sizeof(altitudeMessage))
      {
        altitudeMessage myAltitude;
        String alt_string = "Altitude: ";
        myAltitude = parseMessage<altitudeMessage>(msg);
        long altitude = myAltitude.sealevel;

        if (abs(altitude) < 5000)
        {
          resultAltitude = alt_string + altitude + metres; 
        }
        else if ((abs(altitude) >= 5000) && (altitude < pow(10, 6)))
        {
          resultAltitude = alt_string + altitude/pow(10,3) + kilo_metres;
        }
        else 
        {
          resultAltitude = alt_string + altitude/pow(10,6) + mega_metres;
        }      
      }
      break;
    case APSIDES_MESSAGE:
      if (msgSize == sizeof(apsidesMessage))
      {
        apsidesMessage myApside;
        myApside = parseMessage<apsidesMessage>(msg);
        String apo_string = "Apoapsis: ";
        String peri_string = "Periapsis: ";
        long apoapsis = myApside.apoapsis;
        long periapsis = myApside.periapsis;

        if (abs(apoapsis) < 5000)
        {
          resultApoapsis = apo_string + apoapsis + metres; 
        }
        else if ((abs(apoapsis) >= 5000) && (apoapsis < pow(10, 6)))
        {
          resultApoapsis = apo_string + apoapsis/pow(10,3) + kilo_metres;
        }
        else 
        {
          resultApoapsis = apo_string + apoapsis/pow(10,6) + mega_metres;
        }

        if (abs(periapsis) < 5000)
        {
          resultPeriapsis = peri_string + periapsis + metres; 
        }
        else if ((abs(periapsis) >= 5000) && (periapsis < pow(10, 6)))
        {
          resultPeriapsis = peri_string + periapsis/pow(10,3) + kilo_metres;
        }
        else 
        {
          resultPeriapsis = peri_string + periapsis/pow(10,6) + mega_metres;
        }
      }
      break;
    case VELOCITY_MESSAGE:
      if (msgSize == sizeof(velocityMessage))
      {
        velocityMessage myVelocity;
        String orb_velo_string = "Orb. Velo.: ";
        myVelocity = parseMessage<velocityMessage>(msg);
        int orbitalVelocity = myVelocity.orbital;

        resultOrbitalVelocity = orb_velo_string + orbitalVelocity + metres_per_second;
      }
      break;
    case DELTAV_MESSAGE:
      if (msgSize == sizeof(deltaVMessage))
      {
        deltaVMessage myDeltaV;
        String deltaVString = "Delta-V:     ";
        myDeltaV = parseMessage<deltaVMessage>(msg);
        int deltaV = myDeltaV.totalDeltaV;
        resultDeltaV = deltaVString + deltaV + metres_per_second;

      }
      break;
    case LF_MESSAGE:
      if (msgSize == sizeof(resourceMessage))
      {
        resourceMessage myLiquidFuel;
        String liquidFuel_string = "LNG:         ";
        myLiquidFuel = parseMessage<resourceMessage>(msg);
        long LiquidFuel = myLiquidFuel.available;
        resultLNG = liquidFuel_string + LiquidFuel + litres;
      }
      break;
    case OX_MESSAGE:
      if (msgSize == sizeof(resourceMessage))
      {
        resourceMessage myLOX;
        String LOX_string = "LOX:         ";
        myLOX = parseMessage<resourceMessage>(msg);
        long LOX = myLOX.available;
        resultLOX = LOX_string + LOX + litres; 
      }
      break;
    case SF_MESSAGE:
      if ( msgSize == sizeof(resourceMessage))
      {
        resourceMessage mySolidFuel;
        String solidFuel_string = "SF:          ";
        mySolidFuel = parseMessage<resourceMessage>(msg);
        long SF = mySolidFuel.available;
        resultSolidFuel = solidFuel_string + SF + litres;
      }
      break;
    case SCENE_CHANGE_MESSAGE:
      isFlying = !msg[0];
      break;

  }
}
