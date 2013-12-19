  /*
   * PID3piLineFollower - demo code for the Pololu 3pi Robot
   * 
   * This code will follow a black line on a white background, using a
   * PID-based algorithm.
   *
   * http://www.pololu.com/docs/0J21
   * http://www.pololu.com
   * http://forum.pololu.com
   *
   */
  
  // The following libraries will be needed by this demo
  #include <Pololu3pi.h>
  #include <PololuQTRSensors.h>
  #include <OrangutanMotors.h>
  #include <OrangutanAnalog.h>
  #include <OrangutanLEDs.h>
  #include <OrangutanLCD.h>
  #include <OrangutanPushbuttons.h>
  #include <OrangutanBuzzer.h>
  #define NUMBEROFELEMENTS(x) (sizeof(x)/sizeof(*x))
  
  
  Pololu3pi robot;
  unsigned int sensors[5]; // an array to hold sensor values
  
  // This include file allows data to be stored in program space.  The
  // ATmega168 has 16k of program space compared to 1k of RAM, so large
  // pieces of static data should be stored in program space.
  #include <avr/pgmspace.h>
  
  // Introductory messages.  The "PROGMEM" identifier causes the data to
  // go into program space.
  const char welcome_line1[] PROGMEM = " Pololu";
  const char welcome_line2[] PROGMEM = "3\xf7 Robot";
  const char demo_name_line1[] PROGMEM = "PID Line";
  const char demo_name_line2[] PROGMEM = "follower";
  
  // A couple of simple tunes, stored in program space.
  const char welcome[] PROGMEM = ">g32>>c32";
  const char go[] PROGMEM = "L16 cdegreg4";
    
  // My custom menu for setting the values of basemaximum speed, KP, KI, and KD
  const char maximumspeed[] PROGMEM = "Speed";
  const char kproportionalname[] PROGMEM = "KP";
  const char kintegralname[] PROGMEM = "KI";
  const char kderivativename[] PROGMEM = "KD";
  
  const int basemaximum[] = { 80, 100, 150 };
  
//  const int basemaximum0 = 80;
//  const int basemaximum1 = 100;
//  const int basemaximum2 = 150;
  int basemaximumcount = 0;
  
  
  int subtractoneloop(int *variable, int lowerbound, int upperbound) {
    *variable = *variable - 1;
    if ((*variable)<lowerbound) {
      *variable = upperbound-1;
    }
  }
  
  int addoneloop(int *variable, int lowerbound, int upperbound) {
    *variable = *variable + 1;
    if ((*variable)>upperbound-1) {
      *variable = lowerbound;
    }
  }
  
  const float kproportional[] = { 0, .07, 1 };
  const char* kproportionalstring[] = { "0", ".07", "1" };
//  const int kproportional0 = 0;
//  const int kproportional1 = .07;
//  const int kproportional2 = 1;
  int kproportionalcount = 1;
  
  const float kintegral[] = { 0, .0005, 1};
  const char* kintegralstring[] = { "0", ".0005", "1"};
//  const int kintegral0 = 0;
//  const int kintegral1 = .0005;
//  const int kintegral2 = 1;
  int kintegralcount = 1;
  
  const float kderivative[] = { 0, 1.7, 10};
  const char* kderivativestring[] = { "0", "1.7", "10"};
//  const int kderivative0 = 0;
//  const int kderivative1 = 1.7;
//  const int kderivative2 = 10;
  int kderivativecount = 1;
  
  // Data for generating the characters used in load_custom_characters
  // and display_readings.  By reading levels[] starting at various
  // offsets, we can generate all of the 7 extra characters needed for a
  // bargraph.  This is also stored in program space.
  const char levels[] PROGMEM = {
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111
  };
  
  // This function loads custom characters into the LCD.  Up to 8
  // characters can be loaded; we use them for 7 levels of a bar graph.
  void load_custom_characters()
  {
    OrangutanLCD::loadCustomCharacter(levels + 0, 0); // no offset, e.g. one bar
    OrangutanLCD::loadCustomCharacter(levels + 1, 1); // two bars
    OrangutanLCD::loadCustomCharacter(levels + 2, 2); // etc...
    OrangutanLCD::loadCustomCharacter(levels + 3, 3);
    OrangutanLCD::loadCustomCharacter(levels + 4, 4);
    OrangutanLCD::loadCustomCharacter(levels + 5, 5);
    OrangutanLCD::loadCustomCharacter(levels + 6, 6);
    OrangutanLCD::clear(); // the LCD must be cleared for the characters to take effect
  }
  
  // This function displays the sensor readings using a bar graph.
  void display_readings(const unsigned int *calibrated_values)
  {
    unsigned char i;
  
    for (i=0;i<5;i++) {
      // Initialize the array of characters that we will use for the
      // graph.  Using the space, an extra copy of the one-bar
      // character, and character 255 (a full black box), we get 10
      // characters in the array.
      const char display_characters[10] = { ' ', 0, 0, 1, 2, 3, 4, 5, 6, 255 };
  
      // The variable c will have values from 0 to 9, since
      // calibrated values are in the range of 0 to 1000, and
      // 1000/101 is 9 with integer math.
      char c = display_characters[calibrated_values[i] / 101];
  
      // Display the bar graph character.
      OrangutanLCD::print(c);
    }
  }
  
  // Initializes the 3pi, displays a welcome message, calibrates, and
  // plays the initial music.  This function is automatically called
  // by the Arduino framework at the start of program execution.
  void setup()
  {
    unsigned int counter; // used as a simple timer
  
    // This must be called at the beginning of 3pi code, to set up the
    // sensors.  We use a value of 2000 for the timeout, which
    // corresponds to 2000*0.4 us = 0.8 ms on our 20 MHz processor.
    robot.init(2000);
  
    load_custom_characters(); // load the custom characters
  
    // Play welcome music and display a message
    OrangutanLCD::printFromProgramSpace(welcome_line1);
    OrangutanLCD::gotoXY(0, 1);
    OrangutanLCD::printFromProgramSpace(welcome_line2);
    OrangutanBuzzer::playFromProgramSpace(welcome);
    delay(1000);
  
    OrangutanLCD::clear();
    OrangutanLCD::printFromProgramSpace(demo_name_line1);
    OrangutanLCD::gotoXY(0, 1);
    OrangutanLCD::printFromProgramSpace(demo_name_line2);
    delay(1000);
    
    while(!OrangutanPushbuttons::isPressed(BUTTON_B)) {
      if (OrangutanPushbuttons::isPressed(BUTTON_A)) {
        subtractoneloop(&basemaximumcount, 0, 2);
      }  
      if (OrangutanPushbuttons::isPressed(BUTTON_C)) {
        addoneloop(&basemaximumcount, 0, NUMBEROFELEMENTS(basemaximum));
      }
      OrangutanLCD::clear();
      OrangutanLCD::printFromProgramSpace(maximumspeed);
      OrangutanLCD::gotoXY(0,1);
      OrangutanLCD::print(basemaximum[basemaximumcount]);
      delay(500);
    }
    
    // Always wait for the button to be released so that 3pi doesn't
    // start moving until your hand is away from it.
    OrangutanPushbuttons::waitForRelease(BUTTON_B);
    delay(1000);
      
    while(!OrangutanPushbuttons::isPressed(BUTTON_B)) {
      if (OrangutanPushbuttons::isPressed(BUTTON_A)) {
        subtractoneloop(&kproportionalcount, 0, NUMBEROFELEMENTS(kproportional));
      }
      if (OrangutanPushbuttons::isPressed(BUTTON_C)) {
        addoneloop(&kproportionalcount, 0, NUMBEROFELEMENTS(kproportional));
      }
      OrangutanLCD::clear();
      OrangutanLCD::printFromProgramSpace(kproportionalname);
      OrangutanLCD::gotoXY(0,1);
      OrangutanLCD::print(kproportionalstring[kproportionalcount]);
      delay(500);
    }
    
    // Always wait for the button to be released so that 3pi doesn't
    // start moving until your hand is away from it.
    OrangutanPushbuttons::waitForRelease(BUTTON_B);
    
    while(!OrangutanPushbuttons::isPressed(BUTTON_B)) {
      if (OrangutanPushbuttons::isPressed(BUTTON_A)) {
        subtractoneloop(&kintegralcount, 0, NUMBEROFELEMENTS(kintegral));
      }
      if (OrangutanPushbuttons::isPressed(BUTTON_C)) {
        addoneloop(&kintegralcount, 0, NUMBEROFELEMENTS(kintegral));
      }
      OrangutanLCD::clear();
      OrangutanLCD::printFromProgramSpace(kintegralname);
      OrangutanLCD::gotoXY(0,1);
      OrangutanLCD::print(kintegralstring[kintegralcount]);
      delay(500);
    }
    
    // Always wait for the button to be released so that 3pi doesn't
    // start moving until your hand is away from it.
    OrangutanPushbuttons::waitForRelease(BUTTON_B);
  
    while(!OrangutanPushbuttons::isPressed(BUTTON_B)) {
      if (OrangutanPushbuttons::isPressed(BUTTON_A)) {
        subtractoneloop(&kderivativecount, 0, NUMBEROFELEMENTS(kderivative));
      }
      if (OrangutanPushbuttons::isPressed(BUTTON_C)) {
        addoneloop(&kderivativecount, 0, NUMBEROFELEMENTS(kderivative));
      }
      OrangutanLCD::clear();
      OrangutanLCD::printFromProgramSpace(kderivativename);
      OrangutanLCD::gotoXY(0,1);
      OrangutanLCD::print(kderivativestring[kderivativecount]);
      delay(500);
    }
    
    // Always wait for the button to be released so that 3pi doesn't
    // start moving until your hand is away from it.
    OrangutanPushbuttons::waitForRelease(BUTTON_B);
    
    // Display battery voltage and wait for button press
    while (!OrangutanPushbuttons::isPressed(BUTTON_B))
    {
      int bat = OrangutanAnalog::readBatteryMillivolts();
  
      OrangutanLCD::clear();
      OrangutanLCD::print(bat);
      OrangutanLCD::print("mV");
      OrangutanLCD::gotoXY(0, 1);
      OrangutanLCD::print("Press B");
  
      delay(100);
    }
  
    // Always wait for the button to be released so that 3pi doesn't
    // start moving until your hand is away from it.
    OrangutanPushbuttons::waitForRelease(BUTTON_B);
    delay(1000);
  
    // Auto-calibration: turn right and left while calibrating the
    // sensors.
    for (counter=0; counter<80; counter++)
    {
      if (counter < 20 || counter >= 60)
        OrangutanMotors::setSpeeds(40, -40);
      else
        OrangutanMotors::setSpeeds(-40, 40);
  
      // This function records a set of sensor readings and keeps
      // track of the minimum and maximum values encountered.  The
      // IR_EMITTERS_ON argument means that the IR LEDs will be
      // turned on during the reading, which is usually what you
      // want.
      robot.calibrateLineSensors(IR_EMITTERS_ON);
  
      // Since our counter runs to 80, the total delay will be
      // 80*20 = 1600 ms.
      delay(20);
    }
    OrangutanMotors::setSpeeds(0, 0);
  
    // Display calibrated values as a bar graph.
    while (!OrangutanPushbuttons::isPressed(BUTTON_B))
    {
      // Read the sensor values and get the position measurement.
      unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);
  
      // Display the position measurement, which will go from 0
      // (when the leftmost sensor is over the line) to 4000 (when
      // the rightmost sensor is over the line) on the 3pi, along
      // with a bar graph of the sensor readings.  This allows you
      // to make sure the robot is ready to go.
      OrangutanLCD::clear();
      OrangutanLCD::print(position);
      OrangutanLCD::gotoXY(0, 1);
      display_readings(sensors);
  
      delay(100);
    }
    OrangutanPushbuttons::waitForRelease(BUTTON_B);
  
    OrangutanLCD::clear();
  
    OrangutanLCD::print("Go!");		
  
    // Play music and wait for it to finish before we start driving.
    OrangutanBuzzer::playFromProgramSpace(go);
    while(OrangutanBuzzer::isPlaying());
  }
  
  #define SETPOINT 2000 // 2000 is the center of the robot
  int error=0; // The difference between the linePosition and SETPOINT
  int proportional=0; // The error * the proportional gain (KP)
  int integral=0; // The error summed continuosly * the integral ratio (KI)
  float derivative=0; // The error's average change times the derivative ratio (KD)
  #define AVERAGEERRORDELTA (error - lastError)/(2.0) // The error's average change
  int lastError = 0;
  float output; // The sum of proportional, integral and derivative  
  int maximum = 100; //Fastest possible motor speed
  
  float KP = kproportional[kproportionalcount];
  float KI = kintegral[kintegralcount];
  float KD = kderivative[kderivativecount];
  
  #define SPEEDRATIO 1  //the ratio of to convert the output to speed
  
  boolean isCorner;
  boolean isTurning = false;
  
  void loop() {
    // Gives a value between 0 and 4000 based off of the location of the line.
    // The value of 0 means that the line is under one side,
    // 2000 is under the center, and 4000 is on the other side.
    unsigned int linePosition = robot.readLine(sensors, IR_EMITTERS_ON);

    maximum = basemaximum[basemaximumcount];
    
    error = linePosition - SETPOINT;
    proportional = error * KP;
    integral = integral + error * KI;
    
    // Keeps the integral under a cap to prevent infinite spinning in circles
    if (integral > maximum)
      integral = maximum;
    if (integral < -maximum)
      integral = -maximum;
    
    derivative = AVERAGEERRORDELTA * KD;
    
    output = proportional + integral + derivative;
    
    lastError = error;
    
    isCorner = false;
    
    // Compute the actual motor settings.  We never set either motor
    // to a negative value.
    // Forces -maximum <= output <= maximum
    if (output > maximum) {
        maximum = 60;
        output = maximum;
        isCorner = true;
        isTurning = true;
    }
    else if (output < -maximum) {  
        maximum = 60;
        output = -maximum;
        isCorner = true;
        isTurning = true;
    }
    else {
      if (isTurning == true) {
        isTurning = false;
//        OrangutanMotors::setSpeeds(0,0);
        OrangutanBuzzer::playNote(36, 100, 15);
//        delay(1000);
      }
    }
    
//    if (isCorner == true) {
//      isTurning = true;
//    }
//    else {
//      if (isTurning == true) {
//        OrangutanMotors::setSpeeds(0,0);
//        OrangutanBuzzer::playNote(36, 100, 15);
//        delay(1000);
//      }
//      isTurning = false;
//    }
    
    if (output < 0)
      OrangutanMotors::setSpeeds(maximum + output, maximum - (SPEEDRATIO * output)); //slows the left motor and speeds up the right motor to cause it to turn left
    else
      OrangutanMotors::setSpeeds(maximum + (SPEEDRATIO * output), maximum - output); //speeds the left motor and slows the right motor to cause it to turn right
  }
