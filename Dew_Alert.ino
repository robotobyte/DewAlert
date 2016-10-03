 // ****************************************************************************

/*
 * DEW ALERT: Dew Point Detection and Warning System
 * -------------------------------------------------
 * Code by W.Witt; V1.00-beta-06; September 2016
 *
 * The code below implements a dew point detection and warning system
 * It senses temperature and relative humidity, uses that data to
 * calculate the dew point, and then provides an indication of how
 * close ambient conditions are to reaching the dew point. Based on the
 * temperature distance to the dew point (e.g. 2 deg-C away) it signals
 * that conditions are safe (i.e. far away from dew), the dew point is
 * near, the dew point is very close (warning condition) or the dew
 * point has been reached. Alternatively, proximity to the dew point
 * and associated dew formation risk may be assessed purely based on
 * relative humidity, and the overall dew state is then computed based
 * on worst case (i.e. smaller) of temperature margin and relative
 * humidity margin.
 * 
 * The intended application for this environment monitor is to track
 * when equipment (e.g. telescopes during a night-time observing
 * session) need to be protected before they're affected by moisture.
 *
 * The dew point state (SAFE, NEAR, WARN, DEW) is signalled via three
 * LEDs (green, yellow, red) and a buzzer. The detector includes an
 * ambient light sensor to adjust the brightness of the indicator LEDs
 * based on ambient light level, such that the LEDs will not be
 * disturbingly bright during night-time conditions.
 * 
 * Additionally a small LCD panel displays the measured temperature
 * and relative humidity values, the calculated dew point and the dew
 * point state.
 *
 * With the display in place, the temperature display mode (degrees
 * Celsius or Fahrenheit) and the various thresholds (e.g. NEAR/SAFE
 * humidity threshold) are user configurable (i.e. don't require
 * source code changes). The configuration state is stored in EEPROM.
 * 
 */

// ----------------------------------------------------------------------------

// LCD Column:                    0123456789012345
#define DEW_ALERT_NAME_STRING    "** Dew Alert **"
#define DEW_ALERT_VERSION_STRING "V1.00-beta-06"

// ----------------------------------------------------------------------------
// Pin Assignments:
// ----------------------------------------------------------------------------

/* 
 * For Arduino UNO prototype build, pin and wire assignments are as follows:
 *
 * Pin          Function                      Pin Mode      Wire
 * ----------   ---------------------------   -----------   ----------
 * Digital  0
 * Digital  1
 * Digital  2   Enter/Action button           Input         orange
 * Digital  3   LCD back-light LED, red       Output, PWM   (actually via I2C)
 * Digital  4   Low-battery indication        Input(*)      white
 * Digital  5   Power button light            Output, PWM   white/blue
 * Digital  6   Enter/Action button light     Output, PWM   green
 * Digital  7   Up button                     Input         violet
 * Digital  8   Down button                   Input         light blue
 * Digital  9   Indicator LED Warn (yellow)   Output, PWM   white/yellow
 * Digital 10   Indicator LED Safe (green)    Output, PWM   white/green
 * Digital 11   Indicator LED Dew  (red)      Output, PWM   white/red
 * Digital 12   Piezo buzzer                  Output        white
 * Digital 13
 * Analog  A0   Ambient light sensor (LDR)    Input         gray
 * Analog  A1
 * Analog  A2
 * Analog  A3
 * Analog  A4   I2C SDA (data)                              blue
 * Analog  A5   I2C SCL (clock)                             yellow
 *
 * I2C --> Temp/Humidity Sensor, LCD Display
 * 
 * (*) INPUT_PULLUP (provide external pull-up in next hardware revision?)
 *
 */

// ****************************************************************************

// Standard include file for I2C interface...
#include <Wire.h>

// Include EEPROM functions for saving and retrieving configuration data...
#include <EEPROM.h>

// Include files for Adafruit environmental sensors...
// BME280 Sensor: https://www.adafruit.com/product/2652
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
// Alternative sensors:
// SHT31-D: https://www.adafruit.com/products/2857
// HTU21D-F: https://www.adafruit.com/products/1899

// Include files for Adafruit LCD with I2C backpack...
// LCD Panel: https://www.adafruit.com/products/399
// I2C Backpack: https://www.adafruit.com/products/292
#include <Adafruit_LiquidCrystal.h>

// Include file for advanced math functions (e.g. log)... 
#include <math.h>

// Include files for handy button-debounce and timer classes...
#include <CwwButton.h>
#include <CwwElapseTimer.h>

// Include files for LED controller and hysteresis filter classes...
#include <CwwLedController.h>
#include <CwwFilterHysteresis.h>

// ----------------------------------------------------------------------------

// Address to store configuration data structure in EEPROM...
#define EEPROM_CONFIG_ADDRESS 0x000

// Plus/minus offsets around thresholds for hysteresis filters...
#define HYSTERESIS_OFFSET_TEMP_MARGIN_C    0.1
#define HYSTERESIS_OFFSET_HUMIDITY_PERCENT 1.0

// Codes to set liquid crystal display backlight behavior...
#define LCD_BACKLIGHT_ON  0x01  // back-light red
#define LCD_BACKLIGHT_OFF 0x00  // back-light off

// For time-limited audible alarm mode, repeat limited alarm every
// LIMITED_ALARM_REPEAT_MINUTES minutes (0 means no repeat)...
#define LIMITED_ALARM_REPEAT_MINUTES 5

// ----------------------------------------------------------------------------

// Enumerated type for the state of the ambient light...
enum enumLightLevel {
  LIGHT_LOW   = 0,
  LIGHT_MED   = 1,
  LIGHT_HIGH  = 2,
  LIGHT_RESET = 255
};

// Enumerated type for the state of the dew detector (i.e. how close
// ambient conditions are to the dew point)...
enum enumDewState {
  DP_RESET      = 0,
  DP_SAFE       = 1,
  DP_NEAR       = 2,
  DP_WARN_ALARM = 3,
  DP_WARN       = 4,
  DP_DEW_ALARM  = 5,
  DP_DEW        = 6
};

// Enumerated type for audible alarm modes...
enum enumAlarmMode {
  ALARM_NONE      = 0,  // no audible alarm
  ALARM_LIMITED   = 1,  // automatically time-limited alarm
  ALARM_SUSTAINED = 2   // sustained alarm (until canceled by user)
};
// If the alarm modes are changed (some added or deleted), also fix up
// associated code in setConfigFromUser function.

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

struct dewConfigStruct {
  float         thTempMarginC [3];
  float         thHumPercent  [3];
  boolean       showTempAsF;
  enumAlarmMode audibleAlarmMode;
  // For threshold arrays, indexes are:
  // 0: DEW/WARN threshold
  // 1: WARN/NEAR threshold
  // 2: NEAR/SAFE threshold
};

// ----------------------------------------------------------------------------

// Instantiate bme280 temperature, humidity, pressure sensor... 
Adafruit_BME280 bme280;  // I2C address 0x77 assigned by call to begin method

// Instantiate LCD display panel with I2C interface...
Adafruit_LiquidCrystal lcd ( 0x00 );  // I2C address 0x00

// ----------------------------------------------------------------------------

// Define input pin for low-battery signal from power converter board...
const uint8_t pinOfLowBat = 4;

// Define pin of light sensor (light-dependent resistor, LDR) for ambient
// light measurement...
const uint8_t pinOfLdr = A0;
// Prototype was built with 300 to 200k ohm LDR. Wiring is
//   Power---LDR---Resistor---Ground
// with pinOfLdr (analog input pin) connected between the LDR and the
// resistor, and with the value of the resistor (R) at about 10k ohm. If
// a different LDR is used, the resistor will likely need to be changed:
//    R = sqrt( LDR_max / LDR_min ) * LDR_min
// where LDR_max and LDR_min are the highest (fully dark) and lowest (fully
// lit) resistance values of the light-dependent resistor, respectively.
// For a 300 to 200k ohm LDR, the optimal resistor is 7.7k ohm, but using
// 10k ohm because it's much easier to find and close enough.
// The range of values that may be read from the analog pin (after analog-
// to-digital conversion) depends on LDR and series resistor values.
// With LDR of 300 to 200k ohm and series resistor of 10k ohm, expected
// pin value range is 49 through 994 (out of 0 to 1023).

// Instantiate LED controller objects for primary indicator lights
// ( pin, PWM-mode )...
CwwLedController indicatorLedSafe ( 10, true );  // Assume green LED
CwwLedController indicatorLedWarn (  9, true );  // Assume yellow LED
CwwLedController indicatorLedDew  ( 11, true );  // Assume red LED
// Using PWM pins for all to allow for dimming of indicator LEDs based
// on ambient light conditions.

// Use another LED controller object for a piezo buzzer...
CwwLedController indicatorBuzzer  ( 12, false );

// And two more LED controllers for the power button and the 
// alternative, lit enter button (see primary enter button below)...
CwwLedController powerButtonLight ( 5, true );
CwwLedController enterButtonLight ( 6, true );

// Create special action sequences for LED controllers that handle
// the buzzer and the enter button for time-limited audible and visual
// alarm...
CwwLedSequence limitedAlarmSequenceWarn;
CwwLedSequence limitedAlarmSequenceDew;

// Create object for momentary off(on) push button for enter function...
CwwButton buttonEnter ( 2, 10 );  // pin 2, debounce time of 10ms
// Button is assumed active low, wired as
//   Power---Resistor---Button---Ground
// with the pin connected between the resistor and button.
// Reasonable resistor value is around 10k ohm.

// Similarly, create objects for up and down navigation buttons...
CwwButton buttonUp   ( 7, 10 );
CwwButton buttonDown ( 8, 10 );

// ----------------------------------------------------------------------------

// Instantiate two hysteresis filters, one for the distance to dew and
// another for the ambient light...

// Temperature margin to dew point is used for assessing dew risk;
// use three thresholds for dew, translating to four zones (from
// low to high): DEW, WARN, NEAR and SAFE...
CwwFilterHysteresis dewPointMarginFilter ( 3 );

// Similar to dew point margin filter, set up another filter for
// dew risk purely based on relative humidity percentage...
CwwFilterHysteresis humidityFilter ( 3 );

// Use two thresholds for light level, translating to three zones
// (from low to high): LOW, MED, HIGH...
CwwFilterHysteresis ambientLightLevelFilter ( 2 ); 

// These filters are to mask the effects of each sensor bouncing or
// wobbling around a threshold, so that such wobble does not bleed
// through to the indicators. Threshold values are configured below.

// ----------------------------------------------------------------------------

// State variable to hold current dew state...
enumDewState   dewState;

// State variable to track ambient light level...
enumLightLevel lightLevel;

// Timer to control how frequently to update LCD panel...
CwwElapseTimer displayTimer ( 1000 );  // display refresh rate of once per second

// Another timer for displaying the low-battery alarm (when necessary)...
CwwElapseTimer batteryTimer ( 3000 );

// Miscellaneous operating state flags...
boolean showTempAsF;
boolean audibleAlarmMode;
boolean inTestMode;
boolean batteryIsOkay;

// ============================================================================

// Code in setup() runs once after Arduino power-on or reset...
void setup () {

  // Declare local variables and objects...
  CwwElapseTimer  configEntryTimer;
  dewConfigStruct dewConfig;

  // Light up power button to indicate device is on...
  powerButtonLight.turnHigh ();

  // Initialize display...
  initDisplay ();

  // If battery is okay, say hello;
  // otherwise, stop with low-battery message...
  pinMode ( pinOfLowBat, INPUT_PULLUP );
  batteryIsOkay = digitalRead(pinOfLowBat) == HIGH;
  if ( batteryIsOkay ) {
    displayIntro ();
  }
  else {
    displayLowBat ();
    while ( true );
  }

  // Define periods for oscillating (fade-up/fade-down) or blinking
  // (on/off) indicator LEDs...
  indicatorLedWarn.setOscillatePeriod ( 1000 );  // one second
  indicatorLedDew. setOscillatePeriod ( 1000 );  // one second
  indicatorLedDew. setBlinkPeriod     (  500 );  // half a second

  // Set "blink", really beep, period of piezo buzzer indicator...
  indicatorBuzzer.setBlinkPeriod  ( 500 );  // half a second
  // Enter button will light/blink just like buzzer...
  enterButtonLight.setBlinkPeriod ( 500 );

  // Define time-limited buzzer sequence for WARN alarm...
  limitedAlarmSequenceWarn.addStep (   0, LED_HIGH );
  limitedAlarmSequenceWarn.addStep ( 250, LED_LOW  );
#if LIMITED_ALARM_REPEAT_MINUTES > 0
  limitedAlarmSequenceWarn.addStep ( 250, LED_HIGH );
  limitedAlarmSequenceWarn.addStep ( 250, LED_LOW  );
  limitedAlarmSequenceWarn.addStep ( 250, LED_HIGH );
  limitedAlarmSequenceWarn.addStep ( 250, LED_LOW  );
  limitedAlarmSequenceWarn.addStep ( 250, LED_HIGH );
  limitedAlarmSequenceWarn.addStep ( 250, LED_LOW  );
  limitedAlarmSequenceWarn.addStep ( 250, LED_HIGH );
  limitedAlarmSequenceWarn.addStep ( 250, LED_LOW  );
  limitedAlarmSequenceWarn.addStep ( LIMITED_ALARM_REPEAT_MINUTES*60000L, LED_LOW );
  limitedAlarmSequenceWarn.setRepeatCount ( 0 );
#else
  limitedAlarmSequenceWarn.addStep ( 250, LED_LOW  );
  limitedAlarmSequenceWarn.setRepeatCount ( 5 );
#endif

  // Define time-limited buzzer sequence for DEW alarm...
  limitedAlarmSequenceDew.addStep (   0,  LED_HIGH );
  limitedAlarmSequenceDew.addStep ( 5000, LED_LOW  );
#if LIMITED_ALARM_REPEAT_MINUTES > 0
  limitedAlarmSequenceWarn.addStep ( LIMITED_ALARM_REPEAT_MINUTES*60000L, LED_LOW );
  limitedAlarmSequenceWarn.setRepeatCount ( 0 );
#else
  limitedAlarmSequenceWarn.setRepeatCount ( 1 );
#endif

  // Check for magic button press to allow user a chance to enter
  // configuration mode and either set configuration by hand or
  // reset configuration to defaults: enter button down for 3
  // seconds to enter configuration mode, another 5 seconds to
  // perform factory reset...
  delay ( 500 );
  if ( buttonEnter.isLowStable() ) {
    configEntryTimer.start ( 3000 );
    while ( buttonEnter.isLow() && ! configEntryTimer.hasElapsed() );
    if ( configEntryTimer.hasElapsed() ) {
      lcd.clear ();
      lcd.setCursor ( 0, 0 );
      lcd.print ( "Dew Config Mode" );
      lcd.setCursor ( 0, 1 );
      lcd.print ( "[Hold for Reset]" );
      configEntryTimer.start ( 5000 );
      while ( buttonEnter.isLow() && ! configEntryTimer.hasElapsed() );
      if ( configEntryTimer.hasElapsed() ) resetConfigToDefault ();
      else                                 setConfigFromUser    ();
      displayIntro ();
    }
  }
  // Load configuration from non-volatile memory...
  loadConfig ( &dewConfig );

  // Save temperature display and audible alarm modes for global
  // consumption (e.g. for use by loop function)...
  showTempAsF      = dewConfig.showTempAsF;
  audibleAlarmMode = dewConfig.audibleAlarmMode;
  
  // Define the three temperature margin thresholds for the dew
  // point hysteresis filter; measurement unit for input value is
  // degrees C; use same hysteresis offset for all...
  dewPointMarginFilter.defineThreshold ( 0, dewConfig.thTempMarginC[0], HYSTERESIS_OFFSET_TEMP_MARGIN_C );  // DEW/WARN threshold
  dewPointMarginFilter.defineThreshold ( 1, dewConfig.thTempMarginC[1], HYSTERESIS_OFFSET_TEMP_MARGIN_C );  // WARN/NEAR threshold
  dewPointMarginFilter.defineThreshold ( 2, dewConfig.thTempMarginC[2], HYSTERESIS_OFFSET_TEMP_MARGIN_C );  // NEAR/SAFE threshold
  dewPointMarginFilter.initHistory ( 10.0 );  // arbitrarily start filter assuming 10C to dew point

  // Similarly, define three relative humidity thresholds for the
  // alternate dew point hysteresis filter; measurement unit for
  // input value is relative humidity percent; use same hysteresis
  // offset for all...
  humidityFilter.defineThreshold ( 0, dewConfig.thHumPercent[0], HYSTERESIS_OFFSET_HUMIDITY_PERCENT );  // DEW/WARN threshold
  humidityFilter.defineThreshold ( 1, dewConfig.thHumPercent[1], HYSTERESIS_OFFSET_HUMIDITY_PERCENT );  // WARN/NEAR threshold
  humidityFilter.defineThreshold ( 2, dewConfig.thHumPercent[2], HYSTERESIS_OFFSET_HUMIDITY_PERCENT );  // NEAR/SAFE threshold
  humidityFilter.initHistory ( 25.0 );  // arbitrarily start filter assuming 25% humidity

  // Define the two thresholds for the ambient light hysteresis 
  // filter (measurement unit for input value is 0-1023 analogRead()
  // result, i.e. 10-bit A-to-D converter)...
  ambientLightLevelFilter.defineThreshold ( 0, 400, 25 );  // LOW/MED threshold:  400 +/- 25
  ambientLightLevelFilter.defineThreshold ( 1, 750, 50 );  // MED/HIGH threshold: 750 +/- 50
  ambientLightLevelFilter.initHistory ( 0 );  // arbitrarily start filter assuming low light

  // Initialize ambient light level state (start at reset state for no
  // assumption about the initial lighting state)...
  lightLevel = LIGHT_RESET;
  updateLightLevel ();

  // Run indicator lights and buzzer through a quick test to demonstrate
  // they all work...
  runIndicatorLedTest ();

  // Display ready message and get user to press button to start (partly
  // to test button works)...
  displayReadyAndTestButton ( true );

  // Activate communication with primary environmental (temperature,
  // humidity) sensor...
  bme280.begin ( 0x77 );  // I2C address of 0x77

  inTestMode = false;
  // Check for another magic button press -- enter button held down
  // for five seconds -- to enter test mode...
  if ( buttonEnter.isLowStable() ) {
    configEntryTimer.start ( 5000 );
    while ( buttonEnter.isLow() && ! configEntryTimer.hasElapsed() );
    if ( configEntryTimer.hasElapsed() ) {
      lcd.clear ();
      lcd.setCursor ( 0, 0 );
      lcd.print ( "State change" );
      lcd.setCursor ( 0, 1 );
      lcd.print ( "test mode..." );
      inTestMode = true;
      delay ( 1000 );
    }
  }

  // Set up display for main operating behavior...
  formatDewDisplay ( showTempAsF );
  displayTimer.start ();

  // Initialize dew state to reset state, indicating that initial state
  // is unknown...
  dewState = DP_RESET;
 
}

// ----------------------------------------------------------------------------

// Code in loop() keeps running after setup...
void loop () {

  // Declare local variables...
  float        sensorTemperatureDegC;
  float        sensorHumidityPercent;
  float        dewPointTemperatureDegC;
  float        marginToDewDegC;
  uint8_t      marginToDewQuantized;
  uint8_t      marginToH100Quantized;
  uint8_t      effectiveMargin;
  boolean      humIsTrigger;
  enumDewState dewStateNext;

  // Check battery state, and issue alert if battery is low...
  if ( ! batteryIsOkay || digitalRead(pinOfLowBat) == LOW ) {
    if ( batteryIsOkay || batteryTimer.hasElapsed() ) {
      displayLowBat ();
      indicatorBuzzer.turnOn  ();
      delay ( 200 );
      indicatorBuzzer.turnOff ();
      delay ( 800 ); 
      formatDewDisplay ( showTempAsF );
      batteryTimer.start ();
    }   
    batteryIsOkay = false;
  }

  // Check ambient light level and adapt indicator LEDs accordingly...
  updateLightLevel ();
  
  // Check whether to toggle LCD back-light based on button presses...
  if ( ! inTestMode ) {
    if      ( buttonUp.  isLowStable() ) lcd.setBacklight ( LCD_BACKLIGHT_ON  );
  	else if ( buttonDown.isLowStable() ) lcd.setBacklight ( LCD_BACKLIGHT_OFF );
  }

  // Read primary environmental sensor...
  sensorTemperatureDegC = bme280.readTemperature ();
  sensorHumidityPercent = bme280.readHumidity ();

  // Calculate dew point temperature from current temperature and
  // relative humidity values...
  dewPointTemperatureDegC = tempOfDewPoint1 ( sensorHumidityPercent, sensorTemperatureDegC );
  // Calculate margin from current temperature to dew-point temperature...
  marginToDewDegC = sensorTemperatureDegC - dewPointTemperatureDegC;

  // If in state change test mode, choose zone that determines state
  // based on button presses...
  if ( inTestMode ) {

    switch ( dewState ) {
      case DP_RESET:
      case DP_SAFE:
        effectiveMargin = 3;
        break;
      case DP_NEAR:
        effectiveMargin = 2;
        break;
      case DP_WARN_ALARM:
      case DP_WARN:
        effectiveMargin = 1;
        break;
      case DP_DEW_ALARM:
      case DP_DEW:
        effectiveMargin = 0;
        break;
    }

    // Up and down buttons pressed together exit test mode, otherwise
    // up and down buttons change state...
    if ( buttonUp.isLowStable() && buttonDown.isLowStable() ) {
      inTestMode = false;
      dewState = DP_RESET;
      formatDewDisplay ( showTempAsF );
      delay ( 250 );
    }
    else {
      if ( buttonUp.isLowStable() ) {
        if ( effectiveMargin < 3 ) effectiveMargin++;
        delay ( 250 );
      }
      if ( buttonDown.isLowStable() ) {
        if ( effectiveMargin > 0 ) effectiveMargin--;
        delay ( 250 );
      }
      humIsTrigger = false;
    }

  }  // if ( inTestMode )

  // If not in test mode (or test mode was just exited), decide
  // state based on sensor results...
  if ( ! inTestMode ) {
  
    // Assess dew risk with two parallel methods:
    // 1. temperature margin to dew point
    // 2. margin to 100% relative humidity
    // Note that dew may form on items long before the temperature
    // reaches the dew point and relative humidity reaches 100%.

    // Run temperature margin through hysteresis filter; map margin value
    // to detection zone...
    marginToDewQuantized = dewPointMarginFilter.mapValueToZone ( marginToDewDegC );
    // Similarly, run relative humidity through hysteresis filter (here
    // higher zone means lower margin, so need to invert the result)...
    marginToH100Quantized = humidityFilter.zoneCount() - 1 -
                            humidityFilter.mapValueToZone ( sensorHumidityPercent );

  	// Take the worst case (smallest margin) of the two...
    effectiveMargin = min ( marginToDewQuantized, marginToH100Quantized );
    humIsTrigger = marginToH100Quantized < marginToDewQuantized;
 
  }  // if ( ! inTestMode )

  // Map zone to new dew state...
  switch ( effectiveMargin ) {
    case 0:  // zone below dew point; there's dew now
      // If this zone was newly reached from another, non-DEW zone,
      // go to alarm sub-state first; otherwise, just stay in DEW...
      if ( dewState != DP_DEW ) {
        // Move from DEW_ALARM to DEW (i.e. allow user to cancel
        // audible alarm) based on press of enter button...
        if ( buttonEnter.isLowStable() ) dewStateNext = DP_DEW;
        else                             dewStateNext = DP_DEW_ALARM;
      }
      else {
        dewStateNext = DP_DEW;
      }
      break;
    case 1:  // zone just before dew point; time to prepare
      // If this zone was was reached from NEAR or SAFE, then
      // start with alarm first...
      if ( dewState != DP_WARN && dewState != DP_DEW && dewState != DP_DEW_ALARM ) {
        // Move from WARN_ALARM to WARN (i.e. allow user to cancel
        // audible alarm) based on press of enter button...
        if ( buttonEnter.isLowStable() ) dewStateNext = DP_WARN;
        else                             dewStateNext = DP_WARN_ALARM;
      }
      else {
        dewStateNext = DP_WARN;
      }
      break;
    case 2:  // zone near dew point, but no reason to panic
      dewStateNext = DP_NEAR;
      break;
    case 3:  // zone far away from dew point; safe
      dewStateNext = DP_SAFE;
      break;
  }

  // If the dew state has changed, modify indicator behavior
  // accordingly...
  if ( dewStateNext != dewState ) {
    dewState = dewStateNext;
    switch ( dewState ) {
      case DP_SAFE:
        indicatorLedSafe.turnHigh  ();
        indicatorLedWarn.turnOff   ();
        indicatorLedDew .turnOff   ();
        indicatorBuzzer .turnOff   ();
        enterButtonLight.turnOff   ();
        break;
      case DP_NEAR:
        indicatorLedSafe.turnOff   ();
        indicatorLedWarn.oscillate ();
        indicatorLedDew .turnOff   ();
        indicatorBuzzer .turnOff   ();
        enterButtonLight.turnOff   ();
        break;
      case DP_WARN_ALARM:
        // Force LCD back-light on in case it had been turned off...
  	    lcd.setBacklight ( LCD_BACKLIGHT_ON  );
      case DP_WARN:
        if ( dewState == DP_WARN_ALARM ) {
          switch ( audibleAlarmMode ) {
            case ALARM_SUSTAINED:
              indicatorBuzzer .blinkMax   ();
              enterButtonLight.blinkLevel ();
              break;
            case ALARM_LIMITED:
              indicatorBuzzer .installSequence ( &limitedAlarmSequenceWarn );
              enterButtonLight.installSequence ( &limitedAlarmSequenceWarn );
              indicatorBuzzer .startSequence ();
              enterButtonLight.startSequence ();
              break;
            case ALARM_NONE:
              indicatorBuzzer .turnOff ();
              enterButtonLight.turnOff ();
              break;
          }
        }
        else {
          indicatorBuzzer .turnOff ();
          enterButtonLight.turnOff ();
        }
        indicatorLedSafe.turnOff   ();
        indicatorLedWarn.turnHigh  ();
        indicatorLedDew .oscillate ();
        break;
      case DP_DEW_ALARM:
        // Force LCD back-light on in case it had been turned off...
	      lcd.setBacklight ( LCD_BACKLIGHT_ON  );
      case DP_DEW:
        if ( dewState == DP_DEW_ALARM ) {
          switch ( audibleAlarmMode ) {
            case ALARM_SUSTAINED:
              indicatorBuzzer .turnOn   ();
              enterButtonLight.turnHigh ();
              break;
            case ALARM_LIMITED:
              indicatorBuzzer .installSequence ( &limitedAlarmSequenceDew );
              enterButtonLight.installSequence ( &limitedAlarmSequenceDew );
              indicatorBuzzer .startSequence ();
              enterButtonLight.startSequence ();
              break;
            case ALARM_NONE:
              indicatorBuzzer .turnOff ();
              enterButtonLight.turnOff ();
              break;
          }
        }
        else {
          indicatorBuzzer .turnOff ();
          enterButtonLight.turnOff ();
        }
        indicatorLedSafe.turnOff    ();
        indicatorLedWarn.turnOff    ();
        indicatorLedDew .blinkLevel ();
        break;
    }
  }
 
  // Update all indicator LEDs and buzzer...
  powerButtonLight.updateNow ();
  indicatorLedSafe.updateNow ();
  indicatorLedWarn.updateNow ();
  indicatorLedDew .updateNow ();
  indicatorBuzzer .updateNow ();
  enterButtonLight.updateNow ();

  // Update display panel based on display refresh rate timer...
  if ( displayTimer.hasElapsed() ) {
    displayDewStatus (
      sensorTemperatureDegC,
      sensorHumidityPercent,
      dewPointTemperatureDegC,
      dewState,
      humIsTrigger,
      showTempAsF
    );
    displayTimer.start ();
  }

}

// ============================================================================
// Miscellaneous Indicator LED Functions:
// ============================================================================

void updateLightLevel () {

  uint16_t ldrValue;
  uint8_t  ldrZone;

  // Sense ambient light level and pass it through hysteresis filter...
  ldrValue = analogRead ( pinOfLdr );
  ldrZone = ambientLightLevelFilter.mapValueToZone ( ldrValue );

  // Update indicator LED level settings only if ambient light
  // level has changed...
  if ( (enumLightLevel) ldrZone != lightLevel ) {
    switch ( (enumLightLevel) ldrZone ) {
      case LIGHT_LOW:
        powerButtonLight.setLevelMax (  05 );
        indicatorLedSafe.setLevelMax (  15 );
        indicatorLedWarn.setLevelMax (  15 );
        indicatorLedDew. setLevelMax (  50 );
        enterButtonLight.setLevelMax (  50 );
        lightLevel = LIGHT_LOW;
        break;
      case LIGHT_MED:
        powerButtonLight.setLevelMax (  30 );
        enterButtonLight.setLevelMax (  50 );        
        indicatorLedSafe.setLevelMax ( 100 );
        indicatorLedWarn.setLevelMax ( 100 );
        indicatorLedDew. setLevelMax ( 125 );
        enterButtonLight.setLevelMax ( 125 );
        lightLevel = LIGHT_MED;
        break;
      case LIGHT_HIGH:
        powerButtonLight.setLevelMax ( 200 );
        indicatorLedSafe.setLevelMax ( 255 );
        indicatorLedWarn.setLevelMax ( 255 );
        indicatorLedDew. setLevelMax ( 255 );
        enterButtonLight.setLevelMax ( 255 );
        lightLevel = LIGHT_HIGH;
        break;
    }
  }

}

// ----------------------------------------------------------------------------

void runIndicatorLedTest () {

  indicatorLedSafe.turnOn  ();
  indicatorLedWarn.turnOn  ();
  indicatorLedDew .turnOn  ();

  if ( audibleAlarmMode != ALARM_NONE ) {
    delay ( 250 );
    indicatorBuzzer.turnOn  ();
    delay ( 250 );
    indicatorBuzzer.turnOff ();
  }
  
  delay ( 500 );

  indicatorLedSafe.turnOff ();
  delay ( 500 );
  indicatorLedSafe.turnOn  ();
  indicatorLedWarn.turnOff ();
  delay ( 500 );
  indicatorLedWarn.turnOn  ();
  indicatorLedDew. turnOff ();
  delay ( 500 );
  indicatorLedDew. turnOn  ();
  delay ( 500 );

  indicatorLedSafe.turnOff ();
  indicatorLedWarn.turnOff ();
  indicatorLedDew .turnOff ();
  delay ( 500 );

  indicatorLedDew .turnOn  ();
  delay ( 500 );
  indicatorLedWarn.turnOn  ();
  indicatorLedDew .turnOff ();
  delay ( 500 );
  indicatorLedSafe.turnOn  ();
  indicatorLedWarn.turnOff ();
  delay ( 500 );
  indicatorLedSafe.turnOff ();
  delay ( 1000 );

}

// ============================================================================
// LCD Panel Functions:
// ============================================================================

void initDisplay () {
 
  lcd.begin ( 16, 2 );                    // 16 columns, 2 rows
  lcd.setBacklight ( LCD_BACKLIGHT_ON );  // should be red back-light

}

// ----------------------------------------------------------------------------

void displayIntro () {

  lcd.clear ();
  lcd.setCursor ( 0, 0 );
  lcd.print ( DEW_ALERT_NAME_STRING    );
  lcd.setCursor ( 0, 1 );
  lcd.print ( DEW_ALERT_VERSION_STRING );

}

// ----------------------------------------------------------------------------

void displayReadyAndTestButton ( boolean autoTimeout ) {

  CwwElapseTimer timeoutTimer;

  lcd.clear ();
  lcd.setCursor ( 0, 0 );
  lcd.print ( "READY! Press" );
  lcd.setCursor ( 0, 1 );
  lcd.print ( "Enter to Start." );
  
  if ( autoTimeout ) timeoutTimer.start ( 60000L * 5 );  // five minutes

  while ( ! ( buttonEnter.isLowStable() || autoTimeout && timeoutTimer.hasElapsed() ) );
  delay ( 500 );

  // Instead of waiting for a button press, the timeout timer is another
  // way past the READY! prompt. The timeout timer is a safety in case the
  // battery is low, but a user has walked away and forgotten to pay
  // attention to the device. Since the code is not checking for a low-
  // battery condition while waiting for the button press, the battery
  // could run down and over-discharge were it not for a failsafe. The
  // timeout timer is this failsafe. The device will not indefinitely get
  // stuck at the READY! prompt. When the timer expires, the code will
  // advance to a state where battery conditions are checked and a low-
  // battery alarm will sound if the battery is in danger of discharging
  // too much.

}

// ----------------------------------------------------------------------------

void formatDewDisplay ( boolean showTempAsF ) {

  // Set up display with labels and units...
  lcd.clear ();
  lcd.setCursor (  0, 0 );
  lcd.print ( "T =" );
  lcd.setCursor (  8, 0 );
  lcd.print ( showTempAsF ? "F" : "C" );
  lcd.print ( " H=" );
  lcd.setCursor ( 15, 0 );
  lcd.print ( "%" );
  lcd.setCursor (  0, 1 );
  lcd.print ( "Td=" );
  lcd.setCursor (  8, 1 );
  lcd.print ( showTempAsF ? "F" : "C" );

}

// ----------------------------------------------------------------------------

void displayDewStatus (
  float        currTempDegC,
  float        currHumPercent,
  float        dewTempDegC,
  enumDewState dewState,
  boolean      humIsTrigger,
  boolean      showTempAsF
) {

  float currTemp;
  float dewTemp;

  char currTempText [6];  // for format: -##.# (sign may be + or -)
  char currHumText  [4];  // for format: ###   (always assumed positive)
  char dewTempText  [6];  // for format: -##.# (sign may be + or -)

  currTemp = showTempAsF ? tempCtoF(currTempDegC) : currTempDegC;
  dewTemp  = showTempAsF ? tempCtoF(dewTempDegC)  : dewTempDegC;

  dtostrf ( currTemp,       5, 1, currTempText );
  dtostrf ( currHumPercent, 3, 0, currHumText  );
  dtostrf ( dewTemp,        5, 1, dewTempText  );

  lcd.setCursor (  3, 0 );
  lcd.print ( currTempText );
  
  lcd.setCursor ( 12, 0 );
  lcd.print ( currHumText  );
  
  lcd.setCursor (  3, 1 );
  lcd.print ( dewTempText );
 
  lcd.setCursor ( 10, 1 );
  switch ( dewState ) {
    case DP_SAFE:
      lcd.print ( " " );
      break;
    case DP_NEAR:
    case DP_WARN:
    case DP_WARN_ALARM:
    case DP_DEW:
    case DP_DEW_ALARM:
      if ( humIsTrigger ) lcd.print ( "H-" );
      else                lcd.print ( "T-" );
      break;
  }
  switch ( dewState ) {
    case DP_SAFE:
      lcd.print ( "SAFE " );
      break;
    case DP_NEAR:
      lcd.print ( "NEAR" );
      break;
    case DP_WARN:
    case DP_WARN_ALARM:
      lcd.print ( "WARN" );
      break;
    case DP_DEW:
    case DP_DEW_ALARM:
      lcd.print ( "DEW!" );
      break;
  }
  
}

// ----------------------------------------------------------------------------

void displayLowBat () {

  lcd.clear ();
  lcd.setCursor ( 0, 0 );
  lcd.print ( "* BATTERY LOW *" );
  lcd.setCursor ( 0, 1 );
  lcd.print ( "Charge battery!" );

}

// ============================================================================
// Dew Point Calculation Functions:
// ============================================================================

const float TdpConstA = 6.112;  // Millibar
const float TdpConstB = 17.67;  // Unit-less
const float TdpConstC = 243.5;  // Degrees C

// ----------------------------------------------------------------------------

float tempOfDewPoint1 ( float relativeHumidityPercent, float airTemperatureDegC ) {

  // This function implements the simple dew point approximation
  // (Magnus formula) documented this Wikipedia article:
  // https://en.wikipedia.org/wiki/Dew_point

  float gamma;
  float tdp;

  gamma = log ( relativeHumidityPercent / 100.0 ) + 
          ( TdpConstB * airTemperatureDegC ) / ( TdpConstC + airTemperatureDegC );

  tdp = ( TdpConstC * gamma ) / ( TdpConstB - gamma );

  return tdp;

}

// ----------------------------------------------------------------------------

// Consider implementing and using more precise dew point model, call it
// tempOfDewPoint2(), as alternative to tempOfDewPoint1() above.

// ============================================================================
// Temperature Conversion Functions:
// ============================================================================

float relativeTempCtoF ( float tempC ) {

  // Relative Celsius to Fahrenheit conversion...
  return tempC * 9.0 / 5.0;
   
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

float tempCtoF ( float tempC ) {

  // Absolute Celsius to Fahrenheit conversion...
  return relativeTempCtoF ( tempC ) + 32.0;
   
}

// ----------------------------------------------------------------------------

float relativeTempFtoC ( float tempF ) {

  // Relative Fahrenheit to Celsius conversion...
  return tempF * 5.0 / 9.0;
   
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

float tempFtoC ( float tempF ) {

  // Absolute Fahrenheit to Celsius conversion...
  return relativeTempFtoC ( tempF - 32.0 );
   
}

// ============================================================================
// Configuration Functions:
// ============================================================================

void resetConfigToDefault () {

  dewConfigStruct dewConfig;

  lcd.clear ();
  lcd.setCursor ( 0, 0 );
  lcd.print ( "Factory reset..." );

  dewConfig.thTempMarginC[0] =  0.5;  // DEW/WARN  threshold: 0.5C 
  dewConfig.thTempMarginC[1] =  3.0;  // WARN/NEAR threshold: 3.0C
  dewConfig.thTempMarginC[2] =  5.0;  // NEAR/SAFE threshold: 5.0C 
  
  dewConfig.thHumPercent[0]  = 70.0;  // NEAR/SAFE threshold: 70%
  dewConfig.thHumPercent[1]  = 80.0;  // WARN/NEAR threshold: 80%
  dewConfig.thHumPercent[2]  = 95.0;  // DEW/WARN  threshold: 95%
  
  dewConfig.showTempAsF      = false;
  dewConfig.audibleAlarmMode = ALARM_SUSTAINED;
  
  EEPROM.put ( EEPROM_CONFIG_ADDRESS, dewConfig );

  // Insert a short delay to give the user the impression that
  // something happened...
  delay ( 750 );
  lcd.setCursor ( 0, 1 );
  lcd.print ( "Done." );

  delay ( 3000 );
  lcd.clear ();

}

// ----------------------------------------------------------------------------

void setConfigFromUser () {

  dewConfigStruct dewConfig;
  
  int8_t          configIndex;
  float           configValueMin;
  float           configValueMax;
  float           configValue;
  char            configValueStr [5];  // for format ##.#

  enumAlarmMode   configAlarmMode;

  boolean         buttonPressed;
  CwwElapseTimer  btnRepeatTimer ( 500 );  // Button auto-repeat delay of 500ms

  loadConfig ( &dewConfig );

  lcd.clear ();
  lcd.setCursor ( 0, 0 );
  lcd.print ( "* User Config *" );
  lcd.setCursor ( 0, 1 );
  lcd.print ( "Enter to Start." );
  while ( ! buttonEnter.isLowStable() );
  while ( ! buttonEnter.isHighStable() );

  lcd.clear ();
  lcd.setCursor ( 0, 0 );
  lcd.print ( "Show temperature" );
  do {
    lcd.setCursor ( 0, 1 );
    lcd.print ( dewConfig.showTempAsF ? "in Fahrenheit." : "in Celsius.   " );
    if ( buttonUp.isLowStable() || buttonDown.isLowStable() ) {
      dewConfig.showTempAsF = ! dewConfig.showTempAsF;
      while ( ! ( buttonUp.isHighStable() && buttonDown.isHighStable() ) );
    }
  } while ( ! buttonEnter.isLowStable() );
  while ( ! buttonEnter.isHighStable() );

  lcd.clear ();
  lcd.setCursor ( 0, 0 );
  lcd.print ( "Temp Margin Thr." );
  lcd.setCursor ( 15, 1 );
  lcd.print ( dewConfig.showTempAsF ? "F" : "C" );
  for ( configIndex = 0; configIndex <= 2; configIndex++ ) {
    lcd.setCursor ( 0, 1 );
    switch ( configIndex ) {
      case 0:
        lcd.print ( "WARN/DEW " );
        configValueMin = 0.0;
        configValueMax = 50.0 - 2.0 * HYSTERESIS_OFFSET_TEMP_MARGIN_C;
        break;
      case 1:
        lcd.print ( "NEAR/WARN" );
        configValueMin = dewConfig.thTempMarginC[0] + HYSTERESIS_OFFSET_TEMP_MARGIN_C;
        configValueMax = 50.0 - 1.0 * HYSTERESIS_OFFSET_TEMP_MARGIN_C;
        break;
      case 2:
        lcd.print ( "SAFE/NEAR" );
        configValueMin = dewConfig.thTempMarginC[1] + HYSTERESIS_OFFSET_TEMP_MARGIN_C;
        configValueMax = 50.0;
        break;
    }
    configValue = dewConfig.thTempMarginC[configIndex];
    if ( configValue < configValueMin ) configValue = configValueMin;
    if ( dewConfig.showTempAsF ) {
      configValue    = relativeTempCtoF ( configValue    );
      configValueMin = relativeTempCtoF ( configValueMin );
      configValueMax = relativeTempCtoF ( configValueMax );
    }
    buttonPressed = false;
    do {
      dtostrf ( configValue, 4, 1, configValueStr );  // format is ##.#
      lcd.setCursor ( 11, 1 );
      lcd.print ( configValueStr );
      if ( buttonPressed ) {
        btnRepeatTimer.start ();
        while ( ( buttonUp.isLow() || buttonDown.isLow() ) && ! btnRepeatTimer.hasElapsed() );
        buttonPressed = false;
      }
      if ( buttonUp.isLowStable()   ) {
        configValue += 0.1;
        if ( configValue > configValueMax ) configValue = configValueMax;
        buttonPressed = true;
      }
      if ( buttonDown.isLowStable() ) {
        configValue -= 0.1;
        if ( configValue < configValueMin ) configValue = configValueMin;
        buttonPressed = true;
      }
    } while ( ! buttonEnter.isLowStable() );
    if ( dewConfig.showTempAsF ) configValue = relativeTempFtoC ( configValue );
    dewConfig.thTempMarginC[configIndex] = configValue;
    while ( ! buttonEnter.isHighStable() );
  } 

  lcd.clear ();
  lcd.setCursor ( 0, 0 );
  lcd.print ( "Humidity Thresh." );
  lcd.setCursor ( 15, 1 );
  lcd.print ( "%" );
  for ( configIndex = 2; configIndex >= 0; configIndex-- ) {
    lcd.setCursor ( 0, 1 );
    switch ( configIndex ) {
      case 2:
        lcd.print ( "WARN/DEW " );
        configValueMin = 2.0 * HYSTERESIS_OFFSET_HUMIDITY_PERCENT;
        configValueMax = 100.0;
        break;
      case 1:
        lcd.print ( "NEAR/WARN" );
        configValueMin = 1.0 * HYSTERESIS_OFFSET_HUMIDITY_PERCENT;
        configValueMax = dewConfig.thHumPercent[2] - HYSTERESIS_OFFSET_HUMIDITY_PERCENT;
        break;
      case 0:
        lcd.print ( "SAFE/NEAR" );
        configValueMin = 0.0;
        configValueMax = dewConfig.thHumPercent[1] - HYSTERESIS_OFFSET_HUMIDITY_PERCENT;
        break;
    }
    configValue = dewConfig.thHumPercent[configIndex];
    if ( configValue > configValueMax ) configValue = configValueMax;
    buttonPressed = false;
    do {
      dtostrf ( configValue, 3, 0, configValueStr );  // format is ###
      lcd.setCursor ( 12, 1 );
      lcd.print ( configValueStr );
      if ( buttonPressed ) {
        btnRepeatTimer.start ();
        while ( ( buttonUp.isLow() || buttonDown.isLow() ) && ! btnRepeatTimer.hasElapsed() );
        buttonPressed = false;
      }
      if ( buttonUp.isLowStable()   ) {
        configValue += 1.0;
        if ( configValue > configValueMax ) configValue = configValueMax;
        buttonPressed = true;
      }
      if ( buttonDown.isLowStable() ) {
        configValue -= 1.0;
        if ( configValue < configValueMin ) configValue = configValueMin;
        buttonPressed = true;
      }
    } while ( ! buttonEnter.isLowStable() );
    dewConfig.thHumPercent[configIndex] = configValue;
    while ( ! buttonEnter.isHighStable() );
  } 

  lcd.clear ();
  lcd.setCursor ( 0, 0 );
  lcd.print ( "Audible alarm" );
  lcd.setCursor ( 0, 1 );
  lcd.print ( "mode:" );
  configAlarmMode = dewConfig.audibleAlarmMode;
  do {
    lcd.setCursor ( 6, 1 );
    switch ( configAlarmMode ) {
      case ALARM_SUSTAINED:
        lcd.print ( "sustained" );
        break;
      case ALARM_LIMITED:
        lcd.print ( "limited  " );
        break;
      case ALARM_NONE:
        lcd.print ( "none     " );
        break;
    }
    if ( buttonUp.isLowStable()   ) {
      if ( (int) configAlarmMode <= 2 ) configAlarmMode = (enumAlarmMode) ( (int) configAlarmMode + 1 );
      else                              configAlarmMode = (enumAlarmMode) 0;  // wrap-around
    }
    if ( buttonDown.isLowStable() ) {
      if ( (int) configAlarmMode >= 0 ) configAlarmMode = (enumAlarmMode) ( (int) configAlarmMode - 1 );
      else                              configAlarmMode = (enumAlarmMode) 2;  // wrap-around
    }
    while ( ! ( buttonUp.isHighStable() && buttonDown.isHighStable() ) );
  } while ( ! buttonEnter.isLowStable() );
  dewConfig.audibleAlarmMode = configAlarmMode;
  while ( ! buttonEnter.isHighStable() );

  EEPROM.put ( EEPROM_CONFIG_ADDRESS, dewConfig );

  lcd.clear ();
  lcd.setCursor ( 0, 0 );
  lcd.print ( "Configuration" );
  lcd.setCursor ( 0, 1 );
  lcd.print ( "saved." );

  delay ( 3000 );
  lcd.clear ();

}

// ----------------------------------------------------------------------------

void loadConfig ( dewConfigStruct * dewConfig ) {

  EEPROM.get ( EEPROM_CONFIG_ADDRESS, *dewConfig );
  
}

// ****************************************************************************
