 // ****************************************************************************

/*
 * DEW ALERT: Dew Point Detection and Warning System
 * -------------------------------------------------
 * Code by W.Witt; V1.00-beta-02; July 2016
 *
 * The code below implements a dew point detection and warming system
 * It senses temperature and relative humidity, uses that data to
 * calculate the dew point, and then provides an indication of how
 * close ambient conditions are to reaching the dew point. Based on the
 * temperature distance to the dew point (e.g. 1 deg-C away) it signals
 * that conditions are safe (i.e. far away from dew), the dew point is
 * near, the dew point is very close (warning condition) or the dew
 * point has been reached.
 * 
 * The intended application for this environment monitor is to track
 * when equipment (e.g. telescopes during a night-time observing
 * session) need to be protected before they're affected by moisture.
 *
 * The dew point state (SAFE, NEAR, WARN, DEW) is signaled via three
 * LEDs (green, yellow, red) and a buzzer. The detector includes an
 * ambient light sensor to adjust the brightness of the indicator LEDs
 * based on ambient light level, such that the LEDs will not be
 * disturbingly bright during night-time conditions.
 * 
 * Additionally or alternatively, a small LCD panel displays the
 * measured temperature and relative humidity values, the calculated
 * dew point and the dew point state.
 * 
 */

// ----------------------------------------------------------------------------

#define DEW_ALERT_VERSION_STRING "V1.00-beta-02"

// ****************************************************************************

// Standard incude files for I2C and SPI interfaces...
#include <Wire.h>
#include <SPI.h>

// Include files for Adafruit environmental sensors...
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// Include files for Adafruit RGB LCD shield...
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>

// Include file for advanced math functions (e.g. log)... 
#include <math.h>

// Inculde files for LED controller and hysteresis filter classes...
#include <CwwLedController.h>
#include <CwwFilterHysteresis.h>
#include <CwwElapseTimer.h>

// ----------------------------------------------------------------------------

// If multiple indicator LEDs need to pulse (fade/oscillate) with
// perfect synchronization, define the SYNC_LEDS flag to enable the
// handshake mechanism among the different LED controller instances...
// #define SYNC_LEDS

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
  DP_RESET,
  DP_SAFE,
  DP_NEAR,
  DP_WARN_ALARM,
  DP_WARN,
  DP_DEW_ALARM,
  DP_DEW
};

// ----------------------------------------------------------------------------

// Define Arduino pins for software-SPI interface to sensor...
const uint8_t pinOfBmeCS   = 10;   // SPI pin: slave select
const uint8_t pinOfBmeMOSI = 11;   // SPI pin: master out, slave in
const uint8_t pinOfBmeMISO = 12;   // SPI pin: master in, slave out
const uint8_t pinOfBmeSCK  = 13;   // SPI pin: clock
// TODO: In future, might move sensor to I2C to share pins with LCD panel.

// Instantiate bme280 temperature, humidity, pressure sensor... 
Adafruit_BME280 bme280 ( pinOfBmeCS, pinOfBmeMOSI, pinOfBmeMISO, pinOfBmeSCK );
// Sensor info: https://www.adafruit.com/product/2652

// Instantiate LCD contoller (I2C)...
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
// http://http://www.adafruit.com/products/714
// TODO: Migrate from LCD shield to simpler LCD with I2C backpack.

// ----------------------------------------------------------------------------

// Define pin of light sensor (light-dependent resistor, LDR) for ambient
// light measurement...
const uint8_t pinOfLdr = A0;
// Prototype was built with 300 to 200k ohm light LDR. Wiring is
//   Power---LDR---Resistor---Ground
// with pinOfLdr (analog input pin) connected between the LDR and the
// resistor, and with the value of the resistor (R) at about 10k ohm. If
// a different LDR is used, the resistor will likely need to be changed:
//    R = sqrt( LDR_max / LDR_min ) * LDR_min
// For a 200 to 300k ohm LDR, the optimal resistor is 7.7k ohm, but using
// 10k ohm because it's much easier to find and close enough.
// The range of values that may be read from the analog pin (after analog-
// to-digital conversion) depends on LDR and series resistor values.
// With LDR of 300 to 200k ohm and series resistor of 10k ohm, expected
// pin value range is 49 through 994.

// Instantiate LED controller objects ( pin, PWM-mode )...
CwwLedController indicatorLedSafe ( 6, true  );  // Assume green LED
CwwLedController indicatorLedWarn ( 5, true  );  // Assume yellow LED
CwwLedController indicatorLedDew  ( 3, true  );  // Assume red LED
// Using PWM pins for all to allow for dimming of indicator LEDs based
// on ambient light conditions.

// Use another LED controller object for a piezo buzzer...
CwwLedController indicatorBuzzer  ( 2, false );

#ifdef SYNC_LEDS
CWW_LC_SYNC_WORD ledHandshake;
#endif

// Define pin of momentary off(on) push button...
const uint8_t pinOfButton = 8;
// Button is assumed active low, wired as
//   Power---Resistor---Button---Ground
// with the pin connected between the resistor and button.
// Reasonable resistor value is between 1k and 10k ohm.

// ----------------------------------------------------------------------------

// Instantiate two hysteresis filters, one for the distance to dew and
// another for the ambient light...

// Temperature margin to dew point is used for assessinig dew risk;
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
// through to the indicators.

// ----------------------------------------------------------------------------

// State variable to hold current dew state...
enumDewState   dewState;

// State variable to track ambient light level...
enumLightLevel lightLevel;

// Timer to control how frequently to update LCD panel...
CwwElapseTimer displayTimer ( 1000 );  // display refresh rate of one second

// ============================================================================

// Code in setup() runs once after Arduino power-on or reset...
void setup () {

  // Initialize display...
  initDisplay ();

  // Set button's digital pin to input mode for reading...
  pinMode ( pinOfButton, INPUT );

  // Define periods for oscillating (fade-up/fade-down) or blinking
  // (on/off) indicator LEDs...
  indicatorLedWarn.setOscillatePeriod ( 1000 );  // one second
  indicatorLedDew. setOscillatePeriod ( 1000 );  // one second
  indicatorLedDew. setBlinkPeriod     (  500 );  // half a second

  // Set "blink", really beep, period of piezo buzzer indicator...
  indicatorBuzzer.setBlinkPeriod ( 500 );  // half a second

#ifdef SYNC_LEDS
  // If multiple LEDs are to oscillate together with perfect
  // synchronization is required, set up handshake link among
  // LED controller instances...
  indicatorLedSafe.attachSyncHandshake ( &ledHandshake, true );
  indicatorLedWarn.attachSyncHandshake ( &ledHandshake );
  indicatorLedDew. attachSyncHandshake ( &ledHandshake );
  indicatorLedSafe.initSyncHandshake ();
  indicatorLedWarn.initSyncHandshake ();
  indicatorLedDew. initSyncHandshake ();
#endif

  // Define the three temperature margin thresholds for the dew
  // point hysteresis filter (measurement unit for input value is
  // degrees C)...
  dewPointMarginFilter.defineThreshold ( 0, 0.5, 0.1 );  // DEW/WARN threshold:  0.5C +/- 0.1C
  dewPointMarginFilter.defineThreshold ( 1, 3.0, 0.1 );  // WARN/NEAR threshold: 3.0C +/- 0.1C
  dewPointMarginFilter.defineThreshold ( 2, 5.0, 0.1 );  // NEAR/SAFE threshold: 5.0C +/- 0.1C
  dewPointMarginFilter.initHistory ( 10.0 );  // arbitrarily start filter assuming 10C to dew point

  // Similarly, define three relative humidity thresholds for the
  // alternate dew point hysteresis filter (measurement unit for
  // input value is relative humidity percent)...
  humidityFilter.defineThreshold ( 0, 65.0, 1.0 );  // DEW/WARN threshold:  65% +/- 1%
  humidityFilter.defineThreshold ( 1, 80.0, 1.0 );  // WARN/NEAR threshold: 80% +/- 1%
  humidityFilter.defineThreshold ( 2, 95.0, 1.0 );  // NEAR/SAFE threshold: 95% +/- 1%
  humidityFilter.initHistory ( 25.0 );  // arbitrarily start filter assuming 25% humidity

  // Define the two thresholds for the ambient light hysteresis 
  // filter (measurement unit for input value is 0-1023 analogRead()
  // result, i.e. 10-bit A-to-D converter)...
  ambientLightLevelFilter.defineThreshold ( 0, 500, 25 );  // LOW/MED threshold:  500 +/- 25
  ambientLightLevelFilter.defineThreshold ( 1, 800, 50 );  // MED/HIGH threshold: 800 +/- 50
  ambientLightLevelFilter.initHistory ( 0 );  // arbitrarily start filter asuming low light

  // Initialize ambient light level state (start at reset state for no
  // assumption about the initial lighting state)...
  lightLevel = LIGHT_RESET;
  updateLightLevel ();

  // Run indicator lights and buzzer through a quick test to demonstrate
  // they all work...
  runIndicatorLedTest ();

  // Activate communication with primary environmental (temperature,
  // humidity) sensor...
  bme280.begin ();

  // Get display ready to show temperature, humidity and dew status...
  formatDewDisplay ();
  displayTimer.start ();

  // Start dew tracking state in reset state (i.e. real state is unknown)...
  dewState = DP_RESET;

}

// ----------------------------------------------------------------------------

// Code in loop() keeps running after setup...
void loop () {

  float        sensorTemperatureDegC;
  float        sensorHumidityPercent;
  float        dewPointTemperatureDegC;
  float        marginToDewDegC;
  uint8_t      marginToDewQuantized;
  uint8_t      marginToH100Quantized;
  uint8_t      effectiveMargin;
  boolean      humIsTrigger;
  enumDewState dewStateNext;

  // Checkc ambient light level and adapt indicator LEDs accordingly...
  updateLightLevel ();

  // Read primary environmental sensor...
  sensorTemperatureDegC = bme280.readTemperature ();
  sensorHumidityPercent = bme280.readHumidity ();

  // Calculate dew point temperature from current temperature and
  // relative humidity values...
  dewPointTemperatureDegC = tempOfDewPoint1 ( sensorHumidityPercent, sensorTemperatureDegC );
  // Calculate margin from current temperature to dew-point temperature...
  marginToDewDegC = sensorTemperatureDegC - dewPointTemperatureDegC;

  // Assess dew risk with two parallel methods:
  // 1. temperature margin to dew point
  // 2. margin to 100% relative humidity
  // Note that dew may form on items long before the temperature
  // reaches the dew point and relative humidity reaches 100%.

  // Run temperature margin through hysteresis filter; map margin value
  // to detection zone...
  marginToDewQuantized = dewPointMarginFilter.mapValueToZone ( marginToDewDegC );
  // Similarly, run relative humidity through hysteresis filter...
  marginToH100Quantized = humidityFilter.zoneCount() -
                          humidityFilter.mapValueToZone ( sensorHumidityPercent );

  // Take the worst case (smallest margin) of the two...
  effectiveMargin = min ( marginToDewQuantized, marginToH100Quantized );
  humIsTrigger = marginToH100Quantized  < marginToDewQuantized;

  // Map zone to new dew state...
  switch ( effectiveMargin ) {
    case 0:  // zone below dew point; there's dew now
      // If this zone was newly reached from another, non-DEW zone,
      // go to alarm sub-state first; otherwise, just stay in DEW...
      if ( dewState != DP_DEW  ) {
        if ( digitalRead ( pinOfButton ) == LOW ) dewStateNext = DP_DEW;
        else                                      dewStateNext = DP_DEW_ALARM;
      }
      else {
        dewStateNext = DP_DEW;
      }
      break;
    case 1:  // zone just before dew point; time to prepare
      // If this zone was was reached from NEAR or SAFE, then
      // start with alarm first...
      if ( dewState != DP_WARN ) {
        if ( digitalRead ( pinOfButton ) == LOW ) dewStateNext = DP_WARN;
        else                                      dewStateNext = DP_WARN_ALARM;
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
        indicatorLedSafe.turnHigh ();
        indicatorLedWarn.turnOff  ();
        indicatorLedDew. turnOff  ();
        break;
      case DP_NEAR:
        indicatorLedSafe.turnOff   ();
        indicatorLedWarn.oscillate ();
        indicatorLedDew. turnOff   ();
        break;
      case DP_WARN_ALARM:
      case DP_WARN:
        if ( dewState == DP_WARN_ALARM ) indicatorBuzzer.blink   ();
        else                             indicatorBuzzer.turnOff ();
        indicatorLedSafe.turnOff   ();
        indicatorLedWarn.turnHigh  ();
        indicatorLedDew. oscillate ();
        break;
      case DP_DEW_ALARM:
      case DP_DEW:
        if ( dewState == DP_DEW_ALARM ) indicatorBuzzer.turnOn  ();
        else                            indicatorBuzzer.turnOff ();
        indicatorLedSafe.turnOff ();
        indicatorLedWarn.turnOff ();
        indicatorLedDew. blink   ();
        break;
    }
  }
 
  // Update all indicator LEDs and buzzer...
  indicatorLedSafe.updateNow ();
  indicatorLedWarn.updateNow ();
  indicatorLedDew. updateNow ();
  indicatorBuzzer. updateNow ();

  // Update LCD panel...
  if ( displayTimer.hasElapsed() ) {
    displayDewStatus (
      sensorTemperatureDegC,
      sensorHumidityPercent,
      dewPointTemperatureDegC,
      dewState,
      humIsTrigger
    );
    displayTimer.restart ();
  }

  // Sleep a bit before next sense/update iteration...
  delay ( 10 );

}

// ============================================================================
// Miscellaneous Indicator LED Functions:
// ============================================================================

void updateLightLevel () {

  uint16_t ldrValue;
  uint8_t  ldrZone;

  // Sense ambient light level and run through hysteresis filter...
  ldrValue = analogRead ( pinOfLdr );
  ldrZone = ambientLightLevelFilter.mapValueToZone ( ldrValue );

  // Update indicator LED level settings only if ambient light
  // level has changed...
  if ( (enumLightLevel) ldrZone != lightLevel ) {
    switch ( (enumLightLevel) ldrZone ) {
      case LIGHT_LOW:
        indicatorLedSafe.setLevelMax (  20 );
        indicatorLedWarn.setLevelMax (  20 );
        indicatorLedDew. setLevelMax (  75 );
        lightLevel = LIGHT_LOW;
        break;
      case LIGHT_MED:
        indicatorLedSafe.setLevelMax ( 100 );
        indicatorLedWarn.setLevelMax ( 100 );
        indicatorLedDew. setLevelMax ( 125 );
        lightLevel = LIGHT_MED;
        break;
      case LIGHT_HIGH:
        indicatorLedSafe.setLevelMax ( 255 );
        indicatorLedWarn.setLevelMax ( 255 );
        indicatorLedDew. setLevelMax ( 255 );
        lightLevel = LIGHT_HIGH;
        break;
    }
  }

}

// ----------------------------------------------------------------------------

void runIndicatorLedTest () {

  indicatorLedSafe.turnOn  ();
  indicatorLedWarn.turnOn  ();
  indicatorLedDew. turnOn  ();
  delay ( 250 );
  indicatorBuzzer. turnOn  ();
  delay ( 250 );
  indicatorBuzzer. turnOff ();
  delay ( 500 );

  indicatorLedSafe.turnOff ();
  delay ( 500 );
  indicatorLedSafe.turnOn  ();
  indicatorLedWarn.turnOff ();
  delay ( 500 );
  indicatorLedWarn.turnOn  ();
  indicatorLedDew. turnOff ();
  delay ( 500 );

  indicatorLedSafe.turnOff ();
  indicatorLedWarn.turnOff ();
  indicatorLedDew. turnOff ();
  delay ( 500 );

  indicatorLedSafe.turnOff ();
  indicatorLedWarn.turnOff ();
  indicatorLedDew. turnOn  ();
  delay ( 500 );
  indicatorLedWarn.turnOn  ();
  indicatorLedDew. turnOff ();
  delay ( 500 );
  indicatorLedSafe.turnOn  ();
  indicatorLedWarn.turnOff ();
  delay ( 500 );

  indicatorLedSafe.turnOff ();
  indicatorLedWarn.turnOff ();
  indicatorLedDew. turnOff ();
  delay ( 1000 );

}

// ============================================================================
// LCD Panel Functions:
// ============================================================================

void initDisplay () {
 
  lcd.begin ( 16, 2 );       // 16 columns, 2 rows
  lcd.setBacklight ( 0x1 );  // red backlight
 
  lcd.setCursor ( 0, 0 );
  lcd.print ( "* Dew Alert *" );
  lcd.setCursor ( 0, 1 );
  lcd.print ( DEW_ALERT_VERSION_STRING );

}

// ----------------------------------------------------------------------------

void formatDewDisplay () {

  // Set up display with labels and units...
  lcd.clear ();
  lcd.setCursor (  0, 0 );
  lcd.print ( "T =" );
  lcd.setCursor (  8, 0 );
  lcd.print ( "C H=" );
  lcd.setCursor ( 15, 0 );
  lcd.print ( "%" );
  lcd.setCursor (  0, 1 );
  lcd.print ( "Td=" );
  lcd.setCursor (  8, 1 );
  lcd.print ( "C" );

}

// ----------------------------------------------------------------------------

void displayDewStatus (
  float        currTempDegC,
  float        currHumPercent,
  float        dewTempDegC,
  enumDewState dewState,
  boolean      humIsTrigger
) {

  char currTempText[6];  // for format: -##.# (sign may be + or -)
  char currHumText[4];   // for format: ###   (always assumed positive)
  char dewTempText[6];   // for format: -##.# (sign may be + or -)

  dtostrf ( currTempDegC,   5, 1, currTempText );
  dtostrf ( currHumPercent, 3, 0, currHumText  );
  dtostrf ( dewTempDegC,    5, 1, dewTempText  );

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
      lcd.print ( "SAFE" );
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

// ============================================================================
// Dew Point Calculation Functions:
// ============================================================================

const float TdpConstA = 6.112;  // Millibar
const float TdpConstB = 17.67;  // Unit-less
const float TdpConstC = 243.5;  // Degrees C

// ----------------------------------------------------------------------------

float tempOfDewPoint1 ( float relativeHumidityPercent, float airTemperatureDegC ) {

  // This funcion implements the simple dew point approximation
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

// ****************************************************************************

