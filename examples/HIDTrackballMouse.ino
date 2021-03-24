/**********************************************************************
* Use a seeeduino XIAO or any Arduino-compatible microcontroller that
* supports the TinyUSB stack and a Pimoroni Trackball breakout as a 
* USB mouse. 
#include <Adafruit_TinyUSB.h>

// HID report descriptor using TinyUSB's template
// Single Report (no ID) descriptor
uint8_t const desc_hid_report[] =
{
  TUD_HID_REPORT_DESC_MOUSE()
};

// USB HID object
Adafruit_USBD_HID usb_hid;

#include <Pimoroni_Trackball.h>

Pimoroni_Trackball trackball;

void setup() {
  usb_hid.setPollInterval(2);
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  //usb_hid.setStringDescriptor("TinyUSB Mouse");

  usb_hid.begin();

  Serial.begin(115200);
  // put your setup code here, to run once:
  trackball.begin();
  delay(500);
  trackball.setLEDs(0x33333333);
  delay(500);

  // wait until device mounted
  while( !USBDevice.mounted() ) delay(1);

}

void loop()
{
  // get relative motion edge counts from hall sensors on trackball breakout
  int8_t rawx = 0, rawy = 0;
  trackball.getMotion(rawx, rawy);

  // get relative button state, 1 for pressed, 0 for no change and -1 for released
  static int8_t button = 0;
  static int8_t last_button = 0;
  int8_t button_change = trackball.getButton();
  if(button_change > 0)
    button = 1;
  else if(button_change < 0)
    button = 0;
  
  // convert raw x and y transition counts to float
  float fx = rawx;
  float fy = rawy;

  // apply exponential smoothing to x and y independently
  static float sx = 0, sy = 0;
  static const float smooth = 0.075;
  
  sx = fx*smooth + (1.0-smooth)*sx;
  sy = fy*smooth + (1.0-smooth)*sy;

  // acceleration is based on magnitude of motion vector. 
  float flen = sqrt(fx*fx + fy*fy);

  // apply asymmetric exponential smoothing (aka envelope follower 
  // with differing rise and fall rates, for the synthesiser fans)
  // to the magnitude
  static const float slenrise = 0.05, slenfall = 0.0125;
  static float slen = 0;
  if(flen < slen)
    slen = flen * slenfall + (1.0 - slenfall) * slen;
  else
    slen = flen * slenrise + (1.0 - slenrise) * slen;

  // scale and clip acceleration factor to taste, then multiply by the smoothed x and y, 
  // so our acceleration is simply a modified, distorted exponential curve with order two 
  static const float scalefact = 10.5;
  static const float scalemin = 1.0;
  float scaledx = sx * max(scalemin, slen * scalefact);
  float scaledy = sy * max(scalemin, slen * scalefact);

  // add any rounding error from the previous report to this one, before rounding. 
  // this way subpixel movement is still reported as intermittent single pixel moves,
  // and motion in general feels much more natural and easy to control with the ball
  static float residx = 0;
  static float residy = 0;
  scaledx = scaledx - residx;
  scaledy = scaledy - residy;
  residx = (float)round(scaledx) - scaledx;
  residy = (float)round(scaledy) - scaledy;
  int x = round(scaledx);
  int y = round(scaledy);

  // make a report to the usb host only if there was any motion on button activity
  if(button != last_button || x != 0 || y != 0)
  {
    last_button = button;
    /*
    Serial.print("button = ");
    Serial.print(button);
    Serial.print(" x = ");
    Serial.print(x);
    Serial.print(" y = ");
    Serial.print(y);
    Serial.print("\n");
		*/
		
    // Remote wakeup
    if ( USBDevice.suspended() )
    {
      // Wake up host if we are in suspend mode
      // and REMOTE_WAKEUP feature is enabled by host
      USBDevice.remoteWakeup();
    }
  
    if ( usb_hid.ready() )
    {
      usb_hid.mouseReport(0,button?1:0,-x,y,/*wheel*/0,0);
    }

  }
  delay(1);
}
