/*
 *  solar_metric_clock: Clock that displays the current time in several formats.
 *
 *  This clock uses NTP over WiFi to get the current time of day. It sets a real-time clock
 *  (rtc, a DS3231 module) for accuracy, and resynchronizes the clock periodically to NTP
 *  to maintain accuracy.
 *
 *  WiFi network SSID and password are hard-coded in the Arduino "Secret" tab.
 *
 *  The times it displays are:
 I
 *    Local time: The usual hh:mm format, accounting for daylight saving time changes and time
 *    zone. Time zone and DST change rules are hard-coded in this file.
 *
 *    Local metric time: the number of hundred-microday (10,000ths of a day) intervals
 *    elapsed since midnight, and displays that time in nnnn format. Local metric time is
 *    computed directly from local hh:mm time, so also accounts for DST changes.
 *
 *    Solar metric time: I use a fast Equation of Time (eot) calculation to compute the actual
 *    solar time to within thirteen seconds. Solar time is noon when the sun is at its apogee,
 *    relative to your position on the planet. Solar noon needs to account for your longitude and 
 *    for the position of the Earth on its elliptical orbit around the sun. The clock displays the
 *    current metric solar time in nnnn format. If you're a reasonable distance from the Prime
 *    Meridian, solar noon will consistently lag or consistently lead UTC noon, but that lag
 *    or lead will vary over the course of the year. At the meridian, you'll sometimes lag,
 *    sometimes lead. Solar metric time doesn't care about DST.
 *
 *    Longitude is hard-coded in the Arduino "Secret" tab.
 *
 *  You can think of the metric time displays as "percent of day elapsed."
 *
 *  For the metric times it displays, the clock also turns on the decimal point following
 *  the last digit if the time is a prime number.
 *
 *  Licensed under the 3-clause BSD license:
 *
 *  Copyright 2023 Michael A. Olson
 *
 *  Redistribution and use in source and binary forms, with or without modification, are permitted
 *  provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this list of conditions
 *     and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice, this list of
 *     conditions and the following disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its contributors may be used to
 *     endorse or promote products derived from this software without specific prior written
 *     permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
 *  OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 *  AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdlib.h>
#include <math.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFiNINA_Generic.h>
#include <TimeLib.h>
#include <Timezone_Generic.h>
#include <DS3231.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

//===================== Code and variables for displaying prime time ===========================
int PrimeList[] =
{
	2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47,
  53, 59, 61, 67, 71, 73, 79, 83, 89, 97, 101
};
int NPrimes = 26;

/*
 * prime_time -- Is the supplied metric time prime?
 *
 *  Determines whether the time supplied is a prime number. This is in response to a
 *  feature request from Doug Cutting.
 *
 *  We try to divide the supplied time by all primes up to 101, which is just over the
 *  square root of 10,000, the maximum time that can be displayed. If a number is divisible
 *  by any divisor d, then it is also divisble by (number/d). The largest divisor we need
 *  to check is thus the square root of the number; it must also have a divisor less than
 *  that divisor.
 */

bool
prime_time(int metric_time)
{
  int i;

  if (metric_time <= 1)
    return (false);

  for (i = 0; i < NPrimes; i++)
  {
    if (metric_time == PrimeList[i])
      return (true);
  
    if (metric_time % PrimeList[i] == 0)
      return (false);
  }

  return (true);
}

//===================== Code and variables for 7-seg matrix displays ===========================

/*
 * We use three four-digit seven-segment LED displays to show the time. One displays local time
 * in conventional hh:mm format. One converts that local time, accounting for time zones and DST,
 * to metric time (hundred-thousandths of a day since midnight). The third dipslays metric time
 * according to solar time, calculating a correct "midnight" for actual solar position based on
 * date and longitude.
 */

Adafruit_7segment hhmm_matrix = Adafruit_7segment();
Adafruit_7segment local_matrix = Adafruit_7segment();
Adafruit_7segment solar_matrix = Adafruit_7segment();
int MatrixBright = 6;  // 0 = dimmest, 15 = brightest

// We attach a pushbutton to Arduino digital pin 2 to allow the user to adjust brightness
#define BRIGHTNESS_BUTTON_PIN 2   // pinMode is INPUT_PULLUP, button wired to GND
#define BUTTON_DEBOUNCE_DELAY 10  // 10ms delay seems to do the trick
int lastButtonState = HIGH;       // off

// Adjust brightness level. We cycle through 6 levels, 15/12/9/6/3/0
void
matrix_adjust_brightness()
{
  MatrixBright -= 3;
  
  if (MatrixBright < 0)
    MatrixBright = 15;

  hhmm_matrix.setBrightness(MatrixBright);
  local_matrix.setBrightness(MatrixBright);
  solar_matrix.setBrightness(MatrixBright);
}

// blank_displays() -- Whenever we're trying to acquire a time sync, we want the displays blanked out
void
blank_displays()
{
  hhmm_matrix.print("----");
  hhmm_matrix.writeDisplay();
  
  local_matrix.print("----");
  local_matrix.writeDisplay();
  
  solar_matrix.print("----");
  solar_matrix.writeDisplay();
}

// display_time: Display the four-digit time supplied on one of the 7-segment LED displays
void
display_time(Adafruit_7segment display, uint16_t t, bool drawColon, bool isPrime)
{
  display.writeDigitNum(0, (t / 1000), false);
  display.writeDigitNum(1, (t / 100) % 10, false);
  display.drawColon(drawColon);
  display.writeDigitNum(3, (t / 10) % 10, false);
  display.writeDigitNum(4, t % 10, isPrime);

  display.writeDisplay();
}

// matrix_init() -- initialize the three 7-segment LED matrix displays
void
matrix_init()
{
  hhmm_matrix.begin(0x70);
  hhmm_matrix.setBrightness(MatrixBright);
  local_matrix.begin(0x71);
  local_matrix.setBrightness(MatrixBright);
  solar_matrix.begin(0x72);
  solar_matrix.setBrightness(MatrixBright);
  
  // Button is wired to ground, so when it's pressed, pin state will be low
  pinMode(BRIGHTNESS_BUTTON_PIN, INPUT_PULLUP);
}

//===================== Variables for time zone management ===============================

// XXX SET DST RULES
// US Pacific. Change for your time zone.
TimeChangeRule pdt = {"PDT", Second, Sun, Mar, 2, -420};  // Daylight time = UTC - 7 hours
TimeChangeRule pst = {"PST", First, Sun, Nov, 2, -480};   // Standard time = UTC - 8 hours

// US Central.
// TimeChangeRule cdt = {"CDT", Second, Sun, Mar, 2, -300};  // Daylight time = UTC - 5 hours
// TimeChangeRule cst = {"CST", First, Sun, Nov, 2, -360};   // Standard time = UTC - 6 hours

// UK
// TimeChangeRule ukdt = {"UKDT", Last, Sun, Mar, 1, 60}; // Daylight time = UTC + 1 hour
// TimeChangeRule ukst = {"UKST", Last, Sun, Oct, 2, 0}; // Standard time = UTC

// Western Europe
// TimeChangeRule cedt = {"CEDT", Last, Sun, Mar, 1, 120}; // Daylight time = UTC + 2 hour
// TimeChangeRule cest = {"CEST", Last, Sun, Oct, 2, 60}; // Standard time = UCT + 1 hour

// XXX SET TIMEZONE
//Timezone
Timezone tz(pdt, pst);
// Timezone tz(cdt, cst);
// Timezone tz(ukdt, ukst);
// Timezone tz(cedt, cest);

//===================== Code and variables for real-time clock ===========================

// DS3231
DS3231 myRTC;

/*
 *  The DS3231 I have is accurate to 2 parts per million, or one part per five hundred thousand. I
 *  break a metric day into 10,000 100-microday units, so 500,000 / 10,000 = 50 days before I am
 *  at risk of an error in the lowest digit.
 */

// Number of ticks before we have to resync RTC to NTP. Depends on your RTC model.
// Sept 2023: Clock drifts faster than specified. Also, you notice the drift of several seconds even
// though the metric and hh:mm times turn over more slowly than once per second, because everyone has
// an accurate-to-the-second clock in their pockets these days. I arbitrarily set the sync interval to
// every five days, instead of the 50 days I originally used. This has been satisfyingly accurate on my
// desktop SMC.
// #define RTC_SYNC_INTERVAL 500000L
#define RTC_SYNC_INTERVAL 50000L
long long SyncCounter = 0L;       // Count ticks here

// myRTC_init() -- initialize the important stuff on the RTC at setup time
void
myRTC_init()
{
  myRTC.setClockMode(false); // 24-hour mode
}

//===================== Code and variables for network time sync ===========================

// XXX SET SSID AND WIFI PASSWORD
#define SECRET_CLOCK_SSID "ENTER YOUR SSID HERE"
char CLOCK_SSID[] = SECRET_CLOCK_SSID;
#define SECRET_CLOCK_PASS "ENTER YOUR WIFI PASSWORD HERE"
char CLOCK_PASSWORD[] = SECRET_CLOCK_PASS;

#define UDP_LOCAL 2390              // local port to listen for UDP packets
#define NTP_PACKET_SIZE 48          // NTP time stamp is in the first 48 bytes of the message
#define NTP_PORT_NUMBER 123         // NTP requests go to port 123
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

IPAddress timeServer(132, 163, 96, 1); // time-a-b.nist.gov NTP server
// IPAddress timeServer(169, 229, 128, 134); // ntp1.net.berkeley.edu NTP server
// IPAddress timeServer(162, 159, 200, 1); // time.cloudflare.com NTP server
// IPAddress timeServer(216, 239, 35, 0); // time1.google.com
// IPAddress timeServer(131, 111, 8, 171); // ntp1.csx.cam.ac.uk
// IPAddress timeServer(139, 143, 5, 30); // ntp1.npl.co.uk UK Nat'l Physics Laboratory NTP server
// IPAddress timeServer(87, 117, 251, 3); // 0.uk.pool.ntp.org

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;

// send an NTP request to the time server at the given IP address
void
sendNTPpacket(IPAddress& address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);

  // Build the NTP request packet
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;            // Stratum, or type of clock
  packetBuffer[2] = 6;            // Polling Interval
  packetBuffer[3] = 0xEC;         // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // Send request
  Udp.beginPacket(address, NTP_PORT_NUMBER);
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

// SetTimeFromNetwork() -- Set the real-time clock from the NTP server.
void
SetTimeFromNetwork()
{
  int i, status;
  time_t currentTime = 0L;

  // blank the displays anytime we come in here to sync the clock
  blank_displays();

  // Connect to the local area network WiFi access point
  status = WL_IDLE_STATUS;
  
  while (status != WL_CONNECTED)
  {
    // Serial.println(F("try to connect"));
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(CLOCK_SSID, CLOCK_PASSWORD);
    Serial.println("Connecting...");
    
    // if the connection failed, sleep for a little bit, then try again
    delay(1000);
  }
  
  // Serial.println(F("Connected to wifi"));
  
  // Send an ntp request to the ntpd server
  Udp.begin(UDP_LOCAL);
  sendNTPpacket(timeServer);
  
  // Hang around waiting for a reply
  for (i = 0; i < 1000 && !Udp.parsePacket(); i++)
  {
    delay(1);
  }

  if (i < 1000)
  {
    Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    // Parse the packet. Timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long.
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    unsigned long secsSince1900 = highWord << 16 | lowWord;

    // NTP time is seconds since 1/1/1900. Unix time is seconds since 1/1/1970, so we need
    // to subtract 70 years from NTP time to get Unix time.
    const unsigned long seventyYears = 2208988800UL;  // Seconds from 1/1/1900 to 1/1/1970
    currentTime = (long long) (secsSince1900 - seventyYears);

    // set the RTC time
    myRTC.setEpoch(currentTime, false);
  }
  
  // don't need the wifi connection anymore
  WiFi.disconnect();
  WiFi.end();
  // Serial.println(F("Time set from network."));
}

//===================== Code and variables for Equation of Time calculation ===========================

/*
 * days_since_y2k(year, month, day, hour):
 *
 *	Integer number of days since Jan 1, 2000
 *
 *	A century has 36,500 days (plus leap days), so this number needs
 *	to be stored in a 32-1bit integer type.
 *
 *	Input year needs to be between 2000 and 2099, but we don't check
 *	input values for reasonableness.
 */

long long
days_since_y2k(int year, int month, int day, int hour)
{
	long long a, b, c, d;   // Quantity c in particular can overflow a 16-bit integer on Arduino
	long long dy2k;

	if (month <= 2)
	{
		year -= 1;
		month += 12;
	}

	a = (long long) year / 100L;
	b = 2L - a + (a / 4L);
	c = (long long) (365.25 * (float) year);
	d = (long long) (30.6001 * ((float) month + 1.0));
	
	dy2k = (long long) ((float) (b + c + d + day) + ((float) hour / 24.0) - 730550.5);

	return (dy2k);
}

/*
 * equation_of_time(year, month, day, hour):
 *
 *	Calculate the adjustment, in minutes, of the actual solar time
 *	at the UTC meridian on the date and hour supplied. Minutes are
 *	computed as a double-precision floating point number, so will
 *	need to be converted to seconds for civil time math.
 *
 *	Solar time is computed from the actual position of the sun in
 *	the sky. Civil time assumes an invariant 24-hour day. The Earth's
 *	orbit is elliptical, and the equatorial plane is slanted relative
 *	to the planet's orbit, so over the course of the year, noon civil
 *	time drifts relative to solar time.
 *
 *	There are other sources of inaccuracy between observed noon and
 *	civil time, including atmospheric refraction that moves the
 *	apparent sun a little bit, small variations in actual length of
 *	day due to mountains slamming into high-pressure blocks of air,
 *	slowing of the Earth's rotation over time, and so on. These are
 *	all negligible for our purposes here.
 *
 *	Researchers at NASA/JPL used a large series of actual observations
 *	of the sun's position in the sky, compared to civil time, and
 *	used linear regression to create a function that approximates
 *	actual solar position from civil time based on the major components
 *	of the curve of the observed data. This technique is called
 *	"Fourier analysis," so the website calls this the Fourier method.
 *
 *	Background and the Visual Basic code are at
 *
 *		https://equation-of-time.info/calculating-the-equation-of-time
 *
 *	This function is accurate to +/-13 seconds for any hour between
 *	2000 and 2099.  It's a cheap function to calculate to get that
 *	degree of accuracy.
 */

double
equation_of_time(int year, int month, int day, int hour)
{
	long long dy2k;
	double cycle, theta, eot1, eot2, eot3, eot4;
	double eot;

	dy2k = days_since_y2k(year, month, day, hour);
	cycle = 4.0 * (double) dy2k;
	cycle -= ((double) ((int) (cycle / 1461.0))) * 1461.0;
	theta = cycle * 0.004301;
	eot1 = 7.353 * sin(1.0 * theta + 6.209);
	eot2 = 9.927 * sin(2.0 * theta + 0.37);
	eot3 = 0.337 * sin(3.0 * theta + 0.304);
	eot4 = 0.232 * sin(4.0 * theta + 0.715);

	eot = 0.019 + eot1 + eot2 + eot3 + eot4;

	return (eot);
}

/*
 * eot_adjust_seconds(year, month, day, hour, longitude):
 *
 *	Return the adjustment, in seconds, required to get solar time
 *	from UTC time.
 *
 *	The functions above all assume we're at the meridian. This
 *	function computes the seconds of adjustment required there, and
 *	then further adjusts by considering the time difference based
 *	on actual longitude, supplied in degrees (negative is west of
 *	the meridian).
 *
 *	We handle this quantity as double-precision in order to avoid
 *	overflowing sixteen-bit integer types on some architectures
 *	(hello, Arduino!).
 */

double
eot_adjust_seconds(int year, int month, int day, int hour, float longitude)
{
	double utc_adj, local_adj;

	/* equation of time function returns minutes, we want seconds */
	utc_adj = equation_of_time(year, month, day, hour) * 60.0;

	/* one degree of longitude is 4 minutes, or 240 seconds, of time */
	/* sign of eot matters -- we use this to adjust civil to solar, so must invert */
	local_adj = -utc_adj + ((double) longitude * 240.0);

	return (local_adj);
}

//===================================== Main program ===================================== 

// XXX SET LONGITUDE
// Enter sensitive data in the Secret tab/arduino_secrets.h
// This is the longitude where you have the clock.
// Sign matters. Negative is west of the prime meridian.
#define SECRET_LONGITUDE "ENTER YOUR LONGITUDE HERE"
char MY_LONGITUDE[] = SECRET_LONGITUDE;

void
setup()
{
  Serial.begin(9600);
  Wire.begin();
  
  matrix_init();
  myRTC_init();

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE)
  {
    Serial.println(F("Communication with WiFi module failed!"));
    // don't continue
    while (true);
  }
  Serial.println("Gonna set time from network");

  SetTimeFromNetwork();
}

// keep track of the time displayed on each led matrix, and only update when necessary
uint16_t local_metric_save = 0;
uint16_t solar_metric_save = 0;
uint16_t hhmm_save = 0;

void loop()
{
  DateTime dt;
  time_t utc, local, solar;
  float secs_since_midnight; // 86,400 is too many for the 16-bit int types on the Arduino Uno
  uint16_t local_metric_time, solar_metric_time, hhmm_time;
  int buttonState;

  // check to see if the user is adjusting brightness
  if ((buttonState = digitalRead(BRIGHTNESS_BUTTON_PIN)) != lastButtonState)
  {
    // wait for 10ms for state to settle
    delay(BUTTON_DEBOUNCE_DELAY);
    if ((buttonState = digitalRead(BRIGHTNESS_BUTTON_PIN)) != lastButtonState)
    {
      // if the button is pressed, adjust display brightness
      if (buttonState == LOW)
        matrix_adjust_brightness();
    }
    lastButtonState = buttonState;
  }
  
  // RTC is authoritative. Get current Unix time.
  dt = RTClib::now();
  utc = dt.unixtime();

  // compute local time and solar time from current utc
  local = tz.toLocal(utc);
  solar = utc + (int32_t) eot_adjust_seconds(year(utc), month(utc), day(utc), hour(utc),
                                              atof(MY_LONGITUDE));

  // compute a four-digit integer that is current time hhmm
  hhmm_time = 100 * hour(local) + minute(local);

  // We want to convert the current Unix time to the number of 100-microday intervals since midnight.
  // A hundred microdays is 8.64 seconds. We turn current time into seconds since midnight, then
  // convert that to 100-microday count. We do the "seconds" calculation in floats because otherwise
  // we overflow the sixteen-bit integer types on the Arduino Uno. The number of 100-microday units
  // in a day is 10,000, and that fits comfortably in an unsigned integer after dividing by 8.64.

  // solar metric
  secs_since_midnight = (hour(solar) * 3600.0) + (minute(solar) * 60.0) + (float) second(solar);
  solar_metric_time = (uint16_t) (secs_since_midnight / 8.64);

  // local metric
  secs_since_midnight = (hour(local) * 3600.0) + (minute(local) * 60.0) + (float) second(local);
  local_metric_time = (uint16_t) (secs_since_midnight / 8.64);

  // display any of the time values that have changed
  if (hhmm_time != hhmm_save)
  {
    display_time(hhmm_matrix, hhmm_time, true, false);
    hhmm_save = hhmm_time;
  }
  
  if (local_metric_time != local_metric_save)
  {
    display_time(local_matrix, local_metric_time, false, prime_time(local_metric_time));
    local_metric_save = local_metric_time;
    
    // 100 microdays just passed, so bump the counter to know when to resync
    ++SyncCounter;
  }
  
  if (solar_metric_time != solar_metric_save)
  {
    display_time(solar_matrix, solar_metric_time, false, prime_time(solar_metric_time));
    solar_metric_save = solar_metric_time;
  }
  
  if (SyncCounter >= RTC_SYNC_INTERVAL)
  {
    SetTimeFromNetwork();
    SyncCounter = 0;
    
    // force redisplay of all times on next entry to loop
    local_metric_save = solar_metric_save = hhmm_save = 0;
  }
  
  // chill out for a microday
  delay(86);
}
