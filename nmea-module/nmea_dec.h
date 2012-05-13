/*
 * NMEA Message Decode
 * Stuart Cording 28th Sep 2005
 *
 * All GPS sentances start with the two character prefix 'GP' followed by a 
 * three character posfix. These are what are listed below.
 *
 */

#include "ascii_16.h"

/*----------------------------------------------------------------------------
 Declare the possible two prefix characters

 These values are made of the two chars left-shifted into an Uint16

----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------

 GP

----------------------------------------------------------------------------*/
#define NMEA_GP		0x4750

/*----------------------------------------------------------------------------
 Declare the possible three postfix characters

 These values are made of the three chars added to one another in an Uint16

----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------

 GSV - Satellites in view

These sentences describe the sky position of a UPS satellite in view.
Typically they're shipped in a group of 2 or 3.

          1 2 3 4 5 6 7     n
         | | | | | | |     |
 $--GSV,x,x,x,x,x,x,x,...*hh<CR><LF>

 Field Number: 
  1) total number of messages
  2) message number
  3) satellites in view
  4) satellite number
  5) elevation in degrees (0-90)
  6) azimuth in degrees to true north (0-359)
  7) SNR in dB (0-99)
  more satellite infos like 4)-7)
  n) checksum

Example:
    $GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74
    $GPGSV,3,2,11,14,25,170,00,16,57,208,39,18,67,296,40,19,40,246,00*74
    $GPGSV,3,3,11,22,42,067,42,24,14,311,43,27,05,244,00,,,,*4D

----------------------------------------------------------------------------*/

#define NMEA_GPGSV		0x00F0



/*----------------------------------------------------------------------------

 GLL - Geographic Position - Latitude/Longitude

          1       2 3        4 5         6 7   8
         |       | |        | |         | |   |
 $--GLL,llll.ll,a,yyyyy.yy,a,hhmmss.ss,a,m,*hh<CR><LF>

 Field Number: 
  1) Latitude
  2) N or S (North or South)
  3) Longitude
  4) E or W (East or West)
  5) Universal Time Coordinated (UTC)
  6) Status A - Data Valid, V - Data Invalid
  7) FAA mode indicator (NMEA 2.3 and later)
  8) Checksum

Introduced in NMEA 3.0.

----------------------------------------------------------------------------*/

#define NMEA_GPGLL				0x00DF
// Define valid and invalid using ascii chars
#define NMEA_GPGLL_VALID		A_A
#define NMEA_GPGLL_INVALID		A_V
#define NMEA_GPGLL_ERROR		0x0000
// Define FAA Mode
#define NMEA_GPGLL_UNKNOWN		0x0000
// This says how many values we support after decimal point in lat and long
#define NMEA_GPGLL_PRECISION	4

/*----------------------------------------------------------------------------

GGA - Global Positioning System Fix Data
Time, Position and fix related data for a GPS receiver.

          1         2       3 4        5 6 7  8   9  10 |  12 13  14   15
         |         |       | |        | | |  |   |   | |   | |   |    |
 $--GGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh<CR><LF>

 Field Number: 
  1) Universal Time Coordinated (UTC)
  2) Latitude
  3) N or S (North or South)
  4) Longitude
  5) E or W (East or West)
  6) GPS Quality Indicator,
     0 - fix not available,
     1 - GPS fix,
     2 - Differential GPS fix
     (values above 2 are 2.3 features)
     3 = PPS fix
     4 = Real Time Kinematic
     5 = Float RTK
     6 = estimated (dead reckoning)
     7 = Manual input mode
     8 = Simulation mode
  7) Number of satellites in view, 00 - 12
  8) Horizontal Dilution of precision (meters)
  9) Antenna Altitude above/below mean-sea-level (geoid) (in meters)
 10) Units of antenna altitude, meters
 11) Geoidal separation, the difference between the WGS-84 earth
     ellipsoid and mean-sea-level (geoid), "-" means mean-sea-level
     below ellipsoid
 12) Units of geoidal separation, meters
 13) Age of differential GPS data, time in seconds since last SC104
     type 1 or 9 update, null field when DGPS is not used
 14) Differential reference station ID, 0000-1023
 15) Checksum

----------------------------------------------------------------------------*/

#define NMEA_GPGGA		0x00CF


/*----------------------------------------------------------------------------

 RMC - Recommended Minimum Navigation Information
                                                              12
          1         2 3       4 5        6  7   8   9    10 11|  13
         |         | |       | |        |  |   |   |    |  | |   |
 $--RMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,xxxx,x.x,a,m,*hh<CR><LF>

 Field Number: 
  1) UTC Time
  2) Status, V=Navigation receiver warning A=Valid
  3) Latitude
  4) N or S
  5) Longitude
  6) E or W
  7) Speed over ground, knots
  8) Track made good, degrees true
  9) Date, ddmmyy
 10) Magnetic Variation, degrees
 11) E or W
 12) FAA mode indicator (NMEA 2.3 and later)
 13) Checksum

A status of V means the GPS has a valid fix that is below an internal
quality threshold, e.g. because the dilution of precision is too high 
or an elevation mask test failed.

----------------------------------------------------------------------------*/

#define NMEA_GPRMC		0x00E2


/*----------------------------------------------------------------------------

 GSA - GPS DOP and active satellites

          1 2 3                  14 15  16  17  18
         | | |                   |  |   |   |   |
 $--GSA,a,a,x,x,x,x,x,x,x,x,x,x,x,x,x,x,x.x,x.x,x.x*hh<CR><LF>

 Field Number: 
  1) Selection mode
  2) Mode (1 = no fix, 2 = 2D fix, 3 = 3D fix)
  3) ID of 1st satellite used for fix
  4) ID of 2nd satellite used for fix
  ...
  14) ID of 12th satellite used for fix
  15) PDOP
  16) HDOP
  17) VDOP
  18) checksum

Robin Darroch writes: "As I understand it, DOP is unit-less, and can
only be compared meaningfully to other DOP figures.  A DOP of 4
indicates twice the likelihood of a given position error compared with
a DOP of 2.  The DOP is calculated from the expected errors due to
current geometry of the satellites used to obtain the fix.  The
estimated position errors should show a strong correlation with DOP,
but be completely different in value as they are measured in distance
units (i.e.  metres), and they are trying to tell you "you're very
probably within x metres of this point" rather than "I'm about twice
as sure of my position as I was a couple of minutes ago".

----------------------------------------------------------------------------*/

#define NMEA_GPGSA		0x00DB


/*----------------------------------------------------------------------------
 This structure defines the contents of a GPGSV message

 The content is basically:
	Uint16 Satellite Number
	Int16 Elevation
	Int16 Azimuth
	Int16 SNR
 
 All values are integers
----------------------------------------------------------------------------*/
typedef struct {
	Uint16 satelliteNumber;
	Int16  elevation;
	Int16  azimuth;
	Int16  signalNoiseRatio;
} nmeaSatelliteInView;

/*----------------------------------------------------------------------------
 This structure defines the contents of a GPGLL message

 The content is basically:
	Int16 Latitude Degrees (+ve = North, -ve = South)
		(format DDMMMMMM, where last four values are <1 of a minute)
	Int16 Latitude Minutes
	Int16 Latitude SubMinutes
	Int16 Longitude Degrees (+ve = East, -ve = West)
		(format DDDMMMMMM, where last four values are <1 of a minute)
	Int16 Longitude Minutes
	Int16 Longitude SubMinutes
	Uint16 UTC Time Hours
		(format HHMMSS (we ignore parts of seconds))
	Uint16 UTC Time Minutes
	Uint16 UTC Time Seconds
	Uint16 Status
	Uint16 FAA Mode
 
 All values are integers
----------------------------------------------------------------------------*/
typedef struct {
	Int16 utcHours;
	Int16 utcMinutes;
	Int16 utcSeconds;
} utcTime;

typedef struct {
	Int16 gpsDegrees;
	Int16 gpsMinutes;
	Int16 gpsSubMinutes;
} gpsCoord;

typedef struct {
	gpsCoord	latitude;
	gpsCoord	longitude;
	utcTime		utcGpsTime;
	Uint16 		status;
	Uint16 		faaMode;
} nmeaGeographicPosition;
