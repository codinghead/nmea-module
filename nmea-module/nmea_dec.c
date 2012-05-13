/*
 * NMEA Decoder for GPS Systems
 * Stuart Cording 27th Aug 2005
 */
 
/*
 *  Include Files
 */
#include <std.h>
#include <csl.h>
#include <swi.h>
#include <log.h>
#include "..\audioappcfg.h"
#include "ascii_16.h"
#include "nmea_dec.h"
#include "dsk5510_tl16c750.h"

/* 
 *  Prototypes
 */
void processNmea(void);						// Processes NMEA messages and checks checksum
void decodeNmea(void);						// Decodes the NMEA and extracts the content
Uint16 asciiToHex(Uint16 ascii);			// Converts ASCII to Hex
void incDataPointer(Uint16 * pointer);		// Increments the data pointer and keeps it inside
											// the circular buffer
void GPGSV_decode(void);					// Decode GPGSV messages
void GPGLL_decode(void);					// Decode GPGLL messages

/*
 *  Debugging functions for NMEA display
 */

//#define OUTPUT_GPGSV_DATA
#ifdef OUTPUT_GPGSV_DATA
void outputGPGSV(void);
#endif
#define OUTPUT_GPGLL_DATA
#ifdef OUTPUT_GPGLL_DATA
void outputGPGLL(void);
#endif



/*
 *  Declarations
 */
#define NMEABUFFSIZE		256				// Should hold one and 1/2 complete messages
											// keep a ^2 - circular buffer!!!
#define UARTBUFFSIZE		64				// Dependent on UART settings

/*
 *  Global Variables
 */
Uint16 nmeaBuffer[NMEABUFFSIZE];			// Circular buffer to store messages
//Uint16 nmeaDataIn = 0;						// Pointer to circ buffer input
Uint16 nmeaDataOut = 0;						// Pointer to circ buffer output
extern Uint16 uartDataBuffer[UARTBUFFSIZE];		// UART buffer contents as acquired by uartHwi

nmeaSatelliteInView satsInView[12];			// Twelve structs to store sat-in-view info (GPGSV)
Uint16 satellitesInView;					// How many satellites we can see

nmeaGeographicPosition geographicPos;		// Structure holding out position & time (GPGLL)

/*
 * Routines
 */
// Note - this is a SWI function
void processNmea(void)
{
//	Uint16 nmeaBuffer[NMEABUFFSIZE];		// Storage for NMEA message
	Uint16 uartCount;						// Count how much data came out of UART


	static CSLBool foundDollar = FALSE;		// Holds if dollar start symbol was found
	static CSLBool foundStar = FALSE;		// Holds if star end symbol was found
	static Uint16 nmeaChecksum = 0;			// NMEA Checksum
	static Uint16 nmeaChkSum = 0;			// Actual Checksum
	static Uint16 nmeaChkSumChars = 0;		// Counts how many bytes after '*' symbol
	static Uint16 storeChksum = 0;			// Holds storage point for checksum correct/incorrect flag in nmeaBuffer
//	static Uint16 count = 0;
	static Uint16 nmeaDataIn = 0;			// Pointer to circ buffer input

/*
	CSLBool foundDollar = FALSE;		// Holds if dollar start symbol was found
	CSLBool foundStar = FALSE;		// Holds if star end symbol was found
	Uint16 nmeaPointer = 0;			// Points to next empty space in nmeaBuffer
	Uint16 nmeaChecksum = 0;			// NMEA Checksum
	Uint16 nmeaChkSum = 0;			// Actual Checksum
	Uint16 nmeaChkSumChars = 0;		// Counts how many bytes after '*' symbol
	Uint16 storeChksum = 0;			// Holds storage point for checksum correct/incorrect flag
*/

	Uint16 i;								// Standard counter variable
	Uint16 tempBuffer[UARTBUFFSIZE];		// Bug fix

	// Set uartCount to zero
//	uartCount = 0;
	
	// Find out how many bytes get!
	uartCount = SWI_getmbox();
	
	// Note: uartCount now holds how many bytes we read!!!
//	LOG_printf(&logNmea, "NMEA has %d bytes" , uartCount);

//	count++;
//	if (count == 2)
//	{
//		LOG_printf(&logNmea, "NMEA second round");
//	}

	// Temp bug fix because the uartDataBuffer var as a global was
	// not being clearly displayed in CCS3.2
	for (i=0; i < uartCount; i++)
	{
		tempBuffer[i] = uartDataBuffer[i];
	}
	
	// Now a nice loop in which we copy the data across to the nmeaBuffer
	// and check the checksum etc.
	// All flags are static as a GPS message may lie across many UART buffers
	for (i = 0; i < uartCount; i++)
	{
		// Search for the $ symbol if we don't already have it
		// and if we also haven't found the '*' yet
		// from a previous buffer load
		if (!foundDollar & !foundStar)
		{
			// Set counter to zero
	//		uartCount = 0;
			// Make sure we don't overrun the amount of data we read
			while (i < uartCount)
			{
				// If we find the '$', set the flag
				if (tempBuffer[i] == A_DOLLAR)
				{
					i++;
					// Set the foundDollar flag
					foundDollar = TRUE;
					// Clear the foundStar flag
					foundStar = FALSE;
					// Clear the checksum variable (calculated)
					nmeaChecksum = 0;
					// Reset the nmeaChkSumChars counter
					nmeaChkSumChars = 0;
					// Clear the checksum variable (from NMEA string)
					nmeaChkSum = 0;
					// Save a space at start of NMEA string for
					// our checksum correct flag
					storeChksum = nmeaDataIn;
					nmeaDataIn++;
					// Check nmeaDataIn for circ buff loop
					if (nmeaDataIn == NMEABUFFSIZE)
					{
						nmeaDataIn = 0;
					}
					break;
				}
				// Otherwise just increment the counter
				else
				{
					i++;
				}
			}
		}
		
		// If we have found the '$', but not the '*' yet,
		// start to put message in Circular NMEA buffer
		if (foundDollar & !foundStar)
		{
			// Make sure we don't overrun the buffer
			while (i < uartCount)
			{
				// Look for the '*' for the end
				if (tempBuffer[i] != A_STAR)
				{
					// Not the '*' end symbol, so copy this byte
					nmeaBuffer[nmeaDataIn] = tempBuffer[i];
					// Calculate the new checksum value (XOR)
					nmeaChecksum ^= nmeaBuffer[nmeaDataIn];
					// Increment our counters
					i++;
					nmeaDataIn++;
					// Check nmeaDataIn for circ buff loop
					if (nmeaDataIn == NMEABUFFSIZE)
					{
						nmeaDataIn = 0;
					}
				}
				// Otherwise we found the star!
				else
				{
					// Set the found star flag
					foundStar = TRUE;
					// Increment counter
					i++;

					break;
				}
			}
		}
		
		// If we found the '$' and the '*' we need to read the last
		// two bytes and compare the checksum
		if (foundDollar && foundStar)
		{
			// Make sure we don't overrun the buffer
			while (i < uartCount)
			{
				// Shift the last nibble left
				nmeaChkSum <<= 4;

				// The following code converts the ASCII to HEX
				nmeaChkSum += asciiToHex(tempBuffer[i]);

				// Now note how many checksum ASCII chars we have acquired so far
				nmeaChkSumChars ++;

				// Increment counter too
				i ++;

				// If we got both check sum values figure out if the
				// checksum is correct
				if (nmeaChkSumChars == 2)
				{
					if (nmeaChecksum == nmeaChkSum)
					{
						nmeaBuffer[storeChksum] = A_STAR;

						// Also put an end marker just in case there is no next nmeaMessage
						// and therefore no '*' to look for (ETX = End Of Text)
						nmeaBuffer[nmeaDataIn] = A_ETX;
						nmeaDataIn++;

						// Check nmeaDataIn for circ buff loop
						if (nmeaDataIn == NMEABUFFSIZE)
						{
							nmeaDataIn = 0;
						}

						// Now clear our flags
						foundDollar = FALSE;
						foundStar = FALSE;

						// Temp code to test NMEA decode output
//						nmeaDataIn = 0;

//						LOG_printf(&logNmea, "ChkSum GOOD - Posting SWI");

						// Now post the SWI and any old number - it isn't important
//						SWI_or(&decodeNmeaSwi, 0xFF);
						SWI_post(&decodeNmeaSwi);

						break;
					}
					else
					{
						nmeaBuffer[storeChksum] = A_STAR | 0xFF00;

						// Also put an end marker just in case there is no next nmeaMessage
						// and therefore no '*' to look for (ETX = End Of Text)
						nmeaBuffer[nmeaDataIn] = A_ETX;
						nmeaDataIn++;

						// Check nmeaDataIn for circ buff loop
						if (nmeaDataIn == NMEABUFFSIZE)
						{
							nmeaDataIn = 0;
						}

						// Now clear our flags
						foundDollar = FALSE;
						foundStar = FALSE;

						// Temp code to test NMEA decode output
//						nmeaDataIn = 0;

						LOG_printf(&logNmea, "<< ChkSum BAD : Exp %x, Got %x >>", nmeaChkSum, nmeaChecksum);
						break;
					}
				}
				
			}
		}
		// Now find the next dollar and reset the flags etc.
	}
}

void decodeNmea(void)
{
//	static	Uint16 nmeaDataOut = 0;			// Pointer to circ buffer output
//	Uint16 nmeaDataOut;					// Pointer to circ buffer output
//	CSLBool checksumGood = FALSE;			// FALSE until we checked it is true
	Uint16	nmeaSentence;				// Holds sentence content, first prefix 'GP' and then the
											// three letter postfix, or the module specific message
//h	Uns oldST1;

//	oldST1 = HWI_disable();

	nmeaSentence = 0;
//	nmeaDataOut = 0;
	// Make a loop to work until we get to the end of the message (ETX marker)
	do
	{
		// First check to see that we have a '*' and that it was a good checksum
		if (nmeaBuffer[nmeaDataOut] != A_STAR)
		{
			LOG_printf(&logNmea, "<< Decoder - ChkSum BAD %c : %x >>", nmeaBuffer[nmeaDataOut], nmeaBuffer[nmeaDataOut]);

			// If checksum was bad, roll to ETX
			while (nmeaBuffer[nmeaDataOut] != A_ETX)
			{
				incDataPointer(&nmeaDataOut);
			}

			// Check to see if there is another sentence to decode
			// Mask so we ignore if the checksum failed or not flag
			if ((nmeaBuffer[nmeaDataOut + 1] & 0x00FF) == A_STAR)
			{
				// Increment the counter so we don't drop out of the do-while loop yet!
				incDataPointer(&nmeaDataOut);
			}

			// If we got here, then nmeaBuffer[nmeaDataOut] == A_ETX, so the do-while
			// loop is going to finish
		}

		// Now enter the next stage of decoding if there is a '*' and valid checksum
		else
		{
//			LOG_printf(&logNmea, "Decoder - ChkSum GOOD");

			// First, move past the '*' to the first character
			incDataPointer(&nmeaDataOut);

			// Now figure out if we start with 'GP' or not
			nmeaSentence += nmeaBuffer[nmeaDataOut];
			incDataPointer(&nmeaDataOut);
			nmeaSentence <<= 8;
			nmeaSentence += nmeaBuffer[nmeaDataOut];
			incDataPointer(&nmeaDataOut);

			// Check if it is 'GP'
			if (nmeaSentence == NMEA_GP)
			{
				// Clear the nmeaSentence var so we can use it again
				nmeaSentence = 0;

				// Feed the three postfix chars into the variable
				nmeaSentence += nmeaBuffer[nmeaDataOut];
				incDataPointer(&nmeaDataOut);
				nmeaSentence += nmeaBuffer[nmeaDataOut];
				incDataPointer(&nmeaDataOut);
				nmeaSentence += nmeaBuffer[nmeaDataOut];
				incDataPointer(&nmeaDataOut);
				
				// Now decode the message based upon this data

 				switch (nmeaSentence)
				{
				// GSV - Satellites in view
				case NMEA_GPGSV:
					LOG_printf(&logNmea, "GPGSV Sentence");
					GPGSV_decode();
					break;
				
				case NMEA_GPGLL:
					LOG_printf(&logNmea, "GPGLL Sentence");
					GPGLL_decode();
					if (geographicPos.status == NMEA_GPGLL_VALID)
					{
						SEM_postBinary(&locationCheckSem);
					}
					break;
				// next case

				default:
					// Go to end of this sentence
					while (nmeaBuffer[nmeaDataOut] != A_ETX)
					{
						incDataPointer(&nmeaDataOut);
					}
//					incDataPointer(&nmeaDataOut);
					LOG_printf(&logNmea, "<< NMEA Sentence not recognised >>");

				} // END OF SWITCH STATEMENT

			} // END OF IF GP SENTENCE
			
			// Handle here if not a 'GP' sentence (GPS module specific)
			else
			{
				// For now just skip the contents
				while (nmeaBuffer[nmeaDataOut] != A_ETX)
				{
					incDataPointer(&nmeaDataOut);
				}
//					incDataPointer(&nmeaDataOut);
				LOG_printf(&logNmea, "<< Not a GP sentence >>");
			}

		} // END OF ELSE NOT A STAR STATEMENT

	} while (nmeaBuffer[nmeaDataOut] != A_ETX);

	// Increment pointer to start of next message
	incDataPointer(&nmeaDataOut);

//	HWI_restore(oldST1);
}

Uint16 asciiToHex(Uint16 ascii)
{
	if (ascii >= '0' && ascii <= '9')
	{
		// Return the value minus ASCII '0'
		return (ascii - '0');
	}
	else
	{
		// Return the value minus 0x37, which changes ASCII A to 0x0A etc.
		return (ascii - 0x37);
	}

}

void incDataPointer(Uint16 * pointer)
{
	(*pointer) ++;

	if ((*pointer) > NMEABUFFSIZE)
	{
		(*pointer) = 0;
	}
}


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

NOTE: We only support maximum 12 satellites - this is only for debug
info anyway, we probably won't use this info for anything except to
confirm that we can see satellites

WARNING: Make use of the "satellitesInView" value. If it is less than 12,
it is possible that the the last elements of the the sentences will be
full of rubbish.
e.g. if "satellites in view" is 10, then the 3rd message, last two elements
could be filled with meaningless rubbish.
----------------------------------------------------------------------------*/
void GPGSV_decode(void)
{
	Uint16  nmeaTemp1;						// Temp var for use during decoding
	Uint16  nmeaTemp2;						// Temp var for use during decoding
	Uint16  nmeaCount;						// Counter for this function
	Uint16 nmeaBufferLocal[NMEABUFFSIZE];			// Circular buffer to store messages
	Uint16 i;

	for (i=0; i< NMEABUFFSIZE; i++)
	{
		nmeaBufferLocal[i] = nmeaBuffer[i];
	}
	// Reset the counter so we count the data for
	// each sat into the array
	nmeaCount = 0;

	// Move pointer past the comma we are pointing to
	incDataPointer(&nmeaDataOut);

	// Read out the number of messages (total)
	while (nmeaBufferLocal[nmeaDataOut] != A_COMMA)
	{
		incDataPointer(&nmeaDataOut);
	}

	// Move pointer past the comma we are pointing to
	incDataPointer(&nmeaDataOut);

	// Read out message number
	nmeaTemp1 = 0;
	while (nmeaBufferLocal[nmeaDataOut] != A_COMMA)
	{
		nmeaTemp1 *= 10;
		nmeaTemp1 += asciiToHex(nmeaBufferLocal[nmeaDataOut]);
		incDataPointer(&nmeaDataOut);
	}
	// Check to make sure that number is not greater than 3
	if (nmeaTemp1 > 3)
	{
		LOG_printf(&logNmea, "ERROR in NMEA GPGSV: Too many messages (total = %d)", nmeaTemp1);
		return;
	}
	// Now subtract one and multiply by 4 to use this value 
	// for accessing our array
	nmeaTemp1 = (nmeaTemp1 - 1) * 4;

	// Move pointer past the comma we are pointing to
	incDataPointer(&nmeaDataOut);

	// Find out number of visable satellites
	// We use temp variable until we know the value
	// as someone may be reading it. When we have the 
	// complete value, then we copy, else someone may 
	// read '0' by error, just 'cause we are in the 
	// middle of figuring out the data
	nmeaTemp2 = 0;
	while (nmeaBufferLocal[nmeaDataOut] != A_COMMA)
	{
		nmeaTemp2 *= 10;
		nmeaTemp2 += asciiToHex(nmeaBufferLocal[nmeaDataOut]);
		incDataPointer(&nmeaDataOut);
	}
	satellitesInView = nmeaTemp2;

	// Work through the rest of the sentence 'till we get to 
	// the end marker
	while (nmeaBufferLocal[nmeaDataOut] != A_ETX)
	{
		// Move pointer past the comma we are pointing to
		incDataPointer(&nmeaDataOut);
		// Get the sat number
		nmeaTemp2 = 0;
		while(nmeaBufferLocal[nmeaDataOut] != A_COMMA)
		{
			nmeaTemp2 *= 10;
			nmeaTemp2 += asciiToHex(nmeaBufferLocal[nmeaDataOut]);
			incDataPointer(&nmeaDataOut);
		}
		satsInView[nmeaTemp1].satelliteNumber = nmeaTemp2;
		// Move pointer past the comma we are pointing to
		incDataPointer(&nmeaDataOut);
		// Get the sat elevation
		nmeaTemp2 = 0;
		while(nmeaBufferLocal[nmeaDataOut] != A_COMMA)
		{
			nmeaTemp2 *= 10;
			nmeaTemp2 += asciiToHex(nmeaBufferLocal[nmeaDataOut]);
			incDataPointer(&nmeaDataOut);
		}
		satsInView[nmeaTemp1].elevation = nmeaTemp2;
		// Move pointer past the comma we are pointing to
		incDataPointer(&nmeaDataOut);
		// Get the sat azimuth
		nmeaTemp2 = 0;
		while(nmeaBufferLocal[nmeaDataOut] != A_COMMA)
		{
			nmeaTemp2 *= 10;
			nmeaTemp2 += asciiToHex(nmeaBufferLocal[nmeaDataOut]);
			incDataPointer(&nmeaDataOut);
		}
		satsInView[nmeaTemp1].azimuth = nmeaTemp2;
		// Move pointer past the comma we are pointing to
		incDataPointer(&nmeaDataOut);
		// Get the sat SNR
		nmeaTemp2 = 0;
		// Note, sometimes no comma if there is no SNR at end of sentence
		while(nmeaBufferLocal[nmeaDataOut] != A_COMMA &&
				nmeaBufferLocal[nmeaDataOut] != A_ETX)
		{
			nmeaTemp2 *= 10;
			nmeaTemp2 += asciiToHex(nmeaBufferLocal[nmeaDataOut]);
			incDataPointer(&nmeaDataOut);
		}
		satsInView[nmeaTemp1].signalNoiseRatio = nmeaTemp2;

		// Increment nmeaTemp1
		nmeaTemp1 ++;

		// Now increment our counter
		nmeaCount ++;
		// Check it is not greater than 4
		// no more than four sat's per sentence
		// NOTE: Probably redundant
		if (nmeaCount > 4)
		{
			break;
		}
		// Now go round again and get the next sat stats!
	}
	// Move pointer past the A_ETX value we are pointing to
//	incDataPointer(&nmeaDataOut);

	// Actually, by the time we get here we should be pointing to 
	// A_ETX. The calling function needs this marker to finish
	// his do-while loop
#ifdef OUTPUT_GPGSV_DATA
	outputGPGSV();
#endif
}

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
void GPGLL_decode(void)
{
	Int32  nmeaTemp1;						// Temp var for use during decoding
	Uint16  nmeaTemp2;						// Temp var for use during decoding
	Uint16  nmeaCount;						// Counter for this function
	Uint16 nmeaBufferLocal[NMEABUFFSIZE];	// Circular buffer to store messages
	Uint16 i;

//	nmeaGeographicPosition geographicPos;		// Structure holding out position & time (GPGLL)

	for (i=0; i< NMEABUFFSIZE; i++)
	{
		nmeaBufferLocal[i] = nmeaBuffer[i];
	}
	// Reset the counter so we count the data for
	// each sat into the array
	nmeaCount = 0;

	// Move pointer past the comma we are pointing to
	incDataPointer(&nmeaDataOut);

	// Read out the latitude Degrees
	nmeaTemp1 = 0;
	{
		nmeaTemp1 += asciiToHex(nmeaBufferLocal[nmeaDataOut]);
		incDataPointer(&nmeaDataOut);
		nmeaTemp1 *= 10;
		nmeaTemp1 += asciiToHex(nmeaBufferLocal[nmeaDataOut]);
		incDataPointer(&nmeaDataOut);
	}

	// Now store value
	geographicPos.latitude.gpsDegrees = nmeaTemp1;
	
	// Read out the latitude Minutes
	nmeaTemp1 = 0;
	while (nmeaBufferLocal[nmeaDataOut] != A_FULLSTOP)
	{
		nmeaTemp1 *= 10;
		nmeaTemp1 += asciiToHex(nmeaBufferLocal[nmeaDataOut]);
		incDataPointer(&nmeaDataOut);
	}

	// Now store value
	geographicPos.latitude.gpsMinutes = nmeaTemp1;
	
	// Move pointer past the decimal point we are pointing to
	incDataPointer(&nmeaDataOut);

	// Read out the latitude MMMM after decimal point (max. four chars)
	nmeaCount = 0;
	nmeaTemp1 = 0;
	while (nmeaBufferLocal[nmeaDataOut] != A_COMMA)
	{
		nmeaTemp1 *= 10;
		nmeaTemp1 += asciiToHex(nmeaBufferLocal[nmeaDataOut]);
		incDataPointer(&nmeaDataOut);
		nmeaCount ++;
		// Make sure we only read maximum four values of precision
		if (nmeaCount > NMEA_GPGLL_PRECISION)
		{
			// Loop round until we find the comma
			while (nmeaBufferLocal[nmeaDataOut] != A_COMMA)
			{
				incDataPointer(&nmeaDataOut);
			}
		}
	}

	// Now check that there were four values of precision to read
	if (nmeaCount < NMEA_GPGLL_PRECISION)
	{
		// This increases value in nmeaTemp1 by 10, 100, 1000 etc. if necessary
		// to make up for lost precision
		for (nmeaTemp2 = 0; nmeaTemp2 < (NMEA_GPGLL_PRECISION - nmeaCount); nmeaTemp2++)
		{
			nmeaTemp1 *= 10;
		}
	}
	
	// Now store value
	geographicPos.latitude.gpsSubMinutes = nmeaTemp1;

	// Move pointer past the comma we are pointing to
	incDataPointer(&nmeaDataOut);

	// Check for S latitude for sign change
	if (nmeaBufferLocal[nmeaDataOut] == A_S || \
		nmeaBufferLocal[nmeaDataOut] == A_s)
	{
		geographicPos.latitude.gpsDegrees *= -1;
	}

	// Move pointer past the N/S letter we are pointing to
	incDataPointer(&nmeaDataOut);
	
	// Move pointer past the comma we are pointing to
	incDataPointer(&nmeaDataOut);

	// Read out the longitude DDD
	nmeaTemp1 = 0;
	{
		nmeaTemp1 += asciiToHex(nmeaBufferLocal[nmeaDataOut]);
		incDataPointer(&nmeaDataOut);
		nmeaTemp1 *= 10;
		nmeaTemp1 += asciiToHex(nmeaBufferLocal[nmeaDataOut]);
		incDataPointer(&nmeaDataOut);
		nmeaTemp1 *= 10;
		nmeaTemp1 += asciiToHex(nmeaBufferLocal[nmeaDataOut]);
		incDataPointer(&nmeaDataOut);
	}

	// Now store value
	geographicPos.longitude.gpsDegrees = nmeaTemp1;

	// Read out the longitude MM
	nmeaTemp1 = 0;
	while (nmeaBufferLocal[nmeaDataOut] != A_FULLSTOP)
	{
		nmeaTemp1 *= 10;
		nmeaTemp1 += asciiToHex(nmeaBufferLocal[nmeaDataOut]);
		incDataPointer(&nmeaDataOut);
	}

	// Now store value
	geographicPos.longitude.gpsMinutes = nmeaTemp1;

	// Move pointer past the decimal point we are pointing to
	incDataPointer(&nmeaDataOut);

	// Read out the longitude MMMM after decimal point (max. four chars)
	nmeaCount = 0;
	nmeaTemp1 = 0;
	while (nmeaBufferLocal[nmeaDataOut] != A_COMMA)
	{
		nmeaTemp1 *= 10;
		nmeaTemp1 += asciiToHex(nmeaBufferLocal[nmeaDataOut]);
		incDataPointer(&nmeaDataOut);
		nmeaCount ++;
		// Make sure we only read maximum four values of precision
		if (nmeaCount > NMEA_GPGLL_PRECISION)
		{
			// Loop round until we find the comma
			while (nmeaBufferLocal[nmeaDataOut] != A_COMMA)
			{
				incDataPointer(&nmeaDataOut);
			}
		}
	}

	// Now check that there were four values of precision to read
	if (nmeaCount < NMEA_GPGLL_PRECISION)
	{
		// This increases value in nmeaTemp1 by 10, 100, 1000 etc. if necessary
		// to make up for lost precision
		for (nmeaTemp2 = 0; nmeaTemp2 < (NMEA_GPGLL_PRECISION - nmeaCount); nmeaTemp2++)
		{
			nmeaTemp1 *= 10;
		}
	}

	// Now store value
	geographicPos.longitude.gpsSubMinutes = nmeaTemp1;

	// Move pointer past the comma we are pointing to
	incDataPointer(&nmeaDataOut);

	// Check for W longitude for sign change
	if (nmeaBufferLocal[nmeaDataOut] == A_W || \
		nmeaBufferLocal[nmeaDataOut] == A_w)
	{
		geographicPos.longitude.gpsDegrees *= -1;
	}

	// Move pointer past the E/W letter we are pointing to
	incDataPointer(&nmeaDataOut);

	// Move pointer past the comma we are pointing to
	incDataPointer(&nmeaDataOut);

	// Read out the UTC HHMMSS
	nmeaTemp1 = 0;
	while (nmeaBufferLocal[nmeaDataOut] != A_FULLSTOP)
	{
		// Hours first
		nmeaTemp1 += asciiToHex(nmeaBufferLocal[nmeaDataOut]);
		incDataPointer(&nmeaDataOut);
		nmeaTemp1 *= 10;
		nmeaTemp1 += asciiToHex(nmeaBufferLocal[nmeaDataOut]);
		incDataPointer(&nmeaDataOut);
		geographicPos.utcGpsTime.utcHours = nmeaTemp1;
		// minutes next
		nmeaTemp1 = 0;
		nmeaTemp1 += asciiToHex(nmeaBufferLocal[nmeaDataOut]);
		incDataPointer(&nmeaDataOut);
		nmeaTemp1 *= 10;
		nmeaTemp1 += asciiToHex(nmeaBufferLocal[nmeaDataOut]);
		incDataPointer(&nmeaDataOut);
		geographicPos.utcGpsTime.utcMinutes = nmeaTemp1;
		// Seconds last
		nmeaTemp1 = 0;
		nmeaTemp1 += asciiToHex(nmeaBufferLocal[nmeaDataOut]);
		incDataPointer(&nmeaDataOut);
		nmeaTemp1 *= 10;
		nmeaTemp1 += asciiToHex(nmeaBufferLocal[nmeaDataOut]);
		incDataPointer(&nmeaDataOut);
		geographicPos.utcGpsTime.utcSeconds = nmeaTemp1;
	}

	// Now increment until we reach the next comma
	// (skipping the fractions of seconds)
	while (nmeaBufferLocal[nmeaDataOut] != A_COMMA)
	{
		incDataPointer(&nmeaDataOut);
	}

	// Move pointer past the comma we are pointing to
	incDataPointer(&nmeaDataOut);

	// Read out the validity value
	if (nmeaBufferLocal[nmeaDataOut] == A_A ||\
		nmeaBufferLocal[nmeaDataOut] == A_a)
	{
		geographicPos.status = NMEA_GPGLL_VALID;
	}
	else if (nmeaBufferLocal[nmeaDataOut] == A_V ||\
		nmeaBufferLocal[nmeaDataOut] == A_v)
	{
		geographicPos.status = NMEA_GPGLL_INVALID;
	}
	else
	{
		// We should never get here
		geographicPos.status = NMEA_GPGLL_ERROR;
	}

	// Move pointer past status value we are pointing to
	incDataPointer(&nmeaDataOut);

	// Now, if we haven't reached the end, read FAA code
	if (nmeaBufferLocal[nmeaDataOut] != A_ETX)
	{
		geographicPos.faaMode = nmeaBufferLocal[nmeaDataOut];
		incDataPointer(&nmeaDataOut);
	}
	else
	{
		geographicPos.faaMode = NMEA_GPGLL_UNKNOWN;
	}

	// The following is a debug routine to check that the GSP location
	// values are not too large - think values are being corrupted by the 
	// circular buffer, and this screws up our calculations
/*	{
		if (geographicPos.latitude.gpsDegrees > 90)
		{
			geographicPos.status =  NMEA_GPGLL_INVALID;
		}
		if (geographicPos.latitude.gpsMinutes > 59 || geographicPos.latitude.gpsMinutes < 0)
		{
			geographicPos.status =  NMEA_GPGLL_INVALID;
		}
		if (geographicPos.latitude.gpsSubMinutes > 9999 || geographicPos.latitude.gpsSubMinutes < 0)
		{
			geographicPos.status =  NMEA_GPGLL_INVALID;
		}
		if (geographicPos.longitude.gpsDegrees > 90)
		{
			geographicPos.status =  NMEA_GPGLL_INVALID;
		}
		if (geographicPos.longitude.gpsMinutes > 59 || geographicPos.longitude.gpsMinutes < 0)
		{
			geographicPos.status =  NMEA_GPGLL_INVALID;
		}
		if (geographicPos.longitude.gpsSubMinutes > 9999 || geographicPos.longitude.gpsSubMinutes < 0)
		{
			geographicPos.status =  NMEA_GPGLL_INVALID;
		}
	}
*/
	// Actually, by the time we get here we should be pointing to 
	// A_ETX. The calling function needs this marker to finish
	// his do-while loop
#ifdef OUTPUT_GPGLL_DATA
	outputGPGLL();
#endif
}


#ifdef OUTPUT_GPGSV_DATA
void outputGPGSV(void)
{
	int i;

	LOG_printf(&logNmeaData, "Satellites in view %d", satellitesInView);

	for (i = 0; i < 12; i++)
	{
		{
			LOG_printf(&logNmeaData, "Satellite No: %d", satsInView[i].satelliteNumber);
			LOG_printf(&logNmeaData, "Elevation   : %d", satsInView[i].elevation);
			LOG_printf(&logNmeaData, "Azimuth     : %d", satsInView[i].azimuth);
			LOG_printf(&logNmeaData, "SNR         : %d", satsInView[i].signalNoiseRatio);
			LOG_printf(&logNmeaData, " ");
		}
	}
}
#endif

#ifdef OUTPUT_GPGLL_DATA
void outputGPGLL(void)
{
	LOG_printf(&logNmeaData, "Latitude  Degrees : %d", geographicPos.latitude.gpsDegrees);
	LOG_printf(&logNmeaData, "Latitude  Minutes : %d.%d", geographicPos.latitude.gpsMinutes, geographicPos.latitude.gpsSubMinutes);
	LOG_printf(&logNmeaData, "Longitude Degrees : %d", geographicPos.longitude.gpsDegrees);
	LOG_printf(&logNmeaData, "Longitude Minutes : %d.%d", geographicPos.longitude.gpsMinutes, geographicPos.longitude.gpsSubMinutes);
	LOG_printf(&logNmeaData, "UTC Time  HH:MM   : %d:%d", geographicPos.utcGpsTime.utcHours, geographicPos.utcGpsTime.utcMinutes);
	LOG_printf(&logNmeaData, "UTC Time  SS      : %d", geographicPos.utcGpsTime.utcSeconds);
	LOG_printf(&logNmeaData, "Status      : %c", geographicPos.status);
	LOG_printf(&logNmeaData, "FAA Mode    : %d", geographicPos.faaMode);
	LOG_printf(&logNmeaData, " ");
}
#endif




