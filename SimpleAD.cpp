/* Revision: 2.8.5 */

/******************************************************************************
* Copyright 1998-2017 NetBurner, Inc.  ALL RIGHTS RESERVED
*
*    Permission is hereby granted to purchasers of NetBurner Hardware to use or
*    modify this computer program for any use as long as the resultant program
*    is only executed on NetBurner provided hardware.
*
*    No other rights to use this program or its derivatives in part or in
*    whole are granted.
*
*    It may be possible to license this or other NetBurner software for use on
*    non-NetBurner Hardware. Contact sales@Netburner.com for more information.
*
*    NetBurner makes no representation or warranties with respect to the
*    performance of this computer program, and specifically disclaims any
*    responsibility for any damages, special or consequential, connected with
*    the use of this program.
*
* NetBurner
* 5405 Morehouse Dr.
* San Diego, CA 92121
* www.netburner.com
******************************************************************************/
#include <predef.h>
#include <stdio.h>
#include <basictypes.h>
#include <sim.h>

#include "introspec.h"
LOGFILEINFO;

#define NUM_ADC 4

void InitSingleEndAD()
{
    // See MCF5441X RM Chapter 29
    sim2.adc.cr1 = 0;       // Set up control register1
    sim2.adc.cr2 = 0;       // Set up control register2
    sim2.adc.zccr = 0;      // Disable ZC
    sim2.adc.lst1 = 0x3210; // Set samples 0-3 to ADC_IN0-3
    //sim2.adc.lst2 = 0x7654; // Set samples 4-7 to ADC_IN4-7
    sim2.adc.sdis = 0;      // Enable all samples
    sim2.adc.sr = 0xFFFF;   // Clear Status Register

    for (int i = 0; i < NUM_ADC; i++) // Clear result + offset registers
    {
        sim2.adc.rslt[i] = 0;
        sim2.adc.ofs[i] = 0;
    }
    sim2.adc.lsr = 0xFFFF;  // Clear limit register
    sim2.adc.zcsr = 0xFFFF; // Clear ZC

    sim2.adc.pwr = 0;       // Power all peripherals
    sim2.adc.cal = 0;       // User internal calibration
    sim2.adc.pwr2 = 0x0005; // Default power conversion
    sim2.adc.div = 0x505;   // Set default divider
    sim2.adc.asdiv = 0x13;  // AS divisor set
}

void StartAD()
{
   sim2.adc.sr = 0xffff;   // Clear status codes
   sim2.adc.cr1 |= 0x2000; // Starting ADC
}

bool ADDone()
{
   if (sim2.adc.sr & 0x0800)
      return true;
   else
      return false;
}

uint16_t GetADResult(int ch) //Get the AD Result
{
    if ((ch < 0) || (ch > NUM_ADC))
        return 0;
    else
        return sim2.adc.rslt[ch];
}

