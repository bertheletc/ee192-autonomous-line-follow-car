/*
 * FastAnalogIn_KLXX_K20D50.cpp
 *
 *  Created on: May 9, 2017
 *      Author: Nick
 */



#include "FastAnalogIn.h"
#include <PeripheralPins.h>

#define MAX_FADC            6000000
#define CHANNELS_A_SHIFT    5

FastAnalogIn::FastAnalogIn(PinName pin, bool enabled)
{
    ADCnumber = (ADCName)pinmap_peripheral(pin, PinMap_ADC);
    if (ADCnumber == (ADCName)NC) {
        error("ADC pin mapping failed");
    }

    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;

    uint32_t port = (uint32_t)pin >> GPIO_PORT_SHIFT;
    SIM->SCGC5 |= 1 << (SIM_SCGC5_PORTA_SHIFT + port);

    uint32_t cfg2_muxsel = ADC_CFG2_MUXSEL_MASK;
    if (ADCnumber & (1 << CHANNELS_A_SHIFT)) {
        cfg2_muxsel = 0;
    }

    // bus clk
    uint32_t PCLK = SystemCoreClock;
    uint32_t clkdiv;
    for (clkdiv = 0; clkdiv < 4; clkdiv++) {
        if ((PCLK >> clkdiv) <= MAX_FADC)
            break;
    }
    if (clkdiv == 4)                    //Set max div
        clkdiv = 0x7;

    ADC0->SC1[1] = ADC_SC1_ADCH(ADCnumber & ~(1 << CHANNELS_A_SHIFT));

    ADC0->CFG1 = ADC_CFG1_ADIV(clkdiv & 0x3)    // Clock Divide Select: (Input Clock)/8
               | ADC_CFG1_MODE(3)               // (16)bits Resolution
               | ADC_CFG1_ADICLK(clkdiv >> 2);  // Input Clock: (Bus Clock)/2

    ADC0->CFG2 = cfg2_muxsel            // ADxxb or ADxxa channels
               | ADC_CFG2_ADACKEN_MASK  // Asynchronous Clock Output Enable
               | ADC_CFG2_ADHSC_MASK;   // High-Speed Configuration

    ADC0->SC2 = ADC_SC2_REFSEL(0);      // Default Voltage Reference

    pinmap_pinout(pin, PinMap_ADC);

    //Enable channel
    running = false;
    enable(enabled);
}

void FastAnalogIn::enable(bool enabled)
{
    //If currently not running
    if (!running) {
        if (enabled) {
            //Enable the ADC channel
            ADC0->SC3 |= ADC_SC3_ADCO_MASK;       // Enable continuous conversion
            ADC0->SC1[0] = ADC_SC1_ADCH(ADCnumber & ~(1 << CHANNELS_A_SHIFT));  //Start conversion
            running = true;
        } else
            disable();
    }
}

void FastAnalogIn::disable( void )
{
    //If currently running
    if (running) {
        ADC0->SC3 &= ~ADC_SC3_ADCO_MASK;      // Disable continuous conversion
    }
    running = false;
}

uint16_t FastAnalogIn::read_u16()
{
    if (!running)
    {
        // start conversion
        ADC0->SC1[0] = ADC_SC1_ADCH(ADCnumber & ~(1 << CHANNELS_A_SHIFT));
        // Wait Conversion Complete
        while ((ADC0->SC1[0] & ADC_SC1_COCO_MASK) != ADC_SC1_COCO_MASK);
    }
    if(running && ((ADC0->SC1[0]&ADC_SC1_ADCH_MASK) != (ADC_SC1_ADCH(ADCnumber & ~(1 << CHANNELS_A_SHIFT)))))
    {
        running = false;
        enable();
        while ((ADC0->SC1[0] & ADC_SC1_COCO_MASK) != ADC_SC1_COCO_MASK);
    }
    // Return value
    return (uint16_t)ADC0->R[0];
}

