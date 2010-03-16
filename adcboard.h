/*
 *
 *    This is free software, written by Richard B. Johnson. It is
 *    presumed to be correct but it is not warranted to do anything
 *    useful. Therefore, one must use this at their own peril since
 *    it is possible for any kernel code to corrupt a machine and
 *    destroy valuable property existing on storage media.
 *
 *    This software is Copyright(c) 2003, by Richard B. Johnson
 *    Everybody is granted a non-exclusive license to use this
 *    software for any purpose. However, you are required to retain
 *    the author's name within this source-code.
 */

#ifndef _ADC_BOARD_H_
#define _ADC_BOARD_H_
#define DEV_MAJOR 177
#define CLOCK     10000000          /* 1 MHz clock on the board     */
#define FILT_COEF 0x20              /* Default value For IIR filter */

/*
 *   This template is the order and kind of data returned from the
 *   ioctl() function, ADC_READ_MULTI. You get all the channels,
 *   all at once.
 */

struct adc_multi {
    int chan00;
    int chan01;
    int chan02;
    int chan03;
    int chan04;
    int chan05;
    int chan06;
    int chan07;
    int chan08;
    int chan09;
    int chan0a;
    int chan0b;
    int chan0c;
    int chan0d;
    int chan0e;
    int chan0f;
    };

/*
 *  This is a template for the data that is passed to the ioctl()
 *  function, ADC_POINT_LIST.
 *  The filter coefficient must be 1 or greater.
 *  The input parameter is the logical OR of the bits shown in
 *  the documentation to configure the input channel. Bits
 *  MA0 through MA3 are ignored. The channels are configured
 *  in order, zero through fifteen. 
 */

typedef struct {
    int input;            /* Input setup from offset 2 ADC/Control */
    int filter;           /* Filter coefficient for this channel   */
    } ADC_LIST;

struct adc_list {
    ADC_LIST list[0x10]; /* Sixteen of these, one for each channel */
    };
/*
 *  These are the supported functions, used as the second parameter of
 *  an ioctl() call.
 */
#define ADC_SINGLE_CHAN  0x00       /* Acquire data from chan set in 0x05 */
#define ADC_MULTI_CHAN   0x01       /* Acquire filtered data from all     */
#define ADC_READ_MULTI   0x02       /* Read all filtered data             */
#define ADC_SET_FILTER   0x03       /* Set filter coefficient             */
#define ADC_SET_TIMER    0x04       /* Set timer rate microseconds        */
#define ADC_SET_CHAN     0x05       /* Set for single channel operation   */
#define ADC_DIAGS        0x06       /* Hook to see what's going on        */
#define ADC_POINT_LIST   0x07       /* Warning! Read the manual!          */
#define ADC_GET_INTRS    0x08       /* Return interrupt count             */
/*
 *   These set the input range and configuration.
 */
#define ADC_BIP_10       0x09       /* Biploar -10V to + 10V              */
#define ADC_BIP_05       0x0a       /* Bipolar -5V to +5V                 */
#define ADC_BIP_2d5      0x0b       /* Bipolar -2.5V to + 2.5V            */ 
#define ADC_BIP_1d25     0x0c       /* Bipolar -1.25 to + 1.25V           */
#define ADC_UNI_10       0x0d       /* Unipolar 0 to 10V                  */
#define ADC_UNI_05       0x0e       /* Unipolar 0 to 5V                   */
#define ADC_UNI_1d25     0x0f       /* Unipolar 1.25 to 3.75 V            */
#define ADC_UNI_6d25     0x10       /* Unipolar 1.25 to 6.25 V            */
/*
 *  These bits are defined in the instruction manual for offset 2
 *  write. These are used in the ADC_POINT_LIST ioctl() function.
 */
#define ADC_R0           0x01
#define ADC_R1           0x02
#define ADC_R2           0x04
#define ADC_DIFF         0x08

#endif

