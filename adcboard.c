/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
/*
 *    This is a device driver for the ACCES I/O Products ADC Converter
 *    board, Model PCI-AI12-16.
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
 *
 *    This software is written using publicly-available data, an
 *    instruction manual downloaded from the vendor's Web Page.
 *    Additional information was obtained experimentally. Therefore
 *    it is believed that no proprietary nor trade-secret information
 *    was used to write this driver.
 *
 *    Please send any bug-fixes or improvements to rjohnson@analogic.com
 *
 *    I have implemented two modes of operation.
 *
 *    (1)  The streaming mode where one channel is used to
 *    acquire data as rapidly as possible. The conversion is timed
 *    by the internal timers so that there is a known sample-rate
 *    that can be used for further data processing.
 *
 *    (2)  The multiplexing mode where all channels are converted in
 *    a round-robin fashion. The data from the channels are filtered
 *    using an IIR filter, and the data may be read at any time without
 *    upsetting the conversion process.  The filtered data are read
 *    from memory without affecting the conversion process.
 *
 *    This is the mode one would normally use to acquire noisy data
 *    in an industrial environment.
 *
 *    In both cases, the sample-rate can be set from user-mode via
 *    ioctl(), and, in the case of the multiplexing mode, the IIR
 *    filter coefficient may be set as well.
 *
 *    Note that this will only run on an ix86 platform, not the
 *    other Linux ones like Sparc, Power/PC, etc.
 *
 */

#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/ioport.h>
#include <linux/wait.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <linux/pci.h>
#include <asm/io.h>
#include "adcboard.h"

#define ADC_CLASS   0xff00
#define ADC_DEV     0xaca8
#define ADC_DEVA    0xaca9
#define ADC_VENDOR  0x494f
#define TRUE 1

#define NoDEBUG

#ifdef  DEBUG
#define DEB(f) f
#else
#define DEB(f)
#endif

static const char devname[]="ACCES I/O PRODUCTS PCI-AI12-16";
static const char vers[]="V1.1";
MODULE_AUTHOR("Richard B. Johnson");
MODULE_DESCRIPTION("Driver for PCI-AI12-16(A) A/D converter board");
MODULE_SUPPORTED_DEVICE("PCI-AI12-16(A)");
/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
/*
 *   Macros. These are supposed to reduce code clutter.
 */
/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
/*
 *   The ADC Control registers.
 */
#define ADC_STAR       0x0000
#define ADC_CONT       0x0002
#define ADC_DIGI       0x0003
#define ADC_OPTC       0x0004
#define ADC_GPOT       0x0005
#define ADC_TIM0       0x0008
#define ADC_TIM1       0x0009
#define ADC_TIM2       0x000a
#define ADC_TIMC       0x000b
#define ADC_DATA       0x0000
#define ADC_CRDB       0x0002
#define ADC_ORDB       0x0003
#define ADC_STAT       0x0004
#define ADC_GPIN       0x0005
#define ADC_R0         0x01
#define ADC_R1         0x02
#define ADC_R2         0x04
#define ADC_MA0        0x10
#define ADC_MA1        0x20
#define ADC_MA2        0x40
#define ADC_MA3        0x80
#define ADC_CTR        0x01
#define ADC_EI         0x04
#define ADC_EIFH       0x04
#define ADC_XSCE       0x08
#define ADC_EXT        0x01
#define ADC_CF         0x08
#define ADC_CCF        0x40
#define ADC_FE         0x02

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/

#define BUF_LEN 0x1000
#define BUF_MSK (BUF_LEN -1)
#define DIVISOR 100                 /* Default timer divisor  */
#define CTL_LEN 0x10                /* Length of I/O space    */
#define NR_CHAN 0x10                /* Number of A/D Channels */
#define INTERRUPT_TIMEOUT (1 * HZ)
#define PCI_INDEX(v) (PCI_BASE_ADDRESS_0 + ((v) * sizeof(long)))
#define ArraySize(a) (sizeof(a) / sizeof(a[0]))

typedef struct {
    union {
    struct adc_multi ch;     /* Individual channel data   */
    int val[NR_CHAN];        /* Same, but in an array     */
    } var;                   /* For ADC variables         */
    int mod[NR_CHAN];        /* Saved modulus from divide */
    int filter[NR_CHAN];     /* Filter coefficients       */
    } ADC;

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
/*
 *  This is the device-specific information and data storage. There is
 *  only one global data element, a pointer which points to this structure.
 */

typedef struct {
    int buf[BUF_LEN];         /* ADC Data buffer       */
    int out[BUF_LEN];         /* ADC Data buffer       */
    char dev[0x30];           /* Device name           */
    ADC adc;                  /* ADC filtered data     */
    struct pci_dev *pci;      /* Save to shut down     */
    spinlock_t adc_lock;      /* Spin-lock variable    */
    wait_queue_head_t wait;   /* For poll (select)     */
    int has_fifo;             /* TRUE if it has a FIFO */
    int head;                 /* Circular buffer index */
    int tail;                 /* Circular buffer index */
    int major;                /* Device major number   */
    int irq;                  /* Interrupt to use      */
    atomic_t unload;          /* Unload flag           */
    atomic_t busy;            /* Busy flag             */
    u32 iolen;                /* Length of our window  */
    size_t adc_chan;          /* Input channel in use  */
    size_t use_chan;          /* Input channel to use  */
    unsigned int inters;      /* Number of intrerrupts */
    unsigned int poll_mask;   /* For select() (poll)   */
    unsigned int base;        /* PCI/Bus IO address    */
    unsigned int conv_mode;   /* Conversion mode       */
    unsigned int acq_mode;    /* Data acquisition mode */
    unsigned int timer;       /* ADC timer divisor     */
    } INFO;

static INFO *info;

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
/*
 *  Function prototypes.
 */
int __init init_module(void);
void cleanup_module(void);
static int adc_close(struct inode *inp, struct file *fp);
static int adc_open(struct inode *inp, struct file *fp);
static int adc_read(struct file *fp, char *cp, size_t len, loff_t *ppos);
static int adc_ioctl(struct inode *inp, struct file *fp, unsigned int cmd, unsigned long ptr);
static unsigned int adc_poll(struct file *fp, struct poll_table_struct *pt);
static size_t next_chan(INFO *inf);
static void adc_isr(int irq, void *lcl);
static void acq_data(INFO *inf);
static void set_mode(INFO *inf);
static void stop_board(INFO *inf);
static void ctr_mode(unsigned int addr, unsigned int cntr, unsigned int mode);
static void ctr_load(unsigned int addr, unsigned int cntr, unsigned int val);
static void set_point_list(INFO *inf);
static int remote_set_point_list(INFO *inf, struct adc_list *sdc);
static void set_all_filters(INFO *inf, int val);
static void clear_all_data(INFO *inf);

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
/*
 *   File operation structure contains function pointers to various routines.
 */
static struct file_operations  adc_fops = {
    owner             : THIS_MODULE ,
    read              : adc_read ,
    poll              : adc_poll ,
    ioctl             : adc_ioctl ,
    open              : adc_open ,
    release           : adc_close ,
    get_unmapped_area : NULL };    

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
/*
 *  Set the filter values for all the channels at once.
 */
static void set_all_filters(INFO *inf, int val)
{
   size_t i;
   for(i=0; i< NR_CHAN; i++)
        inf->adc.filter[i] = val;
}
/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
/*
 *  clear all the channel data at once.
 */
static void clear_all_data(INFO *inf)
{
    memset(inf->adc.var.val, 0x00, sizeof(inf->adc.var.val));
    memset(inf->adc.mod, 0x00, sizeof(inf->adc.mod));
}
/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
/*
 *   Set a counter mode.
 */
static void ctr_mode(unsigned int addr, unsigned int cntr, unsigned int mode)
{
    outb((cntr << 6) | 0x30 | (mode << 1), addr + ADC_TIMC);
    return;
}
/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
/*
 *   Load a counter divisor.
 */
static void ctr_load(unsigned int addr, unsigned int cntr, unsigned int val)
{
    outb( val & 0xff         , addr + cntr + ADC_TIM0);
    outb((val >> 0x08) & 0xff, addr + cntr + ADC_TIM0);
    return;
}
/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
/*
 *  This returns the current input channel and sets the next one
 *  so conversion can proceed at any time.
 */
static size_t next_chan(INFO *inf)
{
    size_t chan;
    chan = inf->adc_chan++;               /* Channel when acquired */
    inf->adc_chan &= 0x0f;                /* Auto-wrap             */
    outb(inf->conv_mode | (inf->adc_chan << 4), inf->base + ADC_CONT);
    return chan;
}
/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
/*
 *   Acquire and/or filter the data. Note that this is called under a
 *   spin-lock.
 */
#define IIR_FILTER \
 old = inf->adc.var.val[chan];         /* Get previous value       */\
 sav = new;                            /* Save new value           */\
 new = old     +      (inf->adc.mod[chan] + new - old) / inf->adc.filter[chan];\
 inf->adc.mod[chan] = (inf->adc.mod[chan] + sav - old) % inf->adc.filter[chan];\
 inf->adc.var.val[chan] = new

static void acq_data(INFO *inf)
{
    int new, old, sav;
    size_t chan;
    switch(inf->acq_mode)
    {
    case ADC_SINGLE_CHAN:
        do {
            new = inw(inf->base + ADC_DATA);  /* Get the converter data   */
            new <<= 0x14;                     /* Sign-extend this         */
            new >>= 0x14;
            sav = inf->head;                  /* Last buffer position     */
            inf->buf[sav++] = new;            /* Put data into the buffer */
            sav &= BUF_MSK;                   /* Auto-wrap around buffer  */
            if(sav != inf->tail)              /* If there room in buffer  */
                inf->head = sav;              /* Is room in the buffer    */
        } while (inf->has_fifo && inb(inf->base + ADC_STAT) & ADC_FE);
        break;    
    case ADC_MULTI_CHAN:
        if(inf->has_fifo)
        {
            while(inb(inf->base + ADC_STAT) & ADC_FE)
            {
                new = inw(inf->base+ADC_DATA); /* Get the converter data   */
                chan = new >> 0x0c;            /* Top 4 bits show channel  */
                new <<= 0x14;                  /* Sign-extend this         */
                new >>= 0x04;                  /* Leave 65536 times larger */
                IIR_FILTER;                    /* Filter and save the data */
            }
        }
        else
        {
            chan = next_chan(inf);            /* Channel when acquired    */
            new = inw(inf->base + ADC_DATA);  /* Get the converter data   */
            new <<= 0x14;                     /* Sign-extend this         */
            new >>= 0x04;                     /* Leave 65536 times larger */
            IIR_FILTER;                       /* Filter and save the data */
        }
        break;
    default:
        printk(KERN_ALERT"%s : Unknown acquisition mode (%d)\n",
                              inf->dev, inf->acq_mode);
        break; 
        
    }
    (void) inb(inf->base + ADC_STAT);         /* Clear interrupt          */
    return;
}
/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
/*
 *   This sets up the point-list for the FIFO. It is only used
 *   by the boards that have a FIFO.
 */
static void set_point_list(INFO *inf)
{
    size_t i;
    unsigned short list;
    outb(ADC_CF|ADC_CCF, inf->base + ADC_OPTC);   /* Reset (empty) FIFO */
    outb(0, inf->base + ADC_OPTC);                /* Now clear to begin */
    for(i=0; i< 2048; i++)
    {
        if(inf->acq_mode == ADC_SINGLE_CHAN)
            list = inf->conv_mode       |
                   (inf->use_chan << 4) | 
                   (inf->use_chan << 12);
        else
            list = inf->conv_mode |
                   ((i % 0x10) << 4) |
                   ((i % 0x10) << 12);
        outw(list, inf->base + ADC_CONT);
    }
    (void)inw(inf->base + ADC_CONT);
}
/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
/*
 *   This sets up the point-list for the FIFO. It is only used
 *   by the boards that have a FIFO.
 */
static int remote_set_point_list(INFO *inf, struct adc_list *adc)
{
    size_t i;
    unsigned long flags;
    unsigned short list;
    if(!inf->has_fifo)
        return -EINVAL;
    for(i=0; i< NR_CHAN; i++)
        if(!adc->list[i].filter)                  /* Check for div/zero    */
            return -EINVAL;                       /* NotGood(tm)           */
    outb(0, inf->base + ADC_OPTC);                /* Clear, stop interrupt */
    outb(0, inf->base + ADC_CONT);                /* Set input mode        */
    spin_lock_irqsave(&inf->adc_lock, flags);
    for(i=0; i< NR_CHAN; i++)
        inf->adc.filter[i] = adc->list[i].filter; /* Set all the filters   */
    outb(ADC_CF|ADC_CCF, inf->base + ADC_OPTC);   /* Reset (empty) FIFO    */
    outb(0, inf->base + ADC_OPTC);                /* Now clear to begin    */
    for(i=0; i< 2048; i++)
    {
        if(inf->acq_mode == ADC_SINGLE_CHAN)
            list = (adc->list[i % 0x10].input & 0x07) |
                   (inf->use_chan << 4)               | 
                   (inf->use_chan << 12);
        else
            list = (adc->list[i % 0x10].input & 0x07) |
                   ((i % 0x10) << 4)                  |
                   ((i % 0x10) << 12);
        outw(list, inf->base + ADC_CONT);
    }
    (void)inw(inf->base + ADC_CONT);              /* Read and throw away    */
    outb(ADC_CTR|ADC_EIFH, inf->base + ADC_OPTC); /* Set interrupt/timer    */
    inf->head = inf->tail = 0;                    /* Clear interrupt buffer */
    clear_all_data(inf);                          /* Clear filter array     */
    inf->poll_mask = 0;                           /* Clear poll mask        */
    wake_up_interruptible(&inf->wait);            /* No data are avaiable   */
    spin_unlock_irqrestore(&inf->adc_lock, flags);
    return 0;
}
/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
/*
 *   Set the conversion mode for the device. This is not called under
 *   a lock. Therefore, the board must be stopped first before calling
 *   this. This will restart the board.
 */
static void set_mode(INFO *inf)
{
    int count;
    unsigned long flags;
    stop_board(inf);
    count = 2;
    while((inf->timer/count) > 65534)          /* Last even count  */
        count <<=1;
    count |= (inf->timer & 1);                 /* In case it's odd */
    DEB(printk("Timer value = %d\n", inf->timer));
    DEB(printk("Count value = %d\n", count));

    spin_lock_irqsave(&inf->adc_lock, flags);
    inf->adc_chan = inf->use_chan;            
    switch(inf->acq_mode)
    {
    case ADC_SINGLE_CHAN:          /* Acquire streaming data from chan 0 */
        ctr_mode(inf->base, 0, 2);       /* Counter 0 to square-wave (2) */
        ctr_mode(inf->base, 1, 2);       /* Counter 1 to square-wave (2) */
        ctr_mode(inf->base, 2, 2);       /* Counter 2 to square-wave (2) */
        ctr_load(inf->base, 0, 0xffff);           /* Counter 0 divisor   */
        ctr_load(inf->base, 1, count);            /* Counter 1 divisor   */
        ctr_load(inf->base, 2, inf->timer/count); /* Counter 2 divisor   */
        if(inf->has_fifo)
        {
            set_point_list(inf);
            outb(ADC_CTR|ADC_EIFH, inf->base + ADC_OPTC);
        }
        else
        {
            outb(inf->conv_mode | (inf->adc_chan << 4), inf->base + ADC_CONT);
            outb(ADC_CTR|ADC_EI, inf->base + ADC_OPTC);
        }
        break;
    case ADC_MULTI_CHAN:           /* Acquire filtered data from all     */
        ctr_mode(inf->base, 0, 2);       /* Counter 0 to square-wave (2) */
        ctr_mode(inf->base, 1, 2);       /* Counter 1 to square-wave (2) */
        ctr_mode(inf->base, 2, 2);       /* Counter 2 to square-wave (2) */
        ctr_load(inf->base, 0, 0xffff);           /* Counter 0 divisor   */
        ctr_load(inf->base, 1, count);            /* Counter 1 divisor   */
        ctr_load(inf->base, 2, inf->timer/count); /* Counter 2 divisor   */
        if(inf->has_fifo)
        {
            set_point_list(inf);
            outb(ADC_CTR|ADC_EIFH, inf->base + ADC_OPTC);
        }
        else
        {
            outb(inf->conv_mode | (inf->adc_chan << 4), inf->base + ADC_CONT);
            outb(ADC_CTR|ADC_EI, inf->base + ADC_OPTC);
        }
        break;
    default:
        printk(KERN_ALERT"%s : Unknown acquisition mode (%d)\n",
                              inf->dev, inf->acq_mode);
        break; 
    }
    inf->head = inf->tail = 0;                  /* Clear interrupt buffer */
    clear_all_data(inf);                        /* Clear filter array     */
    inf->poll_mask = 0;                         /* Clear poll mask        */
    wake_up_interruptible(&inf->wait);          /* No data are avaiable   */
    spin_unlock_irqrestore(&inf->adc_lock, flags);
    return;
}
/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
/*
 *   This stops the board from converting and interrupting.
 */
static void stop_board(INFO *inf)
{
    outb(0, inf->base + ADC_OPTC);         /* Clear, stop interrupt   */
    outb(0, inf->base + ADC_CONT);         /* Set input mode          */
    inf->head = inf->tail = 0;             /* Clear interrupt buffer  */
    return; 
}
/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
/*
 *   Device poll. We execute a wake-up from the ISR everytime data
 *   are available. Therefore, there is a spin-lock around the poll
 *   mask change.
 */
static unsigned int adc_poll(struct file *fp, struct poll_table_struct *pt)
{
    unsigned long flags;
    unsigned int mask;
    poll_wait(fp, &info->wait, pt);
    spin_lock_irqsave(&info->adc_lock, flags);
    mask = info->poll_mask;
    info->poll_mask = 0;
    spin_unlock_irqrestore(&info->adc_lock, flags);
    return mask;
}
/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
/*
 *  For device control. We set various modes and parameters from here
 *  and also grab a copy of the filtered channel data.
 */
static int adc_ioctl(struct inode *inp, struct file *fp, unsigned int cmd,
                                                   unsigned long ptr)
{
    size_t i;
    unsigned long flags;
    int tmp;
    struct adc_list *adc;
    switch(cmd)
    {
    case ADC_SINGLE_CHAN:          /* Acquire streaming data from chan 0 */
    case ADC_MULTI_CHAN:           /* Acquire filtered data from all     */
        info->acq_mode = cmd;      /* Set data acquisition mode          */
        set_mode(info);            /* Set up the board                   */
        break;
    case ADC_READ_MULTI:           /* Read all filtered data             */
/*
 *   Here I copy the filtered data under a lock from the filter buffer
 *   to an output buffer. In the process, I divide the data by 65536
 *   to normalize it because it was filtered at a fixed-point multiplier
 *   of the same value.
 */
        spin_lock_irqsave(&info->adc_lock, flags);
        for(i=0; i< ArraySize(info->adc.var.val); i++)
           info->out[i] = info->adc.var.val[i] >> 0x10; 
        spin_unlock_irqrestore(&info->adc_lock, flags);
/*
 *   Release the lock, then copy to the user.
 */
        if(copy_to_user((char *)ptr, info->out, sizeof(struct adc_multi)))
            return -EFAULT;
        break;
    case ADC_SET_FILTER:
        if(get_user(tmp, (int *)ptr))
            return -EFAULT;
        if(tmp <= 0) return -EINVAL; /* Bad user input value  */
        set_all_filters(info, tmp);  /* Okay, set the filters */
        break;
    case ADC_SET_TIMER:
        if(get_user(tmp, (int *)ptr))
            return -EFAULT;
        if(tmp <= 0)
            return -EINVAL;
        info->timer = tmp;
        set_mode(info);              /* Restart with new parameters      */
        break;
    case ADC_SET_CHAN:               /* Set a new input channel */
        if(get_user(tmp, (int *)ptr))
            return -EFAULT;
        if((tmp < 0) || (tmp > 0x0F))
            return -EINVAL;
        info->use_chan = tmp;
        set_mode(info);              /* Restart with new parameters      */
        break;
    case ADC_DIAGS:
        info->use_chan  = 0;
        info->acq_mode  = ADC_SINGLE_CHAN; /* Data acquisition mode      */
        info->timer     = 100;             /* Small counter value        */
        info->conv_mode = ADC_R0|ADC_R1;   /* +/- 2.5 volts, chan 0      */
        set_mode(info);
        for(i=0; i< 0x10; i++)
            ((char *)info->out)[i] = (char)inb(info->base + i);
        if(copy_to_user((char *)ptr, info->out, 0x10))
            return -EFAULT;
        break;
    case ADC_POINT_LIST:
        adc = (struct adc_list *) info->out; 
        if(copy_from_user(info->out, (char *)ptr, sizeof(struct adc_list)))
            return -EFAULT; 
        return remote_set_point_list(info, adc);
    case ADC_GET_INTRS:
        if(put_user(info->inters, (int *)ptr))
            return -EFAULT;
        break;
    case ADC_BIP_10:                        /* Biploar -10V to + 10V     */
        info->conv_mode = 0;
        set_mode(info);
        break;
    case ADC_BIP_05:                        /* Bipolar -5V to +5V        */
        info->conv_mode = ADC_R0;
        set_mode(info);
        break;
    case ADC_BIP_2d5:                      /* Bipolar -2.5V to + 2.5V   */
        info->conv_mode = ADC_R1;
        set_mode(info); 
        break;
    case ADC_BIP_1d25:                     /* Bipolar -1.25 to + 1.25V  */
        info->conv_mode = ADC_R0|ADC_R1;
        set_mode(info); 
        break;
    case ADC_UNI_10:                       /* Unipolar 0 to 10V         */
        info->conv_mode = ADC_R2;
        set_mode(info); 
        break;
    case ADC_UNI_05:                       /* Unipolar 0 to 5V          */
        info->conv_mode = ADC_R2|ADC_R0;
        set_mode(info); 
        break;
    case ADC_UNI_1d25:                    /* Unipolar 1.25 to 3.75 V   */
        info->conv_mode = ADC_R2|ADC_R1;
        set_mode(info); 
        break;
    case ADC_UNI_6d25:                    /* Unipolar 1.25 to 6.25 V   */
        info->conv_mode = ADC_R2|ADC_R1|ADC_R0;
        set_mode(info); 
        break;
    default:
        return -EINVAL;
    }
    return 0;
}
/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
/*
 *   Read data from the device. Note that the input and output lengths
 *   are in bytes, although the data are in 4-byte integers. The byte-
 *   lengths are in keeping with the de facto Unix standard so sizeof()
 *   can be used from the API.
 */
static int adc_read(struct file *fp, char *cp, size_t len, loff_t *ppos)
{
    unsigned long flags;
    ssize_t i = 0;
    len /= sizeof(int);                              /* Make word-length */
    spin_lock_irqsave(&info->adc_lock, flags);
    while((info->head != (info->tail &= BUF_MSK)) && (len--))
        info->out[i++] = info->buf[info->tail++];
    spin_unlock_irqrestore(&info->adc_lock, flags);
    if((i *= sizeof(int) ))                          /* Make byte-count  */
    {
        if(copy_to_user(cp, info->out, i))
            return -EFAULT;
    }
    return i;
}
/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
/*
 *  Open the device.
 */
static int adc_open(struct inode *inp, struct file *fp)
{
    atomic_inc(&info->busy);                      /* Show it's busy now   */
    if(atomic_read(&info->unload))                /* Is it being unloaded */
    {
        atomic_dec(&info->busy);                  /* Bump the busy flag   */
        return -ENODEV;                           /* It's being unloaded  */
    }
//    MOD_INC_USE_COUNT;                            /* Bump usage count     */
    if(atomic_read(&info->busy) == 1)             /* First user open      */
        set_mode(info);
    return 0;
}
/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
/*
 *  Close the device.
 */
static int adc_close(struct inode *inp, struct file *fp)
{
//    MOD_DEC_USE_COUNT;                         /* Decrement usage count */
    atomic_dec(&info->busy);
    if(!atomic_read(&info->busy))              /* If the last user      */
        stop_board(info);                      /* Stop interrupts       */
    return 0;
}
/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
/*
 *   Simple ADC interrupt service routine.
 */
static void adc_isr(int irq, void *lcl)
{
    spin_lock(&info->adc_lock);
    acq_data(info);                          /* Get the data             */
    info->poll_mask = POLLIN|POLLRDNORM;     /* Set poll mask            */
    spin_unlock(&info->adc_lock);
    info->inters++;                          /* Show we got an interrupt */
    wake_up_interruptible(&info->wait);      /* Show data are avaiable   */
    return;
}
/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
/*
 *  Initialize and register the module. The device is on a PCI bridge.
 *  FYI, there are two `gotos` in this code. If goto freaks send me
 *  patches to remove them, they are wasting their time.
 */
int __init init_module()
{
    int result;
    size_t i;
    struct pci_dev *pdev = NULL;
    u32 ioport;
    u32 iolen;
    unsigned long timer;

    while ((pdev = pci_get_class(ADC_CLASS << 8, pdev)))
    {
        DEB(printk("Found %08x (%08x)\n", pdev->vendor, pdev->device));
        if((pdev->vendor == ADC_VENDOR) &&
          ((pdev->device == ADC_DEV) || (pdev->device == ADC_DEVA)) )
            goto dev_found;
    }
    printk(KERN_INFO"%s: PCI device not found\n", devname);
    return -ENODEV;
    dev_found: pci_disable_device(pdev);          /* Disable for now      */
/*
 *  Okay, we found the device. But all is not well in San Diego.
 *  This PCI interface chip is used for a lot of stuff. The first
 *  I/O address is NOT the one we use to actually talk to the
 *  chip! We need to search for the I/O address that is 16 bytes
 *  wide. That's the correct one.
 */
    for(i = 0; i< 6; i++)                        /* Max base addresses   */
    {
        ioport = 0L;
        pci_read_config_dword(pdev, PCI_INDEX(i), &ioport);
        if(!(ioport & PCI_BASE_ADDRESS_SPACE_IO))
            continue;
        ioport &= ~PCI_BASE_ADDRESS_SPACE_IO;  /* Clear that bit */
/*
 *  Set all bits in the address decode register and read the result.
 */
        iolen = 0xffffffff;
        pci_write_config_dword(pdev, PCI_INDEX(i), iolen);
        pci_read_config_dword(pdev,  PCI_INDEX(i), &iolen);
/*
 *  Write the original address decode bits back.
 */
        pci_write_config_dword(pdev, PCI_INDEX(i), ioport);
        iolen = (~iolen + 2) & 0xfffe;             /* Required window size   */
        DEB(printk("iolen = %04x\n", iolen));
        if(iolen == CTL_LEN) goto addr_found;
    }
    printk(KERN_INFO"%s: No I/O space for ADC board\n", devname);
    return -ENODEV;
    addr_found: (void)pci_enable_device(pdev); /* Now enable the device  */
    if((info = (INFO *) kmalloc(sizeof(INFO), GFP_KERNEL)) == NULL)
    {
        printk(KERN_CRIT"%s: Can't allocate memory\n", devname);
        pci_disable_device(pdev);             /* Disable the device     */
        pci_dev_put(pdev);
        return -ENOMEM;
    }
    memset(info, 0x00, sizeof(INFO));         /* Clear the structure  */
    strcpy(info->dev, devname);
    if(pdev->device == ADC_DEVA)
    {
        info->has_fifo = TRUE;
        strcat(info->dev, " A");
    }
    info->pci      = pdev;
    info->major    = DEV_MAJOR;
    info->adc_lock = SPIN_LOCK_UNLOCKED;
    info->irq      = pdev->irq;
    info->base     = ioport;
    info->iolen    = iolen;
    info->conv_mode = ADC_R0|ADC_R1;           /* +/- 1.25 volts, chan 0 */
    info->timer     = DIVISOR;
    info->acq_mode  = ADC_MULTI_CHAN;
    atomic_inc(&info->busy);
    atomic_inc(&info->unload);
    set_all_filters(info, FILT_COEF);
    stop_board(info);                          /* No interrupts for now */
    init_waitqueue_head(&info->wait);          /* Set up wait queue     */
    if((result = register_chrdev(info->major, info->dev, &adc_fops)) < 0)
    {
        printk(KERN_ALERT"%s: Can't register major number %d\n",
                                              devname, info->major);
        pci_disable_device(pdev);             /* Disable the device     */
        pci_dev_put(pdev);
        kfree(info);
        return result;
    }
    request_region(info->base, info->iolen, info->dev);
    if((result=request_irq(info->irq, adc_isr, SA_SHIRQ, info->dev, info)) < 0)
    {
        printk(KERN_ALERT"%s: Can't allocate IRQ %d\n", devname, info->irq);
        pci_disable_device(pdev);             /* Disable the device     */
        pci_dev_put(pdev);
        (void)unregister_chrdev(info->major, info->dev);
        release_region(info->base, info->iolen);
        kfree(info);
        return result;
    }
    set_mode(info);                      /* Start the board */
/*
 *  Let the auto multi-channel mode run for a bit to see if
 *  the board interrupts okay.
 */
    timer = jiffies + INTERRUPT_TIMEOUT;
    while(time_before(jiffies, timer))
    {
        if(info->inters) break;
        schedule_timeout(2);
    }
    stop_board(info);                  /* Stop the board  */
    printk(KERN_INFO"%s: Version    = %s\n",   info->dev, vers);
    printk(KERN_INFO"%s: I/O port   = %04X\n", info->dev, info->base);
    printk(KERN_INFO"%s: I/O length = %d\n",   info->dev, info->iolen);
    printk(KERN_INFO"%s: Interrupt  = %d\n",   info->dev, info->irq);
    printk(KERN_INFO"%s: Dev. major = %d\n",   info->dev, info->major);
    printk(KERN_INFO"%s: Patches to rjohnson@analogic.com\n", info->dev);
    if(!info->inters)
       printk(KERN_ALERT"%s: Warning, no interrupt detected\n", info->dev);
    printk(KERN_INFO"%s: Initialization complete\n", info->dev);
    atomic_dec(&info->busy);
    atomic_dec(&info->unload);
    return 0;
}
/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
/*
 *   Release the module.
 */
void cleanup_module()
{
    atomic_inc(&info->unload);
    if(atomic_read(&info->busy))
    {
        atomic_dec(&info->unload);
        printk(KERN_ALERT "%s: Can't unregister, busy.\n", info->dev);
        return;
    }
    stop_board(info);                         /* Stop the board         */
    pci_disable_device(info->pci);            /* Disable the device     */
    unregister_chrdev(info->major, info->dev);
    free_irq(info->irq, info);
    release_region(info->base, info->iolen);
    pci_dev_put(info->pci);
    printk(KERN_INFO"%s: Module removed\n", info->dev);
    kfree(info);
    return;
}
/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
