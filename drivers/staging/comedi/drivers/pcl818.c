/*
 * comedi/drivers/pcl818.c
 *
 * Driver: pcl818
 * Description: Advantech PCL-818 cards, PCL-718
 * Author: Michal Dobes <dobes@tesnet.cz>
 * Devices: [Advantech] PCL-818L (pcl818l), PCL-818H (pcl818h),
 *   PCL-818HD (pcl818hd), PCL-818HG (pcl818hg), PCL-818 (pcl818),
 *   PCL-718 (pcl718)
 * Status: works
 *
 * All cards have 16 SE/8 DIFF ADCs, one or two DACs, 16 DI and 16 DO.
 * Differences are only at maximal sample speed, range list and FIFO
 * support.
 * The driver support AI mode 0, 1, 3 other subdevices (AO, DI, DO) support
 * only mode 0. If DMA/FIFO/INT are disabled then AI support only mode 0.
 * PCL-818HD and PCL-818HG support 1kword FIFO. Driver support this FIFO
 * but this code is untested.
 * A word or two about DMA. Driver support DMA operations at two ways:
 * 1) DMA uses two buffers and after one is filled then is generated
 *    INT and DMA restart with second buffer. With this mode I'm unable run
 *    more that 80Ksamples/secs without data dropouts on K6/233.
 * 2) DMA uses one buffer and run in autoinit mode and the data are
 *    from DMA buffer moved on the fly with 2kHz interrupts from RTC.
 *    This mode is used if the interrupt 8 is available for allocation.
 *    If not, then first DMA mode is used. With this I can run at
 *    full speed one card (100ksamples/secs) or two cards with
 *    60ksamples/secs each (more is problem on account of ISA limitations).
 *    To use this mode you must have compiled  kernel with disabled
 *    "Enhanced Real Time Clock Support".
 *    Maybe you can have problems if you use xntpd or similar.
 *    If you've data dropouts with DMA mode 2 then:
 *     a) disable IDE DMA
 *     b) switch text mode console to fb.
 *
 *  Options for PCL-818L:
 *  [0] - IO Base
 *  [1] - IRQ        (0=disable, 2, 3, 4, 5, 6, 7)
 *  [2] - DMA        (0=disable, 1, 3)
 *  [3] - 0, 10=10MHz clock for 8254
 *            1= 1MHz clock for 8254
 *  [4] - 0,  5=A/D input  -5V.. +5V
 *        1, 10=A/D input -10V..+10V
 *  [5] - 0,  5=D/A output 0-5V  (internal reference -5V)
 *        1, 10=D/A output 0-10V (internal reference -10V)
 *        2    =D/A output unknown (external reference)
 *
 *  Options for PCL-818, PCL-818H:
 *  [0] - IO Base
 *  [1] - IRQ        (0=disable, 2, 3, 4, 5, 6, 7)
 *  [2] - DMA        (0=disable, 1, 3)
 *  [3] - 0, 10=10MHz clock for 8254
 *            1= 1MHz clock for 8254
 *  [4] - 0,  5=D/A output 0-5V  (internal reference -5V)
 *        1, 10=D/A output 0-10V (internal reference -10V)
 *        2    =D/A output unknown (external reference)
 *
 *  Options for PCL-818HD, PCL-818HG:
 *  [0] - IO Base
 *  [1] - IRQ        (0=disable, 2, 3, 4, 5, 6, 7)
 *  [2] - DMA/FIFO  (-1=use FIFO, 0=disable both FIFO and DMA,
 *                    1=use DMA ch 1, 3=use DMA ch 3)
 *  [3] - 0, 10=10MHz clock for 8254
 *            1= 1MHz clock for 8254
 *  [4] - 0,  5=D/A output 0-5V  (internal reference -5V)
 *        1, 10=D/A output 0-10V (internal reference -10V)
 *        2    =D/A output unknown (external reference)
 *
 *  Options for PCL-718:
 *  [0] - IO Base
 *  [1] - IRQ        (0=disable, 2, 3, 4, 5, 6, 7)
 *  [2] - DMA        (0=disable, 1, 3)
 *  [3] - 0, 10=10MHz clock for 8254
 *            1= 1MHz clock for 8254
 *  [4] -     0=A/D Range is +/-10V
 *            1=             +/-5V
 *            2=             +/-2.5V
 *            3=             +/-1V
 *            4=             +/-0.5V
 *            5=             user defined bipolar
 *            6=             0-10V
 *            7=             0-5V
 *            8=             0-2V
 *            9=             0-1V
 *           10=             user defined unipolar
 *  [5] - 0,  5=D/A outputs 0-5V  (internal reference -5V)
 *        1, 10=D/A outputs 0-10V (internal reference -10V)
 *            2=D/A outputs unknown (external reference)
 *  [6] - 0, 60=max  60kHz A/D sampling
 *        1,100=max 100kHz A/D sampling (PCL-718 with Option 001 installed)
 *
 */

#include <linux/module.h>
#include <linux/gfp.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/interrupt.h>

#include "../comedidev.h"

#include "comedi_isadma.h"
#include "comedi_fc.h"
#include "8253.h"

/* boards constants */

#define boardPCL818L 0
#define boardPCL818H 1
#define boardPCL818HD 2
#define boardPCL818HG 3
#define boardPCL818 4
#define boardPCL718 5

/*
 * Register I/O map
 */
#define PCL818_AI_LSB_REG			0x00
#define PCL818_AI_MSB_REG			0x01
#define PCL818_RANGE_REG			0x01
#define PCL818_MUX_REG				0x02
#define PCL818_MUX_SCAN(_first, _last)		(((_last) << 4) | (_first))
#define PCL818_DO_DI_LSB_REG			0x03
#define PCL818_AO_LSB_REG(x)			(0x04 + ((x) * 2))
#define PCL818_AO_MSB_REG(x)			(0x05 + ((x) * 2))
#define PCL818_STATUS_REG			0x08
#define PCL818_STATUS_NEXT_CHAN_MASK		(0xf << 0)
#define PCL818_STATUS_INT			(1 << 4)
#define PCL818_STATUS_MUX			(1 << 5)
#define PCL818_STATUS_UNI			(1 << 6)
#define PCL818_STATUS_EOC			(1 << 7)
#define PCL818_CTRL_REG				0x09
#define PCL818_CTRL_DISABLE_TRIG		(0 << 0)
#define PCL818_CTRL_SOFT_TRIG			(1 << 0)
#define PCL818_CTRL_EXT_TRIG			(2 << 0)
#define PCL818_CTRL_PACER_TRIG			(3 << 0)
#define PCL818_CTRL_DMAE			(1 << 2)
#define PCL818_CTRL_IRQ(x)			((x) << 4)
#define PCL818_CTRL_INTE			(1 << 7)
#define PCL818_CNTENABLE_REG			0x0a
#define PCL818_CNTENABLE_PACER_ENA		(0 << 0)
#define PCL818_CNTENABLE_PACER_TRIG0		(1 << 0)
#define PCL818_CNTENABLE_CNT0_EXT_CLK		(0 << 1)
#define PCL818_CNTENABLE_CNT0_INT_CLK		(1 << 1)
#define PCL818_DO_DI_MSB_REG			0x0b
#define PCL818_TIMER_BASE			0x0c

/* W: fifo enable/disable */
#define PCL818_FI_ENABLE 6
/* W: fifo interrupt clear */
#define PCL818_FI_INTCLR 20
/* W: fifo interrupt clear */
#define PCL818_FI_FLUSH 25
/* R: fifo status */
#define PCL818_FI_STATUS 25
/* R: one record from FIFO */
#define PCL818_FI_DATALO 23
*/
static void setup_channel_list(struct comedi_device *dev,
			       struct comedi_subdevice *s,
			       unsigned int *chanlist, unsigned int n_chan,
			       unsigned int seglen);
static int check_channel_list(struct comedi_device *dev,
			      struct comedi_subdevice *s,
			      unsigned int *chanlist, unsigned int n_chan);

static int pcl818_ai_cancel(struct comedi_device *dev,
			    struct comedi_subdevice *s);
static void start_pacer(struct comedi_device *dev, int mode,
			unsigned int divisor1, unsigned int divisor2);

#ifdef unused
static int set_rtc_irq_bit(unsigned char bit);
static void rtc_dropped_irq(unsigned long data);
static int rtc_setfreq_irq(int freq);
#endif

/*
==============================================================================
   ANALOG INPUT MODE0, 818 cards, slow version
*/
static int pcl818_ai_insn_read(struct comedi_device *dev,
			       struct comedi_subdevice *s,
			       struct comedi_insn *insn, unsigned int *data)
{
	int n;
	int timeout;

	/* software trigger, DMA and INT off */
	outb(0, dev->iobase + PCL818_CONTROL);

	/* select channel */
	outb(muxonechan[CR_CHAN(insn->chanspec)], dev->iobase + PCL818_MUX);

	/* select gain */
	outb(CR_RANGE(insn->chanspec), dev->iobase + PCL818_RANGE);

	for (n = 0; n < insn->n; n++) {

		/* clear INT (conversion end) flag */
		outb(0, dev->iobase + PCL818_CLRINT);

		/* start conversion */
		outb(0, dev->iobase + PCL818_AD_LO);

		timeout = 100;
		while (timeout--) {
			if (inb(dev->iobase + PCL818_STATUS) & 0x10)
				goto conv_finish;
			udelay(1);
		}
		comedi_error(dev, "A/D insn timeout");
		/* clear INT (conversion end) flag */
		outb(0, dev->iobase + PCL818_CLRINT);
		return -EIO;

conv_finish:
		data[n] = ((inb(dev->iobase + PCL818_AD_HI) << 4) |
			   (inb(dev->iobase + PCL818_AD_LO) >> 4));
	}

	return n;
}

/*
==============================================================================
   ANALOG OUTPUT MODE0, 818 cards
   only one sample per call is supported
*/
static int pcl818_ao_insn_read(struct comedi_device *dev,
			       struct comedi_subdevice *s,
			       struct comedi_insn *insn, unsigned int *data)
{
	int n;
	int chan = CR_CHAN(insn->chanspec);

	for (n = 0; n < insn->n; n++)
		data[n] = devpriv->ao_readback[chan];

	return n;
}

static int pcl818_ao_insn_write(struct comedi_device *dev,
				struct comedi_subdevice *s,
				struct comedi_insn *insn, unsigned int *data)
{
	int n;
	int chan = CR_CHAN(insn->chanspec);

	for (n = 0; n < insn->n; n++) {
		devpriv->ao_readback[chan] = data[n];
		outb((data[n] & 0x000f) << 4, dev->iobase +
		     (chan ? PCL718_DA2_LO : PCL818_DA_LO));
		outb((data[n] & 0x0ff0) >> 4, dev->iobase +
		     (chan ? PCL718_DA2_HI : PCL818_DA_HI));
	}

	return n;
}

/*
==============================================================================
   DIGITAL INPUT MODE0, 818 cards

   only one sample per call is supported
*/
static int pcl818_di_insn_bits(struct comedi_device *dev,
			       struct comedi_subdevice *s,
			       struct comedi_insn *insn, unsigned int *data)
{
	if (insn->n != 2)
		return -EINVAL;

	data[1] = inb(dev->iobase + PCL818_DI_LO) |
	    (inb(dev->iobase + PCL818_DI_HI) << 8);

	return 2;
}

/*
==============================================================================
   DIGITAL OUTPUT MODE0, 818 cards

   only one sample per call is supported
*/
static int pcl818_do_insn_bits(struct comedi_device *dev,
			       struct comedi_subdevice *s,
			       struct comedi_insn *insn, unsigned int *data)
{
	if (insn->n != 2)
		return -EINVAL;

	s->state &= ~data[0];
	s->state |= (data[0] & data[1]);

	outb(s->state & 0xff, dev->iobase + PCL818_DO_LO);
	outb((s->state >> 8), dev->iobase + PCL818_DO_HI);

	data[1] = s->state;

	return 2;
}

/*
==============================================================================
   analog input interrupt mode 1 & 3, 818 cards
   one sample per interrupt version
*/
static irqreturn_t interrupt_pcl818_ai_mode13_int(int irq, void *d)
{
	struct comedi_device *dev = d;
	struct comedi_subdevice *s = dev->subdevices + 0;
	int low;
	int timeout = 50;	/* wait max 50us */

	while (timeout--) {
		if (inb(dev->iobase + PCL818_STATUS) & 0x10)
			goto conv_finish;
		udelay(1);
	}
	outb(0, dev->iobase + PCL818_STATUS);	/* clear INT request */
	comedi_error(dev, "A/D mode1/3 IRQ without DRDY!");
	pcl818_ai_cancel(dev, s);
	s->async->events |= COMEDI_CB_EOA | COMEDI_CB_ERROR;
	comedi_event(dev, s);
	return IRQ_HANDLED;

conv_finish:
	low = inb(dev->iobase + PCL818_AD_LO);
	comedi_buf_put(s->async, ((inb(dev->iobase + PCL818_AD_HI) << 4) | (low >> 4)));	/*  get one sample */
	outb(0, dev->iobase + PCL818_CLRINT);	/* clear INT request */

	if ((low & 0xf) != devpriv->act_chanlist[devpriv->act_chanlist_pos]) {	/*  dropout! */
		printk
		    ("comedi: A/D mode1/3 IRQ - channel dropout %x!=%x !\n",
		     (low & 0xf),
		     devpriv->act_chanlist[devpriv->act_chanlist_pos]);
		pcl818_ai_cancel(dev, s);
		s->async->events |= COMEDI_CB_EOA | COMEDI_CB_ERROR;
		comedi_event(dev, s);
		return IRQ_HANDLED;
	}
=======
#define PCL818_FI_DATAHI 24

#define MAGIC_DMA_WORD 0x5a5a

static const struct comedi_lrange range_pcl818h_ai = {
	9, {
		BIP_RANGE(5),
		BIP_RANGE(2.5),
		BIP_RANGE(1.25),
		BIP_RANGE(0.625),
		UNI_RANGE(10),
		UNI_RANGE(5),
		UNI_RANGE(2.5),
		UNI_RANGE(1.25),
		BIP_RANGE(10)
	}
};

static const struct comedi_lrange range_pcl818hg_ai = {
	10, {
		BIP_RANGE(5),
		BIP_RANGE(0.5),
		BIP_RANGE(0.05),
		BIP_RANGE(0.005),
		UNI_RANGE(10),
		UNI_RANGE(1),
		UNI_RANGE(0.1),
		UNI_RANGE(0.01),
		BIP_RANGE(10),
		BIP_RANGE(1),
		BIP_RANGE(0.1),
		BIP_RANGE(0.01)
	}
};

static const struct comedi_lrange range_pcl818l_l_ai = {
	4, {
		BIP_RANGE(5),
		BIP_RANGE(2.5),
		BIP_RANGE(1.25),
		BIP_RANGE(0.625)
	}
};

static const struct comedi_lrange range_pcl818l_h_ai = {
	4, {
		BIP_RANGE(10),
		BIP_RANGE(5),
		BIP_RANGE(2.5),
		BIP_RANGE(1.25)
	}
};

static const struct comedi_lrange range718_bipolar1 = {
	1, {
		BIP_RANGE(1)
	}
};

static const struct comedi_lrange range718_bipolar0_5 = {
	1, {
		BIP_RANGE(0.5)
	}
};

static const struct comedi_lrange range718_unipolar2 = {
	1, {
		UNI_RANGE(2)
	}
};

static const struct comedi_lrange range718_unipolar1 = {
	1, {
		BIP_RANGE(1)
	}
};

struct pcl818_board {
	const char *name;
	unsigned int ns_min;
	int n_aochan;
	const struct comedi_lrange *ai_range_type;
	unsigned int has_dma:1;
	unsigned int has_fifo:1;
	unsigned int is_818:1;
};

static const struct pcl818_board boardtypes[] = {
	{
		.name		= "pcl818l",
		.ns_min		= 25000,
		.n_aochan	= 1,
		.ai_range_type	= &range_pcl818l_l_ai,
		.has_dma	= 1,
		.is_818		= 1,
	}, {
		.name		= "pcl818h",
		.ns_min		= 10000,
		.n_aochan	= 1,
		.ai_range_type	= &range_pcl818h_ai,
		.has_dma	= 1,
		.is_818		= 1,
	}, {
		.name		= "pcl818hd",
		.ns_min		= 10000,
		.n_aochan	= 1,
		.ai_range_type	= &range_pcl818h_ai,
		.has_dma	= 1,
		.has_fifo	= 1,
		.is_818		= 1,
	}, {
		.name		= "pcl818hg",
		.ns_min		= 10000,
		.n_aochan	= 1,
		.ai_range_type	= &range_pcl818hg_ai,
		.has_dma	= 1,
		.has_fifo	= 1,
		.is_818		= 1,
	}, {
		.name		= "pcl818",
		.ns_min		= 10000,
		.n_aochan	= 2,
		.ai_range_type	= &range_pcl818h_ai,
		.has_dma	= 1,
		.is_818		= 1,
	}, {
		.name		= "pcl718",
		.ns_min		= 16000,
		.n_aochan	= 2,
		.ai_range_type	= &range_unipolar5,
		.has_dma	= 1,
	}, {
		.name		= "pcm3718",
		.ns_min		= 10000,
		.ai_range_type	= &range_pcl818h_ai,
		.has_dma	= 1,
		.is_818		= 1,
	},
};

struct pcl818_private {
	struct comedi_isadma *dma;
	/*  manimal allowed delay between samples (in us) for actual card */
	unsigned int ns_min;
	int i8253_osc_base;	/*  1/frequency of on board oscilator in ns */
	/*  MUX setting for actual AI operations */
	unsigned int act_chanlist[16];
	unsigned int act_chanlist_len;	/*  how long is actual MUX list */
	unsigned int act_chanlist_pos;	/*  actual position in MUX list */
	unsigned int divisor1;
	unsigned int divisor2;
	unsigned int usefifo:1;
	unsigned int ai_cmd_running:1;
	unsigned int ai_cmd_canceled:1;
};

static void pcl818_start_pacer(struct comedi_device *dev, bool load_counters)
{
	struct pcl818_private *devpriv = dev->private;
	unsigned long timer_base = dev->iobase + PCL818_TIMER_BASE;

	i8254_set_mode(timer_base, 0, 2, I8254_MODE2 | I8254_BINARY);
	i8254_set_mode(timer_base, 0, 1, I8254_MODE2 | I8254_BINARY);
	udelay(1);

	if (load_counters) {
		i8254_write(timer_base, 0, 2, devpriv->divisor2);
		i8254_write(timer_base, 0, 1, devpriv->divisor1);
	}
}

static void pcl818_ai_setup_dma(struct comedi_device *dev,
				struct comedi_subdevice *s,
				unsigned int unread_samples)
{
	struct pcl818_private *devpriv = dev->private;
	struct comedi_isadma *dma = devpriv->dma;
	struct comedi_isadma_desc *desc = &dma->desc[dma->cur_dma];
	unsigned int max_samples = comedi_bytes_to_samples(s, desc->maxsize);
	unsigned int nsamples;

	comedi_isadma_disable(dma->chan);

	/*
	 * Determine dma size based on the buffer maxsize plus the number of
	 * unread samples and the number of samples remaining in the command.
	 */
	nsamples = comedi_nsamples_left(s, max_samples + unread_samples);
	if (nsamples > unread_samples) {
		nsamples -= unread_samples;
		desc->size = comedi_samples_to_bytes(s, nsamples);
		comedi_isadma_program(desc);
	}
}

static void pcl818_ai_set_chan_range(struct comedi_device *dev,
				     unsigned int chan,
				     unsigned int range)
{
	outb(chan, dev->iobase + PCL818_MUX_REG);
	outb(range, dev->iobase + PCL818_RANGE_REG);
}

static void pcl818_ai_set_chan_scan(struct comedi_device *dev,
				    unsigned int first_chan,
				    unsigned int last_chan)
{
	outb(PCL818_MUX_SCAN(first_chan, last_chan),
	     dev->iobase + PCL818_MUX_REG);
}

static void pcl818_ai_setup_chanlist(struct comedi_device *dev,
				     unsigned int *chanlist,
				     unsigned int seglen)
{
	struct pcl818_private *devpriv = dev->private;
	unsigned int first_chan = CR_CHAN(chanlist[0]);
	unsigned int last_chan;
	unsigned int range;
	int i;

	devpriv->act_chanlist_len = seglen;
	devpriv->act_chanlist_pos = 0;

	/* store range list to card */
	for (i = 0; i < seglen; i++) {
		last_chan = CR_CHAN(chanlist[i]);
		range = CR_RANGE(chanlist[i]);

		devpriv->act_chanlist[i] = last_chan;

		pcl818_ai_set_chan_range(dev, last_chan, range);
	}

	udelay(1);

	pcl818_ai_set_chan_scan(dev, first_chan, last_chan);
}

static void pcl818_ai_clear_eoc(struct comedi_device *dev)
{
	/* writing any value clears the interrupt request */
	outb(0, dev->iobase + PCL818_STATUS_REG);
}

static void pcl818_ai_soft_trig(struct comedi_device *dev)
{
	/* writing any value triggers a software conversion */
	outb(0, dev->iobase + PCL818_AI_LSB_REG);
}

static unsigned int pcl818_ai_get_fifo_sample(struct comedi_device *dev,
					      struct comedi_subdevice *s,
					      unsigned int *chan)
{
	unsigned int val;

	val = inb(dev->iobase + PCL818_FI_DATALO);
	val |= (inb(dev->iobase + PCL818_FI_DATAHI) << 8);

	if (chan)
		*chan = val & 0xf;

	return (val >> 4) & s->maxdata;
}

static unsigned int pcl818_ai_get_sample(struct comedi_device *dev,
					 struct comedi_subdevice *s,
					 unsigned int *chan)
{
	unsigned int val;

	val = inb(dev->iobase + PCL818_AI_MSB_REG) << 8;
	val |= inb(dev->iobase + PCL818_AI_LSB_REG);

	if (chan)
		*chan = val & 0xf;

	return (val >> 4) & s->maxdata;
}

static int pcl818_ai_eoc(struct comedi_device *dev,
			 struct comedi_subdevice *s,
			 struct comedi_insn *insn,
			 unsigned long context)
{
	unsigned int status;

	status = inb(dev->iobase + PCL818_STATUS_REG);
	if (status & PCL818_STATUS_INT)
		return 0;
	return -EBUSY;
}

static bool pcl818_ai_write_sample(struct comedi_device *dev,
				   struct comedi_subdevice *s,
				   unsigned int chan, unsigned int val)
{
	struct pcl818_private *devpriv = dev->private;
	struct comedi_cmd *cmd = &s->async->cmd;
	unsigned int expected_chan;

	expected_chan = devpriv->act_chanlist[devpriv->act_chanlist_pos];
	if (chan != expected_chan) {
		dev_dbg(dev->class_dev,
			"A/D mode1/3 %s - channel dropout %d!=%d !\n",
			(devpriv->dma) ? "DMA" :
			(devpriv->usefifo) ? "FIFO" : "IRQ",
			chan, expected_chan);
		s->async->events |= COMEDI_CB_ERROR;
		return false;
	}

	comedi_buf_write_samples(s, &val, 1);

	devpriv->act_chanlist_pos++;
	if (devpriv->act_chanlist_pos >= devpriv->act_chanlist_len)
		devpriv->act_chanlist_pos = 0;

   analog input dma mode 1 & 3, 818 cards
*/
static irqreturn_t interrupt_pcl818_ai_mode13_dma(int irq, void *d)
{
	struct comedi_device *dev = d;
	struct comedi_subdevice *s = dev->subdevices + 0;
	int i, len, bufptr;
	unsigned long flags;
	short *ptr;

	disable_dma(devpriv->dma);
	devpriv->next_dma_buf = 1 - devpriv->next_dma_buf;
	if ((devpriv->dma_runs_to_end) > -1 || devpriv->neverending_ai) {	/*  switch dma bufs */
		set_dma_mode(devpriv->dma, DMA_MODE_READ);
		flags = claim_dma_lock();
		set_dma_addr(devpriv->dma,
			     devpriv->hwdmaptr[devpriv->next_dma_buf]);
		if (devpriv->dma_runs_to_end || devpriv->neverending_ai) {
			set_dma_count(devpriv->dma,
				      devpriv->hwdmasize[devpriv->
							 next_dma_buf]);
		} else {
			set_dma_count(devpriv->dma, devpriv->last_dma_run);
		}
		release_dma_lock(flags);
		enable_dma(devpriv->dma);
	}
	printk("comedi: A/D mode1/3 IRQ \n");

	devpriv->dma_runs_to_end--;
	outb(0, dev->iobase + PCL818_CLRINT);	/* clear INT request */
	ptr = (short *)devpriv->dmabuf[1 - devpriv->next_dma_buf];

	len = devpriv->hwdmasize[0] >> 1;
	bufptr = 0;

	for (i = 0; i < len; i++) {
		if ((ptr[bufptr] & 0xf) != devpriv->act_chanlist[devpriv->act_chanlist_pos]) {	/*  dropout! */
			printk
			    ("comedi: A/D mode1/3 DMA - channel dropout %d(card)!=%d(chanlist) at %d !\n",
			     (ptr[bufptr] & 0xf),
			     devpriv->act_chanlist[devpriv->act_chanlist_pos],
			     devpriv->act_chanlist_pos);
			pcl818_ai_cancel(dev, s);
			s->async->events |= COMEDI_CB_EOA | COMEDI_CB_ERROR;
			comedi_event(dev, s);
			return IRQ_HANDLED;
		}

		comedi_buf_put(s->async, ptr[bufptr++] >> 4);	/*  get one sample */

		devpriv->act_chanlist_pos++;
		if (devpriv->act_chanlist_pos >= devpriv->act_chanlist_len)
			devpriv->act_chanlist_pos = 0;

		s->async->cur_chan++;
		if (s->async->cur_chan >= devpriv->ai_n_chan) {
			s->async->cur_chan = 0;
			devpriv->ai_act_scan--;
		}

		if (!devpriv->neverending_ai)
			if (devpriv->ai_act_scan == 0) {	/* all data sampled */
				pcl818_ai_cancel(dev, s);
				s->async->events |= COMEDI_CB_EOA;
				comedi_event(dev, s);
				/*  printk("done int ai13 dma\n"); */
				return IRQ_HANDLED;
			}
	}

	if (len > 0)
		comedi_event(dev, s);
	return IRQ_HANDLED;
}

#ifdef unused
/*
==============================================================================
   analog input dma mode 1 & 3 over RTC, 818 cards
*/
static irqreturn_t interrupt_pcl818_ai_mode13_dma_rtc(int irq, void *d)
{
	struct comedi_device *dev = d;
	struct comedi_subdevice *s = dev->subdevices + 0;
	unsigned long tmp;
	unsigned int top1, top2, i, bufptr;
	long ofs_dats;
	short *dmabuf = (short *)devpriv->dmabuf[0];

	/* outb(2,0x378); */
	switch (devpriv->ai_mode) {
	case INT_TYPE_AI1_DMA_RTC:
	case INT_TYPE_AI3_DMA_RTC:
		tmp = (CMOS_READ(RTC_INTR_FLAGS) & 0xF0);
		mod_timer(&devpriv->rtc_irq_timer,
			  jiffies + HZ / devpriv->rtc_freq + 2 * HZ / 100);

		for (i = 0; i < 10; i++) {
			top1 = get_dma_residue(devpriv->dma);
			top2 = get_dma_residue(devpriv->dma);
			if (top1 == top2)
				break;
		}

		if (top1 != top2)
			return IRQ_HANDLED;
		top1 = devpriv->hwdmasize[0] - top1;	/*  where is now DMA in buffer */
		top1 >>= 1;
		ofs_dats = top1 - devpriv->last_top_dma;	/*  new samples from last call */
		if (ofs_dats < 0)
			ofs_dats = (devpriv->dmasamplsize) + ofs_dats;
		if (!ofs_dats)
			return IRQ_HANDLED;	/*  exit=no new samples from last call */
		/*  obsluz data */
		i = devpriv->last_top_dma - 1;
		i &= (devpriv->dmasamplsize - 1);

		if (dmabuf[i] != MAGIC_DMA_WORD) {	/*  DMA overflow! */
			comedi_error(dev, "A/D mode1/3 DMA buffer overflow!");
			/* printk("I %d dmabuf[i] %d %d\n",i,dmabuf[i],devpriv->dmasamplsize); */
			pcl818_ai_cancel(dev, s);
			s->async->events |= COMEDI_CB_EOA | COMEDI_CB_ERROR;
			comedi_event(dev, s);
			return IRQ_HANDLED;
		}
		/* printk("r %ld ",ofs_dats); */

		bufptr = devpriv->last_top_dma;

		for (i = 0; i < ofs_dats; i++) {
			if ((dmabuf[bufptr] & 0xf) != devpriv->act_chanlist[devpriv->act_chanlist_pos]) {	/*  dropout! */
				printk
				    ("comedi: A/D mode1/3 DMA - channel dropout %d!=%d !\n",
				     (dmabuf[bufptr] & 0xf),
				     devpriv->
				     act_chanlist[devpriv->act_chanlist_pos]);
				pcl818_ai_cancel(dev, s);
				s->async->events |=
				    COMEDI_CB_EOA | COMEDI_CB_ERROR;
				comedi_event(dev, s);
				return IRQ_HANDLED;
			}

			comedi_buf_put(s->async, dmabuf[bufptr++] >> 4);	/*  get one sample */
			bufptr &= (devpriv->dmasamplsize - 1);

			devpriv->act_chanlist_pos++;
			if (devpriv->act_chanlist_pos >=
					devpriv->act_chanlist_len) {
				devpriv->act_chanlist_pos = 0;
			}
			s->async->cur_chan++;
			if (s->async->cur_chan >= devpriv->ai_n_chan) {
				s->async->cur_chan = 0;
				devpriv->ai_act_scan--;
			}

			if (!devpriv->neverending_ai)
				if (devpriv->ai_act_scan == 0) {	/* all data sampled */
					pcl818_ai_cancel(dev, s);
					s->async->events |= COMEDI_CB_EOA;
					comedi_event(dev, s);
					/* printk("done int ai13 dma\n"); */
					return IRQ_HANDLED;
				}
		}

		devpriv->last_top_dma = bufptr;
		bufptr--;
		bufptr &= (devpriv->dmasamplsize - 1);
		dmabuf[bufptr] = MAGIC_DMA_WORD;
		comedi_event(dev, s);
		/* outb(0,0x378); */
		return IRQ_HANDLED;
	}

	/* outb(0,0x378); */
	return IRQ_HANDLED;
}
#endif

/*
==============================================================================
   analog input interrupt mode 1 & 3, 818HD/HG cards
*/
static irqreturn_t interrupt_pcl818_ai_mode13_fifo(int irq, void *d)
{
	struct comedi_device *dev = d;
	struct comedi_subdevice *s = dev->subdevices + 0;
	int i, len, lo;

	outb(0, dev->iobase + PCL818_FI_INTCLR);	/*  clear fifo int request */

	lo = inb(dev->iobase + PCL818_FI_STATUS);

	if (lo & 4) {
		comedi_error(dev, "A/D mode1/3 FIFO overflow!");
		pcl818_ai_cancel(dev, s);
		s->async->events |= COMEDI_CB_EOA | COMEDI_CB_ERROR;
		comedi_event(dev, s);
		return IRQ_HANDLED;
	}

	if (lo & 1) {
		comedi_error(dev, "A/D mode1/3 FIFO interrupt without data!");
		pcl818_ai_cancel(dev, s);
		s->async->events |= COMEDI_CB_EOA | COMEDI_CB_ERROR;
		comedi_event(dev, s);
		return IRQ_HANDLED;
	}

	if (lo & 2)
=======
	if (cmd->stop_src == TRIG_COUNT &&
	    s->async->scans_done >= cmd->stop_arg) {
		s->async->events |= COMEDI_CB_EOA;
		return false;
	}

	return true;
}

static void pcl818_handle_eoc(struct comedi_device *dev,
			      struct comedi_subdevice *s)
{
	unsigned int chan;
	unsigned int val;

	if (pcl818_ai_eoc(dev, s, NULL, 0)) {
		dev_err(dev->class_dev, "A/D mode1/3 IRQ without DRDY!\n");
		s->async->events |= COMEDI_CB_ERROR;
		return;
	}

	val = pcl818_ai_get_sample(dev, s, &chan);
	pcl818_ai_write_sample(dev, s, chan, val);
}

static void pcl818_handle_dma(struct comedi_device *dev,
			      struct comedi_subdevice *s)
{
	struct pcl818_private *devpriv = dev->private;
	struct comedi_isadma *dma = devpriv->dma;
	struct comedi_isadma_desc *desc = &dma->desc[dma->cur_dma];
	unsigned short *ptr = desc->virt_addr;
	unsigned int nsamples = comedi_bytes_to_samples(s, desc->size);
	unsigned int chan;
	unsigned int val;
	int i;

	/* restart dma with the next buffer */
	dma->cur_dma = 1 - dma->cur_dma;
	pcl818_ai_setup_dma(dev, s, nsamples);

	for (i = 0; i < nsamples; i++) {
		val = ptr[i];
		chan = val & 0xf;
		val = (val >> 4) & s->maxdata;
		if (!pcl818_ai_write_sample(dev, s, chan, val))
			break;
	}
}

static void pcl818_handle_fifo(struct comedi_device *dev,
			       struct comedi_subdevice *s)
{
	unsigned int status;
	unsigned int chan;
	unsigned int val;
	int i, len;

	status = inb(dev->iobase + PCL818_FI_STATUS);

	if (status & 4) {
		dev_err(dev->class_dev, "A/D mode1/3 FIFO overflow!\n");
		s->async->events |= COMEDI_CB_ERROR;
		return;
	}

	if (status & 1) {
		dev_err(dev->class_dev,
			"A/D mode1/3 FIFO interrupt without data!\n");
		s->async->events |= COMEDI_CB_ERROR;
		return;
	}

	if (status & 2)
		len = 512;
	else
		len = 0;

	for (i = 0; i < len; i++) {
    INT procedure
*/
static irqreturn_t interrupt_pcl818(int irq, void *d)
{
	struct comedi_device *dev = d;

	if (!dev->attached) {
		comedi_error(dev, "premature interrupt");
		return IRQ_HANDLED;
	}
	/* printk("I\n"); */

	if (devpriv->irq_blocked && devpriv->irq_was_now_closed) {
		if ((devpriv->neverending_ai || (!devpriv->neverending_ai &&
						 devpriv->ai_act_scan > 0)) &&
		    (devpriv->ai_mode == INT_TYPE_AI1_DMA ||
		     devpriv->ai_mode == INT_TYPE_AI3_DMA)) {
			/* The cleanup from ai_cancel() has been delayed
			   until now because the card doesn't seem to like
			   being reprogrammed while a DMA transfer is in
			   progress.
			 */
			struct comedi_subdevice *s = dev->subdevices + 0;
			devpriv->ai_act_scan = 0;
			devpriv->neverending_ai = 0;
			pcl818_ai_cancel(dev, s);
		}

		outb(0, dev->iobase + PCL818_CLRINT);	/* clear INT request */

		return IRQ_HANDLED;
	}

	switch (devpriv->ai_mode) {
	case INT_TYPE_AI1_DMA:
	case INT_TYPE_AI3_DMA:
		return interrupt_pcl818_ai_mode13_dma(irq, d);
	case INT_TYPE_AI1_INT:
	case INT_TYPE_AI3_INT:
		return interrupt_pcl818_ai_mode13_int(irq, d);
	case INT_TYPE_AI1_FIFO:
	case INT_TYPE_AI3_FIFO:
		return interrupt_pcl818_ai_mode13_fifo(irq, d);
#ifdef PCL818_MODE13_AO
	case INT_TYPE_AO1_INT:
	case INT_TYPE_AO3_INT:
		return interrupt_pcl818_ao_mode13_int(irq, d);
#endif
	default:
		break;
	}

	outb(0, dev->iobase + PCL818_CLRINT);	/* clear INT request */

	if ((!dev->irq) || (!devpriv->irq_free) || (!devpriv->irq_blocked)
	    || (!devpriv->ai_mode)) {
		comedi_error(dev, "bad IRQ!");
		return IRQ_NONE;
	}

	comedi_error(dev, "IRQ from unknown source!");
	return IRQ_NONE;
}

/*
==============================================================================
   ANALOG INPUT MODE 1 or 3 DMA , 818 cards
*/
static void pcl818_ai_mode13dma_int(int mode, struct comedi_device *dev,
				    struct comedi_subdevice *s)
{
	unsigned int flags;
	unsigned int bytes;

	printk("mode13dma_int, mode: %d\n", mode);
	disable_dma(devpriv->dma);	/*  disable dma */
	bytes = devpriv->hwdmasize[0];
	if (!devpriv->neverending_ai) {
		bytes = devpriv->ai_n_chan * devpriv->ai_scans * sizeof(short);	/*  how many */
		devpriv->dma_runs_to_end = bytes / devpriv->hwdmasize[0];	/*  how many DMA pages we must fiil */
		devpriv->last_dma_run = bytes % devpriv->hwdmasize[0];	/* on last dma transfer must be moved */
		devpriv->dma_runs_to_end--;
		if (devpriv->dma_runs_to_end >= 0)
			bytes = devpriv->hwdmasize[0];
	}

	devpriv->next_dma_buf = 0;
	set_dma_mode(devpriv->dma, DMA_MODE_READ);
	flags = claim_dma_lock();
	clear_dma_ff(devpriv->dma);
	set_dma_addr(devpriv->dma, devpriv->hwdmaptr[0]);
	set_dma_count(devpriv->dma, bytes);
	release_dma_lock(flags);
	enable_dma(devpriv->dma);

	if (mode == 1) {
		devpriv->ai_mode = INT_TYPE_AI1_DMA;
		outb(0x87 | (dev->irq << 4), dev->iobase + PCL818_CONTROL);	/* Pacer+IRQ+DMA */
	} else {
		devpriv->ai_mode = INT_TYPE_AI3_DMA;
		outb(0x86 | (dev->irq << 4), dev->iobase + PCL818_CONTROL);	/* Ext trig+IRQ+DMA */
	};
}

#ifdef unused
/*
==============================================================================
   ANALOG INPUT MODE 1 or 3 DMA rtc, 818 cards
*/
static void pcl818_ai_mode13dma_rtc(int mode, struct comedi_device *dev,
				    struct comedi_subdevice *s)
{
	unsigned int flags;
	short *pole;

	set_dma_mode(devpriv->dma, DMA_MODE_READ | DMA_AUTOINIT);
	flags = claim_dma_lock();
	clear_dma_ff(devpriv->dma);
	set_dma_addr(devpriv->dma, devpriv->hwdmaptr[0]);
	set_dma_count(devpriv->dma, devpriv->hwdmasize[0]);
	release_dma_lock(flags);
	enable_dma(devpriv->dma);
	devpriv->last_top_dma = 0;	/* devpriv->hwdmasize[0]; */
	pole = (short *)devpriv->dmabuf[0];
	devpriv->dmasamplsize = devpriv->hwdmasize[0] / 2;
	pole[devpriv->dmasamplsize - 1] = MAGIC_DMA_WORD;
#ifdef unused
	devpriv->rtc_freq = rtc_setfreq_irq(2048);
	devpriv->rtc_irq_timer.expires =
	    jiffies + HZ / devpriv->rtc_freq + 2 * HZ / 100;
	devpriv->rtc_irq_timer.data = (unsigned long)dev;
	devpriv->rtc_irq_timer.function = rtc_dropped_irq;

	add_timer(&devpriv->rtc_irq_timer);
#endif

	if (mode == 1) {
		devpriv->int818_mode = INT_TYPE_AI1_DMA_RTC;
		outb(0x07 | (dev->irq << 4), dev->iobase + PCL818_CONTROL);	/* Pacer+DMA */
	} else {
		devpriv->int818_mode = INT_TYPE_AI3_DMA_RTC;
		outb(0x06 | (dev->irq << 4), dev->iobase + PCL818_CONTROL);	/* Ext trig+DMA */
	};
}
#endif

/*
==============================================================================
   ANALOG INPUT MODE 1 or 3, 818 cards
*/
static int pcl818_ai_cmd_mode(int mode, struct comedi_device *dev,
			      struct comedi_subdevice *s)
{
	struct comedi_cmd *cmd = &s->async->cmd;
	int divisor1 = 0, divisor2 = 0;
	unsigned int seglen;

	dev_dbg(dev->hw_dev, "pcl818_ai_cmd_mode()\n");
	if ((!dev->irq) && (!devpriv->dma_rtc)) {
		comedi_error(dev, "IRQ not defined!");
		return -EINVAL;
	}

	if (devpriv->irq_blocked)
		return -EBUSY;

	start_pacer(dev, -1, 0, 0);	/*  stop pacer */

	seglen = check_channel_list(dev, s, devpriv->ai_chanlist,
				    devpriv->ai_n_chan);
	if (seglen < 1)
		return -EINVAL;
	setup_channel_list(dev, s, devpriv->ai_chanlist,
			   devpriv->ai_n_chan, seglen);

	udelay(1);

	devpriv->ai_act_scan = devpriv->ai_scans;
	devpriv->ai_act_chan = 0;
	devpriv->irq_blocked = 1;
	devpriv->irq_was_now_closed = 0;
	devpriv->neverending_ai = 0;
	devpriv->act_chanlist_pos = 0;
	devpriv->dma_runs_to_end = 0;

	if ((devpriv->ai_scans == 0) || (devpriv->ai_scans == -1))
		devpriv->neverending_ai = 1;	/* well, user want neverending */

	if (mode == 1) {
		i8253_cascade_ns_to_timer(devpriv->i8253_osc_base, &divisor1,
					  &divisor2, &cmd->convert_arg,
					  TRIG_ROUND_NEAREST);
		if (divisor1 == 1) {	/* PCL718/818 crash if any divisor is set to 1 */
			divisor1 = 2;
			divisor2 /= 2;
		}
		if (divisor2 == 1) {
			divisor2 = 2;
			divisor1 /= 2;
		}
	}

	outb(0, dev->iobase + PCL818_CNTENABLE);	/* enable pacer */

	switch (devpriv->dma) {
	case 1:		/*  DMA */
	case 3:
		if (devpriv->dma_rtc == 0) {
			pcl818_ai_mode13dma_int(mode, dev, s);
		}
#ifdef unused
		else {
			pcl818_ai_mode13dma_rtc(mode, dev, s);
		}
#else
		else {
			return -EINVAL;
		}
#endif
		break;
	case 0:
		if (!devpriv->usefifo) {
			/* IRQ */
			/* printk("IRQ\n"); */
			if (mode == 1) {
				devpriv->ai_mode = INT_TYPE_AI1_INT;
				/* Pacer+IRQ */
				outb(0x83 | (dev->irq << 4),
				     dev->iobase + PCL818_CONTROL);
			} else {
				devpriv->ai_mode = INT_TYPE_AI3_INT;
				/* Ext trig+IRQ */
				outb(0x82 | (dev->irq << 4),
				     dev->iobase + PCL818_CONTROL);
			}
		} else {
			/* FIFO */
			/* enable FIFO */
			outb(1, dev->iobase + PCL818_FI_ENABLE);
			if (mode == 1) {
				devpriv->ai_mode = INT_TYPE_AI1_FIFO;
				/* Pacer */
				outb(0x03, dev->iobase + PCL818_CONTROL);
			} else {
				devpriv->ai_mode = INT_TYPE_AI3_FIFO;
				outb(0x02, dev->iobase + PCL818_CONTROL);
			}
		}
	}

	start_pacer(dev, mode, divisor1, divisor2);

#ifdef unused
	switch (devpriv->ai_mode) {
	case INT_TYPE_AI1_DMA_RTC:
	case INT_TYPE_AI3_DMA_RTC:
		set_rtc_irq_bit(1);	/* start RTC */
		break;
	}
#endif
	dev_dbg(dev->hw_dev, "pcl818_ai_cmd_mode() end\n");
	return 0;
}

#ifdef unused
/*
==============================================================================
   ANALOG OUTPUT MODE 1 or 3, 818 cards
*/
#ifdef PCL818_MODE13_AO
static int pcl818_ao_mode13(int mode, struct comedi_device *dev,
			    struct comedi_subdevice *s, comedi_trig * it)
{
	int divisor1 = 0, divisor2 = 0;

	if (!dev->irq) {
		comedi_error(dev, "IRQ not defined!");
		return -EINVAL;
	}

	if (devpriv->irq_blocked)
		return -EBUSY;

	start_pacer(dev, -1, 0, 0);	/*  stop pacer */

	devpriv->int13_act_scan = it->n;
	devpriv->int13_act_chan = 0;
	devpriv->irq_blocked = 1;
	devpriv->irq_was_now_closed = 0;
	devpriv->neverending_ai = 0;
	devpriv->act_chanlist_pos = 0;

	if (mode == 1) {
		i8253_cascade_ns_to_timer(devpriv->i8253_osc_base, &divisor1,
					  &divisor2, &it->trigvar,
					  TRIG_ROUND_NEAREST);
		if (divisor1 == 1) {	/* PCL818 crash if any divisor is set to 1 */
			divisor1 = 2;
			divisor2 /= 2;
		}
		if (divisor2 == 1) {
			divisor2 = 2;
			divisor1 /= 2;
		}
	}

	outb(0, dev->iobase + PCL818_CNTENABLE);	/* enable pacer */
	if (mode == 1) {
		devpriv->int818_mode = INT_TYPE_AO1_INT;
		outb(0x83 | (dev->irq << 4), dev->iobase + PCL818_CONTROL);	/* Pacer+IRQ */
	} else {
		devpriv->int818_mode = INT_TYPE_AO3_INT;
		outb(0x82 | (dev->irq << 4), dev->iobase + PCL818_CONTROL);	/* Ext trig+IRQ */
	};

	start_pacer(dev, mode, divisor1, divisor2);

	return 0;
}

/*
==============================================================================
   ANALOG OUTPUT MODE 1, 818 cards
*/
static int pcl818_ao_mode1(struct comedi_device *dev,
			   struct comedi_subdevice *s, comedi_trig * it)
{
	return pcl818_ao_mode13(1, dev, s, it);
}

/*
==============================================================================
   ANALOG OUTPUT MODE 3, 818 cards
*/
static int pcl818_ao_mode3(struct comedi_device *dev,
			   struct comedi_subdevice *s, comedi_trig * it)
{
	return pcl818_ao_mode13(3, dev, s, it);
}
#endif
#endif

/*
==============================================================================
 Start/stop pacer onboard pacer
*/
static void start_pacer(struct comedi_device *dev, int mode,
			unsigned int divisor1, unsigned int divisor2)
{
	outb(0xb4, dev->iobase + PCL818_CTRCTL);
	outb(0x74, dev->iobase + PCL818_CTRCTL);
	udelay(1);

	if (mode == 1) {
		outb(divisor2 & 0xff, dev->iobase + PCL818_CTR2);
		outb((divisor2 >> 8) & 0xff, dev->iobase + PCL818_CTR2);
		outb(divisor1 & 0xff, dev->iobase + PCL818_CTR1);
		outb((divisor1 >> 8) & 0xff, dev->iobase + PCL818_CTR1);
	}
}

/*
==============================================================================
 Check if channel list from user is builded correctly
 If it's ok, then program scan/gain logic
*/
=======
		val = pcl818_ai_get_fifo_sample(dev, s, &chan);
		if (!pcl818_ai_write_sample(dev, s, chan, val))
			break;
	}
}

static irqreturn_t pcl818_interrupt(int irq, void *d)
{
	struct comedi_device *dev = d;
	struct pcl818_private *devpriv = dev->private;
	struct comedi_subdevice *s = dev->read_subdev;
	struct comedi_cmd *cmd = &s->async->cmd;

	if (!dev->attached || !devpriv->ai_cmd_running) {
		pcl818_ai_clear_eoc(dev);
		return IRQ_HANDLED;
	}

	if (devpriv->ai_cmd_canceled) {
		/*
		 * The cleanup from ai_cancel() has been delayed
		 * until now because the card doesn't seem to like
		 * being reprogrammed while a DMA transfer is in
		 * progress.
		 */
		s->async->scans_done = cmd->stop_arg;
		s->cancel(dev, s);
		return IRQ_HANDLED;
	}

	if (devpriv->dma)
		pcl818_handle_dma(dev, s);
	else if (devpriv->usefifo)
		pcl818_handle_fifo(dev, s);
	else
		pcl818_handle_eoc(dev, s);

	pcl818_ai_clear_eoc(dev);

	comedi_handle_events(dev, s);
	return IRQ_HANDLED;
}

static int check_channel_list(struct comedi_device *dev,
			      struct comedi_subdevice *s,
			      unsigned int *chanlist, unsigned int n_chan)
{
	unsigned int chansegment[16];
	unsigned int i, nowmustbechan, seglen, segpos;

	/* correct channel and range number check itself comedi/range.c */
	if (n_chan < 1) {
		dev_err(dev->class_dev, "range/channel list is empty!\n");
		return 0;
	}

	if (n_chan > 1) {
		/*  first channel is every time ok */
		chansegment[0] = chanlist[0];
		/*  build part of chanlist */
		for (i = 1, seglen = 1; i < n_chan; i++, seglen++) {
			/* we detect loop, this must by finish */

			if (chanlist[0] == chanlist[i])
				break;
			nowmustbechan =
			    (CR_CHAN(chansegment[i - 1]) + 1) % s->n_chan;
			if (nowmustbechan != CR_CHAN(chanlist[i])) {
				/*  channel list isn't continuous :-( */
				dev_dbg(dev->class_dev,
					"channel list must be continuous! chanlist[%i]=%d but must be %d or %d!\n",
					i, CR_CHAN(chanlist[i]), nowmustbechan,
					CR_CHAN(chanlist[0]));
				return 0;
			}
			/*  well, this is next correct channel in list */
			chansegment[i] = chanlist[i];
		}

		/*  check whole chanlist */
		for (i = 0, segpos = 0; i < n_chan; i++) {
			if (chanlist[i] != chansegment[i % seglen]) {
				dev_dbg(dev->class_dev,
					"bad channel or range number! chanlist[%i]=%d,%d,%d and not %d,%d,%d!\n",
					i, CR_CHAN(chansegment[i]),
					CR_RANGE(chansegment[i]),
					CR_AREF(chansegment[i]),
					CR_CHAN(chanlist[i % seglen]),
					CR_RANGE(chanlist[i % seglen]),
					CR_AREF(chansegment[i % seglen]));
				return 0;	/*  chan/gain list is strange */
			}
		}
	} else {
		seglen = 1;
	}
 Check if board is switched to SE (1) or DIFF(0) mode
*/
static int check_single_ended(unsigned int port)
{
	if (inb(port + PCL818_STATUS) & 0x20)
=======
	return seglen;
}

static int check_single_ended(unsigned int port)
{
	if (inb(port + PCL818_STATUS_REG) & PCL818_STATUS_MUX)
		return 1;
	return 0;
}

*/
static int ai_cmdtest(struct comedi_device *dev, struct comedi_subdevice *s,
		      struct comedi_cmd *cmd)
{
	int err = 0;
	int tmp, divisor1 = 0, divisor2 = 0;

	/* step 1: make sure trigger sources are trivially valid */

	tmp = cmd->start_src;
	cmd->start_src &= TRIG_NOW;
	if (!cmd->start_src || tmp != cmd->start_src)
		err++;

	tmp = cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_FOLLOW;
	if (!cmd->scan_begin_src || tmp != cmd->scan_begin_src)
		err++;

	tmp = cmd->convert_src;
	cmd->convert_src &= TRIG_TIMER | TRIG_EXT;
	if (!cmd->convert_src || tmp != cmd->convert_src)
		err++;

	tmp = cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if (!cmd->scan_end_src || tmp != cmd->scan_end_src)
		err++;

	tmp = cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT | TRIG_NONE;
	if (!cmd->stop_src || tmp != cmd->stop_src)
		err++;
=======
static int ai_cmdtest(struct comedi_device *dev, struct comedi_subdevice *s,
		      struct comedi_cmd *cmd)
{
	const struct pcl818_board *board = dev->board_ptr;
	struct pcl818_private *devpriv = dev->private;
	int err = 0;
	unsigned int arg;

	/* Step 1 : check if triggers are trivially valid */

	err |= cfc_check_trigger_src(&cmd->start_src, TRIG_NOW);
	err |= cfc_check_trigger_src(&cmd->scan_begin_src, TRIG_FOLLOW);
	err |= cfc_check_trigger_src(&cmd->convert_src, TRIG_TIMER | TRIG_EXT);
	err |= cfc_check_trigger_src(&cmd->scan_end_src, TRIG_COUNT);
	err |= cfc_check_trigger_src(&cmd->stop_src, TRIG_COUNT | TRIG_NONE);

	if (err)
		return 1;

	/* Step 2a : make sure trigger sources are unique */

	err |= cfc_check_trigger_is_unique(cmd->convert_src);
	err |= cfc_check_trigger_is_unique(cmd->stop_src);

	/* Step 2b : and mutually compatible */

	if (err)
		return 2;

	/* Step 3: check if arguments are trivially valid */

	err |= cfc_check_trigger_arg_is(&cmd->start_arg, 0);
	err |= cfc_check_trigger_arg_is(&cmd->scan_begin_arg, 0);

	if (cmd->convert_src == TRIG_TIMER)
		err |= cfc_check_trigger_arg_min(&cmd->convert_arg,
						 board->ns_min);
	else	/* TRIG_EXT */
		err |= cfc_check_trigger_arg_is(&cmd->convert_arg, 0);

	err |= cfc_check_trigger_arg_is(&cmd->scan_end_arg, cmd->chanlist_len);

	if (cmd->stop_src == TRIG_COUNT)
		err |= cfc_check_trigger_arg_min(&cmd->stop_arg, 1);
	else	/* TRIG_NONE */
		err |= cfc_check_trigger_arg_is(&cmd->stop_arg, 0);

	if (err)
		return 3;

	/* step 4: fix up any arguments */

	if (cmd->convert_src == TRIG_TIMER) {
		arg = cmd->convert_arg;
		i8253_cascade_ns_to_timer(devpriv->i8253_osc_base,
					  &devpriv->divisor1,
					  &devpriv->divisor2,
					  &arg, cmd->flags);
		err |= cfc_check_trigger_arg_is(&cmd->convert_arg, arg);
	}

	if (err)
		return 4;

	/* step 5: complain about special chanlist considerations */

	if (cmd->chanlist) {
		if (!check_channel_list(dev, s, cmd->chanlist,
					cmd->chanlist_len))
			return 5;	/*  incorrect channels list */
	}

	return 0;
}

*/
static int ai_cmd(struct comedi_device *dev, struct comedi_subdevice *s)
{
	struct comedi_cmd *cmd = &s->async->cmd;
	int retval;

	dev_dbg(dev->hw_dev, "pcl818_ai_cmd()\n");
	devpriv->ai_n_chan = cmd->chanlist_len;
	devpriv->ai_chanlist = cmd->chanlist;
	devpriv->ai_flags = cmd->flags;
	devpriv->ai_data_len = s->async->prealloc_bufsz;
	devpriv->ai_data = s->async->prealloc_buf;
	devpriv->ai_timer1 = 0;
	devpriv->ai_timer2 = 0;

	if (cmd->stop_src == TRIG_COUNT)
		devpriv->ai_scans = cmd->stop_arg;
	else
		devpriv->ai_scans = 0;

	if (cmd->scan_begin_src == TRIG_FOLLOW) {	/*  mode 1, 3 */
		if (cmd->convert_src == TRIG_TIMER) {	/*  mode 1 */
			devpriv->ai_timer1 = cmd->convert_arg;
			retval = pcl818_ai_cmd_mode(1, dev, s);
			dev_dbg(dev->hw_dev, "pcl818_ai_cmd() end\n");
			return retval;
		}
		if (cmd->convert_src == TRIG_EXT) {	/*  mode 3 */
			return pcl818_ai_cmd_mode(3, dev, s);
		}
	}

	return -1;
}

/*
==============================================================================
 cancel any mode 1-4 AI
*/
static int pcl818_ai_cancel(struct comedi_device *dev,
			    struct comedi_subdevice *s)
{
	if (devpriv->irq_blocked > 0) {
		dev_dbg(dev->hw_dev, "pcl818_ai_cancel()\n");
		devpriv->irq_was_now_closed = 1;

		switch (devpriv->ai_mode) {
#ifdef unused
		case INT_TYPE_AI1_DMA_RTC:
		case INT_TYPE_AI3_DMA_RTC:
			set_rtc_irq_bit(0);	/*  stop RTC */
			del_timer(&devpriv->rtc_irq_timer);
#endif
		case INT_TYPE_AI1_DMA:
		case INT_TYPE_AI3_DMA:
			if (devpriv->neverending_ai ||
			    (!devpriv->neverending_ai &&
			     devpriv->ai_act_scan > 0)) {
				/* wait for running dma transfer to end, do cleanup in interrupt */
				goto end;
			}
			disable_dma(devpriv->dma);
		case INT_TYPE_AI1_INT:
		case INT_TYPE_AI3_INT:
		case INT_TYPE_AI1_FIFO:
		case INT_TYPE_AI3_FIFO:
#ifdef PCL818_MODE13_AO
		case INT_TYPE_AO1_INT:
		case INT_TYPE_AO3_INT:
#endif
			outb(inb(dev->iobase + PCL818_CONTROL) & 0x73, dev->iobase + PCL818_CONTROL);	/* Stop A/D */
			udelay(1);
			start_pacer(dev, -1, 0, 0);
			outb(0, dev->iobase + PCL818_AD_LO);
			inb(dev->iobase + PCL818_AD_LO);
			inb(dev->iobase + PCL818_AD_HI);
			outb(0, dev->iobase + PCL818_CLRINT);	/* clear INT request */
			outb(0, dev->iobase + PCL818_CONTROL);	/* Stop A/D */
			if (devpriv->usefifo) {	/*  FIFO shutdown */
				outb(0, dev->iobase + PCL818_FI_INTCLR);
				outb(0, dev->iobase + PCL818_FI_FLUSH);
				outb(0, dev->iobase + PCL818_FI_ENABLE);
			}
			devpriv->irq_blocked = 0;
			devpriv->last_int_sub = s;
			devpriv->neverending_ai = 0;
			devpriv->ai_mode = 0;
			devpriv->irq_was_now_closed = 0;
			break;
		}
	}

end:
	dev_dbg(dev->hw_dev, "pcl818_ai_cancel() end\n");
	return 0;
}

/*
==============================================================================
 chech for PCL818
*/
static int pcl818_check(unsigned long iobase)
{
	outb(0x00, iobase + PCL818_MUX);
	udelay(1);
	if (inb(iobase + PCL818_MUX) != 0x00)
		return 1;	/* there isn't card */
	outb(0x55, iobase + PCL818_MUX);
	udelay(1);
	if (inb(iobase + PCL818_MUX) != 0x55)
		return 1;	/* there isn't card */
	outb(0x00, iobase + PCL818_MUX);
	udelay(1);
	outb(0x18, iobase + PCL818_CONTROL);
	udelay(1);
	if (inb(iobase + PCL818_CONTROL) != 0x18)
		return 1;	/* there isn't card */
	return 0;		/*  ok, card exist */
}

/*
==============================================================================
 reset whole PCL-818 cards
*/
static void pcl818_reset(struct comedi_device *dev)
{
=======
static int pcl818_ai_cmd(struct comedi_device *dev,
			 struct comedi_subdevice *s)
{
	struct pcl818_private *devpriv = dev->private;
	struct comedi_isadma *dma = devpriv->dma;
	struct comedi_cmd *cmd = &s->async->cmd;
	unsigned int ctrl = 0;
	unsigned int seglen;

	if (devpriv->ai_cmd_running)
		return -EBUSY;

	pcl818_start_pacer(dev, false);

	seglen = check_channel_list(dev, s, cmd->chanlist, cmd->chanlist_len);
	if (seglen < 1)
		return -EINVAL;
	pcl818_ai_setup_chanlist(dev, cmd->chanlist, seglen);

	devpriv->ai_cmd_running = 1;
	devpriv->ai_cmd_canceled = 0;
	devpriv->act_chanlist_pos = 0;

	if (cmd->convert_src == TRIG_TIMER)
		ctrl |= PCL818_CTRL_PACER_TRIG;
	else
		ctrl |= PCL818_CTRL_EXT_TRIG;

	outb(PCL818_CNTENABLE_PACER_ENA, dev->iobase + PCL818_CNTENABLE_REG);

	if (dma) {
		/* setup and enable dma for the first buffer */
		dma->cur_dma = 0;
		pcl818_ai_setup_dma(dev, s, 0);

		ctrl |= PCL818_CTRL_INTE | PCL818_CTRL_IRQ(dev->irq) |
			PCL818_CTRL_DMAE;
	} else if (devpriv->usefifo) {
		/* enable FIFO */
		outb(1, dev->iobase + PCL818_FI_ENABLE);
	} else {
		ctrl |= PCL818_CTRL_INTE | PCL818_CTRL_IRQ(dev->irq);
	}
	outb(ctrl, dev->iobase + PCL818_CTRL_REG);

	if (cmd->convert_src == TRIG_TIMER)
		pcl818_start_pacer(dev, true);

	return 0;
}

static int pcl818_ai_cancel(struct comedi_device *dev,
			    struct comedi_subdevice *s)
{
	struct pcl818_private *devpriv = dev->private;
	struct comedi_isadma *dma = devpriv->dma;
	struct comedi_cmd *cmd = &s->async->cmd;

	if (!devpriv->ai_cmd_running)
		return 0;

	if (dma) {
		if (cmd->stop_src == TRIG_NONE ||
		    (cmd->stop_src == TRIG_COUNT &&
		     s->async->scans_done < cmd->stop_arg)) {
			if (!devpriv->ai_cmd_canceled) {
				/*
				* Wait for running dma transfer to end,
				* do cleanup in interrupt.
				*/
				devpriv->ai_cmd_canceled = 1;
				return 0;
			}
		}
		comedi_isadma_disable(dma->chan);
	}

	outb(PCL818_CTRL_DISABLE_TRIG, dev->iobase + PCL818_CTRL_REG);
	pcl818_start_pacer(dev, false);
	pcl818_ai_clear_eoc(dev);

	if (devpriv->usefifo) {	/*  FIFO shutdown */
		outb(0, dev->iobase + PCL818_FI_INTCLR);
		outb(0, dev->iobase + PCL818_FI_FLUSH);
		outb(0, dev->iobase + PCL818_FI_ENABLE);
	}
  Enable(1)/disable(0) periodic interrupts from RTC
*/
static int set_rtc_irq_bit(unsigned char bit)
{
	unsigned char val;
	unsigned long flags;

	if (bit == 1) {
		RTC_timer_lock++;
		if (RTC_timer_lock > 1)
			return 0;
	} else {
		RTC_timer_lock--;
		if (RTC_timer_lock < 0)
			RTC_timer_lock = 0;
		if (RTC_timer_lock > 0)
			return 0;
	}

	save_flags(flags);
	cli();
	val = CMOS_READ(RTC_CONTROL);
	if (bit)
		val |= RTC_PIE;
	else
		val &= ~RTC_PIE;

	CMOS_WRITE(val, RTC_CONTROL);
	CMOS_READ(RTC_INTR_FLAGS);
	restore_flags(flags);
	return 0;
}

/*
==============================================================================
  Restart RTC if something stop it (xntpd every 11 mins or large IDE transfers)
*/
static void rtc_dropped_irq(unsigned long data)
{
	struct comedi_device *dev = (void *)data;
	unsigned long flags, tmp;

	switch (devpriv->int818_mode) {
	case INT_TYPE_AI1_DMA_RTC:
	case INT_TYPE_AI3_DMA_RTC:
		mod_timer(&devpriv->rtc_irq_timer,
			  jiffies + HZ / devpriv->rtc_freq + 2 * HZ / 100);
		save_flags(flags);
		cli();
		tmp = (CMOS_READ(RTC_INTR_FLAGS) & 0xF0);	/* restart */
		restore_flags(flags);
		break;
	}
}

/*
==============================================================================
  Set frequency of interrupts from RTC
*/
static int rtc_setfreq_irq(int freq)
{
	int tmp = 0;
	int rtc_freq;
	unsigned char val;
	unsigned long flags;

	if (freq < 2)
		freq = 2;
	if (freq > 8192)
		freq = 8192;

	while (freq > (1 << tmp))
		tmp++;

	rtc_freq = 1 << tmp;

	save_flags(flags);
	cli();
	val = CMOS_READ(RTC_FREQ_SELECT) & 0xf0;
	val |= (16 - tmp);
	CMOS_WRITE(val, RTC_FREQ_SELECT);
	restore_flags(flags);
	return rtc_freq;
}
#endif

/*
==============================================================================
  Free any resources that we have claimed
*/
static void free_resources(struct comedi_device *dev)
{
	/* printk("free_resource()\n"); */
	if (dev->private) {
		pcl818_ai_cancel(dev, devpriv->sub_ai);
		pcl818_reset(dev);
		if (devpriv->dma)
			free_dma(devpriv->dma);
		if (devpriv->dmabuf[0])
			free_pages(devpriv->dmabuf[0], devpriv->dmapages[0]);
		if (devpriv->dmabuf[1])
			free_pages(devpriv->dmabuf[1], devpriv->dmapages[1]);
#ifdef unused
		if (devpriv->rtc_irq)
			free_irq(devpriv->rtc_irq, dev);
		if ((devpriv->dma_rtc) && (RTC_lock == 1)) {
			if (devpriv->rtc_iobase)
				release_region(devpriv->rtc_iobase,
					       devpriv->rtc_iosize);
		}
		if (devpriv->dma_rtc)
			RTC_lock--;
#endif
	}

	if (dev->irq)
		free_irq(dev->irq, dev);
	if (dev->iobase)
		release_region(dev->iobase, devpriv->io_range);
	/* printk("free_resource() end\n"); */
}

/*
==============================================================================

   Initialization

*/
static int pcl818_attach(struct comedi_device *dev, struct comedi_devconfig *it)
{
	int ret;
	unsigned long iobase;
	unsigned int irq;
	int dma;
	unsigned long pages;
	struct comedi_subdevice *s;

	ret = alloc_private(dev, sizeof(struct pcl818_private));
	if (ret < 0)
		return ret;	/* Can't alloc mem */

	/* claim our I/O space */
	iobase = it->options[0];
	printk
	    ("comedi%d: pcl818:  board=%s, ioport=0x%03lx",
	     dev->minor, this_board->name, iobase);
	devpriv->io_range = this_board->io_range;
	if ((this_board->fifo) && (it->options[2] == -1)) {	/*  we've board with FIFO and we want to use FIFO */
		devpriv->io_range = PCLx1xFIFO_RANGE;
		devpriv->usefifo = 1;
	}
	if (!request_region(iobase, devpriv->io_range, "pcl818")) {
		comedi_error(dev, "I/O port conflict\n");
		return -EIO;
	}

	dev->iobase = iobase;

	if (pcl818_check(iobase)) {
		comedi_error(dev, "I can't detect board. FAIL!\n");
		return -EIO;
	}

	/* set up some name stuff */
	dev->board_name = this_board->name;
	/* grab our IRQ */
	irq = 0;
	if (this_board->IRQbits != 0) {	/* board support IRQ */
		irq = it->options[1];
		if (irq) {	/* we want to use IRQ */
			if (((1 << irq) & this_board->IRQbits) == 0) {
				printk
				    (", IRQ %u is out of allowed range, DISABLING IT",
				     irq);
				irq = 0;	/* Bad IRQ */
			} else {
				if (request_irq
				    (irq, interrupt_pcl818, 0, "pcl818", dev)) {
					printk
					    (", unable to allocate IRQ %u, DISABLING IT",
					     irq);
					irq = 0;	/* Can't use IRQ */
				} else {
					printk(KERN_DEBUG "irq=%u", irq);
				}
			}
		}
	}

	dev->irq = irq;
	if (irq)
		devpriv->irq_free = 1;   /* 1=we have allocated irq */
	else
		devpriv->irq_free = 0;

	devpriv->irq_blocked = 0;	/* number of subdevice which use IRQ */
	devpriv->ai_mode = 0;	/* mode of irq */

#ifdef unused
	/* grab RTC for DMA operations */
	devpriv->dma_rtc = 0;
	if (it->options[2] > 0) {	/*  we want to use DMA */
		if (RTC_lock == 0) {
			if (!request_region(RTC_PORT(0), RTC_IO_EXTENT,
					    "pcl818 (RTC)"))
				goto no_rtc;
		}
		devpriv->rtc_iobase = RTC_PORT(0);
		devpriv->rtc_iosize = RTC_IO_EXTENT;
		RTC_lock++;
		if (!request_irq(RTC_IRQ, interrupt_pcl818_ai_mode13_dma_rtc, 0,
				 "pcl818 DMA (RTC)", dev)) {
			devpriv->dma_rtc = 1;
			devpriv->rtc_irq = RTC_IRQ;
			printk(KERN_DEBUG "dma_irq=%u", devpriv->rtc_irq);
		} else {
			RTC_lock--;
			if (RTC_lock == 0) {
				if (devpriv->rtc_iobase)
					release_region(devpriv->rtc_iobase,
						       devpriv->rtc_iosize);
			}
			devpriv->rtc_iobase = 0;
			devpriv->rtc_iosize = 0;
		}
	}

no_rtc:
#endif
	/* grab our DMA */
	dma = 0;
	devpriv->dma = dma;
	if ((devpriv->irq_free == 0) && (devpriv->dma_rtc == 0))
		goto no_dma;	/* if we haven't IRQ, we can't use DMA */
	if (this_board->DMAbits != 0) {	/* board support DMA */
		dma = it->options[2];
		if (dma < 1)
			goto no_dma;	/* DMA disabled */
		if (((1 << dma) & this_board->DMAbits) == 0) {
			printk(KERN_ERR "DMA is out of allowed range, FAIL!\n");
			return -EINVAL;	/* Bad DMA */
		}
		ret = request_dma(dma, "pcl818");
		if (ret)
			return -EBUSY;	/* DMA isn't free */
		devpriv->dma = dma;
		pages = 2;	/* we need 16KB */
		devpriv->dmabuf[0] = __get_dma_pages(GFP_KERNEL, pages);
		if (!devpriv->dmabuf[0])
			/* maybe experiment with try_to_free_pages() will help .... */
			return -EBUSY;	/* no buffer :-( */
		devpriv->dmapages[0] = pages;
		devpriv->hwdmaptr[0] = virt_to_bus((void *)devpriv->dmabuf[0]);
		devpriv->hwdmasize[0] = (1 << pages) * PAGE_SIZE;
		/* printk("%d %d %ld, ",devpriv->dmapages[0],devpriv->hwdmasize[0],PAGE_SIZE); */
		if (devpriv->dma_rtc == 0) {	/*  we must do duble buff :-( */
			devpriv->dmabuf[1] = __get_dma_pages(GFP_KERNEL, pages);
			if (!devpriv->dmabuf[1])
				return -EBUSY;
			devpriv->dmapages[1] = pages;
			devpriv->hwdmaptr[1] =
			    virt_to_bus((void *)devpriv->dmabuf[1]);
			devpriv->hwdmasize[1] = (1 << pages) * PAGE_SIZE;
		}
	}

no_dma:

	ret = alloc_subdevices(dev, 4);
	if (ret < 0)
		return ret;

	s = dev->subdevices + 0;
	if (!this_board->n_aichan_se) {
		s->type = COMEDI_SUBD_UNUSED;
	} else {
		s->type = COMEDI_SUBD_AI;
		devpriv->sub_ai = s;
		s->subdev_flags = SDF_READABLE;
		if (check_single_ended(dev->iobase)) {
			s->n_chan = this_board->n_aichan_se;
			s->subdev_flags |= SDF_COMMON | SDF_GROUND;
			printk(", %dchans S.E. DAC", s->n_chan);
		} else {
			s->n_chan = this_board->n_aichan_diff;
			s->subdev_flags |= SDF_DIFF;
			printk(", %dchans DIFF DAC", s->n_chan);
		}
		s->maxdata = this_board->ai_maxdata;
		s->len_chanlist = s->n_chan;
		s->range_table = this_board->ai_range_type;
		s->cancel = pcl818_ai_cancel;
		s->insn_read = pcl818_ai_insn_read;
		if ((irq) || (devpriv->dma_rtc)) {
			dev->read_subdev = s;
			s->subdev_flags |= SDF_CMD_READ;
			s->do_cmdtest = ai_cmdtest;
			s->do_cmd = ai_cmd;
		}
		if (this_board->is_818) {
			if ((it->options[4] == 1) || (it->options[4] == 10))
				s->range_table = &range_pcl818l_h_ai;	/*  secondary range list jumper selectable */
		} else {
			switch (it->options[4]) {
			case 0:
				s->range_table = &range_bipolar10;
				break;
			case 1:
				s->range_table = &range_bipolar5;
				break;
			case 2:
				s->range_table = &range_bipolar2_5;
				break;
			case 3:
				s->range_table = &range718_bipolar1;
				break;
			case 4:
				s->range_table = &range718_bipolar0_5;
				break;
			case 6:
				s->range_table = &range_unipolar10;
				break;
			case 7:
				s->range_table = &range_unipolar5;
				break;
			case 8:
				s->range_table = &range718_unipolar2;
				break;
			case 9:
				s->range_table = &range718_unipolar1;
				break;
			default:
				s->range_table = &range_unknown;
				break;
			}
		}
	}

	s = dev->subdevices + 1;
	if (!this_board->n_aochan) {
		s->type = COMEDI_SUBD_UNUSED;
	} else {
		s->type = COMEDI_SUBD_AO;
		s->subdev_flags = SDF_WRITABLE | SDF_GROUND;
		s->n_chan = this_board->n_aochan;
		s->maxdata = this_board->ao_maxdata;
		s->len_chanlist = this_board->n_aochan;
		s->range_table = this_board->ao_range_type;
		s->insn_read = pcl818_ao_insn_read;
		s->insn_write = pcl818_ao_insn_write;
#ifdef unused
#ifdef PCL818_MODE13_AO
		if (irq) {
			s->trig[1] = pcl818_ao_mode1;
			s->trig[3] = pcl818_ao_mode3;
		}
#endif
#endif
		if (this_board->is_818) {
=======
	devpriv->ai_cmd_running = 0;
	devpriv->ai_cmd_canceled = 0;

	return 0;
}

static int pcl818_ai_insn_read(struct comedi_device *dev,
			       struct comedi_subdevice *s,
			       struct comedi_insn *insn,
			       unsigned int *data)
{
	unsigned int chan = CR_CHAN(insn->chanspec);
	unsigned int range = CR_RANGE(insn->chanspec);
	int ret = 0;
	int i;

	outb(PCL818_CTRL_SOFT_TRIG, dev->iobase + PCL818_CTRL_REG);

	pcl818_ai_set_chan_range(dev, chan, range);
	pcl818_ai_set_chan_scan(dev, chan, chan);

	for (i = 0; i < insn->n; i++) {
		pcl818_ai_clear_eoc(dev);
		pcl818_ai_soft_trig(dev);

		ret = comedi_timeout(dev, s, insn, pcl818_ai_eoc, 0);
		if (ret)
			break;

		data[i] = pcl818_ai_get_sample(dev, s, NULL);
	}
	pcl818_ai_clear_eoc(dev);

	return ret ? ret : insn->n;
}

static int pcl818_ao_insn_write(struct comedi_device *dev,
				struct comedi_subdevice *s,
				struct comedi_insn *insn,
				unsigned int *data)
{
	unsigned int chan = CR_CHAN(insn->chanspec);
	unsigned int val = s->readback[chan];
	int i;

	for (i = 0; i < insn->n; i++) {
		val = data[i];
		outb((val & 0x000f) << 4,
		     dev->iobase + PCL818_AO_LSB_REG(chan));
		outb((val & 0x0ff0) >> 4,
		     dev->iobase + PCL818_AO_MSB_REG(chan));
	}
	s->readback[chan] = val;

	return insn->n;
}

static int pcl818_di_insn_bits(struct comedi_device *dev,
			       struct comedi_subdevice *s,
			       struct comedi_insn *insn,
			       unsigned int *data)
{
	data[1] = inb(dev->iobase + PCL818_DO_DI_LSB_REG) |
		  (inb(dev->iobase + PCL818_DO_DI_MSB_REG) << 8);

	return insn->n;
}

static int pcl818_do_insn_bits(struct comedi_device *dev,
			       struct comedi_subdevice *s,
			       struct comedi_insn *insn,
			       unsigned int *data)
{
	if (comedi_dio_update_state(s, data)) {
		outb(s->state & 0xff, dev->iobase + PCL818_DO_DI_LSB_REG);
		outb((s->state >> 8), dev->iobase + PCL818_DO_DI_MSB_REG);
	}

	data[1] = s->state;

	return insn->n;
}

static void pcl818_reset(struct comedi_device *dev)
{
	const struct pcl818_board *board = dev->board_ptr;
	unsigned long timer_base = dev->iobase + PCL818_TIMER_BASE;
	unsigned int chan;

	/* flush and disable the FIFO */
	if (board->has_fifo) {
		outb(0, dev->iobase + PCL818_FI_INTCLR);
		outb(0, dev->iobase + PCL818_FI_FLUSH);
		outb(0, dev->iobase + PCL818_FI_ENABLE);
	}

	/* disable analog input trigger */
	outb(PCL818_CTRL_DISABLE_TRIG, dev->iobase + PCL818_CTRL_REG);
	pcl818_ai_clear_eoc(dev);

	pcl818_ai_set_chan_range(dev, 0, 0);

	/* stop pacer */
	outb(PCL818_CNTENABLE_PACER_ENA, dev->iobase + PCL818_CNTENABLE_REG);
	i8254_set_mode(timer_base, 0, 2, I8254_MODE0 | I8254_BINARY);
	i8254_set_mode(timer_base, 0, 1, I8254_MODE0 | I8254_BINARY);
	i8254_set_mode(timer_base, 0, 0, I8254_MODE0 | I8254_BINARY);

	/* set analog output channels to 0V */
	for (chan = 0; chan < board->n_aochan; chan++) {
		outb(0, dev->iobase + PCL818_AO_LSB_REG(chan));
		outb(0, dev->iobase + PCL818_AO_MSB_REG(chan));
	}

	/* set all digital outputs low */
	outb(0, dev->iobase + PCL818_DO_DI_MSB_REG);
	outb(0, dev->iobase + PCL818_DO_DI_LSB_REG);
}

static void pcl818_set_ai_range_table(struct comedi_device *dev,
				      struct comedi_subdevice *s,
				      struct comedi_devconfig *it)
{
	const struct pcl818_board *board = dev->board_ptr;

	/* default to the range table from the boardinfo */
	s->range_table = board->ai_range_type;

	/* now check the user config option based on the boardtype */
	if (board->is_818) {
		if (it->options[4] == 1 || it->options[4] == 10) {
			/* secondary range list jumper selectable */
			s->range_table = &range_pcl818l_h_ai;
		}
	} else {
		switch (it->options[4]) {
		case 0:
			s->range_table = &range_bipolar10;
			break;
		case 1:
			s->range_table = &range_bipolar5;
			break;
		case 2:
			s->range_table = &range_bipolar2_5;
			break;
		case 3:
			s->range_table = &range718_bipolar1;
			break;
		case 4:
			s->range_table = &range718_bipolar0_5;
			break;
		case 6:
			s->range_table = &range_unipolar10;
			break;
		case 7:
			s->range_table = &range_unipolar5;
			break;
		case 8:
			s->range_table = &range718_unipolar2;
			break;
		case 9:
			s->range_table = &range718_unipolar1;
			break;
		default:
			s->range_table = &range_unknown;
			break;
		}
	}
}

static void pcl818_alloc_dma(struct comedi_device *dev, unsigned int dma_chan)
{
	struct pcl818_private *devpriv = dev->private;

	/* only DMA channels 3 and 1 are valid */
	if (!(dma_chan == 3 || dma_chan == 1))
		return;

	/* DMA uses two 16K buffers */
	devpriv->dma = comedi_isadma_alloc(dev, 2, dma_chan, dma_chan,
					   PAGE_SIZE * 4, COMEDI_ISADMA_READ);
}

static void pcl818_free_dma(struct comedi_device *dev)
{
	struct pcl818_private *devpriv = dev->private;

	if (devpriv)
		comedi_isadma_free(devpriv->dma);
}

static int pcl818_attach(struct comedi_device *dev, struct comedi_devconfig *it)
{
	const struct pcl818_board *board = dev->board_ptr;
	struct pcl818_private *devpriv;
	struct comedi_subdevice *s;
	int ret;

	devpriv = comedi_alloc_devpriv(dev, sizeof(*devpriv));
	if (!devpriv)
		return -ENOMEM;

	ret = comedi_request_region(dev, it->options[0],
				    board->has_fifo ? 0x20 : 0x10);
	if (ret)
		return ret;

	/* we can use IRQ 2-7 for async command support */
	if (it->options[1] >= 2 && it->options[1] <= 7) {
		ret = request_irq(it->options[1], pcl818_interrupt, 0,
				  dev->board_name, dev);
		if (ret == 0)
			dev->irq = it->options[1];
	}

	/* should we use the FIFO? */
	if (dev->irq && board->has_fifo && it->options[2] == -1)
		devpriv->usefifo = 1;

	/* we need an IRQ to do DMA on channel 3 or 1 */
	if (dev->irq && board->has_dma)
		pcl818_alloc_dma(dev, it->options[2]);

	ret = comedi_alloc_subdevices(dev, 4);
	if (ret)
		return ret;

	s = &dev->subdevices[0];
	s->type		= COMEDI_SUBD_AI;
	s->subdev_flags	= SDF_READABLE;
	if (check_single_ended(dev->iobase)) {
		s->n_chan	= 16;
		s->subdev_flags	|= SDF_COMMON | SDF_GROUND;
	} else {
		s->n_chan	= 8;
		s->subdev_flags	|= SDF_DIFF;
	}
	s->maxdata	= 0x0fff;

	pcl818_set_ai_range_table(dev, s, it);

	s->insn_read	= pcl818_ai_insn_read;
	if (dev->irq) {
		dev->read_subdev = s;
		s->subdev_flags	|= SDF_CMD_READ;
		s->len_chanlist	= s->n_chan;
		s->do_cmdtest	= ai_cmdtest;
		s->do_cmd	= pcl818_ai_cmd;
		s->cancel	= pcl818_ai_cancel;
	}

	/* Analog Output subdevice */
	s = &dev->subdevices[1];
	if (board->n_aochan) {
		s->type		= COMEDI_SUBD_AO;
		s->subdev_flags	= SDF_WRITABLE | SDF_GROUND;
		s->n_chan	= board->n_aochan;
		s->maxdata	= 0x0fff;
		s->range_table	= &range_unipolar5;
		if (board->is_818) {
			if ((it->options[4] == 1) || (it->options[4] == 10))
				s->range_table = &range_unipolar10;
			if (it->options[4] == 2)
				s->range_table = &range_unknown;
		} else {
			if ((it->options[5] == 1) || (it->options[5] == 10))
				s->range_table = &range_unipolar10;
			if (it->options[5] == 2)
				s->range_table = &range_unknown;
		}
		s->insn_write	= pcl818_ao_insn_write;

		ret = comedi_alloc_subdev_readback(s);
		if (ret)
			return ret;
	} else {
		s->type		= COMEDI_SUBD_UNUSED;
	}

	/* Digital Input subdevice */
	s = &dev->subdevices[2];
	s->type		= COMEDI_SUBD_DI;
	s->subdev_flags	= SDF_READABLE;
	s->n_chan	= 16;
	s->maxdata	= 1;
	s->range_table	= &range_digital;
	s->insn_bits	= pcl818_di_insn_bits;

	/* Digital Output subdevice */
	s = &dev->subdevices[3];
	s->type		= COMEDI_SUBD_DO;
	s->subdev_flags	= SDF_WRITABLE;
	s->n_chan	= 16;
	s->maxdata	= 1;
	s->range_table	= &range_digital;
	s->insn_bits	= pcl818_do_insn_bits;

	/* select 1/10MHz oscilator */
	if ((it->options[3] == 0) || (it->options[3] == 10))
		devpriv->i8253_osc_base = I8254_OSC_BASE_10MHZ;
	else
		devpriv->i8253_osc_base = I8254_OSC_BASE_1MHZ;

	/* max sampling speed */
	devpriv->ns_min = board->ns_min;

	if (!board->is_818) {
		if ((it->options[6] == 1) || (it->options[6] == 100)) {
			/* extended PCL718 to 100kHz DAC */
			devpriv->ns_min = 10000;
		}
	}

	pcl818_reset(dev);

  Removes device
 */
static int pcl818_detach(struct comedi_device *dev)
{
	/*   printk("comedi%d: pcl818: remove\n", dev->minor); */
	free_resources(dev);
	return 0;
}

=======
	return 0;
}

static void pcl818_detach(struct comedi_device *dev)
{
	struct pcl818_private *devpriv = dev->private;

	if (devpriv) {
		pcl818_ai_cancel(dev, dev->read_subdev);
		pcl818_reset(dev);
	}
	pcl818_free_dma(dev);
	comedi_legacy_detach(dev);
}

static struct comedi_driver pcl818_driver = {
	.driver_name	= "pcl818",
	.module		= THIS_MODULE,
	.attach		= pcl818_attach,
	.detach		= pcl818_detach,
	.board_name	= &boardtypes[0].name,
	.num_names	= ARRAY_SIZE(boardtypes),
	.offset		= sizeof(struct pcl818_board),
};
module_comedi_driver(pcl818_driver);

MODULE_AUTHOR("Comedi http://www.comedi.org");
MODULE_DESCRIPTION("Comedi low-level driver");
MODULE_LICENSE("GPL");
