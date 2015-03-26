/*
   comedi/drivers/pcl816.c

   Author:  Juan Grigera <juan@grigera.com.ar>
	    based on pcl818 by Michal Dobes <dobes@tesnet.cz> and bits of pcl812

   hardware driver for Advantech cards:
    card:   PCL-816, PCL814B
    driver: pcl816
*/
/*
Driver: pcl816
Description: Advantech PCL-816 cards, PCL-814
Author: Juan Grigera <juan@grigera.com.ar>
Devices: [Advantech] PCL-816 (pcl816), PCL-814B (pcl814b)
Status: works
Updated: Tue,  2 Apr 2002 23:15:21 -0800

PCL 816 and 814B have 16 SE/DIFF ADCs, 16 DACs, 16 DI and 16 DO.
Differences are at resolution (16 vs 12 bits).

The driver support AI command mode, other subdevices not written.

Analog output and digital input and output are not supported.

Configuration Options:
  [0] - IO Base
  [1] - IRQ	(0=disable, 2, 3, 4, 5, 6, 7)
  [2] - DMA	(0=disable, 1, 3)
  [3] - 0, 10=10MHz clock for 8254
	    1= 1MHz clock for 8254

*/

*/
static int check_channel_list(struct comedi_device *dev,
			      struct comedi_subdevice *s,
			      unsigned int *chanlist, unsigned int chanlen);
static void setup_channel_list(struct comedi_device *dev,
			       struct comedi_subdevice *s,
			       unsigned int *chanlist, unsigned int seglen);
static int pcl816_ai_cancel(struct comedi_device *dev,
			    struct comedi_subdevice *s);
static void start_pacer(struct comedi_device *dev, int mode,
			unsigned int divisor1, unsigned int divisor2);
#ifdef unused
static int set_rtc_irq_bit(unsigned char bit);
#endif

static int pcl816_ai_cmdtest(struct comedi_device *dev,
			     struct comedi_subdevice *s,
			     struct comedi_cmd *cmd);
static int pcl816_ai_cmd(struct comedi_device *dev, struct comedi_subdevice *s);

/*
==============================================================================
   ANALOG INPUT MODE0, 816 cards, slow version
*/
static int pcl816_ai_insn_read(struct comedi_device *dev,
			       struct comedi_subdevice *s,
			       struct comedi_insn *insn, unsigned int *data)
{
	int n;
	int timeout;

	DPRINTK("mode 0 analog input\n");
	/*  software trigger, DMA and INT off */
	outb(0, dev->iobase + PCL816_CONTROL);
	/*  clear INT (conversion end) flag */
	outb(0, dev->iobase + PCL816_CLRINT);

	/*  Set the input channel */
	outb(CR_CHAN(insn->chanspec) & 0xf, dev->iobase + PCL816_MUX);
	/* select gain */
	outb(CR_RANGE(insn->chanspec), dev->iobase + PCL816_RANGE);

	for (n = 0; n < insn->n; n++) {

		outb(0, dev->iobase + PCL816_AD_LO);	/* start conversion */

		timeout = 100;
		while (timeout--) {
			if (!(inb(dev->iobase + PCL816_STATUS) &
			      PCL816_STATUS_DRDY_MASK)) {
				/*  return read value */
				data[n] =
				    ((inb(dev->iobase +
					  PCL816_AD_HI) << 8) |
				     (inb(dev->iobase + PCL816_AD_LO)));
				/* clear INT (conversion end) flag */
				outb(0, dev->iobase + PCL816_CLRINT);
				break;
			}
			udelay(1);
		}
		/*  Return timeout error */
		if (!timeout) {
			comedi_error(dev, "A/D insn timeout\n");
			data[0] = 0;
			/* clear INT (conversion end) flag */
			outb(0, dev->iobase + PCL816_CLRINT);
			return -EIO;
		}

	}
	return n;
}

/*
==============================================================================
   analog input interrupt mode 1 & 3, 818 cards
   one sample per interrupt version
*/
static irqreturn_t interrupt_pcl816_ai_mode13_int(int irq, void *d)
{
	struct comedi_device *dev = d;
	struct comedi_subdevice *s = dev->subdevices + 0;
	int low, hi;
	int timeout = 50;	/* wait max 50us */

	while (timeout--) {
		if (!(inb(dev->iobase + PCL816_STATUS) &
		      PCL816_STATUS_DRDY_MASK))
			break;
		udelay(1);
	}
	if (!timeout) {		/*  timeout, bail error */
		outb(0, dev->iobase + PCL816_CLRINT);	/* clear INT request */
		comedi_error(dev, "A/D mode1/3 IRQ without DRDY!");
		pcl816_ai_cancel(dev, s);
		s->async->events |= COMEDI_CB_EOA | COMEDI_CB_ERROR;
		comedi_event(dev, s);
		return IRQ_HANDLED;

	}

	/*  get the sample */
	low = inb(dev->iobase + PCL816_AD_LO);
	hi = inb(dev->iobase + PCL816_AD_HI);

	comedi_buf_put(s->async, (hi << 8) | low);

	outb(0, dev->iobase + PCL816_CLRINT);	/* clear INT request */

	if (++devpriv->ai_act_chanlist_pos >= devpriv->ai_act_chanlist_len)
		devpriv->ai_act_chanlist_pos = 0;

	s->async->cur_chan++;
	if (s->async->cur_chan >= devpriv->ai_n_chan) {
		s->async->cur_chan = 0;
		devpriv->ai_act_scan++;
	}

	if (!devpriv->ai_neverending)
					/* all data sampled */
		if (devpriv->ai_act_scan >= devpriv->ai_scans) {
			/* all data sampled */
			pcl816_ai_cancel(dev, s);
			s->async->events |= COMEDI_CB_EOA;
		}
	comedi_event(dev, s);
	return IRQ_HANDLED;
}

/*
==============================================================================
   analog input dma mode 1 & 3, 816 cards
*/
static void transfer_from_dma_buf(struct comedi_device *dev,
				  struct comedi_subdevice *s, short *ptr,
				  unsigned int bufptr, unsigned int len)
{
	int i;

	s->async->events = 0;

	for (i = 0; i < len; i++) {

		comedi_buf_put(s->async, ptr[bufptr++]);

		if (++devpriv->ai_act_chanlist_pos >=
		    devpriv->ai_act_chanlist_len) {
			devpriv->ai_act_chanlist_pos = 0;
		}

		s->async->cur_chan++;
		if (s->async->cur_chan >= devpriv->ai_n_chan) {
			s->async->cur_chan = 0;
			devpriv->ai_act_scan++;
		}

		if (!devpriv->ai_neverending)
						/*  all data sampled */
			if (devpriv->ai_act_scan >= devpriv->ai_scans) {
				pcl816_ai_cancel(dev, s);
				s->async->events |= COMEDI_CB_EOA;
				s->async->events |= COMEDI_CB_BLOCK;
				break;
			}
	}

	comedi_event(dev, s);
}

static irqreturn_t interrupt_pcl816_ai_mode13_dma(int irq, void *d)
{
	struct comedi_device *dev = d;
	struct comedi_subdevice *s = dev->subdevices + 0;
	int len, bufptr, this_dma_buf;
	unsigned long dma_flags;
	short *ptr;

	disable_dma(devpriv->dma);
	this_dma_buf = devpriv->next_dma_buf;

	/*  switch dma bufs */
	if ((devpriv->dma_runs_to_end > -1) || devpriv->ai_neverending) {

		devpriv->next_dma_buf = 1 - devpriv->next_dma_buf;
		set_dma_mode(devpriv->dma, DMA_MODE_READ);
		dma_flags = claim_dma_lock();
/* clear_dma_ff (devpriv->dma); */
		set_dma_addr(devpriv->dma,
			     devpriv->hwdmaptr[devpriv->next_dma_buf]);
		if (devpriv->dma_runs_to_end) {
			set_dma_count(devpriv->dma,
				      devpriv->hwdmasize[devpriv->
							 next_dma_buf]);
		} else {
			set_dma_count(devpriv->dma, devpriv->last_dma_run);
		}
		release_dma_lock(dma_flags);
		enable_dma(devpriv->dma);
	}

	devpriv->dma_runs_to_end--;
	outb(0, dev->iobase + PCL816_CLRINT);	/* clear INT request */

	ptr = (short *)devpriv->dmabuf[this_dma_buf];

	len = (devpriv->hwdmasize[0] >> 1) - devpriv->ai_poll_ptr;
	bufptr = devpriv->ai_poll_ptr;
	devpriv->ai_poll_ptr = 0;

	transfer_from_dma_buf(dev, s, ptr, bufptr, len);
	return IRQ_HANDLED;
}

/*
==============================================================================
    INT procedure
*/
static irqreturn_t interrupt_pcl816(int irq, void *d)
{
	struct comedi_device *dev = d;
	DPRINTK("<I>");

	if (!dev->attached) {
		comedi_error(dev, "premature interrupt");
		return IRQ_HANDLED;
	}

	switch (devpriv->int816_mode) {
	case INT_TYPE_AI1_DMA:
	case INT_TYPE_AI3_DMA:
		return interrupt_pcl816_ai_mode13_dma(irq, d);
	case INT_TYPE_AI1_INT:
	case INT_TYPE_AI3_INT:
		return interrupt_pcl816_ai_mode13_int(irq, d);
	}

	outb(0, dev->iobase + PCL816_CLRINT);	/* clear INT request */
	if ((!dev->irq) | (!devpriv->irq_free) | (!devpriv->irq_blocked) |
	    (!devpriv->int816_mode)) {
		if (devpriv->irq_was_now_closed) {
			devpriv->irq_was_now_closed = 0;
			/*  comedi_error(dev,"last IRQ.."); */
			return IRQ_HANDLED;
		}
		comedi_error(dev, "bad IRQ!");
		return IRQ_NONE;
	}
	comedi_error(dev, "IRQ from unknown source!");
	return IRQ_NONE;
}

/*
==============================================================================
   COMMAND MODE
*/
static void pcl816_cmdtest_out(int e, struct comedi_cmd *cmd)
{
	printk(KERN_INFO "pcl816 e=%d startsrc=%x scansrc=%x convsrc=%x\n", e,
	       cmd->start_src, cmd->scan_begin_src, cmd->convert_src);
	printk(KERN_INFO "pcl816 e=%d startarg=%d scanarg=%d convarg=%d\n", e,
	       cmd->start_arg, cmd->scan_begin_arg, cmd->convert_arg);
	printk(KERN_INFO "pcl816 e=%d stopsrc=%x scanend=%x\n", e,
	       cmd->stop_src, cmd->scan_end_src);
	printk(KERN_INFO "pcl816 e=%d stoparg=%d scanendarg=%d chanlistlen=%d\n",
	       e, cmd->stop_arg, cmd->scan_end_arg, cmd->chanlist_len);
}

/*
==============================================================================
*/
static int pcl816_ai_cmdtest(struct comedi_device *dev,
			     struct comedi_subdevice *s, struct comedi_cmd *cmd)
{
	int err = 0;
	int tmp, divisor1 = 0, divisor2 = 0;

	DEBUG(printk(KERN_INFO "pcl816 pcl812_ai_cmdtest\n");
	      pcl816_cmdtest_out(-1, cmd);
	     );

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
	cmd->convert_src &= TRIG_EXT | TRIG_TIMER;
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

	if (err)
		return 1;


	/*
	 * step 2: make sure trigger sources
	 * are unique and mutually compatible
	 */

	if (cmd->start_src != TRIG_NOW) {
		cmd->start_src = TRIG_NOW;
		err++;
	}

	if (cmd->scan_begin_src != TRIG_FOLLOW) {
		cmd->scan_begin_src = TRIG_FOLLOW;
		err++;
	}

	if (cmd->convert_src != TRIG_EXT && cmd->convert_src != TRIG_TIMER) {
		cmd->convert_src = TRIG_TIMER;
		err++;
	}

	if (cmd->scan_end_src != TRIG_COUNT) {
		cmd->scan_end_src = TRIG_COUNT;
		err++;
	}

	if (cmd->stop_src != TRIG_NONE && cmd->stop_src != TRIG_COUNT)
		err++;
=======
#include <linux/module.h>
#include <linux/gfp.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/interrupt.h>

#include "../comedidev.h"

#include "comedi_isadma.h"
#include "comedi_fc.h"
#include "8253.h"

/*
 * Register I/O map
 */
#define PCL816_DO_DI_LSB_REG			0x00
#define PCL816_DO_DI_MSB_REG			0x01
#define PCL816_TIMER_BASE			0x04
#define PCL816_AI_LSB_REG			0x08
#define PCL816_AI_MSB_REG			0x09
#define PCL816_RANGE_REG			0x09
#define PCL816_CLRINT_REG			0x0a
#define PCL816_MUX_REG				0x0b
#define PCL816_MUX_SCAN(_first, _last)		(((_last) << 4) | (_first))
#define PCL816_CTRL_REG				0x0c
#define PCL816_CTRL_DISABLE_TRIG		(0 << 0)
#define PCL816_CTRL_SOFT_TRIG			(1 << 0)
#define PCL816_CTRL_PACER_TRIG			(1 << 1)
#define PCL816_CTRL_EXT_TRIG			(1 << 2)
#define PCL816_CTRL_POE				(1 << 3)
#define PCL816_CTRL_DMAEN			(1 << 4)
#define PCL816_CTRL_INTEN			(1 << 5)
#define PCL816_CTRL_DMASRC_SLOT0		(0 << 6)
#define PCL816_CTRL_DMASRC_SLOT1		(1 << 6)
#define PCL816_CTRL_DMASRC_SLOT2		(2 << 6)
#define PCL816_STATUS_REG			0x0d
#define PCL816_STATUS_NEXT_CHAN_MASK		(0xf << 0)
#define PCL816_STATUS_INTSRC_MASK		(3 << 4)
#define PCL816_STATUS_INTSRC_SLOT0		(0 << 4)
#define PCL816_STATUS_INTSRC_SLOT1		(1 << 4)
#define PCL816_STATUS_INTSRC_SLOT2		(2 << 4)
#define PCL816_STATUS_INTSRC_DMA		(3 << 4)
#define PCL816_STATUS_INTACT			(1 << 6)
#define PCL816_STATUS_DRDY			(1 << 7)

#define MAGIC_DMA_WORD 0x5a5a

static const struct comedi_lrange range_pcl816 = {
	8, {
		BIP_RANGE(10),
		BIP_RANGE(5),
		BIP_RANGE(2.5),
		BIP_RANGE(1.25),
		UNI_RANGE(10),
		UNI_RANGE(5),
		UNI_RANGE(2.5),
		UNI_RANGE(1.25)
	}
};

struct pcl816_board {
	const char *name;
	int ai_maxdata;
	int ao_maxdata;
	int ai_chanlist;
};

static const struct pcl816_board boardtypes[] = {
	{
		.name		= "pcl816",
		.ai_maxdata	= 0xffff,
		.ao_maxdata	= 0xffff,
		.ai_chanlist	= 1024,
	}, {
		.name		= "pcl814b",
		.ai_maxdata	= 0x3fff,
		.ao_maxdata	= 0x3fff,
		.ai_chanlist	= 1024,
	},
};

struct pcl816_private {
	struct comedi_isadma *dma;
	unsigned int ai_poll_ptr;	/*  how many sampes transfer poll */
	unsigned int divisor1;
	unsigned int divisor2;
	unsigned int ai_cmd_running:1;
	unsigned int ai_cmd_canceled:1;
};

static void pcl816_start_pacer(struct comedi_device *dev, bool load_counters)
{
	struct pcl816_private *devpriv = dev->private;
	unsigned long timer_base = dev->iobase + PCL816_TIMER_BASE;

	i8254_set_mode(timer_base, 0, 0, I8254_MODE1 | I8254_BINARY);
	i8254_write(timer_base, 0, 0, 0x00ff);
	udelay(1);

	i8254_set_mode(timer_base, 0, 2, I8254_MODE2 | I8254_BINARY);
	i8254_set_mode(timer_base, 0, 1, I8254_MODE2 | I8254_BINARY);
	udelay(1);

	if (load_counters) {
		i8254_write(timer_base, 0, 2, devpriv->divisor2);
		i8254_write(timer_base, 0, 1, devpriv->divisor1);
	}
}

static void pcl816_ai_setup_dma(struct comedi_device *dev,
				struct comedi_subdevice *s,
				unsigned int unread_samples)
{
	struct pcl816_private *devpriv = dev->private;
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

static void pcl816_ai_set_chan_range(struct comedi_device *dev,
				     unsigned int chan,
				     unsigned int range)
{
	outb(chan, dev->iobase + PCL816_MUX_REG);
	outb(range, dev->iobase + PCL816_RANGE_REG);
}

static void pcl816_ai_set_chan_scan(struct comedi_device *dev,
				    unsigned int first_chan,
				    unsigned int last_chan)
{
	outb(PCL816_MUX_SCAN(first_chan, last_chan),
	     dev->iobase + PCL816_MUX_REG);
}

static void pcl816_ai_setup_chanlist(struct comedi_device *dev,
				     unsigned int *chanlist,
				     unsigned int seglen)
{
	unsigned int first_chan = CR_CHAN(chanlist[0]);
	unsigned int last_chan;
	unsigned int range;
	unsigned int i;

	/* store range list to card */
	for (i = 0; i < seglen; i++) {
		last_chan = CR_CHAN(chanlist[i]);
		range = CR_RANGE(chanlist[i]);

		pcl816_ai_set_chan_range(dev, last_chan, range);
	}

	udelay(1);

	pcl816_ai_set_chan_scan(dev, first_chan, last_chan);
}

static void pcl816_ai_clear_eoc(struct comedi_device *dev)
{
	/* writing any value clears the interrupt request */
	outb(0, dev->iobase + PCL816_CLRINT_REG);
}

static void pcl816_ai_soft_trig(struct comedi_device *dev)
{
	/* writing any value triggers a software conversion */
	outb(0, dev->iobase + PCL816_AI_LSB_REG);
}

static unsigned int pcl816_ai_get_sample(struct comedi_device *dev,
					 struct comedi_subdevice *s)
{
	unsigned int val;

	val = inb(dev->iobase + PCL816_AI_MSB_REG) << 8;
	val |= inb(dev->iobase + PCL816_AI_LSB_REG);

	return val & s->maxdata;
}

static int pcl816_ai_eoc(struct comedi_device *dev,
			 struct comedi_subdevice *s,
			 struct comedi_insn *insn,
			 unsigned long context)
{
	unsigned int status;

	status = inb(dev->iobase + PCL816_STATUS_REG);
	if ((status & PCL816_STATUS_DRDY) == 0)
		return 0;
	return -EBUSY;
}

static bool pcl816_ai_next_chan(struct comedi_device *dev,
				struct comedi_subdevice *s)
{
	struct comedi_cmd *cmd = &s->async->cmd;

	if (cmd->stop_src == TRIG_COUNT &&
	    s->async->scans_done >= cmd->stop_arg) {
		s->async->events |= COMEDI_CB_EOA;
		return false;
	}

	return true;
}

static void transfer_from_dma_buf(struct comedi_device *dev,
				  struct comedi_subdevice *s,
				  unsigned short *ptr,
				  unsigned int bufptr, unsigned int len)
{
	unsigned short val;
	int i;

	for (i = 0; i < len; i++) {
		val = ptr[bufptr++];
		comedi_buf_write_samples(s, &val, 1);

		if (!pcl816_ai_next_chan(dev, s))
			return;
	}
}

static irqreturn_t pcl816_interrupt(int irq, void *d)
{
	struct comedi_device *dev = d;
	struct comedi_subdevice *s = dev->read_subdev;
	struct pcl816_private *devpriv = dev->private;
	struct comedi_isadma *dma = devpriv->dma;
	struct comedi_isadma_desc *desc = &dma->desc[dma->cur_dma];
	unsigned int nsamples;
	unsigned int bufptr;

	if (!dev->attached || !devpriv->ai_cmd_running) {
		pcl816_ai_clear_eoc(dev);
		return IRQ_HANDLED;
	}

	if (devpriv->ai_cmd_canceled) {
		devpriv->ai_cmd_canceled = 0;
		pcl816_ai_clear_eoc(dev);
		return IRQ_HANDLED;
	}

	nsamples = comedi_bytes_to_samples(s, desc->size) -
		   devpriv->ai_poll_ptr;
	bufptr = devpriv->ai_poll_ptr;
	devpriv->ai_poll_ptr = 0;

	/* restart dma with the next buffer */
	dma->cur_dma = 1 - dma->cur_dma;
	pcl816_ai_setup_dma(dev, s, nsamples);

	transfer_from_dma_buf(dev, s, desc->virt_addr, bufptr, nsamples);

	pcl816_ai_clear_eoc(dev);

	comedi_handle_events(dev, s);
	return IRQ_HANDLED;
}

static int check_channel_list(struct comedi_device *dev,
			      struct comedi_subdevice *s,
			      unsigned int *chanlist,
			      unsigned int chanlen)
{
	unsigned int chansegment[16];
	unsigned int i, nowmustbechan, seglen, segpos;

	/*  correct channel and range number check itself comedi/range.c */
	if (chanlen < 1) {
		dev_err(dev->class_dev, "range/channel list is empty!\n");
		return 0;
	}

	if (chanlen > 1) {
		/*  first channel is every time ok */
		chansegment[0] = chanlist[0];
		for (i = 1, seglen = 1; i < chanlen; i++, seglen++) {
			/*  we detect loop, this must by finish */
			    if (chanlist[0] == chanlist[i])
				break;
			nowmustbechan =
			    (CR_CHAN(chansegment[i - 1]) + 1) % chanlen;
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
		for (i = 0, segpos = 0; i < chanlen; i++) {
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

	return seglen;	/*  we can serve this with MUX logic */
}

static int pcl816_ai_cmdtest(struct comedi_device *dev,
			     struct comedi_subdevice *s, struct comedi_cmd *cmd)
{
	struct pcl816_private *devpriv = dev->private;
	int err = 0;
	unsigned int arg;

	/* Step 1 : check if triggers are trivially valid */

	err |= cfc_check_trigger_src(&cmd->start_src, TRIG_NOW);
	err |= cfc_check_trigger_src(&cmd->scan_begin_src, TRIG_FOLLOW);
	err |= cfc_check_trigger_src(&cmd->convert_src, TRIG_EXT | TRIG_TIMER);
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
		err |= cfc_check_trigger_arg_min(&cmd->convert_arg, 10000);
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
		i8253_cascade_ns_to_timer(I8254_OSC_BASE_10MHZ,
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

static int pcl816_ai_cmd(struct comedi_device *dev, struct comedi_subdevice *s)
{
	struct pcl816_private *devpriv = dev->private;
	struct comedi_isadma *dma = devpriv->dma;
	struct comedi_cmd *cmd = &s->async->cmd;
	unsigned int ctrl;
	unsigned int seglen;

	if (devpriv->ai_cmd_running)
		return -EBUSY;

	pcl816_start_pacer(dev, false);

	seglen = check_channel_list(dev, s, cmd->chanlist, cmd->chanlist_len);
	if (seglen < 1)
		return -EINVAL;
	pcl816_ai_setup_chanlist(dev, cmd->chanlist, seglen);
	udelay(1);

	devpriv->ai_cmd_running = 1;
	devpriv->ai_poll_ptr = 0;
	devpriv->ai_cmd_canceled = 0;

	/* setup and enable dma for the first buffer */
	dma->cur_dma = 0;
	pcl816_ai_setup_dma(dev, s, 0);

	pcl816_start_pacer(dev, true);

	ctrl = PCL816_CTRL_INTEN | PCL816_CTRL_DMAEN | PCL816_CTRL_DMASRC_SLOT0;
	if (cmd->convert_src == TRIG_TIMER)
		ctrl |= PCL816_CTRL_PACER_TRIG;
	else	/* TRIG_EXT */
		ctrl |= PCL816_CTRL_EXT_TRIG;

	outb(ctrl, dev->iobase + PCL816_CTRL_REG);
	outb((dma->chan << 4) | dev->irq,
	     dev->iobase + PCL816_STATUS_REG);

	return 0;
}

static int pcl816_ai_poll(struct comedi_device *dev, struct comedi_subdevice *s)
{
 cancel any mode 1-4 AI
*/
static int pcl816_ai_cancel(struct comedi_device *dev,
			    struct comedi_subdevice *s)
{
/* DEBUG(printk("pcl816_ai_cancel()\n");) */

	if (devpriv->irq_blocked > 0) {
		switch (devpriv->int816_mode) {
#ifdef unused
		case INT_TYPE_AI1_DMA_RTC:
		case INT_TYPE_AI3_DMA_RTC:
			set_rtc_irq_bit(0);	/*  stop RTC */
			del_timer(&devpriv->rtc_irq_timer);
#endif
		case INT_TYPE_AI1_DMA:
		case INT_TYPE_AI3_DMA:
			disable_dma(devpriv->dma);
		case INT_TYPE_AI1_INT:
		case INT_TYPE_AI3_INT:
			outb(inb(dev->iobase + PCL816_CONTROL) & 0x73,
			     dev->iobase + PCL816_CONTROL);	/* Stop A/D */
			udelay(1);
			outb(0, dev->iobase + PCL816_CONTROL);	/* Stop A/D */

			/* Stop pacer */
			outb(0xb0, dev->iobase + PCL816_CTRCTL);
			outb(0x70, dev->iobase + PCL816_CTRCTL);
			outb(0, dev->iobase + PCL816_AD_LO);
			inb(dev->iobase + PCL816_AD_LO);
			inb(dev->iobase + PCL816_AD_HI);

			/* clear INT request */
			outb(0, dev->iobase + PCL816_CLRINT);

			/* Stop A/D */
			outb(0, dev->iobase + PCL816_CONTROL);
			devpriv->irq_blocked = 0;
			devpriv->irq_was_now_closed = devpriv->int816_mode;
			devpriv->int816_mode = 0;
			devpriv->last_int_sub = s;
/* s->busy = 0; */
			break;
		}
	}

	DEBUG(printk("comedi: pcl816_ai_cancel() successful\n");)
	    return 0;
}

/*
==============================================================================
 chech for PCL816
*/
static int pcl816_check(unsigned long iobase)
{
	outb(0x00, iobase + PCL816_MUX);
	udelay(1);
	if (inb(iobase + PCL816_MUX) != 0x00)
		return 1;	/* there isn't card */
	outb(0x55, iobase + PCL816_MUX);
	udelay(1);
	if (inb(iobase + PCL816_MUX) != 0x55)
		return 1;	/* there isn't card */
	outb(0x00, iobase + PCL816_MUX);
	udelay(1);
	outb(0x18, iobase + PCL816_CONTROL);
	udelay(1);
	if (inb(iobase + PCL816_CONTROL) != 0x18)
		return 1;	/* there isn't card */
	return 0;		/*  ok, card exist */
}

/*
==============================================================================
 reset whole PCL-816 cards
*/
static void pcl816_reset(struct comedi_device *dev)
{
/* outb (0, dev->iobase + PCL818_DA_LO);         DAC=0V */
/* outb (0, dev->iobase + PCL818_DA_HI); */
/* udelay (1); */
/* outb (0, dev->iobase + PCL818_DO_HI);        DO=$0000 */
/* outb (0, dev->iobase + PCL818_DO_LO); */
/* udelay (1); */
	outb(0, dev->iobase + PCL816_CONTROL);
	outb(0, dev->iobase + PCL816_MUX);
	outb(0, dev->iobase + PCL816_CLRINT);
	outb(0xb0, dev->iobase + PCL816_CTRCTL);	/* Stop pacer */
	outb(0x70, dev->iobase + PCL816_CTRCTL);
	outb(0x30, dev->iobase + PCL816_CTRCTL);
	outb(0, dev->iobase + PCL816_RANGE);
}

/*
==============================================================================
 Start/stop pacer onboard pacer
*/
static void
start_pacer(struct comedi_device *dev, int mode, unsigned int divisor1,
	    unsigned int divisor2)
{
	outb(0x32, dev->iobase + PCL816_CTRCTL);
	outb(0xff, dev->iobase + PCL816_CTR0);
	outb(0x00, dev->iobase + PCL816_CTR0);
	udelay(1);

	/*  set counter 2 as mode 3 */
	outb(0xb4, dev->iobase + PCL816_CTRCTL);
	/*  set counter 1 as mode 3 */
	outb(0x74, dev->iobase + PCL816_CTRCTL);
	udelay(1);

	if (mode == 1) {
		DPRINTK("mode %d, divisor1 %d, divisor2 %d\n", mode, divisor1,
			divisor2);
		outb(divisor2 & 0xff, dev->iobase + PCL816_CTR2);
		outb((divisor2 >> 8) & 0xff, dev->iobase + PCL816_CTR2);
		outb(divisor1 & 0xff, dev->iobase + PCL816_CTR1);
		outb((divisor1 >> 8) & 0xff, dev->iobase + PCL816_CTR1);
	}

	/* clear pending interrupts (just in case) */
/* outb(0, dev->iobase + PCL816_CLRINT); */
}

/*
==============================================================================
 Check if channel list from user is builded correctly
 If it's ok, then return non-zero length of repeated segment of channel list
*/
static int
check_channel_list(struct comedi_device *dev,
		   struct comedi_subdevice *s, unsigned int *chanlist,
		   unsigned int chanlen)
{
	unsigned int chansegment[16];
	unsigned int i, nowmustbechan, seglen, segpos;

	/*  correct channel and range number check itself comedi/range.c */
	if (chanlen < 1) {
		comedi_error(dev, "range/channel list is empty!");
		return 0;
	}

	if (chanlen > 1) {
		/*  first channel is every time ok */
		chansegment[0] = chanlist[0];
		for (i = 1, seglen = 1; i < chanlen; i++, seglen++) {
			/*  build part of chanlist */
			DEBUG(printk(KERN_INFO "%d. %d %d\n", i,
				     CR_CHAN(chanlist[i]),
				     CR_RANGE(chanlist[i]));)

			/*  we detect loop, this must by finish */
			    if (chanlist[0] == chanlist[i])
				break;
			nowmustbechan =
			    (CR_CHAN(chansegment[i - 1]) + 1) % chanlen;
			if (nowmustbechan != CR_CHAN(chanlist[i])) {
				/*  channel list isn't continuous :-( */
				printk(KERN_WARNING
				       "comedi%d: pcl816: channel list must "
				       "be continuous! chanlist[%i]=%d but "
				       "must be %d or %d!\n", dev->minor,
				       i, CR_CHAN(chanlist[i]), nowmustbechan,
				       CR_CHAN(chanlist[0]));
				return 0;
			}
			/*  well, this is next correct channel in list */
			chansegment[i] = chanlist[i];
		}

		/*  check whole chanlist */
		for (i = 0, segpos = 0; i < chanlen; i++) {
			DEBUG(printk("%d %d=%d %d\n",
				     CR_CHAN(chansegment[i % seglen]),
				     CR_RANGE(chansegment[i % seglen]),
				     CR_CHAN(chanlist[i]),
				     CR_RANGE(chanlist[i]));)
			    if (chanlist[i] != chansegment[i % seglen]) {
				printk(KERN_WARNING
				       "comedi%d: pcl816: bad channel or range"
				       " number! chanlist[%i]=%d,%d,%d and not"
				       " %d,%d,%d!\n", dev->minor, i,
				       CR_CHAN(chansegment[i]),
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

	return seglen;	/*  we can serve this with MUX logic */
}

/*
==============================================================================
 Program scan/gain logic with channel list.
*/
static void
setup_channel_list(struct comedi_device *dev,
		   struct comedi_subdevice *s, unsigned int *chanlist,
		   unsigned int seglen)
{
	unsigned int i;

	devpriv->ai_act_chanlist_len = seglen;
	devpriv->ai_act_chanlist_pos = 0;

	for (i = 0; i < seglen; i++) {	/*  store range list to card */
		devpriv->ai_act_chanlist[i] = CR_CHAN(chanlist[i]);
		outb(CR_CHAN(chanlist[0]) & 0xf, dev->iobase + PCL816_MUX);
		/* select gain */
		outb(CR_RANGE(chanlist[0]), dev->iobase + PCL816_RANGE);
	}

	udelay(1);
	/* select channel interval to scan */
	outb(devpriv->ai_act_chanlist[0] |
	     (devpriv->ai_act_chanlist[seglen - 1] << 4),
	     dev->iobase + PCL816_MUX);
}

#ifdef unused
/*
==============================================================================
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
#endif

/*
==============================================================================
  Free any resources that we have claimed
*/
static void free_resources(struct comedi_device *dev)
{
	/* printk("free_resource()\n"); */
	if (dev->private) {
		pcl816_ai_cancel(dev, devpriv->sub_ai);
		pcl816_reset(dev);
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
#endif
	}

	if (dev->irq)
		free_irq(dev->irq, dev);
	if (dev->iobase)
		release_region(dev->iobase, this_board->io_range);
	/* printk("free_resource() end\n"); */
}

/*
==============================================================================

   Initialization

*/
static int pcl816_attach(struct comedi_device *dev, struct comedi_devconfig *it)
{
	int ret;
	unsigned long iobase;
	unsigned int irq, dma;
	unsigned long pages;
	/* int i; */
	struct comedi_subdevice *s;

	/* claim our I/O space */
	iobase = it->options[0];
	printk("comedi%d: pcl816:  board=%s, ioport=0x%03lx", dev->minor,
	       this_board->name, iobase);

	if (!request_region(iobase, this_board->io_range, "pcl816")) {
		printk("I/O port conflict\n");
		return -EIO;
	}

	dev->iobase = iobase;

	if (pcl816_check(iobase)) {
		printk(KERN_ERR ", I cann't detect board. FAIL!\n");
		return -EIO;
	}

	ret = alloc_private(dev, sizeof(struct pcl816_private));
	if (ret < 0)
		return ret;	/* Can't alloc mem */

	/* set up some name stuff */
	dev->board_name = this_board->name;

	/* grab our IRQ */
	irq = 0;
	if (this_board->IRQbits != 0) {	/* board support IRQ */
		irq = it->options[1];
		if (irq) {	/* we want to use IRQ */
			if (((1 << irq) & this_board->IRQbits) == 0) {
				printk
				    (", IRQ %u is out of allowed range, "
				     "DISABLING IT", irq);
				irq = 0;	/* Bad IRQ */
			} else {
				if (request_irq
				    (irq, interrupt_pcl816, 0, "pcl816", dev)) {
					printk
					    (", unable to allocate IRQ %u, "
					     "DISABLING IT", irq);
					irq = 0;	/* Can't use IRQ */
				} else {
					printk(KERN_INFO ", irq=%u", irq);
				}
			}
		}
	}

	dev->irq = irq;
	if (irq)	/* 1=we have allocated irq */
		devpriv->irq_free = 1;
	else
		devpriv->irq_free = 0;

	devpriv->irq_blocked = 0;	/* number of subdevice which use IRQ */
	devpriv->int816_mode = 0;	/* mode of irq */

#ifdef unused
	/* grab RTC for DMA operations */
	devpriv->dma_rtc = 0;
	if (it->options[2] > 0) {	/*  we want to use DMA */
		if (RTC_lock == 0) {
			if (!request_region(RTC_PORT(0), RTC_IO_EXTENT,
					    "pcl816 (RTC)"))
				goto no_rtc;
		}
		devpriv->rtc_iobase = RTC_PORT(0);
		devpriv->rtc_iosize = RTC_IO_EXTENT;
		RTC_lock++;
#ifdef UNTESTED_CODE
		if (!request_irq(RTC_IRQ, interrupt_pcl816_ai_mode13_dma_rtc, 0,
				 "pcl816 DMA (RTC)", dev)) {
			devpriv->dma_rtc = 1;
			devpriv->rtc_irq = RTC_IRQ;
			printk(", dma_irq=%u", devpriv->rtc_irq);
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
#else
		printk("pcl816: RTC code missing");
#endif

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
			printk(", DMA is out of allowed range, FAIL!\n");
			return -EINVAL;	/* Bad DMA */
		}
		ret = request_dma(dma, "pcl816");
		if (ret) {
			printk(KERN_ERR
			       ", unable to allocate DMA %u, FAIL!\n", dma);
			return -EBUSY;	/* DMA isn't free */
		}

		devpriv->dma = dma;
		printk(KERN_INFO ", dma=%u", dma);
		pages = 2;	/* we need 16KB */
		devpriv->dmabuf[0] = __get_dma_pages(GFP_KERNEL, pages);

		if (!devpriv->dmabuf[0]) {
			printk(", unable to allocate DMA buffer, FAIL!\n");
			/*
			 * maybe experiment with try_to_free_pages()
			 * will help ....
			 */
			return -EBUSY;	/* no buffer :-( */
		}
		devpriv->dmapages[0] = pages;
		devpriv->hwdmaptr[0] = virt_to_bus((void *)devpriv->dmabuf[0]);
		devpriv->hwdmasize[0] = (1 << pages) * PAGE_SIZE;
		/* printk("%d %d %ld, ",devpriv->dmapages[0],devpriv->hwdmasize[0],PAGE_SIZE); */

		if (devpriv->dma_rtc == 0) {	/*  we must do duble buff :-( */
			devpriv->dmabuf[1] = __get_dma_pages(GFP_KERNEL, pages);
			if (!devpriv->dmabuf[1]) {
				printk(KERN_ERR
				       ", unable to allocate DMA buffer, "
				       "FAIL!\n");
				return -EBUSY;
			}
			devpriv->dmapages[1] = pages;
			devpriv->hwdmaptr[1] =
			    virt_to_bus((void *)devpriv->dmabuf[1]);
			devpriv->hwdmasize[1] = (1 << pages) * PAGE_SIZE;
		}
	}

no_dma:

/*  if (this_board->n_aochan > 0)
    subdevs[1] = COMEDI_SUBD_AO;
  if (this_board->n_dichan > 0)
    subdevs[2] = COMEDI_SUBD_DI;
  if (this_board->n_dochan > 0)
    subdevs[3] = COMEDI_SUBD_DO;
*/

	ret = alloc_subdevices(dev, 1);
	if (ret < 0)
		return ret;

	s = dev->subdevices + 0;
	if (this_board->n_aichan > 0) {
		s->type = COMEDI_SUBD_AI;
		devpriv->sub_ai = s;
		dev->read_subdev = s;
		s->subdev_flags = SDF_READABLE | SDF_CMD_READ;
		s->n_chan = this_board->n_aichan;
		s->subdev_flags |= SDF_DIFF;
		/* printk (", %dchans DIFF DAC - %d", s->n_chan, i); */
		s->maxdata = this_board->ai_maxdata;
		s->len_chanlist = this_board->ai_chanlist;
		s->range_table = this_board->ai_range_type;
		s->cancel = pcl816_ai_cancel;
		s->do_cmdtest = pcl816_ai_cmdtest;
		s->do_cmd = pcl816_ai_cmd;
		s->poll = pcl816_ai_poll;
		s->insn_read = pcl816_ai_insn_read;
	} else {
		s->type = COMEDI_SUBD_UNUSED;
	}

#if 0
case COMEDI_SUBD_AO:
	s->subdev_flags = SDF_WRITABLE | SDF_GROUND;
	s->n_chan = this_board->n_aochan;
	s->maxdata = this_board->ao_maxdata;
	s->len_chanlist = this_board->ao_chanlist;
	s->range_table = this_board->ao_range_type;
	break;

case COMEDI_SUBD_DI:
	s->subdev_flags = SDF_READABLE;
	s->n_chan = this_board->n_dichan;
	s->maxdata = 1;
	s->len_chanlist = this_board->n_dichan;
	s->range_table = &range_digital;
	break;

case COMEDI_SUBD_DO:
	s->subdev_flags = SDF_WRITABLE;
	s->n_chan = this_board->n_dochan;
	s->maxdata = 1;
	s->len_chanlist = this_board->n_dochan;
	s->range_table = &range_digital;
	break;
#endif

	pcl816_reset(dev);

	printk("\n");
=======
	struct pcl816_private *devpriv = dev->private;
	struct comedi_isadma *dma = devpriv->dma;
	struct comedi_isadma_desc *desc;
	unsigned long flags;
	unsigned int poll;
	int ret;

	spin_lock_irqsave(&dev->spinlock, flags);

	poll = comedi_isadma_poll(dma);
	poll = comedi_bytes_to_samples(s, poll);
	if (poll > devpriv->ai_poll_ptr) {
		desc = &dma->desc[dma->cur_dma];
		transfer_from_dma_buf(dev, s, desc->virt_addr,
				      devpriv->ai_poll_ptr,
				      poll - devpriv->ai_poll_ptr);
		/* new buffer position */
		devpriv->ai_poll_ptr = poll;

		comedi_handle_events(dev, s);

		ret = comedi_buf_n_bytes_ready(s);
	} else {
		/* no new samples */
		ret = 0;
	}
	spin_unlock_irqrestore(&dev->spinlock, flags);

	return ret;
}

static int pcl816_ai_cancel(struct comedi_device *dev,
			    struct comedi_subdevice *s)
{
	struct pcl816_private *devpriv = dev->private;

	if (!devpriv->ai_cmd_running)
		return 0;

	outb(PCL816_CTRL_DISABLE_TRIG, dev->iobase + PCL816_CTRL_REG);
	pcl816_ai_clear_eoc(dev);

	/* Stop pacer */
	i8254_set_mode(dev->iobase + PCL816_TIMER_BASE, 0,
			2, I8254_MODE0 | I8254_BINARY);
	i8254_set_mode(dev->iobase + PCL816_TIMER_BASE, 0,
			1, I8254_MODE0 | I8254_BINARY);

	devpriv->ai_cmd_running = 0;
	devpriv->ai_cmd_canceled = 1;

	return 0;
}

static int pcl816_ai_insn_read(struct comedi_device *dev,
			       struct comedi_subdevice *s,
			       struct comedi_insn *insn,
			       unsigned int *data)
{
	unsigned int chan = CR_CHAN(insn->chanspec);
	unsigned int range = CR_RANGE(insn->chanspec);
	int ret = 0;
	int i;

	outb(PCL816_CTRL_SOFT_TRIG, dev->iobase + PCL816_CTRL_REG);

	pcl816_ai_set_chan_range(dev, chan, range);
	pcl816_ai_set_chan_scan(dev, chan, chan);

	for (i = 0; i < insn->n; i++) {
		pcl816_ai_clear_eoc(dev);
		pcl816_ai_soft_trig(dev);

		ret = comedi_timeout(dev, s, insn, pcl816_ai_eoc, 0);
		if (ret)
			break;

		data[i] = pcl816_ai_get_sample(dev, s);
	}
	outb(PCL816_CTRL_DISABLE_TRIG, dev->iobase + PCL816_CTRL_REG);
	pcl816_ai_clear_eoc(dev);

	return ret ? ret : insn->n;
}

static int pcl816_di_insn_bits(struct comedi_device *dev,
			       struct comedi_subdevice *s,
			       struct comedi_insn *insn,
			       unsigned int *data)
{
	data[1] = inb(dev->iobase + PCL816_DO_DI_LSB_REG) |
		  (inb(dev->iobase + PCL816_DO_DI_MSB_REG) << 8);

	return insn->n;
}

static int pcl816_do_insn_bits(struct comedi_device *dev,
			       struct comedi_subdevice *s,
			       struct comedi_insn *insn,
			       unsigned int *data)
{
	if (comedi_dio_update_state(s, data)) {
		outb(s->state & 0xff, dev->iobase + PCL816_DO_DI_LSB_REG);
		outb((s->state >> 8), dev->iobase + PCL816_DO_DI_MSB_REG);
	}

	data[1] = s->state;

	return insn->n;
}

static void pcl816_reset(struct comedi_device *dev)
{
	unsigned long timer_base = dev->iobase + PCL816_TIMER_BASE;

	outb(PCL816_CTRL_DISABLE_TRIG, dev->iobase + PCL816_CTRL_REG);
	pcl816_ai_set_chan_range(dev, 0, 0);
	pcl816_ai_clear_eoc(dev);

	/* Stop pacer */
	i8254_set_mode(timer_base, 0, 2, I8254_MODE0 | I8254_BINARY);
	i8254_set_mode(timer_base, 0, 1, I8254_MODE0 | I8254_BINARY);
	i8254_set_mode(timer_base, 0, 0, I8254_MODE0 | I8254_BINARY);

	/* set all digital outputs low */
	outb(0, dev->iobase + PCL816_DO_DI_LSB_REG);
	outb(0, dev->iobase + PCL816_DO_DI_MSB_REG);
}

static void pcl816_alloc_irq_and_dma(struct comedi_device *dev,
				     struct comedi_devconfig *it)
{
	struct pcl816_private *devpriv = dev->private;
	unsigned int irq_num = it->options[1];
	unsigned int dma_chan = it->options[2];

	/* only IRQs 2-7 and DMA channels 3 and 1 are valid */
	if (!(irq_num >= 2 && irq_num <= 7) ||
	    !(dma_chan == 3 || dma_chan == 1))
		return;

	if (request_irq(irq_num, pcl816_interrupt, 0, dev->board_name, dev))
		return;

	/* DMA uses two 16K buffers */
	devpriv->dma = comedi_isadma_alloc(dev, 2, dma_chan, dma_chan,
					   PAGE_SIZE * 4, COMEDI_ISADMA_READ);
	if (!devpriv->dma)
		free_irq(irq_num, dev);
	else
		dev->irq = irq_num;
}

static void pcl816_free_dma(struct comedi_device *dev)
{
	struct pcl816_private *devpriv = dev->private;

	if (devpriv)
		comedi_isadma_free(devpriv->dma);
}

static int pcl816_attach(struct comedi_device *dev, struct comedi_devconfig *it)
{
	const struct pcl816_board *board = dev->board_ptr;
	struct pcl816_private *devpriv;
	struct comedi_subdevice *s;
	int ret;

	devpriv = comedi_alloc_devpriv(dev, sizeof(*devpriv));
	if (!devpriv)
		return -ENOMEM;

	ret = comedi_request_region(dev, it->options[0], 0x10);
	if (ret)
		return ret;

	/* an IRQ and DMA are required to support async commands */
	pcl816_alloc_irq_and_dma(dev, it);

	ret = comedi_alloc_subdevices(dev, 4);
	if (ret)
		return ret;

	s = &dev->subdevices[0];
	s->type		= COMEDI_SUBD_AI;
	s->subdev_flags	= SDF_CMD_READ | SDF_DIFF;
	s->n_chan	= 16;
	s->maxdata	= board->ai_maxdata;
	s->range_table	= &range_pcl816;
	s->insn_read	= pcl816_ai_insn_read;
	if (dev->irq) {
		dev->read_subdev = s;
		s->subdev_flags	|= SDF_CMD_READ;
		s->len_chanlist	= board->ai_chanlist;
		s->do_cmdtest	= pcl816_ai_cmdtest;
		s->do_cmd	= pcl816_ai_cmd;
		s->poll		= pcl816_ai_poll;
		s->cancel	= pcl816_ai_cancel;
	}

	/* Analog OUtput subdevice */
	s = &dev->subdevices[2];
	s->type		= COMEDI_SUBD_UNUSED;
#if 0
	subdevs[1] = COMEDI_SUBD_AO;
	s->subdev_flags = SDF_WRITABLE | SDF_GROUND;
	s->n_chan = 1;
	s->maxdata = board->ao_maxdata;
	s->range_table = &range_pcl816;
#endif

	/* Digital Input subdevice */
	s = &dev->subdevices[2];
	s->type		= COMEDI_SUBD_DI;
	s->subdev_flags	= SDF_READABLE;
	s->n_chan	= 16;
	s->maxdata	= 1;
	s->range_table	= &range_digital;
	s->insn_bits	= pcl816_di_insn_bits;

	/* Digital Output subdevice */
	s = &dev->subdevices[3];
	s->type		= COMEDI_SUBD_DO;
	s->subdev_flags	= SDF_WRITABLE;
	s->n_chan	= 16;
	s->maxdata	= 1;
	s->range_table	= &range_digital;
	s->insn_bits	= pcl816_do_insn_bits;

	pcl816_reset(dev);

	return 0;
}

  Removes device
 */
static int pcl816_detach(struct comedi_device *dev)
{
	DEBUG(printk(KERN_INFO "comedi%d: pcl816: remove\n", dev->minor);)
	    free_resources(dev);
#ifdef unused
	if (devpriv->dma_rtc)
		RTC_lock--;
#endif
	return 0;
}

=======
static void pcl816_detach(struct comedi_device *dev)
{
	if (dev->private) {
		pcl816_ai_cancel(dev, dev->read_subdev);
		pcl816_reset(dev);
	}
	pcl816_free_dma(dev);
	comedi_legacy_detach(dev);
}

static struct comedi_driver pcl816_driver = {
	.driver_name	= "pcl816",
	.module		= THIS_MODULE,
	.attach		= pcl816_attach,
	.detach		= pcl816_detach,
	.board_name	= &boardtypes[0].name,
	.num_names	= ARRAY_SIZE(boardtypes),
	.offset		= sizeof(struct pcl816_board),
};
module_comedi_driver(pcl816_driver);

MODULE_AUTHOR("Comedi http://www.comedi.org");
MODULE_DESCRIPTION("Comedi low-level driver");
MODULE_LICENSE("GPL");
