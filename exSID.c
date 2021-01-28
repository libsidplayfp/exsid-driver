//
//  exSID.c
//	A simple I/O library for exSID/exSID+ USB
//
//  (C) 2015-2018,2021 Thibaut VARENE
//  License: GPLv2 - http://www.gnu.org/licenses/gpl-2.0.html

/**
 * @file
 * exSID/exSID+ USB I/O library
 * @author Thibaut VARENE
 * @date 2015-2018,2021
 * @version 2.0pre
 *
 * This driver will control the first exSID device available.
 * All public API functions are only valid after a successful call to exSID_init().
 * To release the device and resources, exSID_exit() and exSID_free() must be called.
 */

#include "exSID.h"
#include "exSID_defs.h"
#include "exSID_ftdiwrap.h"
#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <stdlib.h>

#ifdef	EXSID_THREADED
 #if defined(HAVE_THREADS_H)	// Native C11 threads support
  #include <threads.h>
 #elif defined(HAVE_PTHREAD_H)	// Trivial C11 over pthreads support
  #include "c11threads.h"
 #else
  #error "No thread model available"
 #endif
#endif	// EXSID_TRHREADED

#define ARRAY_SIZE(x)		(sizeof(x) / sizeof(x[0]))
#define xserror(xs, format, ...)      snprintf(xs->xSerrstr, ERRORBUF, "(%s) ERROR " format, __func__, ## __VA_ARGS__)

/**
 * cycles is uint_fast32_t. Technically, clkdrift should be int_fast64_t though
 * overflow should not happen under normal conditions.
 */
typedef int_fast32_t clkdrift_t;

/**
 * This private structure holds hardware-dependent constants.
 */
struct xSconsts_s {
	unsigned int	model;			///< exSID device model in use
	clkdrift_t	write_cycles;		///< number of SID clocks spent in write ops
	clkdrift_t	read_pre_cycles;	///< number of SID clocks spent in read op before data is actually read
	clkdrift_t	read_post_cycles;	///< number of SID clocks spent in read op after data is actually read
	clkdrift_t	read_offset_cycles;	///< read offset adjustment to align with writes (see function documentation)
	clkdrift_t	csioctl_cycles;		///< number of SID clocks spent in chip select ioctl
	clkdrift_t	mindel_cycles;		///< lowest number of SID clocks that can be accounted for in delay
	clkdrift_t	max_adj;		///< maximum number of SID clocks that can be encoded in final delay for read()/write()
	clkdrift_t	ldelay_offs;		///< long delay SID clocks offset
	size_t		buff_size;		///< output buffer size
};

/** Array of supported devices */
const struct {
	const char *		desc;
	const int		pid;
	const int		vid;
	const struct xSconsts_s	xsc;
} xSsupported[] = {
	{
		/* exSID USB */
		.desc = XS_USBDSC,
		.pid = XS_USBPID,
		.vid = XS_USBVID,
		.xsc = (struct xSconsts_s){
			.model = XS_MODEL_STD,
			.write_cycles = XS_CYCIO,
			.read_pre_cycles = XS_CYCCHR,
			.read_post_cycles = XS_CYCCHR,
			.read_offset_cycles = -2,	// see exSID_clkdread() documentation
			.csioctl_cycles = XS_CYCCHR,
			.mindel_cycles = XS_MINDEL,
			.max_adj = XS_MAXADJ,
			.ldelay_offs = XS_LDOFFS,
			.buff_size = XS_BUFFSZ,
		},
	}, {
		/* exSID+ USB */
		.desc = XSP_USBDSC,
		.pid = XSP_USBPID,
		.vid = XSP_USBVID,
		.xsc = (struct xSconsts_s){
			.model = XS_MODEL_PLUS,
			.write_cycles = XSP_CYCIO,
			.read_pre_cycles = XSP_PRE_RD,
			.read_post_cycles = XSP_POSTRD,
			.read_offset_cycles = 0,
			.csioctl_cycles = XSP_CYCCS,
			.mindel_cycles = XSP_MINDEL,
			.max_adj = XSP_MAXADJ,
			.ldelay_offs = XSP_LDOFFS,
			.buff_size = XSP_BUFFSZ,
		},
	}
};

/** exSID private handle */
struct _exsid {
	/** negative values mean we're lagging, positive mean we're ahead. See it as a number of SID clocks queued to be spent. */
	clkdrift_t clkdrift;

	/** Pointer to constants used by all the hardware access routines */
	const struct xSconsts_s * restrict xSconsts;

	void * ftdi;
	int ftdi_status;

#ifdef	DEBUG
	long accdrift;
	unsigned long accioops;
	unsigned long accdelay;
	unsigned long acccycle;	// ensures no overflow with exSID+ up to ~1h of continuous playback
#endif

	int backbuf_idx;
	unsigned char * restrict backbuf;

#ifdef	EXSID_THREADED
	// Variables for flip buffering
	int frontbuf_idx;
	unsigned char * restrict frontbuf;
	mtx_t frontbuf_mtx;	///< mutex protecting access to frontbuf
	cnd_t frontbuf_ready_cnd, frontbuf_done_cnd;
	thrd_t thread_output;
#endif	// EXSID_THREADED

	char xSerrstr[ERRORBUF+1];	// 256-byte max string for error message
};

static inline void _exSID_write(struct _exsid * const xs, uint_least8_t addr, uint8_t data, int flush);

/**
 * Returns a string describing the last recorded error.
 * @param exsid exsid handle
 * @return error message (max 256 bytes long).
 */
const char * exSID_error_str(void * const exsid)
{
	const struct _exsid * const xs = exsid;
	if (!exsid)
		return NULL;
	return (xs->xSerrstr);
}


/**
 * Write routine to send data to the device.
 * @note BLOCKING.
 * @param xs exsid private pointer
 * @param buff pointer to a byte array of data to send
 * @param size number of bytes to send
 */
static inline void xSwrite(struct _exsid * const xs, const unsigned char * buff, int size)
{
	xs->ftdi_status = xSfw_write_data(xs->ftdi, buff, size);
#ifdef	DEBUG
	if (unlikely(xs->ftdi_status < 0)) {
		xsdbg("Error ftdi_write_data(%d): %s\n",
			xs->ftdi_status, xSfw_get_error_string(xs->ftdi));
	}
	if (unlikely(xs->ftdi_status != size)) {
		xsdbg("ftdi_write_data only wrote %d (of %d) bytes\n",
			xs->ftdi_status, size);
	}
#endif
}

/**
 * Read routine to get data from the device.
 * @note BLOCKING.
 * @param xs exsid private pointer
 * @param buff pointer to a byte array that will be filled with read data
 * @param size number of bytes to read
 */
static void xSread(struct _exsid * const xs, unsigned char * buff, int size)
{
#ifdef	EXSID_THREADED
	mtx_lock(&xs->frontbuf_mtx);
	while (xs->frontbuf_idx)
		cnd_wait(&xs->frontbuf_done_cnd, &xs->frontbuf_mtx);
#endif
	xs->ftdi_status = xSfw_read_data(xs->ftdi, buff, size);
#ifdef	EXSID_THREADED
	mtx_unlock(&xs->frontbuf_mtx);
#endif

#ifdef	DEBUG
	if (unlikely(xs->ftdi_status < 0)) {
		xsdbg("Error ftdi_read_data(%d): %s\n",
			xs->ftdi_status, xSfw_get_error_string(xs->ftdi));
	}
	if (unlikely(xs->ftdi_status != size)) {
		xsdbg("ftdi_read_data only read %d (of %d) bytes\n",
			xs->ftdi_status, size);
	}
#endif
}


#ifdef	EXSID_THREADED
/**
 * Writer thread. ** consummer **
 * This thread consumes buffer prepared in xSoutb().
 * Since writes to the FTDI subsystem are blocking, this thread blocks when it's
 * writing to the chip, and also while it's waiting for the front buffer to be ready.
 * This ensures execution time consistency as xSoutb() periodically waits for
 * the front buffer to be ready before flipping buffers.
 * @note BLOCKING.
 * @param arg ignored
 * @return DOES NOT RETURN, exits when frontbuf_idx is negative.
 */
static int _exSID_thread_output(void * arg)
{
	struct _exsid * const xs = arg;

	xsdbg("thread started\n");
	while (1) {
		mtx_lock(&xs->frontbuf_mtx);

		// wait for frontbuf ready (not empty)
		while (!xs->frontbuf_idx)
			cnd_wait(&xs->frontbuf_ready_cnd, &xs->frontbuf_mtx);

		if (unlikely(xs->frontbuf_idx < 0)) {	// exit condition
			xsdbg("thread exiting!\n");
			mtx_unlock(&xs->frontbuf_mtx);
			thrd_exit(0);
		}

		xSwrite(xs, xs->frontbuf, xs->frontbuf_idx);
		xs->frontbuf_idx = 0;

		// xSread() and xSoutb() are in the same thread of execution
		// so it can only be one or the other waiting.
		cnd_signal(&xs->frontbuf_done_cnd);
		mtx_unlock(&xs->frontbuf_mtx);
	}
	return 0;	// make the compiler happy
}
#endif	// EXSID_THREADED

/**
 * Single byte output routine. ** producer **
 * Fills a static buffer with bytes to send to the device until the buffer is
 * full or a forced write is triggered.
 * @note No drift compensation is performed on read operations.
 * @param xs exsid private pointer
 * @param byte byte to send
 * @param flush force write flush if positive, trigger thread exit if negative
 */
static void xSoutb(struct _exsid * const xs, uint8_t byte, int flush)
{
#ifdef	EXSID_THREADED
	unsigned char * bufptr;
#endif

	xs->backbuf[xs->backbuf_idx++] = (unsigned char)byte;

	if (likely((xs->backbuf_idx < xs->xSconsts->buff_size) && !flush))
		return;

#ifdef	EXSID_THREADED
	// buffer dance
	mtx_lock(&xs->frontbuf_mtx);

	// wait for frontbuf available (empty). Only triggers if previous
	// write buffer hasn't been consummed before we get here again.
	while (xs->frontbuf_idx)
		cnd_wait(&xs->frontbuf_done_cnd, &xs->frontbuf_mtx);

	if (unlikely(flush < 0))	// indicate exit request
		xs->frontbuf_idx = -1;
	else {				// flip buffers
		bufptr = xs->frontbuf;
		xs->frontbuf = xs->backbuf;
		xs->frontbuf_idx = xs->backbuf_idx;
		xs->backbuf = bufptr;
		xs->backbuf_idx = 0;
	}

	cnd_signal(&xs->frontbuf_ready_cnd);
	mtx_unlock(&xs->frontbuf_mtx);
#else	// unthreaded
	xSwrite(xs, xs->backbuf, xs->backbuf_idx);
	xs->backbuf_idx = 0;
#endif
}

/**
 * Allocate an exSID handle.
 * @return allocated opaque handle, NULL if error.
 */
void * exSID_new(void)
{
	struct _exsid * xs;
	xs = calloc(1, sizeof(*xs));
	return xs;
}

/**
 * Deallocate an exSID handle.
 * Frees up all memory used by the exSID handle.
 * @param exsid exsid handle
 */
void exSID_free(void * exsid)
{
	struct _exsid * xs = exsid;

	if (!xs)
		return;

	if (xSfw_free)
		xSfw_free(xs->ftdi);

#ifdef	EXSID_THREADED
	if (xs->frontbuf)
		free(xs->frontbuf);
#endif

	if (xs->backbuf)
		free(xs->backbuf);

	free(xs);
}

/**
 * Device init routine.
 * Must be called once before any operation is attempted on the device.
 * Opens first available device, and sets various parameters: baudrate, parity, flow
 * control and USB latency, and finally clears the RX and TX buffers.
 * If this function fails, exSID_free() must still be called.
 * @param exsid allocated exsid handle generated with exSID_new()
 * @return 0 on success, !0 otherwise.
 */
int exSID_init(void * const exsid)
{
	struct _exsid * const xs = exsid;
	unsigned char dummy;
	int i, found, ret;

	if (!exsid) {
		fprintf(stderr, "ERROR: exSID_init: invalid handle\n");
		return -1;
	}

	if (xSfw_dlopen()) {
		xserror(xs, "Failed to open dynamic loader");
		return -1;
	}

	/* Attempt to open all supported devices until first success.
	 * Cleanup ftdi after each try to avoid passing garbage around as we don't know what
	 * the FTDI open routines do with the pointer.
	 * FIXME: despite that, some combinations still don't seem to work */
	found = 0;
	for (i = 0; i < ARRAY_SIZE(xSsupported); i++) {
		if (xSfw_new) {
			xs->ftdi = xSfw_new();
			if (!xs->ftdi) {
				xserror(xs, "ftdi_new failed");
				return -1;
			}
		}

		xsdbg("Trying %s...\n", xSsupported[i].desc);
		xs->xSconsts = &xSsupported[i].xsc;	// setting unconditionnally avoids segfaults if user code does the wrong thing.
		xs->ftdi_status = xSfw_usb_open_desc(&xs->ftdi, xSsupported[i].vid, xSsupported[i].pid, xSsupported[i].desc, NULL);
		if (xs->ftdi_status >= 0) {
			xsdbg("Opened!\n");
			found = 1;
			break;
		}
		else {
			xsdbg("Failed: %d (%s)\n", xs->ftdi_status, xSfw_get_error_string(xs->ftdi));
			if (xSfw_free)
				xSfw_free(xs->ftdi);
			xs->ftdi = NULL;
		}
	}

	if (!found) {
		xserror(xs, "No device could be opened");
		return -1;
	}

	xs->ftdi_status = xSfw_usb_setup(xs->ftdi, XS_BDRATE, XS_USBLAT);
	if (xs->ftdi_status < 0) {
		xserror(xs, "Failed to setup device");
		return -1;
	}

	// success - device is ready
	xsdbg("Device ready\n");

	xs->backbuf = malloc(xs->xSconsts->buff_size);
	if (!xs->backbuf) {
		xserror(xs, "Out of memory!");
		return -1;
	}

#ifdef	EXSID_THREADED
	xsdbg("Thread setup\n");

	xs->frontbuf = malloc(xs->xSconsts->buff_size);
	if (!xs->frontbuf) {
		xserror(xs, "Out of memory!");
		return -1;
	}

	ret = mtx_init(&xs->frontbuf_mtx, mtx_plain);
	ret |= cnd_init(&xs->frontbuf_ready_cnd);
	ret |= cnd_init(&xs->frontbuf_done_cnd);
	ret |= thrd_create(&xs->thread_output, _exSID_thread_output, xs);
	if (ret) {
		xserror(xs, "Thread setup failed");
		return -1;
	}
#endif

	xSfw_usb_purge_buffers(xs->ftdi); // Purge both Rx and Tx buffers

	// Wait for device ready by trying to read FV and wait for the answer
	// XXX Broken with libftdi due to non-blocking read :-/
	xSoutb(xs, XS_AD_IOCTFV, 1);
	xSread(xs, &dummy, 1);

	xsdbg("Rock'n'roll!\n");

#ifdef	DEBUG
	exSID_hwversion(xs);
	xsdbg("XS_BUFFSZ: %d bytes\n", XS_BUFFSZ);
#endif

	return 0;
}

/**
 * Device exit routine.
 * Must be called to release the device.
 * Resets the SIDs and clears RX/TX buffers, closes FTDI device.
 * @param exsid exsid handle
 */
void exSID_exit(void * const exsid)
{
	struct _exsid * const xs = exsid;

	if (!exsid)
		return;

	if (xs->ftdi) {
		exSID_reset(xs, 0);

#ifdef	EXSID_THREADED
		xSoutb(xs, XS_AD_IOCTFV, -1);	// signal end of thread
		thrd_join(xs->thread_output, NULL);
		cnd_destroy(&xs->frontbuf_ready_cnd);
		mtx_destroy(&xs->frontbuf_mtx);
#endif

		xSfw_usb_purge_buffers(xs->ftdi); // Purge both Rx and Tx buffers

		xs->ftdi_status = xSfw_usb_close(xs->ftdi);
		if (xs->ftdi_status < 0)
			xserror(xs, "Unable to close ftdi device: %d (%s)",
				xs->ftdi_status, xSfw_get_error_string(xs->ftdi));

#ifdef	DEBUG
		xsdbg("mean jitter: %.2f cycle(s) over %lu I/O ops\n",
			((float)xs->accdrift/xs->accioops), xs->accioops);
		xsdbg("bandwidth used for I/O ops: %lu%% (approx)\n",
			100-(xs->accdelay*100/xs->acccycle));
		xs->accdrift = xs->accioops = xs->accdelay = xs->acccycle = 0;
#endif
	}

	xs->clkdrift = 0;
	xSfw_dlclose();
}


/**
 * SID reset routine.
 * Performs a hardware reset on the SIDs.
 * @note since the reset procedure in firmware will stall the device,
 * reset forcefully waits for enough time before resuming execution
 * via a call to usleep();
 * @param exsid exsid handle
 * @param volume volume to set the SIDs to after reset.
 */
void exSID_reset(void * const exsid, uint_least8_t volume)
{
	struct _exsid * const xs = exsid;

	if (!xs)
		return;

	xsdbg("rvol: %" PRIxLEAST8 "\n", volume);

	xSoutb(xs, XS_AD_IOCTRS, 1);	// this will stall
	usleep(100);	// sleep for 100us
	_exSID_write(xs, 0x18, volume, 1);	// this only needs 2 bytes which matches the input buffer of the PIC so all is well

	xs->clkdrift = 0;
}


/**
 * exSID+ clock selection routine.
 * Selects between PAL, NTSC and 1MHz clocks.
 * @note upon clock change the hardware resync itself and resets the SIDs, which
 * takes approximately 50us: this function waits for enough time before resuming
 * execution via a call to usleep();
 * Output should be muted before execution
 * @param exsid exsid handle
 * @param clock clock selector value, see exSID.h.
 * @return execution status
 */
int exSID_clockselect(void * const exsid, int clock)
{
	struct _exsid * const xs = exsid;

	if (!xs)
		return -1;

	xsdbg("clk: %d\n", clock);

	if (XS_MODEL_PLUS != xs->xSconsts->model)
		return -1;

	switch (clock) {
		case XS_CL_PAL:
			xSoutb(xs, XSP_AD_IOCTCP, 1);
			break;
		case XS_CL_NTSC:
			xSoutb(xs, XSP_AD_IOCTCN, 1);
			break;
		case XS_CL_1MHZ:
			xSoutb(xs, XSP_AD_IOCTC1, 1);
			break;
		default:
			return -1;
	}

	usleep(100);	// sleep for 100us

	xs->clkdrift = 0;	// reset drift

	return 0;
}

/**
 * exSID+ audio operations routine.
 * Selects the audio mixing / muting option. Only implemented in exSID+ devices.
 * @warning all these operations (excepting unmuting obviously) will mute the
 * output by default.
 * @note no accounting for SID cycles consumed.
 * @param exsid exsid handle
 * @param operation audio operation value, see exSID.h.
 * @return execution status
 */
int exSID_audio_op(void * const exsid, int operation)
{
	struct _exsid * const xs = exsid;

	if (!xs)
		return -1;

	xsdbg("auop: %d\n", operation);

	if (XS_MODEL_PLUS != xs->xSconsts->model)
		return -1;

	switch (operation) {
		case XS_AU_6581_8580:
			xSoutb(xs, XSP_AD_IOCTA0, 0);
			break;
		case XS_AU_8580_6581:
			xSoutb(xs, XSP_AD_IOCTA1, 0);
			break;
		case XS_AU_8580_8580:
			xSoutb(xs, XSP_AD_IOCTA2, 0);
			break;
		case XS_AU_6581_6581:
			xSoutb(xs, XSP_AD_IOCTA3, 0);
			break;
		case XS_AU_MUTE:
			xSoutb(xs, XSP_AD_IOCTAM, 0);
			break;
		case XS_AU_UNMUTE:
			xSoutb(xs, XSP_AD_IOCTAU, 0);
			break;
		default:
			return -1;
	}

	return 0;
}

/**
 * SID chipselect routine.
 * Selects which SID will play the tunes. If neither CHIP0 or CHIP1 is chosen,
 * both SIDs will operate together. Accounts for elapsed cycles.
 * @param exsid exsid handle
 * @param chip SID selector value, see exSID.h.
 */
void exSID_chipselect(void * const exsid, int chip)
{
	struct _exsid * const xs = exsid;

	if (!xs)
		return;

	xs->clkdrift -= xs->xSconsts->csioctl_cycles;

	xsdbg("cs: %d\n", chip);

	if (XS_CS_CHIP0 == chip)
		xSoutb(xs, XS_AD_IOCTS0, 0);
	else if (XS_CS_CHIP1 == chip)
		xSoutb(xs, XS_AD_IOCTS1, 0);
	else
		xSoutb(xs, XS_AD_IOCTSB, 0);
}

/**
 * Device hardware model.
 * Queries the driver for the hardware model currently identified.
 * @param exsid exsid handle
 * @return hardware model as enumerated in exSID.h, negative value on error.
 */
int exSID_hwmodel(void * const exsid)
{
	struct _exsid * const xs = exsid;

	if (!xs)
		return -1;

	int model;

	switch (xs->xSconsts->model) {
		case XS_MODEL_STD:
			model = XS_MD_STD;
			break;
		case XS_MODEL_PLUS:
			model = XS_MD_PLUS;
			break;
		default:
			model = -1;
			break;
	}

	xsdbg("HW model: %d\n", model);

	return model;
}

/**
 * Hardware and firmware version of the device.
 * Queries the device for the hardware revision and current firmware version
 * and returns both in the form of a 16bit integer: MSB is an ASCII
 * character representing the hardware revision (e.g. 0x42 = "B"), and LSB
 * is a number representing the firmware version in decimal integer.
 * Does NOT account for elapsed cycles.
 * @param exsid exsid handle
 * @return version information as described above.
 */
uint16_t exSID_hwversion(void * const exsid)
{
	struct _exsid * const xs = exsid;
	unsigned char inbuf[2];
	uint16_t out = 0;

	if (!xs)
		return -1;

	xSoutb(xs, XS_AD_IOCTHV, 0);
	xSoutb(xs, XS_AD_IOCTFV, 1);
	xSread(xs, inbuf, 2);
	out = inbuf[0] << 8 | inbuf[1];	// ensure proper order regardless of endianness

	xsdbg("HV: %c, FV: %hhu\n", inbuf[0], inbuf[1]);

	return out;
}

/**
 * Private busy delay loop.
 * @note will block every time a device write is triggered, blocking time will be
 * equal to the number of bytes written times mindel_cycles.
 * @param xs exsid private pointer
 * @param cycles how many SID clocks to loop for.
 */
static inline void xSdelay(struct _exsid * const xs, uint_fast32_t cycles)
{
#ifdef	DEBUG
	xs->accdelay += cycles;
#endif
	while (likely(cycles >= xs->xSconsts->mindel_cycles)) {
		xSoutb(xs, XS_AD_IOCTD1, 0);
		cycles -= xs->xSconsts->mindel_cycles;
		xs->clkdrift -= xs->xSconsts->mindel_cycles;
	}
#ifdef	DEBUG
	xs->accdelay -= cycles;
#endif
}

/**
 * Private long delay loop.
 * Calls to IOCTLD delay, for "very" long delays (thousands of SID clocks).
 * Requested delay @b MUST be > ldelay_offs, and for better performance,
 * the requested delay time should ideally be several XS_LDMULT and be close to
 * a multiple of XS_USBLAT milliseconds (on the exSID).
 * @warning polling and NOT CYCLE ACCURATE on exSID
 * @param xs exsid private pointer
 * @param cycles how many SID clocks to wait for.
 */
static void xSlongdelay(struct _exsid * const xs, uint_fast32_t cycles)
{
	int multiple, flush;
	uint_fast32_t delta;
	unsigned char dummy;

	flush = (XS_MODEL_STD == xs->xSconsts->model);

	multiple = cycles - xs->xSconsts->ldelay_offs;
	delta = multiple % XS_LDMULT;
	multiple /= XS_LDMULT;

	//xsdbg("ldelay: %" PRIdFAST32 ", multiple: %d, delta: %" PRIdFAST32 "\n", cycles, multiple, delta);

	if (unlikely(multiple < 0)) {
		xsdbg("Wrong delay!\n");
		return;
	}

#ifdef	DEBUG
	xs->accdelay += (cycles - delta);
#endif

	while (multiple >= 255) {
		_exSID_write(xs, XS_AD_IOCTLD, 255, flush);
		if (flush)
			xSread(xs, &dummy, 1);	// wait for answer with blocking read
		multiple -= 255;
	}

	if (multiple) {
		_exSID_write(xs, XS_AD_IOCTLD, (unsigned char)multiple, flush);
		if (flush)
			xSread(xs, &dummy, 1);	// wait for answer with blocking read
	}

	// deal with remainder
	xSdelay(xs, delta);
}

/**
 * Cycle accurate delay routine.
 * Applies the most efficient strategy to delay for cycles SID clocks
 * while leaving enough lead time for an I/O operation.
 * @param exsid exsid handle
 * @param cycles how many SID clocks to loop for.
 */
void exSID_delay(void * const exsid, uint_fast32_t cycles)
{
	struct _exsid * const xs = exsid;
	uint_fast32_t delay;

	if (unlikely(!xs))
		return;

	xs->clkdrift += cycles;
#ifdef	DEBUG
	xs->acccycle += cycles;
#endif

	if (unlikely(xs->clkdrift <= xs->xSconsts->write_cycles))	// never delay for less than a full write would need
		return;	// too short

	delay = xs->clkdrift - xs->xSconsts->write_cycles;

	switch (xs->xSconsts->model) {
#if 0	// currently breaks sidplayfp - REVIEW
		case XS_MODEL_PLUS:
			if (delay > XS_LDMULT) {
				xSlongdelay(xs, delay);
				break;
			}
#endif
		default:
			xSdelay(xs, delay);
	}
}

/**
 * Private write routine for a tuple address + data.
 * @param xs exsid private pointer
 * @param addr target address to write to.
 * @param data data to write at that address.
 * @param flush if non-zero, force immediate flush to device.
 */
static inline void _exSID_write(struct _exsid * const xs, uint_least8_t addr, uint8_t data, int flush)
{
	xSoutb(xs, (unsigned char)addr, 0);
	xSoutb(xs, (unsigned char)data, flush);
}

/**
 * Timed write routine, attempts cycle-accurate writes.
 * This function will be cycle-accurate provided that no two consecutive reads or writes
 * are less than write_cycles apart and the leftover delay is <= max_adj SID clock cycles.
 * @param exsid exsid handle
 * @param cycles how many SID clocks to wait before the actual data write.
 * @param addr target address.
 * @param data data to write at that address.
 */
void exSID_clkdwrite(void * const exsid, uint_fast32_t cycles, uint_least8_t addr, uint8_t data)
{
	struct _exsid * const xs = exsid;
	int adj;

	if (unlikely(!xs))
		return;

#ifdef	DEBUG
	if (unlikely(addr > 0x18)) {
		xsdbg("Invalid write: %.2" PRIxLEAST8 "\n", addr);
		exSID_delay(xs, cycles);
		return;
	}
#endif

	// actual write will cost write_cycles. Delay for cycles - write_cycles then account for the write
	xs->clkdrift += cycles;
	if (xs->clkdrift > xs->xSconsts->write_cycles)
		xSdelay(xs, xs->clkdrift - xs->xSconsts->write_cycles);

	xs->clkdrift -= xs->xSconsts->write_cycles;	// write is going to consume write_cycles clock ticks

#ifdef	DEBUG
	if (xs->clkdrift >= xs->xSconsts->mindel_cycles)
		xsdbg("Impossible drift adjustment! %" PRIdFAST32 " cycles\n", xs->clkdrift);
	else if (xs->clkdrift < 0)
		xs->accdrift += xs->clkdrift;
#endif

	/* if we are still going to be early, delay actual write by up to XS_MAXAD ticks
	At this point it is guaranted that clkdrift will be < mindel_cycles. */
	if (likely(xs->clkdrift >= 0)) {
		adj = xs->clkdrift % (xs->xSconsts->max_adj+1);
		/* if max_adj is >= clkdrift, modulo will give the same results
		   as the correct test:
		   adj = (clkdrift < max_adj ? clkdrift : max_adj)
		   but without an extra conditional branch. If is is < max_adj, then it
		   seems to provide better results by evening jitter accross writes. So
		   it's the preferred solution for all cases. */
		addr = (unsigned char)(addr | (adj << 5));	// final delay encoded in top 3 bits of address
#ifdef	DEBUG
		xs->accdrift += (xs->clkdrift - adj);
#endif
		//xsdbg("drft: %d, adj: %d, addr: %.2hhx, data: %.2hhx\n", clkdrift, adj, (char)(addr | (adj << 5)), data);
	}

#ifdef	DEBUG
	xs->acccycle += cycles;
	xs->accioops++;
#endif

	//xsdbg("delay: %d, clkdrift: %d\n", cycles, xs->clkdrift);
	_exSID_write(xs, addr, data, 0);
}

/**
 * Private read routine for a given address.
 * @param xs exsid private pointer
 * @param addr target address to read from.
 * @param flush if non-zero, force immediate flush to device.
 * @return data read from address.
 */
static inline uint8_t _exSID_read(struct _exsid * const xs, uint_least8_t addr, int flush)
{
	unsigned char data;

	xSoutb(xs, addr, flush);	// XXX
	xSread(xs, &data, 1);		// blocking

	xsdbg("addr: %.2" PRIxLEAST8 ", data: %.2hhx\n", addr, data);
	return data;
}

/**
 * BLOCKING Timed read routine, attempts cycle-accurate reads.
 * The following description is based on exSID (standard).
 * This function will be cycle-accurate provided that no two consecutive reads or writes
 * are less than XS_CYCIO apart and leftover delay is <= max_adj SID clock cycles.
 * Read result will only be available after a full XS_CYCIO, giving clkdread() the same
 * run time as clkdwrite(). There's a 2-cycle negative adjustment in the code because
 * that's the actual offset from the write calls ('/' denotes falling clock edge latch),
 * which the following ASCII tries to illustrate: <br />
 * Write looks like this in firmware:
 * > ...|_/_|...
 * ...end of data byte read | cycle during which write is enacted / next cycle | etc... <br />
 * Read looks like this in firmware:
 * > ...|_|_|_/_|_|...
 * ...end of address byte read | 2 cycles for address processing | cycle during which SID is read /
 *	then half a cycle later the CYCCHR-long data TX starts, cycle completes | another cycle | etc... <br />
 * This explains why reads happen a relative 2-cycle later than then should with
 * respect to writes.
 * @note The actual time the read will take to complete depends
 * on the USB bus activity and settings. It *should* complete in XS_USBLAT ms, but
 * not less, meaning that read operations are bound to introduce timing inaccuracy.
 * As such, this function is only really provided as a proof of concept but SHOULD
 * BETTER BE AVOIDED.
 * @param exsid exsid handle
 * @param cycles how many SID clocks to wait before the actual data read.
 * @param addr target address.
 * @return data read from address.
 */
uint8_t exSID_clkdread(void * const exsid, uint_fast32_t cycles, uint_least8_t addr)
{
	struct _exsid * const xs = exsid;
	int adj;

	if (unlikely(!xs))
		return 0xFF;

#ifdef	DEBUG
	if (unlikely((addr < 0x19) || (addr > 0x1C))) {
		xsdbg("Invalid read: %.2" PRIxLEAST8 "\n", addr);
		exSID_delay(xs, cycles);
		return 0xFF;
	}
#endif

	// actual read will happen after read_pre_cycles. Delay for cycles - read_pre_cycles then account for the read
	xs->clkdrift += xs->xSconsts->read_offset_cycles;		// 2-cycle offset adjustement, see function documentation.
	xs->clkdrift += cycles;
	if (xs->clkdrift > xs->xSconsts->read_pre_cycles)
		xSdelay(xs, xs->clkdrift - xs->xSconsts->read_pre_cycles);

	xs->clkdrift -= xs->xSconsts->read_pre_cycles;	// read request is going to consume read_pre_cycles clock ticks

#ifdef	DEBUG
	if (xs->clkdrift > xs->xSconsts->mindel_cycles)
		xsdbg("Impossible drift adjustment! %" PRIdFAST32 " cycles", xs->clkdrift);
	else if (xs->clkdrift < 0) {
		xs->accdrift += xs->clkdrift;
		xsdbg("Late read request! %" PRIdFAST32 " cycles\n", xs->clkdrift);
	}
#endif

	// if we are still going to be early, delay actual read by up to max_adj ticks
	if (likely(xs->clkdrift >= 0)) {
		adj = xs->clkdrift % (xs->xSconsts->max_adj+1);	// see clkdwrite()
		addr = (unsigned char)(addr | (adj << 5));	// final delay encoded in top 3 bits of address
#ifdef	DEBUG
		xs->accdrift += (xs->clkdrift - adj);
#endif
		//xsdbg("drft: %d, adj: %d, addr: %.2hhx, data: %.2hhx\n", clkdrift, adj, (char)(addr | (adj << 5)), data);
	}

#ifdef	DEBUG
	xs->acccycle += cycles;
	xs->accioops++;
#endif

	// after read has completed, at least another read_post_cycles will have been spent
	xs->clkdrift -= xs->xSconsts->read_post_cycles;

	//xsdbg("delay: %d, clkdrift: %d\n", cycles, clkdrift);
	return _exSID_read(xs, addr, 1);
}
