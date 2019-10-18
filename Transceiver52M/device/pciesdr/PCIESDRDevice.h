/*
* Copyright 2018 sysmocom - s.f.m.c. GmbH
*
* SPDX-License-Identifier: AGPL-3.0+
*
* This software is distributed under multiple licenses; see the COPYING file in
* the main directory for licensing information for this specific distribution.
*
* This use of this software may be subject to additional restrictions.
* See the LEGAL file in the main directory for details.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

*/

#ifndef _PCIESDR_DEVICE_H_
#define _PCIESDR_DEVICE_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "radioDevice.h"
#include "smpl_buf.h"

#include <sys/time.h>
#include <math.h>
#include <limits.h>
#include <string>
#include <iostream>
extern "C" {
#include "libsdr.h"
}

#define PCIESDR_TX_AMPL  0.707

/** A class to handle a PCIESDR supported device */
class PCIESDRDevice:public RadioDevice {

private:
	MultiSDRState* device;
	SDRStartParams StartParams;
	typedef struct {
		float re;
		float im;
	} sample_t;
	unsigned int dma_buffer_count;
	unsigned int dma_buffer_len;

	std::vector<smpl_buf *> rx_buffers;

	double actualSampleRate;	///< the actual sampling rate

	bool started;			///< flag indicates device has started
	bool skipRx;			///< set if device is transmit-only.

	TIMESTAMP ts_initial, ts_offset;

	std::vector<double> tx_gains, rx_gains;

	bool flush_recv();

	int64_t tx_underflow;
	int64_t rx_overflow;

	/** sets the transmit chan gain, returns the gain setting **/
	double setTxGain(double dB, size_t chan = 0);

	/** get transmit gain */
	double getTxGain(size_t chan = 0);

	/** return maximum Tx Gain **/
	double maxTxGain(void);

public:

	/** Object constructor */
	PCIESDRDevice(size_t tx_sps, size_t rx_sps, InterfaceType iface, size_t chan_num, double lo_offset,
		const std::vector<std::string>& tx_paths,
		const std::vector<std::string>& rx_paths);
	~PCIESDRDevice();

	/** Instantiate the PCIESDR */
	int open(const std::string &args, int ref, bool swap_channels);

	/** Start the PCIESDR */
	bool start();

	/** Stop the PCIESDR */
	bool stop();

	enum TxWindowType getWindowType() {
		return TX_WINDOW_LMS1;
	}

	/**
	Read samples from the PCIESDR.
	@param buf preallocated buf to contain read result
	@param len number of samples desired
	@param overrun Set if read buffer has been overrun, e.g. data not being read fast enough
	@param timestamp The timestamp of the first samples to be read
	@param underrun Set if PCIESDR does not have data to transmit, e.g. data not being sent fast enough
	@return The number of samples actually read
	*/
	int readSamples(std::vector <short *> &buf, int len, bool *overrun,
			TIMESTAMP timestamp = 0xffffffff, bool *underrun = NULL);

	/**
	Write samples to the PCIESDR.
	@param buf Contains the data to be written.
	@param len number of samples to write.
	@param underrun Set if PCIESDR does not have data to transmit, e.g. data not being sent fast enough
	@param timestamp The timestamp of the first sample of the data buffer.
	@return The number of samples actually written
	*/
	int writeSamples(std::vector <short *> &bufs, int len, bool *underrun,
			 TIMESTAMP timestamp = 0xffffffff);

	/** Update the alignment between the read and write timestamps */
	bool updateAlignment(TIMESTAMP timestamp);

	/** Set the transmitter frequency */
	bool setTxFreq(double wFreq, size_t chan = 0);

	/** Set the receiver frequency */
	bool setRxFreq(double wFreq, size_t chan = 0);

	/** Returns the starting write Timestamp*/
	TIMESTAMP initialWriteTimestamp(void) {
		return ts_initial;
	}

	/** Returns the starting read Timestamp*/
	TIMESTAMP initialReadTimestamp(void) {
		return ts_initial;
	}

	/** returns the full-scale transmit amplitude **/
	double fullScaleInputValue() {
		return (double) SHRT_MAX * PCIESDR_TX_AMPL;
	}

	/** returns the full-scale receive amplitude **/
	double fullScaleOutputValue() {
		return (double) SHRT_MAX;
	}

	/** sets the receive chan gain, returns the gain setting **/
	double setRxGain(double dB, size_t chan = 0);

	/** get the current receive gain */
	double getRxGain(size_t chan = 0) {
		return rx_gains[chan];
	}

	/** return maximum Rx Gain **/
	double maxRxGain(void);

	/** return minimum Rx Gain **/
	double minRxGain(void);

	double setPowerAttenuation(int atten, size_t chan);
	double getPowerAttenuation(size_t chan = 0);

	int getNominalTxPower(size_t chan = 0);

	/** sets the RX path to use, returns true if successful and false otherwise */
	bool setRxAntenna(const std::string & ant, size_t chan = 0);

	/** return the used RX path */
	std::string getRxAntenna(size_t chan = 0);

	/** sets the RX path to use, returns true if successful and false otherwise */
	bool setTxAntenna(const std::string & ant, size_t chan = 0);

	/** return the used RX path */
	std::string getTxAntenna(size_t chan = 0);

	/** return whether user drives synchronization of Tx/Rx of USRP */
	bool requiresRadioAlign();

	/** return whether user drives synchronization of Tx/Rx of USRP */
	virtual GSM::Time minLatency();

	/** Return internal status values */
	inline double getTxFreq(size_t chan = 0) {
		return 0;
	}
	inline double getRxFreq(size_t chan = 0) {
		return 0;
	}
	inline double getSampleRate() {
		return actualSampleRate;
	}
};

#endif // _PCIESDR_DEVICE_H_
