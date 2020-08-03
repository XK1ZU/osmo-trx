/*
* Copyright 2018 sysmocom - s.f.m.c. GmbH
*
* SPDX-License-Identifier: AGPL-3.0+
*
	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU Affero General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU Affero General Public License for more details.

	You should have received a copy of the GNU Affero General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include <map>

#include "trx_vty.h"
#include "Logger.h"
#include "Threads.h"
#include "PCIESDRDevice.h"
#include "Utils.h"

#define LIBSDR_HAS_MSDR_CONVERT
extern "C" {
#include "libsdr.h"
}

extern "C" {
#include "../../arch/common/convert.h"
}

extern "C" {
#include "osmo_signal.h"
#include <osmocom/core/utils.h>
}

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

using namespace std;

#define PSAMPLES_NUM  4096

/* Size of Rx / Tx timestamp based Ring buffer, in bytes */
#define SAMPLE_BUF_SZ (1 << 20)

/* greatest common divisor */
static long gcd(long a, long b)
{
	if (a == 0)
		return b;
	else if (b == 0)
		return a;

	if (a < b)
		return gcd(a, b % a);
	else
		return gcd(b, a % b);
}

PCIESDRDevice::PCIESDRDevice(size_t tx_sps, size_t rx_sps, InterfaceType iface, size_t chan_num, double lo_offset,
		             const std::vector<std::string>& tx_paths,
		             const std::vector<std::string>& rx_paths):
	RadioDevice(tx_sps, rx_sps, iface, chan_num, lo_offset, tx_paths, rx_paths)
{
	LOGC(DDEV, INFO) << "creating PCIESDR device...";

	/* The parameter dma_buffer_len is significant for functionality of the Rx chain */
	dma_buffer_count = 10;
	dma_buffer_len = 1000;

	device = NULL;

	rx_buffers.resize(chans);

	/* Set up per-channel Rx timestamp based Ring buffers */
	for (size_t i = 0; i < rx_buffers.size(); i++)
		rx_buffers[i] = new smpl_buf(SAMPLE_BUF_SZ / sizeof(uint32_t));

}

PCIESDRDevice::~PCIESDRDevice()
{
	LOGC(DDEV, INFO) << "Closing PCIESDR device";
	if (device) {
		msdr_close(device);
		device = NULL;
	}

	for (size_t i = 0; i < rx_buffers.size(); i++)
		delete rx_buffers[i];
}

static int parse_config(const char* line, const char* argument, int default_value)
{
	const char* arg_found = strstr(line, argument);
	if (!arg_found)
		return default_value;

	const char* qe_pos = strchr(arg_found, '=');
	if (!qe_pos)
		return default_value;

	int res = strtol(qe_pos + 1, NULL, 10);
	if (res == 0 && errno)
		return default_value;

	return res;
}

int PCIESDRDevice::open(const std::string &args, int ref, bool swap_channels)
{
	int lb_param = parse_config(args.c_str(), "loopback", 0);
	char pciesdr_name[500];
	const char* lend = strchr(args.c_str(), ',');
	int len = (lend) ? (lend - args.c_str()) : sizeof(pciesdr_name) - 1;

	LOGC(DDEV, INFO) << "Opening PCIESDR device..";

	strncpy(pciesdr_name, args.c_str(), len);
	pciesdr_name[len] = 0;
	started = false;

	if (lb_param) {
		LOGC(DDEV, ERROR) << "PCIESDR LOOPBACK mode is not supported!";
	}
	LOGC(DDEV, INFO) << "pciesdr_name"  << pciesdr_name << "";
	device = msdr_open(pciesdr_name);
	if (device == NULL) {
		LOGC(DDEV, ERROR) << "PCIESDR creating failed, device " << pciesdr_name << "";
		return -1;
	}

	msdr_set_default_start_params(device, &StartParams);
	/* RF interface */
	StartParams.interface_type = SDR_INTERFACE_RF;
	/* no time synchronisation */
	StartParams.sync_source = SDR_SYNC_NONE;
	/* sync on internal PPS */
	//StartParams.sync_source = SDR_SYNC_INTERNAL;

	/* calculate sample rate using Euclidean algorithm */
	double rate = (double)GSMRATE*tx_sps;
	double integral = floor(rate);
	double frac = rate - integral;
	/* This is the accuracy */
	const long precision = 1000000000;
	long gcd_ = gcd(round(frac * precision), precision);
	long denominator = precision / gcd_;
	long numerator = round(frac * precision) / gcd_;
	StartParams.sample_rate_num[0] = (int64_t)(integral * denominator + numerator);
	StartParams.sample_rate_den[0] = (int64_t)denominator;
	actualSampleRate = (double)StartParams.sample_rate_num[0] / (double)StartParams.sample_rate_den[0];
	StartParams.rx_bandwidth[0] = actualSampleRate * 0.75;
	StartParams.tx_bandwidth[0] = actualSampleRate * 0.75;
	LOGC(DDEV, INFO) << "PCIESDR device txsps:" << tx_sps << " rxsps:" << rx_sps
	                 << " GSMRATE * tx_sps:" << (double)GSMRATE * tx_sps;
	LOGC(DDEV, INFO) << "PCIESDR sample_rate_num:" << StartParams.sample_rate_num[0]
	                 << " sample_rate_den:" << StartParams.sample_rate_den[0] << " BW:" << StartParams.rx_bandwidth[0];

	switch (ref) {
	case REF_INTERNAL:
		LOGC(DDEV, INFO) << "Setting Internal clock reference";
		/* internal clock, using PPS to correct it */
		StartParams.clock_source = SDR_CLOCK_INTERNAL;
		break;
	default:
		LOGC(DDEV, ERROR) << "Invalid reference type";
		goto out_close;
	}
	/* complex float32 */
	StartParams.rx_sample_fmt = SDR_SAMPLE_FMT_CF32;
	StartParams.tx_sample_fmt = SDR_SAMPLE_FMT_CF32;
	/* choose best format fitting the bandwidth */
	StartParams.rx_sample_hw_fmt = SDR_SAMPLE_HW_FMT_AUTO;
	StartParams.tx_sample_hw_fmt = SDR_SAMPLE_HW_FMT_AUTO;
	StartParams.rx_channel_count = 1;
	StartParams.tx_channel_count = 1;
	StartParams.rx_freq[0] = 1550e6;
	StartParams.tx_freq[0] = 1500e6;
	StartParams.rx_gain[0] = 60;
	StartParams.tx_gain[0] = 40;
	StartParams.rx_antenna[0] = SDR_RX_ANTENNA_RX;
	StartParams.rf_port_count = 1;
	StartParams.tx_port_channel_count[0] = 1;
	StartParams.rx_port_channel_count[0] = 1;
	/* if != 0, set a custom DMA buffer configuration. Otherwise the default is 150 buffers per 10 ms */
	StartParams.dma_buffer_count = dma_buffer_count;
	/* in samples */
	StartParams.dma_buffer_len = dma_buffer_len;

	/* FIXME: estimate it properly */
	/* PCIe radio should have this close to zero */
	/* The MS can connect if the value is between -30 and +5 */
	ts_offset = -16;

	started = false;
	return NORMAL;

out_close:
	LOGC(DDEV, FATAL) << "Error in PCIESDR open, closing";
	msdr_close(device);
	device = NULL;

	return -1;
}

bool PCIESDRDevice::start()
{
	SDRStats stats;
	int res;

	LOGC(DDEV, INFO) << "starting PCIESDR...";

	if (started) {
		LOGC(DDEV, ERROR) << "Device already started";
		return false;
	}
	LOGC(DDEV, INFO) << "starting PCIESDR..., sample rate:" << actualSampleRate;
	res = msdr_start(device, &StartParams);
	if (res) {
		LOGC(DDEV, ERROR) << "msdr_start failed:"<< res;
		return false;
	}
	res = msdr_get_stats(device, &stats);
	if (res != 0) {
		LOGC(DDEV, ERROR) << "PCIESDRDevice start: get_stats failed:" << res;
	} else {
		tx_underflow = stats.tx_underflow_count;
		rx_overflow  = stats.rx_overflow_count;
	}

	flush_recv();

	started = true;
	return true;
}

bool PCIESDRDevice::stop()
{
	int res;

	LOGC(DDEV, INFO) << "PCIESDRDevice stop";

	if (started) {
		res = msdr_stop(device);
		if (res) {
			LOGC(DDEV, ERROR) << "PCIESDR stop failed res: " << res;
		} else {
			LOGC(DDEV, INFO) << "PCIESDR stopped";
			started = false;
		}
	}

	return true;
}

double PCIESDRDevice::maxTxGain()
{
	return 90;
}

double PCIESDRDevice::maxRxGain()
{
	return 50;
}

double PCIESDRDevice::minRxGain()
{
	return 0;
}

double PCIESDRDevice::getTxGain(size_t chan)
{
	if (chan) {
		LOGC(DDEV, ERROR) << "Invalid channel " << chan;
		return 0.0;
	}

	return msdr_get_tx_gain(device, chan);
}

double PCIESDRDevice::setTxGain(double dB, size_t chan)
{
	int res;

	if (chan) {
		LOGC(DDEV, ERROR) << "Invalid channel " << chan;
		return 0.0;
	}

	LOGC(DDEV, INFO) << "Setting TX gain to " << dB << " dB. device:" << device << " chan:" << chan;
	res = msdr_set_tx_gain(device, chan, dB);
	if (res) {
		LOGC(DDEV, INFO) << "Error setting TX gain res: " << res;
	} else {
		StartParams.tx_gain[chan] = dB;
	}

	return StartParams.tx_gain[chan];
}

double PCIESDRDevice::setPowerAttenuation(int atten, size_t chan) {
	double rfGain;

	rfGain = setTxGain(maxTxGain() - atten, chan);
	return maxTxGain() - rfGain;
}

double PCIESDRDevice::getPowerAttenuation(size_t chan) {
	return maxTxGain() - getTxGain(chan);
}

double PCIESDRDevice::setRxGain(double dB, size_t chan)
{
	int res;

	if (chan) {
		LOGC(DDEV, ERROR) << "Invalid channel " << chan;
		return 0.0;
	}

	LOGC(DDEV, INFO) << "Setting RX gain to " << dB << " dB.";
	res = msdr_set_rx_gain(device, chan, dB);
	if (res) {
		LOGC(DDEV, ERROR) << "Error setting RX gain res: " << res;
	} else {
		StartParams.rx_gain[chan] = dB;
	}

	return StartParams.rx_gain[chan];
}

int PCIESDRDevice::getNominalTxPower(size_t chan)
{
	/* TODO: return value based on some experimentally generated table depending on
	* band/arfcn, which is known here thanks to TXTUNE
	*/
	return 0;
}

bool PCIESDRDevice::flush_recv()
{
	unsigned int chan = 0;
	static sample_t samples[PSAMPLES_NUM];
	static sample_t *psamples;
	int64_t timestamp_tmp;
	int expect_smpls = sizeof(samples) / sizeof(samples[0]);
	int rc;

	LOGC(DDEV, INFO) << "PCIESDRDevice flush";

	psamples = &samples[0];

	while ((rc = msdr_read(device, &timestamp_tmp, (void**)&psamples, expect_smpls, chan, 100)) > 1) {
		if (rc < (int)expect_smpls)
			break;
	}
	if (rc < 0)
		return false;

	ts_initial = (TIMESTAMP)timestamp_tmp + rc;

	LOGC(DDEV, INFO) << "Initial timestamp " << ts_initial << std::endl;

	return true;
}

/* NOTE: Assumes sequential reads */
int PCIESDRDevice::readSamples(std::vector <short *> &bufs, int len, bool *overrun,
		               TIMESTAMP timestamp, bool *underrun)
{
	int rc, num_smpls, expect_smpls;
	ssize_t avail_smpls;
	TIMESTAMP expect_timestamp;
	unsigned int i;
	static sample_t samples[PSAMPLES_NUM];
	static sample_t *psamples;
	int64_t timestamp_tmp;
#ifndef LIBSDR_HAS_MSDR_CONVERT
	float powerScaling[] = {1, 1, 1, 1};
#endif

	if (!started)
		return -1;

	if (bufs.size() != chans) {
		LOGC(DDEV, ERROR) << "Invalid channel combination " << bufs.size();
		return -1;
	}

	if (len > (int)(sizeof(samples) / sizeof(*samples))) {
		LOGC(DDEV, ERROR) << "Sample buffer:" << (sizeof(samples) / sizeof(*samples)) << " is smaller than len:" << len;
		return -1;
	}

	*overrun = false;
	*underrun = false;

	/* Check that timestamp is valid */
	rc = rx_buffers[0]->avail_smpls(timestamp);
	if (rc < 0) {
		LOGC(DDEV, ERROR) << "rc < 0";
		LOGC(DDEV, ERROR) << rx_buffers[0]->str_code(rc);
		LOGC(DDEV, ERROR) << rx_buffers[0]->str_status(timestamp);
		return 0;
	}

	for (i = 0; i < chans; i++) {
		/* Receive samples from HW until we have enough */
		while ((avail_smpls = rx_buffers[i]->avail_smpls(timestamp)) < len) {
			expect_smpls = len - avail_smpls;
			expect_smpls = expect_smpls > (int)dma_buffer_len ? expect_smpls : (int)dma_buffer_len;
			expect_timestamp = timestamp + avail_smpls;
			timestamp_tmp = 0;
			psamples = &samples[0];
			num_smpls = msdr_read(device, &timestamp_tmp, (void**)&psamples, expect_smpls, i, 100);
			if (num_smpls < 0) {
				LOGC(DDEV, ERROR) << "PCIESDR readSamples msdr_read failed num_smpls " << num_smpls << " device: " << device
					   << " expect_smpls: " << expect_smpls
					   << ", expTs:" << expect_timestamp << " got " << timestamp_tmp;
				LOGCHAN(i, DDEV, ERROR) << "Device receive timed out (" << rc << " vs exp " << len << ").";
				return -1;
			}

			LOGCHAN(i, DDEV, DEBUG) "Received timestamp = " << (TIMESTAMP)timestamp_tmp << " (" << num_smpls << ")";

#ifdef LIBSDR_HAS_MSDR_CONVERT
			msdr_convert_cf32_to_ci16(bufs[i], (float *)psamples, num_smpls);
#else
			convert_float_short(bufs[i], (float *)psamples, powerScaling[0], num_smpls * 2);
#endif

			if (expect_smpls != num_smpls) {
				LOGCHAN(i, DDEV, DEBUG) << "Unexpected recv buffer len: expect "
					                << expect_smpls << " got " << num_smpls
					                << ", diff=" << expect_smpls - num_smpls
					                << ", expTs:" << expect_timestamp << " got " << timestamp_tmp;
			}

			if (expect_timestamp != (TIMESTAMP)timestamp_tmp) {
				LOGCHAN(i, DDEV, ERROR) << "Unexpected recv buffer timestamp: expect "
					                << expect_timestamp << " got " << timestamp_tmp
					                << ", diff=" << timestamp_tmp - expect_timestamp;
			}
			rc = rx_buffers[i]->write(bufs[i], num_smpls, (TIMESTAMP)timestamp_tmp);
			if (rc < 0) {
				if (rc != smpl_buf::ERROR_OVERFLOW) {
					return 0;
				}
			}
		}
	}

	/* We have enough samples */
	for (size_t i = 0; i < rx_buffers.size(); i++) {
		rc = rx_buffers[i]->read(bufs[i], len, timestamp);
		if ((rc < 0) || (rc != len)) {
			LOGCHAN(i, DDEV, ERROR) << rx_buffers[i]->str_code(rc) << ". "
				                << rx_buffers[i]->str_status(timestamp)
				                << ", (len=" << len << ")";
			return 0;
		}
	}

	return len;
}

int PCIESDRDevice::writeSamples(std::vector<short *> &bufs, int len,
                                bool *underrun, unsigned long long timestamp)
{
	int rc = 0;
	unsigned int i;
	static sample_t samples[PSAMPLES_NUM];
	static sample_t *psamples;
	int64_t hw_time;
	int64_t timestamp_tmp;
	SDRStats stats;

	if (!started)
		return -1;

	if (bufs.size() != chans) {
		LOGC(DDEV, ERROR) << "Invalid channel combination " << bufs.size();
		return -1;
	}

	if (len > (int)(sizeof(samples) / sizeof(*samples))) {
		LOGC(DDEV, ERROR) << "Sample buffer:" << (sizeof(samples) / sizeof(*samples)) << " is smaller than len:" << len;
		return -1;
	}

	timestamp_tmp = timestamp - ts_offset; /* Shift Tx time by offset */

	*underrun = false;
	i = 0;
	for (i = 0; i < chans; i++) {
		LOGCHAN(i, DDEV, DEBUG) << "send buffer of len " << len << " timestamp " << std::hex << timestamp_tmp;
		psamples = &samples[0];

#ifdef LIBSDR_HAS_MSDR_CONVERT
		msdr_convert_ci16_to_cf32((float*)psamples, bufs[i], len);
#else
		convert_short_float((float*)psamples, bufs[i], len * 2);
#endif
		rc = msdr_write(device, timestamp_tmp, (const void**)&psamples, len, i, &hw_time);
		if (rc != len) {
			LOGC(DDEV, ALERT) << "PCIESDR writeSamples: Device send timed out rc:" << rc << " timestamp" << timestamp_tmp << " len:" << len << " hwtime:" << hw_time;
			LOGCHAN(i, DDEV, ERROR) << "PCIESDR: Device Tx timed out (" << rc << " vs exp " << len << ").";
			return -1;
		}
		if (msdr_get_stats(device, &stats)) {
			LOGC(DDEV, ALERT) << "PCIESDR: get_stats failed:" << rc;
		} else if (stats.tx_underflow_count > tx_underflow) {
			tx_underflow = stats.tx_underflow_count;
			LOGC(DDEV, ALERT) << "tx_underflow_count:" << stats.tx_underflow_count << " rx_overflow_count:" << stats.rx_overflow_count;
			*underrun = true;
		}

		if (timestamp_tmp - hw_time > (int64_t)actualSampleRate / 10)
			LOGC(DDEV, ALERT) << "PCIESDR: tx diff more ts_tmp:" << timestamp_tmp << " ts:" << timestamp  << " hwts:" << hw_time;

		if (hw_time > timestamp_tmp) {
			LOGC(DDEV, ALERT) << "PCIESDR: tx underrun ts_tmp:" << timestamp_tmp << " ts:" << timestamp  << " hwts:" << hw_time;
			*underrun = true;
		}
	}

	return rc;
}

bool PCIESDRDevice::setRxAntenna(const std::string & ant, size_t chan)
{
	return true;
}

std::string PCIESDRDevice::getRxAntenna(size_t chan)
{
	return "";
}

bool PCIESDRDevice::setTxAntenna(const std::string & ant, size_t chan)
{
	return true;
}

std::string PCIESDRDevice::getTxAntenna(size_t chan )
{
	return "";
}

bool PCIESDRDevice::requiresRadioAlign()
{
	return false;
}

GSM::Time PCIESDRDevice::minLatency()
{
	return GSM::Time(6,7);
}

bool PCIESDRDevice::updateAlignment(TIMESTAMP timestamp)
{
	LOGC(DDEV, INFO) << "Update Alignment ";

	return true;
}

bool PCIESDRDevice::setTxFreq(double wFreq, size_t chan)
{
	double actual = 0;

	LOGCHAN(chan, DDEV, NOTICE) << "PCIESDR setTxFreq";
	if (chan) {
		LOGC(DDEV, ERROR) << "Invalid channel " << chan;
		return false;
	}
	actual = StartParams.tx_freq[chan];
	StartParams.tx_freq[chan] = wFreq;
	LOGC(DDEV, INFO) << "set TX: " << wFreq << std::endl
			 << "    actual freq: " << actual << std::endl;

	return true;
}

bool PCIESDRDevice::setRxFreq(double wFreq, size_t chan)
{
	double actual = 0;

	LOGCHAN(chan, DDEV, NOTICE) << "PCIESDR setRxFreq";

	if (chan) {
		LOGC(DDEV, ERROR) << "Invalid channel " << chan;
		return false;
	}
	actual = StartParams.rx_freq[chan];
	StartParams.rx_freq[chan] = wFreq;
	LOGC(DDEV, INFO) << "set RX: " << wFreq << std::endl
			 << "    actual freq: " << actual << std::endl;

	return true;
}

RadioDevice *RadioDevice::make(size_t tx_sps, size_t rx_sps,
			       InterfaceType iface, size_t chans, double lo_offset,
			       const std::vector < std::string > &tx_paths,
			       const std::vector < std::string > &rx_paths)
{
	if (tx_sps != rx_sps) {
		LOGC(DDEV, ERROR) << "PCIESDR Requires tx_sps == rx_sps";
		return NULL;
	}
	if (lo_offset != 0.0) {
		LOGC(DDEV, ERROR) << "PCIESDR doesn't support lo_offset";
		return NULL;
	}
	return new PCIESDRDevice(tx_sps, rx_sps, iface, chans, lo_offset, tx_paths, rx_paths);
}
