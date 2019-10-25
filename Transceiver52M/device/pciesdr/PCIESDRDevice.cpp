#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "Threads.h"
#include "Utils.h"
#include "PCIESDRDevice.h"

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

#include <Logger.h>
#include <errno.h>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#define PSAMPLES_NUM  4096
#define SAMPLE_BUF_SZ (1 << 20) /* Size of Rx timestamp based Ring buffer, in bytes */

using namespace std;

PCIESDRDevice::PCIESDRDevice(size_t tx_sps, size_t rx_sps, InterfaceType iface, size_t chans, double lo_offset,
                             const std::vector<std::string>& tx_paths,
                             const std::vector<std::string>& rx_paths):
  RadioDevice(tx_sps, rx_sps, iface, chans, lo_offset, tx_paths, rx_paths)
{
  LOG(INFO) << "creating PCIESDR device...";

  this->txsps = tx_sps;
  this->rxsps = rx_sps;

  LOG(INFO) << "PCIESDR device txsps:" << txsps << " rxsps:" << rxsps << " GSMRATE:" << GSMRATE;
  dma_buffer_count = 10;
  dma_buffer_len = 2500;

  loopback = false;
  device = NULL;

  rx_buffers.resize(chans);
}

PCIESDRDevice::~PCIESDRDevice()
{
  LOG(INFO) << "Closing PCIESDR Device";
  if (device) {
    msdr_close(device);
  }
  device = NULL;

  for (size_t i = 0; i < rx_buffers.size(); i++) {
    delete rx_buffers[i];
  }
}

static int parse_config(const char* line, const char* argument, int default_value)
{
  const char* arg_found = strstr(line, argument);
  if (!arg_found) {
    return default_value;
  }
  const char* qe_pos = strchr(arg_found, '=');
  if (!qe_pos) {
    return default_value;
  }
  int res = strtol(qe_pos + 1, NULL, 10);
  if (res == 0 && errno) {
    return default_value;
  }

  return res;
}

int PCIESDRDevice::open(const std::string &args, int ref, bool swap_channels)
{
  LOG(INFO) << "Opening PCIESDR device '"  << args << "'..";
  int lb_param = parse_config(args.c_str(), "loopback", 0);
  started = false;
  char pciesdr_name[500];
  const char* lend = strchr(args.c_str(), ',');
  int len = (lend) ? (lend - args.c_str()) : sizeof(pciesdr_name) - 1;
  strncpy(pciesdr_name, args.c_str(), len);
  pciesdr_name[len] = 0;

  if (lb_param) {
    LOG(ALERT) << "PCIESDR LOOPBACK mode is not supported!";
    loopback = false;
  }
  LOG(INFO) << "pciesdr_name"  << pciesdr_name << "";
  device = msdr_open(pciesdr_name);
  if (device == NULL) {
    LOG(ERR) << "PCIESDR creating failed, device " << pciesdr_name << "";
    return -1;
  }

  msdr_set_default_start_params(device, &StartParams);
  StartParams.interface_type = SDR_INTERFACE_RF;
  StartParams.sync_source = SDR_SYNC_NONE;

  switch (ref) {
    case REF_INTERNAL:
      LOGC(DDEV, INFO) << "Setting Internal clock reference";
      StartParams.clock_source = SDR_CLOCK_INTERNAL;
      break;
    default:
      LOGC(DDEV, ALERT) << "Invalid reference type";
      goto out_close;
  }

  LOG(INFO) << "PCIESDR device txsps:" << tx_sps << " rxsps:" << rx_sps << " GSMRATE:" << GSMRATE << " GSMRATE*tx_sps:" << (int64_t) GSMRATE*tx_sps;
  StartParams.sample_rate_num[0] = (int64_t)1625000*tx_sps;
  StartParams.sample_rate_den[0] = 6;
  //StartParams.rx_sample_fmt;
  //StartParams.tx_sample_fmt;
  //StartParams.rx_sample_hw_fmt;
  //StartParams.tx_sample_hw_fmt;
  StartParams.rx_channel_count = 1;
  StartParams.tx_channel_count = 1;
  StartParams.rx_freq[0] = 1550e6;
  StartParams.tx_freq[0] = 1500e6;
  StartParams.rx_gain[0] = 60;
  StartParams.tx_gain[0] = 40;
  StartParams.rx_bandwidth[0] = 1.5e6;
  StartParams.tx_bandwidth[0] = 1.2e6;
  StartParams.rx_antenna[0] = SDR_RX_ANTENNA_RX;
  StartParams.rf_port_count = 1;
  StartParams.tx_port_channel_count[0] = 1;
  StartParams.rx_port_channel_count[0] = 1;

  actualSampleRate = (double)StartParams.sample_rate_num[0] / (double)StartParams.sample_rate_den[0];

  /* FIXME: estimate it properly */
  ts_offset = static_cast<TIMESTAMP>(8.9e-5 * GSMRATE * tx_sps); /* time * sample_rate */

  LOG(INFO) << "open PCIESDR device: "  << device << "sample_rate:" << actualSampleRate;

  /* Set up per-channel Rx timestamp based Ring buffers */
  for (size_t i = 0; i < rx_buffers.size(); i++) {
    rx_buffers[i] = new smpl_buf(SAMPLE_BUF_SZ / sizeof(uint32_t));
  }

  started = false;
  return NORMAL;

out_close:
  LOGC(DDEV, ALERT) << "Error in PCIeSDR open, closing";
  msdr_close(device);
  device = NULL;

  return -1;
}

bool PCIESDRDevice::start()
{
  SDRStats    stats;
  int         res;

  LOG(INFO) << "PCIESDRDevice::start";

  if (started) {
    LOG(INFO) << "~PCIESDRDevice already runn";
    return false;
  }
  LOG(INFO) << "starting PCIeSDR..., sample rate:" << actualSampleRate;
  res = msdr_start(device, &StartParams);
  if (res) {
    LOG(ERR) << "msdr_start failed:"<< res;
    return false;
  }
  res = msdr_get_stats(device, &stats);
  if (res != 0) {
    LOG(ERR) << "PCIESDRDevice::start: get_stats failed:" << res;
  } else {
    tx_underflow = stats.tx_underflow_count;
    rx_overflow  = stats.rx_overflow_count;
  }
  LOG(INFO) << "msdr_start ok:";
  started = true;

  return true;
}

bool PCIESDRDevice::stop()
{
  int         res;

  LOG(INFO) << "PCIESDRDevice::stop";

  if (started) {
    res = msdr_stop(device);
    if (res) {
      LOG(ERR) << "PCIESDR stop failed res: " << res;
    } else {
      LOG(INFO) << "PCIESDR stopped";
      started = false;
    }
  }

  return true;
}

double PCIESDRDevice::maxTxGain()
{
  return 60;
}

double PCIESDRDevice::minTxGain()
{
  return 0;
}

double PCIESDRDevice::maxRxGain()
{
  return 50;
}

double PCIESDRDevice::minRxGain()
{
  return 0;
}

double PCIESDRDevice::setTxGain(double dB, size_t chan)
{
  int         res;

  if (chan) {
    LOG(ALERT) << "Invalid channel " << chan;
    return 0.0;
  }
  LOG(INFO) << "Setting TX gain to " << dB << " dB. device:" << device << " chan:" << chan;

  res = msdr_set_tx_gain(device, chan, dB);
  if (res) {
    LOG(ERR) << "Error setting TX gain res: " << res;
  } else {
    StartParams.tx_gain[chan] = dB;
  }

  return StartParams.tx_gain[chan];
}

double PCIESDRDevice::setRxGain(double dB, size_t chan)
{
  int         res;

  if (chan) {
    LOG(ALERT) << "Invalid channel " << chan;
    return 0.0;
  }

  LOG(INFO) << "Setting RX gain to " << dB << " dB.";
  res = msdr_set_rx_gain(device, chan, dB);
  if (res) {
    LOG(ERR) << "Error setting RX gain res: " << res;
  } else {
    StartParams.rx_gain[chan] = dB;
  }

  return StartParams.rx_gain[chan];
}

int PCIESDRDevice::readSamples(std::vector<short *> &bufs, int len, bool *overrun,
                               TIMESTAMP timestamp, bool *underrun, unsigned *RSSI)
{
  int rc, num_smpls, expect_smpls;
  ssize_t            avail_smpls;
  TIMESTAMP          expect_timestamp;
  unsigned int       i;
  static sample_t    samples[PSAMPLES_NUM];
  static sample_t    *psamples;
  int64_t            timestamp_tmp;
#ifndef LIBSDR_HAS_MSDR_CONVERT
  float              powerScaling[] = {1, 1, 1, 1};
#endif

  if (!started) {
    return -1;
  }

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
      expect_timestamp = timestamp + avail_smpls;
      timestamp_tmp = 0;
      psamples = &samples[0];
      num_smpls = msdr_read(device, &timestamp_tmp, (void**)&psamples, len, i, 100);
      if (num_smpls < 0) {
        LOG(ALERT) << "PCIeSDR readSamples msdr_read failed num_smpls " << num_smpls << " device: " << device << " len: " << len
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
        LOGCHAN(i, DDEV, NOTICE) << "Unexpected recv buffer len: expect "
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
      LOGC(DDEV, ERROR) << rx_buffers[i]->str_code(rc);
      LOGC(DDEV, ERROR) << rx_buffers[i]->str_status(timestamp);
      return 0;
    }
  }

  return len;
}

int PCIESDRDevice::writeSamples(std::vector<short *> &bufs, int len,
                                bool *underrun, unsigned long long timestamp,
                                bool isControl)
{
  int                rc = 0;
  unsigned int       i;
  static sample_t    samples[PSAMPLES_NUM];
  static sample_t    *psamples;
  int64_t            hw_time;
  int64_t            timestamp_tmp;
  SDRStats           stats;

  if (!started) {
    return -1;
  }

  if (isControl) {
    LOGC(DDEV, ERROR) << "Control packets not supported";
    return 0;
  }

  if (bufs.size() != chans) {
    LOGC(DDEV, ERROR) << "Invalid channel combination " << bufs.size();
    return -1;
  }

  if (len > (int)(sizeof(samples) / sizeof(*samples))) {
    LOGC(DDEV, ERROR) << "Sample buffer:" << (sizeof(samples) / sizeof(*samples)) << " is smaller than len:" << len;
    return -1;
  }

  timestamp_tmp = timestamp + ts_offset; /* Shift Tx time by offset */

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
      LOGC(DDEV, ALERT) << "PCIeSDR writeSamples: Device send timed out rc:" << rc << " timestamp" << timestamp_tmp << " len:" << len << " hwtime:" << hw_time;
      LOGCHAN(i, DDEV, ERROR) << "PCIeSDR: Device Tx timed out (" << rc << " vs exp " << len << ").";
      return -1;
    }
    if (msdr_get_stats(device, &stats)) {
      LOGC(DDEV, ALERT) << "PCIeSDR: get_stats failed:" << rc;
    } else if (stats.tx_underflow_count > tx_underflow) {
      tx_underflow = stats.tx_underflow_count;
      LOGC(DDEV, ALERT) << "tx_underflow_count:" << stats.tx_underflow_count << " rx_overflow_count:" << stats.rx_overflow_count;
      *underrun = true;
    }

    if (timestamp_tmp - hw_time > 10000) {
      LOGC(DDEV, ALERT) << "PCIeSDR: tx diff more ts_tmp:" << timestamp_tmp << " ts:" << timestamp  << " hwts:" << hw_time;
    }

    if (hw_time > timestamp_tmp) {
      LOGC(DDEV, ALERT) << "PCIeSDR: tx underrun ts_tmp:" << timestamp_tmp << " ts:" << timestamp  << " hwts:" << hw_time;
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
  LOG(ALERT) << "Update Alignment ";

  return true;
}

bool PCIESDRDevice::setTxFreq(double wFreq, size_t chan)
{
  double actual = 0;

  LOG(INFO) << "PCIESDR setTxFreq";
  if (chan) {
    LOG(ALERT) << "Invalid channel " << chan;
    return false;
  }
  actual = StartParams.tx_freq[chan];
  StartParams.tx_freq[chan] = wFreq;
  LOG(INFO) << "set TX: " << wFreq << std::endl
            << "    actual freq: " << actual << std::endl;
  return true;
}

bool PCIESDRDevice::setRxFreq(double wFreq, size_t chan)
{
  double actual = 0;

  LOG(INFO) << "PCIESDR setRxFreq";

  if (chan) {
    LOG(ALERT) << "Invalid channel " << chan;
    return false;
  }
  actual = StartParams.rx_freq[chan];
  StartParams.rx_freq[chan] = wFreq;
  LOG(INFO) << "set RX: " << wFreq << std::endl
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
