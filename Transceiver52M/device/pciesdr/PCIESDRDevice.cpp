#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "Threads.h"
#include "PCIESDRDevice.h"

#define LIBSDR_HAS_MSDR_CONVERT
extern "C" {
#include "libsdr.h"
}

extern "C" {
#include "../../arch/common/convert.h"
}

#include <Logger.h>
#include <errno.h>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#define SAMPLE_RATE GSMRATE
#define PSAMPLES_NUM 4096

using namespace std;

const double defaultRXBandwidth = 1.5e6;
const double defaultTXBandwidth = 1.5e6;

PCIESDRDevice::PCIESDRDevice(size_t tx_sps, size_t rx_sps, InterfaceType iface, size_t chans, double lo_offset,
                             const std::vector<std::string>& tx_paths,
                             const std::vector<std::string>& rx_paths):
  RadioDevice(tx_sps, rx_sps, iface, chans, lo_offset, tx_paths, rx_paths)
{
  LOG(INFO) << "creating PCIESDR device...";

  this->txsps = txsps;
  this->rxsps = rxsps;

  LOG(INFO) << "PCIESDR device txsps:" << txsps << " rxsps:" << rxsps << " GSMRATE:" << GSMRATE;
  rxGain = 0;
  dma_buffer_count = 10;
  dma_buffer_len = 2500;

  loopback = false;
  device = NULL;
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
  LOG(INFO) << "opening PCIESDR device '"  << args << "'..";
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
    LOG(ALERT) << "PCIESDR creating failed, device " << pciesdr_name << "";
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
  StartParams.sample_rate_num[0] = (int64_t)GSMRATE*tx_sps;
  StartParams.sample_rate_den[0] = 1;
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
  StartParams.dma_buffer_count = dma_buffer_count;
  StartParams.dma_buffer_len = dma_buffer_len;

  LOG(INFO) << "open PCIESDR device: "  << device << "sample_rate:" << StartParams.sample_rate_num[0];
  return NORMAL;

out_close:
  LOGC(DDEV, ALERT) << "Error in PCIeSDR open, closing";
  msdr_close(device);
  device = NULL;

  return -1;
}

PCIESDRDevice::~PCIESDRDevice()
{
  LOG(INFO) << "~PCIESDRDevice";
  if (device) {
    msdr_close(device);
  }
}

bool PCIESDRDevice::start()
{
  SDRStats    stats;
  int         res;

  LOG(INFO) << "PCIESDRDevice::start";
  LOG(INFO) << "starting PCIeSDR..., sample rate:" << StartParams.sample_rate_num[0];
  if (started) {
    LOG(INFO) << "~PCIESDRDevice already runn";
    return false;
  }
  res = msdr_start(device, &StartParams);
  if (res) {
    LOG(INFO) << "msdr_start failed:"<< res;
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
      LOG(ALERT) << "PCIESDR stop failed res: " << res;
    } else {
      LOG(INFO) << "PCIESDR stopped";
      started = false;
    }
  }

  return true;
}

TIMESTAMP PCIESDRDevice::initialWriteTimestamp()
{
  LOG(INFO) << "PCIESDRDevice::initialWriteTimestamp";

  return initialReadTimestamp();
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
  double actual = StartParams.tx_gain[chan];
  LOG(NOTICE) << "Setting TX gain to " << dB << " dB. device:" << device << " chan:" << chan;

  res = msdr_set_tx_gain(device, chan, dB);
  if (res) {
    LOG(ERR) << "Error setting TX gain res: " << res;
  } else {
    actual = dB;
  }

  return actual;
}

double PCIESDRDevice::setRxGain(double dB, size_t chan)
{
  int         res;

  if (chan) {
    LOG(ALERT) << "Invalid channel " << chan;
    return 0.0;
  }

  double actual = StartParams.rx_gain[chan];
  LOG(NOTICE) << "Setting RX gain to " << dB << " dB.";
  res = msdr_set_rx_gain(device, chan, dB);
  if (res) {
    LOG(ERR) << "Error setting RX gain res: " << res;
  } else {
    actual = dB;
  }

  return actual;
}

int PCIESDRDevice::readSamples(std::vector<short *> &bufs, int len, bool *overrun,
                               TIMESTAMP timestamp, bool *underrun, unsigned *RSSI)
{
  static sample_t    samples[4096];
  static sample_t    *psamples;
  int64_t            timestamp_tmp;
  int                res = 0;
  int                total = 0;
#ifndef LIBSDR_HAS_MSDR_CONVERT
  float              powerScaling[] = {1, 1, 1, 1};
#endif

  if (!started) {
    return -1;
  }

  if (len > (int)sizeof(samples)) {
    return -1;
  }

  timestamp_tmp = timestamp;
  psamples = &samples[0];
  res = msdr_read(device, &timestamp_tmp, (void**)&psamples, len, 0, 100);
  if (res < 0) {
    LOG(ALERT) << "PCIeSDR readSamples msdr_read failed res " << res << " device: " << device << " len: " << len << " req TS " << timestamp_tmp;
    return -1;
  }
  total += res;
  if (total != len) {
    LOG(ALERT) << "PCIeSDR readSamples msdr_read failed res " << total << " device: " << device << " len: " << len << " TSsmp: " << timestamp_tmp << " TS:" << timestamp;
  }
#ifdef LIBSDR_HAS_MSDR_CONVERT
  msdr_convert_cf32_to_ci16(bufs[0], (float *)psamples, len);
#else
  convert_float_short(bufs[0], (float *)psamples, powerScaling[0], len * 2);
#endif

  return total;
}

int PCIESDRDevice::writeSamples(std::vector<short *> &bufs, int len,
                                bool *underrun, unsigned long long timestamp,
                                bool isControl)
{
  static sample_t    samples[PSAMPLES_NUM];
  static sample_t    *psamples;
  int                res;
  int64_t            hw_time;
  int64_t            timestamp_tmp;
  SDRStats           stats;

  if (!started) {
    return 0;
  }

  if (len > (int)sizeof(samples)) {
    return 0;
  }

  psamples = &samples[0];

#ifdef LIBSDR_HAS_MSDR_CONVERT
  msdr_convert_ci16_to_cf32((float*)psamples, bufs[0], len);
#else
  convert_short_float((float*)psamples, bufs[0], len * 2);
#endif
  timestamp_tmp = timestamp + timeStart;
  res = msdr_write(device, timestamp_tmp, (const void**)&psamples, len, 0, &hw_time);
  if (res != len) {
    LOGC(DDEV, ALERT) << "PCIeSDR writeSamples: Device send timed out res:" << res << " timestamp" << timestamp_tmp << " len:" << len << " hwtime:" << hw_time;
    return 0;
  }
  res = msdr_get_stats(device, &stats);
  if (res != 0) {
    LOGC(DDEV, ALERT) << "PCIeSDR: get_stats failed:" << res;
  } else if (stats.tx_underflow_count > tx_underflow) {
    tx_underflow = stats.tx_underflow_count;
    LOGC(DDEV, ALERT) << "tx_underflow_count:" << stats.tx_underflow_count << " rx_overflow_count:" << stats.rx_overflow_count;
    *underrun = 1;
  }

  if (timestamp_tmp - hw_time > 10000) {
    LOGC(DDEV, ALERT) << "PCIeSDR: tx diff more ts:" << timestamp_tmp << " hwts:" << hw_time;
  }

  if (hw_time > timestamp_tmp) {
    LOGC(DDEV, ALERT) << "PCIeSDR: tx underrun ts:" << timestamp_tmp << " hwts:" << hw_time;
    *underrun = 1;
  }

  return len;
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
  int64_t     hw_time;

  LOG(ALERT) << "Update Aligment " << timestamp;
  msdr_write(device, 0, (const void**)NULL, 0, 0, &hw_time);
  timeStart = hw_time;

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
