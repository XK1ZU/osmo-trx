#ifndef _PCIESDR_DEVICE_H_
#define _PCIESDR_DEVICE_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "common/radioDevice.h"

#include <stdint.h>
#include <sys/time.h>
#include <string>
#include <iostream>
#include "Threads.h"
extern "C" {
#include "libsdr.h"
}

class PCIESDRDevice:public RadioDevice {
private:
  int txsps;
  int rxsps;
  unsigned int dma_buffer_count;
  unsigned int dma_buffer_len;
  double actualTXSampleRate;         ///< the actual sampling rate
  double actualRXSampleRate;         ///< the actual sampling rate
  unsigned long long samplesRead;    ///< number of samples read from PCIESDR
  unsigned long long samplesWritten; ///< number of samples sent to PCIESDR
  bool started;                      ///< flag indicates PCIESDR has started
  TIMESTAMP timeStart;
  TIMESTAMP timeLastHW;
  TIMESTAMP timeRx;
  std::vector<double> tx_gains, rx_gains;
  double rxGain;
  bool loopback;
  int64_t tx_underflow;
  int64_t rx_overflow;
  MultiSDRState* device;
  SDRStartParams StartParams;
  typedef struct {
    float re;
    float im;
  } sample_t;
  sample_t *tx_samples;
  unsigned int tx_samples_index;
  sample_t *rx_samples;

public:
    /** Object constructor */
  PCIESDRDevice(size_t tx_sps, size_t rx_sps, InterfaceType iface, size_t chans, double lo_offset,
                const std::vector<std::string>& tx_paths,
                const std::vector<std::string>& rx_paths);
  ~PCIESDRDevice();
  /** Instantiate the PCIESDR */
  int open(const std::string &args, int ref, bool swap_channels);
  /** Start the PCIESDR */
  bool start();
  /** Stop the PCIESDR */
  bool stop();
  /** Set priority not supported */
  //void setPriority(float prio = 0.5) { }
  enum TxWindowType getWindowType() {
    return TX_WINDOW_FIXED;
  }
  /**
  Read samples from the PCIESDR.
  @param buf preallocated buf to contain read result
  @param len number of samples desired
  @param overrun Set if read buffer has been overrun, e.g. data not being read fast enough
  @param timestamp The timestamp of the first samples to be read
  @param underrun Set if PCIESDR does not have data to transmit, e.g. data not being sent fast enough
  @param RSSI The received signal strength of the read result
  @return The number of samples actually read
  */
  int readSamples(std::vector<short *> &buf, int len, bool *overrun,
                  TIMESTAMP timestamp = 0xffffffff, bool *underrun = NULL,
                  unsigned *RSSI = NULL);
  /**
  Write samples to the PCIESDR.
  @param buf Contains the data to be written.
  @param len number of samples to write.
  @param underrun Set if PCIESDR does not have data to transmit, e.g. data not being sent fast enough
  @param timestamp The timestamp of the first sample of the data buffer.
  @param isControl Set if data is a control packet, e.g. a ping command
  @return The number of samples actually written
  */
  int writeSamples(std::vector<short *> &bufs, int len, bool *underrun,
                   TIMESTAMP timestamp = 0xffffffff, bool isControl = false);
  /** Update the alignment between the read and write timestamps */
  bool updateAlignment(TIMESTAMP timestamp);
  /** Set the transmitter frequency */
  bool setTxFreq(double wFreq, size_t chan = 0);
  /** Set the receiver frequency */
  bool setRxFreq(double wFreq, size_t chan = 0);
  /** Returns the starting write Timestamp*/
  TIMESTAMP initialWriteTimestamp(void);
  /** Returns the starting read Timestamp*/
  TIMESTAMP initialReadTimestamp(void) { return 20000;}
  /** returns the full-scale transmit amplitude **/
  double fullScaleInputValue() {return (double) 32767*0.7;}
  /** returns the full-scale receive amplitude **/
  double fullScaleOutputValue() {return (double) 32767;}
  /** sets the receive chan gain, returns the gain setting **/
  double setRxGain(double dB, size_t chan = 0);
  /** get the current receive gain */
  double getRxGain(size_t chan = 0) { return rxGain; }
  /** return maximum Rx Gain **/
  double maxRxGain(void);
  /** return minimum Rx Gain **/
  double minRxGain(void);
  /** sets the transmit chan gain, returns the gain setting **/
  double setTxGain(double dB, size_t chan = 0);
  /** get transmit gain */
  double getTxGain(size_t chan = 0) {
    return tx_gains[chan];
  }
  /** return maximum Tx Gain **/
  double maxTxGain(void);
  /** return minimum Rx Gain **/
  double minTxGain(void);
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
  inline double getTxFreq(size_t chan = 0) { return 0; }
  inline double getRxFreq(size_t chan = 0) { return 0; }
  inline double getSampleRate() { return actualTXSampleRate; }
  inline double numberRead() { return samplesRead; }
  inline double numberWritten() { return samplesWritten; }
};
#endif // _PCIESDR_DEVICE_H_
