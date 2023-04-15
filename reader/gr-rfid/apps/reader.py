# Original code from Nikos Kargas.
# Adapted to communicate with wisp 5.1 tags by: Laura Arjona, 2018
# laura.arjona@deusto.es
# Further adapted to realize RAScatter by: Kai Huang, 2022
# k.huang[AT]pitt.edu
# -----------------------------------------------------------------------------
# Configuration of the reader to work at 160KhZ, matching the BLF of a WISP tag.
# -----------------------------------------------------------------------------

from gnuradio import gr
from gnuradio import uhd
from gnuradio import blocks
from gnuradio import filter
from gnuradio import analog
from gnuradio import digital
from gnuradio import qtgui
from gnuradio import fft
# from grc_gnuradio import blks2 as grc_blks2
import rfid
import time
import sys

DEBUG = False
class reader_top_block(gr.top_block):

  # Configure usrp source
  def u_source(self):
    self.source = uhd.usrp_source(
    device_addr=self.usrp_address_source,
    stream_args=uhd.stream_args(
    cpu_format="fc32",
    channels=range(1),
    ),
    )
    self.source.set_samp_rate(self.adc_rate)
    self.source.set_center_freq(self.freq, 0)
    self.source.set_gain(self.rx_gain, 0)
    self.source.set_antenna("RX2", 0)
    self.source.set_auto_dc_offset(False) 
    
  # Configure usrp sink
  def u_sink(self):
    self.sink = uhd.usrp_sink(
    device_addr=self.usrp_address_sink,
    stream_args=uhd.stream_args(
    cpu_format="fc32",
    channels=range(1),
    )
    )
    self.sink.set_samp_rate(self.dac_rate)
    self.sink.set_center_freq(self.freq, 0)
    self.sink.set_gain(self.tx_gain, 0)
    self.sink.set_antenna("TX/RX", 0)

  def __init__(self, myfreq):

    gr.top_block.__init__(self)
    rt = gr.enable_realtime_scheduling() # ******************************************************************************

    ######## Variables #########
    
    self.dac_rate = 100e6/50          # DAC rate 
    self.adc_rate = 100e6/22    # ADC rate (4.54....MS/s complex samples)
    self.decim     = 2          # Decimation (downsampling factor)
    self.ampl     = 0.7   # Output signal amplitude (signal power vary for different RFX900 cards)
    self.freq     = myfreq                # Modulation frequency (can be set between 902-920) # 
    self.rx_gain   = -10   # RX Gain (gain at receiver)
    self.tx_gain   = 24.5 #gains configuration without use of Power Amplifier
    omega = 7
    mu = 0.01
    gain_mu = 0.0001
    gain_omega = 0.25 * gain_mu * gain_mu
    omega_relative_limit = 0.05
       
    self.usrp_address_source = "addr=192.168.10.2,recv_frame_size=256"
    self.usrp_address_sink   = "addr=192.168.10.2,recv_frame_size=256"

    # Each FM0 symbol consists of ADC_RATE/BLF samples (4.54...e6/160e3 =28.2  samples)
    # 14 samples per symbol after matched filtering and decimation
    self.num_taps     = [1] *14 # matched to half symbol period
    
    ######## File sinks for debugging (1 for each block) #########
    self.file_sink_source         = blocks.file_sink(gr.sizeof_gr_complex*1, "../misc/data/source", False)
    self.file_sink_matched_filter = blocks.file_sink(gr.sizeof_gr_complex*1, "../misc/data/matched_filter", False)
    self.file_sink_gate           = blocks.file_sink(gr.sizeof_gr_complex*1, "../misc/data/gate", False)
    self.file_sink_decoder        = blocks.file_sink(gr.sizeof_gr_complex*1, "../misc/data/decoder", False)
    self.file_sink_reader         = blocks.file_sink(gr.sizeof_float*1,      "../misc/data/reader", False)
    self.file_sink_preamble = blocks.file_sink(gr.sizeof_gr_complex*1, "../misc/data/preamble", False)
    # self.file_sink_gate_pbr            = blocks.file_sink(gr.sizeof_gr_complex*1, "../misc/data/gate_pbr", False)

    #sample rate (analog-to-digital) = adc_rate/decimation 

    ####### Blocks #########
    self.matched_filter = filter.fir_filter_ccc(self.decim,self.num_taps)
    self.gate = rfid.gate(int(self.adc_rate/self.decim))
    self.tag_decoder    = rfid.tag_decoder(int(self.adc_rate/self.decim))
    self.reader          = rfid.reader(int(self.adc_rate/self.decim),int(self.dac_rate))
    self.amp              = blocks.multiply_const_ff(self.ampl)
    self.to_complex      = blocks.float_to_complex()
    self.rta_amp = rfid.multiply_rta_ff()
    self.s2v = blocks.stream_to_vector(gr.sizeof_gr_complex*1, 256)
    self.fft1 = fft.fft_vcc(256, True, fft.window.blackmanharris(256), True, 2)
    self.dnn_inference = rfid.dnn_inference()
    self.gate.set_processor_affinity([3])
    self.tag_decoder.set_processor_affinity([1])
    self.reader.set_processor_affinity([1])
    self.fft1.set_processor_affinity([2])
    # self.gate.set_thread_priority(99)
    self.dnn_inference.set_processor_affinity([0])
    self.reader.set_thread_priority(98)
    self.matched_filter.set_processor_affinity([4])
    self.rta_amp.set_processor_affinity([5])
    self.s2v.set_processor_affinity([6])
    self.tag_decoder.set_thread_priority(99)
    self.to_complex.set_processor_affinity([7])

    # self.gate_pbr = rfid.pbr_gate(int(self.adc_rate/self.decim))
    # self.feature_extractor_pbr = rfid.pbr_feature_extractor(int(self.adc_rate/self.decim))
    self.gate.set_min_output_buffer(20000)

    if (DEBUG == False) : # Real Time Execution                                                                                                                                                                                          
      # USRP blocks
      self.u_source()
      self.u_sink()
      # print(dir(rfid))

      ######## Connections #########
      self.connect(self.source,  self.matched_filter)
      self.connect(self.matched_filter, self.gate)

      # self.connect(self.source, self.gate_pbr)
      # self.connect(self.gate_pbr, self.feature_extractor_pbr)
      # self.connect(self.source, self.sink_t)
      # self.connect(self.gate_probe, self.sink_t)
      # self.connect(self.source, self.mute2)
      # self.connect(self.mute1, (self.add, 0))
      # self.connect(self.mute2, (self.add, 1))
      # self.connect(self.add, self.gate)
      # self.connect(self.matched_filter, (self.mode_selector, 0))
      # self.connect(self.source, (self.mode_selector, 1))
      # self.connect(self.source, self.gate_t)
      # self.connect(self.gate_t, self.sink_t)
      # self.connecta(self.gate_t, self.tag_decoder_t)
      # self.connect(self.dc_blocker, self.matched_filter)
      # self.connect((self.mode_selector, 0), self.gate)
      # self.connect(self.agc, self.to_mag)
      # self.connect(self.to_mag, self.dc_blocker)
      # self.connect(self.to_mag, self.to_complex_0)
      # self.connect(self.to_complex_0, self.gate)
      # self.connect(self.gate, self.matched_filter)
      # self.connect(self.to_mag_1, self.minus_one)
      # self.connect(self.minus_one, self.to_complex_1)

      self.connect(self.gate, self.tag_decoder)
      self.connect((self.tag_decoder,0), self.reader)
      self.connect((self.tag_decoder,2), self.s2v)
      self.connect(self.s2v, self.fft1)
      self.connect(self.fft1, self.dnn_inference)
      self.connect(self.reader, self.rta_amp)
      self.connect(self.rta_amp, self.to_complex)
      self.connect(self.to_complex, self.sink)

      # self.msg_connect(self.reader, "reader_command", self.sink, "command")
      # self.connect(self.source, self.gate_probe)
      # self.connect(self.gate_probe, self.tag_decoder_probe)
      # self.connect((self.tag_decoder_probe, 0), self.reader_probe)
      # self.connect(self.reader_probe, self.sink_t)

      #File sinks for logging (Remove comments to log data)
      self.connect(self.source, self.file_sink_source)
      # self.connect(self.agc, self.file_sink_agc)


    else :  # Offline Data
      self.file_source               = blocks.file_source(gr.sizeof_gr_complex*1, "../misc/data/file_sink_source",False)   ## instead of uhd.usrp_source
      self.file_sink                 = blocks.file_sink(gr.sizeof_gr_complex*1,   "../misc/data/file_sink", False)     ## instead of uhd.usrp_sink
 
      ######## Connections ######### 
      # self.connect(self.file_source,  self.matched_filter)
      self.connect(self.file_source, self.gate)
      self.connect(self.gate, self.tag_decoder)
      self.connect((self.tag_decoder,0), self.reader)
      self.connect(self.reader, self.amp)
      self.connect(self.amp, self.to_complex)
      self.connect(self.to_complex, self.file_sink)
    
    #File sinks for logging 
    self.connect(self.gate, self.file_sink_gate)
    self.connect((self.tag_decoder,1), self.file_sink_decoder) # (Do not comment this line)
    self.connect(self.reader, self.file_sink_reader)
    self.connect((self.tag_decoder,2), self.file_sink_preamble)
    # self.connect(self.gate_pbr, self.file_sink_gate_pbr)

if __name__ == '__main__':

  #main_block = reader_top_block(float(915e6))
  for k in range(1):
      # print(dir(rfid))
      main_block = reader_top_block(float(915e6))
      main_block.start()
      while(1):
          time.sleep(25)
          break
      timestp = main_block.reader.print_results()
      #filename = "Data/1.5m/0d/metal/" + str(time_stamp) + ".txt"
      #f = file(filename, "a+")
      #f.write(str(main_block.tx_gain))
      #f.close()
      main_block.stop()
  #main_block.start()

  #while(1):
    #time.sleep(4) 
    #break

  #main_block.reader.print_results()
  #main_block.stop()
