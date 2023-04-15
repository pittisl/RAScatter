/* -*- c++ -*- */
/* 
 * Copyright 2022 <Kai Huang (k.huang[AT]pitt.edu)>.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "rfid/global_vars.h"

#include <iostream>
namespace gr {
  namespace rfid {
    // the following params should be updated for each query in the rate adaptation algorithm
    int valid_packet = 0;
    int prev_transmission_state = -1;
    int curr_transmission_state = -1;
    int n_consecutive_success = 0;
    int n_consecutive_failure = 0;
    int just_elevated = 0;
    
    int ENCODING_SCHEME = index_ES_LIST[index_ES]; // 1/2/4/8 --> FM0/M2/M4/M8
    gr_complex preamble_fm0[6 * 14 * 100] = {gr_complex(0, 0)};
    gr_complex preamble_m8[6 * 14 * 8] = {gr_complex(0, 0)};

    float TAG_BIT_D   = 1.0 * ENCODING_SCHEME/T_READER_FREQ * pow(10,6); // Duration in us
    float RN16_D      = (RN16_BITS + TAG_PREAMBLE_BITS) * TAG_BIT_D;
    float EPC_D         = (EPC_BITS  + TAG_PREAMBLE_BITS) * TAG_BIT_D;
    float HANDLE_D = (RN16_BITS - 1  + TAG_PREAMBLE_BITS + 16) * TAG_BIT_D; // RN16 without? dummy-bit  
    float READ_D = (1 + 32+ 16 + 16+ TAG_PREAMBLE_BITS ) * TAG_BIT_D; // RN16 without? dummy-bit
    
    float Tsk = T1_D + RN16_D + T2_D + Tack + T1_D + EPC_D + T2_D;  //Duration of single/collision slot in us
    float Ti  = T1_D + RN16_D + T2_D; //Duration of idle slot in us
    
    float C_th = C_th_LIST[ENCODING_SCHEME];

    int PROBING_MODE = -1;
    float pktloss_table[4] = {0, 0, 0, 0};
    int probing_rate_cnt = 3; // rate index
    int probing_cnt = 3; // count of single rate
    int n_success = 0;
    clock_t dwelling_clk_start;
    clock_t minstrel_timer;
    // end here


    //BLINK
    int BLINK_MODE = 0; // (0: IDLE, 1: PROBING)
    int blink_scan_cnt = 1;
    int blink_scan_start = 1;
    clock_t blink_timer = clock();
    float blink_pktloss_realtime = 0;
    float blink_pktloss_firstscan = 0;
    float blink_pktloss_secondscan = 0;
    float blink_RSSI_firstscan = 0;
    float blink_RSSI_secondscan = 0;
    float blink_pktloss_probe = 0;
    float blink_RSSI_probe = 0;
    float blink_n_success = 1E-4;
    float blink_n_failure = 0;

    //MobiRate
    float lambda = 0.07; // exponential coefficient
    int MobiRate_MODE = 0; // (0: KEEP, 1: HIGHER, 2: LOWER) 
    clock_t mobirate_timer1 = clock();
    clock_t mobirate_timer2 = clock();
    float mobirate_phase_realtime = 0;
    int mobirate_phase_scan_cnt = 0;
    int mobirate_n_failure = 0;
    float mobirate_pktloss_table[4] = {0};
    
    // signal params
    float RSSI = 1E-10;
    float Phase = 0;
    float NoiseI = 1E-10;
    int cnt_RSSI = 1;
    int cnt_NoiseI = 1;
    int cnt_queries = 0;
    int cnt_preamble_collected = 0;
    float cw_ampl = 0;
    float sig_power = 0;
    float rta_ampl = 0.7 * 0.15; // important param !!
    int index_ES = 3;
    int cnt_power_up_delay = 0;

    //AdaBS
 
    float adabs_throughput_table[4] = {0.0};
    float adabs_lossrate_table[4] = {0.0};
    float adabs_goodput_table[4] = {0.0};
    float adabs_probe_RSSI = 0;
    float adabs_probe_NoiseI = 0;
    float adabs_probe_powerup_delay = 0;
    int adabs_probing_en = 0;
    int adabs_nn_en = 0;
    clock_t adabs_timer1 = clock();
    struct timespec tv_start, tv_end;
    struct timespec start_time, end_time, previous_time;
    struct timespec tp_monitor_st, tp_monitor_ed;
    int cnt_correct_epc = 0;
    int cnt_loss_epc = 0;
    int cnt_loss_epc_global = 0;
    float pktCorrectRatio = 0;
    int H_timeWinLen = 28*4;
    int winIndex = 0;  
    float THROUGHPUT_MONITOR_EN = 1;
    float THROUGHPUT_AVAILABLE = 0;
    float Td_MONITORED = 1E-4;
    int ADABS_PROBING_MODE = 1; // should open if wanna use adabs
    int ADABS_PROBING_DONE = 0;
    int INFERENCE_RESULTS_AVAILABLE = 0;
    float throughput_monitored = 0;
    int cnt_tp_monitored = 0;
    float tp_now = 0;
    float amp_inference = 1; // MAX AMP
    int es_inference = 0; // FM0
    std::vector<float> tp_record;
    std::vector<double> t_record;
    std::vector<double> t_preamble_record;
    std::vector<float> p_record;
    float amp_inference_raw = 1.0;
    double total_time = 0;
    int cnt_inference = 0;
    int cnt_fft = 0;
    float E_Tx = 1E-5;
    float accumulative_d_monitor = 1E-5;

    // DNN inputs
    float goodput_monitored = 0;
    int INFERRED = 0;
    float OBJ_G_PREV = 20;
    float PUD_PREV = 1E-10;
    float RSSI_PREV = 1E-10;
    float NOISEI_PREV = 1E-10;
    std::vector<float> Hft_PREV(28*4, 0);
    //std::vector<std::vector<float> > Hft(4, std::vector<float>(28, 0)); // 4x28
    std::vector<float> Hft(28*4, 0);
    float RSSI_probed = 1E-10;
    float NoiseI_probed = 1E-10;
    float Powerup_Delay_probed = 1E-10;

    // retransmission params
    int bulky_N = 3;
    int retran_is_pkt_loss = 0;
    int retran_goodput_cnt = 0;
    float retran_delay = 0;
    int retran_i_window = 0;
    struct timespec retran_end_time, retran_previous_time;
    int TARGET = 0;
    float retran_gp_now = 0;
    float retran_gp_ave = 0;
    int retran_prev_goodput_cnt = 0;
    float retran_total_time = 0;
    float retran_start_time = 0;
    int retran_goodput_pkt_cnt = 0;
    int retran_goodput_pkt_prev_cnt = 0;

    READER_STATE * reader_state;
    float RN16_handle_stored[16];
    void initialize_reader_state()
    {
      reader_state = new READER_STATE;
      reader_state-> reader_stats.n_queries_sent = 0;
      reader_state-> reader_stats.n_powerup_sent = 0;
      reader_state-> reader_stats.n_epc_correct = 0;
      reader_state-> reader_stats.n_epc_detected = 0;
      reader_state-> reader_stats.noise_power = 1;
      reader_state-> reader_stats.bs_power = 0;
      reader_state-> reader_stats.output_energy = 0;
      reader_state-> reader_stats.average_throughput = 0;
      reader_state-> reader_stats.power_up_delay = 0;
      reader_state-> reader_stats.aux_buffer_flag = 0;
      reader_state-> reader_stats.previous_time = clock();

      reader_state-> reader_stats.tn_k = 0; //Total Number of collision slots 
      reader_state-> reader_stats.tn_1 = 0; //Total Number of success slots 
      reader_state-> reader_stats.tn_0 = 0; //Total Number of idle slots 
      reader_state-> reader_stats.tQA = 0; //Total Number of QA sent
      reader_state-> reader_stats.tQ = 0; //Total Number of Q sent
      reader_state-> reader_stats.tQR = 0; //Total Number of QR sent
      reader_state-> reader_stats.sensor_read = 0;
      
      reader_state-> reader_stats.n_k = 0; //Number of collision slots per frame
      reader_state-> reader_stats.n_1 = 0; //Number of success slots per frame
      reader_state-> reader_stats.n_0 = 0; //Number of idle slots per frame

      reader_state-> reader_stats.VAR_Q = 1; //Initial Q value -> L=2^Q
      reader_state-> reader_stats.Qant = 1; 
      reader_state-> reader_stats.Qfp = 1.0; 

      reader_state-> reader_stats.Qupdn = 0; 

      reader_state-> reader_stats.it_timer = 0.0;


      reader_state-> reader_stats.th = 0.0;
      reader_state-> reader_stats.TIR_th = 0.0;
      reader_state-> reader_stats.TIR_exp = 0.0;
      reader_state-> reader_stats.stop = 1;
      /*
      // initialize transmission state vars
      reader_state-> reader_stats.n_consecutive_success = 0;
      reader_state-> reader_stats.n_consecutive_failure = 0;
      reader_state-> reader_stats.prev_transmission_state = -1; // 0 fail, 1 success
      reader_state-> reader_stats.curr_transmission_state = -1; // 0 fail, 1 success
      reader_state-> reader_stats.index_ES = 0;
      reader_state-> reader_stats.just_elevated = 0;

      */
      //reader_state-> reader_stats.Qupdn = 1; //posible values: 0,1,2

      std::vector<int>  unique_tags_round;
      std::map<int,int> tag_reads; 
      std::vector<float> RN16_bits_handle;  
      std::vector<float> RN16_bits_read; 

      reader_state-> status           = RUNNING;
      reader_state-> gen2_logic_status= START;
      reader_state-> gate_status       = GATE_SEEK_RN16;
      reader_state-> decoder_status   = DECODER_DECODE_RN16;

      reader_state-> reader_stats.max_slot_number = pow(2,FIXED_Q);

      reader_state-> reader_stats.cur_inventory_round = 1;
      reader_state-> reader_stats.cur_slot_number     = 1;

      //gettimeofday (&reader_state-> reader_stats.start, NULL);
    }
  } /* namespace rfid */
} /* namespace gr */

