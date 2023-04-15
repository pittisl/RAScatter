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
#include "reader_impl.h"
#include "rfid/global_vars.h"
#include "tag_decoder_impl.h"
#include <sys/time.h>
#include<iomanip>
#include <bitset>
#include <math.h>
using namespace std;

namespace gr {
  namespace rfid {

    reader::sptr
    reader::make(int sample_rate, int dac_rate)
    {
      return gnuradio::get_initial_sptr
        (new reader_impl(sample_rate,dac_rate));
    }

    /*
     * The private constructor
     */
    reader_impl::reader_impl(int sample_rate, int dac_rate)
      : gr::block("reader",
              gr::io_signature::make( 1, 1, sizeof(float)),
              gr::io_signature::make( 1, 1, sizeof(float)))
    {
      //message_port_register_out(pmt::mp("reader_command"));
      command_bits = (char *) malloc( sizeof(char) * 24);
      sample_d = 1.0/dac_rate * pow(10,6);

      // Number of samples for transmitting
      n_data0_s = 2 * PW_D / sample_d;
      n_data1_s = 4 * PW_D / sample_d;
      n_pw_s    = PW_D    / sample_d;
      n_cw_s    = CW_D    / sample_d;
      n_delim_s = DELIM_D / sample_d;
      n_trcal_s = TRCAL_D / sample_d;

      // CW waveforms of different sizes
      n_cwquery_s   = (T1_D+T2_D+RN16_D)/sample_d;     //RN16 ---
      n_cwack_s     = (1*T1_D+T2_D+EPC_D)/sample_d;    //EPC   if it is longer than nominal it wont cause tags to change inventoried flag ---
      n_cwreq_s   = (T1_D+T2_D+HANDLE_D)/sample_d;     //Handle or new rn16
      n_cwread_s   = (T1_D+T2_D+READ_D)/sample_d;     //READ
      n_p_down_s     = (1*P_DOWN_D)/sample_d;  

      p_down.resize(n_p_down_s);        // Power down samples
      cw_query.resize(n_cwquery_s);      // Sent after query/query rep ---
      cw_ack.resize(n_cwack_s);          // Sent after ack ---
      cw_start.resize(n_cw_s);          // Sent after start
      cw_req_rn16.resize(n_cwreq_s);          // Sent after Req_RN16
      cw_read.resize(n_cwread_s);          // Sent after READ


      std::fill_n(cw_query.begin(), cw_query.size(), 1); // ---
      std::fill_n(cw_ack.begin(), cw_ack.size(), 1); // ---
      std::fill_n(cw_start.begin(), cw_start.size(), 1);
      std::fill_n(cw_req_rn16.begin(), cw_req_rn16.size(), 1);
      std::fill_n(cw_read.begin(), cw_read.size(), 1);

      // Construct vectors (resize() default initialization is zero)
      data_0.resize(n_data0_s);
      data_1.resize(n_data1_s);
      cw.resize(n_cw_s);
      delim.resize(n_delim_s);
      rtcal.resize(n_data0_s + n_data1_s);
      trcal.resize(n_trcal_s);

      // Fill vectors with data
      std::fill_n(data_0.begin(), data_0.size()/2, 1);
      std::fill_n(data_1.begin(), 3*data_1.size()/4, 1);
      std::fill_n(cw.begin(), cw.size(), 1);
      std::fill_n(rtcal.begin(), rtcal.size() - n_pw_s, 1); // RTcal
      std::fill_n(trcal.begin(), trcal.size() - n_pw_s, 1); // TRcal

      // create preamble
      preamble.insert( preamble.end(), delim.begin(), delim.end() );
      preamble.insert( preamble.end(), data_0.begin(), data_0.end() );
      preamble.insert( preamble.end(), rtcal.begin(), rtcal.end() );
      preamble.insert( preamble.end(), trcal.begin(), trcal.end() );

      // create framesync
      frame_sync.insert( frame_sync.end(), delim.begin() , delim.end() );
      frame_sync.insert( frame_sync.end(), data_0.begin(), data_0.end() );
      frame_sync.insert( frame_sync.end(), rtcal.begin() , rtcal.end() );
      
      // create query rep
      query_rep.insert( query_rep.end(), frame_sync.begin(), frame_sync.end());
      query_rep.insert( query_rep.end(), data_0.begin(), data_0.end() );
      query_rep.insert( query_rep.end(), data_0.begin(), data_0.end() );
      query_rep.insert( query_rep.end(), data_0.begin(), data_0.end() );
      query_rep.insert( query_rep.end(), data_0.begin(), data_0.end() );

      // create nak
      nak.insert( nak.end(), frame_sync.begin(), frame_sync.end());
      nak.insert( nak.end(), data_1.begin(), data_1.end() );
      nak.insert( nak.end(), data_1.begin(), data_1.end() );
      nak.insert( nak.end(), data_0.begin(), data_0.end() );
      nak.insert( nak.end(), data_0.begin(), data_0.end() );
      nak.insert( nak.end(), data_0.begin(), data_0.end() );
      nak.insert( nak.end(), data_0.begin(), data_0.end() );
      nak.insert( nak.end(), data_0.begin(), data_0.end() );
      nak.insert( nak.end(), data_0.begin(), data_0.end() );



    }

    void reader_impl::gen_query_bits()
    {
      int num_ones = 0, num_zeros = 0;

      query_bits.resize(0);
      query_bits.insert(query_bits.end(), &QUERY_CODE[0], &QUERY_CODE[4]);
      query_bits.push_back(DR);
      /*
      if (cnt_queries > 2000 && cnt_queries < 4000) {
        adabs_nn_en = 1;
      }
      else {
        adabs_nn_en = 0;
      }
      */
      /////////////////////////////////////////////////////////////////////////////////////////
      // ADABS RATE ADAPTATION (my own algorithm)
      /////////////////////////////////////////////////////////////////////////////////////////
      if (ADABS_EN == 1) {
        if (THROUGHPUT_MONITOR_EN == 1 && THROUGHPUT_AVAILABLE == 1) {
          //cout << "\n---tp reading available once---" << endl;
          //float goodput_monitored = pow(pktCorrectRatio, bulky_N - 1) * tp_now;
          goodput_monitored = throughput_monitored;
          cnt_loss_epc = 0;
          cnt_tp_monitored = 0;
          throughput_monitored = 0;
          cout << "tp monitored : " << goodput_monitored << endl;
          float P_Tx = E_Tx / accumulative_d_monitor;
          //std::cout << "BpJ (kbits/J) : " << 0.32 * retran_goodput_pkt_cnt / total_time / P_Tx << std::endl;
          //tp_record.push_back(goodput_monitored);
          //t_record.push_back(total_time);
          if (goodput_monitored < TP_LOWER_LIMIT || goodput_monitored > TP_UPPER_LIMIT) {
            // prepare for probing
            ADABS_PROBING_MODE = 1;
            // FM0 & AMP = 1 for probing
            index_ES = 3;
            rta_ampl = 0.7 * 1;
            
            THROUGHPUT_MONITOR_EN = 0; // when to enable?
            THROUGHPUT_AVAILABLE = 0;
            ADABS_PROBING_DONE = 0;
            RSSI = 1E-10;
            NoiseI = 1E-10;
            cnt_NoiseI = 1;
            cnt_RSSI = 1;
            cout << "\n---start probing---" << endl;
            //cout << "---tp monitor is off---\n" << endl;
          }
          THROUGHPUT_AVAILABLE = 0;
        }
        if (ADABS_PROBING_MODE == 1 && ADABS_PROBING_DONE == 1) { // (powerup-delay is ready)
          // call DNN
          adabs_nn_en = 1; // it should be set 0 by dnn after inference
          //cout << "\n---end probing---\n" << endl;
        }
        if (INFERENCE_RESULTS_AVAILABLE == 1) { // DNN inference is done
          // adjust transmission parameters
          
          rta_ampl = 0.7 * amp_inference;
          index_ES = 3 - es_inference;
          
          if (ADABS_PROBING_DONE == 1) { // if next power-up delay is available, tp monitor will be enabled again (ADABS_PROBING_DONE == 1?)
            INFERENCE_RESULTS_AVAILABLE = 0;
            THROUGHPUT_MONITOR_EN = 1;
            //cout << "\n---tp monitor is on---\n" << endl;
          }
        }
        
        ENCODING_SCHEME = index_ES_LIST[index_ES];
        if (ENCODING_SCHEME == 1)
        {
          query_bits.insert(query_bits.end(), &M_FM0[0], &M_FM0[2]);
          bulky_N = 3;
        }
        else if (ENCODING_SCHEME == 2)
        {
          query_bits.insert(query_bits.end(), &M_Miller2[0], &M_Miller2[2]);
          bulky_N = 2;
        }
        else if (ENCODING_SCHEME == 4)
        {
          query_bits.insert(query_bits.end(), &M_Miller4[0], &M_Miller4[2]);
          bulky_N = 1;
        }
        else
        {
          query_bits.insert(query_bits.end(), &M_Miller8[0], &M_Miller8[2]);
          bulky_N = 1;
        }
        // adjust other params
        TAG_BIT_D   = 1.0 * ENCODING_SCHEME/T_READER_FREQ * pow(10,6); // Duration in us
        RN16_D      = (RN16_BITS + TAG_PREAMBLE_BITS) * TAG_BIT_D;
        EPC_D         = (EPC_BITS  + TAG_PREAMBLE_BITS) * TAG_BIT_D;
        HANDLE_D = (RN16_BITS - 1  + TAG_PREAMBLE_BITS + 16) * TAG_BIT_D; // RN16 without? dummy-bit  
        READ_D = (1 + 32+ 16 + 16+ TAG_PREAMBLE_BITS ) * TAG_BIT_D; // RN16 without? dummy-bit
    
        Tsk = T1_D + RN16_D + T2_D + Tack + T1_D + EPC_D + T2_D;  //Duration of single/collision slot in us
        Ti  = T1_D + RN16_D + T2_D; //Duration of idle slot in us
    
        C_th = C_th_LIST[ENCODING_SCHEME];

        n_cwquery_s   = (T1_D+T2_D+RN16_D)/sample_d;     //RN16 ---
        n_cwack_s     = (1*T1_D+T2_D+EPC_D)/sample_d;    //EPC   if it is longer than nominal it wont cause tags to change inventoried flag ---

        std::vector<float>().swap(cw_query);
        std::vector<float>().swap(cw_ack);

        cw_query.resize(n_cwquery_s);      // Sent after query/query rep ---
        cw_ack.resize(n_cwack_s);          // Sent after ack ---

        std::fill_n(cw_query.begin(), cw_query.size(), 1); // ---
        std::fill_n(cw_ack.begin(), cw_ack.size(), 1); // ---
        valid_packet = 0;

      }
      
      ////////////////////////////////////////////////////////////////////////////////////////
      // Only for data collection
      ////////////////////////////////////////////////////////////////////////////////////////
      if (RFID_LOCALIZATION == 1) {
        ENCODING_SCHEME = 1;
        if (ENCODING_SCHEME == 1)
        {
          query_bits.insert(query_bits.end(), &M_FM0[0], &M_FM0[2]);
        }
        else if (ENCODING_SCHEME == 2)
        {
          query_bits.insert(query_bits.end(), &M_Miller2[0], &M_Miller2[2]);
        }
        else if (ENCODING_SCHEME == 4)
        {
          query_bits.insert(query_bits.end(), &M_Miller4[0], &M_Miller4[2]);
        }
        else
        {
          query_bits.insert(query_bits.end(), &M_Miller8[0], &M_Miller8[2]);
        }
        // adjust other params
        TAG_BIT_D   = 1.0 * ENCODING_SCHEME/T_READER_FREQ * pow(10,6); // Duration in us
        RN16_D      = (RN16_BITS + TAG_PREAMBLE_BITS) * TAG_BIT_D;
        EPC_D         = (EPC_BITS  + TAG_PREAMBLE_BITS) * TAG_BIT_D;
        HANDLE_D = (RN16_BITS - 1  + TAG_PREAMBLE_BITS + 16) * TAG_BIT_D; // RN16 without? dummy-bit  
        READ_D = (1 + 32+ 16 + 16+ TAG_PREAMBLE_BITS ) * TAG_BIT_D; // RN16 without? dummy-bit
    
        Tsk = T1_D + RN16_D + T2_D + Tack + T1_D + EPC_D + T2_D;  //Duration of single/collision slot in us
        Ti  = T1_D + RN16_D + T2_D; //Duration of idle slot in us
    
        C_th = C_th_LIST[ENCODING_SCHEME];

        n_cwquery_s   = (T1_D+T2_D+RN16_D)/sample_d;     //RN16 ---
        n_cwack_s     = (1*T1_D+T2_D+EPC_D)/sample_d;    //EPC   if it is longer than nominal it wont cause tags to change inventoried flag ---

        std::vector<float>().swap(cw_query);
        std::vector<float>().swap(cw_ack);

        cw_query.resize(n_cwquery_s);      // Sent after query/query rep ---
        cw_ack.resize(n_cwack_s);          // Sent after ack ---

        std::fill_n(cw_query.begin(), cw_query.size(), 1); // ---
        std::fill_n(cw_ack.begin(), cw_ack.size(), 1); // ---
        valid_packet = 0;
      }
      
      if (DATA_COLLECTION_EN == 1) {
        if (cnt_queries == 1500) { // warm up
          RSSI = 0;
          NoiseI = 0;
          reader_state->reader_stats.power_up_delay = 0;
          cnt_RSSI = 1;
          cnt_NoiseI = 1;
          cnt_power_up_delay = 0;
          ADABS_PROBING_MODE = 1;
          //cout << "probing started!" << endl;
        }
        if (cnt_queries == 3500) { // probe FM0 after getting channel condition metrics 
          bulky_N = 3;
          //cout << "probing done!" << endl;
          adabs_probe_RSSI = RSSI;
          adabs_probe_NoiseI = NoiseI;
          adabs_probe_powerup_delay = reader_state->reader_stats.power_up_delay;
          //clock_gettime(CLOCK_MONOTONIC, &start_time);
          //reader_state->reader_stats.n_epc_correct = 0;
          //reader_state->reader_stats.average_throughput = 0;
          
          retran_prev_goodput_cnt = retran_goodput_cnt;
          reader_state->reader_stats.n_epc_detected = 0;
          //retran_start_time = retran_total_time;

          reader_state->reader_stats.n_epc_correct = 0;
          cnt_loss_epc = 0;

          throughput_monitored = 0;
          cnt_tp_monitored = 0;
          //retran_gp_ave = 0;
          //retran_goodput_cnt = 0;
          ENCODING_SCHEME = 1;
          index_ES = 3;

          rta_ampl = 0.7 * 0.40;
          AMP = 40;
          CCI = 100;
          SI = 0;

          adabs_probing_en = 0;
          ADABS_PROBING_MODE = 0;
        }
        else if (cnt_queries == 10000) { // probe M2
          bulky_N = 2;
          adabs_throughput_table[0] = 3 * (retran_goodput_cnt - retran_prev_goodput_cnt) / retran_total_time; // record FM0 TP
          //adabs_lossrate_table[0] = (cnt_loss_epc + 1e-5) / (cnt_loss_epc + reader_state->reader_stats.n_epc_correct + 1e-5); // record FM0 lossrate
          //clock_gettime(CLOCK_MONOTONIC, &start_time);
          //reader_state->reader_stats.n_epc_correct = 0;
          //reader_state->reader_stats.average_throughput = 0;
          retran_prev_goodput_cnt = retran_goodput_cnt;
          reader_state->reader_stats.n_epc_detected = 0;
          //retran_start_time = retran_total_time;

          reader_state->reader_stats.n_epc_correct = 0;
          cnt_loss_epc = 0;

          //retran_goodput_cnt = 0;
          //retran_gp_ave = 0;
          cnt_tp_monitored = 0;
          index_ES = 2;
          ENCODING_SCHEME = 2;
        }
        else if (cnt_queries == 16000) { // probe M4
          bulky_N = 1;
          adabs_throughput_table[1] = 2 * (retran_goodput_cnt - retran_prev_goodput_cnt) / retran_total_time; // record M2 TP
          //adabs_lossrate_table[1] = (cnt_loss_epc + 1e-5) / (cnt_loss_epc + reader_state->reader_stats.n_epc_correct + 1e-5); // record M2 lossrate
          //clock_gettime(CLOCK_MONOTONIC, &start_time);
          //reader_state->reader_stats.n_epc_correct = 0;
          //reader_state->reader_stats.average_throughput = 0;
          reader_state->reader_stats.n_epc_detected = 0;
          retran_prev_goodput_cnt = retran_goodput_cnt;
          //retran_start_time = retran_total_time;

          reader_state->reader_stats.n_epc_correct = 0;
          cnt_loss_epc = 0;
           
          //retran_goodput_cnt = 0;
          //retran_gp_ave = 0;
          cnt_tp_monitored = 0;
          index_ES = 1;
          ENCODING_SCHEME = 4;
        }
        else if (cnt_queries == 18000) { // probe M8
          bulky_N = 1;
          adabs_throughput_table[2] = 0 * (retran_goodput_cnt - retran_prev_goodput_cnt) / retran_total_time; // record M4 TP
          //adabs_lossrate_table[2] = (cnt_loss_epc + 1e-5) / (cnt_loss_epc + reader_state->reader_stats.n_epc_correct + 1e-5); // record M4 lossrate
          //clock_gettime(CLOCK_MONOTONIC, &start_time);
          //reader_state->reader_stats.n_epc_correct = 0;
          //reader_state->reader_stats.average_throughput = 0;
          reader_state->reader_stats.n_epc_detected = 0;
          retran_prev_goodput_cnt = retran_goodput_cnt;
          //retran_start_time = retran_total_time;

          reader_state->reader_stats.n_epc_correct = 0;
          cnt_loss_epc = 0;

          //retran_goodput_cnt = 0;
          //retran_gp_ave = 0;
          cnt_tp_monitored = 0;
          index_ES = 0;
          ENCODING_SCHEME = 8;
        }
        else if (cnt_queries == 20000) {
          adabs_throughput_table[3] = 0 * (retran_goodput_cnt - retran_prev_goodput_cnt) / (retran_total_time - retran_start_time); // record M8 TP
          //adabs_lossrate_table[3] = (cnt_loss_epc + 1e-5) / (cnt_loss_epc + reader_state->reader_stats.n_epc_correct + 1e-5); // record M8 lossrate
        }

        if (ENCODING_SCHEME == 1)
        {
          query_bits.insert(query_bits.end(), &M_FM0[0], &M_FM0[2]);
        }
        else if (ENCODING_SCHEME == 2)
        {
          query_bits.insert(query_bits.end(), &M_Miller2[0], &M_Miller2[2]);
        }
        else if (ENCODING_SCHEME == 4)
        {
          query_bits.insert(query_bits.end(), &M_Miller4[0], &M_Miller4[2]);
        }
        else
        {
          query_bits.insert(query_bits.end(), &M_Miller8[0], &M_Miller8[2]);
        }
        // adjust other params
        TAG_BIT_D   = 1.0 * ENCODING_SCHEME/T_READER_FREQ * pow(10,6); // Duration in us
        RN16_D      = (RN16_BITS + TAG_PREAMBLE_BITS) * TAG_BIT_D;
        EPC_D         = (EPC_BITS  + TAG_PREAMBLE_BITS) * TAG_BIT_D;
        HANDLE_D = (RN16_BITS - 1  + TAG_PREAMBLE_BITS + 16) * TAG_BIT_D; // RN16 without? dummy-bit  
        READ_D = (1 + 32+ 16 + 16+ TAG_PREAMBLE_BITS ) * TAG_BIT_D; // RN16 without? dummy-bit
    
        Tsk = T1_D + RN16_D + T2_D + Tack + T1_D + EPC_D + T2_D;  //Duration of single/collision slot in us
        Ti  = T1_D + RN16_D + T2_D; //Duration of idle slot in us
    
        C_th = C_th_LIST[ENCODING_SCHEME];

        n_cwquery_s   = (T1_D+T2_D+RN16_D)/sample_d;     //RN16 ---
        n_cwack_s     = (1*T1_D+T2_D+EPC_D)/sample_d;    //EPC   if it is longer than nominal it wont cause tags to change inventoried flag ---

        std::vector<float>().swap(cw_query);
        std::vector<float>().swap(cw_ack);

        cw_query.resize(n_cwquery_s);      // Sent after query/query rep ---
        cw_ack.resize(n_cwack_s);          // Sent after ack ---

        std::fill_n(cw_query.begin(), cw_query.size(), 1); // ---
        std::fill_n(cw_ack.begin(), cw_ack.size(), 1); // ---
        valid_packet = 0;
      }
      
     /*
      //Adjust Tx power test
      if (cnt_queries == 1000) {
        float amp_scalar = 1;  // [0, 1]
        rta_ampl = 0.7 * amp_scalar;
        cout << "power changed" << endl;
      }
      */
     /*
     if (cnt_queries % 200 == 0) {
       cout << "cw_ampl : " << cw_ampl << endl;
       if (cnt_queries % 1000 == 0) {
         rta_ampl = (rta_ampl > 0.07 ? rta_ampl - 0.07 : rta_ampl);
         cout << "rta_ampl : " << rta_ampl << endl;
       }
     }
     */
      /*
      if (cnt_queries == 1000) {
        pmt::pmt_t command = pmt::make_dict();
        command = pmt::dict_add(command, pmt::mp("gain"), pmt::mp(22.0)); // Specify gain
        command = pmt::dict_add(command, pmt::mp("chan"), pmt::mp(0));
        message_port_pub(pmt::mp("reader_command"), command);
        cout << "processed" << endl;
      } 
 */
      //////////////////////////////////////////////////////////////////////////////////////////////////
      // rate adaptation algorithm 1: Auto Rate Fallback (Debugged)
      // It starts with the lowest rate.
      if (AUTO_RATE_FALLBACK_EN == 1) {
        if (valid_packet == 1) { // no valid limits
          // calculate consecutive transmission vars
          if (prev_transmission_state == -1 && curr_transmission_state == -1) {
            prev_transmission_state = 0;
            curr_transmission_state = 0;
          }
          else if (prev_transmission_state == 0 && curr_transmission_state == 0) {
            n_consecutive_failure++;
          }
          else if (prev_transmission_state == 1 && curr_transmission_state == 1) {
            n_consecutive_success++;
          }
          else if (prev_transmission_state == 1 && curr_transmission_state == 0) {
            n_consecutive_success = 0;
            n_consecutive_failure = 1;
          }
          else { // prev_transmission_state == 0 && curr_transmission_state == 1
            n_consecutive_failure = 0;
            n_consecutive_success = 1;
          }
          
          if (just_elevated == 1 && curr_transmission_state == 1) {
            just_elevated = 0;
          }

          // adjust rate
          // switch to lower rate if 2 successive failures happen or 1 failure happens right after switching to the new rate
          if (n_consecutive_failure == 2 || (n_consecutive_failure == 1 && just_elevated == 1)) {
            n_consecutive_success = 0;
            n_consecutive_failure = 0;
            just_elevated = 0;
            if (index_ES > 0) {
              index_ES--;
              cout << "adjust to " << index_ES_LIST[index_ES] << endl;
            }
          }
          // 
          if (n_consecutive_success == 3) {
            n_consecutive_success = 0;
            n_consecutive_failure = 0;
            if (index_ES < 3) {
              just_elevated = 1;
              index_ES++;
              cout << "adjust to " << index_ES_LIST[index_ES] << endl;
            }
          }
          curr_transmission_state = 0; // new added
        }
        ENCODING_SCHEME = index_ES_LIST[index_ES];
        if (ENCODING_SCHEME == 1)
        {
          query_bits.insert(query_bits.end(), &M_FM0[0], &M_FM0[2]);
        }
        else if (ENCODING_SCHEME == 2)
        {
          query_bits.insert(query_bits.end(), &M_Miller2[0], &M_Miller2[2]);
        }
        else if (ENCODING_SCHEME == 4)
        {
          query_bits.insert(query_bits.end(), &M_Miller4[0], &M_Miller4[2]);
        }
        else
        {
          query_bits.insert(query_bits.end(), &M_Miller8[0], &M_Miller8[2]);
        }
        // adjust other params
        TAG_BIT_D   = 1.0 * ENCODING_SCHEME/T_READER_FREQ * pow(10,6); // Duration in us
        RN16_D      = (RN16_BITS + TAG_PREAMBLE_BITS) * TAG_BIT_D;
        EPC_D         = (EPC_BITS  + TAG_PREAMBLE_BITS) * TAG_BIT_D;
        HANDLE_D = (RN16_BITS - 1  + TAG_PREAMBLE_BITS + 16) * TAG_BIT_D; // RN16 without? dummy-bit  
        READ_D = (1 + 32+ 16 + 16+ TAG_PREAMBLE_BITS ) * TAG_BIT_D; // RN16 without? dummy-bit
    
        Tsk = T1_D + RN16_D + T2_D + Tack + T1_D + EPC_D + T2_D;  //Duration of single/collision slot in us
        Ti  = T1_D + RN16_D + T2_D; //Duration of idle slot in us
    
        C_th = C_th_LIST[ENCODING_SCHEME];

        n_cwquery_s   = (T1_D+T2_D+RN16_D)/sample_d;     //RN16 ---
        n_cwack_s     = (1*T1_D+T2_D+EPC_D)/sample_d;    //EPC   if it is longer than nominal it wont cause tags to change inventoried flag ---

        std::vector<float>().swap(cw_query);
        std::vector<float>().swap(cw_ack);

        cw_query.resize(n_cwquery_s);      // Sent after query/query rep ---
        cw_ack.resize(n_cwack_s);          // Sent after ack ---

        std::fill_n(cw_query.begin(), cw_query.size(), 1); // ---
        std::fill_n(cw_ack.begin(), cw_ack.size(), 1); // ---
        valid_packet = 0;
      }
      /////////////////////////////////////////////////////////////////////////////////////////////////////
      // rate adaptation algorithm 2: Minstrel (Debugged)
      else if (MINSTREL_EN == 1) { // Basically it sends some non-optimal packets to probe channel.
        // In each probing, it evaluates the throughput of each rate and maintain a performance table using EMA.
        // It starts with the highest rate.
        // probing
        if (PROBING_MODE == -1) {
          //cout << "start probing" << endl;
          // start timer for probing
          minstrel_timer = clock();
          PROBING_MODE = 1;
        }
        if (PROBING_MODE == 1) {
          
          n_success += curr_transmission_state * valid_packet;
          // end probing & adjust rate
          double interval = ((double) (clock() - minstrel_timer)) / CLOCKS_PER_SEC;
          if (interval > 1 && probing_rate_cnt == 0) {
            //cout << "end probing" << endl;
            // summarize the last rate
            pktloss_table[probing_rate_cnt] = 0.25 * pktloss_table[probing_rate_cnt] + 0.75 * n_success;
            PROBING_MODE = 0;
            probing_rate_cnt = 3;
            PROBING_MODE = 0;
            // adjust rate
            float max_tp = 0;
            for (int i = 0; i < 3; i++) {
              if (pktloss_table[i] / index_ES_LIST[i] > max_tp) {
                max_tp = pktloss_table[i] / index_ES_LIST[i];
                ENCODING_SCHEME = index_ES_LIST[i];
              }
            }
            //cout << "adjust to rate: " << ENCODING_SCHEME << endl;
            dwelling_clk_start = clock();
          }
          // probing on-going
          else {
            // probing end of one rate
            if (interval > 2.0) {
              //summarize this rate
              pktloss_table[probing_rate_cnt] = 0.25 * pktloss_table[probing_rate_cnt] + 0.75 * n_success / 3;
              probing_rate_cnt--;
              minstrel_timer = clock();
              n_success = 0;
              // switch to next rate
              ENCODING_SCHEME = index_ES_LIST[probing_rate_cnt];
            }
            else {
              ENCODING_SCHEME = index_ES_LIST[probing_rate_cnt];
            }
          }
          valid_packet = 0;
        }
        // keep current rate until timeout
        if ((clock() - dwelling_clk_start) / CLOCKS_PER_SEC > 3.0) {
          PROBING_MODE = -1;
        }

        if (ENCODING_SCHEME == 1)
        {
          query_bits.insert(query_bits.end(), &M_FM0[0], &M_FM0[2]);
        }
        else if (ENCODING_SCHEME == 2)
        {
          query_bits.insert(query_bits.end(), &M_Miller2[0], &M_Miller2[2]);
        }
        else if (ENCODING_SCHEME == 4)
        {
          query_bits.insert(query_bits.end(), &M_Miller4[0], &M_Miller4[2]);
        }
        else
        {
          query_bits.insert(query_bits.end(), &M_Miller8[0], &M_Miller8[2]);
        }
        // adjust other params
        TAG_BIT_D   = 1.0 * ENCODING_SCHEME/T_READER_FREQ * pow(10,6); // Duration in us
        RN16_D      = (RN16_BITS + TAG_PREAMBLE_BITS) * TAG_BIT_D;
        EPC_D         = (EPC_BITS  + TAG_PREAMBLE_BITS) * TAG_BIT_D;
        HANDLE_D = (RN16_BITS - 1  + TAG_PREAMBLE_BITS + 16) * TAG_BIT_D; // RN16 without? dummy-bit  
        READ_D = (1 + 32+ 16 + 16+ TAG_PREAMBLE_BITS ) * TAG_BIT_D; // RN16 without? dummy-bit
    
        Tsk = T1_D + RN16_D + T2_D + Tack + T1_D + EPC_D + T2_D;  //Duration of single/collision slot in us
        Ti  = T1_D + RN16_D + T2_D; //Duration of idle slot in us
    
        C_th = C_th_LIST[ENCODING_SCHEME];

        n_cwquery_s   = (T1_D+T2_D+RN16_D)/sample_d;     //RN16 ---
        n_cwack_s     = (1*T1_D+T2_D+EPC_D)/sample_d;    //EPC   if it is longer than nominal it wont cause tags to change inventoried flag ---

        std::vector<float>().swap(cw_query);
        std::vector<float>().swap(cw_ack);

        cw_query.resize(n_cwquery_s);      // Sent after query/query rep ---
        cw_ack.resize(n_cwack_s);          // Sent after ack ---

        std::fill_n(cw_query.begin(), cw_query.size(), 1); // ---
        std::fill_n(cw_ack.begin(), cw_ack.size(), 1); // ---
        valid_packet = 0;
      }
      ///////////////////////////////////////////////////////////////////////////////////////////////
      // Fixed rate (Debugged)
      else if (FIXED_RATE_EN == 1) {
        /*
        if (rta_ampl > 0) {
          if ((cnt_queries % 1000) == 0) {
            rta_ampl -= 0.035;
          }
        }
        */
        if (ENCODING_SCHEME == 1)
        {
          query_bits.insert(query_bits.end(), &M_FM0[0], &M_FM0[2]);
        }
        else if (ENCODING_SCHEME == 2)
        {
          query_bits.insert(query_bits.end(), &M_Miller2[0], &M_Miller2[2]);
        }
        else if (ENCODING_SCHEME == 4)
        {
          query_bits.insert(query_bits.end(), &M_Miller4[0], &M_Miller4[2]);
        }
        else
        {
          query_bits.insert(query_bits.end(), &M_Miller8[0], &M_Miller8[2]);
        }
      }
      /////////////////////////////////////////////////////////////////////////////////////////
      // BLINK turn on averaging RSSI and pktloss
      else if (BLINK_EN == 1) {
        if (BLINK_MODE == 0) {// IDLE
          if (blink_scan_cnt == 1) { // first scan
            if (blink_scan_start == 1) { // start timer for first scan
              cout << "first scan start" << endl;
              cnt_queries = 0;
              blink_timer = clock();
              blink_scan_start = 0;
              blink_n_success = 1E-4;
              blink_n_failure = 0;
            }
            double interval = ((double) (clock() - blink_timer)) / CLOCKS_PER_SEC;
            if (interval > 2.0) {
              cout << "first scan end" << endl;
              // summarize pktloss & RSSI
              blink_pktloss_firstscan = blink_n_success / cnt_queries;
              blink_RSSI_firstscan = RSSI;
              blink_scan_cnt = 2; // complete first scan
              blink_scan_start = 1;
            }

          }
          else if (blink_scan_cnt == 2) { // second scan
            if (blink_scan_start == 1) { // start timer for first scan
              cout << "second scan start" << endl;
              cnt_queries = 0;
              blink_timer = clock();
              blink_scan_start = 0;
              blink_n_success = 1E-4;
              blink_n_failure = 0;
            }
            double interval = ((double) (clock() - blink_timer)) / CLOCKS_PER_SEC;
            if (interval > 2.0) {
              cout << "second scan end" << endl;
              blink_scan_cnt = 0; // complete two scans
              // summarize pktloss & RSSI
              blink_pktloss_secondscan = blink_n_success / cnt_queries;
              blink_RSSI_secondscan = RSSI;
              blink_scan_start = 1;
            }
          }
          else { // blink_scan_cnt == 0 for trigger
            float diff_RSSI = (blink_RSSI_secondscan - blink_RSSI_firstscan) * (blink_RSSI_secondscan - blink_RSSI_firstscan);
            float diff_pktloss = (blink_pktloss_secondscan - blink_pktloss_firstscan) * (blink_pktloss_secondscan - blink_pktloss_firstscan);
            cout << "dRSSI : " << diff_RSSI << endl;
            cout << "dpktloss : " << diff_pktloss << endl;
            if (diff_RSSI > Th_RSSI || diff_pktloss > Th_pktloss) {
              BLINK_MODE = 1; // probing triggered
              ENCODING_SCHEME = index_ES_LIST[0];
              cout << "probing triggered" << endl;
            }
            blink_scan_cnt = 1;
          }
        }
        else { // BLINK_MODE == 1 // PROBING
          if (blink_scan_start == 1) { //start timer for probing
            ENCODING_SCHEME = index_ES_LIST[0]; // FM0 for probing
            cnt_queries = 0;
            blink_timer = clock();
            blink_scan_start = 0;
            blink_n_success = 1E-4;
            blink_n_failure = 0;
          }
          double interval = ((double) (clock() - blink_timer)) / CLOCKS_PER_SEC;
          if (interval > 5.0) {
            // summarize pktloss & RSSI
            blink_pktloss_probe = blink_n_success / cnt_queries;
            blink_RSSI_probe = RSSI;

            // map link signature to optimal rate
            // To do ...
            ENCODING_SCHEME = 1;
            blink_scan_start = 1;
            BLINK_MODE = 0;
          }
        }

        if (ENCODING_SCHEME == 1)
        {
          query_bits.insert(query_bits.end(), &M_FM0[0], &M_FM0[2]);
          bulky_N = 3;
        }
        else if (ENCODING_SCHEME == 2)
        {
          query_bits.insert(query_bits.end(), &M_Miller2[0], &M_Miller2[2]);
          bulky_N = 2;
        }
        else if (ENCODING_SCHEME == 4)
        {
          query_bits.insert(query_bits.end(), &M_Miller4[0], &M_Miller4[2]);
          bulky_N = 1;
        }
        else
        {
          query_bits.insert(query_bits.end(), &M_Miller8[0], &M_Miller8[2]);
          bulky_N = 1;
        }
        // adjust other params
        TAG_BIT_D   = 1.0 * ENCODING_SCHEME/T_READER_FREQ * pow(10,6); // Duration in us
        RN16_D      = (RN16_BITS + TAG_PREAMBLE_BITS) * TAG_BIT_D;
        EPC_D         = (EPC_BITS  + TAG_PREAMBLE_BITS) * TAG_BIT_D;
        HANDLE_D = (RN16_BITS - 1  + TAG_PREAMBLE_BITS + 16) * TAG_BIT_D; // RN16 without? dummy-bit  
        READ_D = (1 + 32+ 16 + 16+ TAG_PREAMBLE_BITS ) * TAG_BIT_D; // RN16 without? dummy-bit
    
        Tsk = T1_D + RN16_D + T2_D + Tack + T1_D + EPC_D + T2_D;  //Duration of single/collision slot in us
        Ti  = T1_D + RN16_D + T2_D; //Duration of idle slot in us
    
        C_th = C_th_LIST[ENCODING_SCHEME];

        n_cwquery_s   = (T1_D+T2_D+RN16_D)/sample_d;     //RN16 ---
        n_cwack_s     = (1*T1_D+T2_D+EPC_D)/sample_d;    //EPC   if it is longer than nominal it wont cause tags to change inventoried flag ---

        std::vector<float>().swap(cw_query);
        std::vector<float>().swap(cw_ack);

        cw_query.resize(n_cwquery_s);      // Sent after query/query rep ---
        cw_ack.resize(n_cwack_s);          // Sent after ack ---

        std::fill_n(cw_query.begin(), cw_query.size(), 1); // ---
        std::fill_n(cw_ack.begin(), cw_ack.size(), 1); // ---
      }
      
      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // MobiRate
      // It is basically Minstrel/SampleRate with mobility-aware EMA
      // It starts with the lowest rate
      if (MobiRate_EN == 1) {
        if (valid_packet == 1) {
          // monitor phase changes to estimate mobility and set lambda
          float diff_phase = 0;
          if (mobirate_phase_scan_cnt == 0) { // first phase
            mobirate_phase_realtime = Phase;
            mobirate_phase_scan_cnt = 1;
            mobirate_timer1 = clock();
          }
          double interval = ((double) (clock() - mobirate_timer1)) / CLOCKS_PER_SEC; 
          if (mobirate_phase_scan_cnt == 1 && interval > 0.2) { // second scan
            // unwrap phase
            diff_phase = Phase - mobirate_phase_realtime;
            if (diff_phase > PI) {
              diff_phase -= 2 * PI;
            }
            else if (diff_phase < -PI) {
              diff_phase += 2 * PI;
            }
            // estimate velocity
            float v_est = WAVLEN * diff_phase / interval; // v = xx cm/s
            // set lambda
            if (v_est < 0.01) {
              lambda = 0.07;
            } 
            else if (v_est > 0.01 && v_est < 0.8) {
              lambda = 0.28;
            }
            else {
              lambda = 0.39;
            }
            mobirate_phase_scan_cnt = 0;
          } 
        }
        
        if (MobiRate_MODE == 1) { // Probe higher rate
          mobirate_pktloss_table[index_ES] = lambda * mobirate_pktloss_table[index_ES] + (1 - lambda) * curr_transmission_state * valid_packet;
          double interval = ((double) (clock() - mobirate_timer2)) / CLOCKS_PER_SEC;
          if (interval > 2) {
            float diff_pktloss = mobirate_pktloss_table[index_ES] - mobirate_pktloss_table[index_ES - 1];
            if (diff_pktloss < 0) { // keep original rate
              index_ES--;
              cout << "keep original rate" << endl;
            }
            MobiRate_MODE = 0; // back to keep mode
            cout << "end probing higher rate" << endl;
            mobirate_timer2 = clock(); // reset timer for keep mode
          } 
        }
        else if (MobiRate_MODE == 2) { // Probe lower rate
          mobirate_pktloss_table[index_ES] = lambda * mobirate_pktloss_table[index_ES] + (1 - lambda) * curr_transmission_state * valid_packet;
          double interval = ((double) (clock() - mobirate_timer2)) / CLOCKS_PER_SEC;
          if (interval > 2) {
            float diff_pktloss = mobirate_pktloss_table[index_ES] - mobirate_pktloss_table[index_ES + 1];
            if (diff_pktloss < 0) { // keep original rate
              index_ES++;
              cout << "keep original rate" << endl;
            }
            MobiRate_MODE = 0; // back to keep mode
            cout << "end probing lower rate" << endl;
            mobirate_timer2 = clock(); // reset timer for keep mode
          } 
        }
        else if (MobiRate_MODE == 0) { // Keep
          if (curr_transmission_state == 1 && valid_packet == 1) {
            mobirate_n_failure = 0;
          }
          mobirate_n_failure += (1 - curr_transmission_state) * valid_packet; // no valid limits
          valid_packet = 0; // reset valid_packet
          double interval = ((double) (clock() - mobirate_timer2)) / CLOCKS_PER_SEC;
          if (index_ES > 0 && mobirate_n_failure >= 2) {
            mobirate_n_failure = 0;
            mobirate_timer2 = clock(); // start timer for probing
            index_ES--;
            MobiRate_MODE = 2; // gonna probe lower rate
            cout << "probe lower rate" << endl;
          }
          else if (index_ES < 3 && interval > 2) {
            mobirate_n_failure = 0;
            mobirate_timer2 = clock(); // start timer for probing
            index_ES++;
            MobiRate_MODE = 1; // gonna probe higher rate
            cout << "probe higher rate" << endl;
          }
          else { // record packetloss of current rate
            mobirate_pktloss_table[index_ES] = lambda * mobirate_pktloss_table[index_ES] + (1 - lambda) * curr_transmission_state * valid_packet;
          }
        }
        valid_packet = 0;
        curr_transmission_state = 0; // new added

        ENCODING_SCHEME = index_ES_LIST[index_ES];
        if (ENCODING_SCHEME == 1)
        {
          query_bits.insert(query_bits.end(), &M_FM0[0], &M_FM0[2]);
          bulky_N = 3;
        }
        else if (ENCODING_SCHEME == 2)
        {
          query_bits.insert(query_bits.end(), &M_Miller2[0], &M_Miller2[2]);
          bulky_N = 2;
        }
        else if (ENCODING_SCHEME == 4)
        {
          query_bits.insert(query_bits.end(), &M_Miller4[0], &M_Miller4[2]);
          bulky_N = 1;
        }
        else
        {
          query_bits.insert(query_bits.end(), &M_Miller8[0], &M_Miller8[2]);
          bulky_N = 1;
        }
        // adjust other params
        TAG_BIT_D   = 1.0 * ENCODING_SCHEME/T_READER_FREQ * pow(10,6); // Duration in us
        RN16_D      = (RN16_BITS + TAG_PREAMBLE_BITS) * TAG_BIT_D;
        EPC_D         = (EPC_BITS  + TAG_PREAMBLE_BITS) * TAG_BIT_D;
        HANDLE_D = (RN16_BITS - 1  + TAG_PREAMBLE_BITS + 16) * TAG_BIT_D; // RN16 without? dummy-bit  
        READ_D = (1 + 32+ 16 + 16+ TAG_PREAMBLE_BITS ) * TAG_BIT_D; // RN16 without? dummy-bit
    
        Tsk = T1_D + RN16_D + T2_D + Tack + T1_D + EPC_D + T2_D;  //Duration of single/collision slot in us
        Ti  = T1_D + RN16_D + T2_D; //Duration of idle slot in us
    
        C_th = C_th_LIST[ENCODING_SCHEME];

        n_cwquery_s   = (T1_D+T2_D+RN16_D)/sample_d;     //RN16 ---
        n_cwack_s     = (1*T1_D+T2_D+EPC_D)/sample_d;    //EPC   if it is longer than nominal it wont cause tags to change inventoried flag ---

        std::vector<float>().swap(cw_query);
        std::vector<float>().swap(cw_ack);

        cw_query.resize(n_cwquery_s);      // Sent after query/query rep ---
        cw_ack.resize(n_cwack_s);          // Sent after ack ---

        std::fill_n(cw_query.begin(), cw_query.size(), 1); // ---
        std::fill_n(cw_ack.begin(), cw_ack.size(), 1); // ---
      }

      // params update for pbr
      pbr_index_ES = index_ES;
      n_queries_sent = reader_state->reader_stats.n_queries_sent;
      
      query_bits.push_back(TREXT);
      query_bits.insert(query_bits.end(), &SEL[0], &SEL[2]);
      query_bits.insert(query_bits.end(), &SESSION[0], &SESSION[2]);
      query_bits.push_back(TARGET);
      query_bits.insert(query_bits.end(), &Q_VALUE[reader_state->reader_stats.VAR_Q][0], &Q_VALUE[reader_state->reader_stats.VAR_Q][4]);
      crc_append(query_bits,17);

      
    }


    void reader_impl::gen_ack_bits(const float * data_in)
    {
      ack_bits.resize(0);
      ack_bits.insert(ack_bits.end(), &ACK_CODE[0], &ACK_CODE[2]);
      ack_bits.insert(ack_bits.end(), &data_in[0], &data_in[16]);
      
    }
  
    void reader_impl::gen_query_adjust_bits()
    {
      query_adjust_bits.resize(0);
      query_adjust_bits.insert(query_adjust_bits.end(), &QADJ_CODE[0], &QADJ_CODE[4]);
      query_adjust_bits.insert(query_adjust_bits.end(), &SESSION[0], &SESSION[2]);
      query_adjust_bits.insert(query_adjust_bits.end(), &Q_UPDN[reader_state-> reader_stats.Qupdn][0], &Q_UPDN[reader_state-> reader_stats.Qupdn][3]);
     
    }


    void reader_impl::gen_req_rn16_bits(const float * in)
    {
      req_rn16_bits.resize(0);
      req_rn16_bits.insert(req_rn16_bits.end(), &REQ_RN16_CODE[0], &REQ_RN16_CODE[8]);
      req_rn16_bits.insert(req_rn16_bits.end(), &in[0], &in[16]);
      crc16_append(req_rn16_bits,24,command_bits);
      
    }

  void reader_impl::gen_read_bits(const float * in)
    {
      read_bits.resize(0);
      read_bits.insert(read_bits.end(), &READ_CODE[0], &READ_CODE[8]);
      read_bits.insert(read_bits.end(), &MemBank[0], &MemBank[2]);
      read_bits.insert(read_bits.end(), &WordPtr[0], &WordPtr[8]);
      read_bits.insert(read_bits.end(), &Wordcount[0], &Wordcount[8]);
      read_bits.insert(read_bits.end(), &in[0], &in[16]);
      crc16_append(read_bits,42,command_bits);
      
    }

    /*
     * Our virtual destructor.
     */
    reader_impl::~reader_impl()
    {

    }


    int reader_impl::print_results()
    {       
      float pktLossRatio = 1 - 1.0 * reader_state->reader_stats.n_epc_correct / (reader_state->reader_stats.n_epc_correct + cnt_loss_epc_global);
      float aveThroughput = reader_state->reader_stats.average_throughput;
      //float aveGoodput = aveThroughput * std::pow(1 - pktLossRatio, bulky_N - 1);
      float aveGoodput = aveThroughput;
      //E_Tx = E_Tx / 0.49 * 0.1; // 
      float P_Tx = E_Tx / accumulative_d_monitor; // ave P before calibration
      /*
      float A_Tx = std::sqrt(P_Tx);
      float A_Tx_calibrated = 1E-5;
      if (A_Tx < 0.55) {
        A_Tx_calibrated = 1.50356 * (A_Tx - 0.55) + 0.78861;
      }
      else {
        A_Tx_calibrated = -1.0439 * (A_Tx - 1) * (A_Tx - 1) + 1;
      }
      P_Tx = A_Tx_calibrated * A_Tx_calibrated;
      */
      E_Tx = P_Tx * accumulative_d_monitor * 0.1;

      std::cout << "\n --------------------------" << std::endl;
      std::cout << "| Number of Queries/Queryreps Sent : " << reader_state->reader_stats.n_queries_sent  << std::endl;
      std::cout << "| Current Inventory Round : "          << reader_state->reader_stats.cur_inventory_round << std::endl;
      std::cout << " --------------------------"            << std::endl;

      // std::cout << "| Total Collision Slots : "  <<  reader_state-> reader_stats.tn_k     << std::endl;
      // std::cout << "| Total idle slots : "  <<  reader_state-> reader_stats.tn_0     << std::endl;
      std::cout << "| Number of Lost Packets : "  <<  reader_state-> reader_stats.tn_k << std::endl;
      //std::cout << "| Number of Incomplete Packets : " << reader_state-> reader_stats.n_incomplete_pkts << std::endl;
      //std::cout << "| Number of Error Packets : " << reader_state-> reader_stats.n_error_pkts << std::endl;
      std::cout << "| Total Success Slots : "  <<  reader_state-> reader_stats.tn_1     << std::endl;
      std::cout << "| Packet loss Ratio : " <<  pktLossRatio << std::endl;
      //std::cout << "| total QA : "  <<  reader_state-> reader_stats.tQA     << std::endl;
      //std::cout << "| total QR : "  <<  reader_state-> reader_stats.tQR     << std::endl;
      std::cout << " --------------------------"            << std::endl;

      std::cout << "| Correctly Decoded EPC : "  <<  reader_state->reader_stats.n_epc_correct     << std::endl;
      std::cout << "| Totally Detected EPC : " << reader_state->reader_stats.n_epc_detected << std::endl;
      std::cout << "| Number of Unique Tags : "  <<  reader_state->reader_stats.tag_reads.size() << std::endl;
      //std::cout << "| SNR : " << 10 * log10(reader_state->reader_stats.bs_power / reader_state->reader_stats.noise_power) << std::endl;
      std::cout << "| ----------------------------------------------------------------------- " <<  std::endl;
           
            //std::cout << "| TIR theoretic : "  <<  reader_state-> reader_stats.TIR_th     << std::endl;
            //std::cout << "| TIR experimental : "  <<  reader_state-> reader_stats.TIR_exp << std::endl;
      //std::cout << "| Test Throughtput : "  <<  129 * reader_state->reader_stats.n_epc_correct/(((double) (clock() - reader_state->reader_stats.start_time)) / CLOCKS_PER_SEC) << std::endl;
      std::cout << "| Average Reading Rate (reads/s) : " <<  aveThroughput << std::endl;
      std::cout << "| Average Throughput (bps) : " << aveThroughput * 32 << std::endl;
      std::cout << " --------------------------"            << std::endl;
      //std::cout << "| Goodput ref (pkts/s) : " << reader_state->reader_stats.average_throughput << std::endl;
      std::cout << "| Goodput with retransmission (pkts/s) : " << 1.0 * retran_goodput_pkt_cnt / total_time << std::endl;
      std::cout << "| Goodput with retransmission (bps) : " << 32.0 * retran_goodput_pkt_cnt / total_time << std::endl;
      std::cout << "| Bulk Seg Delay with retransmission (ms) : " <<  std::round(1000 * total_time / retran_goodput_pkt_cnt) << std::endl; // 1000 * retran_delay
      std::cout << "| Goodput (Effective Throughput) (reads/s): " << aveGoodput << std::endl; //changed
      std::cout << "| Bulky Segment Delay (ms) : " << 1000 * bulky_N * bulky_N / (aveThroughput * std::pow(1 - pktLossRatio, bulky_N - 1)) << std::endl;
      std::cout << "| DNN Inference Count : " << cnt_inference << std::endl;
      std::cout << "| FFT Count : " << cnt_fft << std::endl;
      std::cout << "| Tx Energy (J) : " << E_Tx << std::endl;
      std::cout << "| Ave Tx Power (mW) : " << P_Tx * 100 << std::endl;
      std::cout << "| BpJ (kbits/J) : " << 0.32 * retran_goodput_pkt_cnt / total_time / P_Tx << std::endl; // 4 bytes sensor data
      std::cout << " --------------------------"            << std::endl;
	    std::cout << "| Power-up Delay (s) : " << reader_state->reader_stats.power_up_delay << std::endl;
      std::cout << "| Average RSSI (dB) : " << RSSI << std::endl;
      std::cout << "| Average Noise Power (dB) : " << NoiseI << std::endl;
      std::cout << "| Average SNR (dB) : " << RSSI - NoiseI << std::endl;
      std::cout << "| Amplitude Scalar : " << rta_ampl / 0.7 << std::endl;
      std::cout << "| Equivalent Tx gain (dB) : " << 25 + 20 * log10(rta_ampl / 0.7) << std::endl;
      std::cout << "| Carrier Wave Amplitude : " << cw_ampl << std::endl;
	    std::cout << "| ----------------------------------------------------------------------- " <<  std::endl;
            
      std::map<int,int>::iterator it;
/*
      for(it = reader_state->reader_stats.tag_reads.begin(); it != reader_state->reader_stats.tag_reads.end(); it++) 
      {
        std::cout << std::hex <<  "| Tag ID : " << it->first << "  ";
        std::cout << "Num of reads : " << std::dec << it->second << std::endl;
      }
*/

      if (0) {
        ofstream f1("TP_TEST/WISP_tp_record.txt", ios::app);
        for (float tp : tp_record) {
          f1 << tp << endl;
        }
        f1.close();

        ofstream f2("TP_TEST/WISP_t_record.txt", ios::app);
        for (float t : t_record) {
          f2 << t << endl;
        }
        f2.close();

        ofstream f3("TP_TEST/WISP_p_record.txt", ios::app);
        for (float p : p_record) {
          f3 << p << endl;
        }
        f3.close();
      }

      if (1) {
        ofstream f2("RFID_localization/reader_4_t.txt", ios::app);
        for (float t : t_preamble_record) {
          f2 << t << endl;
        }
        f2.close();
      }

      if (1) {
        ofstream f4("RFID_localization/reader_4_signal.txt", ios::app);
        for (std::complex<float> t : preamble_fm0) {
          f4 << std::real(t) << endl;
          f4 << std::imag(t) << endl;
        }
        f4.close();
      }

      if (DATA_COLLECTION_EN == 1) {
        
        // Report of Probing
        float mtp = 0;
        int mtp_i = 0;
        const char *eslist[4] = {"FM0", "M2", "M4", "M8"};
        for (int i = 0; i < 4; i++) {
          // adabs_goodput_table[i] = std::pow(1 - adabs_lossrate_table[i], bulky_N - 1) * adabs_throughput_table[i];
          adabs_goodput_table[i] = adabs_throughput_table[i];
          if (adabs_goodput_table[i] > mtp) {
            mtp = adabs_goodput_table[i];
            mtp_i = i;
          }
        }
        std::cout << "\n --------------------------" << std::endl;
        std::cout << "| Probing Summary " << std::endl;
        std::cout << "| Channel Condition Metrics (FM0, AMP=1) " << std::endl;
        std::cout << "| RSSI (dB) : " << adabs_probe_RSSI << std::endl;
        std::cout << "| Noise Power (dB) : " << adabs_probe_NoiseI << std::endl;
        std::cout << "| Power-up Delay (s) : " << adabs_probe_powerup_delay << std::endl;
        std::cout << " --------------------------" << std::endl;
        std::cout << "| Throughput (reads/s) & LossRate & Goodput (reads/s)" << std::endl;
        std::cout << "| FM0 : " << setw(10) << adabs_throughput_table[0] << setw(2) << " | " << adabs_lossrate_table[0] << setw(2) << " | " << adabs_goodput_table[0] << std::endl;
        std::cout << "| M2  : " << setw(10) << adabs_throughput_table[1] << setw(2) << " | " << adabs_lossrate_table[1] << setw(2) << " | " << adabs_goodput_table[1] << std::endl;
        std::cout << "| M4  : " << setw(10) << adabs_throughput_table[2] << setw(2) << " | " << adabs_lossrate_table[2] << setw(2) << " | " << adabs_goodput_table[2] << std::endl;
        std::cout << "| M8  : " << setw(10) << adabs_throughput_table[3] << setw(2) << " | " << adabs_lossrate_table[3] << setw(2) << " | " << adabs_goodput_table[3] << std::endl;
        
        std::cout << "| Max Goodput (reads/s) : " << mtp << std::endl;
        std::cout << "| Encoding Scheme : " << eslist[mtp_i] << std::endl;
        
        // save data sample
        ofstream fout("Data_FinalTry/sample_" + std::to_string(AMP) + "_" + std::to_string(CCI) + "_" +  std::to_string(SI) + ".txt", ios::app);
        for (int i = 0; i < 112; i++) {
          fout << Hft[i] << endl;
        }
        fout << adabs_probe_RSSI << endl;
        fout << adabs_probe_NoiseI << endl;
        fout << adabs_probe_powerup_delay << endl;
        fout << mtp_i << endl;
        fout << rta_ampl / 0.7 << endl;
        fout << mtp << endl;

        fout.close();
        

      }

      
    return reader_state->reader_stats.start_time;       
    }

    void
    reader_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = 0;
    }

    int
    reader_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {

      const float *in = (const float *) input_items[0];
      float *out =  (float*) output_items[0];
      std::vector<float> out_message; 
      int n_output;
      int consumed = 0;
      int written = 0;

      consumed = ninput_items[0];
  
      switch (reader_state->gen2_logic_status)
      {
	      case POWER_UP_CW:
	        reader_state->reader_stats.n_powerup_sent +=1;
	        memcpy(&out[written], &cw_start[0], sizeof(float) * cw_start.size() );
          written += cw_start.size();

	        if (reader_state-> reader_stats.n_powerup_sent <30){
            reader_state->gen2_logic_status = POWER_UP_CW;
          }
          else {
            reader_state->gen2_logic_status = START;
          }
          break;


        case START:

          reader_state->reader_stats.n_queries_sent +=1;

          GR_LOG_INFO(d_debug_logger, "START");
          memcpy(&out[written], &cw_start[0], sizeof(float) * cw_start.size() );
          written += cw_start.size();

          if (reader_state-> reader_stats.n_queries_sent <10){
            reader_state->gen2_logic_status = START;    
          }
          else {		  
            reader_state->gen2_logic_status = SEND_QUERY;    
          }        
          break;

        case POWER_DOWN:
          GR_LOG_INFO(d_debug_logger, "POWER DOWN");
          memcpy(&out[written], &p_down[0], sizeof(float) * p_down.size() );
          written += p_down.size();
          reader_state->gen2_logic_status = START;    
          break;

        case SEND_NAK_QR:
          GR_LOG_INFO(d_debug_logger, "SEND NAK");
          memcpy(&out[written], &nak[0], sizeof(float) * nak.size() );
          written += nak.size();
          memcpy(&out[written], &cw[0], sizeof(float) * cw.size() );
          written+=cw.size();
          reader_state->gen2_logic_status = SEND_QUERY_REP;    
          break;

        case SEND_NAK_Q:

          GR_LOG_INFO(d_debug_logger, "SEND NAK");
          memcpy(&out[written], &nak[0], sizeof(float) * nak.size() );
          written += nak.size();
          memcpy(&out[written], &cw[0], sizeof(float) * cw.size() );
          written+=cw.size();
          reader_state->gen2_logic_status = SEND_QUERY;    
          break;

        case SEND_QUERY:

        //Start timer 
        gettimeofday (&reader_state-> reader_stats.start, NULL);//start timer
        
        reader_state-> reader_stats.tQ += 1;
        cnt_queries++;
        //std::cout << " **********     SEND QUERY       *************** " << std::endl;

        gen_query_bits();

        reader_state->reader_stats.n_queries_sent +=1;  

          // Controls the other two blocks
          reader_state->decoder_status = DECODER_DECODE_RN16;
          reader_state->gate_status    = GATE_SEEK_RN16;

          decoder_status = PBR_DECODER_DECODE_RN16;
          gate_status    = PBR_GATE_SEEK_RN16;

          memcpy(&out[written], &preamble[0], sizeof(float) * preamble.size() );
          written+=preamble.size();
   
          for(int i = 0; i < query_bits.size(); i++)
          {
            if(query_bits[i] == 1)
            {
              memcpy(&out[written], &data_1[0], sizeof(float) * data_1.size() );
              written+=data_1.size();
              
            }
            else
            {
              memcpy(&out[written], &data_0[0], sizeof(float) * data_0.size() );
              written+=data_0.size();

            }
          }
          // Send CW for RN16
          reader_state->gen2_logic_status = SEND_CW_QUERY; 

          // Return to IDLE
          //reader_state->gen2_logic_status = IDLE;      
          break;

        case SEND_ACK:

        
        //std::cout <<  "SEND ACK" << std::endl;

          GR_LOG_INFO(d_debug_logger, "SEND ACK");
          if (ninput_items[0] == RN16_BITS - 1)
          {

            // Controls the other two blocks
            reader_state->decoder_status = DECODER_DECODE_EPC;
            reader_state->gate_status    = GATE_SEEK_EPC;

            decoder_status = PBR_DECODER_DECODE_EPC;
            gate_status    = PBR_GATE_SEEK_EPC;

            gen_ack_bits(in); // this should be replaced by assigning stored rn16
            
            // Send FrameSync
            memcpy(&out[written], &frame_sync[0], sizeof(float) * frame_sync.size() );
            written += frame_sync.size();


           for(int i = 0; i < ack_bits.size(); i++)
            {
              if(ack_bits[i] == 1)
              {
                memcpy(&out[written], &data_1[0], sizeof(float) * data_1.size() );
                written += data_1.size();
                //std::cout << "1 ";
              }
              else  
              {
                memcpy(&out[written], &data_0[0], sizeof(float) * data_0.size() );
                written += data_0.size();
                //std::cout << "0 ";
              }
              //std::cout << endl;
            }

            
            consumed = ninput_items[0];
            reader_state->gen2_logic_status = SEND_CW_ACK; 
            
          }

          break;

        case SEND_CW_ACK:

          GR_LOG_INFO(d_debug_logger, "SEND CW - ack");
          memcpy(&out[written], &cw_ack[0], sizeof(float) * cw_ack.size() );
          written += cw_ack.size();
          reader_state->gen2_logic_status = IDLE;      // Return to IDLE
          break;

        case SEND_CW_QUERY:

          GR_LOG_INFO(d_debug_logger, "SEND CW - query");
          memcpy(&out[written], &cw_query[0], sizeof(float) * cw_query.size() );
          written+=cw_query.size();
          reader_state->gen2_logic_status = IDLE;      // Return to IDLE
          break;


        case SEND_CW_REQ:

          memcpy(&out[written], &cw_req_rn16[0], sizeof(float) * cw_req_rn16.size() );
          written += cw_req_rn16.size();
          reader_state->gen2_logic_status = IDLE;      // Return to IDLE
          break;
        
        case SEND_CW_READ:

          memcpy(&out[written], &cw_read[0], sizeof(float) * cw_read.size() );
          written += cw_read.size();
          reader_state->gen2_logic_status = IDLE;      // Return to IDLE
          break;


        case SEND_QUERY_REP:

          reader_state-> reader_stats.tQR += 1;

          GR_LOG_INFO(d_debug_logger, "INVENTORY ROUND : " + std::to_string(reader_state->reader_stats.cur_inventory_round) + " SLOT NUMBER : " + std::to_string(reader_state->reader_stats.cur_slot_number));
          
          // Controls the other two blocks
          reader_state->decoder_status = DECODER_DECODE_RN16;
          reader_state->gate_status    = GATE_SEEK_RN16;
          reader_state->reader_stats.n_queries_sent +=1;  

          memcpy(&out[written], &query_rep[0], sizeof(float) * query_rep.size() );
          written += query_rep.size();
          reader_state->gen2_logic_status = SEND_CW_QUERY; 
          break;
      
        case SEND_QUERY_ADJUST:

          reader_state-> reader_stats.tQA += 1;

          gen_query_adjust_bits();

          GR_LOG_INFO(d_debug_logger, "SEND QUERY_ADJUST");
          
          // Controls the other two blocks
          reader_state->decoder_status = DECODER_DECODE_RN16;
          reader_state->gate_status    = GATE_SEEK_RN16;
          reader_state->reader_stats.n_queries_sent +=1;  

          memcpy(&out[written], &frame_sync[0], sizeof(float) * frame_sync.size() );
          written += frame_sync.size();

          for(int i = 0; i < query_adjust_bits.size(); i++)
          {
             
            if(query_adjust_bits[i] == 1)
            {
              memcpy(&out[written], &data_1[0], sizeof(float) * data_1.size() );
              written+=data_1.size();
            }
            else
            {
              memcpy(&out[written], &data_0[0], sizeof(float) * data_0.size() );
              written+=data_0.size();
            }
          }

          reader_state->gen2_logic_status = SEND_CW_QUERY; 
          break;


//-----------------------------------------------------------------------------------------------------

    case SEND_REQ_RN16:      
            std::cout << " SEND REQUEST HANDLE" << std::endl;

          reader_state->decoder_status = DECODER_DECODE_HANDLE;
          reader_state->gate_status    = GATE_SEEK_HANDLE;
        
          //Transmit: command + RN16 + CRC
           gen_req_rn16_bits(in);
          
           // Send FrameSync
            memcpy(&out[written], &frame_sync[0], sizeof(float) * frame_sync.size() );
            written += frame_sync.size();


           for(int i = 0; i < req_rn16_bits.size(); i++)
            {
              if(req_rn16_bits[i] == 1)
              {
                memcpy(&out[written], &data_1[0], sizeof(float) * data_1.size() );
                written += data_1.size();
                 
              }
              else  
              {
                memcpy(&out[written], &data_0[0], sizeof(float) * data_0.size() );
                written += data_0.size();
         
              }
            }

            consumed = ninput_items[0];
            reader_state->gen2_logic_status = SEND_CW_REQ; 
        
          break;

//-----------------------------------------------------------------------------------------------------
    case SEND_READ:      
         
          reader_state->decoder_status = DECODER_DECODE_READ;
          reader_state->gate_status    = GATE_SEEK_READ;
          reader_state->reader_stats.n_queries_sent +=1; 


          //Transmit: command + MenmBank + WordPtr + WordCount + RN + CRC16
           gen_read_bits(in);
          
           // Send FrameSync
            memcpy(&out[written], &frame_sync[0], sizeof(float) * frame_sync.size() );
            written += frame_sync.size();


           for(int i = 0; i < read_bits.size(); i++)
            {
              if(read_bits[i] == 1)
              {
                memcpy(&out[written], &data_1[0], sizeof(float) * data_1.size() );
                written += data_1.size();
                 
              }
              else  
              {
                memcpy(&out[written], &data_0[0], sizeof(float) * data_0.size() );
                written += data_0.size();
         
              }
            }


            consumed = ninput_items[0];
            reader_state->gen2_logic_status = SEND_CW_READ; 

          break;


 //-----------------------------------------------------------------------------------------------------

        default:
          // IDLE
        
          break;
      }
      consume_each (consumed);
      return  written;
    }

    /* Function adapted from https://www.cgran.org/wiki/Gen2 */
    void reader_impl::crc_append(std::vector<float> & q, int num_bits)
    {
      
       int crc[] = {1,0,0,1,0};

      for(int i = 0; i < num_bits; i++) //17 because length of query is 17+CRC_5
      {
        int tmp[] = {0,0,0,0,0};
        tmp[4] = crc[3];
        if(crc[4] == 1)
        {
          if (q[i] == 1)
          {
            tmp[0] = 0;
            tmp[1] = crc[0];
            tmp[2] = crc[1];
            tmp[3] = crc[2];
          }
          else
          {
            tmp[0] = 1;
            tmp[1] = crc[0];
            tmp[2] = crc[1];
            if(crc[2] == 1)
            {
              tmp[3] = 0;
            }
            else
            {
              tmp[3] = 1;
            }
          }
        }
        else
        {
          if (q[i] == 1)
          {
            tmp[0] = 1;
            tmp[1] = crc[0];
            tmp[2] = crc[1];
            if(crc[2] == 1)
            {
              tmp[3] = 0;
            }
            else
            {
              tmp[3] = 1;
            }
          }
          else
          {
            tmp[0] = 0;
            tmp[1] = crc[0];
            tmp[2] = crc[1];
            tmp[3] = crc[2];
          }
        }
        memcpy(crc, tmp, 5*sizeof(float));
      }
      for (int i = 4; i >= 0; i--)
        q.push_back(crc[i]);
    }



    void reader_impl::crc16_append(std::vector<float> & q, int num_bits,char * bits_command)
    {
      register unsigned short i, j;
      register unsigned short crc_16;
      unsigned char * data;
      //unsigned short crc16_short;
      int num_bytes = num_bits / 8;
      data = (unsigned char* )malloc(num_bytes );

      int mask;
      
      //--------------------------------------------------------------
      
      for (int i =0; i < num_bits; i ++)
        {
        if (q[i] == 0)
           bits_command[i] = '0';
        else
          bits_command[i] = '1';
        }

      //--------------------------------------------------------------     

      for(i = 0; i < num_bytes; i++)
      {
        mask = 0x80;
        data[i] = 0;
        for(j = 0; j < 8; j++)
        {
          if (bits_command[(i * 8) + j] == '1'){
          data[i] = data[i] | mask;
        }
        mask = mask >> 1;
        }
      }
      //--------------------------------------------------------------
      crc_16 = 0xFFFF; 
      for (i=0; i < num_bytes; i++)
      {
        crc_16^=data[i] << 8;
        for (j=0;j<8;j++)
        {
          if (crc_16&0x8000)
          {
            crc_16 <<= 1;
            crc_16 ^= 0x1021;
          }
          else
            crc_16 <<= 1;
        }
      }
      crc_16 = ~crc_16;
    
    bitset<16> crc16_string(crc_16); 


    for (int i = 15; i >= 0; i--)
        q.push_back(crc16_string[i]);
    
    } //End of crc16 append function

    

  } /* namespace rfid */
} /* namespace gr */

