/* -*- c++ -*- */
/* 
 * Copyright 2020 <Kai Huang (k.huang[AT]pitt.edu)>.
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
#include "pbr_gate_impl.h"
#include <sys/time.h>
#include <iostream>
#include <fstream>
using namespace std;

namespace gr {
  namespace rfid {

    pbr_gate::sptr
    pbr_gate::make(int sample_rate)
    {
      return gnuradio::get_initial_sptr
        (new pbr_gate_impl(sample_rate));
    }

    /*
     * The private constructor
     */
    pbr_gate_impl::pbr_gate_impl(int sample_rate)
      : gr::block("pbr_gate",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
              n_samples(0), win_index(0), dc_index(0), num_pulses(0), signal_state(NEG_EDGE), avg_ampl(0), dc_est(0,0)
    {
      sp_rate = sample_rate;
      n_samples_T1       = T1_D       * (sample_rate / pow(10,6));
      n_samples_PW       = PW_D       * (sample_rate / pow(10,6));
      n_samples_TAG_BIT  = PBR_TAG_BIT_D * (sample_rate / pow(10,6));
      
      win_length = WIN_SIZE_D * (sample_rate/ pow(10,6));
      dc_length  = DC_SIZE_D  * (sample_rate / pow(10,6));

      win_samples.resize(win_length);
      dc_samples.resize(dc_length);

      //set_min_noutput_items(1000);


      // GR_LOG_INFO(d_logger, "Samples of Tag bit : "<< n_samples_TAG_BIT);
      // GR_LOG_INFO(d_logger, "Size of window : " << win_length);
      


      // First block to be scheduled
      //GR_LOG_INFO(d_logger, "Initializing reader state...");
      // initialize_reader_state();
    }

    /*
     * Our virtual destructor.
     */
    pbr_gate_impl::~pbr_gate_impl()
    {
    }

    void
    pbr_gate_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = noutput_items;
    }

    int
    pbr_gate_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {

      // updata timing & other params
      PBR_ENCODING_SCHEME = index_ES_LIST[pbr_index_ES]; // 1/2/4/8 --> FM0/M2/M4/M8
      PBR_E_th = E_th_LIST[PBR_ENCODING_SCHEME];
      PBR_TAG_BIT_D   = 1.0 * PBR_ENCODING_SCHEME/T_READER_FREQ * pow(10,6); // Duration in us
      PBR_RN16_D      = (RN16_BITS + TAG_PREAMBLE_BITS) * PBR_TAG_BIT_D;
      PBR_EPC_D         = (EPC_BITS  + TAG_PREAMBLE_BITS) * PBR_TAG_BIT_D;
      PBR_HANDLE_D = (RN16_BITS - 1  + TAG_PREAMBLE_BITS + 16) * PBR_TAG_BIT_D; // RN16 without? dummy-bit  
      PBR_READ_D = (1 + 32+ 16 + 16+ TAG_PREAMBLE_BITS ) * PBR_TAG_BIT_D; // RN16 without? dummy-bit



      n_samples_TAG_BIT  = PBR_TAG_BIT_D * (sp_rate / pow(10,6));
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];

      int n_items = ninput_items[0];
      int number_samples_consumed = n_items;
      float sample_ampl = 0;
      int written = 0;

      if ( n_queries_sent  > MAX_NUM_QUERIES && status != PBR_TERMINATED)
      {
        status = PBR_TERMINATED;
      }

      // Gate block is controlled by the Gen2 Logic block
      // We only need preamble, but we also need to make sure it is from EPC, not RN16. And we must know ENCODING SCHEME beacuse
      // different ENCODING SCHEMES have different PREAMBLES
      if(gate_status == PBR_GATE_SEEK_EPC)
      {
        if (PBR_ENCODING_SCHEME == 8) {
          gate_status = PBR_GATE_CLOSED;
          // we only need preamble right?
          n_samples_to_ungate = ((EPC_BITS + TAG_PREAMBLE_BITS) * n_samples_TAG_BIT + 10*n_samples_TAG_BIT) / 3;
          n_samples = 0;
        }
        else {
          gate_status = PBR_GATE_CLOSED;
          n_samples_to_ungate = (EPC_BITS + TAG_PREAMBLE_BITS) * n_samples_TAG_BIT + 10*n_samples_TAG_BIT;
          n_samples = 0;
        }
        
      }
      else if (gate_status == PBR_GATE_SEEK_RN16)
      { 
        gate_status = PBR_GATE_CLOSED;
        n_samples_to_ungate = (RN16_BITS + TAG_PREAMBLE_BITS) * n_samples_TAG_BIT + 10*n_samples_TAG_BIT;
        n_samples = 0;
      }
      else if (gate_status == PBR_GATE_SEEK_HANDLE)
      { 
        gate_status = PBR_GATE_CLOSED;
        n_samples_to_ungate = (RN16_BITS  + TAG_PREAMBLE_BITS + 16) * n_samples_TAG_BIT + 4*n_samples_TAG_BIT;
        n_samples = 0;
      }

    else if (gate_status == PBR_GATE_SEEK_READ)
      { 
        gate_status = PBR_GATE_CLOSED;
        n_samples_to_ungate = (32 + RN16_BITS  + TAG_PREAMBLE_BITS + 16) * n_samples_TAG_BIT + 4*n_samples_TAG_BIT;
        n_samples = 0;
      }
      
      if (status == PBR_RUNNING)
      {
        for(int i = 0; i < n_items; i++)
        {
          // Tracking average amplitude
          sample_ampl = std::abs(in[i]);
          avg_ampl = avg_ampl + (sample_ampl - win_samples[win_index])/win_length;  
          win_samples[win_index] = sample_ampl; 
          win_index = (win_index + 1) % win_length;
  
          //Threshold for detecting negative/positive edges
          sample_thresh = avg_ampl * THRESH_FRACTION;  

          if( !(gate_status == PBR_GATE_OPEN) )
          {
            //Tracking DC offset (only during T1)
            dc_est =  dc_est + (in[i] - dc_samples[dc_index])/std::complex<float>(dc_length,0);  
            dc_samples[dc_index] = in[i]; 
            dc_index = (dc_index + 1) % dc_length;
            
            n_samples++;

            // Potitive edge -> Negative edge
            if( sample_ampl < sample_thresh && signal_state == POS_EDGE)
            {
              n_samples = 0;
              signal_state = NEG_EDGE;
            }
            // Negative edge -> Positive edge 
            else if (sample_ampl > sample_thresh && signal_state == NEG_EDGE)
            {
              signal_state = POS_EDGE;
              if (n_samples > n_samples_PW/2)
                num_pulses++; 
              else
                num_pulses = 0; 
              n_samples = 0;
            }

            if(n_samples > n_samples_T1 && signal_state == POS_EDGE && num_pulses > NUM_PULSES_COMMAND)
            {
              
              //GR_LOG_INFO(d_logger, "READER COMMAND DETECTED");
              gate_status = PBR_GATE_OPEN;

              //reader_state->magn_squared_samples.resize(0);

              //reader_state->magn_squared_samples.push_back(std::norm(in[i] - dc_est));
              gr_complex temp = gr_complex((1 - abs(dc_est) / abs(in[i])), 0) * in[i];
              out[written] = temp;  
              written++;

              num_pulses = 0; 
              n_samples =  1; // Count number of samples passed to the next block
              
            }
            
          }
          else
          {
            
            n_samples++;

            //reader_state->magn_squared_samples.push_back(std::norm(in[i] - dc_est));
            gr_complex temp = gr_complex((1 - abs(dc_est) / abs(in[i])), 0) * in[i];
            out[written] = temp; // Remove offset from complex samples           
            written++;
            if (n_samples >= n_samples_to_ungate)
            {
          
              gate_status = PBR_GATE_CLOSED;    
              number_samples_consumed = i+1;
              break;
            
            }
          }
        }
      }
      consume_each (number_samples_consumed);
      // cout << "written: " << written << endl;
      // Tell runtime system how many output items we produced.
      return written;
    }

  } /* namespace rfid */
} /* namespace gr */

