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
#include <gnuradio/prefs.h>
#include <gnuradio/math.h>
#include <cmath>
#include <sys/time.h>
#include "pbr_feature_extractor_impl.h"
#include <iostream>
#include <fstream>

namespace gr {
  namespace rfid {

    pbr_feature_extractor::sptr
    pbr_feature_extractor::make(int sample_rate)
    {
      return gnuradio::get_initial_sptr
        (new pbr_feature_extractor_impl(sample_rate));
    }

    /*
     * The private constructor
     */
    pbr_feature_extractor_impl::pbr_feature_extractor_impl(int sample_rate)
      : gr::sync_block("pbr_feature_extractor",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(0, 0, 0))
    {
      //char_bits = (char *) malloc( sizeof(char) * 128);
      //char_bits_HANDLE = (char *) malloc( sizeof(char) * 32);

      n_samples_TAG_BIT = 14;  
      //n_samples_TAG_BIT = TAG_BIT_D * s_rate / pow(10,6);      
    }

    /*
     * Our virtual destructor.
     */
    pbr_feature_extractor_impl::~pbr_feature_extractor_impl()
    {
    }
    
    int pbr_feature_extractor_impl::tag_sync(const gr_complex * in , int size, int flag)
    {
      int max_index = 0;
      float max = 0,corr,cc;
      gr_complex corr2;
      
      if (flag == 1){ // FM0 encoding
        //n_samples_TAG_BIT = 14;
        // Do not have to check entire vector (not optimal)
        for (int i=0; i < 8 * n_samples_TAG_BIT ; i++)
        {
          corr2 = gr_complex(0,0);
          corr = 0;
	        cc = 0;

          // sync after matched filter (equivalent)
          for (int j = 0; j < FM0_PREAMBLE_LEN; j ++)
          {
            corr2 = corr2 + in[ (int) (i+j*n_samples_TAG_BIT/2) ] * gr_complex(TAG_PREAMBLE_FM0[j],0);
	          cc = cc + std::norm(in[(int) (i+j*n_samples_TAG_BIT/2)]);
          }
          corr = std::norm(corr2) / (FM0_PREAMBLE_POWER * cc);
          if (corr > max)
          {
            max = corr;
            max_index = i;
          }
        }  

        output_energy = max;
        //GR_LOG_INFO(d_logger, " Energy of received signal when RN16: " << reader_state->reader_stats.output_energy);

        // Preamble ({1,1,-1,1,-1,-1,1,-1,-1,-1,1,1} 1 2 4 7 11 12)) 
        h_est = (in[max_index] + in[ (int) (max_index + n_samples_TAG_BIT/2) ] + in[ (int) (max_index + 3*n_samples_TAG_BIT/2) ] + in[ (int) (max_index + 6*n_samples_TAG_BIT/2)] + in[(int) (max_index + 10*n_samples_TAG_BIT/2) ] + in[ (int) (max_index + 11*n_samples_TAG_BIT/2)])/std::complex<float>(6,0);  
        // h_est = corr2 / std::complex<float>(FM0_PREAMBLE_POWER,0);
        // Shifted received waveform by n_samples_TAG_BIT/2
        max_index = max_index + FM0_PREAMBLE_LEN * n_samples_TAG_BIT / 2 + n_samples_TAG_BIT/2; 

      }
 
      else if(flag == 2) //M2 encoding
      {
        // n_samples_TAG_BIT = 14; 
        gr_complex corr2_f = gr_complex(0,0);
        // Do not have to check entire vector (not optimal)
        for (int i=0; i < 16  * n_samples_TAG_BIT ; i++)
        {

          corr2 = gr_complex(0,0);
          corr = 0;
          cc = 0;

          // sync after matched filter (equivalent)
          for (int j = 0; j < M2_PREAMBLE_LEN; j ++)
          {
            corr2 = corr2 + in[ (int) (i+j*n_samples_TAG_BIT/2) ] * gr_complex(TAG_PREAMBLE_M2[j],0);
            cc = cc + std::norm(in[(int) (i+j*n_samples_TAG_BIT/2)]);
          }
          corr = std::norm(corr2) / (M2_PREAMBLE_POWER * cc);
          if (corr > max)
          {
            max = corr;
            max_index = i;
            corr2_f = corr2;
          }

        }

        output_energy = max;

        h_est = corr2_f / std::complex<float>(M2_PREAMBLE_POWER,0);

        // Shifted received waveform by n_samples_TAG_BIT/2
        max_index = max_index + M2_PREAMBLE_LEN * n_samples_TAG_BIT / 2;   

      }

      else if(flag == 4) //M4 encoding
      {
        // n_samples_TAG_BIT = 14; 
        gr_complex corr2_f = gr_complex(0,0);
        // Do not have to check entire vector (not optimal)
        for (int i=0; i < 32 * n_samples_TAG_BIT ; i++)
        {

          corr2 = gr_complex(0,0);
          corr = 0;
          cc = 0;

          // sync after matched filter (equivalent)
          for (int j = 0; j < M4_PREAMBLE_LEN; j ++)
          {
            corr2 = corr2 + in[ (int) (i+j*n_samples_TAG_BIT/2) ] * gr_complex(TAG_PREAMBLE_M4[j],0);
            cc = cc + std::norm(in[(int) (i+j*n_samples_TAG_BIT/2)]);
          }
          corr = std::norm(corr2) / (M4_PREAMBLE_POWER * cc);
          if (corr > max)
          {
            max = corr;
            max_index = i;
            corr2_f = corr2;
          }

        }

        output_energy = max;

        h_est = corr2_f / std::complex<float>(M4_PREAMBLE_POWER,0);

        // Shifted received waveform by n_samples_TAG_BIT/2
        max_index = max_index + M4_PREAMBLE_LEN * n_samples_TAG_BIT / 2;   
      }

      else if(flag == 8) //M8 encoding
      {
        // n_samples_TAG_BIT = 14; 
        gr_complex corr2_f = gr_complex(0,0);
        // Do not have to check entire vector (not optimal)
        for (int i=0; i < 32 * n_samples_TAG_BIT ; i++)
        {

          corr2 = gr_complex(0,0);
          corr = 0;
          cc = 0;
          // sync after matched filter (equivalent)
          
          for (int j = 0; j < M8_PREAMBLE_LEN; j ++)
          {
            corr2 = corr2 + in[ (int) (i+j*n_samples_TAG_BIT/2) ] * gr_complex(TAG_PREAMBLE_M8[j],0);
            cc = cc + std::norm(in[(int) (i+j*n_samples_TAG_BIT/2)]);
              
                 
	    //cout << "[ " << in[(int) (i+j*n_samples_TAG_BIT/2)] << " ] " << endl;
          }
          corr = std::norm(corr2) / (M8_PREAMBLE_POWER * cc);
          if (corr > max)
          {
            max = corr;
            max_index = i;
            corr2_f = corr2;
          }
          
        }
        // cout << "max_index: " << max_index << endl;
        output_energy = max;
        //cout << "max: " << max << endl;
        h_est = corr2_f / std::complex<float>(M8_PREAMBLE_POWER,0);
 
        // Shifted received waveform by n_samples_TAG_BIT/2
        max_index = max_index + M8_PREAMBLE_LEN * n_samples_TAG_BIT / 2;   
      }
      /*
      if (max > 0.81) {
        cout << "corr: " << max << endl;
      }
      */
      return max_index;  
    }

    int
    pbr_feature_extractor_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const  gr_complex *) input_items[0];
      // float *out = (float *) output_items[0];
      // gr_complex *out_2 = (gr_complex *) output_items[1]; // for debugging
      
      //int written_sync =0;
      //int written = 0, consumed = 0;
      int RN16_index, HANDLE_index, READ_index;
      int EPC_index;
      
      //std::vector<float> RN16_samples_real;
      //std::vector<float> EPC_samples_real;
      //std::vector<float> HANDLE_samples_real;
      //std::vector<float> READ_samples_real;

      //std::vector<gr_complex> RN16_samples_complex;
      // std::vector<gr_complex> EPC_samples_complex;
      //std::vector<gr_complex> HANDLE_samples_complex;
      //std::vector<gr_complex> READ_samples_complex;
      //std::vector<float> RN16_bits;

      //int number_of_half_bits = 0;
      //int var = 0;
      //int SW = 0;
      //int delta_Q = 0;

      //std::vector<float> EPC_bits;  
      //std::vector<float> HANDLE_bits;  
      //std::vector<float> READ_bits; 
      // std::cout << "n: " << input_items.size() << std::endl;
      // Processing only after n_samples_to_ungate are available and we need to decode an RN16
      if (decoder_status == PBR_DECODER_DECODE_RN16 && noutput_items > 58)
      {   

        //RN16_index = tag_sync(in,noutput_items,PBR_ENCODING_SCHEME);
       
        if (output_energy >= PBR_E_th)
        {
         // a valid preamble of RN16 detected
         //std::cout << "corr: " << output_energy << std::endl; 
         //std::cout << "Preamble of RN16 Detected!" << std::endl;
         //std::cout << "ampl: " << std::abs(h_est) << std::endl;
        } 
        	
        //out_2[written_sync] = 2; 
        //written_sync ++; 
        //produce(1,written_sync);
              
      }
      
      else if (decoder_status == PBR_DECODER_DECODE_EPC && noutput_items > 58)
      {
        //cout << "stage 1" << endl;
        
        //EPC_index = tag_sync(in,noutput_items,PBR_ENCODING_SCHEME);

        if (output_energy >= PBR_E_th)
        {
         // a valid preamble of EPC detected
         // std::cout << "corr: " << output_energy << std::endl; 
         //std::cout << "Preamble of EPC Detected!" << std::endl;
         //std::cout << "ampl: " << std::abs(h_est) << std::endl;
        }

      }

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace rfid */
} /* namespace gr */

