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
#include <gnuradio/prefs.h>
#include <gnuradio/math.h>
#include <cmath>
#include <sys/time.h>
#include "tag_decoder_impl.h"
#include <iostream>
#include <fstream>

using namespace std;

namespace gr {
  namespace rfid {

    tag_decoder::sptr
    tag_decoder::make(int sample_rate)
    {

      std::vector<int> output_sizes;
      output_sizes.push_back(sizeof(float));
      output_sizes.push_back(sizeof(gr_complex));
      output_sizes.push_back(sizeof(gr_complex));

      return gnuradio::get_initial_sptr
        (new tag_decoder_impl(sample_rate,output_sizes));
    }

    /*
     * The private constructor
     */
    tag_decoder_impl::tag_decoder_impl(int sample_rate, std::vector<int> output_sizes)
      : gr::block("tag_decoder",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::makev(3, 3, output_sizes )),
              s_rate(sample_rate)
    {


      char_bits = (char *) malloc( sizeof(char) * 40);
      char_bits_HANDLE = (char *) malloc( sizeof(char) * 32);

       n_samples_TAG_BIT = 14;
      //n_samples_TAG_BIT = TAG_BIT_D * s_rate / pow(10,6);      
      clock_gettime(CLOCK_MONOTONIC, &previous_time); 
    }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      

    /*
     * Our virtual destructor.
     */
    tag_decoder_impl::~tag_decoder_impl()
    {

    }

    void
    tag_decoder_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
        ninput_items_required[0] = noutput_items;
    }

    int tag_decoder_impl::tag_sync(const gr_complex * in , int size, int flag)
    {
      int max_index = 0;
      float max = 0,corr,cc;
      gr_complex corr2;
      sig_power = 0;
      gr_complex energy;
      
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
            energy = corr2;
          }
        }  

        reader_state->reader_stats.output_energy = max;
        //GR_LOG_INFO(d_logger, " Energy of received signal when RN16: " << reader_state->reader_stats.output_energy);

        // Preamble ({1,1,-1,1,-1,-1,1,-1,-1,-1,1,1} 1 2 4 7 11 12)) 
        h_est = corr2 / std::complex<float>(FM0_PREAMBLE_POWER,0);
        // Shifted received waveform by n_samples_TAG_BIT/2
        preamble_fm0_start = max_index;
        max_index = max_index + FM0_PREAMBLE_LEN * n_samples_TAG_BIT / 2 + n_samples_TAG_BIT/2;
        sig_power = abs(energy);

      }
 
      else if(flag == 2) //M2 encoding
      {
        
        // Preamble detection by cross correlation
        // n_samples_TAG_BIT = 14; 
        gr_complex corr2_f = gr_complex(0,0);
        // Do not have to check entire vector (not optimal)
        for (int i=0; i < 12 * n_samples_TAG_BIT; i++)
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
            energy = corr2;
          }

        }

        reader_state->reader_stats.output_energy = max;

        h_est = corr2_f / std::complex<float>(M2_PREAMBLE_POWER,0);

        // Shifted received waveform by n_samples_TAG_BIT/2
        max_index = max_index + M2_PREAMBLE_LEN * n_samples_TAG_BIT / 2;
        sig_power = abs(energy);  
      }

      else if(flag == 4) //M4 encoding
      {
        // n_samples_TAG_BIT = 14; 
        gr_complex corr2_f = gr_complex(0,0);
        // Do not have to check entire vector (not optimal)
        for (int i=0; i < 18 * n_samples_TAG_BIT ; i++)
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
            energy = corr2;
          }

        }

        reader_state->reader_stats.output_energy = max;

        h_est = corr2_f / std::complex<float>(M4_PREAMBLE_POWER,0);

        // Shifted received waveform by n_samples_TAG_BIT/2
        max_index = max_index + M4_PREAMBLE_LEN * n_samples_TAG_BIT / 2;
        sig_power = abs(energy);
      }

      else if(flag == 8) //M8 encoding
      {
        // n_samples_TAG_BIT = 14; 
        gr_complex corr2_f = gr_complex(0,0);
        // Do not have to check entire vector (not optimal)
        for (int i=0; i < 24 * n_samples_TAG_BIT ; i++)
        {

          corr2 = gr_complex(0,0);
          corr = 0;
          cc = 0;
          // sync after matched filter (equivalent)
          
          for (int j = 0; j < M8_PREAMBLE_LEN; j ++)
          {
            corr2 = corr2 + in[ (int) (i+j*n_samples_TAG_BIT/2) ] * gr_complex(TAG_PREAMBLE_M8[j],0);
            cc = cc + std::norm(in[(int) (i+j*n_samples_TAG_BIT/2)]);
          }
          corr = std::norm(corr2) / (M8_PREAMBLE_POWER * cc);
          if (corr > max)
          {
            max = corr;
            max_index = i;
            corr2_f = corr2;
            energy = corr2;
          }
          
        }
        // cout << "max_index: " << max_index << endl;
        reader_state->reader_stats.output_energy = max;
        //cout << "max: " << max << endl;
        h_est = corr2_f / std::complex<float>(M8_PREAMBLE_POWER,0);
        preamble_m8_start = max_index;
        // Shifted received waveform by n_samples_TAG_BIT/2
        max_index = max_index + M8_PREAMBLE_LEN * n_samples_TAG_BIT / 2;
        sig_power = abs(energy);
      }
      
      // CFO correction
      gr_complex sum_CFO = gr_complex(0, 0);
      for (int i = 0; i < 6 * flag; i++) {
        sum_CFO += std::conj(in[max_index + i * 7]) * in[max_index + i * 7 + 7];
      }
      alpha_CFO = std::arg(sum_CFO) / 7;

      return max_index;  
    }

    //////////////////////////////////////////////////////////////////////////////////////////////7

    std::vector<float>  tag_decoder_impl::tag_detection_RN16(std::vector<gr_complex> & RN16_samples_complex, int index, int flag)
    {
      // n_samples_TAG_BIT = 14; 

     std::vector<float> tag_bits,dist;
      float result=0;
      int prev = 1;
      
      int number_steps = 1000;
      float min_val = n_samples_TAG_BIT/2.0 -  0.25, max_val = n_samples_TAG_BIT/2.0 +  0.25;

      std::vector<float> energy;

      energy.resize(number_steps);
      for (int t = 0; t <number_steps; t++)
      {  
        for (int i =0; i <32 * flag; i++)
        {
          energy[t]+= reader_state->magn_squared_samples[(int) (i * (min_val + t*(max_val-min_val)/(number_steps-1)) + index)];
        }

      }
      int index_T = std::distance(energy.begin(), std::max_element(energy.begin(), energy.end()));
      float T =  min_val + index_T*(max_val-min_val)/(number_steps-1);

      // T estimated
      T_global = T;
      
      for (int i = 0; i < 16; i++) {
        RN16_samples_complex[(int)(i * flag * T + index)] *= std::exp(-gr_complex(0, flag * i * alpha_CFO));
      }
      
      tag_bits = data_decoding(tag_bits, RN16_samples_complex, T, 16, index, flag);
      

      return tag_bits;  
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    std::vector<float> tag_decoder_impl::data_decoding(std::vector<float> & tag_bits, std::vector<gr_complex> & data, float T, int num_bits, int index, int M)
    {
      
      // n_samples_TAG_BIT = 14; 
      if (M == 1) // FM0 decoding
      {
        /*
        int incr = 0;
        int cnt = 0;
        float c_m = real((data[(int) (index)]) * std::conj(h_est));
        while(cnt < 4) { // 7/14/21/28
          float c_l = real((data[(int) (index + incr - 1)]) * std::conj(h_est));
          float c_r = real((data[(int) (index + incr + 1)]) * std::conj(h_est));
          cnt++;
          if (c_l > c_m) {
            c_m = c_l;
            incr += -1;
          }
          if (c_r > c_m) {
            c_m = c_l;
            incr += 1;
            continue;
          }
          if (c_l < c_m && c_r < c_m) {
            break;
          }
        }
        index += incr;
*/
        float result=0;
        int prev = 1;
        for (int j = 0; j < num_bits ; j ++ )
        {
          result = std::real((data[ (int) (j*(2*T) + index) ] - data[ (int) (j*2*T + T + index) ])*std::conj(h_est) ); 
          //result = std::real((data[ (int) (j*(2*T) + index)] * std::exp(-gr_complex(0, 2 * j * alpha_CFO)) - \
          data[ (int) (j*2*T + T + index)] * std::exp(-gr_complex(0, (2 * j + 1) * alpha_CFO)))*std::conj(h_est) ); 
          
          if (result>0){
            if (prev == 1){
              tag_bits.push_back(0);
              //cout << "0";
            }
            else{
              tag_bits.push_back(1);
              //cout << "1";
            }      
            prev = 1;      
          }
          else
          { 
            if (prev == -1) {
              tag_bits.push_back(0);
              //cout << "0";
            }
            else {
              tag_bits.push_back(1);
              //cout << "1";
            }      
            prev = -1;    
          }
        }
        //cout << endl;
      }

      else if (M == 2) // M2 decoding
      {
        /*
        float corr_hp[4] = {0};
        int prev_s = 3;
        for (int j = 0; j < num_bits; j++) {
          memset(corr_hp, 0.0, 4 * sizeof(float));
          for (int k = 0; k < 4; k++) {
            corr_hp[0] += std::norm((data[(int) (j * 2 * M * T + k * T + index)]) * gr_complex(M2_S0[k], 0)); //1010
            corr_hp[1] += std::norm((data[(int) (j * 2 * M * T + k * T + index)]) * gr_complex(M2_S1[k], 0)); //0101
            corr_hp[2] += std::norm((data[(int) (j * 2 * M * T + k * T + index)]) * gr_complex(M2_S2[k], 0)); //1001
            corr_hp[3] += std::norm((data[(int) (j * 2 * M * T + k * T + index)]) * gr_complex(M2_S3[k], 0)); //0110
          }
          float max_corr = 0;
          int max_ind = 0;
          for (int i = 0; i < 4; i++) {
            if (corr_hp[i] > max_corr) {
              max_corr = corr_hp[i];
              max_ind = i;
            }
          }
          if ((prev_s == 0 && max_ind == 2) 
           || (prev_s == 2 && max_ind == 3)
           || (prev_s == 1 && max_ind == 3)
           || (prev_s == 3 && max_ind == 2)) {
             tag_bits.push_back(1);
             //cout << "1";
          }
          else {
             tag_bits.push_back(0);
             //cout << "0";
          }
          prev_s = max_ind;
          
        }
        */
        //cout << endl;
        // T = 1;
        // version 1
        
        for (int j = 0; j < num_bits; j++)
        {
          float s0 = 0;
          float s1 = 0;
          
          s0 = real((data[(int) (j * 2 * M * T + index)] - data[(int) (j * 2 * M * T + T + index)]) * std::conj(h_est));
          s1 = real((data[(int) (j * 2 * M * T + 2 * T + index)] - data[(int) (j * 2 * M * T + 3 * T + index)]) * std::conj(h_est));
          
          float st = s0 * s1;
          
          if (st > 0)
          {
            tag_bits.push_back(0);
            //cout << "0";
          }
          else
          {
            tag_bits.push_back(1);
            //cout << "1";
          }      
        }
        
        //cout << endl;
        
      }

      else if (M == 4) // M4 decoding (777)
      {
        /*
        int incr = 0;
        int cnt = 0;
        float c_m = real((data[(int) (index)]) * std::conj(h_est));
        while(cnt < 3) { // 7/14/21/28
          float c_l = real((data[(int) (index + incr - 1)]) * std::conj(h_est));
          float c_r = real((data[(int) (index + incr + 1)]) * std::conj(h_est));
          cnt++;
          if (c_l < c_m) {
            c_m = c_l;
            incr += -1;
          }
          if (c_r < c_m) {
            c_m = c_l;
            incr += 1;
            continue;
          }
          if (c_l > c_m && c_r > c_m) {
            break;
          }
        }
        //cout << "cnt: " << cnt << endl; 

        index += incr;

        float corr_hp[4] = {0};
        //int prev_s = 3;
        for (int j = 0; j < num_bits; j++) {
          memset(corr_hp, 0.0, 4 * sizeof(float));
          for (int k = 0; k < 8; k++) {
            corr_hp[0] += std::norm((data[(int) (j * 2 * M * T + k * T + index)]) * gr_complex(M4_S0[k], 0)); //1010
            corr_hp[1] += std::norm((data[(int) (j * 2 * M * T + k * T + index)]) * gr_complex(M4_S1[k], 0)); //0101
            corr_hp[2] += std::norm((data[(int) (j * 2 * M * T + k * T + index)]) * gr_complex(M4_S2[k], 0)); //1001
            corr_hp[3] += std::norm((data[(int) (j * 2 * M * T + k * T + index)]) * gr_complex(M4_S3[k], 0)); //0110
          }
          float max_corr = 0;
          int max_ind = 0;
          for (int i = 0; i < 4; i++) {
            if (corr_hp[i] > max_corr) {
              max_corr = corr_hp[i];
              max_ind = i;
            }
          }
          if (max_ind == 2 || max_ind == 3) {
             tag_bits.push_back(1);
             //cout << "1";
          }
          else {
             tag_bits.push_back(0);
             //cout << "0";
          }
          //prev_s = max_ind;
          
        }
        //cout << endl;
        */
        
        //version 1
        //T = 1;
        int incr = 0;
        int cnt = 0;
        float c_m = real((data[(int) (index)]) * std::conj(h_est));
        while(cnt < 3) { // 7/14/21/28
          float c_l = real((data[(int) (index + incr - 1)]) * std::conj(h_est));
          float c_r = real((data[(int) (index + incr + 1)]) * std::conj(h_est));
          cnt++;
          if (c_l < c_m) {
            c_m = c_l;
            incr += -1;
          }
          if (c_r < c_m) {
            c_m = c_l;
            incr += 1;
            continue;
          }
          if (c_l > c_m && c_r > c_m) {
            break;
          }
        }
        //cout << "cnt: " << cnt << endl; 

        index += incr;
        incr = 0;
        for (int j = 0; j < num_bits; j++) {
          float s0 = 0;
          float s1 = 0;
          
          float temp1 = (j * M) * 2 * T;
          float temp2 = (j * M + 1) * 2 * T + incr;
          s0 += real((data[(int) (temp1 + index + incr)] - data[(int) (temp1 + T + index + incr)]) * std::conj(h_est));
          s1 += real((data[(int) (temp1 + M * T + index + incr)] - data[(int) (temp1 + (M + 1) * T + index + incr)])* std::conj(h_est));

          int count = 1;
          float cursor_m = real((data[(int) (temp1 + index + incr)]) * std::conj(h_est));
          if (s0 > 0) {
            while (count < 3) { // 7/14
              float cursor_l = real((data[(int) (temp1 + index + incr - 1)]) * std::conj(h_est));
              float cursor_r = real((data[(int) (temp1 + index + incr + 1)]) * std::conj(h_est));
              count++;
              if (cursor_l > cursor_m) {
                cursor_m = cursor_l;
                incr += -1;
              }
              if (cursor_r > cursor_m){
                cursor_r = cursor_m;
                incr += 1;
                continue;
              }
              if (cursor_l < cursor_m && cursor_r < cursor_m) {
                //incr += 0;
                break;
              }
            } 
          }
          else {
            while (count < 3) { // 7/14
              float cursor_l = real((data[(int) (temp1 + index + incr - 1)]) * std::conj(h_est));
              float cursor_r = real((data[(int) (temp1 + index + incr + 1)]) * std::conj(h_est));
              count++;
              if (cursor_l < cursor_m) {
                cursor_m = cursor_l;
                incr += -1;
              }
              if (cursor_r < cursor_m){
                cursor_m = cursor_r;
                incr += 1;
                continue;
              }
              if (cursor_l > cursor_m && cursor_r > cursor_m) {
                //incr += 0;
                break;
              }
            } 
          }
          // cout << "count: " << count << endl;
          
          s0 += real((data[(int) (temp2 + index)] - data[(int) (temp2 + T + index)]) * std::conj(h_est));
          s1 += real((data[(int) (temp2 + M * T + index)] - data[(int) (temp2 + (M + 1) * T + index)]) * std::conj(h_est));


          

          float st = s0 * s1;
          if (st > 0) {
            tag_bits.push_back(0);
            //cout << "0";
          }
          else {
            tag_bits.push_back(1);
            //cout << "1";
          }
        }
        //cout << endl;
        
      }
      
      else if (M == 8) // M8 decoding
      {
        
        //T = 1;
        int incr = 0;
        int cnt = 0;
        float c_m = real((data[(int) (index)]) * std::conj(h_est));
        while(cnt < 3) { // 3/7
          float c_l = real((data[(int) (index + incr - 1)]) * std::conj(h_est));
          float c_r = real((data[(int) (index + incr + 1)]) * std::conj(h_est));
          cnt++;
          if (c_l < c_m) {
            c_m = c_l;
            incr += -1;
          }
          if (c_r < c_m) {
            c_m = c_l;
            incr += 1;
            continue;
          }
          if (c_l > c_m && c_r > c_m) {
            break;
          }
        }
        //cout << "incr: " << incr << endl; 

        index += incr;
        incr = 0;
        for (int j = 2; j < num_bits + 2; j++) {
          //cout << " [incr: " << incr << "] "; 
          float s0 = 0;
          float s1 = 0;
          
          float temp1 = (j * M) * 2 * T;
          float temp2 = (j * M + 1) * 2 * T + incr;
          float temp3 = (j * M + 2) * 2 * T + incr;
          float temp4 = (j * M + 3) * 2 * T + incr;
          s0 += real((data[(int) (temp1 + index + incr)] - data[(int) (temp1 + T + index + incr)]) * std::conj(h_est));
          s1 += real((data[(int) (temp1 + M * T + index + incr)] - data[(int) (temp1 + (M + 1) * T + index + incr)]) * std::conj(h_est));

          int count = 0;
          float cursor_m = real((data[(int) (temp1 + index + incr)]) * std::conj(h_est));
          if (s0 > 0) {
            while (count < 3) { // 3/7
              float cursor_l = real((data[(int) (temp1 + index + incr - 1)]) * std::conj(h_est));
              float cursor_r = real((data[(int) (temp1 + index + incr + 1)]) * std::conj(h_est));
              count++;
              if (cursor_l > cursor_m) {
                cursor_m = cursor_l;
                incr += -1;
              }
              if (cursor_r > cursor_m){
                cursor_r = cursor_m;
                incr += 1;
                continue;
              }
              if (cursor_l < cursor_m && cursor_r < cursor_m) {
                //incr += 0;
                break;
              }
            } 
          }
          else {
            while (count < 3) { // 3/7
              float cursor_l = real((data[(int) (temp1 + index + incr - 1)]) * std::conj(h_est));
              float cursor_r = real((data[(int) (temp1 + index + incr + 1)]) * std::conj(h_est));
              count++;
              if (cursor_l < cursor_m) {
                cursor_m = cursor_l;
                incr += -1;
              }
              if (cursor_r < cursor_m){
                cursor_m = cursor_r;
                incr += 1;
                continue;
              }
              if (cursor_l > cursor_m && cursor_r > cursor_m) {
                //incr += 0;
                break;
              }
            } 
          }
          
          s0 += real((data[(int) (temp2 + index)] - data[(int) (temp2 + T + index)]) * std::conj(h_est));
          s1 += real((data[(int) (temp2 + M * T + index)] - data[(int) (temp2 + (M + 1) * T + index)]) * std::conj(h_est));

          s0 += real((data[(int) (temp3 + index + incr)] - data[(int) (temp3 + T + index)]) * std::conj(h_est));
          s1 += real((data[(int) (temp3 + M * T + index)] - data[(int) (temp3 + (M + 1) * T + index)]) * std::conj(h_est));

          s0 += real((data[(int) (temp4 + index)] - data[(int) (temp4 + T + index)]) * std::conj(h_est));
          s1 += real((data[(int) (temp4 + M * T + index)] - data[(int) (temp4 + (M + 1) * T + index)]) * std::conj(h_est));
          

          float st = s0 * s1;
          if (st > 0) {
            tag_bits.push_back(0);
            //cout << "0";
          }
          else {
            tag_bits.push_back(1);
            //cout << "1";
          }
        }
        //cout << endl;
       
/*
      /////////////////////////////////////////////////////////////////////////////////
        int i = 0;
        int voltage_cnt = 0;
        int in_neg_pulse = 0;
        int in_pos_pulse = 0;
        int len_neg_pulse = 0;
        int len_pos_pulse = 0;
        int num_short_pulse = 0;
        cout << real(h_est) << endl;
        cout << data.size() << endl;
        std::vector<int> EPC_temp;
        while(voltage_cnt < num_bits * 16 && (index + i) < data.size()) {
          float pulse_sign = norm(data[index + i]) - 9.0/16*norm(h_est);
          //float pulse_sign = norm(data[(int)(index + i)]) - norm(h_est);

          // in pos pulse
          if (pulse_sign > 0 && in_neg_pulse == 0) {
            in_pos_pulse = 1;
            len_pos_pulse++;
          }
          // neg end, new pos start
          else if (pulse_sign > 0 && in_neg_pulse == 1) {
            
            if (len_neg_pulse >= 13) {
              EPC_temp.push_back(1);
              EPC_temp.push_back(1);
              voltage_cnt += 2;
            }
            else {
              EPC_temp.push_back(1);
              voltage_cnt += 1;
            }
            len_neg_pulse = 0;
            in_pos_pulse = 1;
            in_neg_pulse = 0;
            len_pos_pulse++;
          }
          // in neg pulse
          else if (pulse_sign < 0 && in_pos_pulse == 0) {
            in_neg_pulse = 1;
            len_neg_pulse++;
          }
          // pos end, new neg start
          else {
            
            if (len_pos_pulse >= 13) {
              EPC_temp.push_back(0);
              EPC_temp.push_back(0);
              voltage_cnt += 2;
            }
            else {
              EPC_temp.push_back(0);
              voltage_cnt += 1;
            }
              len_pos_pulse = 0;
              in_neg_pulse = 1;
              in_pos_pulse = 0;
              len_neg_pulse++;
          }
          i++;
        }
        cout << EPC_temp.size() << endl;
        for (int j = 0; j < num_bits * 16; j++) {
          cout << EPC_temp[j];
          
        }
        cout << endl;
        for (int j = 0; j < num_bits; j++) {
          if (EPC_temp[j * 2 * M + M] == EPC_temp[j * 2 * M + M - 1]) {
            tag_bits.push_back(1);
            cout << "1";
          }
          else{
            tag_bits.push_back(0);
            cout << "0";
          }
        }
        cout << endl;
        //EPC_temp.clear();
        //vector<int>().swap(EPC_temp);
        //////////////////////////////////////////
        */
      }

      return tag_bits;
    }

   //////////////////////////////////////////////////////////////////////////////////////////////////////

    std::vector<float>  tag_decoder_impl::tag_detection_EPC(std::vector<gr_complex> & EPC_samples_complex, int index, int flag)
    {

    // n_samples_TAG_BIT = 14; 
      std::vector<float> tag_bits,dist;
      float result=0;
      int prev = 1;
      
      int number_steps = 1000;
      float min_val = n_samples_TAG_BIT/2.0 - 0.25, max_val = n_samples_TAG_BIT/2.0 +  0.25;

      std::vector<float> energy;

      energy.resize(number_steps);
      for (int t = 0; t <number_steps; t++)
      {  
        for (int i =0; i <256 * flag; i++)
        {
          energy[t]+= reader_state->magn_squared_samples[(int) (i * (min_val + t*(max_val-min_val)/(number_steps-1)) + index)];
        }

      }
     

      int index_T = std::distance(energy.begin(), std::max_element(energy.begin(), energy.end()));
      float T =  min_val + index_T*(max_val-min_val)/(number_steps-1);

      // T estimated
      T_global = T;
      
      for (int i = 0; i < 128; i++) {
        EPC_samples_complex[(int)(i * flag * T + index)] *= std::exp(-gr_complex(0, flag * i * alpha_CFO));
      }

      tag_bits = data_decoding(tag_bits, EPC_samples_complex, T, 128, index, flag);

      return tag_bits;
    }

   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    std::vector<float>  tag_decoder_impl::tag_detection_HANDLE(std::vector<gr_complex> & HANDLE_samples_complex, int index, int flag)
    {

    n_samples_TAG_BIT = 14; 
      std::vector<float> tag_bits,dist;
      float result=0;
      int prev = 1;
      
      int number_steps = 100;
      float min_val = n_samples_TAG_BIT/2.0 - 1.0, max_val = n_samples_TAG_BIT/2.0 +  1.0;

      std::vector<float> energy;

      energy.resize(number_steps);
      for (int t = 0; t <number_steps; t++)
      {  
        for (int i =0; i <64 * flag; i++)
        {
          energy[t]+= reader_state->magn_squared_samples[(int) (i * (min_val + t*(max_val-min_val)/(number_steps-1)) + index)];
        }

      }
     

      int index_T = std::distance(energy.begin(), std::max_element(energy.begin(), energy.end()));
      float T =  min_val + index_T*(max_val-min_val)/(number_steps-1);

      // T estimated
      T_global = T;

     
      tag_bits = data_decoding(tag_bits, HANDLE_samples_complex, T, 32, index, flag);

     return tag_bits;
    }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////7

std::vector<float>  tag_decoder_impl::tag_detection_READ(std::vector<gr_complex> & READ_samples_complex, int index, int flag)
    {

    n_samples_TAG_BIT = 14; 
      std::vector<float> tag_bits,dist;
      float result=0;
      int prev = 1;
      
      int number_steps = 100;
      float min_val = n_samples_TAG_BIT/2.0 - 1.0, max_val = n_samples_TAG_BIT/2.0 +  1.0;

      std::vector<float> energy;

      energy.resize(number_steps);
      for (int t = 0; t <number_steps; t++)
      {  
        for (int i =0; i <65*2 * flag; i++)
        {
          energy[t]+= reader_state->magn_squared_samples[(int) (i * (min_val + t*(max_val-min_val)/(number_steps-1)) + index)];
        }

      }
     
      int index_T = std::distance(energy.begin(), std::max_element(energy.begin(), energy.end()));
      float T =  min_val + index_T*(max_val-min_val)/(number_steps-1);

      // T estimated
      T_global = T;

     
      tag_bits = data_decoding(tag_bits, READ_samples_complex, T, 65, index, flag);

     return tag_bits;
    }



/////////////////////////////////////////////////////////////////////////////////////////


    int
    tag_decoder_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const  gr_complex *) input_items[0];
      float *out = (float *) output_items[0];
      gr_complex *out_2 = (gr_complex *) output_items[1]; // for debugging
      gr_complex *out_preamble = (gr_complex *) output_items[2]; // provide preamble
      
      int written_sync =0;
      int written = 0, consumed = 0;
      int preamble_sync = 0;
      int RN16_index, HANDLE_index, READ_index;
      //int EPC_index;

      std::vector<float> RN16_samples_real;
      std::vector<float> EPC_samples_real;
      std::vector<float> HANDLE_samples_real;
      std::vector<float> READ_samples_real;

      std::vector<gr_complex> RN16_samples_complex;
      // std::vector<gr_complex> EPC_samples_complex;
      std::vector<gr_complex> HANDLE_samples_complex;
      std::vector<gr_complex> READ_samples_complex;
      std::vector<float> RN16_bits;

      int number_of_half_bits = 0;
      int var = 0;
      int SW = 0;
      int delta_Q = 0;

      std::vector<float> EPC_bits;  
      std::vector<float> HANDLE_bits;  
      std::vector<float> READ_bits; 
      
      // Processing only after n_samples_to_ungate are available and we need to decode an RN16
      if (reader_state->decoder_status == DECODER_DECODE_RN16 && ninput_items[0] >= reader_state->n_samples_to_ungate)
      {   

       RN16_index = tag_sync(in,ninput_items[0],ENCODING_SCHEME);
       //std::cout << "RN16 INDEX:  " << RN16_index << std::endl;
   

        if (reader_state->reader_stats.output_energy > C_th) // loose condition sig_power > E_th && reader_state->reader_stats.output_energy > C_th
        {
          
         //cout << "corr: " << reader_state->reader_stats.output_energy << endl;
         for (int j = 0; j < ninput_items[0]; j++ )
          {
          
          RN16_samples_complex.push_back(in[j]);
          out_2[written_sync] = in[j];  // Save data of decoder into file for debugging purposes   
           written_sync ++;
         } 
        
        	
        out_2[written_sync] = 2; 
        written_sync ++; 
        produce(1,written_sync);

            
            //GR_LOG_INFO(d_debug_logger, "RN16 DECODED");
            RN16_bits  = tag_detection_RN16(RN16_samples_complex,RN16_index, ENCODING_SCHEME);

              // Show on the terminal the bit-string of the RN16
             //---------------------------------------------------------------         
             //for (std::vector<float>::const_iterator i = RN16_bits.begin(); i != RN16_bits.end(); ++i) {
               //std::cout << *i;
             //}
             //std::cout << "--END OF RN16" << std::endl;
              //---------------------------------------------------------------         

             
              for(int bit=0; bit<RN16_bits.size(); bit++)
              {
                out[written] =  RN16_bits[bit];
                RN16_handle_stored[bit] = RN16_bits[bit];
                written ++;
              }

              reader_state->reader_stats.RN16_bits_handle = RN16_bits;
              produce(0,written);

              reader_state->gen2_logic_status = SEND_ACK;
              
        }

        else // no response from tags -- power of output signal too low
        {
          // -------------------   IDLE SLOT ------------------------------------
          // cout << "Noise Power  : " << 10 * log10(std::norm(h_est)) << endl;
          NoiseI = NoiseI + (10 * log10(std::norm(h_est)) - NoiseI) / cnt_NoiseI;
          cnt_NoiseI++;
          reader_state->reader_stats.n_0+=1;
          reader_state->reader_stats.tn_0 +=1 ; 

          reader_state-> reader_stats.Qfp = reader_state-> reader_stats.Qfp - C;

          //Qf = max(0, Qfp-C)
          var = round(reader_state-> reader_stats.Qfp);
          if (var > 0)
          {
            if(var >15)
            {
              reader_state-> reader_stats.Qant = 15;
            }
            else
            {
              reader_state-> reader_stats.Qant = var;
            }
          }
          else
          {
            reader_state-> reader_stats.Qant =  0; 
          }

          update_slot();
          //reader_state->gen2_logic_status = SEND_QUERY;
        }
       consumed = reader_state->n_samples_to_ungate;
      
      } 
      
      else if (reader_state->decoder_status == DECODER_DECODE_EPC && ninput_items[0] >= reader_state->n_samples_to_ungate )
      {

        
        vector<gr_complex>().swap(reader_state->reader_stats.aux_EPC_samples_complex);
        reader_state->reader_stats.aux_EPC_index = tag_sync(in,ninput_items[0],ENCODING_SCHEME);
        /*
        if (reader_state->reader_stats.aux_buffer_flag == 3 || ENCODING_SCHEME != 10) {
          vector<gr_complex>().swap(reader_state->reader_stats.aux_EPC_samples_complex);
          reader_state->reader_stats.aux_EPC_index = tag_sync(in,ninput_items[0],ENCODING_SCHEME);
          
          //cout << "A EPC index:" << reader_state->reader_stats.aux_EPC_index << endl;
        }
        */
        for (int j = 0; j < ninput_items[0]; j++ )
        {
          reader_state->reader_stats.aux_EPC_samples_complex.push_back(in[j]);

           //out_2[written_sync] = in[j]; 
           //written_sync ++;
        }
        
        //out_2[written_sync] = 2; 
        //written_sync ++; 
        //produce(1,written_sync);
        /*
        if (reader_state->reader_stats.aux_buffer_flag > 1) {
          reader_state->reader_stats.aux_buffer_flag--;
          consumed = reader_state->n_samples_to_ungate;
          consume_each (consumed);
          //reader_state->gen2_logic_status = SEND_CW_ACK;
          return WORK_CALLED_PRODUCE;
        } 
        if (reader_state->reader_stats.aux_buffer_flag == 1){
          reader_state->reader_stats.aux_buffer_flag--;
        }
        */
        //cout << "stage 2" << endl;
        //cout << "B EPC index:" << reader_state->reader_stats.aux_EPC_index << endl;
        EPC_bits   = tag_detection_EPC(reader_state->reader_stats.aux_EPC_samples_complex,reader_state->reader_stats.aux_EPC_index, ENCODING_SCHEME);
        vector<gr_complex>().swap(reader_state->reader_stats.aux_EPC_samples_complex);
       
      
        if (EPC_bits.size() == EPC_BITS - 1  && sig_power > E_th && reader_state->reader_stats.output_energy > C_th)
        { //   
          // collect M8 preamble
          /*
          if (ENCODING_SCHEME == 8) {
            for (int pb_i = 0; pb_i < n_samples_TAG_BIT * 48; pb_i++) {
              preamble_m8[pb_i] = (gr_complex(cnt_preamble_collected, 0) * preamble_m8[pb_i] + in[preamble_m8_start + 2 * pb_i]) / gr_complex(cnt_preamble_collected + 1, 0);
            }
            cnt_preamble_collected++;
          }
          */
          // collect localization data
          if (cnt_preamble_collected < 100 && reader_state->reader_stats.n_epc_detected >= 1) {
            for (int pb_i = 0; pb_i < n_samples_TAG_BIT * 6; pb_i++) {
              preamble_fm0[cnt_preamble_collected*64 + pb_i] = gr_complex(1, 0) * in[preamble_fm0_start + pb_i];
            }
            double t_preamble;
            clock_gettime(CLOCK_MONOTONIC, &end_time);
            t_preamble = (end_time.tv_sec - start_time.tv_sec) * 1e9; 
            t_preamble = (t_preamble + (end_time.tv_nsec - start_time.tv_nsec)) * 1e-9;
            t_preamble_record.push_back(t_preamble);

            cnt_preamble_collected++;
          }

          // collect fm0 preamble
          if (DATA_COLLECTION_EN == 1 && ENCODING_SCHEME == 1 && cnt_queries < 2500) {
            for (int pb_i = 0; pb_i < n_samples_TAG_BIT * 6; pb_i++) {
              preamble_fm0[pb_i] = (gr_complex(cnt_preamble_collected, 0) * preamble_fm0[pb_i] + in[preamble_fm0_start + pb_i]) / gr_complex(cnt_preamble_collected + 1, 0);
            }
            

            cnt_preamble_collected++;
          }
          // probing fm0 preamble
          if (adabs_probing_en == 1 && ENCODING_SCHEME == 1) {
            int k = 0;
            for (; k < n_samples_TAG_BIT * 6; k++) {
              out_preamble[preamble_sync] = in[preamble_fm0_start + k];
              preamble_sync++;
            }
            for (; k < 256; k++) {
              out_preamble[preamble_sync] = gr_complex(0, 0);
              preamble_sync++;
            }

            produce(2,preamble_sync);
          }
          //cout << "corr : " << reader_state->reader_stats.output_energy << endl;
          RSSI = 10 * log10(std::norm(h_est));
          RSSI = RSSI + (10 * log10(std::norm(h_est)) - RSSI) / cnt_RSSI;
          cnt_RSSI++;
          Phase = std::arg(h_est);
          //cout << "RSSI: " << RSSI << endl;
          //cout << "Phase: " << std::arg(h_est) << endl;  
          
          valid_packet = 1;
	        if(reader_state->reader_stats.n_epc_detected < 1){
            clock_gettime(CLOCK_MONOTONIC, &start_time);
            // and monitoring starts
            clock_gettime(CLOCK_MONOTONIC, &tp_monitor_st);
	        }
	        else{
            // record time
            if (0) {
            double t_preamble;
            clock_gettime(CLOCK_MONOTONIC, &end_time);
            t_preamble = (end_time.tv_sec - start_time.tv_sec) * 1e9; 
            t_preamble = (t_preamble + (end_time.tv_nsec - start_time.tv_nsec)) * 1e-9;
            t_preamble_record.push_back(t_preamble);
            }
            // TO DO: TIMEOUT & WHERE TO START WHEN RECOVERED
            // throughput monitoring per duty cycle
            clock_gettime(CLOCK_MONOTONIC, &tp_monitor_ed);
            double d_monitor;
            d_monitor = (tp_monitor_ed.tv_sec - tp_monitor_st.tv_sec) * 1e9; 

            d_monitor = (d_monitor + (tp_monitor_ed.tv_nsec - tp_monitor_st.tv_nsec)) * 1e-9;
            float amp_cal = 0;
            // record Tx energy cost here
            if (rta_ampl / 0.7 < 0.55) {
               amp_cal = 1.50356 * (rta_ampl / 0.7 - 0.55) + 0.78861;
            }
            else {
               amp_cal = -1.0439 * (rta_ampl /0.7 - 1) * (rta_ampl / 0.7 - 1) + 1;
            }
            E_Tx += (d_monitor * amp_cal * amp_cal);
            accumulative_d_monitor += d_monitor; 
            // don't monitor when probing
            if (d_monitor > 0.04) {
              
              pktCorrectRatio = 1.0 * cnt_correct_epc / (cnt_loss_epc + cnt_correct_epc);
              if (ADABS_PROBING_MODE == 0) {
                tp_now = 1.0 * (retran_goodput_pkt_cnt - retran_goodput_pkt_prev_cnt) / d_monitor;
                throughput_monitored = (tp_now + cnt_tp_monitored * throughput_monitored) / (cnt_tp_monitored + 1);
                tp_record.push_back(throughput_monitored);
                t_record.push_back(total_time);
                p_record.push_back(amp_inference_raw);
                if (cnt_tp_monitored == 2) {
                  THROUGHPUT_AVAILABLE = 1; // ready for reading
                  //cnt_tp_monitored = 0;
                  //throughput_monitored = 0;
                }
                else {
                  cnt_tp_monitored++;
                }
              }
              
              // cout << "Throughput Monitored : " << throughput_monitored << endl;
              clock_gettime(CLOCK_MONOTONIC, &tp_monitor_st);
              cnt_correct_epc = 0;
              retran_goodput_pkt_prev_cnt = retran_goodput_pkt_cnt;
            }
            

            // power-up delay and accumulative tp monitoring
            //if (ADABS_PROBING_DONE == 0) {
            clock_gettime(CLOCK_MONOTONIC, &end_time);
            Powerup_Delay_probed = (end_time.tv_sec - previous_time.tv_sec) * 1e9; 
            Powerup_Delay_probed = (Powerup_Delay_probed + (end_time.tv_nsec - previous_time.tv_nsec)) * 1e-9; 
            //}
		        if (Powerup_Delay_probed > 0.04){
              ADABS_PROBING_DONE = 1; // power-up delay is available
		          reader_state->reader_stats.power_up_delay = (Powerup_Delay_probed + cnt_power_up_delay * reader_state->reader_stats.power_up_delay) / (cnt_power_up_delay + 1);
              cnt_power_up_delay++;

		          //double total_time; 
              total_time = (end_time.tv_sec - start_time.tv_sec) * 1e9; 
              total_time = (total_time + (end_time.tv_nsec - start_time.tv_nsec)) * 1e-9; 
              //t_record.push_back(total_time);
              reader_state->reader_stats.average_throughput = reader_state->reader_stats.n_epc_correct / total_time;
		        }
             
	        }
          clock_gettime(CLOCK_MONOTONIC, &previous_time); 

	  
          // float to char -> use Buettner's function
          for (int i =0; i < 40; i ++)
          {
            if (EPC_bits[i + 88] == 0)
              char_bits[i] = '0';
            else
              char_bits[i] = '1';
          }
	        reader_state->reader_stats.n_epc_detected+=1;
          // correct epc
          if(check_crc(char_bits,40) == 1)
          {
            cnt_correct_epc++;
            reader_state->reader_stats.n_epc_correct+=1;
            reader_state->reader_stats.n_1+=1;
            reader_state->reader_stats.tn_1  +=1;

            // record transmission state
            prev_transmission_state = curr_transmission_state;
            curr_transmission_state = 1;
            blink_n_success += 1;

            int result = 0;
            for(int i = 0 ; i < 32 ; ++i)
            {
              result += std::pow(2,31-i) * EPC_bits[80+i] ;
            }
	          printf("EPC: %x\n", result);
            /*
            // interprete temperature readings
            int temp_sign = 1;
            int temp = 0;
            for (int i = 0; i < 16; i++) {
              temp += std::pow(2, 15-i) * EPC_bits[89 + i];
            }
            if (EPC_bits[88] == 1) {
              temp_sign = -1;
              temp = ~temp + 1;
            }
            temp &= 32767;
            temp *= temp_sign;
            printf("T = %.2f C\n", temp/1000.0);
            */
           /*
            // interprete accel reading
            // accel of x
            int accel_sign = 1;
            int accel_x = 0;
            for (int i = 0; i < 7; i++) {
              accel_x += std::pow(2, 6-i) * EPC_bits[89 + i];
            }
            if (EPC_bits[88] == 1) {
              accel_sign = -1;
              accel_x = ~accel_x + 1;
            }
            accel_x &= 127;
            accel_x *= accel_sign;

            // accel of y
            accel_sign = 1;
            int accel_y = 0;
            for (int i = 0; i < 7; i++) {
              accel_y += std::pow(2, 6-i) * EPC_bits[97 + i];
            }
            if (EPC_bits[96] == 1) {
              accel_sign = -1;
              accel_y = ~accel_y + 1;
            }
            accel_y &= 127;
            accel_y *= accel_sign;

            // accel of z
            accel_sign = 1;
            int accel_z = 0;
            for (int i = 0; i < 7; i++) {
              accel_z += std::pow(2, 6-i) * EPC_bits[105 + i];
            }
            if (EPC_bits[104] == 1) {
              accel_sign = -1;
              accel_z = ~accel_z + 1;
            }
            accel_z &= 127;
            accel_z *= accel_sign;
            printf("[Ax: %.3f g, Ay: %.3f g, Az: %.3f g]\n", accel_x/64.0, accel_y/64.0, accel_z/64.0);
          */
            // Save part of Tag's EPC message (EPC[104:111] in decimal) + number of reads
            /*
            std::map<int,int>::iterator it = reader_state->reader_stats.tag_reads.find(result);
            if ( it != reader_state->reader_stats.tag_reads.end())
            {
              it->second ++;
            }
            else
            {
              reader_state->reader_stats.tag_reads[result]=1;
            }
*/
          // ----------------------------------------------------------------------------------------------------
            //update_slot();
            reader_state->gen2_logic_status = SEND_ACK;

            for(int bit=0; bit<16; bit++)
              {
                out[written] =  RN16_handle_stored[bit];
                written ++;
              }

              reader_state->reader_stats.RN16_bits_handle = RN16_bits;
              produce(0,written);
          }

          else // valid packet but bit-error
          {
           
           // ---------------------- COLLISION SLOT ---------------------------------------
            /*
            int lsym = 0;
            for(int i = 24 ; i < 32 ; ++i)
            {
              lsym += std::pow(2,31-i) * EPC_bits[80+i] ;
            }
            if (lsym == 12) {
              cnt_loss_epc++;
	            printf("%d : bit-error\n", reader_state->reader_stats.n_queries_sent);

              // record transmission state
              prev_transmission_state = curr_transmission_state;
              curr_transmission_state = 0;

              blink_n_failure += 1;
            }
            */
            cnt_loss_epc++;
            cnt_loss_epc_global++;
            retran_is_pkt_loss = 1;
	          printf("%d : bit-error\n", reader_state->reader_stats.n_queries_sent);

            // record transmission state
            prev_transmission_state = curr_transmission_state;
            curr_transmission_state = 0;

            blink_n_failure += 1;
            reader_state->reader_stats.tn_k  +=1; 
            reader_state->reader_stats.n_k+=1;
      
            
            
      
	    //printf("EPC: %x\n", result_0);

            reader_state-> reader_stats.Qfp = reader_state-> reader_stats.Qfp + C;

            //Q = min(15, Qfp+C)
            var = round(reader_state-> reader_stats.Qfp);
            if (var < 15)
            {
              if(var<0)
              {
                reader_state-> reader_stats.Qant  = 0;
              }
              else
              {
                reader_state-> reader_stats.Qant = var;
              }
            }
            else
            {
              reader_state-> reader_stats.Qant = 15; //maximum value for Q is 15
            } 
            update_slot();
       
          }
     
        }  
        else
        {
          //cout << "Noise Power : " << 10 * log10(std::norm(h_est)) << endl;
          cnt_loss_epc++;
          cnt_loss_epc_global++;
          retran_is_pkt_loss = 1;
          printf("%d : no epc pkt found\n", reader_state->reader_stats.n_queries_sent);
          valid_packet = 1;
          reader_state->gen2_logic_status = SEND_QUERY;
          //GR_LOG_INFO(d_logger, "CHECK ME");
          //GR_LOG_EMERG(d_debug_logger, "CHECK ME");  
        }
        
        if (TARGET == 1) {
          TARGET = 0;
        } 
        
        // summarize retran-goodput & delay
        if (retran_i_window >= (bulky_N - 1)) { // window finished
          if (retran_is_pkt_loss == 0) { // N bulk packets successfully received
            retran_goodput_cnt += 1; // 1 or indES
            retran_goodput_pkt_cnt += bulky_N;
            
            // calculate delay of receving the last N bulk packets
            double curr_delay = 0;
            clock_gettime(CLOCK_MONOTONIC, &retran_end_time);
            retran_total_time = (retran_end_time.tv_sec - start_time.tv_sec) * 1e9;
            retran_total_time = (retran_total_time + (retran_end_time.tv_nsec - start_time.tv_nsec)) * 1e-9; 
            
            if (retran_goodput_cnt <= 1) {
              curr_delay = retran_total_time / bulky_N;
            }
            else {
              curr_delay = (retran_end_time.tv_sec - retran_previous_time.tv_sec) * 1e9; 
              curr_delay = (curr_delay + (retran_end_time.tv_nsec - retran_previous_time.tv_nsec)) * 1e-9; 
            }
            clock_gettime(CLOCK_MONOTONIC, &retran_previous_time);
            retran_delay = retran_delay + (curr_delay - retran_delay) / retran_goodput_cnt;

          }
          else { // there is packet loss within these N bulk packets
            TARGET = 1;
            reader_state->gen2_logic_status = SEND_QUERY;
          }
          // reset flags
          retran_i_window = 0;
          retran_is_pkt_loss = 0;
        }
        else { // windows slides by 1
          retran_i_window += 1;
        }

        consumed = reader_state->n_samples_to_ungate;
        //consumed = ninput_items[0];
      }

  ///////////////////////////////////////////////////////////////////
    else if (reader_state->decoder_status == DECODER_DECODE_HANDLE && ninput_items[0] >= reader_state->n_samples_to_ungate ){

        HANDLE_index = tag_sync(in,ninput_items[0],0);
        
        for (int j = 0; j < ninput_items[0]; j++ )
          {
            HANDLE_samples_complex.push_back(in[j]);
            out_2[written_sync] = in[j]; 
             written_sync ++;

         } 
         std::cout << "HANDLE INDEX: " << HANDLE_index << std::endl;

         out_2[written_sync] = 2; 

         written_sync ++; 
         produce(1,written_sync);

         HANDLE_bits  = tag_detection_HANDLE(HANDLE_samples_complex,HANDLE_index, ENCODING_SCHEME);
         //This variable contains only 16 bits of the handle.
         //We also need to get the next 16 bits of the CRC


         //OBTAIN THE CRC OF TAG REPLY 
          // float to char -> use Buettner's function
          for (int i =0; i < 32; i ++)
          {
            if (HANDLE_bits[i] == 0)
              char_bits_HANDLE[i] = '0';
            else
              char_bits_HANDLE[i] = '1';
          }
      
         if(check_crc(char_bits_HANDLE,32) == 1)
          {
            std::cout << " *********** HANDLE CORRECT ***************" << std::endl;
            reader_state->reader_stats.RN16_bits_read = HANDLE_bits;

            for(int bit=0; bit<16; bit++)
              {
                out[written] =  reader_state->reader_stats.RN16_bits_read[bit];
                written ++;
              }
              
              produce(0,written); //We pass the RN16-HANDLE bits to the next block

              
            reader_state->gen2_logic_status = SEND_READ;
          }
          else{
            std::cout << " *********** WRONG CRC OF HANDLE  ***************" << std::endl;
            update_slot();
          }

        

         consumed = reader_state->n_samples_to_ungate;

    }
//-----------------------------------------------------------------------------------------
     else if (reader_state->decoder_status == DECODER_DECODE_READ && ninput_items[0] >= reader_state->n_samples_to_ungate ){

        READ_index = tag_sync(in,ninput_items[0],1);

       for (int j = 0; j < ninput_items[0]; j++ )
          {
            READ_samples_complex.push_back(in[j]);
            out_2[written_sync] = in[j]; 
             written_sync ++;

         } 
         std::cout << "READ INDEX: " << READ_index << std::endl;

         out_2[written_sync] = 2; 

         written_sync ++; 
         produce(1,written_sync);

         READ_bits  = tag_detection_READ(READ_samples_complex,READ_index, ENCODING_SCHEME);

         reader_state-> reader_stats.sensor_read += 1;


              update_slot();

              consumed = reader_state->n_samples_to_ungate;
}
  ///////////////////////////////////////////////////////////////////////

      consume_each(consumed);
     return WORK_CALLED_PRODUCE;
    }


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void tag_decoder_impl::performance_evaluation()
      {   

      if (reader_state-> reader_stats.stop == 1)
        {
          reader_state-> reader_stats.stop = 0;

          float it_reader = reader_state-> reader_stats.tQA* TQA + reader_state-> reader_stats.tQR * TQR + TQ;
          float it_tag = Tsk*reader_state->reader_stats.tn_k+Tsk*reader_state->reader_stats.tn_1+Ti*reader_state->reader_stats.tn_0;

          reader_state-> reader_stats.TIR_th =reader_state->reader_stats.n_epc_correct  /(it_reader* pow(10,-6) + it_tag* pow(10,-6));
          reader_state-> reader_stats.TIR_exp = reader_state->reader_stats.n_epc_correct/(float)((reader_state-> reader_stats.it_timer));

           std::cout <<"| ----------------------------------------------------------------------- " <<  std::endl;
          std::cout << "| TIR theoretic : "  <<  reader_state-> reader_stats.TIR_th     << std::endl;
          std::cout << "| TIR experimental : "  <<  reader_state-> reader_stats.TIR_exp << std::endl;

          //WRITE AND SAVE RESULTS INTO FILE TO PLOT LATER IN MATLAB
          ofstream myfile;

          myfile.open ("ntags_read.txt", ios::app);
          myfile << reader_state->reader_stats.tag_reads.size();
          myfile << "\n";
          myfile.close();

          myfile.open ("TIR_timer.txt", ios::app);
          myfile << reader_state-> reader_stats.TIR_exp;
          myfile << "\n";
          myfile.close();

          myfile.open ("TIR_formula.txt", ios::app);
          myfile << reader_state-> reader_stats.TIR_th;
          myfile << "\n";
          myfile.close();
         
          myfile.open ("tQA.txt", ios::app);
          myfile << reader_state-> reader_stats.tQA;
          myfile << "\n";
          myfile.close();

          myfile.open ("tQR.txt", ios::app);
          myfile << reader_state-> reader_stats.tQR;
          myfile << "\n";
          myfile.close();

          myfile.open ("ck.txt", ios::app);
          myfile << reader_state->reader_stats.tn_k;
          myfile << "\n";
          myfile.close();

          myfile.open ("ci.txt", ios::app);
          myfile << reader_state->reader_stats.tn_0;
          myfile << "\n";
          myfile.close();


          }
     }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   void tag_decoder_impl::update_slot()
    {
      
      //Evaluate results when ntags are identified, and the current frame is terminated
       if(reader_state-> reader_stats.tag_reads.size() >= NUMBER_UNIQUE_TAGS && reader_state->reader_stats.cur_slot_number == pow(2,reader_state-> reader_stats.VAR_Q)) //make calculations
        {
          gettimeofday (&reader_state-> reader_stats.end, NULL);
          reader_state-> reader_stats.it_timer = reader_state-> reader_stats.end.tv_sec - reader_state-> reader_stats.start.tv_sec+(reader_state-> reader_stats.end.tv_usec*pow(10,-6) - reader_state-> reader_stats.start.tv_usec*pow(10,-6));

          performance_evaluation();
        } 

      //If End of frame or Q modified(at any slot) => new frame
      if (reader_state->reader_stats.cur_slot_number == pow(2,reader_state-> reader_stats.VAR_Q) || reader_state-> reader_stats.VAR_Q != reader_state->reader_stats.Qant)
      { 
	      /*
        if (reader_state->reader_stats.Qant > reader_state-> reader_stats.VAR_Q) 
        {
          reader_state-> reader_stats.Qupdn = 6; //increase Q by one
          reader_state-> reader_stats.VAR_Q = reader_state->reader_stats.Qant;
        }
        else if(reader_state->reader_stats.Qant <  reader_state-> reader_stats.VAR_Q)
        {
          reader_state-> reader_stats.Qupdn = 3; //decrease Q by one
          reader_state-> reader_stats.VAR_Q = reader_state->reader_stats.Qant;
        }
        else
        {
          reader_state-> reader_stats.Qupdn = 0;// Q unchanged
        }
	*/

	reader_state-> reader_stats.VAR_Q = reader_state-> reader_stats.Qupdn;

        reader_state->reader_stats.cur_slot_number = 1;
        reader_state->reader_stats.unique_tags_round.push_back(reader_state->reader_stats.tag_reads.size());
        reader_state->reader_stats.cur_inventory_round += 1;
        reader_state->reader_stats.n_k = 0;
        reader_state->reader_stats.n_1 = 0;
        reader_state->reader_stats.n_0 = 0;

        reader_state->gen2_logic_status = SEND_QUERY;
      }

      else 
      {
	reader_state-> reader_stats.VAR_Q = reader_state-> reader_stats.Qupdn;
        reader_state->reader_stats.cur_slot_number++;
        reader_state->gen2_logic_status = SEND_QUERY;
      }
    }


    /* Function adapted from https://www.cgran.org/wiki/Gen2 */
    int tag_decoder_impl::check_crc(char * bits, int num_bits)
    {
      register unsigned short i, j;
      register unsigned short crc_16, rcvd_crc;
      unsigned char * data;
      int num_bytes = num_bits / 8;
      data = (unsigned char* )malloc(num_bytes );
      int mask;

      for(i = 0; i < num_bytes; i++)
      {
        mask = 0x80;
        data[i] = 0;
        for(j = 0; j < 8; j++)
        {
          if (bits[(i * 8) + j] == '1'){
          data[i] = data[i] | mask;
        }
        mask = mask >> 1;
        }
      }

      rcvd_crc = (data[num_bytes - 2] << 8) + data[num_bytes -1];

      crc_16 = 0xFFFF; 
      for (i=0; i < num_bytes - 2; i++)
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
      //printf("rcvd_crc : %x vs. crc : %x", rcvd_crc, crc_16);
      if(rcvd_crc != crc_16)
        return -1;
      else
        return 1;
    }
    
  } /* namespace rfid */
} /* namespace gr */