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

#ifndef INCLUDED_RFID_PBR_FEATURE_EXTRACTOR_IMPL_H
#define INCLUDED_RFID_PBR_FEATURE_EXTRACTOR_IMPL_H

#include <rfid/pbr_feature_extractor.h>
#include <vector>
#include <rfid/interaction_global_vars.h>
#include "rfid/pbr_global_vars.h"
#include <time.h>
#include <numeric>
#include <fstream>

namespace gr {
  namespace rfid {

    class pbr_feature_extractor_impl : public pbr_feature_extractor
    {
     private:
      float output_energy;
      float n_samples_TAG_BIT;
      gr_complex h_est;
      int tag_sync(const gr_complex * in, int size, int flag);

     public:
      pbr_feature_extractor_impl(int sample_rate);
      ~pbr_feature_extractor_impl();

      // Where all the action really happens
      int work(int noutput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
    };

  } // namespace rfid
} // namespace gr

#endif /* INCLUDED_RFID_PBR_FEATURE_EXTRACTOR_IMPL_H */

