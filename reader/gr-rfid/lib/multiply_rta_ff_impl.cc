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
#include "multiply_rta_ff_impl.h"
#include <volk/volk.h>

namespace gr {
  namespace rfid {

    multiply_rta_ff::sptr
    multiply_rta_ff::make()
    {
      return gnuradio::get_initial_sptr
        (new multiply_rta_ff_impl());
    }

    /*
     * The private constructor
     */
    multiply_rta_ff_impl::multiply_rta_ff_impl()
      : gr::sync_block("multiply_rta_ff",
              gr::io_signature::make(1, 1, sizeof(float)),
              gr::io_signature::make(1, 1, sizeof(float)))
    {
      const int alignment_multiple = volk_get_alignment() / sizeof(float);
      set_alignment(std::max(1, alignment_multiple));
    }

    /*
     * Our virtual destructor.
     */
    multiply_rta_ff_impl::~multiply_rta_ff_impl()
    {
    }

    int
    multiply_rta_ff_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      const float *in = (const float *) input_items[0];
      float *out = (float *) output_items[0];
      int noi = noutput_items;

      volk_32f_s32f_multiply_32f(out, in, rta_ampl, noi);

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace rfid */
} /* namespace gr */

