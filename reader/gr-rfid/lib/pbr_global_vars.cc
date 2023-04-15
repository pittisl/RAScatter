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
#include <rfid/pbr_global_vars.h>
#include <rfid/interaction_global_vars.h>

namespace gr {
  namespace rfid {

    int PBR_ENCODING_SCHEME = index_ES_LIST[0]; // 1/2/4/8 --> FM0/M2/M4/M8
    int n_samples_to_ungate = 0;

    float PBR_TAG_BIT_D   = 1.0 * PBR_ENCODING_SCHEME/T_READER_FREQ * pow(10,6); // Duration in us
    float PBR_RN16_D      = (RN16_BITS + TAG_PREAMBLE_BITS) * PBR_TAG_BIT_D;
    float PBR_EPC_D         = (EPC_BITS  + TAG_PREAMBLE_BITS) * PBR_TAG_BIT_D;
    float PBR_HANDLE_D = (RN16_BITS - 1  + TAG_PREAMBLE_BITS + 16) * PBR_TAG_BIT_D; // RN16 without? dummy-bit  
    float PBR_READ_D = (1 + 32+ 16 + 16+ TAG_PREAMBLE_BITS ) * PBR_TAG_BIT_D; // RN16 without? dummy-bit

    float PBR_E_th = E_th_LIST[PBR_ENCODING_SCHEME];

  } /* namespace rfid */
} /* namespace gr */

