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
#include <rfid/interaction_global_vars.h>

namespace gr {
  namespace rfid {

    int pbr_index_ES = 0;
    PBR_STATUS status = PBR_RUNNING;
    int n_queries_sent = 0;
    PBR_GATE_STATUS gate_status = PBR_GATE_SEEK_RN16;
    PBR_DECODER_STATUS decoder_status = PBR_DECODER_DECODE_RN16;

  } /* namespace rfid */
} /* namespace gr */

