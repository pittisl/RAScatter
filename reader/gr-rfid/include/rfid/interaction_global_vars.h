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


#ifndef INCLUDED_RFID_INTERACTION_GLOBAL_VARS_H
#define INCLUDED_RFID_INTERACTION_GLOBAL_VARS_H

#include <rfid/api.h>

namespace gr {
  namespace rfid {

    enum PBR_STATUS             {PBR_RUNNING, PBR_TERMINATED};
    enum PBR_GATE_STATUS        {PBR_GATE_OPEN, PBR_GATE_CLOSED, PBR_GATE_SEEK_RN16, PBR_GATE_SEEK_EPC, PBR_GATE_SEEK_HANDLE, PBR_GATE_SEEK_READ};  
    enum PBR_DECODER_STATUS     {PBR_DECODER_DECODE_RN16, PBR_DECODER_DECODE_EPC, PBR_DECODER_DECODE_HANDLE, PBR_DECODER_DECODE_READ};

    extern int pbr_index_ES;
    extern PBR_STATUS status;
    extern int n_queries_sent;
    extern PBR_GATE_STATUS gate_status;
    extern PBR_DECODER_STATUS decoder_status;

  } // namespace rfid
} // namespace gr

#endif /* INCLUDED_RFID_INTERACTION_GLOBAL_VARS_H */

