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


#ifndef INCLUDED_RFID_MULTIPLY_RTA_FF_H
#define INCLUDED_RFID_MULTIPLY_RTA_FF_H

#include <rfid/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
  namespace rfid {

    /*!
     * \brief <+description of block+>
     * \ingroup rfid
     *
     */
    class RFID_API multiply_rta_ff : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<multiply_rta_ff> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of rfid::multiply_rta_ff.
       *
       * To avoid accidental use of raw pointers, rfid::multiply_rta_ff's
       * constructor is in a private implementation
       * class. rfid::multiply_rta_ff::make is the public interface for
       * creating new instances.
       */
      static sptr make();
    };

  } // namespace rfid
} // namespace gr

#endif /* INCLUDED_RFID_MULTIPLY_RTA_FF_H */

