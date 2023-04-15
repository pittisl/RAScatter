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


#ifndef INCLUDED_RFID_PBR_GLOBAL_VARS_H
#define INCLUDED_RFID_PBR_GLOBAL_VARS_H

#include <rfid/api.h>
#include <map>
#include <sys/time.h>

namespace gr {
  namespace rfid {

    extern int n_samples_to_ungate;

    // CONSTANTS (READER CONFIGURATION)

    // Fixed number of slots (2^(FIXED_Q))  
    const int FIXED_Q              =0;
    
     const float C = 0.2;

    // Termination criteria
    
    const int MAX_NUM_QUERIES     = 8000;     // Stop after MAX_NUM_QUERIES have been sent

    // valid values for Q
    const int Q_VALUE [16][4] =  
    {
        {0,0,0,0}, {0,0,0,1}, {0,0,1,0}, {0,0,1,1}, 
        {0,1,0,0}, {0,1,0,1}, {0,1,1,0}, {0,1,1,1}, 
        {1,0,0,0}, {1,0,0,1}, {1,0,1,0}, {1,0,1,1},
        {1,1,0,0}, {1,1,0,1}, {1,1,1,0}, {1,1,1,1}
    };  
    const bool P_DOWN = false;

    //Tpri = 1/BLF =  us

   // Duration in us
    const int CW_D         = 1500;    // Carrier wave
    const int P_DOWN_D     = 2000;    // power down
    
    const int T1_D         = 56;    // Time from Interrogator transmission to Tag response (62.5 us)
    const int T2_D         = 120;    // Time from Tag response to Interrogator transmission. Max value = 20.0 * T_tag = 125us 

    const float PW_D         = 6.25;      // Half Tari 

    const float DELIM_D       = 12.5;      // A preamble shall comprise a fixed-length start delimiter 12.5us +/-5%
    const float TRCAL_D     = 50;    // BLF = DR/TRCAL => 133.33 for DR=1(64/3), 50 for DR=0(8)
    const int RTCAL_D     = 36;      // 31.25for data_1 = 1.5*data_0; 37.5 for data_1=2*data_0

    const int NUM_PULSES_COMMAND = 4;       // Number of pulses to detect a reader command // 
    const int NUMBER_UNIQUE_TAGS = 100;      // Stop after NUMBER_UNIQUE_TAGS have been read 

    // Number of bits
    const int PILOT_TONE         = 12;  // Optional
    const int TAG_PREAMBLE_BITS  = 6;   // Number of preamble bits
    const int RN16_BITS          = 17;  // Dummy bit at the end
    const int EPC_BITS           = 129;  // PC + EPC + CRC16 + Dummy => (6--preamble) + 16 + 96 + 16 + 1 = 135
    const int QUERY_LENGTH       = 22;  // Query length in bits
    
    const int T_READER_FREQ = 160e3;     // BLF
    
    // initial settings of timing variables
    // ----------------------------------------------------------------------------------------------------
    extern int PBR_ENCODING_SCHEME; // 1/2/4/8 --> FM0/M2/M4/M8

    extern float PBR_TAG_BIT_D; // Duration in us
    extern float PBR_RN16_D;
    extern float PBR_EPC_D;
    extern float PBR_HANDLE_D; // RN16 without? dummy-bit  
    extern float PBR_READ_D; // RN16 without? dummy-bit
    // ---------------------------------------------------------------------------------------------------------
    const int QUERY_CODE[4] = {1,0,0,0};

    
    const int M_FM0[2] = {0,0}; 
    const int M_Miller2[2] = {0,1};
    const int M_Miller4[2] = {1,0};
    const int M_Miller8[2] = {1,1};
    const int SEL[2]         = {0,0};
    const int SESSION[2]     = {0,0};
    const int TARGET         = 0; // 0=A. 1=B 
    const int TREXT         = 0; //Pilot tone
    const int DR            = 0;  //  0 for DR=8, 1 for DR=64/3

    //Slots duration
    const int T_pr = DELIM_D+ 2*PW_D + RTCAL_D + TRCAL_D; //Preamble duration in us
    const int T_FSY = DELIM_D+ 2*PW_D + RTCAL_D; //Frame Sync duration in us
    const float Rdr = 2/(6*PW_D* pow(10,-6)); //Reader data rate (bps)

    const float Tack = T_FSY + (18/Rdr)* pow(10,6) ; //in us
    const float TQR  = T_FSY + (4/Rdr) * pow(10,6) ; //in us
    const float TQA  = T_FSY + (9/Rdr) * pow(10,6) ; //in us
    const float TQ   = T_pr  + (22/Rdr)* pow(10,6) ; //in us
    // -------------------------------------------------------------------------------------
    //extern float Tsk;  //Duration of single/collision slot in us
    //extern float Ti; //Duration of idle slot in us
    // ----------------------------------------------------------------------------------------
    const int NAK_CODE[8]   = {1,1,0,0,0,0,0,0};

    // ACK command
    const int ACK_CODE[2]   = {0,1};

    // QueryAdjust command
    const int QADJ_CODE[4]   = {1,0,0,1};

   
    const int Q_UPDN[7][3]  = { {0,0,0}, {0,0,1}, {0,1,0}, {0,1,1}, {1,0,0}, {1,0,1}, {1,1,0}};

    // Encoding preamble sequences
    const int TAG_PREAMBLE_FM0[] = {1,1,0,1,0,0,1,0,0,0,1,1};
    const int TAG_PREAMBLE_M2[] = {
      1,0,1,0, 1,0,0,1, 0,1,0,1, 0,1,1,0, 1,0,0,1, 0,1,1,0};
    const int TAG_PREAMBLE_M4[] = {
      1,0,1,0,1,0,1,0, 1,0,1,0,0,1,0,1, 0,1,0,1,0,1,0,1, 0,1,0,1,1,0,1,0, 1,0,1,0,0,1,0,1, 0,1,0,1,1,0,1,0};
    const int TAG_PREAMBLE_M8[] = {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
                                   1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,
                                   0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,
                                   0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,
                                   1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,
                                   0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0};

    const int M2_DATA_ONE[] = {1,0,0,1};
    const int M2_DATA_ONE_1[] = {0,1,1,0};
    const int M4_DATA_ONE[] = {1,0,1,0,0,1,0,1};
    const int M8_DATA_ONE[] = {1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1};

    const int M2_ONE_LEN = 4;
    const int M4_ONE_LEN = 8;
    const int M8_ONE_LEN = 16;

    const int FM0_PREAMBLE_LEN = 12;
    const int M2_PREAMBLE_LEN = 24;
    const int M4_PREAMBLE_LEN = 48;
    const int M8_PREAMBLE_LEN = 96;

    const int FM0_PREAMBLE_POWER = 6;
    const int M2_PREAMBLE_POWER = 12;
    const int M4_PREAMBLE_POWER = 24;
    const int M8_PREAMBLE_POWER = 48;
    
    //ACCESS COMMANDS
    const int REQ_RN16_CODE[8] = {1,1,0,0,0,0,0,1};

    const int READ_CODE[8] = {1,1,0,0,0,0,1,0};
    //Parameters for accelerometer READ command
    const int MemBank[2] = {0,1}; //Read EPC
    const int WordPtr [8] = {0,0,0,0,0,1,0,1}; //Read EPC[8],EPC[9],EPC[10],EPC[11]  -- starting adrress pointer--5
    const int Wordcount[8] = {0,0,0,0,0,0,1,0}; //Number of words (16 bits) to read -- 2 words to read

    // Gate block parameters
    const float THRESH_FRACTION = 0.75;     
    const float WIN_SIZE_D         = 75;  

    // Duration in which dc offset is estimated (T1_D is 58)
    const int DC_SIZE_D         =14;

    const float E_th_LIST[] = {0, 0.76, 0.7, 0, 0.6, 0, 0, 0, 0.6};
    extern float PBR_E_th;

    const int index_ES_LIST[] = {8, 4, 2, 1};

  } // namespace rfid
} // namespace gr

#endif /* INCLUDED_RFID_PBR_GLOBAL_VARS_H */

