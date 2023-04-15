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

#ifndef INCLUDED_RFID_GLOBAL_VARS_H
#define INCLUDED_RFID_GLOBAL_VARS_H

#include <rfid/api.h>
#include <map>
#include <sys/time.h>

namespace gr {
  namespace rfid {

    enum STATUS             {RUNNING, TERMINATED};
    enum GEN2_LOGIC_STATUS  {CW_AUX, POWER_UP_CW, SEND_QUERY, SEND_ACK, SEND_QUERY_REP, IDLE, SEND_CW_ACK, SEND_CW_QUERY, SEND_CW_REQ, SEND_CW_READ, START, SEND_QUERY_ADJUST, SEND_REQ_RN16,SEND_READ, SEND_NAK_QR, SEND_NAK_Q, POWER_DOWN}; 
    enum GATE_STATUS        {GATE_OPEN, GATE_CLOSED, GATE_SEEK_RN16, GATE_SEEK_EPC, GATE_SEEK_HANDLE, GATE_SEEK_READ};  
    enum DECODER_STATUS     {DECODER_DECODE_RN16, DECODER_DECODE_EPC, DECODER_DECODE_HANDLE, DECODER_DECODE_READ};
    
    struct READER_STATS
    {
      int n_queries_sent;
      int n_powerup_sent;
      int n_epc_detected;
      int aux_buffer_flag;
      int n_consecutive_success;
      int n_consecutive_failure;
      int prev_transmission_state; // 0 fail, 1 success
      int curr_transmission_state; // 0 fail, 1 success
      int index_ES;
      int just_elevated;    
      clock_t start_time;
      clock_t end_time;
      clock_t previous_time;
      float average_throughput;
      float noise_power;
      float bs_power;
      float power_up_delay;
      int cur_inventory_round;
      int cur_slot_number;
      int max_slot_number;
      int max_inventory_round;
      int n_epc_correct;
      float output_energy;
      int n_0;
      int n_1;
      int n_k;
      int tn_k;  
      int tn_1;
      int tn_0;
      int tQA;
      int tQR;
      int tQ;
      int sensor_read;
      int VAR_Q;   
      int Qant; 
      float Qfp; 
      int Qupdn;
      float it_timer;
      float th;
      float TIR_th;
      float TIR_exp;
      int stop;
      
      std::vector<int>  unique_tags_round;
      std::map<int,int> tag_reads;   
      std::vector<float> RN16_bits_handle; 
      std::vector<float> RN16_bits_read;
      std::vector<gr_complex> aux_EPC_samples_complex;
      int aux_EPC_index;

      struct timeval start, end; 
    };

    struct READER_STATE
    {
      STATUS               status;
      GEN2_LOGIC_STATUS   gen2_logic_status;
      GATE_STATUS         gate_status;
      DECODER_STATUS       decoder_status;
      READER_STATS         reader_stats;

      std::vector<float> magn_squared_samples; // used for sync
      int n_samples_to_ungate; // used by the GATE and DECODER block
    };
    
    extern float RN16_handle_stored[16];

    extern READER_STATE * reader_state;
    extern void initialize_reader_state();

    // CONSTANTS (READER CONFIGURATION)

    // Fixed number of slots (2^(FIXED_Q))  
    const int FIXED_Q = 0;
    const float C = 0.2;

    // Termination criteria
    
    const int MAX_NUM_QUERIES     = 29000;     //(11000)3000+3000+3000+3000+3000 Stop after MAX_NUM_QUERIES have been sent

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
    const int CW_D         = 2500;    // Carrier wave
    const int P_DOWN_D     = 2000;    // power down
    
    const int T1_D         = 56;    // Time from Interrogator transmission to Tag response (62.5 us)
    const int T2_D         = 120;    // Time from Tag response to Interrogator transmission. Max value = 20.0 * T_tag = 125us 

    const float PW_D         = 6.25;      // Half Tari 

    const float DELIM_D       = 12.5;      // A preamble shall comprise a fixed-length start delimiter 12.5us +/-5%
    const float TRCAL_D     = 50;    // BLF = DR/TRCAL => 133.33 for DR=1(64/3), 50 for DR=0(8)
    const int RTCAL_D     = 36;      // 31.25for data_1 = 1.5*data_0; 37.5 for data_1=2*data_0

    const int NUM_PULSES_COMMAND = 4;       // 4Number of pulses to detect a reader command // 
    const int NUMBER_UNIQUE_TAGS = 500;      // Stop after NUMBER_UNIQUE_TAGS have been read 

    // Number of bits
    const int PILOT_TONE         = 12;  // Optional
    const int TAG_PREAMBLE_BITS  = 6;   // Number of preamble bits
    const int RN16_BITS          = 17;  // Dummy bit at the end
    const int EPC_BITS           = 129;  // PC + EPC + CRC16 + Dummy => (6--preamble) + 16 + 96 + 16 + 1 = 135
    const int QUERY_LENGTH       = 22;  // Query length in bits
    
    const int T_READER_FREQ = 160e3;     // BLF
    
    // initial settings of timing variables
    // ----------------------------------------------------------------------------------------------------
    extern int ENCODING_SCHEME; // 1/2/4/8 --> FM0/M2/M4/M8
    extern float TAG_BIT_D; // Duration in us
    extern float RN16_D;
    extern float EPC_D;
    extern float HANDLE_D; // RN16 without? dummy-bit  
    extern float READ_D; // RN16 without? dummy-bit
    // ---------------------------------------------------------------------------------------------------------
    const int QUERY_CODE[4] = {1,0,0,0};

    const int M_FM0[2] = {0,0}; 
    const int M_Miller2[2] = {0,1};
    const int M_Miller4[2] = {1,0};
    const int M_Miller8[2] = {1,1};
    const int SEL[2]         = {0,0};
    const int SESSION[2]     = {0,0};
    extern int TARGET; // 0=A. 1=B // retransmission request (0/1)
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
    extern float Tsk;  //Duration of single/collision slot in us
    extern float Ti; //Duration of idle slot in us
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

    const int M2_PilotTone[] = {1,0,1,0, 1,0,1,0, 1,0,1,0, 1,0,1,0};
    const int M2_PilotTone_LEN = 16;

    const int M2_S0[] = {1,0,1,0};
    const int M2_S1[] = {0,1,0,1};
    const int M2_S2[] = {1,0,0,1};
    const int M2_S3[] = {0,1,1,0};

    const int M4_S0[] = {1,0,1,0, 1,0,1,0};
    const int M4_S1[] = {0,1,0,1, 0,1,0,1};
    const int M4_S2[] = {1,0,1,0, 0,1,0,1};
    const int M4_S3[] = {0,1,0,1, 1,0,1,0};

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
    const int DC_SIZE_D         = 28;

    // Thresholds
    const float C_th_LIST[] = {0, 0.90, 0.81, 0, 0.64, 0, 0, 0, 0.60};
    extern float C_th;
    const float E_th = 0.000;
    
    // Encoding Scheme Index List
    const int index_ES_LIST[] = {8, 4, 2, 1};
    const int Retran_Winsize[4] = {1, 1, 2, 3};
    // Fixed Encoding Scheme params
    const int FIXED_RATE_EN = 1;

    // Minstrel params
    const int MINSTREL_EN = 0;
    extern clock_t minstrel_timer;
    extern float pktloss_table[4];
    extern int PROBING_MODE;
    extern int probing_rate_cnt; // rate index
    extern int probing_cnt; // count of single rate
    extern int n_success;
    extern clock_t dwelling_clk_start;
    
    // Auto Rate Fallback params
    const int AUTO_RATE_FALLBACK_EN = 0;
    extern int index_ES;
    extern int prev_transmission_state;
    extern int curr_transmission_state;
    extern int n_consecutive_success;
    extern int n_consecutive_failure;
    extern int just_elevated;
    extern int valid_packet;
    
    // BLINK params
    const int BLINK_EN = 0;
    const float Th_RSSI = 0.20;
    const float Th_pktloss = 0.15;
    extern int BLINK_MODE; // (0: KEEP, 1: PROBING)
    extern int blink_scan_start;
    extern int blink_scan_cnt;
    extern clock_t blink_timer;
    extern float blink_n_success;
    extern float blink_n_failure;
    extern float blink_pktloss_realtime;
    extern float blink_pktloss_firstscan;
    extern float blink_pktloss_secondscan;
    extern float blink_RSSI_firstscan;
    extern float blink_RSSI_secondscan;
    extern float blink_pktloss_probe;
    extern float blink_RSSI_probe;
    
    // MobiRate params
    const int MobiRate_EN = 0;
    extern float lambda; // exponential coefficient
    extern int MobiRate_MODE; // (0: KEEP, 1: HIGHER, 2: LOWER) 
    extern clock_t mobirate_timer1;
    extern clock_t mobirate_timer2;
    extern float mobirate_phase_realtime;
    extern int mobirate_phase_scan_cnt;
    extern int mobirate_n_failure;
    extern float mobirate_pktloss_table[4];
    const float PI = 3.14159;  
    const float WAVLEN = 30000.0 / (915 * 4 * PI); 
    
    // Signal params
    extern float sig_power;
    extern float RSSI;
    extern float Phase;
    extern float NoiseI;
    extern int cnt_RSSI;
    extern int cnt_NoiseI;
    extern int cnt_queries;
    extern int cnt_preamble_collected;
    extern float cw_ampl;
    extern float rta_ampl;
    extern gr_complex preamble_fm0[6 * 14 * 100];
    extern gr_complex preamble_m8[6 * 14 * 8];
    extern int cnt_power_up_delay;

    // AdaBS params
    const int ADABS_EN = 0;
    // User available params
    const float OBJ_THROUGHPUT = 20; // reads/s
    const float OBJ_THROUGHPUT_MARGIN = 0.2; // 5~20% is reasonable
    const float TP_UPPER_LIMIT = (1 + 0.2) * OBJ_THROUGHPUT;
    const float TP_LOWER_LIMIT = (1 - OBJ_THROUGHPUT_MARGIN) * OBJ_THROUGHPUT;
    const float MIN_THROUGHPUT = 100; // (bps) 50~129 is reasonable
    const int ADA_WIN_LEN_EN = 0; // whether to enable adaptive time window length
    const int ADA_WIN_LEN_K = 3; // Th of doubling window length
    const int ADA_WIN_LEN_P = 2; // Th of halving window length
    
    // User unavailable params
    extern int INFERRED;
    extern float OBJ_G_PREV;
    extern float PUD_PREV;
    extern float RSSI_PREV;
    extern float NOISEI_PREV;
    const float TP_MONITOR_INTERVAL = 129.0 / MIN_THROUGHPUT;
    extern int H_timeWinLen;  
    extern int winIndex;
    extern float THROUGHPUT_MONITOR_EN;
    extern float THROUGHPUT_AVAILABLE;
    extern float Td_MONITORED;
    extern int ADABS_PROBING_MODE;
    extern int ADABS_PROBING_DONE;
    extern int INFERENCE_RESULTS_AVAILABLE;
    extern int cnt_correct_epc;
    extern int cnt_loss_epc;
    extern int cnt_loss_epc_global;
    extern float pktCorrectRatio;
    extern float tp_now;
    extern float throughput_monitored;
    extern struct timespec tp_monitor_st, tp_monitor_ed;
    const int RFID_LOCALIZATION = 1;
    const int DATA_COLLECTION_EN = 0;
    const int EXAUSTIVE_SEARCH = 0; //exHaustive
    extern int adabs_probing_en;
    extern int adabs_nn_en;

    extern float adabs_throughput_table[4]; //FM0/M2/M4/M8
    extern float adabs_lossrate_table[4];
    extern float adabs_goodput_table[4];
    extern float adabs_probe_RSSI;
    extern float adabs_probe_NoiseI;
    extern float adabs_probe_powerup_delay;
    extern struct timespec tv_start, tv_end;
    extern struct timespec start_time, end_time, previous_time;
    extern std::vector<float> tp_record;
    extern std::vector<double> t_record;
    extern std::vector<float> p_record;
    extern float amp_inference_raw;
    extern std::vector<double> t_preamble_record;
    extern double total_time;
    const float preamble_xf_roi[] = {
        2.7166994e-02,
        2.8061297e-02,
        3.0451524e-02,
        3.3325145e-02,
        3.5374872e-02,
        3.6316973e-02,
        3.6891438e-02,
        3.7954189e-02,
        4.0419560e-02,
        4.5567103e-02,
        5.5338395e-02,
        7.2157674e-02,
        9.4598276e-02,
        1.1017986e-01,
        1.1718183e-01,
        1.3268087e-01,
        1.7574574e-01,
        2.9323869e-01,
        7.0947340e-01,
        7.2960464e-01,
        3.9173959e-01,
        2.9749013e-01,
        2.7988147e-01,
        3.0827931e-01,
        3.6842781e-01,
        4.1007985e-01,
        4.0294551e-01,
        3.9871161e-01
    };

    // DNN inputs
    //extern std::vector<std::vector <float> > Hft;
    extern float goodput_monitored;
    extern std::vector<float> Hft;
    extern std::vector<float> Hft_PREV;
    extern float RSSI_probed;
    extern float NoiseI_probed;
    extern float Powerup_Delay_probed;
    extern int cnt_tp_monitored;
    //extern float Powerup_Delay_probed;
    extern float amp_inference;
    extern int es_inference; //FM0/M2/M4/M8 -> 0/1/2/3
    extern int bulky_N;
    extern int cnt_inference;
    extern int cnt_fft;
    extern float E_Tx;
    extern float accumulative_d_monitor;
    // retransmission params
    extern int retran_is_pkt_loss;
    extern int retran_goodput_cnt;
    extern float retran_delay;
    extern struct timespec retran_end_time, retran_previous_time;
    extern int retran_i_window;
    extern float retran_gp_now;
    extern float retran_gp_ave;
    extern int retran_prev_goodput_cnt;
    extern float retran_total_time;
    extern float retran_start_time;
    extern int retran_goodput_pkt_cnt;
    extern int retran_goodput_pkt_prev_cnt;
  } // namespace rfid
} // namespace gr

#endif /* INCLUDED_RFID_GLOBAL_VARS_H */

