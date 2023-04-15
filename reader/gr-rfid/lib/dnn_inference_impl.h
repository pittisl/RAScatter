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

#ifndef INCLUDED_RFID_DNN_INFERENCE_IMPL_H
#define INCLUDED_RFID_DNN_INFERENCE_IMPL_H

#include <rfid/dnn_inference.h>
#include "rfid/global_vars.h"
#include <fstream>
#include "../tflib/include/Model.h"
#include "../tflib/include/Tensor.h"
#include <volk/volk.h>
#include <numeric>
#include <iomanip>

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>

namespace gr {
  namespace rfid {

    class dnn_inference_impl : public dnn_inference
    {
     private:
	 typedef struct model_t {
	     TF_Graph* graph;
	     TF_Session* session;
	     TF_Status* status;
	
	     TF_Output channel_response, power_up_delay, rssi, noisei, obj_throughput, act_throughput, amp, es_scores;
	
	     TF_Operation *init_op, *train_op, *save_op, *restore_op;
	     TF_Output checkpoint_file;
      } model_t;
      model_t model;
	  enum SaveOrRestore { SAVE, RESTORE };
	  /*
      Model model{"../apps/Model/NN23/frozen_model.pb"};
      Tensor channelResponseProbe{model, "CHANNEL_RESPONSE"};
      Tensor noiseiProbe{model, "NOISEI"};
      Tensor rssiProbe{model, "RSSI"};
      Tensor powerupDelayProbe{model, "POWER_UP_DELAY"};
      Tensor objThroughput{model, "OBJ_THROUGHPUT"};
      Tensor ampScalarPred{model, "AdaBackscatterNet_v6/amp"};
      Tensor encodingSchemeScore{model, "AdaBackscatterNet_v6/es_scores"};
	  */
      float AMPs[5] = {0.0};
      int ESs[5] = {0};
      float BpJs[5] = {0};
      int dnn_t_measure = 0;
	  
	    int ModelCreate(model_t* model, const char* graph_def_filename);
      void ModelDestroy(model_t* model);
      int ModelInit(model_t* model);
      /* TO DO: batch needs to be splitted */
      int ModelPredict(model_t* model, 
                 float* batch_chrsp, 
				 float* batch_pud,
				 float* batch_rssi,
				 float* batch_noisei,
				 float* batch_obj_tp,
				 float* batch_amp,
				 float* batch_es,
				 int batch_size);
      void NextBatchForTraining(TF_Tensor** chrsp_tensor, 
                          TF_Tensor** pud_tensor,
						  TF_Tensor** rssi_tensor,
						  TF_Tensor** noisei_tensor,
						  TF_Tensor** obj_tp_tensor,
						  TF_Tensor** act_tp_tensor);
      int ModelRunTrainStep(model_t* model);
      int ModelCheckpoint(model_t* model, const char* checkpoint_prefix, int type);
      int Okay(TF_Status* status);
      TF_Buffer* ReadFile(const char* filename);
      TF_Tensor* ScalarStringTensor(const char* data, TF_Status* status);
      int DirectoryExists(const char* dirname);
      //int LoadModel(model_t* model);
	  
     public:
      dnn_inference_impl();
      ~dnn_inference_impl();
	  
      // Where all the action really happens
      int work(int noutput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
    };

  } // namespace rfid
} // namespace gr

#endif /* INCLUDED_RFID_DNN_INFERENCE_IMPL_H */

