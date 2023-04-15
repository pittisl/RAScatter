/* -*- c++ -*- */
/* 
 * Copyright 2020 <+YOU OR YOUR COMPANY+>.
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
#include "dnn_inference_impl.h"

namespace gr {
  namespace rfid {

    dnn_inference::sptr
    dnn_inference::make()
    {
      return gnuradio::get_initial_sptr
        (new dnn_inference_impl());
    }

    /*
     * The private constructor
     */
    dnn_inference_impl::dnn_inference_impl()
      : gr::sync_block("dnn_inference",
              gr::io_signature::make(1, 1, sizeof(gr_complex) * 256),
              gr::io_signature::make(0, 0, 0))
    {
      const int alignment_multiple = volk_get_alignment() / sizeof(gr_complex) / 256;
      set_alignment(std::max(1, alignment_multiple));
      clock_gettime(CLOCK_MONOTONIC, &tv_start);
	  // load model
	        
		    const char* graph_def_filename = "Model/model.pb";
	        const char* checkpoint_prefix = "Model/model.ckpt-500"; // TO DO: check the path
	        int restore = DirectoryExists("Model");
	        
	        
	        printf("Loading graph\n");
	        if (!ModelCreate(&model, graph_def_filename)){};
	        if (restore) {
		        printf("Restoring weights from checkpoint (remove the checkpoints directory to reset)\n");
                if (!ModelCheckpoint(&model, checkpoint_prefix, RESTORE)){} //return 1;
	        } else {
		        printf("Initializing model weights\n");
		        if (!ModelInit(&model)) {
			        //return 1;
		        }
	        }
    }

    /*
     * Our virtual destructor.
     */
    dnn_inference_impl::~dnn_inference_impl()
    {
    }
	
	// list your functions from here (remember to fill up the .h file)
	// ...
	
	int dnn_inference_impl::ModelCreate(model_t* model, const char* graph_def_filename) {
	    model->status = TF_NewStatus();
	    model->graph = TF_NewGraph();
	    {
		    // Create the session
		    TF_SessionOptions* opts = TF_NewSessionOptions();
		    model->session = TF_NewSession(model->graph, opts, model->status);
		    TF_DeleteSessionOptions(opts);
		    if (!Okay(model->status)) return 0;
	    }
	
	    TF_Graph* g = model->graph;
	
	    {
		    // Import the graph
		    TF_Buffer* graph_def = ReadFile(graph_def_filename);
		    if (graph_def == NULL) return 0;
		    printf("Read GraphDef of %zu bytes\n", graph_def->length);
		    TF_ImportGraphDefOptions* opts = TF_NewImportGraphDefOptions();
		    TF_GraphImportGraphDef(g, graph_def, opts, model->status);
		    TF_DeleteImportGraphDefOptions(opts);
		    TF_DeleteBuffer(graph_def);
		    if (!Okay(model->status)) return 0; 
	    }
	
	    // Handles to the interesting operations in the graph.
	    model->channel_response.oper = TF_GraphOperationByName(g, "CHANNEL_RESPONSE");
	    model->channel_response.index = 0;
	    model->power_up_delay.oper = TF_GraphOperationByName(g, "POWER_UP_DELAY");
	    model->power_up_delay.index = 0;
	    model->rssi.oper = TF_GraphOperationByName(g, "RSSI");
	    model->rssi.index = 0;
	    model->noisei.oper = TF_GraphOperationByName(g, "NOISEI");
	    model->noisei.index = 0;
	    model->obj_throughput.oper = TF_GraphOperationByName(g, "OBJ_THROUGHPUT");
	    model->obj_throughput.index = 0;
	    model->act_throughput.oper = TF_GraphOperationByName(g, "ACTUAL_THROUGHPUT");
	    model->act_throughput.index = 0;
	    model->amp.oper = TF_GraphOperationByName(g, "AdaBackscatterNet_v7/amp");
	    model->amp.index = 0;
	    model->es_scores.oper = TF_GraphOperationByName(g, "AdaBackscatterNet_v7/es_scores");
	    model->es_scores.index = 0;
	
	    model->init_op = TF_GraphOperationByName(g, "init");
	    model->train_op = TF_GraphOperationByName(g, "train_online");
	
	    model->save_op = TF_GraphOperationByName(g, "save/control_dependency");
	    //model->save_op.index = 0;
	
	    model->restore_op = TF_GraphOperationByName(g, "save/restore_all");
	
	    model->checkpoint_file.oper = TF_GraphOperationByName(g, "save/Const");
	    model->checkpoint_file.index = 0;
	
	    return 1;
    }

    void dnn_inference_impl::ModelDestroy(model_t* model) {
	    TF_DeleteSession(model->session, model->status);
	    Okay(model->status);
	    TF_DeleteGraph(model->graph);
	    TF_DeleteStatus(model->status);
    }

    int dnn_inference_impl::ModelInit(model_t* model) {
	    const TF_Operation* init_op[1] = {model->init_op};
	    TF_SessionRun(model->session, NULL,
	        /* No inputs */
			NULL, NULL, 0,
			/* No outputs */
			NULL, NULL, 0,
			/* Just the init operation */
			init_op, 1,
			/* No metadata */
			NULL, model->status);
	    return Okay(model->status);
    }
	
    int dnn_inference_impl::ModelCheckpoint(model_t* model, const char* checkpoint_prefix, int type) {
	    TF_Tensor* t = ScalarStringTensor(checkpoint_prefix, model->status);
	    if (!Okay(model->status)) {
		    TF_DeleteTensor(t);
		    return 0;
	    }
	    TF_Output inputs[1] = {model->checkpoint_file};
	    TF_Tensor* input_values[1] = {t};
	    const TF_Operation* op[1] = {type == SAVE ? model->save_op
	                                           : model->restore_op};
	    TF_SessionRun(model->session, NULL, inputs, input_values, 1,
	              /* No outputs */
				  NULL, NULL, 0,
				  /* The operation */
				  op, 1, NULL, model->status);
	    TF_DeleteTensor(t);
	    return Okay(model->status);
    }

    int dnn_inference_impl::ModelPredict(model_t* model, 
                 float* batch_chrsp, 
				 float* batch_pud,
				 float* batch_rssi,
				 float* batch_noisei,
				 float* batch_obj_tp,
				 float* batch_amp,
				 float* batch_es,
				 int batch_size) {
	    // batch consists of ?x? matrices
	    const int64_t channel_response_dims[4] = {batch_size, 4, 28, 1};
	    const int64_t power_up_delay_dims[2] = {batch_size, 1};
	    const int64_t rssi_dims[2] = {batch_size, 1};
	    const int64_t noisei_dims[2] = {batch_size, 1};
	    const int64_t obj_throughput_dims[2] = {batch_size, 1};
	    //const int64_t act_throughput_dims[2] = {batch_size, 1};
	    const int64_t amp_dims[2] = {batch_size, 1};
	    const int64_t es_scores_dims[2] = {batch_size, 4};
	    const size_t nbytes_1 = 112 * batch_size * sizeof(float);
	    const size_t nbytes_2 = batch_size * sizeof(float);
	    const size_t nbytes_3 = 4 * batch_size * sizeof(float);
	
	    TF_Tensor* channel_response_t = TF_AllocateTensor(TF_FLOAT, channel_response_dims, 4, nbytes_1);
	    memcpy(TF_TensorData(channel_response_t), batch_chrsp, nbytes_1);
	    
	    TF_Tensor* power_up_delay_t = TF_AllocateTensor(TF_FLOAT, power_up_delay_dims, 2, nbytes_2);
	    memcpy(TF_TensorData(power_up_delay_t), batch_pud, nbytes_2);
	    
	    TF_Tensor* rssi_t = TF_AllocateTensor(TF_FLOAT, rssi_dims, 2, nbytes_2);
	    memcpy(TF_TensorData(rssi_t), batch_rssi, nbytes_2);
	    
	    TF_Tensor* noisei_t = TF_AllocateTensor(TF_FLOAT, noisei_dims, 2, nbytes_2);
	    memcpy(TF_TensorData(noisei_t), batch_noisei, nbytes_2);
	    
	    TF_Tensor* obj_throughput_t = TF_AllocateTensor(TF_FLOAT, obj_throughput_dims, 2, nbytes_2);
	    memcpy(TF_TensorData(obj_throughput_t), batch_obj_tp, nbytes_2);
	    
	    TF_Output inputs[5] = {model->channel_response,
	                       model->power_up_delay,
						   model->rssi,
						   model->noisei,
						   model->obj_throughput};
	    TF_Tensor* input_values[5] = {channel_response_t,
	                              power_up_delay_t,
								  rssi_t,
								  noisei_t,
								  obj_throughput_t};
	    TF_Output outputs[2] = {model->amp, model->es_scores};
	    TF_Tensor* output_values[2] = {NULL, NULL};
	    
	    TF_SessionRun(model->session, NULL,
	              inputs, input_values, 5,
				  outputs, output_values, 2,
				  /* No target operations to run */ /* TO DO: investigate params' meanings*/
				  NULL, 0, NULL, model->status);
	
	    TF_DeleteTensor(channel_response_t);
	    TF_DeleteTensor(power_up_delay_t);
	    TF_DeleteTensor(rssi_t);
	    TF_DeleteTensor(noisei_t);
	    TF_DeleteTensor(obj_throughput_t);
	    if (!Okay(model->status)) return 0;
	    
	    if (TF_TensorByteSize(output_values[0]) != nbytes_2) {
		    fprintf(stderr,
		        "ERROR: Expected predictions tensor to have %zu bytes, has %zu\n",
				nbytes_2, TF_TensorByteSize(output_values[0]));
		    //TF_DeleteTensor(amp_val[0]);
		    TF_DeleteTensor(output_values[0]);
		    return 0;
	    }
	
	    // use global vals & needs some transformations
	    float* amp_pred = (float*)malloc(nbytes_2);
	    float* es_scores_pred = (float*)malloc(nbytes_3);
	    
	    memcpy(amp_pred, TF_TensorData(output_values[0]), nbytes_2);
	    memcpy(es_scores_pred, TF_TensorData(output_values[1]), nbytes_3);
	    
	    TF_DeleteTensor(output_values[0]);
	    //TF_DeleteTensor(es_scores_val[0]);
	    
	    //printf("Predictions:\n");
		batch_amp[0] = amp_pred[0];
		float max_t = 0;
		int es_ind = 0;
		for (int i = 0; i < 4; i++) {
			if (es_scores_pred[i] > max_t) {
				max_t = es_scores_pred[i];
				es_ind = i;
			}
		}
		batch_es[0] = es_ind;
		/*
	    for (int i = 0; i < batch_size; ++i) {
		    // TO DO: add es scores [1 2 3 4]
		    printf("\t amp = %f\n", amp_pred[i]);
	    }
		*/
		
	    free(amp_pred);
	    free(es_scores_pred);
	    return 1;
    }

    void dnn_inference_impl::NextBatchForTraining(TF_Tensor** chrsp_tensor, 
                          TF_Tensor** pud_tensor,
						  TF_Tensor** rssi_tensor,
						  TF_Tensor** noisei_tensor,
						  TF_Tensor** obj_tp_tensor,
						  TF_Tensor** act_tp_tensor) {
#define BATCH_SIZE 1
    // TO DO: Here prepares the training data
	
	    //float chrsp[BATCH_SIZE * 4 * 28 * 1] = {0};
		float chrsp[112 * BATCH_SIZE]= {0};
        float pud[BATCH_SIZE] = {PUD_PREV};
	    float rssi[BATCH_SIZE] = {RSSI_PREV};
	    float noisei[BATCH_SIZE] = {NOISEI_PREV};
	    float obj_tp[BATCH_SIZE] = {OBJ_G_PREV};
	    float act_tp[BATCH_SIZE] = {goodput_monitored};

		for (int i = 0; i < 112; i++) {
			chrsp[i] = Hft_PREV[i];
		}
	    
	    const int64_t dims_1[4] = {BATCH_SIZE, 4, 28, 1};
	    const int64_t dims_2[2] = {BATCH_SIZE, 1};
	    size_t nbytes_1 = 4 * 28 * BATCH_SIZE * sizeof(float);
	    size_t nbytes_2 = BATCH_SIZE * sizeof(float);
	    
	    *chrsp_tensor = TF_AllocateTensor(TF_FLOAT, dims_1, 4, nbytes_1);
	    *pud_tensor = TF_AllocateTensor(TF_FLOAT, dims_2, 2, nbytes_2);
	    *rssi_tensor = TF_AllocateTensor(TF_FLOAT, dims_2, 2, nbytes_2);
	    *noisei_tensor = TF_AllocateTensor(TF_FLOAT, dims_2, 2, nbytes_2);
	    *obj_tp_tensor = TF_AllocateTensor(TF_FLOAT, dims_2, 2, nbytes_2);
	    *act_tp_tensor = TF_AllocateTensor(TF_FLOAT, dims_2, 2, nbytes_2);
	    
	    memcpy(TF_TensorData(*chrsp_tensor), chrsp, nbytes_1);
	    memcpy(TF_TensorData(*pud_tensor), pud, nbytes_2);
	    memcpy(TF_TensorData(*rssi_tensor), rssi, nbytes_2);
	    memcpy(TF_TensorData(*noisei_tensor), noisei, nbytes_2);
	    memcpy(TF_TensorData(*obj_tp_tensor), obj_tp, nbytes_2);
	    memcpy(TF_TensorData(*act_tp_tensor), act_tp, nbytes_2);
	
#undef BATCH_SIZE
    }

    int dnn_inference_impl::ModelRunTrainStep(model_t* model) {
	    TF_Tensor *chrsp, *pud, *rssi, *noisei, *obj_tp, *act_tp;
	    NextBatchForTraining(&chrsp, &pud, &rssi, &noisei, &obj_tp, &act_tp);
	    TF_Output inputs[6] = {model->channel_response,
	                       model->power_up_delay,
						   model->rssi,
						   model->noisei,
						   model->obj_throughput,
						   model->act_throughput};
	    TF_Tensor* input_values[6] = {chrsp, pud, rssi, noisei, obj_tp, act_tp};
	    const TF_Operation* train_op[1] = {model->train_op};
        
		 //clock_gettime(CLOCK_MONOTONIC, &tv_start); 
		  TF_SessionRun(model->session, NULL, inputs, input_values, 6,
	              /* No outputs */
				  NULL, NULL, 0, train_op, 1, NULL, model->status);
		 
		 // clock_gettime(CLOCK_MONOTONIC, &tv_end);
		 /*
		  double time_taken; 
          time_taken = (tv_end.tv_sec - tv_start.tv_sec) * 1e9; 
          time_taken = (time_taken + (tv_end.tv_nsec - tv_start.tv_nsec)) * 1e-9; 
		  std::cout << "Time taken by program is : " << std::fixed 
         << time_taken << std::setprecision(9); 
         std::cout << " sec" << std::endl;
         */
	    


	    
	    TF_DeleteTensor(chrsp);
	    TF_DeleteTensor(rssi);
	    TF_DeleteTensor(noisei);
	    TF_DeleteTensor(obj_tp);
	    TF_DeleteTensor(act_tp);
	    return Okay(model->status);
    }

    int dnn_inference_impl::Okay(TF_Status* status) {
	    if (TF_GetCode(status) != TF_OK) {
		    fprintf(stderr, "ERROR: %s\n", TF_Message(status));
		    return 0;
	    }
	    return 1;
    }

    TF_Buffer* dnn_inference_impl::ReadFile(const char* filename) {
	    int fd = open(filename, 0);
	    if (fd < 0) {
		    perror("failed to open file: ");
		    return NULL;
	    }
	    struct stat stat;
	    if (fstat(fd, &stat) != 0) {
		    perror("failed to read file: ");
		    return NULL;
	    }
	    char* data = (char*)malloc(stat.st_size);
	    ssize_t nread = read(fd, data, stat.st_size);
	    if (nread < 0) {
		    perror("failed to read file: ");
		    free(data);
		    return NULL;
	    }
	    if (nread != stat.st_size) {
		    fprintf(stderr, "read %zd bytes, expected to read %zd\n", nread, stat.st_size);
		    free(data);
		    return NULL;
	    }
	    TF_Buffer* ret = TF_NewBufferFromString(data, stat.st_size);
	    free(data);
	    return ret;
    }

    TF_Tensor* dnn_inference_impl::ScalarStringTensor(const char* str, TF_Status* status) {
        size_t nbytes = 8 + TF_StringEncodedSize(strlen(str));
        TF_Tensor* t = TF_AllocateTensor(TF_STRING, NULL, 0, nbytes);
        void* data = TF_TensorData(t);
        memset(data, 0, 8);  // 8-byte offset of first string.
        TF_StringEncode(str, strlen(str), (char*)(data) + 8, nbytes - 8, status);
        return t;
    }

    int dnn_inference_impl::DirectoryExists(const char* dirname) {
        struct stat buf;
        return stat(dirname, &buf) == 0;
    }
	/*
    int dnn_inference_impl::LoadModel() {
		
	}
*/
	// functions end here
	
    int
    dnn_inference_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
		
	    if (0) {
			printf("START TRAINING WITH ONE SAMPLE:\n");
		 
		  //printf("Train once\n");

		  ModelRunTrainStep(&model);

		  //printf("Saving checkpoint\n");
	      //if (!ModelCheckpoint(&model, checkpoint_prefix, SAVE)) return 1;
	      //ModelDestroy(&model);
		  INFERRED = 0;
		  printf("DONE TRAINING WITH ONE SAMPLE\n");
	  }
      // TO DO: adaptive window

      // probing is enabled and chrsp hasn't been filled up
      if (ADABS_PROBING_MODE == 1 && winIndex != H_timeWinLen) {
        for (int i = 0; i < noutput_items; i++) {
          for (int j = 0; j < 28; j++) {
              Hft[j + winIndex] = std::abs(in[j + 128]) * preamble_xf_roi[j];
          }
        }
        winIndex += 28;
      }
     
      // chrsp is completely loaded and other params are ready (get inference cmd from reader)
      if (winIndex == H_timeWinLen && adabs_nn_en == 1) {
        cnt_fft += 4;
		printf("START INFERENCE:\n");
		// load model
		
		
        // Estimate where is the peak by getting one more sample 
        float G0 = OBJ_THROUGHPUT;
        float G1 = OBJ_THROUGHPUT * 1.10;
		
		//float chrsp[4 * 28] = {0}; // blurrrr
	    float pud[1] = {Powerup_Delay_probed};
	    float rssi[1] = {RSSI};
	    float noisei[1] = {NoiseI};
	    float obj_tp[1] = {G0};
		float amp_p[1] = {1}; // max power
		float es_p[1] = {0}; // fastest rate
		
		PUD_PREV = Powerup_Delay_probed;
		RSSI_PREV = RSSI;
		NOISEI_PREV = NoiseI;
		
	    // printf("Initial predictions\n");
	    ModelPredict(&model, &Hft[0], &pud[0], &rssi[0], &noisei[0], &obj_tp[0], &amp_p[0], &es_p[0], 1);
		AMPs[0] = amp_p[0];
		ESs[0] = es_p[0];
		
        // Get one more sample
		obj_tp[0] = G1;
		ModelPredict(&model, &Hft[0], &pud[0], &rssi[0], &noisei[0], &obj_tp[0], &amp_p[0], &es_p[0], 1);
		AMPs[1] = amp_p[0];
		ESs[1] = es_p[0];
		
        // Compare BpJ of two samples
        BpJs[0] = G0 / (AMPs[0] * AMPs[0]);
        BpJs[1] = G1 / (AMPs[1] * AMPs[1]);
        if (BpJs[0] > BpJs[1]) {
          // Peak is at left. The point is already optimal.
		  OBJ_G_PREV = G0;
          amp_inference = AMPs[0];
          es_inference = ESs[0];
          cnt_inference += 1;
        } else {
          // Peak is at right. Do further search.
          // Compute search upper bound
          float coeff_k = (G1 - G0) / (AMPs[1] - AMPs[0]);
          float coeff_b = coeff_k * AMPs[0] - G0;

          float ub = 2 * coeff_b - G1;
          float g_step = (ub - G1) / 3;

          if (g_step > 0) {

            float max_BpJ = BpJs[1];
            int max_point = 1;

            for (int k = 2; k < 5; k++) {
			  obj_tp[0] = G1 + g_step * (k - 1);
			  ModelPredict(&model, &Hft[0], &pud[0], &rssi[0], &noisei[0], &obj_tp[0], &amp_p[0], &es_p[0], 1);
              cnt_inference += 1;
        
              AMPs[k] = amp_p[0];
              ESs[k] = es_p[0];
              BpJs[k] = (G1 + g_step * (k - 1)) / (AMPs[k] * AMPs[k]);
			  
              if (BpJs[k] > max_BpJ) {
                max_BpJ = BpJs[k];
                max_point = k;
				OBJ_G_PREV = G1 + g_step * (k - 1);
              }
            }
            amp_inference = AMPs[max_point];
            es_inference = ESs[max_point];
          }
        }
        
       /*
        clock_gettime(CLOCK_MONOTONIC, &tv_end); 
  
        // Calculating total time taken by the program. 
        double time_taken; 
        time_taken = (tv_end.tv_sec - tv_start.tv_sec) * 1e9; 
        time_taken = (time_taken + (tv_end.tv_nsec - tv_start.tv_nsec)) * 1e-9; 
  
        std::cout << "Time taken by program is : " << std::fixed 
         << time_taken << std::setprecision(9); 
        std::cout << " sec" << std::endl;
      */
        //std::cout << "Time window index : " << winIndex << " Run once!" << std::endl;
        
        winIndex = 0;
        ADABS_PROBING_MODE = 0;
        INFERENCE_RESULTS_AVAILABLE = 1;
        ADABS_PROBING_DONE = 0;
        adabs_nn_en = 0;
		amp_inference_raw = amp_inference;
        if (amp_inference < 0.78861) {
            amp_inference = 0.66509 * (amp_inference - 0.78861) + 0.55;
        }
        else if (amp_inference >= 0.78861 && amp_inference < 1) {
          amp_inference = 1 - std::sqrt(0.95795 * (1 - amp_inference));
        }
        else {
          amp_inference = 1.0;
        }
        //std::cout << "\n---dnn inference done---\n" << std::endl;
        //std::cout << "amp : " << amp_inference << " es : " << es_inference << std::endl;
        dnn_t_measure = 1;
		for (int v = 0; v < 112; v++) {
			Hft_PREV[v] = Hft[v];
		}
		INFERRED = 1;

		printf("DONE INFERENCE\n");
      }
      /*
      // Calculating total time taken by the program. 
         clock_gettime(CLOCK_MONOTONIC, &tv_end); 
         double time_taken; 
         time_taken = (tv_end.tv_sec - tv_start.tv_sec) * 1e9; 
         time_taken = (time_taken + (tv_end.tv_nsec - tv_start.tv_nsec)) * 1e-9; 
         if (dnn_t_measure == 1) {
            std::cout << "Time taken by program is : " << std::fixed 
         << time_taken << std::setprecision(9); 
         std::cout << " sec" << std::endl;
            dnn_t_measure = 0; 
         }
        
        clock_gettime(CLOCK_MONOTONIC, &tv_start); 
     */
  ///here
      
      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace rfid */
} /* namespace gr */

