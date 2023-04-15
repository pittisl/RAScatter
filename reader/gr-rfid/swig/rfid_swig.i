/* -*- c++ -*- */

#define RFID_API

%include "gnuradio.i"           // the common stuff

//load generated python docstrings
%include "rfid_swig_doc.i"

%{
#include "rfid/reader.h"
#include "rfid/gate.h"
#include "rfid/tag_decoder.h"
#include "rfid/pbr_gate.h"
#include "rfid/pbr_feature_extractor.h"
#include "rfid/multiply_rta_ff.h"
#include "rfid/dnn_inference.h"
%}

%include "rfid/reader.h"
GR_SWIG_BLOCK_MAGIC2(rfid, reader);


%include "rfid/gate.h"
GR_SWIG_BLOCK_MAGIC2(rfid, gate);
%include "rfid/tag_decoder.h"
GR_SWIG_BLOCK_MAGIC2(rfid, tag_decoder);

%include "rfid/pbr_feature_extractor.h"
GR_SWIG_BLOCK_MAGIC2(rfid, pbr_feature_extractor);
%include "rfid/multiply_rta_ff.h"
GR_SWIG_BLOCK_MAGIC2(rfid, multiply_rta_ff);
%include "rfid/dnn_inference.h"
GR_SWIG_BLOCK_MAGIC2(rfid, dnn_inference);
