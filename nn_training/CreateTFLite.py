import tensorflow as tf

# Convert the model.
converter = tf.compat.v1.lite.TFLiteConverter.from_frozen_graph(
    graph_def_file='model_v1/frozen_model.pb',
    input_arrays=['CHANNEL_RESPONSE', 'POWER_UP_DELAY', 'RSSI', 'NOISEI', 'OBJ_THROUGHPUT'],
    input_shapes={'CHANNEL_RESPONSE' : [1, 4, 28, 1], 'POWER_UP_DELAY' : [1, 1], 'RSSI' : [1, 1], 'NOISEI' : [1, 1], 'OBJ_THROUGHPUT' : [1, 1]},
    output_arrays=['AdaBackscatterNet_v6/amp', 'AdaBackscatterNet_v6/es_scores'],
)
# converter.quantized_input_stats = {'input' : (0., 1.)}  # mean, std_dev (input range is [-1, 1])
# converter.inference_type = tf.int8 # this is the recommended type.
# converter.inference_input_type=tf.uint8 # optional
# converter.inference_output_type=tf.uint8 # optional
tflite_model = converter.convert()

# Save the model.
with open('model_v1/model_lite.tflite', 'wb') as f:
  f.write(tflite_model)