import  tensorflow as tf


saver = tf.train.import_meta_graph('model_v1/model.ckpt-500.meta', clear_devices=True)
graph = tf.get_default_graph()
input_graph_def = graph.as_graph_def()
sess = tf.Session()
saver.restore(sess, "model_v1/model.ckpt-500")

output_node_names="AdaBackscatterNet_v7/amp,AdaBackscatterNet_v7/es_scores"
output_graph_def = tf.graph_util.convert_variables_to_constants(
            sess, # The session
            input_graph_def, # input_graph_def is useful for retrieving the nodes
            output_node_names.split(",")
)

output_graph = "model_v1/frozen_model.pb"
with tf.gfile.GFile(output_graph, "wb") as f:
    f.write(output_graph_def.SerializeToString())

sess.close()