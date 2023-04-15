import tensorflow as tf
import numpy as np
import os


with tf.Session() as sess:
    ckpt = tf.train.get_checkpoint_state('model_v1/')
    saver = tf.train.import_meta_graph(ckpt.model_checkpoint_path + '.meta')
    saver.restore(sess, tf.train.latest_checkpoint('model_v1/'))  # find the latest checkpoint

    graph = tf.get_default_graph()
    CHANNEL_RESPONSE = graph.get_tensor_by_name("CHANNEL_RESPONSE:0")
    NOISEI = graph.get_tensor_by_name("NOISEI:0")
    RSSI = graph.get_tensor_by_name("RSSI:0")
    POWER_UP_DELAY = graph.get_tensor_by_name("POWER_UP_DELAY:0")
    OBJ_THROUGHPUT = graph.get_tensor_by_name("OBJ_THROUGHPUT:0")
    AMP = graph.get_tensor_by_name("AdaBackscatterNet_v5/amp:0")

    p1_name = "Tx2Tag_final/Tx2Tag_20"
    #p2_name = "Tx2Tag_final/Tx2Tag_25"
    p3_name = "Tx2Tag_final/Tx2Tag_30"
    p4_name = "Tx2Tag_final/Tx2Tag_40"
    p5_name = "Tx2Tag_final/Tx2Tag_50"
    p6_name = "Tx2Tag_final/Tx2Tag_60"
    p7_name = "Tx2Tag_final/Tx2Tag_70"
    p8_name = "Tx2Tag_final/Tx2Tag_80"
    pList = [p1_name, p3_name, p4_name, p5_name, p6_name, p7_name, p8_name]
    for p in pList:
        fList = os.listdir(p)
        numFiles = len(fList)
        ampRecord = np.zeros((121, 1))
        for f in fList:
            fName = p + '/' + f
            sample = np.loadtxt(fName)
            if sample.shape[0] != 118:
                continue
            sample = sample.reshape(1, 118)
            # print("RSSI : ", sample[0, 112])
            # print("Power-up Delay : ", sample[0, 114])
            # print("AMP : ", sample[0, 116])
            # print("TP : ", sample[0, 117])
            ampList = []
            for k in range(0, 121):
                objective_throughput = np.array([k])
                objective_throughput = objective_throughput.reshape(1, 1)

                result = sess.run(AMP, feed_dict={CHANNEL_RESPONSE: sample[0, :112].reshape(1, 4, 28, 1),
                                                  RSSI: sample[0, 112].reshape(1, 1),
                                                  NOISEI: sample[0, 113].reshape(1, 1),
                                                  POWER_UP_DELAY: sample[0, 114].reshape(1, 1),
                                                  OBJ_THROUGHPUT: objective_throughput})
                # print(k, result)
                ampList.append(result.reshape(1, ))
            ampRecord += np.array(ampList)
        np.savetxt(p[15:] + "_curve.txt", ampRecord / numFiles, fmt='%f', delimiter='\n')

