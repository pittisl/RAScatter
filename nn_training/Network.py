import tensorflow as tf


class AdaBackscatterNet(object):
    def __init__(self, net_version):
        self.net_version = net_version

    def __call__(self, CHANNL_RESPONSE, POWER_UP_DELAY, RSSI, NOISEI, OBJ_THROUGHPUT, reuse=False):
        if self.net_version == 4:
            Tx_power, Encoding_scheme = self.AdaBackscatterNet_v4(CHANNL_RESPONSE, POWER_UP_DELAY, RSSI, NOISEI, OBJ_THROUGHPUT, reuse)
            return Tx_power, Encoding_scheme
        elif self.net_version == 5:
            Tx_power, Encoding_scheme = self.AdaBackscatterNet_v5(CHANNL_RESPONSE, POWER_UP_DELAY, RSSI, NOISEI, OBJ_THROUGHPUT, reuse)
            return Tx_power, Encoding_scheme
        elif self.net_version == 6:
            Tx_power, Encoding_scheme = self.AdaBackscatterNet_v6(CHANNL_RESPONSE, POWER_UP_DELAY, RSSI, NOISEI, OBJ_THROUGHPUT, reuse)
            return Tx_power, Encoding_scheme
        elif self.net_version == 7:
            f_r, w, mu, sigma, Tx_power, Encoding_scheme = self.AdaBackscatterNet_v7(CHANNL_RESPONSE, POWER_UP_DELAY, RSSI, NOISEI, OBJ_THROUGHPUT, reuse)
            return f_r, w, mu, sigma, Tx_power, Encoding_scheme

        else:
            Tx_power, Encoding_scheme = self.AdaBackscatterNet_v1(CHANNL_RESPONSE, POWER_UP_DELAY, RSSI, NOISEI, OBJ_THROUGHPUT, reuse)
            return Tx_power, Encoding_scheme


    def AdaBackscatterNet_v1(self, CHANNL_RESPONSE, NOISE_AMP, MODULATION_DEPTH, POWER_UP_DELAY, OBJ_THROUGHPUT, reuse):
        with tf.variable_scope("AdaBackscatterNet_v1", reuse=reuse) as vs:
            x = tf.concat([CHANNL_RESPONSE, NOISE_AMP, MODULATION_DEPTH, POWER_UP_DELAY, OBJ_THROUGHPUT], 1)
            net = tf.contrib.layers.fully_connected(x, 40, activation_fn=tf.nn.relu, scope='fc_1')
            net = tf.contrib.layers.fully_connected(net, 40, activation_fn=tf.nn.relu, scope='fc_2')
            net = tf.contrib.layers.fully_connected(net, 40, activation_fn=tf.nn.relu, scope='fc_3')
            out = tf.contrib.layers.fully_connected(net, 1, activation_fn=tf.nn.relu, scope='fc_4')
            out = 15 + 5.5 * tf.keras.backend.hard_sigmoid(out)
        return out

    def AdaBackscatterNet_v2(self, CHANNL_RESPONSE, NOISE_AMP, MODULATION_DEPTH, POWER_UP_DELAY, OBJ_THROUGHPUT, reuse):
        with tf.variable_scope("AdaBackscatterNet_v2", reuse=reuse) as vs:
            branch = tf.contrib.layers.fully_connected(CHANNL_RESPONSE, 60, activation_fn=tf.nn.relu, scope="fc_1")
            branch = tf.contrib.layers.fully_connected(branch, 40, activation_fn=tf.nn.relu, scope="fc_2")
            branch = tf.contrib.layers.fully_connected(branch, 20, activation_fn=tf.nn.relu, scope="fc_3")

            backbone = tf.concat([branch, NOISE_AMP, MODULATION_DEPTH, POWER_UP_DELAY, OBJ_THROUGHPUT], 1)
            backbone = tf.contrib.layers.fully_connected(backbone, 30, activation_fn=tf.nn.relu, scope="fc_4")
            backbone = tf.contrib.layers.fully_connected(backbone, 30, activation_fn=tf.nn.relu, scope="fc_5")
            out = tf.layers.dense(backbone, 1, name="ds_1")

            Tx_power = 7 + 7 * tf.keras.backend.hard_sigmoid(out)
            Tx_power = tf.identity(Tx_power, name="TX_POWER_PREDICTION")
        return Tx_power

    def AdaBackscatterNet_v3(self, CHANNL_RESPONSE, NOISE_AMP, MODULATION_DEPTH, POWER_UP_DELAY, OBJ_THROUGHPUT, reuse):
        with tf.variable_scope("AdaBackscatterNet_v3", reuse=reuse) as vs:
            branch_1 = tf.contrib.layers.fully_connected(CHANNL_RESPONSE, 20, activation_fn=tf.nn.relu, scope="fc_1")
            branch_1 = tf.contrib.layers.fully_connected(branch_1, 20, activation_fn=tf.nn.relu, scope="fc_2")
            branch_1 = tf.contrib.layers.fully_connected(branch_1, 20, activation_fn=tf.nn.relu, scope="fc_3")

            branch_2 = tf.concat([branch_1, NOISE_AMP, MODULATION_DEPTH, POWER_UP_DELAY], 1)
            branch_2 = tf.contrib.layers.fully_connected(branch_2, 20, activation_fn=tf.nn.relu, scope="fc_4")
            branch_2 = tf.contrib.layers.fully_connected(branch_2, 20, activation_fn=tf.nn.relu, scope="fc_5")

            backbone = tf.concat([branch_2, OBJ_THROUGHPUT], 1)
            backbone = tf.contrib.layers.fully_connected(backbone, 20, activation_fn=tf.nn.relu, scope="fc_6")
            out = tf.layers.dense(backbone, 1, name="ds_1")

            Tx_power = 7 + 7 * tf.keras.backend.hard_sigmoid(out)
            Tx_power = tf.identity(Tx_power, name="TX_POWER_PREDICTION")
        return Tx_power

    # NMod + Nloss
    def AdaBackscatterNet_v4(self, CHANNL_RESPONSE, POWER_UP_DELAY, RSSI, NOISEI, OBJ_THROUGHPUT, reuse):
        with tf.variable_scope("AdaBackscatterNet_v4", reuse=reuse) as vs:
            chrsp_features = tf.reshape(CHANNL_RESPONSE, (-1, 28*4))
            channel_features = tf.concat([chrsp_features, POWER_UP_DELAY, RSSI, NOISEI, OBJ_THROUGHPUT], 1)
            # Tx power predictor
            #PowerFactor = tf.concat([channel_features, OBJ_THROUGHPUT], 1)
            subNet2 = tf.contrib.layers.fully_connected(channel_features, 20, activation_fn=tf.nn.relu,
                                                        scope="sub2_fc_0")
            subNet2 = tf.contrib.layers.fully_connected(subNet2, 20, activation_fn=tf.nn.relu, scope="sub2_fc_1")
            subNet2 = tf.contrib.layers.fully_connected(subNet2, 20, activation_fn=tf.nn.relu, scope="sub2_fc_2")
            subNet2 = tf.contrib.layers.fully_connected(subNet2, 20, activation_fn=tf.nn.relu, scope="sub2_fc_3")
            subNet2 = tf.contrib.layers.fully_connected(channel_features, 20, activation_fn=tf.nn.relu,
                                                        scope="sub2_fc_4")

            FinalLayer = tf.layers.dense(subNet2, 5, name="sub2_ds_1")
            RateClassification = FinalLayer[:, :4]

            ENCODING_SCHEME = tf.nn.softmax(RateClassification, name="sub2_softmax_1")
            ENCODING_SCHEME = tf.identity(ENCODING_SCHEME, name="es_scores")

            PowerRegression = FinalLayer[:, 4]
            AMP_SCALAR = tf.nn.sigmoid(PowerRegression, name="sub2_sigmoid_1")
            AMP_SCALAR = tf.identity(AMP_SCALAR, name="amp")

            return AMP_SCALAR, ENCODING_SCHEME

    def AdaBackscatterNet_v5(self, CHANNL_RESPONSE, POWER_UP_DELAY, RSSI, NOISEI, OBJ_THROUGHPUT, reuse):
        with tf.variable_scope("AdaBackscatterNet_v5", reuse=reuse) as vs:

            # channel feature extractor
            cnn = tf.contrib.slim.conv2d(CHANNL_RESPONSE, 1, (2, 3), stride=1,
                                         activation_fn=tf.nn.relu, padding='SAME', scope='conv1')
            cnn = tf.contrib.slim.conv2d(cnn, 2, (1, 1), stride=1,
                                         activation_fn=tf.nn.relu, padding='SAME', scope='conv2')
            cnn = tf.reduce_mean(cnn, [1, 2], keepdims=True)
            chrsp_features = tf.squeeze(cnn, [1, 2])

            channel_features = tf.concat([chrsp_features, POWER_UP_DELAY, RSSI, NOISEI], 1)
            # subNet1 = tf.contrib.layers.fully_connected(channel_metrics, 20, activation_fn=tf.nn.relu, scope="sub1_fc_1")
            # channel_features = tf.contrib.layers.fully_connected(subNet1, 10, activation_fn=tf.nn.relu, scope="sub1_fc_2")

            # Tx power predictor
            PowerFactor = tf.concat([channel_features, OBJ_THROUGHPUT], 1)
            PowerFactor = tf.contrib.layers.fully_connected(PowerFactor, 20, activation_fn=tf.nn.leaky_relu, scope="sub2_fc_1")
            subNet2 = tf.contrib.layers.fully_connected(PowerFactor, 20, activation_fn=tf.nn.leaky_relu, scope="sub2_fc_2")
            subNet2 = tf.contrib.layers.fully_connected(subNet2, 20, activation_fn=tf.nn.leaky_relu, scope="sub2_fc_3")
            # attention_weights = tf.nn.softmax(channel_features)

            # subNet2 = tf.nn.sigmoid(subNet2)
            # AMP_SCALAR = subNet2 * attention_weights
            # AMP_SCALAR = tf.reduce_sum(AMP_SCALAR, keepdims=True, axis=1)
            subNet2 = tf.layers.dense(subNet2, 1)
            AMP_SCALAR = tf.nn.sigmoid(subNet2)
            AMP_SCALAR = tf.identity(AMP_SCALAR, name="amp")

            # encoding scheme (data rate) predictor
            RateFactor = tf.concat([channel_features, AMP_SCALAR], 1)
            subNet3 = tf.contrib.layers.fully_connected(RateFactor, 10, activation_fn=tf.nn.relu, scope="sub3_fc_1")
            subNet3 = tf.contrib.layers.fully_connected(subNet3, 10, activation_fn=tf.nn.relu, scope="sub3_fc_2")
            # subNet3 = tf.contrib.layers.fully_connected(subNet3, 10, activation_fn=tf.nn.relu, scope="sub3_fc_3")
            subNet3 = tf.layers.dense(subNet3, 4, name="sub3_ds_1")
            ENCODING_SCHEME = tf.nn.softmax(subNet3, name="sub3_softmax_1")
            ENCODING_SCHEME = tf.identity(ENCODING_SCHEME, name="es_scores")
            return AMP_SCALAR, ENCODING_SCHEME

    # Mod. + DLoss/Nloss
    def AdaBackscatterNet_v6(self, CHANNL_RESPONSE, POWER_UP_DELAY, RSSI, NOISEI, OBJ_THROUGHPUT, reuse):
        with tf.variable_scope("AdaBackscatterNet_v6", reuse=reuse) as vs:
            # channel feature extractor
            cnn = tf.contrib.slim.conv2d(CHANNL_RESPONSE, 1, (2, 3), stride=1, activation_fn=tf.nn.relu, padding='SAME', scope='conv1')
            cnn = tf.contrib.slim.conv2d(cnn, 2, (1, 1), stride=1, activation_fn=tf.nn.relu, padding='SAME', scope='conv2')
            cnn = tf.contrib.slim.conv2d(cnn, 2, (1, 1), stride=1, activation_fn=tf.nn.relu, padding='SAME', scope='conv3')
            cnn = tf.contrib.slim.conv2d(cnn, 2, (1, 1), stride=1, activation_fn=tf.nn.relu, padding='SAME', scope='conv4')
            cnn = tf.reduce_mean(cnn, [1, 2], keepdims=True)
            chrsp_features = tf.squeeze(cnn, [1, 2])

            channel_features = tf.concat([chrsp_features, POWER_UP_DELAY, RSSI, NOISEI], 1)
            channel_features_h = tf.layers.dense(channel_features, 8)
            attention_weights = tf.nn.softmax(channel_features_h)
            channel_features = attention_weights * channel_features_h

            # Tx power predictor
            PowerFactor = tf.concat([channel_features, OBJ_THROUGHPUT], 1)
            subNet2 = tf.contrib.layers.fully_connected(PowerFactor, 10, activation_fn=tf.nn.relu, scope="sub2_fc_1")
            subNet2 = tf.contrib.layers.fully_connected(subNet2, 10, activation_fn=tf.nn.relu, scope="sub2_fc_2")
            subNet2 = tf.contrib.layers.fully_connected(subNet2, 10, activation_fn=tf.nn.relu, scope="sub2_fc_3")
            subNet2 = tf.contrib.layers.fully_connected(subNet2, 10, activation_fn=tf.nn.relu, scope="sub2_fc_4")
            subNet2 = tf.contrib.layers.fully_connected(subNet2, 10, activation_fn=tf.nn.relu, scope="sub2_fc_5")
            subNet2 = tf.contrib.layers.fully_connected(subNet2, 10, activation_fn=tf.nn.relu, scope="sub2_fc_6")
            subNet2 = tf.layers.dense(subNet2, 1)
            AMP_SCALAR = tf.nn.sigmoid(subNet2)
            AMP_SCALAR = tf.identity(AMP_SCALAR, name="amp")

            # encoding scheme (data rate) predictor
            RateFactor = tf.concat([channel_features, AMP_SCALAR], 1)
            subNet3 = tf.contrib.layers.fully_connected(RateFactor, 10, activation_fn=tf.nn.relu, scope="sub3_fc_1")
            subNet3 = tf.contrib.layers.fully_connected(subNet3, 10, activation_fn=tf.nn.relu, scope="sub3_fc_2")
            subNet3 = tf.contrib.layers.fully_connected(subNet3, 10, activation_fn=tf.nn.relu, scope="sub3_fc_3")
            subNet3 = tf.contrib.layers.fully_connected(subNet3, 10, activation_fn=tf.nn.relu, scope="sub3_fc_4")
            subNet3 = tf.contrib.layers.fully_connected(subNet3, 10, activation_fn=tf.nn.relu, scope="sub3_fc_5")
            subNet3 = tf.contrib.layers.fully_connected(subNet3, 10, activation_fn=tf.nn.relu, scope="sub3_fc_6")
            subNet3 = tf.layers.dense(subNet3, 4, name="sub3_ds_1")
            ENCODING_SCHEME = tf.nn.softmax(subNet3, name="sub3_softmax_1")
            ENCODING_SCHEME = tf.identity(ENCODING_SCHEME, name="es_scores")

            return AMP_SCALAR, ENCODING_SCHEME
    
    def AdaBackscatterNet_v7(self, CHANNL_RESPONSE, POWER_UP_DELAY, RSSI, NOISEI, OBJ_THROUGHPUT, reuse):
        with tf.variable_scope("AdaBackscatterNet_v7", reuse=reuse) as vs:
            # channel feature extractor
            cnn = tf.contrib.slim.conv2d(CHANNL_RESPONSE, 1, (2, 3), stride=1, activation_fn=tf.nn.relu, padding='SAME', scope='conv1')
            cnn = tf.contrib.slim.conv2d(cnn, 2, (1, 1), stride=1, activation_fn=tf.nn.relu, padding='SAME', scope='conv2')
            
            cnn = tf.reduce_mean(cnn, [1, 2], keepdims=True)
            chrsp_features = tf.squeeze(cnn, [1, 2])

            channel_features = tf.concat([chrsp_features, POWER_UP_DELAY, RSSI, NOISEI], 1)

            # Feature weighting starts
            channel_features = tf.layers.dense(channel_features, 8)
            # channel_features = tf.contrib.layers.fully_connected(channel_features, 8, activation_fn=tf.nn.relu)
            channel_features_h = tf.layers.dense(channel_features, 8, name='w_ds_0')
            attention_weights = tf.nn.softmax(channel_features_h)
            channel_features = attention_weights * channel_features

            # normalization
            u = tf.reduce_mean(channel_features, axis=1, keepdims=True)
            s = tf.math.reduce_std(channel_features, axis=1, keepdims=True)
            channel_features = 1.0 / s * (channel_features - u)
            # Feature weighting ends

            # Tx power predictor
            PowerFactor = tf.concat([channel_features, OBJ_THROUGHPUT], 1)
            subNet2 = tf.contrib.layers.fully_connected(PowerFactor, 10, activation_fn=tf.nn.relu, scope="sub2_fc_1")
            subNet2 = tf.contrib.layers.fully_connected(subNet2, 10, activation_fn=tf.nn.relu, scope="sub2_fc_2")
            subNet2 = tf.contrib.layers.fully_connected(subNet2, 10, activation_fn=tf.nn.relu, scope="sub2_fc_3")
            
            subNet2 = tf.layers.dense(subNet2, 2)
            AMP_SCALAR_mu = tf.nn.sigmoid(subNet2[:, 0:1])
            AMP_SCALAR_sigma = subNet2[:, 1:]
            AMP_SCALAR = AMP_SCALAR_mu + AMP_SCALAR_sigma * tf.random.normal(tf.shape(AMP_SCALAR_mu), mean=0.0, stddev=1.0)
            AMP_SCALAR = tf.identity(AMP_SCALAR, name="amp")

            # encoding scheme (data rate) predictor
            RateFactor = tf.concat([channel_features, AMP_SCALAR_mu, AMP_SCALAR_sigma], 1)
            subNet3 = tf.contrib.layers.fully_connected(RateFactor, 10, activation_fn=tf.nn.relu, scope="sub3_fc_1")
            subNet3 = tf.contrib.layers.fully_connected(subNet3, 10, activation_fn=tf.nn.relu, scope="sub3_fc_2")
            subNet3 = tf.contrib.layers.fully_connected(subNet3, 10, activation_fn=tf.nn.relu, scope="sub3_fc_3")
            
            subNet3 = tf.layers.dense(subNet3, 4, name="sub3_ds_1")
            ENCODING_SCHEME = tf.nn.softmax(subNet3, name="sub3_softmax_1")
            ENCODING_SCHEME = tf.identity(ENCODING_SCHEME, name="es_scores")

            return channel_features_h, attention_weights, AMP_SCALAR_mu, AMP_SCALAR_sigma, AMP_SCALAR, ENCODING_SCHEME


