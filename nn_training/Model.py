from Network import *
from Utils import *
from tqdm import tqdm
from matplotlib import pyplot as plt
import time


class AIBackscatter(object):

    def __init__(self, sess, conf):
        self.sess = sess
        self.conf = conf
        self.n_components = 28
        self.metric_start = 0
        self.net_version = self.conf.net_version
        self.batch_size = self.conf.batch_size
        self.lr = tf.placeholder(tf.float32, name="lr") # you forget the learning rate...
        self.loss_OFL = tf.placeholder(tf.float32)
        self.loss_OL = tf.placeholder(tf.float32)
        self.CHANNEL_RESPONSE = tf.placeholder(tf.float32, shape=(None, 4, self.n_components, 1), name="CHANNEL_RESPONSE")
        self.NOISEI = tf.placeholder(tf.float32, shape=(None, 1), name="NOISEI")
        self.RSSI = tf.placeholder(tf.float32, shape=(None, 1), name="RSSI")
        self.POWER_UP_DELAY = tf.placeholder(tf.float32, shape=(None, 1), name="POWER_UP_DELAY")
        self.AMP_SCALAR = tf.placeholder(tf.float32, shape=(None, 1), name="AMP_SCALAR")
        self.ENCODING_SCHEME = tf.placeholder(tf.int32, shape=(None, 1), name="ENCODING_SCHEME")
        self.OBJ_THROUGHPUT = tf.placeholder(tf.float32, shape=(None, 1), name="OBJ_THROUGHPUT")
        self.AMP_SCALAR_PRED = tf.placeholder(tf.float32, shape=(None, 1), name="AMP_SCALAR_PRED")
        self.ENCODING_SCHEME_PRED = tf.placeholder(tf.float32, shape=(None, 4), name="ENCODING_SCHEME_PRED")
        self.ACTUAL_THROUGHPUT = tf.placeholder(tf.float32, shape=(None, 1), name="ACTUAL_THROUGHPUT")
        self.w = tf.placeholder(tf.float32, shape=(None, 8), name="f_w")
        self.f_raw = tf.placeholder(tf.float32, shape=(None, 8), name="f_raw")

    def setup(self, training):
        print('---Setup input interfaces...')
        air = AdaBackscatterNet(net_version=self.net_version)

        if training:
            print('Setup training components...')
            self.f_raw, self.w, mu, sigma, self.AMP_SCALAR_PRED, self.ENCODING_SCHEME_PRED = air(self.CHANNEL_RESPONSE, self.POWER_UP_DELAY,
                                                                  self.RSSI, self.NOISEI, self.OBJ_THROUGHPUT)
            
            # ---MONOTONIC LOSS
            tp_loss = tf.maximum(0.0, tf.reduce_mean(-tf.gradients(self.AMP_SCALAR_PRED, self.OBJ_THROUGHPUT)[0]))
            # tp2_loss = tf.maximum(0.0, tf.reduce_mean(-tf.hessians(self.AMP_SCALAR_PRED, self.OBJ_THROUGHPUT)[0]))
            pud_loss = tf.maximum(0.0, tf.reduce_mean(-tf.gradients(self.AMP_SCALAR_PRED, self.POWER_UP_DELAY)[0]))
            # rssi_loss = tf.maximum(0.0, tf.reduce_mean(tf.gradients(self.AMP_SCALAR_PRED, self.RSSI)[0]))
            
            # l2_loss = tf.losses.get_regularization_loss()
            # self.loss += l2_loss

            # ---OFFLINE LOSS
            #loss_A = 1000 * tf.reduce_mean((self.AMP_SCALAR - self.AMP_SCALAR_PRED)**2)
            loss_E = tf.losses.softmax_cross_entropy(
                onehot_labels=tf.squeeze(tf.one_hot(self.ENCODING_SCHEME, 4), 1),
                logits=self.ENCODING_SCHEME_PRED
            )
            loss_A = 1000 * tf.losses.absolute_difference(self.AMP_SCALAR, self.AMP_SCALAR_PRED)
            self.loss_OFL = loss_A + loss_E
            # self.loss_OFL += 10 * (10 * tp_loss + pud_loss)

            # ---ONLINE LOSS
            # instantaneous mse (g_obj - g_act).^2
            g_error = tf.reduce_mean((self.ACTUAL_THROUGHPUT - self.OBJ_THROUGHPUT)**2)

            # log likelihood of rate classification
            r_chosen_probs = tf.math.reduce_max(self.ENCODING_SCHEME_PRED, axis=1, keepdims=True)
            # r_chosen = tf.random.categorical(tf.math.log(self.ENCODING_SCHEME_PRED), 1)
            # r_chosen = tf.math.argmax(self.ENCODING_SCHEME_PRED, axis=1)
            # r_chosen = tf.reshape(r_chosen, tf.shape(mu))
            
            # print(r_chosen.shape)
            # r_chosen = tf.squeeze(r_chosen, 1)
            # r_chosen_probs = tf.gather_nd(self.ENCODING_SCHEME_PRED, r_chosen) # predicted p of rate [p_t1 p_t2 ....]
            # print(r_chosen_probs.shape)
            r_ll = tf.math.log(r_chosen_probs)

            # log likelihood of power regression
            p_ll = tf.math.log(tf.exp(-0.5 * (self.AMP_SCALAR_PRED - mu) / sigma) ** 2 \
                * 1 / (sigma * tf.sqrt(2 * np.pi)))
            
            # expectation of mse
            g_error_expection = tf.math.reduce_sum((r_ll + p_ll) * g_error)

            # R/P factor
            # r_supervisor = tf.math.reduce_sum(self.ENCODING_SCHEME_PRED * [8.0, 4.0, 2.0, 1.0]) / self.AMP_SCALAR_PRED
            r_supervisor = self.AMP_SCALAR_PRED / tf.math.reduce_sum(self.ENCODING_SCHEME_PRED * [8.0, 4.0, 2.0, 1.0])
            # form online loss
            self.loss_OL = 1000 * g_error_expection + r_supervisor
            self.loss_OL += 10 * (10 * tp_loss + pud_loss)
            
            nn_vars = tf.trainable_variables()
            # optimization
            self.optim_OFL = tf.train.AdamOptimizer(6e-4).minimize(self.loss_OFL, var_list=nn_vars, name='train_offline')
            self.optim_OL = tf.train.GradientDescentOptimizer(1e-4).minimize(self.loss_OL, var_list=nn_vars, name='train_online')
            print('Run this operation for a train step            : ', self.optim_OL.name)
            print('---Setup the Saver for saving models...')
            self.saver = tf.train.Saver(var_list=tf.global_variables(), max_to_keep=0)
            saver_def = self.saver.as_saver_def()
            print('Feed this tensor to set the checkpoint filename: ', saver_def.filename_tensor_name)
            print('Run this operation to save a checkpoint        : ', saver_def.save_tensor_name)
            print('Run this operation to restore a checkpoint     : ', saver_def.restore_op_name)
            # save graph definition
            with open(self.conf.modeldir + '/' + 'model.pb', 'wb') as f:
                f.write(tf.get_default_graph().as_graph_def().SerializeToString())
            print('The pb file has been created.')

            tf.train.write_graph(self.sess.graph.as_graph_def(), '.', 'model_v1/model.pbtxt', as_text=True)
            print('The pbtxt file has been created.')

        else:
            print('---Setup testing components')
            _0, _1, _2, self.AMP_SCALAR_PRED, self.ENCODING_SCHEME_PRED = air(self.CHANNEL_RESPONSE, self.POWER_UP_DELAY,
                                                                  self.RSSI, self.NOISEI, self.OBJ_THROUGHPUT, reuse=True)

            print('---Setup the Saver for loading models...')
            self.loader = tf.train.Saver(var_list=tf.global_variables())


    def train(self, X_train,  max_epoch):
        print('###Train###')

        self.setup(True)
        init = tf.global_variables_initializer()
        print('Run this operation to initialize variables     : ', init.name)
        self.sess.run(init)

        num_samples = X_train.shape[0]
        num_batches = int(num_samples / self.conf.batch_size)
        lr = self.conf.lr

        print('---Run...')
        loss_t = []
        t = []
        for epoch in range(1, max_epoch+1):

            start_time = time.time()
            # Shuffle
            shuffle_index = np.random.permutation(num_samples)
            curr_X_train = X_train[shuffle_index]
            
            # learning rate decay
            # if epoch > 99 and epoch % 50 == 0:
                # lr /= 10
            # get dense layer weights
            all_vars = tf.global_variables()
            def get_var(name):
                for i in range(len(all_vars)):
                    if all_vars[i].name.startswith(name):
                        return all_vars[i]
                return None
            ds_var = get_var('AdaBackscatterNet_v7/w_ds_0')
            ds = []
            f_raw = []
            w = []
            loss_value = []
            for i in range(num_batches):

                x_batch = np.array([x for x in curr_X_train[i*self.conf.batch_size:(i+1)*self.conf.batch_size]])

                Hft_flat = x_batch[:, :112]
                CHANNEL_RESPONSE = Hft_flat.reshape(self.batch_size, 4, 28, 1)

                RSSI = x_batch[:, 112].reshape(self.batch_size, 1)
                NOISEI = x_batch[:, 113].reshape(self.batch_size, 1)
                POWER_UP_DELAY = x_batch[:, 114].reshape(self.batch_size, 1)
                ENCODING_SCHEME = x_batch[:, 115].reshape(self.batch_size, 1)
                AMP_SCALAR = x_batch[:, 116].reshape(self.batch_size, 1)
                OBJ_THROUGHPUT = x_batch[:, 117].reshape(self.batch_size, 1)

                # Run
                feed_dict = {self.CHANNEL_RESPONSE: CHANNEL_RESPONSE,
                             self.POWER_UP_DELAY: POWER_UP_DELAY,
                             self.RSSI: RSSI,
                             self.NOISEI: NOISEI,
                             self.ENCODING_SCHEME: ENCODING_SCHEME,
                             self.AMP_SCALAR: AMP_SCALAR,
                             self.OBJ_THROUGHPUT: OBJ_THROUGHPUT,
                             }
                ds, f_raw, w, loss, _ =self.sess.run([ds_var, self.f_raw, self.w, self.loss_OFL, self.optim_OFL], feed_dict)
                loss_value.append(loss)
                # print('Batch {:d}/{:d} Loss {:.6f}'.format(i, num_batches, loss), end='\r', flush=True)
            duration = time.time() - start_time
            loss = np.mean(loss_value)
            loss_t.append(loss)
            t.append(time.time())
            print('Epoch {:d} Loss {:.6f} Duration {:.3f} seconds.'.format(epoch, loss, duration))
            print('ds:')
            print(ds)
            print('f_raw:')
            print(f_raw)
            print('w:')
            print(w)
            if epoch % self.conf.save_interval == 0:
                self.save(self.saver, epoch)
        
        np.savetxt("Loss/t.txt", np.array(t), fmt='%f', delimiter='\n')
        np.savetxt("Loss/loss.txt", np.array(loss_t), fmt='%f', delimiter='\n')

    def test_or_validate(self, X, checkpoint_num_list):
        print('###Test or Validate###')

        self.setup(False)
        self.sess.run(tf.global_variables_initializer())

        # load checkpoint
        for checkpoint_num in checkpoint_num_list:
            checkpointfile = self.conf.modeldir + '/model.ckpt-' + str(checkpoint_num)
            self.load(self.loader, checkpointfile)

            amp_scalar_preds = []
            encoding_scheme_preds = []
            AMP_SCALAR_gt = []
            ENCODING_SCHEME_gt = []
            for i in tqdm(range(X.shape[0])):
                # CHANNEL_RESPONSE = X[i, self.metric_start:self.metric_start+self.n_components].reshape(1, self.n_components)

                Hft_flat = X[i, :112].reshape(1, 112)
                CHANNEL_RESPONSE = Hft_flat.reshape(1, 4, 28, 1)

                RSSI = X[i, 112].reshape(1, 1)
                NOISEI = X[i, 113].reshape(1, 1)
                POWER_UP_DELAY = X[i, 114].reshape(1, 1)
                ENCODING_SCHEME = X[i, 115].reshape(1, 1)
                AMP_SCALAR = X[i, 116].reshape(1, 1)
                OBJ_THROUGHPUT = X[i, 117].reshape(1, 1)

                feed_dict = {self.CHANNEL_RESPONSE: CHANNEL_RESPONSE,
                             self.POWER_UP_DELAY: POWER_UP_DELAY,
                             self.RSSI: RSSI,
                             self.NOISEI: NOISEI,
                             self.OBJ_THROUGHPUT: OBJ_THROUGHPUT}
                AMP_SCALAR_PRED, ENCODING_SCHEME_PROB = self.sess.run([self.AMP_SCALAR_PRED, self.ENCODING_SCHEME_PRED], feed_dict=feed_dict)
                ENCODING_SCHEME_PRED = np.argmax(ENCODING_SCHEME_PROB)
                amp_scalar_preds.append(AMP_SCALAR_PRED)
                encoding_scheme_preds.append(ENCODING_SCHEME_PRED)
                AMP_SCALAR_gt.append(AMP_SCALAR)
                ENCODING_SCHEME_gt.append(ENCODING_SCHEME)

            amp_scalar_preds = np.array(amp_scalar_preds).reshape(X.shape[0], 1)
            encoding_scheme_preds = np.array(encoding_scheme_preds).reshape(X.shape[0], 1)
            AMP_SCALAR_gt = np.array(AMP_SCALAR_gt).reshape(X.shape[0], 1)
            ENCODING_SCHEME_gt = np.array(ENCODING_SCHEME_gt).reshape(X.shape[0], 1)

            # print pairs
            for k in range(amp_scalar_preds.shape[0]):
                print("AMP: ", amp_scalar_preds[k], AMP_SCALAR_gt[k], " ES: ", encoding_scheme_preds[k], ENCODING_SCHEME_gt[k])

            ampMeanError = np.sum(np.abs(amp_scalar_preds - AMP_SCALAR_gt) / AMP_SCALAR_gt) / AMP_SCALAR_gt.shape[0]
            esAcc = np.sum(encoding_scheme_preds == ENCODING_SCHEME_gt) / ENCODING_SCHEME_gt.shape[0]
            print('\nTest ampMeanError: {:.4f}%'.format(100 * ampMeanError))
            print('\nTest esAcc: {:.4f}%'.format(100 * esAcc))

    def save(self, saver, step):
        model_name = 'model.ckpt'
        checkpoint_path = os.path.join(self.conf.modeldir, model_name)
        if not os.path.exists(self.conf.modeldir):
            os.makedirs(self.conf.modeldir)
        # saver.save(self.sess, checkpoint_path)
        saver.save(self.sess, checkpoint_path, global_step=step)
        print('The checkpoint has been created.')


    def load(self, loader, filename):
        loader.restore(self.sess, filename)
        print("Restored model parameters from {}".format(filename))

    def preds_constrained(self, x):
        y = np.zeros((x.shape[0], 1))
        for k in range(x.shape[0]):
            intx = int(x[k])
            if x[k] >= intx and x[k] < intx + 0.25:
                y[k] = intx
            elif x[k] >= intx + 0.25 and x[k] <= intx + 0.75:
                y[k] = intx + 0.5
            else:
                y[k] = intx + 1

        return y

