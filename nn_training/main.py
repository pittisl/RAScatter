import tensorflow as tf
from Model import AIBackscatter
from Utils import load_data_golden, train_valid_split
import os


def configure():
    flags = tf.app.flags

    flags.DEFINE_integer('net_version', 7, 'the version of AdaBSNet')
    flags.DEFINE_integer('batch_size', 2, 'training batch size')
    flags.DEFINE_integer('save_interval', 500, 'save the checkpoint when epoch MOD save_interval == 0')
    flags.DEFINE_string('modeldir', 'model_v1', 'model directory')
    flags.DEFINE_float('lr', 6e-4, 'learning rate')

    flags.FLAGS.__dict__['__parsed'] = False
    return flags.FLAGS

def main(_):
    config_proto = tf.ConfigProto(log_device_placement=False, allow_soft_placement=True)
    config_proto.gpu_options.allow_growth = True
    sess = tf.Session(config=config_proto)

    print('---Prepare data...')

    data_dir = 'Tx2Tag_retran_'
    x_train, x_test = load_data_golden(data_dir)

    model = AIBackscatter(sess, configure())
    model.train(x_train, 500)
    model.test_or_validate(x_train, [500])
    # model.test_or_validate(x_test, [100])


if __name__ == '__main__':
    os.environ['CUDA_VISIBLE_DEVICES'] = '-1' # CPU-only if -1
    tf.app.run()