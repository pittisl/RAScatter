import os
import pickle
from numpy.random import default_rng
import numpy as np

# 0-111   : Hft (4x28)
# 112     : RSSI (1)
# 113     : NoiseI (1)
# 114     : Power-up Delay
# 115     : Encoding Scheme (1) 0-3
# 116     : Amplitude Scalar (1)
# 117     : Throughput (1)

def load_data_golden(data_dir, DR=999/1000):
    fList = os.listdir(data_dir)
    numFiles = len(fList)
    numTrain = int(DR * numFiles)
    indList = np.random.permutation(numFiles)
    indListTrain = indList[:numTrain]
    indListTest = indList[numTrain:]
    x_train = []
    x_test = []
    for k in indListTrain:
        fName = data_dir + '/' + fList[k]
        sample = np.loadtxt(fName)
        if sample.size != 118 and sample[0] != 0:
            continue
        # sample[116] = sample[116] * sample[116]
        #sample[116] = PA_transform(sample[116])

        x_train.append(sample)

    for k in indListTest:
        fName = data_dir + '/' + fList[k]
        sample = np.loadtxt(fName)
        if sample.size != 118 and sample[0] != 0:
            continue
        # sample[116] = sample[116] * sample[116]
        #sample[116] = PA_transform(sample[116])
        x_test.append(sample)

    x_train = np.array(x_train)
    x_test = np.array(x_test)

    return x_train, x_test



def load_data_premium(data_dir, DR=35/40):
    foldList = os.listdir(data_dir)
    numPowerLevel = 15
    numFiles = 40
    powerInd_Ch_Tensor = np.zeros((numPowerLevel, numFiles, 75))
    powerInd_power_Tp_Tensor = np.zeros((numPowerLevel, numFiles, 2))
    for k in range(numPowerLevel):
        eachFold = foldList[k]
        fileList = os.listdir(data_dir + '/' + eachFold)

        for v in range(numFiles):
            f = fileList[v]
            f_name = data_dir + '/' + eachFold + '/' + f
            sample = np.loadtxt(f_name)
            if sample.size != 77:
                continue
            sample = sample.reshape((1, 1, 77))
            powerInd_Ch_Tensor[k, v, :] = sample[0, 0, :75]
            powerInd_power_Tp_Tensor[k, v, :] = sample[0, 0, 75:77]

    X = np.zeros((1, 77))
    for m in range(numPowerLevel):
        for n in range(numFiles):
            power_tp = powerInd_power_Tp_Tensor[m, n, :]
            power_tp = power_tp.reshape(1, 2)
            # add noise
            # power_tp += np.random.normal(0, 0.25, (1, 2))
            for l in range(numPowerLevel):
                Ch = powerInd_Ch_Tensor[l, n, :]
                Ch = Ch.reshape(1, 75)
                sample_new = np.concatenate((Ch, power_tp), 1)
                X = np.concatenate((X, sample_new), 0)
    X = X[1:, :]
    numSamples = X.shape[0]
    split_index = int(DR * numSamples)
    IndexList = np.random.permutation(numSamples)
    train_index = IndexList[:split_index]
    test_index = IndexList[split_index:]
    x_train = np.zeros((1, 77))
    x_test = np.zeros((1, 77))

    for ind in train_index:
        sample = X[ind, :].reshape(1, 77)
        if sample[0, 75] < 7:
            continue
        x_train = np.concatenate((x_train, sample), 0)

    for ind in test_index:
        sample = X[ind, :].reshape(1, 77)
        if sample[0, 75] < 7:
            continue
        x_test = np.concatenate((x_test, sample), 0)

    x_train = x_train[1:, :]
    x_test = x_test[1:, :]
    print(x_train.shape, x_test.shape)
    return x_train, x_test



def load_data(data_dir, DR=35/40):

    foldList = os.listdir(data_dir)
    num_samples = 40
    x_train = np.zeros((1, 72 + 5))
    x_test = np.zeros((1, 72 + 5))
    for eachFold in foldList:
        fileList = os.listdir(data_dir + '/' + eachFold)
        f_index = np.random.permutation(len(fileList))
        split_index = int(DR * num_samples)
        x_train_index = f_index[:split_index]
        x_test_index = f_index[split_index:]
        for ind in x_train_index:
            f = fileList[ind]
            f_name = data_dir + '/' + eachFold + '/' + f
            sample = np.loadtxt(f_name)
            if sample.size != 77:
                continue
            sample = sample.reshape(1, 77)
            if sample[0, 75] < 7:
                print(sample)
            # sample = np.concatenate((chrsp_phase(sample[0, :336]), sample[0, 336:].reshape(1, 3)), 1)
            x_train = np.concatenate((x_train, sample), 0)

        for ind in x_test_index:
            f = fileList[ind]
            f_name = data_dir + '/' + eachFold + '/' + f
            sample = np.loadtxt(f_name)
            if sample.size != 77:
                continue
            sample = sample.reshape(1, 77)
            if sample[0, 75] < 7:
                print(sample)
            # sample = np.concatenate((chrsp_phase(sample[0, :336]), sample[0, 336:].reshape(1, 3)), 1)
            x_test = np.concatenate((x_test, sample), 0)

    x_train = x_train[1:, :]
    x_test = x_test[1:, :]

    # pca

    # pca = PCA(n_components=100)
    # ch_train = pca.fit_transform(x_train[:, :168])
    # ch_test = pca.fit_transform(x_test[:, :168])
    # x_train = np.concatenate((ch_train, x_train[:, 168:]), 1)
    # x_test = np.concatenate((ch_test, x_test[:, 168:]), 1)

    # normalize chrsp
    # scaler = MinMaxScaler()

    # ch_train = scaler.fit_transform(x_train[:, :168].T)
    # ch_test = scaler.fit_transform(x_test[:, :168].T)
    # x_train = np.concatenate((ch_train.T, x_train[:, 168:]), 1)
    # x_test = np.concatenate((ch_test.T, x_test[:, 168:]), 1)
    return x_train, x_test


def train_valid_split(x_train, DR=6/7):
    split_index = int(DR * x_train.shape[0])
    x_train_new = x_train[:split_index]
    x_valid = x_train[split_index:]

    return x_train_new, x_valid

def chrsp_amp(x):
    M = 336
    y = np.zeros((1, 168))
    for k in range(0, 2, M):
        y[k] = np.sqrt(x[k] * x[k] + x[k + 1] * x[k + 1])
    y = y.reshape(1, 168)
    return y

def chrsp_phase(x):
    M = 336
    y = np.zeros((1, 168))
    for k in range(0, 2, M):
        y[k] = np.arctan2(x[k], x[k + 1])
    y = y.reshape(1, 168)
    return y

def PA_transform(x):
    if x >= 0.6:
        y = -1.24875 * (x - 1) * (x - 1) + 1
    else:
        y = 1.43343 * (x - 0.6) + 0.8002
    return y


if __name__ == '__main__':
    x_train, x_test = load_data_golden('Tx2Tag_retran_')
    print(x_train.shape, x_test.shape)

