import numpy as np
import matplotlib.pyplot as plt

def load_bin_file(samplerate=2e6, type="complex", bfile="../data/file_source_test", plot=False, start_us=0, end_us=0):
    if type not in ["complex", "real"]:
        print("data type must be complex or real.")
        exit()
    with open(bfile, "rb") as f:
        data = np.fromfile(f, dtype=np.float32)
    if type == "complex":
        data = data[::2] + 1j * data[1:][::2]
    acq_time_us = np.linspace(1, len(data), len(data)) / samplerate * 1e6
    start_index = np.where(acq_time_us>start_us)[0][0]
    if end_us == 0:
        end_index = len(acq_time_us) - 1
    else:
        end_index = np.where(acq_time_us>end_us)[0][0]
    if plot:
        plt.plot(acq_time_us[start_index:end_index], abs(data[start_index:end_index]), color="k")
        plt.grid()
        plt.ylabel("Amplitude")
        plt.xlabel("Time(us)")
        plt.show()
    return data

if __name__ == "__main__":
    data = load_bin_file(plot=True, start_us=26000, end_us=43000)
