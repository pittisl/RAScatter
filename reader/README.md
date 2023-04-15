# Introduction
This code is massively modified from https://github.com/arjonal/CoS_RFID/tree/master/wisp_5/SlotCounter. Please check their hardware and software instructions to run. Note that running with TPB scheduler can be much faster than STS

```
sudo GR_SCHEDULER=TPB nice -n -20 python ./reader.py 915e6
```
To try different fixed encoding schemes, set 
1. `FIXED_RATE_EN = 1` at line 268 in [`global_vars.h`](gr-rfid/include/rfid/global_vars.h)
2. modify variable `index_ES` at line 100 in [`global_vars.cc`](gr-rfid/lib/global_vars.cc) to 3 for FM0, 2 for Miller-2, 1 for Miller-4, 0 for Miller-8. 

To enable RAScatter, set
1. `ADABS_EN = 1` at line 337 in [`global_vars.h`](gr-rfid/include/rfid/global_vars.h)
2. `adabs_probing_en = 1` and `adabs_nn_en = 1` at line 112 and 113 respectively in [`global_vars.cc`](gr-rfid/lib/global_vars.cc)
