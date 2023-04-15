# Introduction
This repository contains the source code for paper "RAScatter: Achieving Energy-Efficient Backscatter Readers via AI-Assisted Power Adaptation (IoTDI'22)". Check our [paper](https://ieeexplore.ieee.org/document/9797444) for details. Technically, this work makes three possibly meaningful attempts:
1. **Multi-Rate Computational RFID Tag**: It modifies the firmware of [WISP-5.1 Tag (160kHz)](https://github.com/arjonal/wisp5) to support backscattering with FM0/M2/M4/M8 encodings, while the original WISP-5.1 Tag only supports FM0. This makes WISP possible with rate adaptation schemes. See our core assembly code to generate different encodings [here](multi_rate_wisp/CCS/wisp-base/RFID/TxFM0.asm).
2. **AI for Backscatter**: A neural network based predictor is used at runtime to (a) adjust reader's **Tx power** and (b) let the reader send commands to the tag to adjust its backscatter **data rate**, i.e., switch encoding schemes. Such adaptation aims to ***achieve the user-specified backscatter goodput with minimally required RF power from the reader***. To implement this, we managed to jointly compile the GNU Radio code with TensorFlow v1 C API.
3. **Knowledge-guided AI design**: The neural network design is guided by backscatter theory. It adopts a modular structure and adds a special regularizer (Section IV.C in paper) to training for better generazability.

:relieved: ***Sadly, our efforts were paid towards a wrong direction***: Most people think it's unnecessary to save reader's RF power since they are usually plugged into the wall, though it could benefit several [portable readers](https://www.atlasrfidstore.com/handheld-rfid-readers/).

:relaxed: The code is less likely to be reused. The main purpose of this code releasing is to tell we indeed built an end-to-end system where AI and backscatter are magically integrated together, though in a less recognized way. The tag's code looks fine, but the reader's and NN's training code looks messy. I may find sometime to clean them up in the future.

# Citation
:hugs: If you find our efforts inspired your work, please kindly cite our paper:
```
@inproceedings{huang2022rascatter,
  title={RAScatter: Achieving Energy-Efficient Backscatter Readers via AI-Assisted Power Adaptation},
  author={Huang, Kai and Chen, Ruirong and Gao, Wei},
  booktitle={2022 IEEE/ACM Seventh International Conference on Internet-of-Things Design and Implementation (IoTDI)},
  pages={1--13},
  year={2022},
  organization={IEEE}
}
```
