# AgriBot: Intelligent Agricultural Robot 

## History

* 11/11/2023: Test in the latest TensorFlow
* 11/11/2023: Change model format to Keras
* 11/11/2023: Clean up
* 10/31/2023: Start with submodules


## How to Clone 

This repo has submodules. Please clone with the following command.

```bash
git clone --recurse-submodules https://github.com/jrkwon/agribot
```

## TensorFlow Installation with CUDA, cuDNN in Anaconda Env
* https://jrkwon.com/2023/11/11/install-tensorflow-with-cuda-cudnn-in-anaconda-environment/


## Additional SDKs

To run `agribot`, you must install SDKs for sensors and additional packages, including `YDLIDAR SDK` and `Cartographer`. The details can be found https://github.com/jrkwon/agribot_ros.

## How to Use

Activate the `agribot` environment.
```
conda activate agribot
```

Then `source` the `setup.bash`. You must be at the `agribot` directory when you do this.
```
source setup.bash
```

## Acknowledgments

### System Design and Implementation
- Jaerock Kwon, PhD, Assistant Professor, Electrical and Computer Engineering, University of Michigan-Dearborn

### Implementation and Contributors
- Elahe Delavari, PhD student, Electrical and Computer Engineering, University of Michigan-Dearborn
- Feeza Khanzada, PhD student, Electrical and Computer Engineering, University of Michigan-Dearborn
