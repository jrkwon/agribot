# AgriBot: Intelligent Agricultural Robot 

[![demo-image](e2e-dataset/jaerock/2023-10-31-14-56-16_scout-jaerock-orchard_n3_2023-11-14-01-07-43-650_view/2023-10-31-14-56-16/2023-10-31-14-57-53-788068_-0.22_-19.61.jpg)](https://youtu.be/hPp8q_WmuDs)

## History

* 11/14/2023: Add several How-To's
* 11/14/2023: Add `view_drive.py` avoiding a direct use of `drive_view.py`
* 11/12/2023: Add `test_drive.py` replacing a direct use of `drive_log.py`
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

## Prior to Use

Create the `agribot` conda environment.

```
conda env create --file config/conda/environment.yaml
```

## How to Use

Activate the `agribot` environment.
```
conda activate agribot
```

Then `source` the `setup.bash`. You must be at the `agribot` directory when you do this.
```
source setup.bash
```

## How to Collect Data

The default data location is `agribot/e2e-dataset/<data-id>`. If `jaerock` collected a data at 5:50:10 PM, Nov 12, 2023, then the directory name of the dataset is `agribot/e2e-dataset/jaerock/2023-11-12-17-50-10`.

The data directory has a lot of images in most cases. So, it can be slow when you open the directory. Users need to open a `CSV` file inside the directory in many cases. Here is a script that can make a symblic link to the `agribot/e2e-dataset/<data-id>`.

```bash
cd path/to/agribot

sh make_simlink.sh path/to/data-dir
```


## How to Train

Note:
- All checkpoint files are now saved at the `yy-mm-dd-hh-mm-ss_ckpt` directory in `agribot/e2e-dataset/<data-id>`
- The trained model is saved as the latest Keras model.
- All pictures and csv files (comprison, err_hist, log, scatter) are generated with the format: `<dataset-dir-name>_<config_name>_<trained-model-name>_<timestamp>` to identify when the training/driving was done with which dataset, which configuration, and which model.
- When neural_net related scripts are run, you are expected at the `agribot` directory.

Here is an example:
```bash
conda activate agribots
cd path/to/agribot

source setup.bash
python neural_net/train.py e2e-dataset/<id>/yyy-mm-dd-hh-mm-ss
```

## How to Do Test Drive

Once you have a trained model, you can do a test drive with a dataset.

Let us assume followings as an example.
- data id: `jaerock`
- dataset name: `2023-10-31-14-56-16`
- config name: `scout-jaerock-orchard--aug`
- network id: `3` 
- timestamp: `2023-11-12-16-25-31-079`

Then, a trained model name is 
`2023-10-31-14-56-16_scout-jaerock-orchard--aug_n3_2023-11-12-16-25-31-079.keras`

Let us test the model with a dataset (this is ideally a test set, but you may test the trained network with your training dataset for an initial trial.

```bash
conda activate agribots
cd path/to/agribot

source setup.bash
python neural_net/test_drive.py path/to/model path/to/data-dir
```

## How to Test Single Prediction

```bash
conda activate agribots
cd path/to/agribot

source setup.bash
python neural_net/test_run.py path/to/model path/to/image
```

## How to Visualize Heatmap

```bash
conda activate agribots
cd path/to/agribot

source setup.bash
python neural_net/visualize_heatmap.py path/to/model path/to/image
```

## How to Generate Images and Movie Showing Control Signals

- Generate images showing control signals.
    ```bash
    conda activate agribots
    cd path/to/agribot

    source setup.bash
    python neural_net/view_drive.py path/to/model path/to/images
    ```
    The images will be generated under `path/to/images/data-dir-name`

- Generate mp4 using the generated images.
    ```bash
    cd path/to/agribot

    sh make_video.sh path/to/images/data-dir-name
    ```
    Caution: Do not include `'/'` at the last of the directory name.


## Acknowledgments

### System Design and Implementation
- Jaerock Kwon, PhD, Assistant Professor, Electrical and Computer Engineering, University of Michigan-Dearborn

### Implementation and Contributors
- Elahe Delavari, PhD student, Electrical and Computer Engineering, University of Michigan-Dearborn
- Feeza Khanzada, PhD student, Electrical and Computer Engineering, University of Michigan-Dearborn
