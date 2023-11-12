# System Configuration

## conda
- `environment.yaml`: conda environment file


```
$ conda env create -f conda/environment.yaml
```

## config.yaml
Specify yaml file names for each system setting.
- `neural_net`
- `data_collection`
- `run_neural`

### Naming Convention
Use `<vehicle>-<user>-<track>{--note}.yaml` for a yaml filename. 

For example, `scout-jaerock-orchard--aug` means that vehicle `scout` is being used by user `jaerock` on the `orchard` world with the `augmentation` option.

