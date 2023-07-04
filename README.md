# ros2_prescyent
Ros2 node for Prescyent  

# Install

Install prescyent locally before building node:  

```bash
cd prescyent  # check that you are in the branch of the version you want to build
pip install .
```

Then build this ros2 node in your ros2 workspace using:  
```bash
colcon build
source ~/.bashrc
```

# Run

Launch with default params using `ros2 run ros2_prescyent ros2_predict`  
You can also override default ros params using the ros2 launch command `ros2 launch ros2_prescyent ros2_predict.launch.py`  

# Ros2 Params
|NAME|TYPE|DESCRIPTION|DEFAULT|
| :--- | :----: | ---: | ---: |
| predictor_path | str | Path to the prescyent predictor to load | "" (An empty path inits to the "ConstantPredictor" as default) |
| history_size | int | Size of the PoseArray buffer used as input for the prediction | 10 |
| future_size | int | Size of the PoseArray buffer that is outputed by the predictor | 10 |
| predictor_frequency | int | The frequency of the predictor model's inputs and outputs in Hz | 10 |

With theses default values, we have a frequency of 10hz and inputs and outputs sequences of size 10, meaning that we have a history of 1 second to predict one second in the future.  