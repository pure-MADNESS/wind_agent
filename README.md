# wind_agent plugin for MADS

This is a Source plugin for [MADS](https://github.com/MADS-NET/MADS). 

<provide here some introductory info>

*Required MADS version: 2.0.0.*


## Supported platforms

Currently, the supported platforms are:

* **Linux** 
* **MacOS**
* **Windows**


## Installation

Linux and MacOS:

```bash
cmake -Bbuild -DCMAKE_INSTALL_PREFIX="$(mads -p)"
cmake --build build -j4
sudo cmake --install build
```

Windows:

```powershell
cmake -Bbuild -DCMAKE_INSTALL_PREFIX="$(mads -p)"
cmake --build build --config Release
cmake --install build --config Release
```


## INI settings

The plugin supports the following settings in the INI file:

```ini
[wind_agent]
# Describe the settings available to the plugin
```

---

# Overview
This agent implements a management system for a wind power source. It considers a simplified model on which an Extended Kalman Filter is running. It considers to have access to measurements on angular velocity (from a gyroscope or IMU) and angular step move (from an encoder). The input data can be either simulated or coming from real experiments.

As a source node, it provides to the others its mean generating power and covariance. Then it will negotiate with the other sources the amount of power to actually provide.
For the second step, this node as access to weather forecasts, in order to extend the ergodic feedback on a future time window.
