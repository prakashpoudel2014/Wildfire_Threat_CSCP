# A-CSCP Forest Fire Simulation

MATLAB code for simulating adaptive path planning and sensor tasking in a dynamic forest fire environment. The repository implements a forest-fire variant of the A-CSCP framework, combining environment propagation, mobile sensing, state estimation, image-based observation generation, segmentation, and path replanning for an ego vehicle operating under evolving threat conditions.

## Overview

The codebase is organized around a main live script that runs the full simulation workflow. Supporting classes define the environment, actor, sensors, and sensor network, while modular functions handle fire-state propagation, estimation, image generation, path planning, segmentation, and sensor retasking.

At a high level, the framework supports:

- simulation of a dynamic forest fire environment
- ego-vehicle path planning under threat
- coordinated tasking of mobile sensors
- state estimation from intermittent observations
- synthetic image generation and image-based fire-state classification
- comparison of multiple sensor retasking strategies
- visualization of coverage and simulation outputs

## Main Entry Point

- `main_ACSCP_ForestFire.mlx`  
  Primary script for running the forest fire A-CSCP simulation.

## Main Classes

- `Environment.m` / `ForestFireEnvironment.m`  
  Base and specialized environment representations.

- `SensorNetwork.m` / `ForestFireSensorNetwork.m`  
  Base and specialized sensor-network controllers.

- `Platform.m`  
  Base platform representation.

- `Actor.m`  
  Ego vehicle logic and path execution.

- `Sensor.m`  
  Sensor platform behavior and measurement handling.

## Core Functions

- `computeOptimalPath.m`  
  Path planning for the ego vehicle.

- `estimateFn.m` / `estimateFn_POD.m`  
  State-estimation routines.

- `fireStepFn.m`  
  Fire-state propagation dynamics.

- `initialStateFn.m`  
  Initialization of the simulation state.

- `imageGenFn.m`  
  Synthetic image-generation support for sensor observations.

## Sensor Retasking Methods

The repository includes multiple external sensor-retasking strategies for comparison:

- `retaskSensors_external.m`
- `retaskSensors_external_CRMI.m`
- `retaskSensors_external_CRMI_POD.m`
- `retaskSensors_external_CRI_PS.m`
- `retaskSensors_external_SMI.m`
- `retaskSensors_external_SMI_POD.m`
- `retaskSensors_external_lawnmower.m`
- `retaskSensors_external_random.m`

These functions provide alternative policies for assigning sensors to new measurement locations.

## Additional Files and Models

- `fig_edit.m`  
  Figure editing / export utility.

- `POD_model_50`  
  POD model data used by reduced-order estimation methods.

- `checkpoint_epoch_090`  
  Trained ResNet-based image generator used to produce synthetic observations.

- `unet_final_model`  
  Trained U-Net segmentation model used to classify generated fire imagery.

- `Map1.png`  
  Map or domain asset used by the simulation.

## Usage

Open and run:

```matlab
main_ACSCP_ForestFire
```

Ensure all repository files are on the MATLAB path and that required model/data files are available before execution.

## License

See the `LICENSE` file for licensing and usage terms.
