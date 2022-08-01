# WF_APC
Closed-loop wind farm controller (active power tracking / thrust balance / load constraint) object for SOWFA simulations 

Author: J G Silva, D van der Hoek, B M Doekemeijer, R Ferrari and and J W van Wingerden

Date: May 30th, 2022

## Introduction
This work is to provide an open-source open-access controller that can be used by the wind energy community.
The proposed wind farm controller make use of down-regulation to track an active power reference, to balance the thrust forces between the wind turbines in the farm, and to constraint aerodynamic loads in defined turbines.
The controller is written in MATLAB using the ZeroMQ interface. An example case uses an ADM-R implementation of DTU 10MW turbines and the TotalControl 32-turbine reference wind power plant layout.

## Using SOWFA
Details on how to install and use SOWFA can be found in https://www.nrel.gov/wind/nwtc/sowfa.html. The adopted work is based on the SOWFA version in
https://github.com/TUDelft-DataDrivenControl/SOWFA which includes the ZeroMQ supercontroller.

## Content
This repository provides an example case including the simulation files for the precursor and the wind farm simulation. The precursor is used to generate the turbulent inflow field using the windPlantSolver. In the wind farm simulation, the user can activate or deactivate the controller options which provide the centralized power losses compensator, the thrust force balancer and the user-defined thrust contrained turbine.

## Referencing
If this controller played a role in your research, please cite:
J. G. Silva, B. Doekemeijer, R. Ferrari and J. -W. van Wingerden, "Active Power Control of Waked Wind Farms: Compensation of Turbine Saturation and Thrust Force Balance," 2021 European Control Conference (ECC), 2021, pp. 1223-1228, doi: 10.23919/ECC54610.2021.9655154.

For LaTeX users:
```
   @INPROCEEDINGS{9655154,
  author={Silva, Jean Gonzalez and Doekemeijer, Bart and Ferrari, Riccardo and van Wingerden, Jan-Willem},
  booktitle={2021 European Control Conference (ECC)}, 
  title={Active Power Control of Waked Wind Farms: Compensation of Turbine Saturation and Thrust Force Balance}, 
  year={2021},
  volume={},
  number={},
  pages={1223-1228},
  doi={10.23919/ECC54610.2021.9655154}}
```

## Acknowledgments 
The authors would like to acknowledge the WATEREYE project (grant no. 851207). This project has received funding from the European Union Horizon 2020 research and innovation programme under the call H2020-LC-SC3-2019-RES-TwoStages.
