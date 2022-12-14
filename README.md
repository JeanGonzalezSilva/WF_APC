# WF_APC
Closed-loop wind farm controller (active power tracking / thrust balancing / load-limiting) object for SOWFA simulations 

Author: J G Silva, D van der Hoek, B M Doekemeijer, R Ferrari and and J W van Wingerden

Date: May 30th, 2022

## Introduction
This work is to provide an open-source open-access controller that the wind energy community can use.
The proposed wind farm controller uses down-regulation to track an active power reference; balance the thrust forces between the wind turbines in the farm; and constrain aerodynamic loads in defined turbines.
The controller is written in MATLAB using the ZeroMQ interface. An example case uses an ADM-R implementation of DTU 10MW turbines and the TotalControl 32-turbine reference wind power plant layout.

## Using SOWFA and installation
Details on how to install and use SOWFA can be found in https://www.nrel.gov/wind/nwtc/sowfa.html. The adopted work is based on the SOWFA version in
https://github.com/TUDelft-DataDrivenControl/SOWFA which includes the ZeroMQ supercontroller.

## Content and instructions
This repository provides an example case including the simulation files for the precursor and the wind farm simulation. 
The SOWFA examples are universal, and should be usable on any cluster, but might need some alteration in the 'runscript.solve*' files. The simulation files herein are set for the DCSC cluster at the Delft University of Technology. 

The precursor is used to generate the turbulent inflow field using the windPlantSolver. To generate the precursor data from 'precursorCase\example_01_ABL_precursor_neutral_9x9x1km_lowTI_10mps' folder, run the 'runscript.preprocess' and consecutively both the 'runscript.solve.1' and the 'runscript.solve.2'. The first solver script simulates a long period of time, typically 20,000 seconds, so that turbulent structures can arise and let a quasi-equilibrium form. Then, the second solver script continues/restarts the simulation for more 3,000 seconds and generates the precursor data. 
You should see a folder called 'boundaryDataPre' and 'SourceHistory' in your 'postProcessing' folder. The folder 'boundaryDataPre' will contain folders '20000.5', '20001', '20001.5', and so on. The folder '20000.5' should be copied to a new folder called '20000' as the very first folder is missing.
```
cd postProcessing
cp -r boundaryDataPre/20000.5 boundaryDataPre/20000
```
Now, the contents from 'SOWFA\tools\boundaryDataConversion' should be copied to the 'postProcessing' folder, and run the script 'makeBoundaryDataFiles.west.sh' (note: you may have to change the first line of 'makeBoundaryDataFiles/data.py' and of 'makeBoundaryDataFiles/points.py' to fit your cluster). This will generate a folder called 'boundaryData' containing the inflow data for the 3,000 seconds of simulation with correct formatting. Copy the contents from 'SOWFA\tools\sourceDataConversion' to the 'postProcessing' folder, and run the script 'sourceHistoryRead.py' (python ./sourceHistoryRead.py). This will create a file called 'sources' in your 'postProcessing' folder. Then, create a new folder in your main case directory (i.e., next to the folders 'constant' and 'system') called 'drivingData'. Copy the file 'sources' and the folder 'boundaryData' to 'drivingData' (cp -r postProcessing/sources drivingData/sources && cp -r postProcessing/boundaryData/. drivingData/boundaryData/). Finally, in your main case directory, gather the state information from the various processors at time = 20,000 using OpenFOAM. The following command will generate your initial condition for the wind farm simulation.
```
reconstructPar -time 20000 -fields '(k kappat nuSgs p_rgh qwall Rwall T U)'
```

For the wind farm simulation example, 'exampleCase\example_01_TotalControl_wps_DTU10MW_ADM_refinements_zmqSSC' folder is given. Update the precursorDir to the directory of your precursor case in the file 'runscript.preprocess'. This directory should contain the folder 'drivingData' (containing the sources file and the boundaryData folder) and the folder '20000' (containing your initial conditions). 

The user can activate or deactivate the controller options in the 'ssc\SSC.m', which contains the wind farm controller including the centralized power losses compensator, the thrust force balancer and the user-defined thrust contrained turbine. The look up tables for the down-regulation is located in the 'ssc\controlTables' folder; and the implemented wind speed estimator based on the improved I&I tecnique (https://research.tudelft.nl/en/publications/the-immersion-and-invariance-wind-speed-estimator-revisited-and-n) is in the 'windSpeedEstimator' folder. 

To run the wind farm simulation example, run 'runscript.preprocess' and finally submit the 'runscript.solve' in the cluster.

## Referencing
If this controller played a role in your research, please cite either of the following articles:

J. G. Silva, B. Doekemeijer, R. Ferrari and J. -W. van Wingerden, "Active Power Control of Waked Wind Farms: Compensation of Turbine Saturation and Thrust Force Balance," 2021 European Control Conference (ECC), 2021, pp. 1223-1228, doi: 10.23919/ECC54610.2021.9655154.

J. G. Silva, D. van der Hoek, S. P. Mulders, R. Ferrari and J. -W. van Wingerden, "A Switching Thrust Tracking Controller for Load Constrained Wind Turbines," 2022 American Control Conference (ACC), 2022, pp. 4230-4235, doi: 10.23919/ACC53348.2022.9867888.

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
```
@INPROCEEDINGS{9867888,
  author={Silva, Jean Gonzalez and van der Hoek, Daan and Mulders, Sebastiaan Paul and Ferrari, Riccardo and van Wingerden, Jan-Willem},
  booktitle={2022 American Control Conference (ACC)}, 
  title={A Switching Thrust Tracking Controller for Load Constrained Wind Turbines}, 
  year={2022},
  volume={},
  number={},
  pages={4230-4235},
  doi={10.23919/ACC53348.2022.9867888}}
```

## Acknowledgments 
The authors would like to acknowledge the WATEREYE project (grant no. 851207). This project has received funding from the European Union Horizon 2020 research and innovation programme under the call H2020-LC-SC3-2019-RES-TwoStages.
