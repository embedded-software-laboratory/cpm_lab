# CPM Lab
This repository contains source code for the [CPM Lab](https://cpm-lab.embedded.rwth-aachen.de), an open source platform for networked and autonomous vehicles. 
The CPM Lab provides a simulation environment and 20 model-scale vehicles for experiments. The software architecture enables the seamless transfer of algorithms tested in simulation to real experiments without any adaptions. Experiments with the 20 vehicles can be extended by unlimited additional simulated vehicles. A Data Distribution Service (DDS) based middleware is responsible for synchronizing all entities and establishes a logical execution time approach. It further allows to change the number of vehicles during experiments. \
The CPM Lab lets researchers as well as students from different disciplines see their ideas develop into reality.
More information is provided in our publication [1], which we kindly ask you to consider citing if you find the CPM Lab helpful for your work.

## Switching between the RTI and FastDDS Versions

The ``install.sh`` script in both RTI and FastDDS versions of the CPM Lab set
some required, but different environment variables in a source file.
When switching back from FastDDS to RTI this can lead to unexpected and hard to debug errors.
Running ``sudo install.sh`` would fix this, but each install takes quite a bit of time.

Contained is a ``quick_install.sh`` script, that sets these variables,
but does not repeat long-running steps of the installer.
Thus, when the CPM Lab is not building properly after switching back from FastDDS,
it is recommended to run ``sudo bash quick_install.sh``, specify the RTI license file,
specify domain ID (usually 21), reboot the computer and then run ``bash build_all.bash``.

## Documentation
The [documentation](https://cpm-lab.embedded.rwth-aachen.de/doc/) describes how to set up the simulation environment of the lab and how to set up the complete lab hardware. It also explains its usage with tutorials and documents its components.

## Acknowledgements
This research is supported by the Deutsche Forschungsgemeinschaft (German Research Foundation) within the Priority Program SPP 1835 "Cooperative Interacting Automobiles" (grant number: KO 1430/17-1), the Post Graduate Program GRK 1856 "Integrated Energy Supply Modules for Roadbound E-Mobility", the project "CiTi – Center for integrative Traffic investigation", the fund “digital teaching and learning infrastructure” of the state NRW, and the quality improvement funds at RWTH Aachen University.

## References
* [1] M. Kloock, P. Scheffe, J. Maczijewski, A. Kampmann, A. Mokhtarian, S. Kowalewski and B. Alrifaee. "Cyber-physical mobility lab: An open-source platform for networked and autonomous vehicles". 2021 European Control Conference (ECC). IEEE, 2021. \
  https://www.youtube.com/watch?v=PfM0qdzorCc  
* [2] M. Kloock, P. Scheffe, I. Tülleners, J. Maczijewski, S. Kowalewski and B. Alrifaee. "Vision-Based Real-Time Indoor Positioning System for Multiple Vehicles". IFAC World Congress 2020. \
  https://www.youtube.com/watch?v=k6aD5G9DW4o
* [3] P. Scheffe, J. Maczijewski, M. Kloock, A. Derks, S. Kowalewski, B. Alrifaee. "Networked and Autonomous Model-scale Vehicles for Experiments in Research and Education". IFAC World Congress 2020. \
  https://www.youtube.com/watch?v=84R28cSxWQU
