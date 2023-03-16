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
1.	M. Kloock, P. Scheffe, O. Greß, and B. Alrifaee, “An Architecture for Experiments in Connected and Automated Vehicles,” IEEE Open Journal of Intelligent Transportation Systems, pp. 1–1, 2023, doi: 10.1109/OJITS.2023.3250951.\
https://youtu.be/PfM0qdzorCc \
https://cpm.embedded.rwth-aachen.de/architecture/ 
2.	M. Kloock, P. Scheffe, and B. Alrifaee, “Testing Distributed Trajectory Planning in the Cyber-Physical Mobility Lab.” Accepted in at - Automatisierungstechnik, 2023.\
https://youtu.be/fzuB0xsl4Jc 
3.	A. Mokhtarian, P. Scheffe, S. Kowalewski, and B. Alrifaee, “Remote Teaching with the Cyber-Physical Mobility Lab,” IFAC-PapersOnLine, vol. 55, no. 17, pp. 386–391, 2022, doi: 10.1016/j.ifacol.2022.09.310.\
https://cpm.embedded.rwth-aachen.de/education/ \
https://cpm-remote.embedded.rwth-aachen.de/academy 
4.	A. Mokhtarian, S. Schäfer, and B. Alrifaee, “CPM Olympics: Development of Scenarios for Benchmarking in Networked and Autonomous Driving,” in 2022 IEEE Intelligent Vehicles Symposium (IV), 2022, pp. 9–15. doi: 10.1109/IV51971.2022.9827299.\
https://cpm-remote.embedded.rwth-aachen.de/olympics 
5.	A. Mokhtarian and B. Alrifaee, “CPM Remote: A Remote Access to the CPM Lab,” in 2022 8th International Conference on Control, Decision and Information Technologies (CoDIT), 2022, vol. 1, pp. 1124–1129. doi: 10.1109/CoDIT55151.2022.9804088.\
https://youtu.be/U-A0JYubyAI \
https://cpm-remote.embedded.rwth-aachen.de/ 
6.	M. Kloock, P. Scheffe, J. Maczijewski, A. Kampmann, A. Mokhtarian, S. Kowalewski, and B. Alrifaee, “Cyber-Physical Mobility Lab: An Open-Source Platform for Networked and Autonomous Vehicles,” in 2021 European Control Conference (ECC), 2021, pp. 1937–1944. doi: 10.23919/ECC54610.2021.9654986.\
https://youtu.be/mQoCkMYvmLE \
https://youtu.be/PfM0qdzorCc \
https://cpm.embedded.rwth-aachen.de/ 
7.	P. Scheffe, J. Maczijewski, M. Kloock, A. Kampmann, A. Derks, S. Kowalewski, and B. Alrifaee, “Networked and Autonomous Model-scale Vehicles for Experiments in Research and Education,” IFAC-PapersOnLine, vol. 53, no. 2, pp. 17332–17337, 2020, doi: 10.1016/j.ifacol.2020.12.1821.\
https://youtu.be/Xv1DwgW0oDc \
https://youtu.be/84R28cSxWQU \
https://cpm.embedded.rwth-aachen.de/vehicle/ 
8.	M. Kloock, P. Scheffe, I. Tülleners, J. Maczijewski, S. Kowalewski, and B. Alrifaee, “Vision-Based Real-Time Indoor Positioning System for Multiple Vehicles,” IFAC-PapersOnLine, vol. 53, no. 2, pp. 15446–15453, 2020, doi: 10.1016/j.ifacol.2020.12.2367.\
https://youtu.be/Ph48c2jE59c \
https://youtu.be/k6aD5G9DW4o \
https://cpm.embedded.rwth-aachen.de/indoor-positioning-system/ 
