# ⚠️ This repository is no longer maintained.

Please note that this repository is archived and will not receive further updates. For the latest developments, please refer to https://github.com/bassamlab/cpm-lab.

# CPM Lab
This repository contains source code for the  [Cyber-Physical Mobility Lab (CPM Lab)](https://cpm.lrt.unibw.de).
The CPM Lab is an open-source, remotely accessible testbed for Connected and Automated Vehicles (CAVs), with a focus on multi-agent decision-making. It integrates a digital twin with 20 physical small-scale vehicles (µCars), enabling seamless testing and validation of algorithms across simulation and real-world experiments without code changes. The CPM Lab architecture relies on a Data Distribution Service (DDS)-based middleware that synchronizes all entities via a logical execution time approach, supporting reproducibility of experiments.

As one of the pioneering testbeds for CAVs, the CPM Lab has been rebuilt at other universities and continues to serve as a valuable platform for research, education, and training the next generation of engineers in intelligent transportation systems. For more details, please refer to our publication [1], which we kindly ask you to cite if you find the CPM Lab helpful for your work.

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
The [documentation](https://cpm.lrt.unibw.de/doc/) describes how to set up the simulation environment of the lab and how to set up the complete lab hardware. It also explains its usage with tutorials and documents its components.

## References
1.	M. Kloock, P. Scheffe, O. Greß, and B. Alrifaee, “An Architecture for Experiments in Connected and Automated Vehicles,” IEEE Open Journal of Intelligent Transportation Systems, pp. 1–1, 2023, doi: 10.1109/OJITS.2023.3250951.\
https://youtu.be/PfM0qdzorCc \
https://cpm.lrt.unibw.de/architecture/ 

For a full list of publications related to the CPM Lab see https://cpm.lrt.unibw.de/publications.
