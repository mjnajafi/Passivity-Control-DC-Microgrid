Distributed Dissipativity-Based Controller and Topology Co-Design for DC Microgrids

This repository contains the simulation files associated with the paper:

Distributed Dissipativity-Based Controller and Topology Co-Design for DC Microgrids

Mohammad Javad Najafirad, Shirantha Welikala

Link to paper

Overview

This repository includes simulation files that implement the distributed control approach proposed in our paper. The approach focuses on voltage regulation in DC microgrids (MGs) which consist of interconnected distributed generators (DGs), distribution lines, and loads.

Key Features

Distributed Control Approach: Describes the closed-loop DC MG with a distributed controller as a networked system, with DGs and lines as subsystems interconnected via a static interconnection matrix.
Controller Design: Uses dissipative properties of subsystems to formulate a global linear matrix inequality (LMI) problem for controller gain design.
Local Controller Design: Proposes a local controller for each DG and line subsystem through a local LMI problem.
Compositional Design: Ensures stability during plug-and-play (PnP) operation by designing controllers based on the dynamics of new subsystems and the dissipativity information of coupled subsystems.


Dependencies
MATLAB R2020b or later
Control System Toolbox
Optimization Toolbox
References
Najafirad, M. J., & Welikala, S. (2024). Distributed Dissipativity-Based Controller and Topology Co-Design for DC Microgrids. arXiv:2404.18210

Contact
For any questions or issues, please contact:

Mohammad Javad Najafirad: mnajafir@stevens.edu

License
This project is licensed under the MIT License - see the LICENSE file for details.

