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
Files
simulation.m: MATLAB script for running the simulations.
controller_design.m: MATLAB script for designing the distributed controllers.
network_topology.m: MATLAB script for setting up the communication topology.
data/: Directory containing any required data files.
How to Use
Clone this repository to your local machine:

bash
Copy code
git clone https://github.com/yourusername/yourrepository.git
Navigate to the directory:

bash
Copy code
cd yourrepository
Open MATLAB and run the simulation.m script to start the simulation.

Modify controller_design.m and network_topology.m as needed for your specific configurations.

Dependencies
MATLAB R2020b or later
Control System Toolbox
Optimization Toolbox
References
Najafirad, M. J., & Welikala, S. (2024). Distributed Dissipativity-Based Controller and Topology Co-Design for DC Microgrids. arXiv:2404.18210
Contact
For any questions or issues, please contact:

Mohammad Javad Najafirad: email@example.com
License
This project is licensed under the MIT License - see the LICENSE file for details.

