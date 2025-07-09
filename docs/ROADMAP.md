# **OHMS-NetSim: A Phased Release Plan**

## **1\. Introduction and Guiding Principles**

To ensure the long-term sustainability, usability, and growth of OHMS-NetSim, we are adopting a structured, phased release plan. This plan is designed to provide transparency to our users and contributors, outlining the project's development trajectory from its current prototype stage to a stable, feature-rich simulation tool.

Our development will adhere to the principles of Semantic Versioning (SemVer). All releases will follow a `MAJOR`.`MINOR`.`PATCH` format:

* MAJOR version increments indicate incompatible API changes.  
* MINOR version increments signify the addition of functionality in a backwards-compatible manner.  
* PATCH version increments are for backwards-compatible bug fixes.

This approach will ensure that users can update the software with confidence, understanding the scope of changes in each new version.

## **2\. Release Phases and Timeline**

### **Phase 1: v0.1.0 — Initial Prototype**

This is the version submitted for academic review. It serves as a proof-of-concept, demonstrating the core functionalities and the scientific underpinnings of the simulator.

* Status: Prototype  
* Key Features:  
  * ROS 2 Humble integration for multi-robot simulation in Ignition Gazebo.  
  * Modular architecture using reusable launch files and YAML-based configuration for defining complex scenarios.  
  * A custom Gazebo plugin, `CommsEmulatorHelper`, for simulating network conditions.  
  * Initial implementation of scientifically-grounded path loss models for forested environments, with in-code citations to relevant literature.  
  * A library of pre-configured robot models (UGVs and UAVs) and environment worlds.  
  * Version-Controlled Documentation: All project documentation is maintained in Markdown format directly within the main GitHub repository. This approach ensures that the documentation is version-controlled in lockstep with the source code. The initial documentation is structured with a comprehensive `README.md` as the primary entry point, which links to a `/docs` directory containing detailed guides, tutorials, and supplementary materials.  
  * Community Governance Framework: To foster a collaborative and inclusive community, this initial version establishes a clear governance framework. This includes the creation of a `CONTRIBUTING.md` file, which details the process for submitting bug reports, feature requests, and pull requests, alongside the adoption of a `CODE\_OF\_CONDUCT.md` to ensure a welcoming development environment for all contributors.  
* Target Audience: Researchers and developers requiring a foundational tool for multi-robot simulation with basic communication emulation.

### **Phase 2: v0.2.0 — First Public Release**

This release will focus on solidifying the project's foundations, addressing the key concerns raised during the peer-review process, and preparing the simulator for wider community adoption.

* Target Timescale: 1-2 months post-publication.  
* Primary Goals: Enhance usability, foster community engagement, and improve verifiability.  
* Planned Features:  
  * Expansion of Documentation: The documentation within the `/docs` directory will be expanded to include: step-by-step tutorials for creating new simulation scenarios and guides for extending the simulator with new robot models.  
  * Expanded and Configurable Communication Models:  
    * Enable the configuration of communication model parameters from within the world's SDF file, allowing for self-contained simulation environments.  
    * Allow users to select from multiple path loss models via configuration files.  
  * Stability and Refinement: This release will also serve as an opportunity to enhance the stability and robustness of the simulator. We will actively incorporate any bug fixes and minor refinements that have been identified during the peer-review process and from initial feedback provided by early adopters.

### **Phase 3: v1.0.0 — First Stable Release**

This will be the first official, stable release of OHMS-NetSim. The focus will be on robustness, reliability, and establishing the simulator as a trusted tool for repeatable scientific experiments.

* Target Timescale: 6-9 months post-publication.  
* Primary Goals: Ensure long-term stability and expand core simulation capabilities.  
* Planned Features:  
  * Ongoing Maintenance and Bug Fixes: This release marks our commitment to long-term support. We will provide regular patch releases to address bugs, security vulnerabilities, and other issues reported by the community, ensuring the simulator remains a reliable tool for research.

### **Phase 4: Long-Term Vision (Post-v1.0.0)**

Beyond the first stable release, we envision OHMS-NetSim evolving to meet the growing needs of the robotics research community. Potential future directions include:

* Evolving the Gazebo plugin into a fully-featured network emulation suite with a modular architecture for the straightforward integration of new propagation and channel models.  
* Support for more advanced network emulation, including dynamic latency, and jitter models.  
* Development of a broader, community-contributed library of robot models, sensors, and simulation worlds.
