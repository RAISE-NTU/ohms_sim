# Contributing to OHMS-NetSim

Thank you for your interest in contributing to the Outdoor Heterogeneous Multi-Robot System Simulator (OHMS-NetSim)! We are delighted to have you. Every contribution, whether it's a bug report, a feature request, or a code change, is valuable to us.

To ensure a smooth and effective collaboration, please take a moment to review this document.

## Code of Conduct

All participants in this project are expected to adhere to our [Code of Conduct](./CODE_OF_CONDUCT.md). Please ensure you have read and understood it before contributing.

## How Can I Contribute?

There are several ways you can contribute to the project:

* **Reporting Bugs:** If you encounter a bug, please let us know by opening an issue.
* **Suggesting Enhancements:** If you have an idea for a new feature or an improvement to an existing one, we would love to hear about it.
* **Improving Documentation:** If you find parts of the documentation that are unclear or could be improved, you can open an issue or submit a pull request with your proposed changes.
* **Submitting Code:** If you would like to contribute code to fix a bug or implement a new feature, please submit a pull request.

## Reporting Bugs

Before submitting a new bug report, please check the existing issues to see if the bug has already been reported.

When submitting a bug report, please include the following information:

* **A clear and descriptive title:** For example, "Gazebo crashes when spawning more than 10 UAVs in the `forest5x` world."
* **A detailed description of the problem:** Explain the issue and what you expected to happen.
* **Steps to reproduce the behaviour:** Provide a clear, step-by-step description of how to reproduce the bug.
* **Your system setup:**
    * Operating System (e.g., Ubuntu 22.04)
    * ROS 2 Version (e.g., Humble Hawksbill)
    * Gazebo Version
    * Any other relevant details about your environment.

## Suggesting Enhancements

We welcome suggestions for new features and improvements. To submit a suggestion, please open an issue and provide:

* **A clear and descriptive title.**
* **A detailed description of the proposed enhancement.** Explain why this enhancement would be useful to the project.
* **Any relevant context,** such as mock-ups, examples, or links to related work.

## Submitting Pull Requests

If you would like to contribute code, please follow these steps:

1.  **Fork the repository** to your own GitHub account.
2.  **Create a new branch** for your changes: `git checkout -b feature/my-new-feature` or `git checkout -b fix/issue-number`.
3.  **Make your changes.** Please ensure your code adheres to the project's style guides.
4.  **Commit your changes** with a clear and descriptive commit message.
5.  **Push your branch** to your fork: `git push origin feature/my-new-feature`.
6.  **Open a pull request** to the `main` branch of the main OHMS-NetSim repository.
7.  **Provide a clear description** of the changes in your pull request, linking to any relevant issues.

### Development Setup

OHMS-NetSim is a standard ROS 2 package. To set up your development environment, you will need to have ROS 2 Humble installed. You can then clone your fork of the repository into your colcon workspace and build it using:

```bash
cd your_ros2_ws
colcon build --packages-select ohms_sim
source install/setup.bash
```

### Styleguides

To maintain a consistent code style throughout the project, please adhere to the following guidelines:

* **C++:** Please follow the [ROS 2 C++ Style Guide](https://docs.ros.org/en/humble/Contributing/Code-Style-Language-Versions.html#c).
* **Python:** Please follow the [PEP 8 Style Guide](https://peps.python.org/pep-0008/) and the [ROS 2 Python Style Guide](https://docs.ros.org/en/humble/Contributing/Code-Style-Language-Versions.html#python).
* **CMake:** Please follow the [ROS 2 CMake Style Guide](https://docs.ros.org/en/humble/Contributing/Code-Style-Language-Versions.html#cmake).

We look forward to your contributions!
