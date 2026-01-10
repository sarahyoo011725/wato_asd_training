## Project Demo
This video demonstrates the current prototype of our autonomous robot, showcasing LiDAR-based SLAM mapping and path planning in action.  

[![Watch the Demo](https://img.youtube.com/vi/wIKEoOrXFpg/0.jpg)](https://www.youtube.com/watch?v=wIKEoOrXFpg)  
*Click the image to watch the full demo on YouTube.*

## Prerequisite Installation

These steps will set up the monorepo on your local machine. We use **Docker** to ensure reproducibility and simplify deployment.  

1. **Operating System Support:**  
   This project supports **Linux (Ubuntu ≥ 22.04), Windows (via WSL), and macOS**. You can set up your environment using one of the following approaches:  
   - [Ubuntu Virtual Machine](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview)  
   - [Windows Subsystem for Linux (WSL)](https://learn.microsoft.com/en-us/windows/wsl/install)  
   - [Dual Boot Linux Setup](https://opensource.com/article/18/5/dual-boot-linux)  

2. **Install Docker:**  
   Once your Linux environment is ready, follow the [official Docker installation guide](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository) to install Docker Engine.  

3. **Build and Run the Project:**  
   - Build the project:  
     ```bash
     ./watod build
     ```  
   - Start the simulation locally on Foxglobe:  
     ```bash
     ./watod up
     ```  
