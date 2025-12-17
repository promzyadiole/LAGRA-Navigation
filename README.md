# LAGRA: Language-Grounded Robot Autonomy

**LAGRA** (Language-Grounded Robot Autonomy) is a research-oriented ROS 2 framework for
multimodal, language-driven mobile robot navigation and interaction.

The framework integrates **large language models (LLMs)** and **vision–language models (VLMs)**
with classical robotic navigation systems (**ROS 2 Nav2**) to enable robots to interpret
natural-language commands, perform semantic perception, and execute autonomous navigation
in structured indoor environments.

LAGRA is designed for **research, reproducibility, and extensibility**, and serves as a
baseline for work in language-grounded robotics, multimodal perception, and human–robot
interaction.


## 🧩 Architecture Diagrams

### Overall System Architecture
![LAGRA System Architecture](docs/architecture/lagra_system_architecture.png)

### Execution Pipeline
![LAGRA Execution Pipeline](docs/architecture/lagra_execution_pipeline.png)

---

## 🔧 Automatic Dependency Installation

To install all required system and Python dependencies, run:

```bash
chmod +x install_deps.sh
./install_deps.sh


## 🎓 For PhD Supervisors and Research Collaborators

This repository accompanies the Master’s thesis:

**“Multimodal Language-Grounded Frameworks for Autonomous Robot Navigation and Interaction”**

The work investigates how **large language models (LLMs)** and **vision–language models (VLMs)**
can be integrated with classical robotic navigation systems (**ROS 2 Nav2**) to enable
semantic, language-driven autonomy in mobile robots.

### Research Contributions

- A modular ROS 2 framework bridging symbolic language understanding and geometric navigation  
- Integration of LLM-based reasoning with vision grounding (CLIP, BLIP-2, SAM)  
- A task execution layer that converts semantic intents into executable Nav2 goals  
- A reproducible simulation pipeline for evaluating language-grounded navigation  

### Relevance for PhD Research

The framework is designed to be extensible toward future doctoral research in:

- Human–Robot Interaction (HRI)  
- Language-grounded and multimodal robot autonomy  
- Foundation models for robotics  
- Task and motion planning with natural language interfaces  
- Cognitive robotics and embodied AI  

The codebase emphasizes **modularity**, **reproducibility**, and **research clarity**, making it
a strong baseline for further experimental and theoretical investigation.

For citation and academic reference, please use the provided **CITATION.cff** file.

