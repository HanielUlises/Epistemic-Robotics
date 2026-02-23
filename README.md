# Epistemic-Robotics

**Formal multi-agent task planning under epistemic uncertainty**

Epistemic-Robotics is a research-oriented framework for multi-agent task planning in partially observable and dynamically evolving environments. The project combines epistemic logic, symbolic planning, and robotic execution to bridge the gap between formal semantics and real-world autonomous systems.

At its core, the system integrates **Dynamic Epistemic Logic (DEL)** with Kripke-based belief modeling to represent and update agents’ knowledge about the world and about each other. These epistemic models are connected to an EPDDL-inspired task planning layer, enabling reasoning over actions with epistemic preconditions and effects (e.g., sensing, announcements, information gain).

The planning component is designed to interoperate with PDDL-style planners and SysPlan-like task planning pipelines, allowing epistemic planning problems to be compiled or coordinated with classical planners when appropriate. This supports hybrid workflows where symbolic task planning, epistemic reasoning, and robotic execution coexist within a unified architecture.

## Architecture Overview

The system is structured in three conceptual layers:

**Cognitive Layer**  
Implements the epistemic world model, DEL-based belief updates, task planner, and dispatcher. This layer handles symbolic reasoning, epistemic queries, and the generation of executable task plans under uncertainty.

**Execution Layer**  
Built on ROS2, this layer coordinates multi-robot task execution. It translates high-level epistemic plans into concrete robotic behaviors, ensuring synchronization, communication, and distributed action execution.

**Perception Layer**  
Connects SLAM and sensor-driven estimation modules to the epistemic model. Perceptual events (e.g., LIDAR observations) are lifted into epistemic events, enabling belief revision and information-aware planning in uncertain or partially known environments.

## Research Focus

The project explores:

- Task planning under partial observability and knowledge constraints  
- EPDDL-style modeling of epistemic actions  
- Integration of symbolic planning with ROS2-based execution  
- Event-based belief revision grounded in real sensor input  
- Alignment between formal logic specifications and executable robotic systems  

Epistemic-Robotics aims to serve both as a practical experimental platform and as a formal research artifact for studying epistemic task planning in autonomous multi-agent robotics.