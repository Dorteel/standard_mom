# A Standard Model of the Mind

This is an ongoing attempt to implement the Standard Model of the Mind as proposed by [Laird et al.](https://ojs.aaai.org/index.php/aimagazine/article/view/2744).

&nbsp;
## General architecture

The following sections describe the details of the different components as described in the [original paper](https://ojs.aaai.org/index.php/aimagazine/article/view/2744).

&nbsp;
### Working Memory

This component is responsible for composing new symbol structures from the outputs of the perception and long-term memory modules.

It has buffers that store the results of perception and the declarative memory retirevals, as well as storing the motor commands and memory queries.

It should also temprorary information necessary for:

- behaviour production
- problem solving
- information about goals
- model of a task 

The components of this module should be available for introspection to the procedural long-term memory module.

&nbsp;
### Procedural memory
&nbsp;
This module contains knowledge about external and internal procedures, which includes both the conditions (rules) and the actions. These conditions and actions are defined over the contents of the working memory.

The output of the procedural memory is a single action (which enters the action buffer).

&nbsp;
### Declarative memory

This memory structure can be represented as a knowledge graph, containing facts and symbolic relations.

The authors do not exclude the possibility of dividing this memory system into contextualized episodic memory and semantic memory.
&nbsp;

&nbsp;

![Employee data](/images/the_structure_of_the_standard_model.png?raw=true "The Structure of the Standard Model")
&nbsp;

---
### TO DO
- Implement Metadata
- Connect with ROS
- Create launch files
- Implement buffers