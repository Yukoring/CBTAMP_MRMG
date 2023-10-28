# CBTAMP_MRMG

This repository contains code for "Conflict-Based Task and Motion Planning for Multi-Robot Multi-Goal Problem (CBTAMP)" - Junho Lee & Derek Long

This solver consists of CBTAMP and their baseline techniques, including
  * Task Planner Only Method
  * Prioritizd Planning

### Dependencies

This program is written in python 3.7 and uses matplotlib, scipy, shapely
'''shell script
pip3 install matplotlib scipy shapely
'''

### Usage
Run the code
'''shell script
python3 framework.py -m map_file/Sample.yaml -d pddl_domain/early_cbtamp.pddl -s CBTAMP -pl -v -t 20
'''

- m: The name of input map file
- d: The name of input domain file
- s: The name of solver (Default: CBTAMP)
- pl: The use of visualization (Default: False)
- v: The use of Voronoi sample (Default: False)
- t: The number of trial

You can find more details and explanation for all parameters with:
'''shell script
python3 framework.py --help
'''

* In 'map_file' folder, Scenario and Scalability evaluation maps available.
* In 'figure' and 'pddl_prob_plan' folder, output figure and plans are generated.
* Before running, check the planners and wrappers runnable in 'planners' folder.




