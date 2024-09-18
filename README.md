# Multi-Agent Combinatorial Path Finding with Task Duration

This is the code repo for the paper [Multi-Agent Combinatorial Path Finding with Heterogeneous Task Duration](https://arxiv.org/abs/2311.15330).

The code is distributed for academic and non-commercial use.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

## Demo
Compare the two methods proposed in our work — CBSS-TPG and CBSS-D:

![Demo1](https://github.com/rap-lab-org/public_pymcpf-d/blob/main/data/demo/demo_1.gif)

Deploy CBSS-D as the planner and TPG as the scheduler in Gazebo and Rviz:

![Demo2](https://github.com/rap-lab-org/public_pymcpf-d/blob/main/data/demo/demo_2.gif)

## Requirements

* We use Python (3.8.10) and Ubuntu 20.04. Lower or higher version may also work.
* [LKH-2.10.0](http://webhotel4.ruc.dk/~keld/research/LKH/) is required as the underlying TSP solver. The executable of LKH should be placed at location: pytspbridge/tsp_solver/LKH-2.10.0/LKH. In other words, run `pytspbridge/tsp_solver/LKH-2.10.0/LKH` command in the terminal should be able to invoke LKH.

## Instructions
* For example, run:
  ```python
  python3 run_example.py
  ```

* For large-scale benchmark, run:
  ```python
  python3 run_cbss_mcpfd.py
  ```
  
  It may take a long time to finish running all the scens in the [dataset](https://movingai.com/benchmarks/mapf.html). You can adjust the params `N_list`, `M_list` and `duration_list` to decrease the workload.

## Notes
Under `libmcpfd`, the code structure is as follows:
- `cbss_d`: contains the code for CBSS-D with the new branching rules
- `cbss_d_old`: contains the code for CBSS-D with the old branching rules
- `cbss_tpg`: contains the code for CBSS-TPG

The rest python scripts are the basic components supporting the codes above.

## Others

### About the Low-Level Search
The original implementation has a flaw in its low-level search, which runs sequential A* and ignores the influence across targets. As a result, it may not return an optimal individual path. The latest version (tag: v1.1 and thereafter) has fixes this issue on the low-level search.

### About Solution Optimality

CBSS is theoretically guaranteed to find an optimal or bounded sub-optimal solution joint path, when the underlying TSP solver is guaranteed to solve TSPs to optimality.
The current implementation of CBSS depends on LKH, which is a popular heuristic algorithm that is not guaranteed to find an optimal solution to TSP. Therefore, the resulting CBSS implementation is not guaranteed to return an optimal solution.
However, LKH has been shown to return an optimal solution for numerous TSP instances in practice, this implementation of CBSS should also be able to provide optimal solutions for many MCPF instances.
If the optimality of the solution must be guaranteed, one can consider leveraging the [Concorde](https://www.math.uwaterloo.ca/tsp/concorde.html) TSP solver (or other TSP solvers that can guarantee solution optimality) to replace LKH.

### Related Papers

[1] Yuanhang Zhang, Hesheng Wang, and Zhongqiang Ren. "Multi-Agent Combinatorial Path Finding with Heterogeneous Task Duration." under review.
[[arXiv](https://arxiv.org/abs/2311.15330)]
[[Video](https://www.youtube.com/embed/sSX0HdzjmY4)]
[[Code](https://github.com/hang0610/public_pymcpf-d)]

[2] Ren, Zhongqiang, Sivakumar Rathinam, and Howie Choset. "CBSS: A New Approach for Multiagent Combinatorial Path Finding." IEEE Transaction on Robotics (T-RO), 2023.\
[[Bibtex](https://wonderren.github.io/files/bibtex_ren23cbssTRO.txt)]
[[Paper](https://wonderren.github.io/files/ren23_CBSS_TRO.pdf)]
[[Talk](https://youtu.be/V17vQSZP5Zs?t=2853)]

## Contact
If you have any questions, feel free to contact [Yuanhang Zhang](https://hang0610.github.io/).
