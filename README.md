# Multi-Agent Combinatorial Path Finding with Task Duration (Under Construction)

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
Please refer to the [video](https://sciencecast.org/casts/9oyjmr42cn8t).

## Requirements

* We use Python (3.8.10) and Ubuntu 20.04. Lower or higher version may also work.
* [LKH-2.10.0](http://webhotel4.ruc.dk/~keld/research/LKH/) is required as the underlying TSP solver. The executable of LKH should be placed at location: pytspbridge/tsp_solver/LKH-2.10.0/LKH. In other words, run `pytspbridge/tsp_solver/LKH-2.10.0/LKH` command in the terminal should be able to invoke LKH.

## Instructions
* ```python
  python3 run_cbss_mcpfd.py
  ```
  
  It may take a long time to finish running all the scens in the [dataset](https://movingai.com/benchmarks/mapf.html). You can adjust the params `N_list`, `M_list` and `duration_list` to decrease the workload.
* ```python
  python3 analytics.py
  ```
  
  It can analyze the results obtained from running `run_cbss_mcpfd.py`. Be sure to align the `N_list`, `M_list` and `duration_list` with those in `run_cbss_mcpfd.py`.

## Notes
Under `libmcpfd`, the code structure is as follows:
- `cbss_d`: contains the code for CBSS-D with the new branching rules
- `cbss_d_old`: contains the code for CBSS-D with the old branching rules
- `cbss_tpg`: contains the code for CBSS-TPG

The rest python scripts are the basic components supporting the codes above.

## Others

### About Solution Optimality

CBSS is theoretically guaranteed to find an optimal or bounded sub-optimal solution joint path, when the underlying TSP solver is guaranteed to solve TSPs to optimality.
The current implementation of CBSS depends on LKH, which is a popular heuristic algorithm that is not guaranteed to find an optimal solution to TSP. Therefore, the resulting CBSS implementation is not guaranteed to return an optimal solution.
However, LKH has been shown to return an optimal solution for numerous TSP instances in practice, this implementation of CBSS should also be able to provide optimal solutions for many MCPF instances.
If the optimality of the solution must be guaranteed, one can consider leveraging the [Concorde](https://www.math.uwaterloo.ca/tsp/concorde.html) TSP solver (or other TSP solvers that can guarantee solution optimality) to replace LKH.

### Related Papers

[1] Yuanhang Zhang, Hesheng Wang, and Zhongqiang Ren. "Multi-Agent Combinatorial Path Finding with Heterogeneous Task Duration." IEEE Transaction on Automation System and Engineering (T-ASE, under review), 2023.
[[arXiv](https://arxiv.org/abs/2311.15330)]
[[Video](https://www.youtube.com/embed/sSX0HdzjmY4)]
[[Code](https://github.com/hang0610/public_pymcpf-d)]

## Contact
If you have any questions, feel free to contact [me](https://hang0610.github.io/).
