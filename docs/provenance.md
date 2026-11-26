# PDPTW source provenance

This branch imports the Amdahl stochastic PDPTW solver from a standalone source
tree. The dated source/runtime archive records the original transfer and has
the following SHA-256 digest.

~~~text
amdahl-stochastic-source-runtime-20261013.tar.gz
4f4a1c0b084d4be7c9648312bffc0fa98761f6e16f582479cc8db73c307a8dea
~~~

The transferred Amdahl directory had no Git metadata. This import creates a
new history and preserves the source author and license notices found in the
files. The maintained tree uses the current Amdahl source for the CUDA
evaluator, demand reader, random-number helpers, memetic search configuration,
experiment programs, and build definitions. The SCIP and GPU memory
benchmarks, standalone experiment runner, and plotter come from the same
working source tree.

Benchmark inputs and best-known routes are provided as recorded in the source
archive. They retain the original case-sensitive filenames and input format.
See the [Li & Lim benchmark](https://www.sintef.no/projectweb/top/pdptw/li-lim-benchmark/)
for benchmark attribution and source data.

The manuscript uses several distinct PDPTW comparisons. The fixed-candidate
scenario-size panel reranks a saved plan pool. The timing study uses a
screened and deduplicated evaluator overlay. The timed search experiments
use saved CPU and CUDA trajectories, and the small exact comparison uses a
SCIP bound trace. The pilot runner in this branch generates fresh experiments
with its configured inputs and seeds. Exact regeneration of archived paper
panels also requires their pinned experiment overlays and records.
