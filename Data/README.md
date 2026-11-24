# Benchmark inputs

The 936 tracked `.dat` files retain their paths and bytes from the [ICLR 2026 DSIRP implementation](https://github.com/Jingyi-poly/2-stage-MIP-GPU/tree/f3931bd798a2e82048f3744ea48c66405437d059/Data). See the [paper](https://arxiv.org/abs/2602.05179) for the benchmark description and attribution.

| Input | Purpose |
|---|---|
| [`Small/Istanze0105h3/0.dat`](Small/Istanze0105h3/0.dat) | Local CPU and gRPC smoke tests |
| [`Big/Istanze001005/abs5n100_4.dat`](Big/Istanze001005/abs5n100_4.dat) | Input inherited by the later local 100-customer, six-day experiment |

The pinned snapshot also contains original solution, best-known-solution, and timing files. The maintained solver writes new solution and timing files in `results/` at the working directory.
