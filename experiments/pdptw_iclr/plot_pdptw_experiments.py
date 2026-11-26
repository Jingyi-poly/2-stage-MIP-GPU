#!/usr/bin/env python3
"""CSV-driven figures for the stochastic-demand PDPTW five-experiment pilot."""

from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


COL_GPU = "#087E8B"
COL_CPU = "#F2A541"
COL_SCIP = "#C44536"
COL_PROOF = "#6C5B7B"
COL_REF = "#6B7280"
COL_GRID = "#D8DEE8"
FORMAL_STOCHASTIC_DRAWS = 1_000_000
NOMINAL_SCENARIO_SLOTS = 1
FORMAL_TOTAL_SCENARIOS = FORMAL_STOCHASTIC_DRAWS + NOMINAL_SCENARIO_SLOTS

REQUIRED = {
    "oos.csv": {
        "run_id", "instance", "requests", "method", "train_scenarios", "train_seed",
        "test_scenarios", "test_seed", "algo_seed", "wall_time_s", "heldout_quality",
        "reference_quality", "reference_kind", "profile",
    },
    "runtime.csv": {
        "run_id", "instance", "requests", "scenarios", "repetition", "threads",
        "node_num", "route_count", "execution_order", "cpu_time_ms", "gpu_time_ms", "speedup",
        "max_abs_diff", "benchmark_scope", "device", "profile",
    },
    "convergence.csv": {
        "run_id", "instance", "requests", "train_scenarios", "evaluation_scenarios",
        "train_seed", "evaluation_seed", "algo_seed", "method", "nominal_budget_s",
        "wall_time_s", "generations", "heldout_objective", "distance", "tw_violation",
        "oos_mean_capacity_violation", "capacity_scale", "demand_spread", "extra_routes",
        "from_scratch", "profile",
    },
    "scip_compare.csv": {
        "run_id", "instance", "requests", "scenarios", "algo_seed", "method",
        "wall_time_s", "incumbent_quality", "dual_bound", "proof_gap_fraction",
        "reference_quality", "reference_kind", "capacity_scale", "demand_spread",
        "threads", "time_limit_s", "terminal_status", "profile",
    },
    "gpu_memory.csv": {
        "run_id", "instance", "requests", "scenarios", "parallelism", "mode", "measured_gib",
        "theoretical_gib", "device_capacity_gib", "device_name", "memory_kind", "status", "profile",
    },
}

EXPERIMENT_FILE = {
    "oos": "oos.csv",
    "runtime": "runtime.csv",
    "convergence": "convergence.csv",
    "scip": "scip_compare.csv",
    "memory": "gpu_memory.csv",
}


def style() -> None:
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 10.5,
            "axes.titlesize": 12,
            "axes.labelsize": 10.5,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "axes.grid": True,
            "axes.axisbelow": True,
            "grid.color": COL_GRID,
            "grid.linewidth": 0.7,
            "grid.alpha": 0.75,
            "legend.frameon": False,
            "figure.dpi": 130,
            "savefig.bbox": "tight",
        }
    )


def read_csv(path: Path, strict: bool) -> list[dict[str, str]]:
    if not path.exists():
        if strict:
            raise FileNotFoundError(path)
        return []
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        fields = set(reader.fieldnames or [])
        missing = REQUIRED[path.name] - fields
        if missing:
            raise ValueError(f"{path.name}: missing columns {sorted(missing)}")
        rows = list(reader)
    if strict and not rows:
        raise ValueError(f"{path.name}: no rows")
    print(f"[OK] {path.name}: {len(rows)} rows")
    return rows


def f(row: dict[str, str], key: str) -> float:
    value = row.get(key, "")
    return float(value) if value not in ("", "nan", "NA", "None") else math.nan


def strict_number(row: dict[str, str], key: str, dataset: str, row_number: int,
                  *, positive: bool = False, nonnegative: bool = False) -> float:
    try:
        value = float(row.get(key, ""))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{dataset} row {row_number}: {key} is not numeric") from exc
    if not math.isfinite(value):
        raise ValueError(f"{dataset} row {row_number}: {key} must be finite")
    if positive and value <= 0:
        raise ValueError(f"{dataset} row {row_number}: {key} must be positive")
    if nonnegative and value < 0:
        raise ValueError(f"{dataset} row {row_number}: {key} must be nonnegative")
    return value


def strict_integer(row: dict[str, str], key: str, dataset: str, row_number: int,
                   *, positive: bool = False, nonnegative: bool = False) -> int:
    value = strict_number(
        row, key, dataset, row_number, positive=positive, nonnegative=nonnegative
    )
    if not value.is_integer():
        raise ValueError(f"{dataset} row {row_number}: {key} must be an integer")
    return int(value)


def require_one_value(rows: list[dict[str, str]], key: str, dataset: str) -> str:
    values = {row.get(key, "") for row in rows}
    if "" in values or len(values) != 1:
        raise ValueError(f"{dataset}: expected one nonempty {key}, got {sorted(values)}")
    return next(iter(values))


def validate_strict(name: str, rows: list[dict[str, str]], memory_mode: str) -> None:
    if name == "oos.csv":
        require_one_value(rows, "instance", name)
        method = require_one_value(rows, "method", name)
        if method != "GPU memetic":
            raise ValueError(f"{name}: expected method 'GPU memetic', got {method!r}")
        require_one_value(rows, "requests", name)
        test_scenario_count = int(require_one_value(rows, "test_scenarios", name))
        if test_scenario_count != FORMAL_TOTAL_SCENARIOS:
            raise ValueError(
                f"{name}: formal OOS evaluation requires exactly "
                f"{FORMAL_TOTAL_SCENARIOS:,} total slots (one nominal plus "
                f"{FORMAL_STOCHASTIC_DRAWS:,} stochastic draws); got {test_scenario_count:,}"
            )
        require_one_value(rows, "test_seed", name)
        require_one_value(rows, "train_seed", name)
        require_one_value(rows, "profile", name)
        require_one_value(rows, "reference_kind", name)
        require_one_value(rows, "capacity_scale", name)
        require_one_value(rows, "demand_spread", name)
        require_one_value(rows, "extra_routes", name)
        seeds_by_training_size: dict[int, set[tuple[int, int]]] = defaultdict(set)
        seen_design_points: set[tuple[int, int, int]] = set()
        for index, row in enumerate(rows, 2):
            train_scenarios = strict_integer(row, "train_scenarios", name, index, positive=True)
            train_seed = strict_integer(row, "train_seed", name, index, nonnegative=True)
            algo_seed = strict_integer(row, "algo_seed", name, index, nonnegative=True)
            design_point = (train_scenarios, train_seed, algo_seed)
            if design_point in seen_design_points:
                raise ValueError(f"{name} row {index}: duplicate OOS design point {design_point}")
            seen_design_points.add(design_point)
            seeds_by_training_size[train_scenarios].add((train_seed, algo_seed))
            test_scenarios = strict_integer(row, "test_scenarios", name, index, positive=True)
            if test_scenarios != FORMAL_TOTAL_SCENARIOS:
                raise ValueError(
                    f"{name} row {index}: test_scenarios must equal "
                    f"{FORMAL_TOTAL_SCENARIOS:,}"
                )
            strict_number(row, "wall_time_s", name, index, nonnegative=True)
            strict_number(row, "heldout_quality", name, index, positive=True)
            strict_number(row, "reference_quality", name, index, positive=True)
        if len(seeds_by_training_size) < 4:
            raise ValueError(
                f"{name}: need at least 4 distinct training-scenario sizes; "
                f"got {len(seeds_by_training_size)}"
            )
        seed_designs = list(seeds_by_training_size.values())
        if any(seed_set != seed_designs[0] for seed_set in seed_designs[1:]):
            raise ValueError(
                f"{name}: every training-scenario size must use the same "
                "training/algo seed pairs"
            )
        seed_count = len(seed_designs[0])
        if seed_count != 1 and seed_count < 3:
            raise ValueError(
                f"{name}: use one fixed pre-declared algo seed at every size, "
                "or at least 3 distinct seeds at every size; got "
                f"{dict(sorted((n, len(seeds)) for n, seeds in seeds_by_training_size.items()))}"
            )
    elif name == "runtime.csv":
        scenario_count = int(require_one_value(rows, "scenarios", name))
        if scenario_count != FORMAL_TOTAL_SCENARIOS:
            raise ValueError(
                f"{name}: formal paired runtime requires exactly "
                f"{FORMAL_TOTAL_SCENARIOS:,} total slots (one nominal plus "
                f"{FORMAL_STOCHASTIC_DRAWS:,} stochastic draws); got {scenario_count:,}"
            )
        require_one_value(rows, "threads", name)
        require_one_value(rows, "device", name)
        require_one_value(rows, "profile", name)
        scope = require_one_value(rows, "benchmark_scope", name)
        if scope != "paired_lean_route_violation_same_demand_matrix_and_route_batch_after_warmup":
            raise ValueError(f"{name}: unexpected benchmark_scope={scope!r}")
        repetitions_by_instance: dict[str, set[int]] = defaultdict(set)
        orders_by_instance: dict[str, dict[str, int]] = defaultdict(
            lambda: {"CPU-first": 0, "GPU-first": 0}
        )
        for index, row in enumerate(rows, 2):
            requests = strict_integer(row, "requests", name, index, positive=True)
            strict_integer(row, "scenarios", name, index, positive=True)
            threads = strict_integer(row, "threads", name, index, positive=True)
            if threads != 1:
                raise ValueError(f"{name} row {index}: CPU lean evaluator must use one host thread")
            node_num = strict_integer(row, "node_num", name, index, positive=True)
            if node_num != 2 * requests + 1:
                raise ValueError(
                    f"{name} row {index}: node_num={node_num} does not match "
                    f"2*requests+1={2 * requests + 1}"
                )
            strict_integer(row, "route_count", name, index, positive=True)
            repetition = strict_integer(row, "repetition", name, index, nonnegative=True)
            repetitions_by_instance[row["instance"]].add(repetition)
            execution_order = row.get("execution_order", "")
            if execution_order not in {"CPU-first", "GPU-first"}:
                raise ValueError(
                    f"{name} row {index}: execution_order must be CPU-first or GPU-first"
                )
            expected_order = "CPU-first" if repetition % 2 == 0 else "GPU-first"
            if execution_order != expected_order:
                raise ValueError(
                    f"{name} row {index}: repetition {repetition} must be {expected_order}"
                )
            orders_by_instance[row["instance"]][execution_order] += 1
            cpu_time = strict_number(row, "cpu_time_ms", name, index, positive=True)
            gpu_time = strict_number(row, "gpu_time_ms", name, index, positive=True)
            speedup = strict_number(row, "speedup", name, index, positive=True)
            if not math.isclose(speedup, cpu_time / gpu_time, rel_tol=1e-6, abs_tol=1e-9):
                raise ValueError(f"{name} row {index}: speedup does not equal cpu_time/gpu_time")
            max_abs_diff = strict_number(
                row, "max_abs_diff", name, index, nonnegative=True
            )
            if max_abs_diff > 1e-6:
                raise ValueError(
                    f"{name} row {index}: CPU/GPU max_abs_diff={max_abs_diff:g} exceeds 1e-6"
                )
        if len(repetitions_by_instance) < 10:
            raise ValueError(
                f"{name}: need at least 10 distinct instances; got {len(repetitions_by_instance)}"
            )
        undersampled = {
            instance: sorted(repetitions)
            for instance, repetitions in repetitions_by_instance.items()
            if repetitions != {0, 1, 2, 3}
        }
        if undersampled:
            raise ValueError(
                f"{name}: each instance needs exactly repetitions 0,1,2,3; got {undersampled}"
            )
        unbalanced = {
            instance: counts
            for instance, counts in orders_by_instance.items()
            if counts != {"CPU-first": 2, "GPU-first": 2}
        }
        if unbalanced:
            raise ValueError(
                f"{name}: each instance needs two runs in each execution order; got {unbalanced}"
            )
    elif name == "convergence.csv":
        for key in (
            "instance", "requests", "train_scenarios", "evaluation_scenarios",
            "train_seed", "evaluation_seed", "capacity_scale", "demand_spread",
            "extra_routes", "from_scratch", "profile",
        ):
            require_one_value(rows, key, name)
        evaluation_count = int(require_one_value(rows, "evaluation_scenarios", name))
        if evaluation_count != FORMAL_TOTAL_SCENARIOS:
            raise ValueError(
                f"{name}: formal convergence evaluation requires exactly "
                f"{FORMAL_TOTAL_SCENARIOS:,} total slots (one nominal plus "
                f"{FORMAL_STOCHASTIC_DRAWS:,} stochastic draws); got {evaluation_count:,}"
            )
        if require_one_value(rows, "from_scratch", name) not in {"1", "true", "True"}:
            raise ValueError(f"{name}: formal budget runs must start from scratch")
        strict_integer(rows[0], "train_scenarios", name, 2, positive=True)
        strict_integer(rows[0], "train_seed", name, 2, nonnegative=True)
        strict_integer(rows[0], "evaluation_seed", name, 2, nonnegative=True)
        strict_integer(rows[0], "extra_routes", name, 2, nonnegative=True)
        strict_number(rows[0], "capacity_scale", name, 2, positive=True)
        strict_number(rows[0], "demand_spread", name, 2, positive=True)
        seeds_by_device: dict[str, set[int]] = {"CPU": set(), "GPU": set()}
        budgets_by_run: dict[str, set[float]] = defaultdict(set)
        seen_design_points: set[tuple[str, int, float]] = set()
        for index, row in enumerate(rows, 2):
            strict_integer(row, "train_scenarios", name, index, positive=True)
            eval_count = strict_integer(
                row, "evaluation_scenarios", name, index, positive=True
            )
            if eval_count != FORMAL_TOTAL_SCENARIOS:
                raise ValueError(
                    f"{name} row {index}: evaluation_scenarios must equal "
                    f"{FORMAL_TOTAL_SCENARIOS:,}"
                )
            algo_seed = strict_integer(row, "algo_seed", name, index, nonnegative=True)
            budget = strict_number(row, "nominal_budget_s", name, index, positive=True)
            wall_time = strict_number(row, "wall_time_s", name, index, positive=True)
            if wall_time < 0.95 * budget:
                raise ValueError(
                    f"{name} row {index}: wall_time_s={wall_time:g} is below its "
                    f"nominal budget {budget:g} by more than 5%"
                )
            strict_integer(row, "generations", name, index, nonnegative=True)
            objective = strict_number(row, "heldout_objective", name, index, positive=True)
            distance = strict_number(row, "distance", name, index, positive=True)
            tw_violation = strict_number(row, "tw_violation", name, index, nonnegative=True)
            if abs(tw_violation) > 1e-9:
                raise ValueError(
                    f"{name} row {index}: hard-time-window run has "
                    f"tw_violation={tw_violation:g}"
                )
            cap_violation = strict_number(
                row, "oos_mean_capacity_violation", name, index, nonnegative=True
            )
            expected_objective = distance + 20_000.0 * tw_violation + 20_000.0 * cap_violation
            if not math.isclose(
                objective, expected_objective, rel_tol=1e-9, abs_tol=1e-6
            ):
                raise ValueError(
                    f"{name} row {index}: heldout_objective does not match the shared "
                    "distance + TW/capacity penalty definition"
                )
            method_upper = row.get("method", "").upper()
            matching_devices = [device for device in seeds_by_device if device in method_upper]
            if len(matching_devices) != 1:
                raise ValueError(
                    f"{name} row {index}: method must identify exactly one of CPU/GPU"
                )
            device = matching_devices[0]
            seeds_by_device[device].add(algo_seed)
            run_id = row.get("run_id", "").strip()
            if not run_id:
                raise ValueError(f"{name} row {index}: run_id must be nonempty")
            design_point = (run_id, algo_seed, budget)
            if design_point in seen_design_points:
                raise ValueError(
                    f"{name} row {index}: duplicate convergence design point {design_point}"
                )
            seen_design_points.add(design_point)
            budgets_by_run[run_id].add(budget)
        missing_seeds = {
            device: len(seeds) for device, seeds in seeds_by_device.items() if len(seeds) < 3
        }
        if missing_seeds:
            raise ValueError(
                f"{name}: CPU and GPU each need at least 3 distinct algo seeds; got {missing_seeds}"
            )
        budget_designs = list(budgets_by_run.values())
        if not budget_designs or any(design != budget_designs[0] for design in budget_designs[1:]):
            raise ValueError(f"{name}: every method/seed run must use the same budget design")
        budgets = budget_designs[0]
        if len(budgets) < 5 or max(budgets) < 240:
            raise ValueError(
                f"{name}: need at least 5 predeclared budgets reaching 240 s; got {sorted(budgets)}"
            )
    elif name == "scip_compare.csv":
        require_one_value(rows, "instance", name)
        require_one_value(rows, "scenarios", name)
        require_one_value(rows, "profile", name)
        require_one_value(rows, "reference_kind", name)
        methods = {row.get("method", "") for row in rows}
        gpu_methods = {method for method in methods if "GPU" in method.upper()}
        unexpected_methods = methods - {"SCIP"} - gpu_methods
        if "SCIP" not in methods or len(gpu_methods) != 1 or unexpected_methods:
            raise ValueError(f"{name}: comparison must contain both SCIP and GPU rows; got {sorted(methods)}")
        capacity_scales: list[float] = []
        demand_spreads: list[float] = []
        rows_by_run: dict[str, list[dict[str, str]]] = defaultdict(list)
        for index, row in enumerate(rows, 2):
            run_id = row.get("run_id", "").strip()
            if not run_id:
                raise ValueError(f"{name} row {index}: run_id must be nonempty")
            rows_by_run[run_id].append(row)
            strict_integer(row, "scenarios", name, index, positive=True)
            strict_integer(row, "threads", name, index, positive=True)
            strict_number(row, "wall_time_s", name, index, nonnegative=True)
            strict_number(row, "reference_quality", name, index, positive=True)
            time_limit = strict_number(row, "time_limit_s", name, index, positive=True)
            if time_limit < 1000:
                raise ValueError(f"{name} row {index}: time_limit_s must be at least 1000")
            capacity_scales.append(
                strict_number(row, "capacity_scale", name, index, positive=True)
            )
            demand_spreads.append(
                strict_number(row, "demand_spread", name, index, positive=True)
            )
            proof = row.get("proof_gap_fraction", "")
            if proof not in ("", "nan", "NA", "None"):
                strict_number(row, "proof_gap_fraction", name, index, nonnegative=True)
        run_ids_by_method: dict[str, set[str]] = defaultdict(set)
        for run_id, run_rows in rows_by_run.items():
            method = require_one_value(run_rows, "method", f"{name} run {run_id}")
            require_one_value(run_rows, "terminal_status", f"{name} run {run_id}")
            limits = [f(row, "time_limit_s") for row in run_rows]
            limit = limits[0]
            if any(not math.isclose(value, limit, rel_tol=0.0, abs_tol=1e-12)
                   for value in limits[1:]):
                raise ValueError(f"{name} run {run_id}: inconsistent time_limit_s")
            terminal_row = max(run_rows, key=lambda row: f(row, "wall_time_s"))
            max_wall = f(terminal_row, "wall_time_s")
            final_proof = f(terminal_row, "proof_gap_fraction")
            scip_proven_early = (
                method == "SCIP"
                and terminal_row.get("reference_kind") == "scip_proven_optimum"
                and math.isfinite(f(terminal_row, "incumbent_quality"))
                and math.isfinite(final_proof)
                and final_proof <= 1e-9
            )
            if max_wall < 0.95 * limit and not scip_proven_early:
                raise ValueError(
                    f"{name} run {run_id}: trace stops at {max_wall:.3f}s, "
                    f"before 95% of the declared {limit:.3f}s budget"
                )
            run_ids_by_method[method].add(run_id)
        duplicate_runs = {
            method: sorted(run_ids)
            for method, run_ids in run_ids_by_method.items()
            if len(run_ids) != 1
        }
        if duplicate_runs:
            raise ValueError(
                f"{name}: formal anytime figure expects one trace per method; "
                f"got {duplicate_runs}"
            )
        for key, values in (
            ("capacity_scale", capacity_scales),
            ("demand_spread", demand_spreads),
        ):
            first = values[0]
            if any(not math.isclose(value, first, rel_tol=0.0, abs_tol=1e-12) for value in values[1:]):
                raise ValueError(f"{name}: {key} must be consistent across SCIP and GPU rows")
        if not any(math.isfinite(f(row, "incumbent_quality")) for row in rows):
            raise ValueError(f"{name}: no finite incumbent")
        if not any(f(row, "wall_time_s") > 0 for row in rows):
            raise ValueError(f"{name}: no positive wall-clock sample")
    elif name == "gpu_memory.csv":
        if require_one_value(rows, "mode", name) != memory_mode:
            raise ValueError(f"{name}: selected rows do not match mode={memory_mode}")
        for key in ("run_id", "instance", "profile", "device_name", "memory_kind"):
            require_one_value(rows, key, name)
        cells: set[tuple[int, int]] = set()
        scenarios: set[int] = set()
        parallelism: set[int] = set()
        for index, row in enumerate(rows, 2):
            scenario = strict_integer(row, "scenarios", name, index, positive=True)
            workers = strict_integer(row, "parallelism", name, index, positive=True)
            cell = (workers, scenario)
            if cell in cells:
                raise ValueError(f"{name}: duplicate memory cell parallelism={workers}, scenarios={scenario}")
            cells.add(cell)
            scenarios.add(scenario)
            parallelism.add(workers)
            strict_number(row, "theoretical_gib", name, index, positive=True)
            strict_number(row, "device_capacity_gib", name, index, positive=True)
            if row.get("status") != "ok":
                raise ValueError(
                    f"{name} row {index}: formal memory heatmap requires a successful "
                    f"measurement, got status={row.get('status')!r}"
                )
            strict_number(row, "measured_gib", name, index, positive=True)
        expected = {(workers, scenario) for workers in parallelism for scenario in scenarios}
        if cells != expected:
            missing = sorted(expected - cells)
            raise ValueError(f"{name}: incomplete scenario/parallelism grid; missing {missing}")
        if len(scenarios) < 5:
            raise ValueError(f"{name}: need at least 5 distinct scenario levels; got {len(scenarios)}")
        if len(parallelism) < 4:
            raise ValueError(f"{name}: need at least 4 distinct parallelism levels; got {len(parallelism)}")
        required_total = FORMAL_STOCHASTIC_DRAWS + NOMINAL_SCENARIO_SLOTS
        if required_total not in scenarios:
            raise ValueError(
                f"{name}: memory grid must contain {required_total:,} allocated slots "
                f"(one nominal plus {FORMAL_STOCHASTIC_DRAWS:,} stochastic draws); "
                f"got {sorted(scenarios)}"
            )


def q(values: list[float]) -> tuple[float, float, float]:
    arr = np.asarray([v for v in values if np.isfinite(v)], dtype=float)
    if not len(arr):
        return math.nan, math.nan, math.nan
    return tuple(float(x) for x in np.quantile(arr, [0.25, 0.5, 0.75]))


def column_quantiles(samples: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Finite-only column quantiles without all-NaN warnings before every run starts."""
    stats = np.asarray([q(samples[:, index].tolist()) for index in range(samples.shape[1])])
    return stats[:, 0], stats[:, 1], stats[:, 2]


def save(fig: plt.Figure, out: Path, stem: str) -> None:
    out.mkdir(parents=True, exist_ok=True)
    fig.savefig(out / f"{stem}.png", dpi=230)
    fig.savefig(out / f"{stem}.pdf")
    plt.close(fig)
    print(f"[WROTE] {stem}.png/.pdf")


def fig_oos(rows: list[dict[str, str]], out: Path) -> None:
    grouped: dict[int, list[float]] = defaultdict(list)
    for row in rows:
        val = f(row, "heldout_quality")
        if np.isfinite(val):
            grouped[int(row["train_scenarios"])].append(val)
    xs = sorted(grouped)
    med = np.asarray([float(np.median(grouped[x])) for x in xs])
    fig, ax = plt.subplots(figsize=(7.2, 4.8))
    ax.plot(xs, med, "o-", color=COL_GPU, lw=2.4, ms=6.5, label="GPU")
    for x, value in zip(xs, med):
        ax.annotate(
            f"{value:,.1f}",
            xy=(x, value),
            xytext=(0, 8),
            textcoords="offset points",
            ha="center",
            va="bottom",
            color=COL_GPU,
            fontsize=9,
        )
    ax.set_xscale("log")
    ax.set_xticks(xs)
    ax.set_xticklabels([f"$10^{{{int(round(math.log10(x)))}}}$" if x >= 10 else str(x) for x in xs])
    ax.set_xlabel("Train scenarios")
    ax.set_ylabel("Objective")
    ax.margins(y=0.18)
    test_n = max(int(r["test_scenarios"]) for r in rows) - NOMINAL_SCENARIO_SLOTS
    instance = rows[0]["instance"]
    ax.set_title(f"OOS objective - {instance}")
    ax.text(0.99, 0.03, f"Test: {test_n / 1_000_000:g}M scenarios", transform=ax.transAxes,
            ha="right", va="bottom", color=COL_REF, fontsize=9)
    ax.legend(loc="lower left")
    save(fig, out, "01_oos_quality")


def fig_runtime(rows: list[dict[str, str]], out: Path) -> None:
    order: list[str] = []
    grouped: dict[str, list[tuple[float, float]]] = defaultdict(list)
    for row in rows:
        name = row["instance"]
        if name not in grouped:
            order.append(name)
        grouped[name].append((f(row, "cpu_time_ms"), f(row, "gpu_time_ms")))
    cpu = np.asarray([np.median([a for a, _ in grouped[n]]) for n in order])
    gpu = np.asarray([np.median([b for _, b in grouped[n]]) for n in order])
    speed = cpu / gpu
    x = np.arange(len(order))
    width = 0.37

    fig, ax = plt.subplots(figsize=(10.6, 5.0))
    b1 = ax.bar(x - width / 2, cpu, width, color=COL_CPU, label="CPU")
    b2 = ax.bar(x + width / 2, gpu, width, color=COL_GPU, label="GPU")
    if float(np.nanmax(cpu) / max(np.nanmin(gpu), 1e-12)) > 25:
        ax.set_yscale("log")
    for bar, s in zip(b2, speed):
        ax.annotate(f"{s:.1f}×", (bar.get_x() + bar.get_width() / 2, bar.get_height()),
                    xytext=(0, 5), textcoords="offset points", ha="center", va="bottom",
                    color=COL_GPU, fontsize=8.5, fontweight="bold")
    ax.set_xticks(x)
    ax.set_xticklabels(order, rotation=24, ha="right")
    ax.set_ylabel("Time (ms)")
    scenarios = max(int(r["scenarios"]) for r in rows) - NOMINAL_SCENARIO_SLOTS
    ax.set_title(f"Runtime - {scenarios / 1_000_000:g}M scenarios")
    ax.set_ylim(0.0, float(np.nanmax(cpu)) * 1.28)
    ax.legend(loc="upper left", ncol=2)
    ax.grid(axis="x", visible=False)
    fig.subplots_adjust(bottom=0.29)
    fig.text(
        0.5,
        0.025,
        "4 runs · setup/warm-up excluded",
        ha="center",
        color=COL_REF,
        fontsize=8.3,
    )
    save(fig, out, "02_runtime_cpu_gpu")

    summary_path = out / "02_runtime_summary.csv"
    with summary_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["instance", "cpu_median_ms", "gpu_median_ms", "paired_speedup"])
        for name, c, g, s in zip(order, cpu, gpu, speed):
            writer.writerow([name, f"{c:.6f}", f"{g:.6f}", f"{s:.6f}"])
        writer.writerow(["geometric_mean", "", "", f"{math.exp(float(np.mean(np.log(speed)))):.6f}"])


def step_value(times: np.ndarray, values: np.ndarray, grid: np.ndarray) -> np.ndarray:
    idx = np.searchsorted(times, grid, side="right") - 1
    out = np.full(grid.shape, np.nan, dtype=float)
    valid = idx >= 0
    out[valid] = values[idx[valid]]
    return out


def aggregate_traces(rows: list[dict[str, str]], value_key: str,
                     method: str, grid: np.ndarray) -> tuple[list[tuple[np.ndarray, np.ndarray]], np.ndarray]:
    by_run: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        if row["method"] == method:
            by_run[row["run_id"]].append(row)
    individual = []
    sampled = []
    for run_rows in by_run.values():
        run_rows.sort(key=lambda r: f(r, "wall_time_s"))
        t = np.asarray([f(r, "wall_time_s") for r in run_rows])
        vals = np.asarray([f(r, value_key) for r in run_rows])
        individual.append((t, vals))
        sampled.append(step_value(t, vals, grid))
    return individual, np.asarray(sampled)


def fig_convergence(rows: list[dict[str, str]], out: Path) -> None:
    methods = sorted(set(r["method"] for r in rows), key=lambda x: ("GPU" not in x, x))
    budgets = sorted({f(row, "nominal_budget_s") for row in rows})
    colors = {m: (COL_GPU if "GPU" in m else COL_CPU) for m in methods}

    fig, ax = plt.subplots(figsize=(7.6, 4.9))
    all_wall_times: list[float] = []
    for method in methods:
        by_seed: dict[int, dict[float, tuple[float, float]]] = defaultdict(dict)
        for row in rows:
            if row["method"] != method:
                continue
            seed = int(row["algo_seed"])
            budget = f(row, "nominal_budget_s")
            by_seed[seed][budget] = (
                f(row, "wall_time_s"),
                f(row, "heldout_objective"),
            )
        wall_samples = np.asarray(
            [[by_seed[seed][budget][0] for budget in budgets] for seed in sorted(by_seed)],
            dtype=float,
        )
        objective_samples = np.asarray(
            [[by_seed[seed][budget][1] for budget in budgets] for seed in sorted(by_seed)],
            dtype=float,
        )
        for wall_times, values in zip(wall_samples, objective_samples):
            # Raw held-out objectives from separate fixed-seed, from-scratch
            # solves.  Generation-boundary jitter can make a smaller nominal
            # budget take slightly longer than the next budget, so connect
            # points in measured-time order.  Do not force objectives to be
            # monotone.
            time_order = np.argsort(wall_times)
            ax.plot(
                wall_times[time_order], values[time_order], "o-", color=colors[method],
                alpha=0.16, lw=0.8, ms=2.8,
            )
            all_wall_times.extend(wall_times.tolist())
        median_wall = np.median(wall_samples, axis=0)
        median_objective = np.median(objective_samples, axis=0)
        median_time_order = np.argsort(median_wall)
        ax.plot(
            median_wall[median_time_order], median_objective[median_time_order],
            "o-", color=colors[method], lw=2.3, ms=5.5,
            label=("GPU" if "GPU" in method else "CPU"),
        )
    ax.set_xscale("log")
    ax.set_xlim(min(all_wall_times) * 0.94, max(all_wall_times) * 1.06)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Objective")
    train_total = int(rows[0]["train_scenarios"])
    stochastic_draws = int(rows[0]["evaluation_scenarios"]) - NOMINAL_SCENARIO_SLOTS
    ax.set_title(
        f"Convergence - {rows[0]['instance']}\n"
        f"{train_total / 1_000:g}k train · {stochastic_draws / 1_000_000:g}M test"
    )
    ax.legend(loc="upper right")
    fig.subplots_adjust(bottom=0.21)
    fig.text(
        0.5,
        0.025,
        "Wall time · 3 seeds · raw objective",
        ha="center",
        color=COL_REF,
        fontsize=8.4,
    )
    save(fig, out, "03_convergence_anytime")


def fig_scip(rows: list[dict[str, str]], out: Path) -> None:
    limit = max(f(r, "time_limit_s") for r in rows)
    positive = [f(r, "wall_time_s") for r in rows if f(r, "wall_time_s") > 0]
    xmin = max(0.1, min(positive))
    grid = np.geomspace(xmin, limit, 320)

    fig, ax = plt.subplots(figsize=(8.2, 5.2))
    gpu_methods = sorted({r["method"] for r in rows if "GPU" in r["method"]})
    if gpu_methods:
        method = gpu_methods[0]
        indiv, samples = aggregate_traces(rows, "incumbent_quality", method, grid)
        if len(indiv) == 1:
            t, objective = indiv[0]
            if len(t) and t[-1] < limit:
                t = np.append(t, limit)
                objective = np.append(objective, objective[-1])
            ax.step(t, objective, where="post", color=COL_GPU, lw=2.35,
                    label="GPU")
        else:
            for t, objective in indiv:
                ax.step(t, objective, where="post", color=COL_GPU, alpha=0.17, lw=0.85)
            for run_index, (t, objective) in enumerate(indiv, start=1):
                ax.step(
                    t,
                    objective,
                    where="post",
                    color=COL_GPU,
                    alpha=0.55,
                    lw=1.5,
                    label=("GPU" if run_index == 1 else None),
                )

    scip_rows = [r for r in rows if r["method"] == "SCIP"]
    scip_rows.sort(key=lambda r: f(r, "wall_time_s"))
    if scip_rows:
        t = np.asarray([f(r, "wall_time_s") for r in scip_rows])
        raw_inc = np.asarray([f(r, "incumbent_quality") for r in scip_rows])
        inc = raw_inc
        incumbent_valid = np.isfinite(inc)
        inc_t = t[incumbent_valid]
        objective = inc[incumbent_valid]
        if len(inc_t) and inc_t[-1] < limit:
            inc_t = np.append(inc_t, limit)
            objective = np.append(objective, objective[-1])
        ax.step(inc_t, objective, where="post", color=COL_SCIP, lw=2.0,
                label="SCIP")
        raw_bound = np.asarray([f(r, "dual_bound") for r in scip_rows])
        valid = np.isfinite(raw_bound) & (raw_bound > 0)
        bound_t, bound = t[valid], raw_bound[valid]
        if len(bound_t) and bound_t[-1] < limit:
            bound_t = np.append(bound_t, limit)
            bound = np.append(bound, bound[-1])
        ax.step(bound_t, bound, where="post", color=COL_PROOF, lw=2.0, ls="--",
                label="SCIP bound")
        if len(bound_t):
            ax.annotate(
                f"{bound[-1]:,.1f}",
                xy=(bound_t[-1], bound[-1]),
                xytext=(-6, 7),
                textcoords="offset points",
                ha="right",
                va="bottom",
                color=COL_PROOF,
            )
    reference = f(rows[0], "reference_quality")
    ax.axhline(reference, color=COL_REF, ls=":", lw=1.1,
               label="Best found")
    ax.set_ylabel("Objective")
    requests = int(f(rows[0], "requests"))
    scenarios = int(f(rows[0], "scenarios"))
    reference_kind = rows[0].get("reference_kind", "")
    reference_note = (
        "proven optimum" if reference_kind == "scip_proven_optimum"
        else "unproven ref."
    )
    ax.set_title(
        f"GPU vs SCIP - PDPTW R={requests}\n"
        f"S={scenarios} · {int(limit)} s · {reference_note}",
        loc="left",
    )
    ax.legend(
        loc="upper center", bbox_to_anchor=(0.5, -0.16), ncol=2,
        fontsize=9, frameon=False,
    )
    ax.set_xlabel("Time (s)")
    ax.set_xscale("log")
    ax.set_xlim(xmin, limit)
    fig.subplots_adjust(bottom=0.25)
    save(fig, out, "04_scip_compare")


def fig_memory(rows: list[dict[str, str]], out: Path) -> None:
    mode = rows[0]["mode"]
    scenarios = sorted({int(r["scenarios"]) for r in rows})
    parallel = sorted({int(r["parallelism"]) for r in rows})
    lookup = {(int(r["parallelism"]), int(r["scenarios"])): r for r in rows}
    mat = np.full((len(parallel), len(scenarios)), np.nan)
    status = [["" for _ in scenarios] for _ in parallel]
    for i, p in enumerate(parallel):
        for j, s in enumerate(scenarios):
            row = lookup.get((p, s))
            if row:
                mat[i, j] = f(row, "measured_gib")
                status[i][j] = row["status"]

    fig, ax = plt.subplots(figsize=(8.4, 4.9))
    masked = np.ma.masked_invalid(mat)
    image = ax.imshow(masked, aspect="auto", cmap="YlGnBu", interpolation="nearest")
    for i in range(len(parallel)):
        for j in range(len(scenarios)):
            if np.isfinite(mat[i, j]):
                text_color = "white" if image.norm(mat[i, j]) > 0.45 else "#1F2937"
                label = "<0.01" if mat[i, j] < 0.005 else f"{mat[i, j]:.2f}"
                ax.text(j, i, label, ha="center", va="center", color=text_color, fontsize=9)
            else:
                ax.text(j, i, status[i][j] or "n/a", ha="center", va="center", color="#7F1D1D", fontsize=8)
    ax.set_xticks(range(len(scenarios)))
    stochastic_draws = [s - NOMINAL_SCENARIO_SLOTS for s in scenarios]
    ax.set_xticklabels(
        [
            f"$10^{{{int(round(math.log10(draws)))}}}$" if draws >= 10 else str(draws)
            for draws in stochastic_draws
        ]
    )
    ax.set_yticks(range(len(parallel)))
    ax.set_yticklabels(parallel)
    ax.set_xlabel("Scenarios")
    ax.set_ylabel("Parallel jobs")
    ax.set_title("GPU memory")
    ax.grid(False)
    cbar = fig.colorbar(image, ax=ax, pad=0.02)
    cbar.set_label("Memory (GiB)")
    device = rows[0]["device_name"]
    capacity = f(rows[0], "device_capacity_gib")
    device_short = device.replace("NVIDIA GeForce ", "")
    ax.text(0.99, -0.18, f"{device_short} · {capacity:.0f} GiB", transform=ax.transAxes,
            ha="right", va="top", color=COL_REF, fontsize=9)
    stem = "05_gpu_memory" if mode == "full" else "05_gpu_memory_lean"
    save(fig, out, stem)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Validate normalized CSV files from run_pdptw_pilot.py and render the five "
            "stochastic-PDPTW figures."
        )
    )
    results_dir = Path(__file__).resolve().parents[2] / "results/pdptw"
    parser.add_argument("--data-dir", type=Path, default=results_dir / "data",
                        help="directory containing the runner's normalized CSV files")
    parser.add_argument("--output-dir", type=Path, default=results_dir / "figures",
                        help="directory for PNG/PDF figures")
    parser.add_argument("--only", choices=["all", *EXPERIMENT_FILE], default="all",
                        help="validate/plot one experiment instead of all five")
    parser.add_argument("--memory-mode", choices=["full", "lean"], default="full",
                        help="memory rows to select; the formal heatmap defaults to full")
    parser.add_argument("--strict", action="store_true",
                        help="require complete, internally consistent, publication-ready inputs")
    parser.add_argument("--validate-only", action="store_true",
                        help="validate selected inputs without plotting; implies --strict")
    args = parser.parse_args()

    style()
    selected = list(REQUIRED) if args.only == "all" else [EXPERIMENT_FILE[args.only]]
    strict = args.strict or args.validate_only
    data: dict[str, list[dict[str, str]]] = {}
    for name in selected:
        rows = read_csv(args.data_dir / name, strict)
        if name == "gpu_memory.csv":
            rows = [row for row in rows if row.get("mode") == args.memory_mode]
            if strict and not rows:
                raise ValueError(f"{name}: no rows for mode={args.memory_mode}")
        if strict:
            validate_strict(name, rows, args.memory_mode)
        data[name] = rows
    if args.validate_only:
        print(f"[VALID] {len(selected)} normalized CSV input(s)")
        return
    plotters = {
        "oos.csv": fig_oos,
        "runtime.csv": fig_runtime,
        "convergence.csv": fig_convergence,
        "scip_compare.csv": fig_scip,
        "gpu_memory.csv": fig_memory,
    }
    for name in selected:
        if data[name]:
            plotters[name](data[name], args.output_dir)


if __name__ == "__main__":
    main()
