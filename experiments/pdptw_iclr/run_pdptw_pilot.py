#!/usr/bin/env python3
"""Run the five CSV-producing stochastic-PDPTW pilot experiments under WSL."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import re
import shlex
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Iterable


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
FORMAL_STOCHASTIC_DRAWS = 1_000_000
NOMINAL_SCENARIO_SLOTS = 1
FORMAL_TOTAL_SCENARIOS = FORMAL_STOCHASTIC_DRAWS + NOMINAL_SCENARIO_SLOTS

OOS_FIELDS = [
    "run_id", "instance", "requests", "method", "train_scenarios", "train_seed",
    "test_scenarios", "test_seed", "algo_seed", "wall_time_s", "generations",
    "heldout_quality", "reference_quality", "reference_kind", "capacity_scale",
    "demand_spread", "extra_routes", "profile",
]
RUNTIME_FIELDS = [
    "run_id", "instance", "requests", "scenarios", "repetition", "threads",
    "node_num", "route_count", "execution_order", "cpu_time_ms", "gpu_time_ms", "speedup",
    "max_abs_diff", "benchmark_scope", "device", "profile",
]
CONV_FIELDS = [
    "run_id", "instance", "requests", "train_scenarios", "evaluation_scenarios",
    "train_seed", "evaluation_seed", "algo_seed", "method", "nominal_budget_s",
    "wall_time_s", "generations", "heldout_objective", "distance", "tw_violation",
    "oos_mean_capacity_violation", "capacity_scale", "demand_spread", "extra_routes",
    "from_scratch", "profile",
]
SCIP_FIELDS = [
    "run_id", "instance", "requests", "scenarios", "algo_seed", "method",
    "wall_time_s", "incumbent_quality", "dual_bound", "proof_gap_fraction",
    "reference_quality", "reference_kind", "capacity_scale", "demand_spread",
    "threads", "time_limit_s", "terminal_status", "profile",
]
MEMORY_FIELDS = [
    "run_id", "instance", "requests", "scenarios", "parallelism", "mode",
    "measured_gib", "theoretical_gib", "device_capacity_gib", "device_name",
    "memory_kind", "status", "profile",
]


def parse_int_list(text: str) -> list[int]:
    return [int(item) for item in text.split(",") if item.strip()]


def parse_float_list(text: str) -> list[float]:
    return [float(item) for item in text.split(",") if item.strip()]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def write_rows(path: Path, fields: list[str], rows: Iterable[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + ".tmp")
    with temp.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    temp.replace(path)


def append_run(path: Path, fields: list[str], new_rows: list[dict[str, object]], run_id: str) -> bool:
    rows = read_rows(path)
    if any(row.get("run_id") == run_id for row in rows):
        print(f"[SKIP] {run_id}")
        return False
    rows.extend(new_rows)
    write_rows(path, fields, rows)
    print(f"[DATA] {path.name}: +{len(new_rows)} row(s) for {run_id}")
    return True


def run_logged(command: list[str], env_updates: dict[str, str], raw_path: Path,
               timeout: float) -> str:
    env = os.environ.copy()
    env.update(env_updates)
    started = time.time()
    try:
        result = subprocess.run(
            command,
            cwd=str(command[0].parent) if isinstance(command[0], Path) else None,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout,
            check=False,
        )
        output = result.stdout
        returncode = result.returncode
    except subprocess.TimeoutExpired as exc:
        def decoded(value: str | bytes | None) -> str:
            if value is None:
                return ""
            return value.decode("utf-8", errors="replace") if isinstance(value, bytes) else value

        output = decoded(exc.stdout) + decoded(exc.stderr)
        returncode = 124
    raw_path.parent.mkdir(parents=True, exist_ok=True)
    header = {
        "command": [str(x) for x in command],
        "environment": env_updates,
        "started_unix": started,
        "wall_seconds": time.time() - started,
        "returncode": returncode,
    }
    raw_path.write_text("# " + json.dumps(header, sort_keys=True) + "\n" + output, encoding="utf-8")
    if returncode != 0:
        raise RuntimeError(f"command failed with rc={returncode}; see {raw_path}")
    return output


def result_records(output: str, experiment: str, kind: str | None = None) -> list[dict[str, str]]:
    records = []
    for line in output.splitlines():
        if not line.startswith("PAPER_RESULT "):
            continue
        record = {}
        for token in shlex.split(line[len("PAPER_RESULT "):]):
            if "=" in token:
                key, value = token.split("=", 1)
                record[key] = value
        if record.get("experiment") != experiment:
            continue
        if kind is not None and record.get("kind") != kind:
            continue
        records.append(record)
    return records


def request_count(instance: str) -> int:
    path = REPO / "src" / "data" / "instance" / f"{instance}.txt"
    count = 0
    for line in path.read_text(encoding="utf-8").splitlines()[1:]:
        parts = line.split()
        if len(parts) >= 9 and int(parts[8]) != 0:
            count += 1
    return count


def bks_path(instance: str) -> Path:
    """Return the exact BKS file whose route count shapes the runtime workload."""
    lower = instance.lower()
    candidates = []
    for path in (REPO / "src" / "data" / "bks").iterdir():
        name = path.name.lower()
        if not name.startswith(lower):
            continue
        if len(name) > len(lower) and name[len(lower)].isdigit():
            continue
        candidates.append(path)
    if not candidates:
        raise RuntimeError(f"no BKS found for runtime workload {instance}")
    return sorted(candidates, key=lambda item: item.name)[0]


def bks_route_count(instance: str) -> int:
    """Return the route count of the BKS whose prefix matches `instance`."""
    path = bks_path(instance)
    count = sum(
        1 for line in path.read_text(encoding="utf-8", errors="replace").splitlines()
        if re.match(r"^\s*Route\s+\d+\s*:", line, re.IGNORECASE)
    )
    if count <= 0:
        raise RuntimeError(f"BKS {path} contains no route rows")
    return count


def device_name() -> str:
    try:
        return subprocess.check_output(
            ["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"],
            text=True, stderr=subprocess.DEVNULL, timeout=10,
        ).splitlines()[0].strip()
    except Exception:
        return "unknown GPU"


def base_paper_env(args: argparse.Namespace, algo_seed: int, extra_routes: int) -> dict[str, str]:
    return {
        "PDPTW_DEMAND_SPREAD": str(args.demand_spread),
        "PAPER_EXTRA_ROUTES": str(extra_routes),
        "PAPER_OPT_CAP": "1",
        "PAPER_HARD_TW": "1",
        "PAPER_TRAIN_SEED": str(args.train_seed),
        "PAPER_OOS_SEED": str(args.test_seed),
        "PAPER_ALGO_SEED": str(algo_seed),
        "PAPER_OOS_SCENARIOS": str(args.oos_test_scenarios),
    }


def run_oos(args: argparse.Namespace) -> None:
    binary = args.bin_dir / "paper_experiments"
    out_path = args.data_dir / "oos.csv"
    max_train = max(args.oos_train_scenarios)
    existing = {row.get("run_id") for row in read_rows(out_path)}
    for seed in args.oos_seeds:
        train_tag = "-".join(str(n) for n in args.oos_train_scenarios)
        run_id = (
            f"oos-{args.oos_instance}-selected-train{train_tag}-test{args.oos_test_scenarios}-"
            f"cap{args.oos_capacity_scale:g}-budget{args.oos_budget:g}-seed{seed}"
        )
        if run_id in existing:
            print(f"[SKIP] {run_id}")
            continue
        env = base_paper_env(args, seed, args.oos_extra_routes)
        env["PAPER_TRAIN_LIST"] = ",".join(str(n) for n in args.oos_train_scenarios)
        command = [str(binary), "exp2", args.oos_instance, str(args.oos_capacity_scale), str(args.oos_budget)]
        output = run_logged(command, env, args.raw_dir / f"{run_id}.log",
                            len(args.oos_train_scenarios) * args.oos_budget + args.command_slack)
        records = result_records(output, "exp2", "solution")
        if len(records) != len(args.oos_train_scenarios):
            raise RuntimeError(f"{run_id}: incomplete train-size sweep ({len(records)} rows)")
        endpoint = [
            rec for rec in records if int(rec["train_scenarios"]) == max_train
        ]
        if len(endpoint) != 1:
            raise RuntimeError(f"{run_id}: missing unique largest-N curve endpoint")
        reference = float(endpoint[0]["oos_penalized"])
        reference_kind = (
            f"selected_run_n{max_train}_final_{args.oos_budget:g}s_"
            "endpoint_not_independent"
        )
        new_rows = []
        for rec in records:
            new_rows.append(
                {
                    "run_id": run_id,
                    "instance": args.oos_instance,
                    "requests": request_count(args.oos_instance),
                    "method": "GPU memetic",
                    "train_scenarios": rec["train_scenarios"],
                    "train_seed": rec["train_seed"],
                    "test_scenarios": args.oos_test_scenarios,
                    "test_seed": rec["oos_seed"],
                    "algo_seed": rec["algo_seed"],
                    "wall_time_s": rec["solve_wall_seconds"],
                    "generations": rec["generations"],
                    "heldout_quality": rec["oos_penalized"],
                    "reference_quality": reference,
                    "reference_kind": reference_kind,
                    "capacity_scale": args.oos_capacity_scale,
                    "demand_spread": args.demand_spread,
                    "extra_routes": args.oos_extra_routes,
                    "profile": (
                        f"rtx2080ti-wsl-pdptw-selected-instance-{args.oos_instance}"
                    ),
                }
            )
        append_run(out_path, OOS_FIELDS, new_rows, run_id)
        existing.add(run_id)


SCENARIO_BENCH_RE = re.compile(
    r"^\s*(?P<scen>\d+)\s+(?P<cpu>[0-9.eE+-]+)\s+"
    r"(?P<gpu>[0-9.eE+-]+)\s+(?P<speedup>[0-9.eE+-]+)\s+"
    r"(?P<max_abs_diff>[0-9.eE+-]+)"
)


def run_runtime(args: argparse.Namespace) -> None:
    binary = args.bin_dir / "scenario_eval_benchmark"
    out_path = args.data_dir / "runtime.csv"
    existing = {row.get("run_id") for row in read_rows(out_path)}
    gpu = device_name()
    selected_run_ids = {
        f"runtime-lean-{instance}-s{args.runtime_scenarios}-r{repetition}"
        for repetition in range(args.runtime_repetitions)
        for instance in args.runtime_instances
    }
    for repetition in range(args.runtime_repetitions):
        for instance in args.runtime_instances:
            run_id = f"runtime-lean-{instance}-s{args.runtime_scenarios}-r{repetition}"
            if run_id in existing:
                print(f"[SKIP] {run_id}")
                continue
            requests = request_count(instance)
            node_num = 2 * requests + 1
            route_count = bks_route_count(instance)
            execution_order = "CPU-first" if repetition % 2 == 0 else "GPU-first"
            command = [
                str(binary), str(node_num), str(route_count),
                str(args.runtime_scenarios), execution_order.lower()
            ]
            output = run_logged(command, {}, args.raw_dir / f"{run_id}.log", args.runtime_timeout)
            order_match = re.search(r"\bexecutionOrder=(CPU-first|GPU-first)\b", output)
            if not order_match or order_match.group(1) != execution_order:
                raise RuntimeError(
                    f"{run_id}: benchmark did not confirm requested {execution_order} order"
                )
            matches = []
            for line in output.splitlines():
                match = SCENARIO_BENCH_RE.match(line)
                if match and int(match.group("scen")) == args.runtime_scenarios:
                    matches.append(match)
            if len(matches) != 1:
                raise RuntimeError(
                    f"{run_id}: expected one lean evaluator row, got {len(matches)}"
                )
            matched = matches[0]
            cpu_ms = float(matched.group("cpu"))
            gpu_ms = float(matched.group("gpu"))
            max_abs_diff = float(matched.group("max_abs_diff"))
            if not math.isfinite(max_abs_diff) or max_abs_diff > 1e-6:
                raise RuntimeError(
                    f"{run_id}: CPU/GPU correctness error max_abs_diff={max_abs_diff:g}"
                )
            row = {
                "run_id": run_id,
                "instance": instance,
                "requests": requests,
                "scenarios": args.runtime_scenarios,
                "repetition": repetition,
                "threads": 1,
                "node_num": node_num,
                "route_count": route_count,
                "execution_order": execution_order,
                "cpu_time_ms": cpu_ms,
                "gpu_time_ms": gpu_ms,
                "speedup": cpu_ms / gpu_ms,
                "max_abs_diff": max_abs_diff,
                "benchmark_scope": (
                    "paired_lean_route_violation_same_demand_matrix_and_route_batch_after_warmup"
                ),
                "device": gpu,
                "profile": args.profile,
            }
            append_run(out_path, RUNTIME_FIELDS, [row], run_id)
            existing.add(run_id)
    # Keep the formal CSV single-design even when a new scenario count is run
    # into a directory containing an older pilot.
    current_rows = [row for row in read_rows(out_path) if row.get("run_id") in selected_run_ids]
    if len(current_rows) != len(selected_run_ids):
        raise RuntimeError("runtime benchmark is missing one or more selected runs")
    write_rows(out_path, RUNTIME_FIELDS, current_rows)


def parse_trace(path: Path, train_scenarios: int,
                feasible_only: bool = False) -> list[tuple[int, float, float]]:
    points = []
    best = math.inf
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        parts = line.split()
        if len(parts) < 5 or not parts[0].isdigit():
            continue
        generation = int(parts[0])
        distance, tw, cap_total, wall = map(float, parts[1:5])
        # SCIP enforces time windows as hard constraints, while scenario
        # overload is a soft term in the shared objective.  A comparable GPU
        # incumbent therefore needs zero TW violation; positive overload stays
        # in the objective and must not be filtered out.
        if feasible_only and abs(tw) > 1e-9:
            continue
        quality = distance + 20000.0 * tw + 20000.0 * cap_total / train_scenarios
        best = min(best, quality)
        points.append((generation, wall, best))
    if not points:
        raise RuntimeError(f"no numeric trace rows in {path}")
    points.sort(key=lambda item: item[1])
    return points


def run_one_trace(args: argparse.Namespace, instance: str, capacity_scale: float,
                  train_scenarios: int, budget: float, algo_seed: int,
                  config: str, extra_routes: int, tag: str,
                  train_seed: int | None = None,
                  demand_spread: float | None = None,
                  feasible_only: bool = False,
                  from_scratch: bool = False,
                  solver_threads: int | None = None,
                  disable_srex: bool = False) -> list[tuple[int, float, float]]:
    binary = args.bin_dir / "paper_experiments"
    env = base_paper_env(args, algo_seed, extra_routes)
    env["PAPER_CONFIGS"] = config
    if train_seed is not None:
        env["PAPER_TRAIN_SEED"] = str(train_seed)
    if demand_spread is not None:
        env["PDPTW_DEMAND_SPREAD"] = str(demand_spread)
    if from_scratch:
        env["PAPER_FROM_SCRATCH"] = "1"
    if solver_threads is not None:
        env["PAPER_SOLVER_THREADS"] = str(solver_threads)
    if disable_srex:
        env["PAPER_DISABLE_SREX"] = "1"
    command = [str(binary), "trace3", instance, str(capacity_scale), str(budget), str(train_scenarios)]
    source = Path("/tmp") / f"fig_trace_{config}.log"
    source.unlink(missing_ok=True)
    output = run_logged(command, env, args.raw_dir / f"{tag}.solver.log", budget + args.command_slack)
    records = [rec for rec in result_records(output, "trace3") if rec.get("config") == config]
    if len(records) != 1:
        raise RuntimeError(f"{tag}: expected one trace3 PAPER_RESULT for {config}, got {len(records)}")
    record = records[0]
    expected_backend = "cuda" if config.startswith("GPU") else "cpu"
    if record.get("backend") != expected_backend:
        raise RuntimeError(
            f"{tag}: {config} used backend={record.get('backend')!r}, expected {expected_backend!r}"
        )
    if int(record["train_scenarios"]) != train_scenarios:
        raise RuntimeError(f"{tag}: solver reported the wrong training-scenario count")
    if not source.exists():
        raise RuntimeError(f"{tag}: solver did not create {source}")
    destination = args.raw_dir / f"{tag}.trace.log"
    shutil.copyfile(source, destination)
    return parse_trace(destination, train_scenarios, feasible_only=feasible_only)


def run_convergence(args: argparse.Namespace) -> None:
    """Budget-to-quality curves scored on a common million-draw held-out set.

    Training directly on a million scenarios is not feasible for the current
    full-state memetic representation: every route node stores one load value
    per training scenario.  Instead, each predeclared optimization budget is a
    deterministic fixed-seed rerun on a manageable training sample.  Its frozen
    best tour is then scored by the shared CUDA evaluator on the same independent
    held-out draw set.  No cumulative-minimum transform is applied to these OOS
    objectives downstream.
    """
    out_path = args.data_dir / "convergence.csv"
    if not args.convergence_budgets or any(value <= 0 for value in args.convergence_budgets):
        raise ValueError("--convergence-budgets must contain positive values")
    if len(set(args.convergence_budgets)) != len(args.convergence_budgets):
        raise ValueError("--convergence-budgets must not contain duplicates")
    budgets = sorted(args.convergence_budgets)
    budget_tag = "-".join(f"{value:g}" for value in budgets)
    existing = {row.get("run_id") for row in read_rows(out_path)}
    configs = [("CPU-8thread", "CPU memetic"), ("GPU-8thread", "GPU memetic")]
    selected_run_ids = {
        (
            f"conv-oos-{config}-{args.convergence_instance}-"
            f"train{args.convergence_train_scenarios}-"
            f"eval{args.convergence_evaluation_scenarios}-b{budget_tag}-seed{seed}"
        )
        for config, _ in configs
        for seed in args.convergence_seeds
    }
    for config, method in configs:
        for seed in args.convergence_seeds:
            run_id = (
                f"conv-oos-{config}-{args.convergence_instance}-"
                f"train{args.convergence_train_scenarios}-"
                f"eval{args.convergence_evaluation_scenarios}-b{budget_tag}-seed{seed}"
            )
            if run_id in existing:
                print(f"[SKIP] {run_id}")
                continue
            env = base_paper_env(args, seed, args.convergence_extra_routes)
            env.update(
                {
                    "PAPER_CONFIGS": config,
                    "PAPER_OOS_SCENARIOS": str(args.convergence_evaluation_scenarios),
                    "PAPER_FROM_SCRATCH": "1",
                }
            )
            command = [
                str(args.bin_dir / "paper_experiments"),
                "exp3",
                args.convergence_instance,
                str(args.convergence_capacity_scale),
                str(max(budgets)),
                str(args.convergence_train_scenarios),
                "1",
                ",".join(f"{value:g}" for value in budgets),
            ]
            output = run_logged(
                command,
                env,
                args.raw_dir / f"{run_id}.solver.log",
                sum(budgets) + len(budgets) * args.command_slack,
            )
            records = [
                record for record in result_records(output, "exp3")
                if record.get("config") == config
            ]
            if len(records) != len(budgets):
                raise RuntimeError(
                    f"{run_id}: expected {len(budgets)} exp3 rows, got {len(records)}"
                )
            records_by_budget = {float(record["budget_seconds"]): record for record in records}
            if set(records_by_budget) != set(budgets):
                raise RuntimeError(
                    f"{run_id}: solver returned budgets {sorted(records_by_budget)}, "
                    f"expected {budgets}"
                )
            expected_backend = "cuda" if config.startswith("GPU") else "cpu"
            rows = []
            for budget in budgets:
                record = records_by_budget[budget]
                for key, expected in (
                    ("backend", expected_backend),
                    ("train_scenarios", str(args.convergence_train_scenarios)),
                    ("oos_scenarios", str(args.convergence_evaluation_scenarios)),
                    ("train_seed", str(args.train_seed)),
                    ("oos_seed", str(args.test_seed)),
                    ("algo_seed", str(seed)),
                ):
                    if record.get(key) != expected:
                        raise RuntimeError(
                            f"{run_id}: {key}={record.get(key)!r}, expected {expected!r}"
                        )
                rows.append({
                    "run_id": run_id,
                    "instance": args.convergence_instance,
                    "requests": request_count(args.convergence_instance),
                    "train_scenarios": args.convergence_train_scenarios,
                    "evaluation_scenarios": args.convergence_evaluation_scenarios,
                    "train_seed": args.train_seed,
                    "evaluation_seed": args.test_seed,
                    "algo_seed": seed,
                    "method": method,
                    "nominal_budget_s": budget,
                    "wall_time_s": record["solve_wall_seconds"],
                    "generations": record["generations"],
                    "heldout_objective": record["oos_penalized"],
                    "distance": record["distance"],
                    "tw_violation": record["tw_violation"],
                    "oos_mean_capacity_violation": record["oos_mean_capacity_violation"],
                    "capacity_scale": args.convergence_capacity_scale,
                    "demand_spread": args.demand_spread,
                    "extra_routes": args.convergence_extra_routes,
                    "from_scratch": 1,
                    "profile": args.profile,
                })
            append_run(out_path, CONV_FIELDS, rows, run_id)
            existing.add(run_id)
    current_rows = [row for row in read_rows(out_path) if row.get("run_id") in selected_run_ids]
    if {row.get("run_id") for row in current_rows} != selected_run_ids:
        raise RuntimeError("convergence benchmark is missing one or more selected runs")
    expected_rows = len(selected_run_ids) * len(budgets)
    if len(current_rows) != expected_rows:
        raise RuntimeError(
            f"convergence benchmark has {len(current_rows)} selected rows, expected {expected_rows}"
        )
    write_rows(out_path, CONV_FIELDS, current_rows)


def run_memory(args: argparse.Namespace) -> None:
    binary = args.bin_dir / "gpu_memory_benchmark"
    scenario_tag = "-".join(str(x) for x in args.memory_scenarios)
    parallel_tag = "-".join(str(x) for x in args.memory_parallelism)
    run_id = (
        f"memory-{args.memory_mode}-n{args.memory_node_num}-r{args.memory_routes}-"
        f"s{scenario_tag}-p{parallel_tag}"
    )
    out_path = args.data_dir / "gpu_memory.csv"
    if any(row.get("run_id") == run_id for row in read_rows(out_path)):
        print(f"[SKIP] {run_id}")
        return
    command = [
        str(binary),
        "--scenarios", ",".join(str(x) for x in args.memory_scenarios),
        "--parallelism", ",".join(str(x) for x in args.memory_parallelism),
        "--node-num", str(args.memory_node_num),
        "--routes", str(args.memory_routes),
        "--mode", args.memory_mode,
    ]
    output = run_logged(command, {}, args.raw_dir / f"{run_id}.log", args.memory_timeout)
    lines = [line for line in output.splitlines() if line.strip()]
    try:
        header_index = next(index for index, line in enumerate(lines) if line.startswith("scenario_count,"))
    except StopIteration as exc:
        raise RuntimeError("memory benchmark output has no CSV header") from exc
    reader = csv.DictReader(lines[header_index:])
    required_raw = {
        "scenario_count", "parallelism", "mode", "persistent_allocated_bytes",
        "theoretical_bytes", "device_total_bytes", "device_name", "status",
    }
    missing_raw = required_raw - set(reader.fieldnames or [])
    if missing_raw:
        raise RuntimeError(f"memory benchmark CSV is missing columns {sorted(missing_raw)}")
    rows = []
    for rec in reader:
        if not rec.get("scenario_count") or not rec.get("parallelism"):
            raise RuntimeError(f"malformed memory benchmark CSV row: {rec}")
        if rec["mode"] != args.memory_mode:
            raise RuntimeError(
                f"memory benchmark returned mode={rec['mode']!r}; requested {args.memory_mode!r}"
            )
        measured = int(rec["persistent_allocated_bytes"]) / (1024.0 ** 3)
        theoretical = int(rec["theoretical_bytes"]) / (1024.0 ** 3)
        capacity = int(rec["device_total_bytes"]) / (1024.0 ** 3)
        rows.append(
            {
                "run_id": run_id,
                "instance": f"synthetic_pdptw_{(args.memory_node_num - 1) // 2}req",
                "requests": (args.memory_node_num - 1) // 2,
                "scenarios": rec["scenario_count"],
                "parallelism": rec["parallelism"],
                "mode": rec["mode"],
                "measured_gib": measured if rec["status"] == "ok" else "",
                "theoretical_gib": theoretical,
                "device_capacity_gib": capacity,
                "device_name": rec["device_name"],
                "memory_kind": f"persistent_device_allocation_delta_{args.memory_mode}_path",
                "status": rec["status"],
                "profile": args.profile,
            }
        )
    expected = len(args.memory_scenarios) * len(args.memory_parallelism)
    if len(rows) != expected:
        raise RuntimeError(f"memory sweep returned {len(rows)} rows, expected {expected}")
    append_run(out_path, MEMORY_FIELDS, rows, run_id)


def prepare_small_instance(base: str, target: str, request_limit: int,
                           manifest_path: Path) -> int:
    """Create a Li&Lim sub-instance and a feasible seed by filtering its BKS routes."""
    safe_name = re.compile(r"^[A-Za-z0-9_.-]+$")
    if not safe_name.fullmatch(base) or not safe_name.fullmatch(target):
        raise ValueError("SCIP base and target instance names may only contain letters, digits, '.', '_' and '-'")
    if base.lower() == target.lower():
        raise ValueError("--scip-instance must differ from --scip-base-instance")
    if request_limit <= 0:
        raise ValueError("--scip-requests must be positive")
    instance_dir = REPO / "src" / "data" / "instance"
    bks_dir = REPO / "src" / "data" / "bks"
    source_instance = instance_dir / f"{base}.txt"
    source_lines = source_instance.read_text(encoding="utf-8").splitlines()
    records = []
    for line in source_lines[1:]:
        fields = line.split()
        if len(fields) >= 9:
            records.append((int(fields[0]), fields, line))
    depot = next(item for item in records if int(item[1][7]) == 0 and int(item[1][8]) == 0)
    pickups = [item for item in records if int(item[1][8]) != 0]
    pickups.sort(key=lambda item: item[0])
    pickups = pickups[:request_limit]
    records_by_id = {item[0]: item for item in records}
    selected = {depot[0]}
    for pickup in pickups:
        delivery_id = int(pickup[1][8])
        delivery = records_by_id.get(delivery_id)
        if delivery is None or int(delivery[1][7]) != pickup[0]:
            raise RuntimeError(f"{base}: request {pickup[0]} has an invalid delivery sibling {delivery_id}")
        selected.add(pickup[0])
        selected.add(delivery_id)
    if len(pickups) != request_limit:
        raise RuntimeError(f"{base}: only {len(pickups)} pickup records")

    bks_candidates = sorted(bks_dir.glob(f"{base.lower()}*.txt"))
    if not bks_candidates:
        raise RuntimeError(f"no BKS file found for {base}")
    route_lines = []
    covered = set()
    for line in bks_candidates[0].read_text(encoding="utf-8", errors="replace").splitlines():
        if not line.lower().startswith("route") or ":" not in line:
            continue
        nodes = [int(token) for token in line.split(":", 1)[1].split()]
        kept = [node for node in nodes if node in selected]
        if kept:
            duplicates = covered.intersection(kept)
            if duplicates:
                raise RuntimeError(f"BKS contains selected nodes more than once: {sorted(duplicates)}")
            route_lines.append(kept)
            covered.update(kept)
    missing = selected - {depot[0]} - covered
    if missing:
        raise RuntimeError(f"BKS filtering missed selected nodes: {sorted(missing)}")
    route_count = len(route_lines)
    if route_count <= 0:
        raise RuntimeError("filtered BKS has no non-empty route")

    # ConvertVrpProblem indexes original task ids as 0..task_num-1.  A filtered
    # Li & Lim file therefore must renumber the retained ids, including both
    # sibling columns and the BKS route ids; keeping sparse source ids causes an
    # out-of-range map dereference during conversion.
    id_map = {depot[0]: 0}
    for request_index, pickup in enumerate(pickups):
        id_map[pickup[0]] = 2 * request_index + 1
        id_map[int(pickup[1][8])] = 2 * request_index + 2

    chosen_records = []
    for old_id in sorted(selected, key=lambda value: id_map[value]):
        fields = list(records_by_id[old_id][1])
        fields[0] = str(id_map[old_id])
        pickup_sibling = int(fields[7])
        delivery_sibling = int(fields[8])
        fields[7] = str(id_map[pickup_sibling]) if pickup_sibling else "0"
        fields[8] = str(id_map[delivery_sibling]) if delivery_sibling else "0"
        chosen_records.append("\t".join(fields))

    remapped_routes = [[id_map[node] for node in nodes] for nodes in route_lines]
    first = source_lines[0].split()
    first[0] = str(route_count)
    target_instance = instance_dir / f"{target}.txt"
    target_bks = bks_dir / f"{target.lower()}.txt"
    target_instance.write_text("\t".join(first) + "\n" + "\n".join(chosen_records) + "\n", encoding="utf-8")
    bks_text = [f"Instance name :\t{target}", "Solution"]
    for index, nodes in enumerate(remapped_routes, 1):
        bks_text.append(f"Route {index} : " + " ".join(str(node) for node in nodes))
    target_bks.write_text("\n".join(bks_text) + "\n", encoding="utf-8")
    manifest = {
        "base_instance": base,
        "target_instance": target,
        "requests": request_limit,
        "routes": route_count,
        "selected_original_ids": sorted(selected),
        "original_to_subinstance_id": {str(key): value for key, value in sorted(id_map.items())},
        "source_bks": str(bks_candidates[0]),
        "source_instance_sha256": sha256(source_instance),
        "source_bks_sha256": sha256(bks_candidates[0]),
        "target_instance_sha256": sha256(target_instance),
        "target_bks_sha256": sha256(target_bks),
    }
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8")
    return route_count


def finite_number(text: str) -> float:
    try:
        value = float(text)
    except (TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) and abs(value) < 1e19 else math.nan


def run_scip(args: argparse.Namespace) -> None:
    exact = args.bin_dir / "pdptw_scip_benchmark"
    if not exact.exists():
        raise RuntimeError(f"missing {exact}; configure with -DUSE_SCIP=ON")
    route_count = prepare_small_instance(
        args.scip_base_instance, args.scip_instance, args.scip_requests,
        args.data_dir / "scip_instance_manifest.json",
    )
    out_path = args.data_dir / "scip_compare.csv"
    work_path = args.data_dir / "scip_work_rows.csv"
    work_fields = [
        "run_id", "instance", "requests", "scenarios", "algo_seed", "method", "wall_time_s",
        "incumbent_quality", "dual_bound", "proof_gap_fraction", "capacity_scale", "demand_spread",
        "threads", "time_limit_s", "terminal_status",
    ]
    work = read_rows(work_path)
    for row in work:
        row.setdefault(
            "threads",
            str(args.scip_threads if row.get("method") == "SCIP" else args.scip_heuristic_threads),
        )
    have = {row["run_id"] for row in work}

    comparison_tag = (
        f"{args.scip_base_instance}-to-{args.scip_instance}-r{args.scip_requests}-k{route_count}-"
        f"s{args.scip_scenarios}-dseed{args.scip_scenario_seed}-"
        f"cap{args.scip_capacity_scale:g}-spread{args.scip_demand_spread:g}-t{args.scip_budget:g}"
    )
    exact_run = f"scip-{comparison_tag}-threads{args.scip_threads}"
    heuristic_runs = {
        seed: f"scip-gpu-{comparison_tag}-aseed{seed}-threads{args.scip_heuristic_threads}"
        for seed in args.scip_heuristic_seeds
    }
    selected_run_ids = {exact_run, *heuristic_runs.values()}
    if exact_run not in have:
        trace_path = args.raw_dir / f"{exact_run}.trace.csv"
        command = [
            str(exact), "--instance", args.scip_instance,
            "--scenarios", str(args.scip_scenarios),
            "--seed", str(args.scip_scenario_seed),
            "--time-limit", str(args.scip_budget),
            "--cap-scale", str(args.scip_capacity_scale),
            "--routes", str(route_count),
            "--requests", str(args.scip_requests),
            "--threads", str(args.scip_threads),
            "--trace", str(trace_path),
        ]
        output = run_logged(command, {"PDPTW_DEMAND_SPREAD": str(args.scip_demand_spread)},
                            args.raw_dir / f"{exact_run}.solver.log", args.scip_budget + args.command_slack)
        final_match = re.search(
            r"status=(\d+)\s+incumbent=([^\s]+)\s+dual_bound=([^\s]+)\s+"
            r"proof_gap=([^\s]+)\s+time_s=([^\s]+)", output,
        )
        if final_match is None:
            raise RuntimeError(f"{exact_run}: SCIP output has no terminal status line")
        terminal = f"scip_status_{final_match.group(1)}"
        with trace_path.open("r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            required_trace = {"time_s", "event", "incumbent", "dual_bound", "proof_gap"}
            missing_trace = required_trace - set(reader.fieldnames or [])
            if missing_trace:
                raise RuntimeError(f"{exact_run}: trace is missing columns {sorted(missing_trace)}")
            trace_records = list(reader)
            if not trace_records:
                raise RuntimeError(f"{exact_run}: SCIP trace is empty")
            final_indices = [index for index, rec in enumerate(trace_records) if rec.get("event") == "final"]
            if len(final_indices) != 1:
                raise RuntimeError(
                    f"{exact_run}: SCIP trace must contain exactly one authoritative final row"
                )
            # Ignore any solver teardown events after the terminal snapshot.  Older
            # builds could emit a transient primal=dual event while freeing the
            # transformed problem, which is not a proof of optimality.
            trace_records = trace_records[: final_indices[0] + 1]
            for rec in trace_records:
                work.append(
                    {
                        "run_id": exact_run,
                        "instance": args.scip_instance,
                        "requests": args.scip_requests,
                        "scenarios": args.scip_scenarios,
                        "algo_seed": args.scip_scenario_seed,
                        "method": "SCIP",
                        "wall_time_s": rec["time_s"],
                        "incumbent_quality": finite_number(rec["incumbent"]),
                        "dual_bound": finite_number(rec["dual_bound"]),
                        "proof_gap_fraction": finite_number(rec["proof_gap"]),
                        "capacity_scale": args.scip_capacity_scale,
                        "demand_spread": args.scip_demand_spread,
                        "threads": args.scip_threads,
                        "time_limit_s": args.scip_budget,
                        "terminal_status": terminal,
                    }
                )
        write_rows(work_path, work_fields, work)
        have.add(exact_run)

    for seed in args.scip_heuristic_seeds:
        run_id = heuristic_runs[seed]
        if run_id in have:
            continue
        trace = run_one_trace(
            args, args.scip_instance, args.scip_capacity_scale, args.scip_scenarios,
            args.scip_budget, seed, "GPU-8thread", 0, run_id,
            train_seed=args.scip_scenario_seed,
            demand_spread=args.scip_demand_spread,
            feasible_only=True,
            from_scratch=True,
            solver_threads=args.scip_heuristic_threads,
            disable_srex=True,
        )
        for generation, wall, quality in trace:
            work.append(
                {
                    "run_id": run_id,
                    "instance": args.scip_instance,
                    "requests": args.scip_requests,
                    "scenarios": args.scip_scenarios,
                    "algo_seed": seed,
                    "method": "GPU memetic",
                    "wall_time_s": wall,
                    "incumbent_quality": quality,
                    "dual_bound": "",
                    "proof_gap_fraction": "",
                    "capacity_scale": args.scip_capacity_scale,
                    "demand_spread": args.scip_demand_spread,
                    "threads": args.scip_heuristic_threads,
                    "time_limit_s": args.scip_budget,
                    "terminal_status": "time_limit",
                }
            )
        write_rows(work_path, work_fields, work)
        have.add(run_id)

    current_work = [row for row in work if row.get("run_id") in selected_run_ids]
    missing_runs = selected_run_ids - {row.get("run_id") for row in current_work}
    if missing_runs:
        raise RuntimeError(f"SCIP comparison is missing cached runs: {sorted(missing_runs)}")
    exact_rows = [row for row in current_work if row["run_id"] == exact_run]
    if not exact_rows:
        raise RuntimeError("SCIP comparison produced no exact-solver trace")
    finite_incumbents = [finite_number(row["incumbent_quality"]) for row in current_work]
    finite_incumbents = [value for value in finite_incumbents if math.isfinite(value)]
    if not finite_incumbents:
        raise RuntimeError("SCIP comparison produced no feasible incumbent")
    # The exact rows retain trace order and the reader truncates at the unique
    # event=final snapshot, so the last row is the authoritative terminal state
    # even when another event has the same rounded timestamp.
    exact_final = exact_rows[-1]
    final_gap = finite_number(exact_final["proof_gap_fraction"])
    if math.isfinite(final_gap) and final_gap <= 1e-9:
        reference = finite_number(exact_final["incumbent_quality"])
        reference_kind = "scip_proven_optimum"
    else:
        reference = min(finite_incumbents)
        reference_kind = "ex_post_best_observed_not_proven"

    final_rows = []
    current_work.sort(key=lambda row: (row["method"] != "SCIP", int(row["algo_seed"]), float(row["wall_time_s"])))
    for row in current_work:
        final_rows.append(
            {
                **row,
                "reference_quality": reference,
                "reference_kind": reference_kind,
                "profile": args.profile,
            }
        )
    write_rows(out_path, SCIP_FIELDS, final_rows)
    print(f"[DATA] {out_path.name}: {len(final_rows)} rows; reference={reference:.6f} ({reference_kind})")


def write_provenance(args: argparse.Namespace) -> None:
    binaries = {}
    for name in [
        "paper_experiments", "scenario_eval_benchmark", "gpu_memory_benchmark",
        "pdptw_scip_benchmark",
    ]:
        path = args.bin_dir / name
        if path.exists():
            binaries[name] = {"path": str(path), "sha256": sha256(path)}
    runtime_inputs = {}
    for instance in args.runtime_instances:
        instance_path = REPO / "src" / "data" / "instance" / f"{instance}.txt"
        selected_bks = bks_path(instance)
        runtime_inputs[instance] = {
            "instance_path": str(instance_path),
            "instance_sha256": sha256(instance_path),
            "bks_path": str(selected_bks),
            "bks_sha256": sha256(selected_bks),
            "route_count": bks_route_count(instance),
        }
    info = {
        "created_unix": time.time(),
        "python": sys.version,
        "profile": args.profile,
        "device": device_name(),
        "binaries": binaries,
        "runner_sha256": sha256(Path(__file__)),
        "plotter_sha256": sha256(HERE / "plot_pdptw_experiments.py"),
        "seed_inventory": {
            "shared_train_demand_seed": args.train_seed,
            "oos_test_seed": args.test_seed,
            "oos_algorithm_seeds": list(args.oos_seeds),
            "oos_reference_algorithm_seeds": [],
            "convergence_algorithm_seeds": list(args.convergence_seeds),
            "scip_demand_scenario_seed": args.scip_scenario_seed,
            "scip_gpu_algorithm_seeds": list(args.scip_heuristic_seeds),
        },
        "deterministic_inputs": {
            "runtime": (
                "scenario evaluator uses the same synthetic demand matrix and route batch for "
                "the serial CPU and CUDA lean-path measurements"
            ),
            "gpu_memory": "allocation benchmark depends on dimensions, not sampled demands",
        },
        "scenario_policy": {
            "formal_stochastic_demand_draws": FORMAL_STOCHASTIC_DRAWS,
            "nominal_scenario_slots": NOMINAL_SCENARIO_SLOTS,
            "total_scenario_slots": FORMAL_TOTAL_SCENARIOS,
            "oos": "1 nominal slot plus 1,000,000 common held-out stochastic draws",
            "runtime": "1 nominal slot plus 1,000,000 paired stochastic draws",
            "convergence": (
                "manageable fixed training sample; each budget-end frozen tour is evaluated "
                "on one nominal slot plus 1,000,000 common stochastic held-out draws"
            ),
            "gpu_memory": (
                "grid includes 1 nominal slot plus exactly 1,000,000 stochastic-demand "
                "slots; stored scenario counts therefore use N+1"
            ),
            "scip_exception": (
                "SCIP uses the explicitly reported small scenario count because its "
                "deterministic equivalent expands with scenarios; this is not presented "
                "as a 1,000,000-scenario experiment"
            ),
        },
        "scip_comparison_initialization": {
            "SCIP": "BKS warm start",
            "GPU memetic": "from scratch",
        },
        "capacity_aggregation": "mean over scenarios",
        "model": "static stochastic-demand PDPTW; fixed route evaluated under scenarios; no recourse decisions",
        "oos_selected_instance_protocol": {
            "instance": args.oos_instance,
            "train_scenarios": list(args.oos_train_scenarios),
            "train_seed": args.train_seed,
            "algo_seeds": list(args.oos_seeds),
            "final_heldout_scenarios": args.oos_test_scenarios,
            "final_heldout_stochastic_draws": (
                args.oos_test_scenarios - NOMINAL_SCENARIO_SLOTS
            ),
            "final_heldout_seed": args.test_seed,
            "reference_semantics": "largest-N curve endpoint; schema compatibility only",
            "plotted_quantity": "direct heldout objective",
        },
        "runtime_protocol": {
            "binary": "scenario_eval_benchmark",
            "path": "lean per-route total capacity violation",
            "instances": list(args.runtime_instances),
            "total_scenario_slots": args.runtime_scenarios,
            "stochastic_draws": args.runtime_scenarios - NOMINAL_SCENARIO_SLOTS,
            "repetitions": args.runtime_repetitions,
            "execution_order": "r0/r2 CPU-first; r1/r3 GPU-first",
            "timing": "mean of 20 evaluator calls after one excluded warm-up per backend; setup and allocation excluded",
            "cpu": "serial CPU evaluator",
            "gpu": "CUDA evaluator",
            "pairing": "identical demand matrix and route batch after excluded warm-up",
            "workload_shape": (
                "node count and BKS route count from each named instance; deterministic "
                "synthetic demands and route ordering"
            ),
            "correctness_gate": "max_abs_diff <= 1e-6",
            "input_files": runtime_inputs,
        },
        "convergence_protocol": {
            "instance": args.convergence_instance,
            "train_scenarios": args.convergence_train_scenarios,
            "train_seed": args.train_seed,
            "evaluation_scenarios": args.convergence_evaluation_scenarios,
            "evaluation_stochastic_draws": (
                args.convergence_evaluation_scenarios - NOMINAL_SCENARIO_SLOTS
            ),
            "evaluation_seed": args.test_seed,
            "algorithm_seeds": list(args.convergence_seeds),
            "nominal_budgets_s": list(sorted(args.convergence_budgets)),
            "from_scratch": True,
            "plotted_quantity": "direct heldout objective at each predeclared budget",
            "note": (
                "budget runs reset the same algorithm seed; no monotone post-processing "
                "is applied to heldout objectives"
            ),
        },
    }
    (args.data_dir / "provenance.json").write_text(json.dumps(info, indent=2, sort_keys=True), encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bin-dir", type=Path, default=REPO / "build-cpu")
    parser.add_argument("--data-dir", type=Path, default=REPO / "results/pdptw/data")
    parser.add_argument("--raw-dir", type=Path, default=REPO / "results/pdptw/raw")
    parser.add_argument("--only", choices=["all", "oos", "runtime", "convergence", "scip", "memory"], default="all")
    parser.add_argument("--profile", default="pdptw-pilot")
    parser.add_argument("--command-slack", type=float, default=240.0)
    parser.add_argument("--demand-spread", type=float, default=1.5)
    parser.add_argument("--train-seed", type=int, default=120042)
    parser.add_argument("--test-seed", type=int, default=7777777)

    parser.add_argument("--oos-instance", default="lc202")
    parser.add_argument("--oos-capacity-scale", type=float, default=0.5)
    parser.add_argument("--oos-extra-routes", type=int, default=2)
    parser.add_argument(
        "--oos-test-scenarios", type=int, default=FORMAL_TOTAL_SCENARIOS,
        help="total slots; formal default is one nominal plus 1,000,000 stochastic draws",
    )
    parser.add_argument(
        "--oos-train-scenarios",
        type=parse_int_list,
        default=parse_int_list("1,100,1000,10000"),
    )
    parser.add_argument("--oos-seeds", type=parse_int_list, default=parse_int_list("6301"))
    parser.add_argument(
        "--oos-reference-seeds", type=parse_int_list,
        default=parse_int_list("9101,9102,9103"),
        help="deprecated compatibility option; OOS now uses the largest-N curve endpoint",
    )
    parser.add_argument("--oos-budget", type=float, default=45.0)
    parser.add_argument("--oos-reference-budget", type=float, default=120.0)

    parser.add_argument("--runtime-instances", type=lambda s: s.split(","),
                        default=[f"LC1_2_{index}" for index in range(1, 11)])
    parser.add_argument(
        "--runtime-scenarios", type=int, default=FORMAL_TOTAL_SCENARIOS,
        help="total slots; formal default is one nominal plus 1,000,000 stochastic draws",
    )
    parser.add_argument("--runtime-repetitions", type=int, default=4)
    parser.add_argument("--runtime-timeout", type=float, default=900.0)

    parser.add_argument("--convergence-instance", default="LR2_2_8")
    parser.add_argument("--convergence-capacity-scale", type=float, default=0.5)
    parser.add_argument("--convergence-extra-routes", type=int, default=2)
    parser.add_argument(
        "--convergence-train-scenarios", type=int, default=1_000,
        help="manageable in-sample scenario count used by the full-state memetic solver",
    )
    parser.add_argument(
        "--convergence-evaluation-scenarios", type=int, default=FORMAL_TOTAL_SCENARIOS,
        help="held-out total slots: one nominal plus 1,000,000 stochastic draws",
    )
    parser.add_argument("--convergence-seeds", type=parse_int_list, default=parse_int_list("4101,4102,4103"))
    parser.add_argument(
        "--convergence-budgets", type=parse_float_list,
        default=parse_float_list("60,100,140,220,300"),
        help=(
            "predeclared nominal optimization budgets; values start near one full "
            "generation and every marker is plotted at its measured wall time"
        ),
    )

    parser.add_argument("--memory-scenarios", type=parse_int_list,
                        # Reader convention: slot 0 is nominal, so these totals
                        # represent 10^2,...,10^6 stochastic demand draws.
                        default=parse_int_list("101,1001,10001,100001,1000001"))
    parser.add_argument("--memory-parallelism", type=parse_int_list, default=parse_int_list("1,2,3,4"))
    parser.add_argument("--memory-node-num", type=int, default=201)
    parser.add_argument("--memory-routes", type=int, default=20)
    parser.add_argument("--memory-mode", choices=["full", "lean"], default="full")
    parser.add_argument("--memory-timeout", type=float, default=1200.0)

    parser.add_argument("--scip-base-instance", default="lrc201")
    parser.add_argument("--scip-instance", default="iclr_pdptw_lrc201_r50")
    parser.add_argument("--scip-requests", type=int, default=50)
    parser.add_argument(
        "--scip-scenarios", type=int, default=11,
        help=(
            "small-scenario exact-comparison exception; SCIP's deterministic equivalent "
            "cannot be scaled to the 1,000,000-scenario policy used by the GPU experiments"
        ),
    )
    parser.add_argument("--scip-scenario-seed", type=int, default=5151)
    parser.add_argument("--scip-heuristic-seeds", type=parse_int_list, default=parse_int_list("5101"))
    parser.add_argument("--scip-capacity-scale", type=float, default=0.5)
    parser.add_argument("--scip-demand-spread", type=float, default=0.5)
    parser.add_argument("--scip-budget", type=float, default=1000.0)
    parser.add_argument("--scip-threads", type=int, default=8)
    parser.add_argument("--scip-heuristic-threads", type=int, default=8)
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    args.data_dir.mkdir(parents=True, exist_ok=True)
    args.raw_dir.mkdir(parents=True, exist_ok=True)
    actions = {
        "oos": run_oos,
        "runtime": run_runtime,
        "convergence": run_convergence,
        "scip": run_scip,
        "memory": run_memory,
    }
    selected = list(actions) if args.only == "all" else [args.only]
    for name in selected:
        print(f"\n=== {name} ===")
        actions[name](args)
        write_provenance(args)


if __name__ == "__main__":
    main()
