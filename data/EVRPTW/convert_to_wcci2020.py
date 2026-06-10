#!/usr/bin/env python3

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path


RAW_DIR = Path(__file__).resolve().parent / "evrptw_instances"
OUT_DIR = Path(__file__).resolve().parent
SKIP_FILES = {"readme.txt"}
VALUE_IN_SLASHES_RE = re.compile(r"/([0-9]+(?:\.[0-9]+)?)/")
TRAILING_INT_RE = re.compile(r"(\d+)$")


@dataclass(frozen=True)
class RawNode:
    string_id: str
    kind: str
    x: float
    y: float
    demand: int


def trailing_int(token: str) -> int:
    match = TRAILING_INT_RE.search(token)
    if match is None:
        raise ValueError(f"Unable to parse trailing integer from token: {token}")
    return int(match.group(1))


def parse_metric(line: str) -> float:
    match = VALUE_IN_SLASHES_RE.search(line)
    if match is None:
        raise ValueError(f"Unable to parse metric from line: {line}")
    return float(match.group(1))


def format_number(value: float) -> str:
    rounded = round(value)
    if abs(value - rounded) < 1e-9:
        return str(int(rounded))
    return f"{value:.6f}".rstrip("0").rstrip(".")


def parse_instance(path: Path) -> tuple[RawNode, list[RawNode], list[RawNode], int, int, float, float]:
    depot: RawNode | None = None
    stations: list[RawNode] = []
    customers: list[RawNode] = []
    energy_capacity: float | None = None
    vehicle_capacity: float | None = None
    energy_consumption: float | None = None

    for raw_line in path.read_text().splitlines():
        line = raw_line.strip()
        if not line:
            continue

        tokens = line.split()
        if len(tokens) >= 8 and tokens[1] in {"d", "f", "c"}:
            node = RawNode(
                string_id=tokens[0],
                kind=tokens[1],
                x=float(tokens[2]),
                y=float(tokens[3]),
                demand=int(round(float(tokens[4]))),
            )
            if node.kind == "d":
                depot = node
            elif node.kind == "f":
                stations.append(node)
            else:
                customers.append(node)
            continue

        if line.startswith("Q Vehicle fuel tank capacity"):
            energy_capacity = parse_metric(line)
        elif line.startswith("C Vehicle load capacity"):
            vehicle_capacity = parse_metric(line)
        elif line.startswith("r fuel consumption rate"):
            energy_consumption = parse_metric(line)

    if depot is None:
        raise ValueError(f"Missing depot in {path}")
    if energy_capacity is None or vehicle_capacity is None or energy_consumption is None:
        raise ValueError(f"Missing vehicle metrics in {path}")

    customers.sort(key=lambda node: trailing_int(node.string_id))
    stations.sort(key=lambda node: trailing_int(node.string_id))

    min_vehicle_count = max(
        1,
        math.ceil(sum(node.demand for node in customers) / vehicle_capacity),
    )
    vehicles = min_vehicle_count

    return (
        depot,
        customers,
        stations,
        vehicles,
        int(round(vehicle_capacity)),
        energy_capacity,
        energy_consumption,
    )


def write_instance(
    output_path: Path,
    source_name: str,
    depot: RawNode,
    customers: list[RawNode],
    stations: list[RawNode],
    vehicles: int,
    vehicle_capacity: int,
    energy_capacity: float,
    energy_consumption: float,
) -> None:
    station_start_id = 2 + len(customers)
    lines = [
        f"NAME: {output_path.name}",
        (
            f"COMMENT: Converted from {source_name} (Schneider et al., 2014); "
            "time windows and service times omitted."
        ),
        "TYPE: EVRP",
        "OPTIMAL_VALUE: -1",
        f"VEHICLES: {vehicles}",
        f"DIMENSION: {1 + len(customers) + len(stations)}",
        f"STATIONS: {len(stations)}",
        f"CAPACITY: {vehicle_capacity}",
        f"ENERGY_CAPACITY: {format_number(energy_capacity)}",
        f"ENERGY_CONSUMPTION: {format_number(energy_consumption)}",
        "EDGE_WEIGHT_TYPE: EUC_2D",
        "NODE_COORD_SECTION",
        f"1 {format_number(depot.x)} {format_number(depot.y)}",
    ]

    for new_id, node in enumerate(customers, start=2):
        lines.append(f"{new_id} {format_number(node.x)} {format_number(node.y)}")

    for new_id, node in enumerate(stations, start=station_start_id):
        lines.append(f"{new_id} {format_number(node.x)} {format_number(node.y)}")

    lines.append("DEMAND_SECTION")
    lines.append("1 0")
    for new_id, node in enumerate(customers, start=2):
        lines.append(f"{new_id} {node.demand}")

    lines.append("STATIONS_COORD_SECTION")
    for station_id in range(station_start_id, station_start_id + len(stations)):
        lines.append(str(station_id))

    lines.append("DEPOT_SECTION")
    lines.append("1")
    lines.append("-1")
    lines.append("EOF")

    output_path.write_text("\n".join(lines) + "\n")


def main() -> None:
    converted_count = 0
    for source_path in sorted(RAW_DIR.glob("*.txt")):
        if source_path.name.lower() in SKIP_FILES:
            continue

        depot, customers, stations, vehicles, vehicle_capacity, energy_capacity, energy_consumption = parse_instance(source_path)
        output_path = OUT_DIR / f"{source_path.stem}.evrp"
        write_instance(
            output_path=output_path,
            source_name=source_path.name,
            depot=depot,
            customers=customers,
            stations=stations,
            vehicles=vehicles,
            vehicle_capacity=vehicle_capacity,
            energy_capacity=energy_capacity,
            energy_consumption=energy_consumption,
        )
        converted_count += 1

    print(f"Converted {converted_count} EVRPTW instances into {OUT_DIR}")


if __name__ == "__main__":
    main()
