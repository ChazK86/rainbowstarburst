"""Immutable formation geometry, adjacency, and influence caches."""

from __future__ import annotations

from collections import deque
import math

import numpy as np

from audio.features import SPECTRUM_FREQUENCIES, SPECTRUM_MAX_HZ, SPECTRUM_MIN_HZ


def _normalize_rows(values: np.ndarray) -> np.ndarray:
    lengths = np.linalg.norm(values, axis=1, keepdims=True)
    lengths[lengths < 1e-9] = 1.0
    return values / lengths


class FormationTopology:
    def __init__(self, pyramids):
        self.count = len(pyramids)
        self.home_paths = []
        bases = []
        apexes = []
        for pyramid in pyramids:
            local = pyramid._apply_corner_offsets_to_local_path(pyramid.local_path)
            path = np.asarray(
                pyramid.transform_path(local, pyramid.physics.position, pyramid.physics.rotation),
                dtype=float,
            )
            self.home_paths.append(path)
            bases.append(path[: pyramid.num_corners].mean(axis=0))
            apex_index = min(len(path) - 1, pyramid.num_corners + 1)
            apexes.append(path[apex_index])
        self.centers = np.asarray(bases, dtype=float) if bases else np.empty((0, 3), dtype=float)
        self.apexes = np.asarray(apexes, dtype=float) if apexes else np.empty((0, 3), dtype=float)
        self.origin = self.centers.mean(axis=0) if self.count else np.zeros(3)
        raw_normals = self.apexes - self.centers
        self.normals = _normalize_rows(raw_normals) if self.count else np.empty((0, 3))
        radii = np.linalg.norm(self.centers - self.origin, axis=1) if self.count else np.zeros(0)
        self.core_radius = max(0.5, float(np.median(radii))) if len(radii) else 1.0
        self.latitude = self.normals[:, 1] if self.count else np.zeros(0)
        self.azimuth = np.arctan2(self.normals[:, 2], self.normals[:, 0]) if self.count else np.zeros(0)
        self.adjacency, shared_edges = self._build_adjacency(pyramids)
        self.shared_surface = shared_edges >= max(1, self.count // 2)
        self.edges = np.asarray(
            [(i, j) for i, neighbors in enumerate(self.adjacency) for j in neighbors if i < j],
            dtype=np.int32,
        ).reshape(-1, 2)
        self.parity = self._build_parity()
        self.band_influence = self._build_band_influence()
        self.pitch_influence = self._build_pitch_influence()
        (
            self.resonant_frequencies,
            self.resonance_bandwidths,
            self.resonance_weights,
            self.resonance_variation,
        ) = self._build_resonance_map()
        self.band_seeds = tuple(int(np.argmax(self.band_influence[:, band])) for band in range(8)) if self.count else ()
        self._distance_cache: dict[int, np.ndarray] = {}

    def _build_adjacency(self, pyramids):
        adjacency = [set() for _ in range(self.count)]
        edge_owners: dict[tuple, int] = {}
        shared = 0
        for index, (pyramid, path) in enumerate(zip(pyramids, self.home_paths)):
            base = path[: pyramid.num_corners]
            for edge_index in range(pyramid.num_corners):
                first = tuple(np.round(base[edge_index], 6))
                second = tuple(np.round(base[(edge_index + 1) % pyramid.num_corners], 6))
                edge = tuple(sorted((first, second)))
                other = edge_owners.get(edge)
                if other is None:
                    edge_owners[edge] = index
                else:
                    adjacency[index].add(other)
                    adjacency[other].add(index)
                    shared += 1

        # Spaced formations have no shared base edges.  A symmetric nearest-
        # neighbor graph gives every mode a stable route without changing shape.
        if self.count > 1 and not any(adjacency):
            neighbor_count = min(4, self.count - 1)
            for index in range(self.count):
                distance = np.linalg.norm(self.centers - self.centers[index], axis=1)
                for other in np.argsort(distance)[1 : neighbor_count + 1]:
                    adjacency[index].add(int(other))
                    adjacency[int(other)].add(index)
        return tuple(tuple(sorted(items)) for items in adjacency), shared

    def _build_parity(self):
        parity = np.zeros(self.count, dtype=int)
        visited = np.zeros(self.count, dtype=bool)
        for root in range(self.count):
            if visited[root]:
                continue
            visited[root] = True
            queue = deque([root])
            while queue:
                node = queue.popleft()
                for neighbor in self.adjacency[node]:
                    if not visited[neighbor]:
                        visited[neighbor] = True
                        parity[neighbor] = 1 - parity[node]
                        queue.append(neighbor)
        return parity

    def _build_band_influence(self):
        if not self.count:
            return np.empty((0, 8), dtype=float)
        targets = np.array([
            [0.0, 1.0, 0.0], [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0], [-0.5, 0.0, 0.866], [-0.5, 0.0, -0.866],
            [0.68, 0.55, 0.48], [-0.78, 0.48, 0.40], [0.05, -0.55, -0.83],
        ], dtype=float)
        targets = _normalize_rows(targets)
        widths = np.array([1.9, 1.9, 2.5, 2.5, 2.5, 4.2, 4.8, 5.4])
        dots = np.clip(self.normals @ targets.T, -1.0, 1.0)
        influence = np.exp((dots - 1.0) * widths[None, :])
        # The secondary-band contribution prevents hard seams.
        influence = 0.75 * influence + 0.125 * np.roll(influence, 1, axis=1) + 0.125 * np.roll(influence, -1, axis=1)
        column_max = np.maximum(influence.max(axis=0, keepdims=True), 1e-9)
        return influence / column_max

    def _build_pitch_influence(self):
        if not self.count:
            return np.empty((0, 12), dtype=float)
        target_angles = np.arange(12, dtype=float) / 12.0 * 2.0 * math.pi - math.pi
        delta = np.angle(np.exp(1j * (self.azimuth[:, None] - target_angles[None, :])))
        return np.exp(-0.5 * np.square(delta / 0.42))

    def _build_resonance_map(self):
        """Assign every pyramid a distinct frequency and acoustic character.

        Frequencies progress over a geometry-derived spiral instead of pyramid
        creation order.  Neighboring resonators overlap, but no two pyramids
        have the exact same center frequency when the formation has >1 member.
        """
        if not self.count:
            return (
                np.zeros(0),
                np.zeros(0),
                np.empty((0, len(SPECTRUM_FREQUENCIES))),
                np.zeros(0),
            )
        index = np.arange(self.count, dtype=float)
        azimuth = np.mod((self.azimuth + math.pi) / (2.0 * math.pi), 1.0)
        latitude = (self.latitude + 1.0) * 0.5
        spiral_key = np.mod(azimuth + latitude * 0.61803398875, 1.0)
        order = np.lexsort((index, latitude, spiral_key))
        rank = np.empty(self.count, dtype=float)
        rank[order] = np.arange(self.count, dtype=float)
        fraction = rank / max(1.0, self.count - 1.0)
        frequencies = SPECTRUM_MIN_HZ * np.power(
            SPECTRUM_MAX_HZ / SPECTRUM_MIN_HZ,
            fraction,
        )
        variation = np.mod(
            np.sin((index + 1.0) * 12.9898 + self.latitude * 4.1414) * 43758.5453,
            1.0,
        )
        bandwidths = 0.26 + 0.46 * variation  # octave standard deviation
        input_frequencies = SPECTRUM_FREQUENCIES[None, :]
        center = frequencies[:, None]

        def gaussian_at(multiplier, width_scale=1.0):
            distance = np.log2(input_frequencies / np.maximum(center * multiplier, 1e-9))
            return np.exp(-0.5 * np.square(distance / (bandwidths[:, None] * width_scale)))

        fundamental = gaussian_at(1.0)
        second_harmonic = gaussian_at(2.0, 1.18) * (0.08 + 0.28 * variation[:, None])
        third_harmonic = gaussian_at(3.0, 1.28) * (0.04 + 0.18 * (1.0 - variation[:, None]))
        subharmonic = gaussian_at(0.5, 1.22) * (0.03 + 0.12 * variation[:, None])
        weights = fundamental + second_harmonic + third_harmonic + subharmonic
        weights /= np.maximum(weights.sum(axis=1, keepdims=True), 1e-12)
        return frequencies, bandwidths, weights, variation

    def distances_from(self, seed: int) -> np.ndarray:
        seed = int(np.clip(seed, 0, max(0, self.count - 1)))
        if seed in self._distance_cache:
            return self._distance_cache[seed]
        distance = np.full(self.count, self.count + 1, dtype=np.int32)
        if self.count:
            distance[seed] = 0
            queue = deque([seed])
            while queue:
                node = queue.popleft()
                for neighbor in self.adjacency[node]:
                    if distance[neighbor] > distance[node] + 1:
                        distance[neighbor] = distance[node] + 1
                        queue.append(neighbor)
        self._distance_cache[seed] = distance
        return distance

    def shortest_path(self, start: int, goal: int) -> tuple[int, ...]:
        if not self.count:
            return ()
        start, goal = int(start), int(goal)
        previous = {start: None}
        queue = deque([start])
        while queue and goal not in previous:
            node = queue.popleft()
            for neighbor in self.adjacency[node]:
                if neighbor not in previous:
                    previous[neighbor] = node
                    queue.append(neighbor)
        if goal not in previous:
            return (start,)
        route = []
        cursor = goal
        while cursor is not None:
            route.append(cursor)
            cursor = previous[cursor]
        return tuple(reversed(route))

    def greedy_route(self, energy: np.ndarray, start: int | None = None, limit=48) -> tuple[int, ...]:
        if not self.count:
            return ()
        values = np.asarray(energy, dtype=float)
        node = int(np.argmax(values) if start is None else start)
        route = [node]
        visited = {node}
        for _ in range(min(int(limit), self.count) - 1):
            options = [item for item in self.adjacency[node] if item not in visited]
            if not options:
                break
            node = max(options, key=lambda item: values[item])
            route.append(node)
            visited.add(node)
        return tuple(route)
