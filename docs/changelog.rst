Release history
===============

`View on PyPI <https://pypi.org/project/w9-pathfinding/#history>`_

**0.1.3** (2026-01-14)

- Fixed unintended nondeterministic behavior in several MAPF algorithms
  (HCA*, WHCA*, and CBS).
- Added an optional random seed parameter to enable reproducible results in CBS.

**0.1.2** (2025-08-03)

- Fixed various bugs in MAPF algorithms.
- Refactored the C++ API.

**0.1.1** (2025-06-19)

- Added more fine-grained control over pause action costs.
- Added sphinx documentation.

**0.1.0** (2025-06-07)

- Added `stay_at_goal` parameter to `SpaceTimeAStar`.
- Reorganized the Python API.
- Reduced memory footprint when creating a large number of grid-based environments.
- Fixed a bug in `SpaceTimeAStar` where the goal could be temporarily
  blocked by a dynamic obstacle.
