# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

Vertexy is a conflict-driven clause learning (CDCL) constraint solver focused on graph-based problems, aimed at procedural content generation for games. Its distinguishing feature is graph-aware learning: learned constraints can be generalized across an entire graph topology rather than applying to individual variables only.

Windows-only (Visual Studio 2019+), C++17.

## Build

CMake generates in-place inside `build/` (the root `CMakeLists.txt` lives there, not at the repo root):

```
cd build
cmake .                                   # generates build/Vertexy.sln
cmake --build . --config Development -- -m
```

If no standalone CMake is installed, use the one bundled with Visual Studio:
`C:/Program Files/Microsoft Visual Studio/2022/Community/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe`

Build configs: **Debug** (no inlining, heavy sanity checks, `-debug` suffix on outputs), **Development** (everyday use, `-dev` suffix), **Release** (fully optimized). If the CMake cache was generated on another machine, delete `build/CMakeCache.txt` and `build/CMakeFiles/` before reconfiguring.

## Running tests

Tests run through the harness executable:

```
build/vertexyTestHarness/bin/Development/VertexyTestHarness.exe            # all registered tests
build/vertexyTestHarness/bin/Development/VertexyTestHarness.exe -run:Sudoku -run:NQueens-Graph   # specific tests
```

The arg parsing comes from EATest (`-run:<TestName>`, `-verbose`, `-?` for help). Test registration is manual in `vertexyTestHarness/src/SolverTest.cpp` — tests are added via `Suite.AddTest(...)` in `main()`, and large blocks are routinely commented in/out depending on what is being worked on. Constants at the top of that file control seed (`FORCE_SEED` — solver runs are fully deterministic per seed), iteration count, and problem sizes. There is no CTest integration; a test "passes" by returning 0 errors from its solve function.

## Project structure

Three CMake targets:

- **VertexyLib** (`vertexy/`) — the core solver static library. Links EASTL, EAAssert, nlohmann_json.
- **VertexyTestsLib** (`vertexyTests/`) — example problems used as tests (Sudoku, NQueens, Maze, TowersOfHanoi, KnightTour, Prefab/Tile tests, ZoneGraph, plus `BasicTests.cpp` for individual constraint types).
- **VertexyTestHarness** (`vertexyTestHarness/`) — thin EATest-based runner executable.

Each target splits headers/sources into `src/public/` (API) and `src/private/` (implementation), with mirrored subdirectory trees.

## Architecture of VertexyLib

`ConstraintSolver` (in `src/public/ConstraintSolver.h`) is the central class that ties everything together. Subsystems, each in its own public/private directory pair:

- **`constraints/`** — constraint implementations (clause, inequality, sum, table, all-different, cardinality, reachability, shortest-path/path-distance, disjunction, iff/offset). New constraint types implement `IConstraint` / `IBacktrackingSolverConstraint`.
- **`topology/`** — graph abstractions (`ITopology`, digraphs, grids, planar topologies) and `IGraphRelation`, which maps graph vertices to variables/values. Graph relations are what let a constraint (and learned clauses) be instantiated across every vertex of a graph. `topology/algo/` holds dynamic graph algorithms (reachability, shortest path, etc.).
- **`program/`** — a template-metaprogramming DSL for defining rule programs in C++ (see `ProgramDSL.h`). Formulas are declared with `VXY_FORMULA(name, arity)` / `VXY_DOMAIN_FORMULA(...)` and rules written with `<<=` (head implied by body), e.g. `stepPassable(vertex,step) <<= cellType(vertex).is(M_PASSABLE);`. `ProgramCompiler` grounds these into rules fed to the solver. `NOTES.md` at the repo root has informal notes on the formula syntax.
- **`rules/`** — `RuleDatabase` and ASP-style rule handling (SCC detection for cyclic dependencies) that bridge compiled programs into constraints.
- **`learning/`** — `ConflictAnalyzer`: conflict-driven clause learning, including graph-generalized learning.
- **`decision/`** — variable-selection heuristics (VSIDS, coarse LRB).
- **`restart/`** — restart policies (Luby, LBD, none).
- **`variable/`** — variable domains, the variable database (`SolverVariableDatabase`), and value propagators.
- **`ds/`** — data structures (e.g. `ValueBitset`).

## Conventions

- Uses **EASTL instead of the C++ standard library** (`eastl::vector`, `hash_map`, etc.); strings are `wstring` (EASTL wide strings). Third-party EA libraries live in `thirdParty/`. `<random>` is the one std exception (no EASTL equivalent).
- Copyright header: `// Copyright Proletariat, Inc. All Rights Reserved.`

## Auxiliary directories

- **`prefabs/`** — sample JSON prefab definitions for tile-based map tests; field documentation in its README.
- **`visualizations/`** — standalone HTML/JS icicle visualizer for solver search traces: open `index.html` and upload the output of the solver's `write()` function.
