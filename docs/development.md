# Development Guide

Guidelines for contributing code to the multimodal assistant. This project is
developed by four students in parallel, so consistency and low coupling are
critical.

## Development Workflow

1. **Fork / branch** - work on a feature branch named after the task
   (`feat/aruco-detection`).
2. **Implement** - follow the module layout described below.
3. **Self-test** - run the linters and build checks locally.
4. **Open a PR** - keep PRs small and focused on one concern.
5. **Review** - CI runs the GitHub Actions workflow on every PR.

## Package Responsibilities

Each contributor owns one package - do not commit to another contributor's
package without coordination:

| Contributor | Package(s)                     |
|-------------|--------------------------------|
| 1           | `assistant_perception`         |
| 2           | `assistant_interaction`        |
| 3           | `assistant_motion`             |
| 4           | `assistant_msgs` + `assistant_bringup` |

## Code Conventions

- Python 3, PEP 8, 79-column lines (flake8 default).
- Type hints on all public functions and methods.
- Docstrings on every class and public method (PEP 257).
- ROS logging (`self.get_logger()`) - never `print()`.
- No hardcoded paths - use parameters declared via
  `declare_parameter(...)`.
- Mark unfinished integration points with `# TODO(...)` comments.

## Node Conventions

Every node:

1. Declares its parameters in `__init__` with defaults.
2. Loads parameters through the central YAML config.
3. Provides a `shutdown_callback()` for clean teardown.
4. Handles `KeyboardInterrupt` in its `main()`.
5. Refuses to crash the graph when a dependency is missing - it logs a warning
   and continues.

## Adding a New Interface

1. Create the `.msg`, `.srv`, or `.action` file under `assistant_msgs/`.
2. Register it in `assistant_msgs/CMakeLists.txt` in the corresponding list.
3. Rebuild `assistant_msgs` and the dependent packages.
4. Update `docs/topics.md`, `docs/services.md`, or `docs/actions.md`.

## Build Checks

```bash
# Linting
cd src/assistant_perception && python3 -m flake8 .
cd src/assistant_interaction && python3 -m flake8 .
cd src/assistant_motion && python3 -m flake8 .

# Full build from workspace root
colcon build --symlink-install
```

## Testing

While `ament_copyright`, `ament_flake8`, `ament_pep257` are declared as test
deps, no mandatory tests are enforced at build time. Contributors are
encouraged to add pytest unit tests for pure logic (e.g. the arbitration rule
in `selection_manager_node`).

## Git Hygiene

- Commit messages follow conventional style: `feat:`, `fix:`, `docs:`, `ref:`.
- Never commit model weights, logs, or build artifacts (see `.gitignore`).
- Rebase frequently to keep history linear and conflicts minimal.

## The Skeleton is the Contract

The generated skeleton defines **interfaces and structure, not behavior**. When
you integrate a real model (YOLO, MediaPipe, Vosk, MoveIt), preserve:

- The existing topic/service/action names.
- The parameter names in `config/system.yaml`.
- The "fail-open" startup behavior.
- The package boundaries.