# Contributing to FR5 ROS 2

## 1. General Principles

- Preserve working functionality.
- Keep changes small and reviewable.
- Treat hardware safety as the highest priority.
- Read `AGENTS.md` before changing code.
- Report actual build and test results.

## 2. Branch Strategy

`main` is the stable integration branch.

| Prefix | Purpose |
|---|---|
| `feature/` | New functionality |
| `fix/` | Bug fixes |
| `refactor/` | Code restructuring |
| `chore/` | Documentation and maintenance |
| `experiment/` | Experimental work |

Examples: `feature/fr5-gripper`, `feature/mujoco-integration`, `ffix/servo-timeout`.

## 3. Commit Convention

Use conventional prefixes:

- `feat`: new functionality
- `fix`: bug fixes
- `refactor`: restructuring
- `docs`: documentation
- `test`: tests
- `build`: build system
- `chore`: maintenance

Example: `feat: add FR5 gripper interface`.

## 4. Development Workflow

Update `main` and create a branch:

```bash
git switch main
git pull --ff-only origin main
git switch -c feature/my-feature
```

Inspect changes:

```bash
git status
git diff --check
git diff
```

Build affected packages and run applicable tests. Then commit and push:

```bash
git add <specific-files>
git commit -m "feat: describe the change"
git push -u origin feature/my-feature
```

## 5. Pull Request Requirements

Every PR should describe its purpose, scope, modified modules, design decisions, build commands, actual test results and known limitations.

State clearly whether real-robot testing was performed. Compilation success is not hardware validation.

## 6. Code Review

Review correctness, ROS 2 dependencies, interface compatibility, SD[ compatibility, hardware safety, error handling, documentation and tests.

Changes involving ServoJ, robot motion, hardware lifecycle or fault handling require additional review.

## 7. Repository Hygiene

Do not commit build artifacts, Python caches, temporary backups, local IDE settings, runtime logs, credentials or unauthorized vendor binaries.

Do not delete historical experimental records without reviewing their purpose.

## 8. Testing Expectations

Prefer this order:

1. Static analysis.
2. Unit tests.
3. Package compilation.
4. Plugin and launch validation.
5. Read-only hardware testing.
6. Explicitly authorized real-robot testing.

Document the exact validation level.

## 9. Documentation

Update `README.md`, `AGENTS.md` and module documentation when architecture, setup instructions or development constraints change.

Record significant changes in `CHANGELOG.md` .
