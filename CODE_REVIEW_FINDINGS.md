# Code Organization Review - FRC 2026

Issues found during a full codebase review. Each item describes what's wrong, where it is, and a suggested fix.

---

## 1. RobotContainer is a 626-line god class

**Where:** `RobotContainer.java`

RobotContainer handles subsystem instantiation, command naming, button bindings, auto configuration, and robot-type switching all in one file. The `nameCommands()` method alone is 117 lines.

**Fix:** Extract into focused classes:
- `SubsystemFactory` — builds subsystems based on robot type (COMP/SIM/ALPHA)
- `CommandBindings` — all button/trigger mappings
- `AutoConfiguration` — PathPlanner named commands and auto chooser setup

---

## 2. Redundant nested `hopper/Hopper/` package

**Where:** `subsystems/hopper/Hopper/Hopper.java`

The `Hopper` subfolder inside `hopper` is capitalized and duplicates the parent name, creating the awkward package `frc.robot.subsystems.hopper.Hopper`. Every other subsystem puts its files directly in the subsystem folder.

**Fix:** Move `Hopper.java`, `HopperIO.java`, etc. directly into `subsystems/hopper/` and delete the nested folder.

---

## 3. Inconsistent folder naming conventions

**Where:** All subsystem folders

Some folders use `snake_case` (`shooter_flywheel`, `climb_claw_pivot`), others use `camelCase` (`intakePivot`, `intakeRollers`), and one uses `PascalCase` (`Hopper`). Pick one convention and stick with it across the entire `subsystems/` tree.

**Fix:** Standardize on one style (Java convention is lowercase, e.g. `shooterflywheel` or `shooter_flywheel`) and rename all folders consistently.

---

## 4. Magic numbers scattered through commands and controllers

**Where:** Multiple files

Examples:
- `RobotContainer.java`: `Units.Degrees.of(-180)`, hardcoded camera indices `3` and `4`, delay values `2` and `7` seconds
- `AutoShootCommand.java`: wait times `1` and `4` seconds with no named constant
- `ShootCommand.java` line 47: timing logic with magic numbers `2` and `24`
- `IntakePivotConstants.java` line 59: `82.7 / 360.0` zero offset
- `ShooterFlywheelConstants.java` line 34: `0.1` velocity adjustment
- `RobotSimState.java` line 27: `START_FUEL_CAPACITY = 8`

**Fix:** Move every literal into a named constant in the relevant `*Constants.java` file with a comment explaining the value.

---

## 5. ShootCommand isn't actually a Command

**Where:** `commands/ShootCommand.java`

`ShootCommand` doesn't extend any WPILib command base class. It's a helper that exposes `whileHeld()` and `onRelease()` methods that return commands. Meanwhile `IntakeCommand` and `AutoShootCommand` properly extend `SequentialCommandGroup`.

This inconsistency makes it confusing to understand what each class in `commands/` actually is.

**Fix:** Either make `ShootCommand` a proper command (extend `Command` or a command group), or rename it to `ShootCommandFactory`/`ShootHelper` to clearly communicate that it's not a command itself.

---

## 6. Dead code and commented-out blocks in RobotContainer

**Where:** `RobotContainer.java` lines 155-157, 172-173, 201, 476-481

```java
// rgb = new RGB(new RGBIOAddressableLED());
// rgb = new RGB(new RGBIOCANdle());
// canWatchdog = new CANWatchdog(new CANWatchdogIOComp(), rgb);
```

Line 201 creates a `VisionIOPhotonvisionSim` that is never assigned to a variable — it's constructed and immediately thrown away. Lines 476-481 have a commented-out bumper binding with complex logic.

**Fix:** Delete dead code. If it's needed later, it's in git history. The orphaned `VisionIOPhotonvisionSim` on line 201 should either be wired into the Vision subsystem or removed.

---

## 7. No shared base class for controller subsystems

**Where:** `IntakeController.java`, `HopperController.java`, `ShooterController.java`, `ClimbController.java`

All controllers extend `SubsystemBase` and follow the same pattern: hold references to sub-mechanisms, define a state enum, expose `setTargetStateCommand()`, and coordinate sub-mechanism targets in `periodic()`. None share a common abstract base.

**Fix:** Create an `AbstractController<S extends Enum<S>>` that provides the shared `setTargetStateCommand()` and periodic pattern. Each controller only defines its state enum and the state-to-target mapping.

---

## 8. No centralized CAN device registry

**Where:** Motor IDs are spread across `IntakePivotConstants.java`, `ShooterFlywheelConstants.java`, `HopperConstants.java`, etc.

Each subsystem's constants file defines its own motor CAN IDs independently. There's no single place to see all CAN device assignments, making it easy to accidentally assign duplicate IDs.

**Fix:** Create a `CANDevices.java` (or a section in `Constants.java`) that maps every CAN device to its ID. Subsystem constants reference this central registry.

---

## 9. Copy-paste subsystem boilerplate (5 files per mechanism)

**Where:** Every mechanism subsystem

Each mechanism has 5 near-identical files:
- `Xyz.java` (subsystem)
- `XyzConstants.java`
- `XyzIO.java` (interface)
- `XyzIOSim.java`
- `XyzIOTalonFX.java`

The shooter alone has 4 sub-mechanisms × 5 files = 20 files that all follow the same template. The `GenericMechanism`/`GenericRollers` in `lib/` was a step toward reducing this, but not all subsystems use it.

**Fix:** Migrate all simple mechanisms to use `GenericMechanism` or `GenericRollers` from `lib/`. For mechanisms that need custom behavior, extend the generic base rather than duplicating the full pattern.

---

## 10. Inconsistent logging strategy

**Where:** Across all subsystems

Some subsystems use `@AutoLogOutput` annotations (AdvantageKit), others make manual `Logger.recordOutput()` calls, and some do both in the same class. This makes log output unpredictable and debugging harder.

**Fix:** Pick one approach per subsystem layer:
- IO implementations: `@AutoLog` on Inputs classes (already partially done)
- Subsystem public state: `@AutoLogOutput` annotations
- Remove manual `Logger.recordOutput()` calls where `@AutoLogOutput` can replace them

---

## 11. RobotState.java mixes too many concerns (508 lines)

**Where:** `RobotState.java`

This class handles pose estimation, vision measurement integration, odometry processing, velocity calculations, interpolation, and field-relative transforms. It also hardcodes field dimensions (`Units.feetToMeters(57.573)`) with no reference to official specs.

**Fix:** Split into:
- `PoseEstimator` — odometry + vision fusion
- `FieldConstants` — field dimensions, key poses
- Keep `RobotState` as a thin facade that delegates to the above

---

## 12. Wildcard imports reduce readability

**Where:** `RobotContainer.java` lines 74-75 and elsewhere

```java
import frc.robot.subsystems.shooter.shooter_flywheel.*;
import frc.robot.subsystems.shooter.shooter_hood.*;
```

Wildcard imports hide which classes are actually used, can mask unused imports, and make it harder for new team members to trace dependencies.

**Fix:** Replace `*` imports with explicit class imports. Most IDEs can do this automatically (Organize Imports).

---

## Summary

| # | Issue | Severity | Effort |
|---|-------|----------|--------|
| 1 | RobotContainer god class | High | High |
| 2 | Redundant hopper/Hopper/ nesting | Low | Low |
| 3 | Inconsistent folder naming | Medium | Medium |
| 4 | Magic numbers everywhere | Medium | Medium |
| 5 | ShootCommand isn't a Command | Medium | Low |
| 6 | Dead/commented-out code | Low | Low |
| 7 | No controller base class | Medium | Medium |
| 8 | No CAN device registry | Medium | Low |
| 9 | Copy-paste subsystem boilerplate | High | High |
| 10 | Inconsistent logging | Medium | Medium |
| 11 | RobotState mixes concerns | Medium | High |
| 12 | Wildcard imports | Low | Low |

Start with the low-effort items (2, 5, 6, 8, 12) for quick wins, then tackle the structural issues (1, 7, 9, 11) when there's time for a larger refactor.
