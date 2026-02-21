// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Mode;
import frc.robot.commands.AlignToPoseCommand;
import frc.robot.commands.VibrateHIDCommand;
import frc.robot.commands.VisionTuningCommands;
import frc.robot.subsystems.canWatchdog.CANWatchdog;
import frc.robot.subsystems.canWatchdog.CANWatchdogIO;
import frc.robot.subsystems.canWatchdog.CANWatchdogIOComp;
import frc.robot.subsystems.climb.ClimbController;
import frc.robot.subsystems.climb.ClimbController.ClimbState;
import frc.robot.subsystems.climb.climb_claw_pivot.ClimbClawPivot;
import frc.robot.subsystems.climb.climb_claw_pivot.ClimbClawPivotIO;
import frc.robot.subsystems.climb.climb_claw_pivot.ClimbClawPivotIOSim;
import frc.robot.subsystems.climb.climb_claw_pivot.ClimbClawPivot.ClimbClawPivotTarget;
import frc.robot.subsystems.climb.climb_deploy_pivot.ClimbDeployPivot;
import frc.robot.subsystems.climb.climb_deploy_pivot.ClimbDeployPivotIO;
import frc.robot.subsystems.climb.climb_deploy_pivot.ClimbDeployPivotIOSim;
import frc.robot.subsystems.climb.climb_deploy_pivot.ClimbDeployPivot.ClimbDeployPivotTarget;
import frc.robot.subsystems.intake.IntakeController;
import frc.robot.subsystems.intake.IntakeController.IntakeState;
import frc.robot.subsystems.intake.intakePivot.IntakePivot;
import frc.robot.subsystems.intake.intakePivot.IntakePivotIO;
import frc.robot.subsystems.intake.intakePivot.IntakePivotIOSim;
import frc.robot.subsystems.intake.intakePivot.IntakePivotIOTalonFX;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollers;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollersIO;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollersIOSim;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollersIOTalonFX;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollers.IntakeRollersTarget;
import frc.robot.subsystems.elastic_updater.ElasticUpdater;
import frc.robot.subsystems.hopper.HopperController;
import frc.robot.subsystems.hopper.Hopper.Hopper;
import frc.robot.subsystems.hopper.Hopper.HopperIO;
import frc.robot.subsystems.hopper.Hopper.HopperIOSim;
import frc.robot.subsystems.hopper.Hopper.HopperIOTalonFX;
import frc.robot.subsystems.hopper.HopperController.HopperControllerState;
import frc.robot.subsystems.rgb.RGB;
import frc.robot.subsystems.rgb.RGBIO;
import frc.robot.subsystems.rgb.RGBIOAddressableLED;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.GyroIOSim;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOTalonFXReal;
import frc.robot.subsystems.swerve.ModuleIOTalonFXSim;
import frc.robot.subsystems.swerve.DriveConstants.ApproachPose;
import frc.robot.subsystems.swerve.controllers.heading.TeleopHeadingController;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonvision;
import frc.robot.subsystems.vision.VisionIOPhotonvisionSim;
import frc.robot.utility.ElasticSetpoints;
import frc.robot.subsystems.shooter.shooter_hood.*;
import frc.robot.subsystems.shooter.shooter_omniwheel.ShooterOmniwheel;
import frc.robot.subsystems.shooter.shooter_omniwheel.ShooterOmniwheelIO;
import frc.robot.subsystems.shooter.shooter_omniwheel.ShooterOmniwheelIOSim;
import frc.robot.subsystems.shooter.shooter_omniwheel.ShooterOmniwheelIOTalonFX;
import frc.robot.subsystems.shooter.ShooterController;
import frc.robot.subsystems.shooter.ShooterController.ShooterState;
import frc.robot.subsystems.shooter.shooter_accelerator.ShooterAccelerator;
import frc.robot.subsystems.shooter.shooter_accelerator.ShooterAcceleratorIO;
import frc.robot.subsystems.shooter.shooter_accelerator.ShooterAcceleratorIOSim;
import frc.robot.subsystems.shooter.shooter_accelerator.ShooterAcceleratorIOTalonFX;
import frc.robot.subsystems.shooter.shooter_flywheel.*;
import frc.robot.lib.generic_subsystems.superstructure.*;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure.ControlMode;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // DO NOT DELETE -- this actually does something important
  private RobotState robotState = RobotState.getInstance();

  private ElasticSetpoints elasticSetpoints = ElasticSetpoints.getInstance();

  private ElasticUpdater matchTimerUpdater = new ElasticUpdater();

  // private SendableChooser<Command> autoChooser;
  private LoggedDashboardChooser<Command> autoChooser;

  private final CommandXboxController driverA = new CommandXboxController(0);
  private final CommandXboxController driverB = new CommandXboxController(1);

  private Drive swerve;
  private Vision vision;
  private RGB rgb;
  private CANWatchdog canWatchdog;
  private IntakePivot intakePivot;
  private IntakeRollers intakeRollers;
  private IntakeController intakeController;
  private Hopper hopper;
  private HopperController hopperController;
  private ShooterFlywheel shooterFlywheels;
  private ShooterHood shooterHood;
  private ShooterController shooterController;
  private ShooterOmniwheel shooterOmniwheel;
  private ShooterAccelerator shooterAccelerator;
  private ClimbClawPivot climbClawPivot;
  private ClimbDeployPivot climbDeployPivot;
  private ClimbController climbController;

  public RobotContainer() {

    if (Constants.getRobotMode() != Mode.REPLAY) {
      switch (Constants.getRobotType()) {
        case COMP -> {
          swerve =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[0]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[1]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[2]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[3]));
          intakePivot = new IntakePivot(new IntakePivotIOTalonFX());
          intakeRollers = new IntakeRollers(new IntakeRollersIOTalonFX());
          //   vision = new Vision(new VisionIOPhotonvision(4), new VisionIOPhotonvision(5));
          rgb = new RGB(new RGBIOAddressableLED());
          // rgb = new RGB(new RGBIOCANdle());
          // canWatchdog = new CANWatchdog(new CANWatchdogIOComp(), rgb);
          shooterFlywheels =
            new ShooterFlywheel(new ShooterFlywheelIOTalonFX());
          shooterHood =
            new ShooterHood(new ShooterHoodIOTalonFX());
          shooterOmniwheel = 
            new ShooterOmniwheel(new ShooterOmniwheelIOTalonFX());
          shooterAccelerator = 
            new ShooterAccelerator(new ShooterAcceleratorIOTalonFX());
          // intakePivot = new IntakePivot(new IntakePivotIOTalonFX());
          // intakeRollers = new IntakeRollers(new IntakeRollersIOTalonFX());
          hopper = new Hopper(new HopperIOTalonFX());
        }
        case VISION -> {
          swerve =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[0]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[1]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[2]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[3]));
          vision = new Vision(new VisionIOPhotonvision("arducam-4", 0), new VisionIOPhotonvision("arducam-5", 1));
        }
        case ALPHA -> {
          swerve =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[0]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[1]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[2]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[3]));
        }
        case SIM -> {
          SwerveDriveSimulation driveSimulation = RobotSimState.getInstance().getDriveSimulation();
          swerve =
              new Drive(
                  new GyroIOSim(driveSimulation.getGyroSimulation()),
                  new ModuleIOTalonFXSim(
                      DriveConstants.MODULE_CONFIGS[0], driveSimulation.getModules()[0]),
                  new ModuleIOTalonFXSim(
                      DriveConstants.MODULE_CONFIGS[1], driveSimulation.getModules()[1]),
                  new ModuleIOTalonFXSim(
                      DriveConstants.MODULE_CONFIGS[2], driveSimulation.getModules()[2]),
                  new ModuleIOTalonFXSim(
                      DriveConstants.MODULE_CONFIGS[3], driveSimulation.getModules()[3]));
          vision =
              new Vision(
                  new VisionIOPhotonvisionSim("arducam-3",3, driveSimulation::getSimulatedDriveTrainPose));
                  new VisionIOPhotonvisionSim("arducam-4", 4, driveSimulation::getSimulatedDriveTrainPose);

          // INTAKE
          intakePivot = new IntakePivot(new IntakePivotIOSim());
          intakeRollers = new IntakeRollers(new IntakeRollersIOSim());

          hopper = new Hopper(new HopperIOSim());

          shooterFlywheels =
            new ShooterFlywheel(new ShooterFlywheelIOSim());
          shooterHood =
            new ShooterHood(new ShooterHoodIOSim());
          shooterOmniwheel = 
            new ShooterOmniwheel(new ShooterOmniwheelIOSim());
          shooterAccelerator = 
            new ShooterAccelerator(new ShooterAcceleratorIOSim());

          climbClawPivot = new ClimbClawPivot(new ClimbClawPivotIOSim());
          climbDeployPivot = new ClimbDeployPivot(new ClimbDeployPivotIOSim());

          SimulatedArena.getInstance().clearGamePieces(); // rebuilt fueld sim is currently cooked so we just sim the shots
        }
      }
    }

    if (swerve == null) {
      swerve =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }
    if (vision == null) {
      vision = new Vision(new VisionIO() {}, new VisionIO() {});
    }

    if (canWatchdog == null) {
      canWatchdog = new CANWatchdog(new CANWatchdogIO() {}, rgb);
    }

    if (rgb == null) {
      rgb = new RGB(new RGBIO() {});
    }

    // INTAKE
    if( intakePivot == null) {
      intakePivot = new IntakePivot( new IntakePivotIO() {});
    }
    if( intakeRollers == null) {
      intakeRollers = new IntakeRollers( new IntakeRollersIO() {});
    }
    intakeController = new IntakeController(intakePivot, intakeRollers);

    if (hopper == null) {
      hopper = new Hopper(new HopperIO() {});
    }
    hopperController = new HopperController(hopper);


    if (shooterFlywheels == null) {
      shooterFlywheels = new ShooterFlywheel(new ShooterFlywheelIO() {});
    }

    if (shooterHood == null) {
      shooterHood = new ShooterHood(new ShooterHoodIO() {});
    }

    if (shooterOmniwheel == null) {
      shooterOmniwheel = new ShooterOmniwheel(new ShooterOmniwheelIO() {});
    }

    if (shooterAccelerator == null) {
      shooterAccelerator = new ShooterAccelerator(new ShooterAcceleratorIO() {});
    }

    shooterController = new ShooterController(shooterFlywheels, shooterHood, shooterOmniwheel, shooterAccelerator);


    // init climb
    if( climbClawPivot == null) {
      climbClawPivot = new ClimbClawPivot( new ClimbClawPivotIO() {});
    }
    if( climbDeployPivot == null) {
      climbDeployPivot = new ClimbDeployPivot( new ClimbDeployPivotIO() {});
    }
    climbController = new ClimbController(climbClawPivot, climbDeployPivot);

    // init shooter with testing values
    RobotState.getInstance().initializeShootingAnglePredictor(
      () -> ChassisSpeeds.fromRobotRelativeSpeeds(swerve.getRobotSpeeds(), RobotState.getInstance().getEstimatedPose().getRotation()), 
      () -> shooterController.getCurrentVelocity(),
      () -> ShooterHoodConstants.BASE_TO_SHOOTER_HOOD_TRANSFORM, Units.Degrees.of(-90)); 

    nameCommands();
    configureAutos();
    configureBindings();
  }

  public void containerMatchStarting() {
    // runs when match starts
    canWatchdog.matchStarting();
  }

  /** Use this method to define the named commands for all of the autos */
  private void nameCommands() {
    // Register Command Names in this method
  }

  private void configureBindings() {
    // -----Driver Controls-----
    swerve.setDefaultCommand(
        swerve
            .run(
                () -> {
                  swerve.driveTeleopController(
                      -driverA.getLeftY(),
                      -driverA.getLeftX(),
                      driverA.getLeftTriggerAxis() - driverA.getRightTriggerAxis(),
                      DriveConstants.DRIVE_CONFIG.maxLinearAcceleration());
                    if (Math.abs(driverA.getLeftTriggerAxis()) > 0.1
                    || Math.abs(driverA.getRightTriggerAxis()) > 0.1) {
                      swerve.clearHeadingControl();
                    }
                })
            .withName("Drive Teleop"));

    configureDriverAButtons();
    configureDriverBButtons();

    //Use pov down and left for testing buttons please!! (Drivers get annoyed when we use other buttons)

    driverA.a().onTrue(new InstantCommand(() -> {
      // Only shoot in simulation
      if (Constants.getRobotType() == Constants.RobotType.SIM) {

        Transform3d shooterPose = ShooterHoodConstants.BASE_TO_SHOOTER_HOOD_TRANSFORM.plus(new Transform3d(
          new Translation3d(),
          new Rotation3d(0, 0, Math.PI/2)
        )); // rotation because of how the modeled shooter was in sim litterally just that i fear

        // Angle shooterAngle = Units.Rotations.of(.25).minus(Units.Rotations.of(shooterHood.getPosition()));
        Angle shooterAngle = RobotState.getInstance().calculateTargetShootingState().shooterAngle();
        LinearVelocity launchVelocity = shooterController.getCurrentVelocity(); 

        // Shoot the fuel using the calculated parameters - velocity must match calculation!
        RobotSimState.getInstance().shootFuel(shooterAngle, shooterPose, launchVelocity);
      }
      })
    );
  }

  private void configureDriverAButtons() {
    driverA.start().onTrue(swerve.zeroGyroCommand());
    driverA.x().onTrue(new InstantCommand(() -> swerve.smartZeroGyro()));
    driverA.b().onTrue(climbController.setTargetStateCommand(ClimbState.STOW)
      .andThen(intakeController.setTargetStateCommand(IntakeState.INTAKE)));
    driverA.y().onTrue(intakeController.setTargetStateCommand(IntakeState.STOW));
    driverA.a().onTrue(shooterController.setTargetStateCommand(ShooterState.SHOOT));
    driverA.a().onFalse(shooterController.setTargetStateCommand(ShooterState.IDLE));
    driverA.povUp().whileTrue(new RunCommand(() -> swerve.setDefenseMode(), swerve));
    driverA.povRight().whileTrue(new InstantCommand(() -> swerve.setTargetHeading(new Rotation2d(0)))
      .andThen(shooterController.setTargetStateCommand(ShooterState.SHOOT)));
    driverA.rightBumper().whileTrue(new AlignToPoseCommand(swerve, () -> RobotState.getInstance().getShootingPose(), true));
    driverA.leftBumper().whileTrue( // automatically go to the right orientation to shoot
      new RunCommand(() -> {
          swerve.setTargetHeading(RobotState.getInstance().calculateTargetShootingState().drivebaseYaw().plus(new Rotation2d(Math.toRadians(90))));
      })
    );
  }
  private void configureDriverBButtons() {
    driverB.leftBumper().onTrue(intakeController.setTargetStateCommand(IntakeState.REVERSE));
    driverB.leftBumper().onFalse(intakeController.setTargetStateCommand(IntakeState.IDLE));
    // TODO: the code below all has something to do with climb, which hasn't been merged into dev, so they're commented for now

    driverB.x().onTrue(shooterController.setStoppedCommand(true)
      .alongWith(intakeController.setStoppedCommand(true))
      .alongWith(new InstantCommand(() -> climbController.setStopped(true))));
    driverB.a().onTrue(intakeController.setTargetStateCommand(IntakeState.STOW)
      .alongWith(climbController.setTargetStateCommand(ClimbState.STOW)));
    driverB.b().onTrue(intakeController.setTargetStateCommand(IntakeState.STOW).andThen(climbController.setTargetStateCommand(ClimbState.DEPLOY)));
    driverB.y().onTrue(intakeController.setTargetStateCommand(IntakeState.STOW).andThen(climbController.setTargetStateCommand(ClimbState.L3)));
    driverB.rightBumper().onTrue(climbController.setTargetStateCommand(ClimbState.STOW)
      .andThen(intakeController.setTargetStateCommand(IntakeState.INTAKE))); 
    
  }

  private void configureAutos() {
    RobotConfig robotConfig;
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      robotConfig = null;
    }

    var passRobotConfig = robotConfig; // workaround TODO: is it necessary?

    AutoBuilder.configure(
        () -> RobotState.getInstance().getEstimatedPose(),
        (pose) -> RobotState.getInstance().resetPose(pose),
        () -> swerve.getRobotSpeeds(),
        (speeds) -> {
          swerve.setTrajectorySpeeds(speeds);
        },
        DriveConstants.HOLONOMIC_DRIVE_CONTROLLER,
        passRobotConfig,
        () -> RobotState.isAllianceRed(),
        swerve);

    autoChooser =
        new LoggedDashboardChooser<Command>("Auto Chooser", AutoBuilder.buildAutoChooser());
    VisionTuningCommands.addTuningCommandsToAutoChooser(vision, autoChooser);
    SmartDashboard.putData("Auto Chooser", autoChooser.getSendableChooser());
  }

  public Command getAutoCommand() {
    return autoChooser.get(); // HACK: Replace once we get auto logging
  }

  // runs when auto starts
  public void autoInit() {
    // Smart zero the robot
    CommandScheduler.getInstance().schedule(new InstantCommand(() -> swerve.smartZeroGyro()));
  }

  // runs when teleop starts
  public void teleopInit() {
    CommandScheduler.getInstance().schedule(new VibrateHIDCommand(driverB.getHID(), 5, .5));

    // vibrate controller at 30 seconds left
    CommandScheduler.getInstance()
        .schedule(new WaitCommand(105).andThen(new VibrateHIDCommand(driverB.getHID(), 3, 0.4)));
  }

  /** Ran when periodic disabled */
  public void updateDashboardStatus() {
    // TODO: Define all of the dashboard outputs here
    SmartDashboard.putString("Current Auto", autoChooser.get().getName());
  }

  public static double doubleToDegrees(double angle) {
    return (angle % 360 + 360) % 360;
  }
  
  public static double relativeAngularDifference(double currentAngle, double newAngle) {
    return (doubleToDegrees(newAngle - currentAngle) + 180) % 360 - 180;
  }

  /** Ran every 20 milliseconds */
  public void updateSimulation() {
    if (Constants.getRobotMode() != Constants.Mode.SIM) return;

    Logger.recordOutput("Testing/BlankPose3d", new Pose3d());

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", RobotSimState.getInstance().getDriveSimulation().getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    
    Logger.recordOutput("FieldSimulation/RobotFuel", RobotSimState.getInstance().getIntakeGamePieces());

    // Update the shooting logic with the correct rollers
    RobotSimState.getInstance().setShooterRunning(shooterFlywheels.getCurrentVelocity().in(MetersPerSecond) > 1.0 && shooterAccelerator.getCurrentVelocity().in(RotationsPerSecond) > 1.0 && shooterOmniwheel.getCurrentVelocity().in(RotationsPerSecond) > 1.0, 10.0, Units.Rotations.of(.25).minus(Units.Rotations.of(shooterHood.getPosition())), ShooterHoodConstants.BASE_TO_SHOOTER_HOOD_TRANSFORM.plus(new Transform3d(
          new Translation3d(),
          new Rotation3d(0, 0, Math.PI/2)
        )), shooterFlywheels.getCurrentVelocity());
    
    // Handle automatic shooter firing
    RobotSimState.getInstance().periodicShooter();
  }
}
