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
import frc.robot.commands.PathPlannerApproachPoseCommand;
import frc.robot.RobotState.TargetShootingState;
import frc.robot.commands.VibrateHIDCommand;
import frc.robot.commands.VisionTuningCommands;
import frc.robot.subsystems.canWatchdog.CANWatchdog;
import frc.robot.subsystems.canWatchdog.CANWatchdogIO;
import frc.robot.subsystems.canWatchdog.CANWatchdogIOComp;
import frc.robot.subsystems.intake.IntakeController;
import frc.robot.subsystems.intake.IntakeController.IntakeControllerState;
import frc.robot.subsystems.intake.intakePivot.IntakePivot;
import frc.robot.subsystems.intake.intakePivot.IntakePivotIO;
import frc.robot.subsystems.intake.intakePivot.IntakePivotIOSim;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollers;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollersIO;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollersIOSim;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollers.IntakeRollersTarget;
import frc.robot.subsystems.elastic_updater.ElasticUpdater;
import frc.robot.subsystems.rgb.RGB;
import frc.robot.subsystems.rgb.RGBIO;
import frc.robot.subsystems.rgb.RGBIOCANdle;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.GyroIOSim;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOTalonFXReal;
import frc.robot.subsystems.swerve.ModuleIOTalonFXSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonvision;
import frc.robot.subsystems.vision.VisionIOPhotonvisionSim;
import frc.robot.utility.ElasticSetpoints;
import frc.robot.subsystems.shooter.shooter_hood.*;
import frc.robot.subsystems.shooter.ShooterController;
import frc.robot.subsystems.shooter.shooter_flywheel.*;
import frc.robot.subsystems.shooter.shooter_accelerator_bottom.*;
import frc.robot.subsystems.shooter.shooter_accelerator_top.*;

import static edu.wpi.first.units.Units.MetersPerSecond;

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
  private ShooterFlywheel shooterFlywheels;
  private ShooterHood shooterHood;
  private ShooterController shooterController;
  private ShooterAcceleratorBottom shooterAcceleratorBottom;
  private ShooterAcceleratorTop shooterAcceleratorTop;

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
          //   vision = new Vision(new VisionIOPhotonvision(4), new VisionIOPhotonvision(5));
          // rgb = new RGB(new RGBIOCANdle());
          // canWatchdog = new CANWatchdog(new CANWatchdogIOComp(), rgb);
          // shooterFlywheels =
          //   new ShooterFlywheel(new ShooterFlywheelIOTalonFX());
          // shooterHood =
          //   new ShooterHood(new ShooterHoodIOTalonFX());
          // shooterAcceleratorBottom = 
          //   new ShooterAcceleratorBottom(new ShooterAcceleratorBottomIOTalonFX());
          // shooterAcceleratorTop = 
          //   new ShooterAcceleratorTop(new ShooterAcceleratorTopIOTalonFX());
          
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
                  new VisionIOPhotonvisionSim("arducam-4",4, driveSimulation::getSimulatedDriveTrainPose),
                  new VisionIOPhotonvisionSim("arducam-5", 5, driveSimulation::getSimulatedDriveTrainPose));

          // INTAKE
          intakePivot = new IntakePivot(new IntakePivotIOSim());
          intakeRollers = new IntakeRollers(new IntakeRollersIOSim());

          shooterFlywheels =
            new ShooterFlywheel(new ShooterFlywheelIOSim());
          shooterHood =
            new ShooterHood(new ShooterHoodIOSim());
          shooterAcceleratorBottom = 
            new ShooterAcceleratorBottom(new ShooterAcceleratorBottomIOSim());
          shooterAcceleratorTop = 
            new ShooterAcceleratorTop(new ShooterAcceleratorTopIOSim());

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

    if (shooterFlywheels == null) {
      shooterFlywheels = new ShooterFlywheel(new ShooterFlywheelIO() {});
    }

    if (shooterHood == null) {
      shooterHood = new ShooterHood(new ShooterHoodIO() {});
    }

    if (shooterAcceleratorBottom == null) {
      shooterAcceleratorBottom = new ShooterAcceleratorBottom(new ShooterAcceleratorBottomIO() {});
    }

    if (shooterAcceleratorTop == null) {
      shooterAcceleratorTop = new ShooterAcceleratorTop(new ShooterAcceleratorTopIO() {});
    }

    shooterController = new ShooterController(shooterFlywheels, shooterHood, shooterAcceleratorBottom, shooterAcceleratorTop);

    // init shooter with testing values
    robotState.initializeShootingAnglePredictor(
      () -> swerve.getRobotSpeeds(), // stationary
      () -> MetersPerSecond.of(10), // test shooter velocity: 10 m/s
      () -> new Transform3d(new Translation3d(0, 0, 0.5), new Rotation3d())); // shooter is 0.5m above robot center

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
                })
            .withName("Drive Teleop"));

    driverA.start().onTrue(swerve.zeroGyroCommand());

    driverA.a().onTrue(new InstantCommand(() -> swerve.smartZeroGyro()));
    // driverA.x().onTrue(new PathPlannerApproachPoseCommand(swerve, new Pose2d(2.499, 3.977, new Rotation2d(0)), true));
    
    // driverA.b().onTrue(new InstantCommand(() -> {
    //   RobotSimState.getInstance().shootFuel(Units.Degrees.of(45), MetersPerSecond.of(3));
    // }));

    driverA.povUp().whileTrue(new RunCommand(() -> swerve.setDefenseMode(), swerve));
    driverA.y().onTrue(intakeController.setTargetStateCommand(IntakeControllerState.STOW));
    driverA.b().onTrue(intakeController.setTargetStateCommand(IntakeControllerState.INTAKE));

    // driverA.y().onTrue(new InstantCommand(() -> {
      
    //   // Calculate target shooting state
    //   TargetShootingState targetState = robotState.calculateTargetShootingState();
      
    //   // Only shoot in simulation
    //   if (Constants.getRobotType() == Constants.RobotType.SIM) {
    //     // Get current robot pose and apply the calculated shooter angle and yaw
    //     Pose3d robotPose3d = RobotSimState.getInstance().getRobotPose3d();
        
    //     // Create shooter endpoint position with calculated yaw and shooter angle
    //     // Shooter is 0.5m above robot center
    //     Pose3d shooterPose = new Pose3d(
    //       robotPose3d.getTranslation().plus(new Translation3d(0, 0, 0.5)),
    //       new Rotation3d(
    //         0, // roll
    //         targetState.shooterAngle().in(Units.Radians), // pitch (shooter angle)
    //         targetState.drivebaseYaw().getRadians() // yaw
    //       )
    //     );
        
    //     // Shoot the fuel using the calculated parameters - velocity must match calculation!
    //     RobotSimState.getInstance().shootFuel(shooterPose, MetersPerSecond.of(10));
    //   }
    //   })
    // );
    // driverA.b().onTrue(shooterController.setTargetCommand(ShooterController.ShooterState.SHOOT));
    driverB.a().onTrue(intakeController.setTargetStateCommand(IntakeControllerState.INTAKE));
    driverB.b().onTrue(intakeController.setTargetStateCommand(IntakeControllerState.STOW));

    driverB.x().onTrue(shooterController.setTargetCommand(ShooterController.ShooterState.SHOOT));
    driverB.y().onTrue(shooterController.setTargetCommand(ShooterController.ShooterState.IDLE));
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

    BooleanSupplier isRedAlliance =
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        };

    AutoBuilder.configure(
        () -> RobotState.getInstance().getEstimatedPose(),
        (pose) -> RobotState.getInstance().resetPose(pose),
        () -> swerve.getRobotSpeeds(),
        (speeds) -> {
          swerve.setTrajectorySpeeds(speeds);
        },
        DriveConstants.HOLONOMIC_DRIVE_CONTROLLER,
        passRobotConfig,
        isRedAlliance,
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
  }
}
