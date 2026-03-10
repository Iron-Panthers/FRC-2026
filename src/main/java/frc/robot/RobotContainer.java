// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.EventMarker;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import frc.robot.utility.FuelSim;
import frc.robot.subsystems.shooter.shooter_hood.*;
import frc.robot.subsystems.shooter.shooter_hood.ShooterHood.ShooterHoodTarget;
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
          vision = new Vision(
            new VisionIOPhotonvision("arducam-7", 0),
            new VisionIOPhotonvision("arducam-8", 1));
          // rgb = new RGB(new RGBIOAddressableLED());
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
          // vision = new Vision(new VisionIOPhotonvision("arducam-4", 0), new VisionIOPhotonvision("arducam-5", 1));
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
        }
      }
    }

    // SWERVE
    if (swerve == null) swerve = new Drive( new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});

    // VISION
    if (vision == null) vision = new Vision(new VisionIO() {}, new VisionIO() {});

    // CAN WATCHDOG
    if (canWatchdog == null) canWatchdog = new CANWatchdog(new CANWatchdogIO() {}, rgb);

    // RGB
    if (rgb == null) rgb = new RGB(new RGBIO() {});

    // INTAKE
    if( intakePivot == null) intakePivot = new IntakePivot( new IntakePivotIO() {});
    if( intakeRollers == null) intakeRollers = new IntakeRollers( new IntakeRollersIO() {});
    intakeController = new IntakeController(intakePivot, intakeRollers);

    // HOPPER
    if (hopper == null) hopper = new Hopper(new HopperIO() {});
    hopperController = new HopperController(hopper);

    // SHOOTER
    if (shooterFlywheels == null) shooterFlywheels = new ShooterFlywheel(new ShooterFlywheelIO() {}); 
    if (shooterHood == null) shooterHood = new ShooterHood(new ShooterHoodIO() {});
    if (shooterOmniwheel == null) shooterOmniwheel = new ShooterOmniwheel(new ShooterOmniwheelIO() {});
    if (shooterAccelerator == null) shooterAccelerator = new ShooterAccelerator(new ShooterAcceleratorIO() {});
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
      () -> ShooterHoodConstants.BASE_TO_SHOOTER_HOOD_TRANSFORM, Units.Degrees.of(-180)); 

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

    new EventTrigger("Intake down").onTrue(new InstantCommand(() ->intakeController.setTargetState(IntakeState.INTAKE)));
    new EventTrigger("Spin up shooter").onTrue(new InstantCommand(() -> {shooterController.setTargetState(ShooterState.TOTAL_SPIN_UP); 
      hopperController.setTargetState(HopperControllerState.INTAKE);}));
    NamedCommands.registerCommand("Smart zero", new InstantCommand(() -> swerve.smartZeroGyro()));
    NamedCommands.registerCommand("Intake down", intakeController.setTargetStateCommand(IntakeState.INTAKE).alongWith(hopperController.setTargetStateCommand(HopperControllerState.SLOW)));
    NamedCommands.registerCommand("Intake stow", intakeController.setTargetStateCommand(IntakeState.STOW));
    NamedCommands.registerCommand("Intake mid", new InstantCommand(() -> intakeController.setTargetState(IntakeState.MIDDLE_STOW)));
    NamedCommands.registerCommand("Spin up shooter", shooterController.setTargetStateCommand(ShooterState.TOTAL_SPIN_UP).alongWith(hopperController.setTargetStateCommand(HopperControllerState.INTAKE)));
    NamedCommands.registerCommand("Shoot", shooterController.setTargetStateCommand(ShooterState.SHOOT).alongWith(hopperController.setTargetStateCommand(HopperControllerState.INTAKE)));
    NamedCommands.registerCommand("Stop shooting", shooterController.setTargetStateCommand(ShooterState.IDLE).alongWith(hopperController.setTargetStateCommand(HopperControllerState.IDLE)));
    NamedCommands.registerCommand("Align to shoot", 
      new InstantCommand(() -> swerve.setTargetHeading(RobotState.getInstance().calculateTargetShootingState().drivebaseYaw().plus(new Rotation2d(Math.toRadians(RobotBase.isReal() ? 0 : 180))))
    ).alongWith(
          shooterController.setTargetStateCommand(ShooterState.TOTAL_SPIN_UP)));
    NamedCommands.registerCommand("Shoot full hopper",
      new InstantCommand(() -> swerve.setTargetHeading(RobotState.getInstance().calculateTargetShootingState().drivebaseYaw().plus(new Rotation2d(Math.toRadians(RobotBase.isReal() ? 0 : 180)))))
        .alongWith(
          shooterController.setTargetStateCommand(ShooterState.TOTAL_SPIN_UP))
      .alongWith(new InstantCommand(() -> intakeController.setTargetState(IntakeState.MIDDLE_STOW)))
      .alongWith(new InstantCommand(() -> shooterController.setTargetStateCommand(ShooterState.SHOOT)))
      .alongWith(new WaitCommand(8))
      .andThen(new InstantCommand(() -> intakeController.setTargetStateCommand(IntakeState.INTAKE)))
      .andThen(new InstantCommand(() -> shooterController.setTargetStateCommand(ShooterState.IDLE))));

    NamedCommands.registerCommand("Shoot preloaded hopper",
      new AlignToPoseCommand(swerve, () -> RobotState.getInstance().getShootingPose(), true, true)
        .alongWith(
          shooterController.setTargetStateCommand(ShooterState.TOTAL_SPIN_UP))
      .andThen(new WaitCommand(0.6))
      .andThen(new InstantCommand(() -> shooterController.setTargetStateCommand(ShooterState.SHOOT)))
      .andThen(new WaitCommand(2))
      .andThen(new InstantCommand(() -> intakeController.setTargetStateCommand(IntakeState.INTAKE)))
      .andThen(new InstantCommand(() -> shooterController.setTargetStateCommand(ShooterState.IDLE))));
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

    // LET THIS BE A WARNING TO ALL THOSE WHO WANT TO DO A 'TRIGGER' FOR YOUR LOGIC
    // TODO: WTF IS THIS TRIGGER IT BROKE OUR AUTOS
    // new Trigger(()-> RobotState.getInstance().isUnderTrench()).onTrue(shooterController.setTargetStateCommand(ShooterState.IDLE));

    configureDriverAButtons();
    configureDriverBButtons();
    new Trigger(()-> matchTimerUpdater.getTime() > 0).onTrue(new RunCommand(
      () -> vibrateIntervals()));
    //Use pov down and left for testing buttons please!! (Drivers get annoyed when we use other buttons)

  }

  private void configureDriverAButtons() {
    // ZERO GYRO
    driverA.start().onTrue(swerve.zeroGyroCommand());
    // SMART ZERO GYRO
    driverA.x().onTrue(new InstantCommand(() -> swerve.smartZeroGyro()));
    // INTAKE
    driverA.b().onTrue(climbController.setTargetStateCommand(ClimbState.STOW)
      .andThen(intakeController.setTargetStateCommand(IntakeState.INTAKE_DOWN))
      .andThen(intakeController.setTargetStateCommand(IntakeState.INTAKE))
      .alongWith(shooterController.setTargetStateCommand(ShooterState.IDLE))
      .alongWith(hopperController.setTargetStateCommand(HopperControllerState.SLOW)));
    // STOW ROBOT
    driverA.y().onTrue(intakeController.setTargetStateCommand(IntakeState.MIDDLE_STOW)
      .alongWith(shooterController.setTargetStateCommand(ShooterState.IDLE))
      .alongWith(hopperController.setTargetStateCommand(HopperControllerState.SLOW)));

    // SHOOTING COMMAND
    driverA.a().whileTrue(
      new InstantCommand(()-> {
          shooterController.setTargetState(shooterController.getTargetState() == ShooterState.TOTAL_SPIN_UP
          // && (matchTimerUpdater.isOurHubActive() || matchTimerUpdater.getTimeUntilOurHubShifts() < 2) // time correct // TODO: Test this more so it works
          ? ShooterState.SHOOT
          : ShooterState.TOTAL_SPIN_UP);
      })
      .alongWith(hopperController.setTargetStateCommand(HopperControllerState.INTAKE))
      .alongWith(shooterController.getTargetState() == ShooterState.SHOOT ? 
      // THIS IS FOR MAKING THE INTAKE GO UP AND DOWN WHILE WE ARE SHOOTING
        ((intakeController.setTargetStateCommand(IntakeState.MIDDLE_STOW))
        .alongWith(new WaitCommand(0.05))
        .alongWith(intakeController.setTargetStateCommand(IntakeState.STOW))
        .alongWith(new WaitCommand(0.05)))
        : new InstantCommand()).repeatedly());
      
    driverA.a().onFalse( new InstantCommand (() -> 
    {
      if (shooterController.getTargetState() == ShooterState.SHOOT){
        shooterController.setTargetState(ShooterState.COMPACT_SPIN_UP);
      }
    })
      .alongWith(hopperController.setTargetStateCommand(HopperControllerState.INTAKE)));
    
    // DEFENSE MODE
    driverA.povUp().whileTrue(new RunCommand(() -> swerve.setDefenseMode(), swerve));

    // SHUTTLE
    driverA.povRight().whileTrue(new InstantCommand(() -> swerve.setTargetHeading(new Rotation2d(0)))
      .andThen(shooterController.setTargetStateCommand(ShooterState.SHUTTLE)));

    // ALIGN TO SHOOT
    driverA.rightBumper().whileTrue(new AlignToPoseCommand(swerve, () -> RobotState.getInstance().getShootingPose(), true)
      .alongWith(
        new WaitUntilCommand(() -> RobotState.getInstance().getEstimatedPose().getTranslation().getDistance(RobotState.getInstance().getAlignPose().getTranslation()) < 1)
        .andThen(shooterController.setTargetStateCommand(ShooterState.TOTAL_SPIN_UP))));

    // // ARC ALIGN
    // driverA.leftBumper().whileTrue( // automatically go to the right orientation to shoot
    //   new RunCommand(() -> {
    //       swerve.setTargetHeading(RobotState.getInstance().calculateTargetShootingState().drivebaseYaw().plus(new Rotation2d(Math.toRadians(RobotBase.isReal() ? 0 : 180))));

    //       final Translation3d hubPosition3d = RobotState.isAllianceRed() ? DriveConstants.RED_HUB_ORIGIN : DriveConstants.BLUE_HUB_ORIGIN;
    //       Translation2d toGoal = hubPosition3d.toTranslation2d().minus(RobotState.getInstance().getEstimatedPose().getTranslation());
    //       double distance = toGoal.getNorm();
    //       Logger.recordOutput("Tuning/DistanceTo", distance);
    //   })
    //   .alongWith(shooterController.setAutoAimCommand(true))
    // );

  }
  private void configureDriverBButtons() {
    driverB.leftBumper().onTrue(intakeController.setTargetStateCommand(IntakeState.REVERSE));
    driverB.leftBumper().onFalse(intakeController.setTargetStateCommand(IntakeState.IDLE));
    // TODO: the code below all has something to do with climb, which hasn't been merged into dev, so they're commented for now

    driverB.x().onTrue(shooterController.setStoppedCommand(true)
      .alongWith(intakeController.setStoppedCommand(true))
      .alongWith(new InstantCommand(() -> climbController.setStopped(true)))
      .alongWith(hopperController.setTargetStateCommand(HopperControllerState.IDLE)));
    driverB.a().onTrue(intakeController.setTargetStateCommand(IntakeState.STOW)
      .alongWith(climbController.setTargetStateCommand(ClimbState.STOW)));
    driverB.b().onTrue(intakeController.setTargetStateCommand(IntakeState.STOW)
      .andThen(climbController.setTargetStateCommand(ClimbState.DEPLOY))
        .alongWith(new AlignToPoseCommand(swerve, () -> RobotState.getInstance().getClimbTarget(), false)));
    driverB.y().onTrue(intakeController.setTargetStateCommand(IntakeState.STOW).andThen(climbController.setTargetStateCommand(ClimbState.L3)));
    driverB.rightBumper().onTrue(climbController.setTargetStateCommand(ClimbState.STOW)
      .andThen(intakeController.setTargetStateCommand(IntakeState.INTAKE))); 
    
      driverB.povLeft().onTrue(intakeController.zeroCommand());
      driverB.povDown().onTrue(shooterController.zeroCommand());
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
    return autoChooser.get(); 
  }

  // runs when auto starts
  public void autoInit() {
    // Smart zero the robot
    CommandScheduler.getInstance().schedule(new InstantCommand(() -> swerve.smartZeroGyro()));
  }

  // runs when teleop starts
  public void teleopInit() {
    CommandScheduler.getInstance().schedule(new VibrateHIDCommand(driverB.getHID(), 5, .5));
  }

  /** Ran when periodic disabled */
  public void updateDashboardStatus() {
    // TODO: Define all of the dashboard outputs here
    var selectedAuto = autoChooser.get();
    SmartDashboard.putString("Current Auto", selectedAuto != null ? selectedAuto.getName() : "None");
  }

  public static double doubleToDegrees(double angle) {
    return (angle % 360 + 360) % 360;
  }
  
  public static double relativeAngularDifference(double currentAngle, double newAngle) {
    return (doubleToDegrees(newAngle - currentAngle) + 180) % 360 - 180;
  }

  public void vibrateIntervals() {
  // vibrate controller at 30 seconds left
    if ((int)matchTimerUpdater.getTime() == 30) {
        new VibrateHIDCommand(driverB.getHID(), 1, 0.4)
        .andThen(new VibrateHIDCommand(driverB.getHID(), 0.166, 0))
        .andThen(new VibrateHIDCommand(driverB.getHID(), 1, 0.4));
    } else if ((int)matchTimerUpdater.getTimeUntilOurHubShifts() == 10 && matchTimerUpdater.isOurHubActive() == false) {
        new VibrateHIDCommand(driverB.getHID(), 0.5, 0.4)
        .andThen(new VibrateHIDCommand(driverB.getHID(), 0.166, 0)) //"wait" command
        .andThen(new VibrateHIDCommand(driverB.getHID(), 0.5, 0.4))
        .andThen(new VibrateHIDCommand(driverB.getHID(), 0.166, 0))
        .andThen(new VibrateHIDCommand(driverB.getHID(), 0.5, 0.4));
    } else if ((int)matchTimerUpdater.getTimeUntilOurHubShifts() == 10 && matchTimerUpdater.isOurHubActive() == true) {
        new VibrateHIDCommand(driverB.getHID(), 0.4, 0.4)
        .andThen(new VibrateHIDCommand(driverB.getHID(), 0.166, 0))
        .andThen(new VibrateHIDCommand(driverB.getHID(), 0.4, 0.4));
    }
  }

  /** Ran every 20 milliseconds */
  public void updateSimulation() {
    if (Constants.getRobotMode() != Constants.Mode.SIM) return;

    Logger.recordOutput("Testing/BlankPose3d", new Pose3d());

    SimulatedArena.getInstance().simulationPeriodic();
    RobotSimState.getInstance().getFuelSim().updateSim();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", RobotSimState.getInstance().getDriveSimulation().getSimulatedDriveTrainPose());
    Logger.recordOutput("FieldSimulation/RobotFuel", RobotSimState.getInstance().getIntakeGamePieces());
    Logger.recordOutput("FieldSimulation/FuelCount", RobotSimState.getInstance().getFuelCount());

    // Update the shooting logic with the correct rollers
    RobotSimState.getInstance().setShooterRunning(shooterFlywheels.getCurrentVelocity().in(MetersPerSecond) > 1.0 && shooterAccelerator.getCurrentVelocity().in(RotationsPerSecond) > 1.0 && shooterOmniwheel.getCurrentVelocity().in(RotationsPerSecond) > 1.0, 8.0, Units.Rotations.of(.25).minus(Units.Rotations.of(shooterHood.getPosition())), ShooterHoodConstants.BASE_TO_SHOOTER_HOOD_TRANSFORM.plus(new Transform3d(
          new Translation3d(),
          new Rotation3d(0, 0, Math.PI/2)
        )), shooterFlywheels.getCurrentVelocity());
    
    // Handle automatic shooter firing
    RobotSimState.getInstance().periodicShooter();
  }
}
