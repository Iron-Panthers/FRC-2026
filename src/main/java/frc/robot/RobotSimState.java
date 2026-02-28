package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.swerve.DriveConstants.DRIVE_CONFIG;

import java.util.List;

import org.dyn4j.geometry.Transform;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.*;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.swerve.DriveConstants;

public class RobotSimState {


    public static final int INTAKE_FUEL_CAPACITY = 100;
    public static final int START_FUEL_CAPACITY = 8;

   private RobotSimState(){
        // init the arena
        Arena2026Rebuilt arena = new Arena2026Rebuilt(false);

        arena.setEfficiencyMode(false);
        arena.placeGamePiecesOnField();

        // start the cloock
        arena.setShouldRunClock(true);

        // Add the drive simulation
        driveSimulation =
            new SwerveDriveSimulation(
                DriveConstants.mapleSimConfig, RobotState.getInstance().getEstimatedPose());
        arena.addDriveTrainSimulation(driveSimulation);

        SimulatedArena.overrideInstance(arena);

        // intake
        intakeSimulation = IntakeSimulation.OverTheBumperIntake("Fuel", driveSimulation, Meters.of(DriveConstants.DRIVE_CONFIG.bumperWidthX()), Meters.of(.3), IntakeSimulation.IntakeSide.RIGHT, INTAKE_FUEL_CAPACITY);
        // load the intake initially
        intakeSimulation.setGamePiecesCount(START_FUEL_CAPACITY);

   } 

   // Singleton instance
   private static RobotSimState instance = null;
   public static RobotSimState getInstance(){
    if(Constants.getRobotType() != RobotType.SIM) {
        // idiot proofing
        System.out.println("WARNING: YOU ARE TRYING TO ACCESS ROBOT SIM STATE FROM AN ACTUAL ROBOT -- THIS IS A CODE ERROR");
    }
    if(instance == null) instance = new RobotSimState();
    return instance;
   }


   // Drive simulation
   private SwerveDriveSimulation driveSimulation;
   public SwerveDriveSimulation getDriveSimulation(){
    return driveSimulation;
   }

   // Get attributes of physical drivebase
   public Pose2d getRobotPose2d(){
    return driveSimulation.getSimulatedDriveTrainPose();
   }

   public Pose3d getRobotPose3d(){
    Pose2d robotPose2d = driveSimulation.getSimulatedDriveTrainPose();
    return new Pose3d(new Translation3d(robotPose2d.getX(), robotPose2d.getY(), 0.0), new Rotation3d(0,0,robotPose2d.getRotation().getRadians()));
   }

   public ChassisSpeeds getChassisSpeedsFieldRelative(){
    return driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();
   }

   // Shooting utilities
   public void shootFuel(Angle launchAngle, Transform3d shooterTransform3d, LinearVelocity launchVelocity){
    Transform3d shooterEndpointPosition3d = new Transform3d(
        shooterTransform3d.getTranslation(),
        new Rotation3d(0, launchAngle.in(Units.Radians), shooterTransform3d.getRotation().getZ())
    );
    shootFuel(getRobotPose3d().plus(shooterEndpointPosition3d), launchVelocity);
   }

   public void shootFuel(Pose3d shooterEndpointPosition3d, LinearVelocity launchVelocity){
    if(!intakeSimulation.obtainGamePieceFromIntake()) return; // remove fuel from intake when shooting


    // Record start time to measure flight duration
    final double startTime = Timer.getFPGATimestamp();
    
    RebuiltFuelOnFly flyingFuel = new RebuiltFuelOnFly(
        shooterEndpointPosition3d.getTranslation().toTranslation2d(), // position of the chassis where t
        new Translation2d(0, 0),
        getChassisSpeedsFieldRelative(),
        new Rotation2d(shooterEndpointPosition3d.getRotation().getZ()), // the yaw rotation of the shooter
        Units.Meters.of(shooterEndpointPosition3d.getZ()), // height of shot
        launchVelocity, // launch velocity
        Units.Radians.of(shooterEndpointPosition3d.getRotation().getY()) // gets the pitch of the shooter endpoint position -- for shooting angle
    );

    flyingFuel.withTargetPosition(() -> RobotState.isAllianceRed() ? DriveConstants.RED_HUB_ORIGIN : DriveConstants.BLUE_HUB_ORIGIN)
        .withTargetTolerance(new Translation3d(.5,.5,.2)) // just an arbitrary tolerance
        .withHitTargetCallBack(() -> {
            double endTime = Timer.getFPGATimestamp();
            double flightTime = endTime - startTime;
            Logger.recordOutput("RobotSimState/LastFlightTimeSeconds", flightTime);
        });


    // Show trajectory and record flight time when target is hit
    flyingFuel.withProjectileTrajectoryDisplayCallBack(
        (pose3ds) -> {
            // Success callback - ball hit target
            Logger.recordOutput("RobotSimState/FuelSuccessfulShot", pose3ds.toArray(Pose3d[]::new));
        },
        (pose3ds) -> {
            // Failure callback - ball missed target
            Logger.recordOutput("RobotSimState/FuelUnsuccessfulShot", pose3ds.toArray(Pose3d[]::new));
        }
    );

    SimulatedArena.getInstance().addGamePieceProjectile(flyingFuel);
   }


   // Intake simulation
   private final IntakeSimulation intakeSimulation;

   public void setIntakeState(boolean extended){
    if(extended){
        intakeSimulation.startIntake();
    }
    else{
        intakeSimulation.stopIntake();
    }
   }

   public Pose3d[] getIntakeGamePieces(){
    int gamePieceAmount = intakeSimulation.getGamePiecesAmount();
    Pose3d[] gamePiecePoses = new Pose3d[gamePieceAmount];
    double spacing = Units.Inches.of(5.91).in(Units.Meters); // arbitrary spacing between game pieces in the intake
    for(int i = 0; i < gamePieceAmount; i++){
        gamePiecePoses[i] = new Pose3d(new Translation3d(driveSimulation.getSimulatedDriveTrainPose().getX(), driveSimulation.getSimulatedDriveTrainPose().getY(), i * spacing), new Rotation3d());
    }
    return gamePiecePoses;
   }

   // Automatic shooter state tracking
   private boolean isShooterRunning = false;
   private double lastShootTime = 0.0;
   private double shootIntervalSeconds = 0.0;

   /**
    * Tells the RobotSimState that the shooter is currently running and should shoot fuel automatically.
    * @param shotsPerSecond The rate at which to shoot fuel (e.g., 2.0 for 2 shots per second)
    * @param shooterAngle The angle at which to shoot
    * @param shooterTransform3d The 3D transform of the shooter relative to the robot
    * @param launchVelocity The velocity at which to launch the fuel
    */
   public void setShooterRunning(boolean running, double shotsPerSecond, Angle shooterAngle, Transform3d shooterTransform3d, LinearVelocity launchVelocity) {
       if (running && !isShooterRunning) {
           // Starting the shooter
           isShooterRunning = true;
           shootIntervalSeconds = 1.0 / shotsPerSecond;
           lastShootTime = Timer.getFPGATimestamp();
       } else if (!running) {
           // Stopping the shooter
           isShooterRunning = false;
       }
       
       // Store the shooting parameters for use in periodic
       this.currentShooterAngle = shooterAngle;
       this.currentShooterTransform = shooterTransform3d;
       this.currentLaunchVelocity = launchVelocity;
   }

   // Store current shooting parameters
   private Angle currentShooterAngle = Units.Radians.of(0);
   private Transform3d currentShooterTransform = new Transform3d();
   private LinearVelocity currentLaunchVelocity = MetersPerSecond.of(0);

   /**
    * Should be called periodically (e.g., in Robot.java's simulationPeriodic).
    * Handles automatic shooting when the shooter is running.
    */
   public void periodicShooter() {
       if (!isShooterRunning) {
           return;
       }

       double currentTime = Timer.getFPGATimestamp();
       if (currentTime - lastShootTime >= shootIntervalSeconds) {
           // Time to shoot another ball
           shootFuel(currentShooterAngle, currentShooterTransform, currentLaunchVelocity);
           lastShootTime = currentTime;
           Logger.recordOutput("RobotSimState/AutoShooterActive", true);
       }
   }

   @AutoLogOutput(key = "RobotSimState/ShooterRunning")
   public boolean isShooterRunning() {
       return isShooterRunning;
   }
}
