package frc.robot;

import java.util.List;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.*;
import org.ironmaple.utils.FieldMirroringUtils;
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
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.swerve.DriveConstants;

public class RobotSimState {
   private RobotSimState(){
          driveSimulation =
              new SwerveDriveSimulation(
                  DriveConstants.mapleSimConfig, RobotState.getInstance().getEstimatedPose());
          SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
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
   public void shootFuel(Angle shooterAngle, LinearVelocity launchVelocity){
    shootFuel(getRobotPose3d().plus(new Transform3d(0, 0, 1, new Rotation3d(0,shooterAngle.in(Units.Radians),0))), launchVelocity);
   }

   public void shootFuel(Pose3d shooterEndpointPosition3d, LinearVelocity launchVelocity){
    RebuiltFuelOnFly flyingFuel = new RebuiltFuelOnFly(
        shooterEndpointPosition3d.getTranslation().toTranslation2d(), // position of the chassis where t
        new Translation2d(0, 0),
        getChassisSpeedsFieldRelative(),
        new Rotation2d(shooterEndpointPosition3d.getRotation().getZ()), // the yaw rotation of the shooter
        Units.Meters.of(shooterEndpointPosition3d.getZ()), // height of shot
        launchVelocity, // launch velocity
        Units.Radians.of(shooterEndpointPosition3d.getRotation().getY()) // gets the pitch of the shooter endpoint position -- for shooting angle
    );

    flyingFuel.withTargetPosition(() -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? DriveConstants.BLUE_HUB_ORIGIN : DriveConstants.RED_HUB_ORIGIN).withTargetTolerance(
        new Translation3d(.2,.2,.2) // just an arbitrary tolerance
    );

    // show trajectory
    flyingFuel.withProjectileTrajectoryDisplayCallBack(
        (pose3ds) -> Logger.recordOutput("RobotSimState/FuelSuccessfulShot", pose3ds.toArray(Pose3d[]::new)), // sucess
        (pose3ds) -> Logger.recordOutput("RobotSimState/FuelUnsuccessfulShot", pose3ds.toArray(Pose3d[]::new)) // unsucess
    );

    SimulatedArena.getInstance().addGamePieceProjectile(flyingFuel);
   }
}
