package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
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
   SwerveDriveSimulation driveSimulation;
   public SwerveDriveSimulation getDriveSimulation(){
    return driveSimulation;
   }

   // Get attributes of physical drivebase
   public Pose2d getRobotPose3d(){
    return driveSimulation.getSimulatedDriveTrainPose();
   }

   public ChassisSpeeds getChassisSpeedsFieldRelative(){
    return driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();
   }

   // Shooting utilities
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
   }
}
