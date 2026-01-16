package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

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
}
