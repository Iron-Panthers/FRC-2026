// UNUSED, THIS WHOLE THING WAS STUPID AND ENGUERRAN TOLD ME A MUCH BETTER WAY TO DO THIS SO UH JUST IGNORE THIS
// i could just delete this file but i am too lazy to.

/*
package frc.robot.subsystems.swerve.controllers.heading;

import java.util.function.Supplier;

import org.littletonrobotics.junction.ConsoleSource.RoboRIO;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.DriveConstants.HeadingControllerConstants;
import edu.wpi.first.math.geometry.Translation2d;

public class HeadingController extends BaseHeadingController {
    public HeadingController (
        Supplier<Rotation2d> headingSupplier,
        Rotation2d targetHeading,
        HeadingControllerConstants headingControllerConstants)
    {super(headingSupplier, targetHeading, headingControllerConstants);}

    public double update() {
        Translation2d Velocity = RobotState.getInstance().getVelocity();
        
        setTargetHeading(Velocity.getAngle());
        return super.update();
    }
}*/