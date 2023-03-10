package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;

public class GotoPlatform extends CommandBase {
   // LIST OF WAYPOINTS TO GET TO THE PLATFORM
/* 
   TrajectoryConfig config =
   new TrajectoryConfig(
           Constants.AutoConstants.kMaxSpeedMetersPerSecond,
           Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
       .setKinematics(Constants.Swerve.swerveKinematics);
*/


   private Translation2d finalTrajectory;
   private Swerve swerve;

   public GotoPlatform(Swerve swerve_in) {
    // POPULATE TRAJECTORY WAYPOINTS HERE. LOOK AT THE FIELD MAP...
    // TOOK A LOOK. I THINK WE CAN GET AWAY WITH JUST GOING FORWARD
    swerve = swerve_in;

   // MAY NEED SOME TWEAKING IF 1 != 1 METRE
    finalTrajectory = new Translation2d(3, 0);
   } 

   public void initialize() {
    // CAN PROBABLY ASSIGN HERE TOO
   }

   public void execute() {
    // FIGURE OUT HOW TO PASS THE TRANSLATION TO THE SWERVE
    swerve.drive(finalTrajectory, 0, true, false);
   }

   public void end() {
    // NOT IMPLEMENTED
   }
}
