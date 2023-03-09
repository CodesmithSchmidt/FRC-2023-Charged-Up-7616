package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class GotoPlatform extends CommandBase {
   // LIST OF WAYPOINTS TO GET TO THE PLATFORM
   private Translation2d finalTrajectory;
   private Swerve swerve;

   public GotoPlatform(Swerve swerve_in) {
    // POPULATE TRAJECTORY WAYPOINTS HERE. LOOK AT THE FIELD MAP...
    // TOOK A LOOK. I THINK WE CAN GET AWAY WITH JUST GOING FORWARD
    swerve = swerve_in;

   // MAY NEED SOME TWEAKING IF 1 != 1 METRE
    finalTrajectory = new Translation2d(1, 0);
   } 

   public void initialize() {
    // CAN PROBABLY ASSIGN HERE TOO
   }

   public void execute() {
    // FIGURE OUT HOW TO PASS THE TRANSLATION TO THE SWERVE
    swerve.drive(finalTrajectory, 0, false, false);
   }

   public void end() {
    // NOT IMPLEMENTED
   }
}
