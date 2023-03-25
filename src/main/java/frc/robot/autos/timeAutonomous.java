// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.Swerve;

public class timeAutonomous extends CommandBase {
  /** Creates a new timeAutonomous. */

  private Swerve a_Swerve = new Swerve();
  private Timer a_timer = new Timer();

  public timeAutonomous(Swerve a_Swerve) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.a_Swerve = a_Swerve;

    addRequirements(a_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    a_timer.reset();
    a_timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double time = a_timer.get();

    //For 1 second drive forward at 25% speed
    while(time < 1){
      a_Swerve.drive(new Translation2d(.25,0), 0, true, false);
    }
    //While gyro is not level, drive the correct direction to level
    while(Math.abs(a_Swerve.gyro.getPitch()) > 10){
      if(a_Swerve.gyro.getPitch() > 0){
        a_Swerve.drive(new Translation2d(.1,0),0,true,false);
      }
      else {
        a_Swerve.drive(new Translation2d(-.1,0),0,true,false);
      }
    }
    
    a_Swerve.drive(new Translation2d(0,0),0,true,false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    a_Swerve.drive(new Translation2d(0,0),0,true,false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    a_Swerve.drive(new Translation2d(0,0),0,true,false);
    return false;
  }
}
