// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Claw;

public class ClawGrab extends CommandBase {
  /** Creates a new ClawGrab. */
private final Claw claw;
//private final boolean open;
//private final boolean cone;
private final double speed;


  public ClawGrab(Claw claw, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.claw = claw;
    this.speed = speed;
    addRequirements(claw);
  }

  /*// Called when the command is initially scheduled.
  @Override
  public void initialize() {}
*/
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    claw.setPosition( speed);
    System.out.println("GRABBIT");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.setPosition(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
