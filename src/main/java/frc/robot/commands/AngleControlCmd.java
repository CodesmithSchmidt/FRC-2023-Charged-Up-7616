// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AngleSet;

public class AngleControlCmd extends CommandBase {
  /** Creates a new ElevatorControlCmd. */
  private final AngleSet angleSet;
  private final double speed;

  public AngleControlCmd(AngleSet angleSet, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angleSet = angleSet;
    this.speed = speed;
    addRequirements(angleSet);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angleSet.setAngleMotor(speed);
    System.out.println("AngleControl started!");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    angleSet.setAngleMotor(0);
    System.out.println("AngleControl ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
