// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AngleSet;

public class AngleSetPID extends CommandBase {
  /** Creates a new AngleSetPID. */

  private final AngleSet angleSet;
  private final PIDController pidController;

  public AngleSetPID(AngleSet angleSet, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angleSet = angleSet;
    this.pidController = new PIDController(3, 0, .8);
    pidController.setSetpoint(setpoint);
    addRequirements(angleSet);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pidController.calculate(angleSet.getEncoderMeters());
    angleSet.setAngleMotor(speed);
    System.out.println("AngleControlPID started!");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    angleSet.setAngleMotor(0);
    System.out.println("AngleControlPID ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
