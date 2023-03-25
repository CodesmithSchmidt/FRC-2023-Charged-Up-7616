// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

  /* One motor (Talon FX):
 *    1) Claw to open/close
 *        -Starting position (open)
 *        -Cube pickup (small move)
 *        -Cone pickup (big move)
 *        -Output sensor info to smartdashboard and find appropriate angles for each piece to store as constants
 *        -2 Commands
 *          -One button to pick up cube; tap same button to drop cube
 *          -One button to pick up cone; tap same button to drop cone
 */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

//import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  VictorSPX clawMotor = new VictorSPX(Constants.clawMotorID);
 // TalonFX clawMotor = new TalonFX(Constants.clawMotorID);

  public Claw() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   // SmartDashboard.putNumber("Claw encoder value", clawMotor.getSelectedSensorPosition());
  }

  public void setPosition(double speed){
    //TODO: test encoder positions and tune for cone/cube/open
    clawMotor.set(VictorSPXControlMode.PercentOutput, speed);
    /* 
    if(open && cone){
      clawMotor.set(TalonFXControlMode.Position, 400);
    } else if (open && !cone){
      clawMotor.set(TalonFXControlMode.Position, 200);
    } else {
      clawMotor.set(TalonFXControlMode.Position, 0);
    }
    */

  }
}
