package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NavXBalance;

public class Balance extends CommandBase{
    private NavXBalance subsystem;
    private float correction;

    public Balance(NavXBalance balanceSys) {
        subsystem = balanceSys;
    }

   public void initalize() {
    subsystem.zeroDisplacement();
   }

   public void execute() {
    while (true) {
        correction = subsystem.checkPosition();
        subsystem.correct(correction);
    }

   }

   public void end() {

   }

   @Override
   public boolean isFinished() {
    return false;
   }
}
