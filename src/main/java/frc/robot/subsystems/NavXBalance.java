package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import edu.wpi.first.wpilibj.SPI;
//import frc.robot.subsystems.Swerve;

public class NavXBalance extends SubsystemBase {
    // CHANGE THIS ACCORDING TO WHAT COMMUNICATION PROTOCOL IS BEING USED FOR THE NAVX
    // LOOKS LIKE IT'S ALREADY SPECIFIED SPI IN Swerve.java
    private AHRS ahrs;
    private Swerve swerve;

    private SwerveModuleState[] correctionSwerveModuleStates;
    private ChassisSpeeds speed;
    private Translation2d centerOfRotation;
    
    public NavXBalance(AHRS ahrs_in, Swerve drive_in, BooleanSupplier supplier) {
        ahrs = ahrs_in;
        swerve = drive_in;
    }
    
    // THRESHOLD VALUE TO TWEAK
    // DEGREES UNTIL ROBOT PERFOMS CORRECTION
    private double threshold = 5;
    // VALUE TO TWEAK FOR CORRECTION SPEED
    private float multiplier = 1;
    
    @Override
    public void periodic() {
        // NOT IMPLEMENTED
    }

    // RESET GYRO
    public void resetGyro() {
        ahrs.reset();
    }

    // RESET DISPLACEMENT
    public void zeroDisplacement() {
        ahrs.resetDisplacement();
    }

    // MEASURES HOW MUCH THE ROBOT IS MOVING
    // OFF THE PLATFORM. WON'T WORK IF THE ROBOT DOESN'T GO STRAIGHT
    // ON THE PLATFORM (X AXIS FORWARD). RETURNS A CORRECTION COEFFICIENT.
    public float checkPosition() {
        if (Math.abs(ahrs.getPitch()) > threshold) {
            return ahrs.getPitch();
        }

        return 0;
    }

   // DRIVES THE MOTORS TO CORRECT
   public void correct(float severity) {
    // SPEED TO MOVE IN X DIRECTION
    speed = new ChassisSpeeds(severity * multiplier, 0, 0);

    correctionSwerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speed, centerOfRotation);
    swerve.setModuleStates(correctionSwerveModuleStates);
   }
}
