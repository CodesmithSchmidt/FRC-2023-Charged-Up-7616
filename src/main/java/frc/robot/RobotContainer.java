package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick driver2 = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kX.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final AngleSet a_angleSet = new AngleSet();
    private final Claw c_claw = new Claw();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver 1 Buttons */
        /* Start Button = zero gyro
         * X = robot centric driving
         * 
         * 
         * Driver 2 Buttons
         * X elevator in (only while angle is at scoring)
         * B elevator out (only while angle is at scoring)
         * Y angle up (if elevator has triggered lim switch)
         * A angle down (if elevator has triggered lim switch)
         * Triggers for claw?
         need to make it so angleSet commands can only run if elevator is in
         need limit switch
         elevator can only run if angleSet distance < set amount
         

         */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        //TODO: Test all motor polarity for forward/back and adjust speed variable
        new JoystickButton(driver2, XboxController.Button.kB.value).whileTrue(new ElevatorControl( a_angleSet, .75));
        new JoystickButton(driver2, XboxController.Button.kX.value).whileTrue(new ElevatorControl( a_angleSet, -.5));
        new JoystickButton(driver2, XboxController.Button.kY.value).whileTrue(new AngleControlCmd(a_angleSet, .75));
        new JoystickButton(driver2, XboxController.Button.kA.value).whileTrue(new AngleControlCmd(a_angleSet, -.75));
        new JoystickButton(driver2, XboxController.Button.kLeftBumper.value).whileTrue(new ClawGrab(c_claw,  .2));
        new JoystickButton(driver2, XboxController.Button.kRightBumper.value).whileTrue(new ClawGrab(c_claw,  -.2));
        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new timeAutonomous(s_Swerve);

       // return new exampleAuto(s_Swerve);
       //return new getBread(s_Swerve, s_Swerve.gyro );
    }
}
