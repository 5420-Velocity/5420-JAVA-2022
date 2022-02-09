/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.*;
import frc.robot.Subsystems.*;

/**
 * Add your docs here.
 */
public class RobotContainer {
    private final Joystick m_controller = new Joystick(0);
    public final Drivetrain m_swerve = new Drivetrain();

    public LimeLight m_limelight = new LimeLight("two");
    private AtomicBoolean m_driveLocked = new AtomicBoolean();

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // PS2 joystick
    private int x = Constants.xJoystickConstants.axis_y, y = Constants.xJoystickConstants.axis_x, r = Constants.xJoystickConstants.axis_rot, t = Constants.xJoystickConstants.axis_throttle;
    // Logitech controller
    //private int x = 1, y = 0, r = 4, t = -1;

    public RobotContainer() {
        // Robot init
        m_limelight.setLedMode(1);
        m_swerve.SetFieldRelative(true);
        m_driveLocked.set(false);

        buttonConfig();
        autoConfig();
    }

    private void buttonConfig() {
        // Turns robot to limelight target
        new JoystickButton(m_controller, Constants.xJoystickConstants.Button_Trigger)
                .whileHeld(new LimelightAimDrive(m_limelight, m_swerve, m_controller, x, y, r, m_driveLocked));

        // Toggles if we drive with field relative
        new JoystickButton(m_controller, Constants.xJoystickConstants.Button_L3)
                .whenPressed(() -> m_swerve.SetFieldRelative(!m_swerve.IsFieldRelative()));

        // Zeros the gyro heading
        new JoystickButton(m_controller, Constants.xJoystickConstants.Button_R3)
                .whenPressed(() -> m_swerve.zeroGyroHeading());

        new JoystickButton(m_controller, Constants.xJoystickConstants.Button_X)
                .whenPressed(() -> m_swerve.setXDefault(!m_swerve.isXDefault()));  
                
            new JoystickButton(m_controller, Constants.xJoystickConstants.Button_Square)
                .whileHeld(new PixyAlign(m_swerve, m_driveLocked, false));
     }

    public void teleopExecute() {
        // Checks if the jotstick drive is being locked out by a command
        if (!m_driveLocked.get()) {
            driveWithJoystick(m_swerve.IsFieldRelative());
        }
    }

    public void driveWithJoystick(boolean fieldRelative) {
        // Get the x speed or forward speed
        final var xSpeed = (-m_controller.getRawAxis(x)) * Constants.DriveTrainConstants.kMaxSpeed;

        // Get the y speed or sideways/strafe speed.
        final var ySpeed = -m_controller.getRawAxis(y) * Constants.DriveTrainConstants.kMaxSpeed;

        // Get the rate of angular rotation.
        final var rot = -m_controller.getRawAxis(r) * Constants.DriveTrainConstants.kMaxAngularSpeed;

        // Increase max speed by throttle axis (inverted and add one makes the axis from 1 to 2)
        if (t != -1) {
            final var throttle = (-m_controller.getRawAxis(t) + 1);
            m_swerve.setMaxSpeed(throttle);
        }

         if ((Math.abs(xSpeed) < Constants.ControllerConstants.NoInputTolerance)
                    && (Math.abs(ySpeed) < Constants.ControllerConstants.NoInputTolerance)
                    && (Math.abs(rot) < Constants.ControllerConstants.NoInputTolerance)
                    && m_swerve.isXDefault()) {
                m_swerve.setWheelAngleStates(45, -45, -45, 45);
        }
        else{
                m_swerve.drive(getCurve(xSpeed), getCurve(ySpeed), getCurve(rot), fieldRelative);
        }
         
    }

    private void autoConfig() {
        this.autoChooser.addOption("Default Auto", new SequentialCommandGroup(
            new AutoDrive(m_swerve, 10, 0.5),
            new AutoLimelight(m_limelight, m_swerve, true),
            new AutoDoNothing(m_swerve)
        ));

        this.autoChooser.addOption("pixy auto", new SequentialCommandGroup(
            new AutoReset(m_swerve),
            new AutoTurn(m_swerve, 3, 1),
            new AutoReset(m_swerve),
            new AutoDrive(m_swerve, 2, -1),
            new AutoReset(m_swerve),
            new PixySearch(m_swerve, 2, 1.5),
            new PixyAlign(m_swerve),
            new PixyDrive(m_swerve, 1),
            new AutoDoNothing(m_swerve)
        ));

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
		return this.autoChooser.getSelected();
	}

    public static double getCurve(double input) {
        double sign = Math.signum(input);

        double value = Math.abs(input);
        value = Math.pow(value, 2);
        value += 0.02;

        return sign * value;
    }
}
