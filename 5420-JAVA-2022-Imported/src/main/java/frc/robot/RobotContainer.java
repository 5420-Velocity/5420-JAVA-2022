/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.*;
import frc.robot.Subsystems.*;
import frc.robot.utils.JoystickDPad;
import frc.robot.utils.DPad.Position;
/**
 * Add your docs here.
 */
public class RobotContainer {
    private final Joystick m_controller = new Joystick(0);
    private final Joystick m_operatorController = new Joystick(1);
    public final Drivetrain m_swerve = new Drivetrain();
    private final Shooter m_shooter = new Shooter();
    private final Intake m_intake = new Intake();

    public LimeLight m_limelight = new LimeLight("two");
    private AtomicBoolean m_driveLocked = new AtomicBoolean();

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // PS2 joystick
    private int x = Constants.ThrustMasterJoystick.Axis_Y, y = Constants.ThrustMasterJoystick.Axis_X, r = Constants.ThrustMasterJoystick.Axis_Rot, t = Constants.ThrustMasterJoystick.Axis_Throttle;
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
        new JoystickButton(m_controller, Constants.ThrustMasterJoystick.Button_Trigger)
                .whileHeld(new LimelightAimDrive(m_limelight, m_swerve, m_controller, x, y, r, m_driveLocked));

        // Toggles if we drive with field relative
        new JoystickButton(m_controller, Constants.ThrustMasterJoystick.Button_Left_Left)
                .whenPressed(() -> m_swerve.SetFieldRelative(!m_swerve.IsFieldRelative()));

        // Zeros the gyro heading
        new JoystickButton(m_controller, Constants.ThrustMasterJoystick.Button_Left_Middle)
                .whenPressed(() -> m_swerve.zeroGyroHeading());

        new JoystickButton(m_controller, Constants.ThrustMasterJoystick.Button_Left_Right)
                .whenPressed(() -> m_swerve.setXDefault(!m_swerve.isXDefault()));  
                
            new JoystickButton(m_controller, Constants.ThrustMasterJoystick.Button_Thumb_Down)
                .whileHeld(new PixyAlign(m_swerve, m_driveLocked, false));

        new JoystickButton(m_operatorController, Constants.ControllerConstants.Yellow_Button_ID)
            .whileHeld(new SimpleIntake(m_intake));

        /**
		 * Used to dynamically adjust the speed used for shooting.
		 */
		AtomicReference<Double> shooterSpeed = new AtomicReference<Double>(0.5);

        /**
		 * Setup Button Events for the Shooter on the Operator Controller
		 */
		new JoystickButton(m_operatorController, Constants.ControllerConstants.Right_Bumper)
            .whileHeld(() -> this.m_shooter.setShooterPower(shooterSpeed.get()))
            .whenReleased(() -> this.m_shooter.setShooterPower(0));

        new JoystickButton(m_operatorController, Constants.ControllerConstants.Red_Button_ID)
            .whileHeld(() -> this.m_shooter.setFeedPower(0.5))
            .whenReleased(() -> this.m_shooter.setFeedPower(0));

        new JoystickDPad(m_operatorController, Position.kUp)
            .whenPressed(() -> {
                double increaseBy = 0.01;
                double newSpeed = shooterSpeed.get() + increaseBy;
                shooterSpeed.set(newSpeed);
        });

        new JoystickDPad(m_operatorController, Position.kDown)
            .whenPressed(() -> {
                double decreaseBy = -0.01;
                double newSpeed = shooterSpeed.get() + decreaseBy;
                shooterSpeed.set(newSpeed);
            });

     }

    public void teleopExecute() {
        // Checks if the jotstick drive is being locked out by a command
        if (!m_driveLocked.get()) {
            driveWithJoystick(m_swerve.IsFieldRelative());
        }
        else{
            m_swerve.CanDrive(true);
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
                    && (Math.abs(rot) < Constants.ControllerConstants.NoInputTolerance)) {
                         if(m_swerve.isXDefault()){
                             m_swerve.CanDrive(true);
                             m_swerve.setWheelAngleStates(45, -45, -45, 45);
                         }
                         else{
                            m_swerve.StopModules();
                            m_swerve.CanDrive(false);
                        }
        }
        else{
            m_swerve.CanDrive(true);
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
