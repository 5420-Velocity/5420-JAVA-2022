/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.networktables.NetworkTableEntry;
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
    public final Intake m_intake = new Intake();
    public final Lift m_lift = new Lift();
    //public final Intake m_reverse_intake = new reverseIntake();

    public LimeLight m_limelight = new LimeLight("two");
    private AtomicBoolean m_driveLocked = new AtomicBoolean();
    private AtomicBoolean m_liftLockout = new AtomicBoolean();

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // private AtomicReference<Double> shooterSpeed = new AtomicReference<Double>(0.5);
    // private NetworkTableEntry shootSpeed = SmartDashboard.getEntry("Shoot Speed");

    // PS2 joystick
    private int x = Constants.ThrustMasterJoystick.Axis_Y,
            y = Constants.ThrustMasterJoystick.Axis_X,
            r = Constants.ThrustMasterJoystick.Axis_Rot,
            t = Constants.ThrustMasterJoystick.Axis_Throttle;

    // Logitech controller
    //private int x = 1, y = 0, r = 4, t = -1;

    public RobotContainer() {
        // Robot init
        m_limelight.setLedMode(1);
        m_swerve.SetFieldRelative(true);
        m_driveLocked.set(false);
        m_liftLockout.set(false);

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
                .whenPressed(m_swerve::zeroGyroHeading);

        // Set the default position for drivetrain to x or none
        new JoystickButton(m_controller, Constants.ThrustMasterJoystick.Button_Left_Right)
                .whenPressed(() -> m_swerve.setXDefault(!m_swerve.isXDefault()));  
                
        // Alligns the robot intake with a cargo of your alliances cargo
        new JoystickButton(m_controller, Constants.ThrustMasterJoystick.Button_Thumb_Down)
                .whileHeld(new PixyAlign(m_swerve, m_driveLocked, false, m_controller, x, y, r));

        new JoystickButton(m_controller, Constants.ThrustMasterJoystick.Button_Right_Left)
                .whileHeld(() -> m_swerve.drive(0.8, 0, 0, false))
                .whileHeld(() -> m_driveLocked.set(true))
                .whenReleased(() -> m_driveLocked.set(false));

         /**
		 * Setup Button Events for the Shooter on the Operator Controller
		 */

        new JoystickButton(m_operatorController, Constants.ControllerConstants.Left_Bumper)
            .whileHeld(new LiftControl(m_lift, 0.90))
            .whenReleased(() -> this.m_lift.setMotorPower(0));

        new JoystickButton(m_operatorController, Constants.ControllerConstants.Right_Bumper)
            .whileHeld(new LiftControl(m_lift, -0.9))
            .whenReleased(() -> this.m_lift.setMotorPower(0));

        new JoystickButton(m_operatorController, Constants.ControllerConstants.Blue_Button_ID)
            .whenPressed(() -> this.m_liftLockout.set(true))
            .whenReleased(() -> this.m_liftLockout.set(false));

        // Sets the intake speed
        new JoystickButton(m_operatorController, Constants.ControllerConstants.Yellow_Button_ID)
            .whileHeld(new SimpleIntake(m_intake, -0.5));
        
        // // reverse intake
        // new JoystickButton (m_operatorController, Constants.ControllerConstants.Blue_Button_ID)
        //     .whileHeld(new SimpleIntake(m_reverse_intake));

        // Sets shooter speed
		// new JoystickButton(m_operatorController, Constants.ControllerConstants.Right_Bumper)
        //     .whileHeld(() -> this.m_shooter.setShooterPower(shooterSpeed.get()))
        //     .whenReleased(() -> this.m_shooter.setShooterPower(0));

        // Sets the feed motors to put cargo in the shooter
        new JoystickButton(m_operatorController, Constants.ControllerConstants.Red_Button_ID)
            .whileHeld(new SimpleIntake(m_intake, 0.4));

        // Shoots the ball with a timed gap between shots
        new JoystickButton(m_operatorController, Constants.ControllerConstants.Green_Button_ID)
            .whenHeld(new AutoShoot(m_shooter, m_limelight, 2));

        new JoystickButton(m_operatorController, Constants.ControllerConstants.Blue_Button_ID)
            .whenPressed(() -> this.m_intake.setReleasePower(-1))
            .whenReleased(() -> this.m_intake.setReleasePower(0));

        /**
		 * Used to dynamically adjust the speed used for shooting.
		 */

        // new JoystickDPad(m_operatorController, Position.kUp)
        //     .whenPressed(() -> {
        //         double increaseBy = 0.01;
        //         double newSpeed = shooterSpeed.get() + increaseBy;
        //         shooterSpeed.set(newSpeed);
        //         shootSpeed.setDouble(shooterSpeed.get());
        // });

        // new JoystickDPad(m_operatorController, Position.kDown)
        //     .whenPressed(() -> {
        //         double decreaseBy = -0.01;
        //         double newSpeed = shooterSpeed.get() + decreaseBy;
        //         shooterSpeed.set(newSpeed);
        //         shootSpeed.setDouble(shooterSpeed.get());
        //     });

        new JoystickDPad(m_operatorController, Position.kLeft)
            .whenHeld(new AutoShoot(m_shooter, 0.40, 2));

        new JoystickDPad(m_operatorController, Position.kUp)
            .whenHeld(new AutoShoot(m_shooter, 0.858, 2));

        new JoystickDPad(m_operatorController, Position.kRight)
            .whenHeld(new AutoShoot(m_shooter, 1, 2));

     }

    public void teleopExecute() {
        // Checks if the jotstick drive is being locked out by a command
        if (!m_driveLocked.get()) {
            driveWithJoystick(m_swerve.IsFieldRelative());
        }
    }

    public void driveWithJoystick(boolean fieldRelative) {
        // Get the x speed or forward speed

        double xSpeed = (-m_controller.getRawAxis(x)) * Constants.DriveTrainConstants.kMaxSpeed;

        // Get the y speed or sideways/strafe speed.
        double ySpeed = -m_controller.getRawAxis(y) * Constants.DriveTrainConstants.kMaxSpeed;
        if(Math.abs(ySpeed) < 0.15){
            ySpeed = 0;
        }

        // Get the rate of angular rotation.
        double rot = -m_controller.getRawAxis(r) * Constants.DriveTrainConstants.kMaxAngularSpeed;

        // Increase max speed by throttle axis (inverted and add one makes the axis from 1 to 2)
        if (t != -1) {
            double throttle = (-m_controller.getRawAxis(t) + 1);
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
                m_swerve.CanDrive(false);
                m_swerve.drive(0, 0, 0, fieldRelative);
            }
        }
        else{
            m_swerve.CanDrive(true);
            m_swerve.drive(getCurve(xSpeed), getCurve(ySpeed), getCurve(rot), fieldRelative);
        }
    }

    private void autoConfig() {

        this.autoChooser.addOption("Shoot pickup", new SequentialCommandGroup(
            new AutoShoot(m_shooter, 0.69, 1),
            new AutoReset(m_swerve),
            new AutoTurn(m_swerve, 3.5, 1),
            new AutoReset(m_swerve),
            new PixySearch(m_swerve, -0.2, 1),
            new PixySearch(m_swerve, 1, 1),
            new PixyPickup(m_swerve, m_intake, 1),
            new TimedIntake(m_intake, 1000),
            new AutoReset(m_swerve),
            new AutoDrive(m_swerve, 3, -1),
            new AutoDoNothing(m_swerve)
        ));

        this.autoChooser.addOption("Shoot taxi", new SequentialCommandGroup(
            new AutoShoot(m_shooter, 0.69, 1),
            new AutoReset(m_swerve),
            //distance
            new AutoDrive(m_swerve, 6, 1),
            new AutoDoNothing(m_swerve)
        ));

        this.autoChooser.addOption("Shoot pickup pickup", new SequentialCommandGroup(
            new AutoShoot(m_shooter, 0.72, 1),
            new AutoReset(m_swerve),
            new AutoTurn(m_swerve, 3.5, 1),
            new AutoReset(m_swerve),
            new PixySearch(m_swerve, -0.5, 1),
            new PixySearch(m_swerve, 1, 1),
            new PixyPickup(m_swerve, m_intake, 1),
            new TimedIntake(m_intake, 1000),
            new PixySearch(m_swerve, 1, 1),
            new PixyPickup(m_swerve, m_intake, 1),
            new TimedIntake(m_intake, 1000),
            new AutoReset(m_swerve),
            new AutoDrive(m_swerve, 1, -1),
            // new AutoShoot(m_shooter, 0.69, 1),
            // new AutoTurn(m_swerve, 3.5, 1),
            new AutoDoNothing(m_swerve)
        ));
        

        this.autoChooser.addOption("Shoot pickup shoot", new SequentialCommandGroup(
            new AutoShoot(m_shooter, 0.72, 1),
            new AutoReset(m_swerve),
            new AutoTurn(m_swerve, 3.5, 1),
            new AutoReset(m_swerve),
            new PixySearch(m_swerve, 1, 1),
            new PixyPickup(m_swerve, m_intake, 1),
            new TimedIntake(m_intake, 1000),
            new AutoTurn(m_swerve, 3.5, 1),
            new AutoLimelight(m_limelight, m_swerve),
            new AutoShoot(m_shooter, m_limelight, 1)
        ));

        this.autoChooser.addOption("other", new SequentialCommandGroup(
            new AutoLimelight(m_limelight, m_swerve)
        ));

        // Shoot low pick up 2 more balls shoot
        // Pick up 1 ball shoot both high

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
