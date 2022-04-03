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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
    private Command liftCommand = new NewLiftControl(m_lift, m_operatorController);

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
        configDefaultCommands();
    }

    private void buttonConfig() {
        // Turns robot to limelight target
        new JoystickButton(m_controller, Constants.ThrustMasterJoystick.Button_Trigger)
                .whileHeld(new LimelightAimDrive(m_limelight, m_swerve, m_controller, x, y, r, m_driveLocked));

        // Toggles if we drive with field relative (CHANGED TO TOGGLE -Jimmy)
        new JoystickButton(m_controller, Constants.ThrustMasterJoystick.Button_Thumb_Down)
                .whenPressed(() -> m_swerve.SetFieldRelative(false))
                .whenReleased(() -> m_swerve.SetFieldRelative(true));

        // Zeros the gyro heading
        new JoystickButton(m_controller, Constants.ThrustMasterJoystick.Button_Left_Middle)
                .whenPressed(m_swerve::zeroGyroHeading);

        // Set the default position for drivetrain to x or none
        new JoystickButton(m_controller, Constants.ThrustMasterJoystick.Button_Left_Right)
                .whenPressed(() -> m_swerve.setXDefault(!m_swerve.isXDefault()));

        // Alligns the robot intake with a cargo of your alliances cargo
        new JoystickButton(m_controller, Constants.ThrustMasterJoystick.Button_Thumb_Left)
                .whileHeld(new PixyAlign(m_swerve, m_driveLocked, false, m_controller, x, y, r));

        new JoystickButton(m_controller, Constants.ThrustMasterJoystick.Button_Thumb_Right)
                .whileHeld(new PixyAlign(m_swerve, m_driveLocked, false, m_controller, x, y, r));

         /**
		 * Setup Button Events for the Shooter on the Operator Controller
		 */

        // new JoystickButton(m_operatorController, Constants.ControllerConstants.Left_Bumper)
        //     .whileHeld(new LiftControl(m_lift, 0.5))
        //     .whenReleased(() -> this.m_lift.setMotorPower(0));

        // new JoystickButton(m_operatorController, Constants.ControllerConstants.Right_Bumper)
        //     .whileHeld(new LiftControl(m_lift, 0.5))
        //     .whenReleased(() -> this.m_lift.setMotorPower(0));

        // Sets the intake speed
        new JoystickButton(m_operatorController, Constants.ControllerConstants.Yellow_Button_ID)
            .whileHeld(new SimpleIntake(m_intake, -0.7));

        // Sets the feed motors to put cargo in the shooter
        new JoystickButton(m_operatorController, Constants.ControllerConstants.Red_Button_ID)
            .whileHeld(new SimpleIntake(m_intake, 0.5));

        new JoystickButton(m_operatorController, Constants.ControllerConstants.Blue_Button_ID)
            .whenPressed(() -> this.m_intake.setReleasePower(0.8))
            .whenReleased(() -> this.m_intake.setReleasePower(0));

        // GREEN BUTTON FOR SEMIAUTOCLIMB
        new JoystickButton(m_operatorController, Constants.ControllerConstants.Green_Button_ID);
            //when pressed, run SemiAutoClimb
        // Shoot buttons with preset speeds

        new JoystickButton(m_operatorController, Constants.ControllerConstants.Left_Bumper)
            .whenHeld(new shootWithVelocity(m_shooter, 2000.0))
            .whenReleased(() -> this.m_shooter.setShooterPower(0))
            .whenReleased(() -> this.m_shooter.setFeedPower(0));

            //change this button for zach?
        new JoystickButton(m_operatorController, Constants.ControllerConstants.Right_Bumper)
            .whenHeld(new shootWithVelocity(m_shooter, 6700.0)) 
            .whenReleased(() -> this.m_shooter.setShooterPower(0))
            .whenReleased(() -> this.m_shooter.setFeedPower(0));

        new JoystickDPad(m_operatorController, Position.kDown)
            .whenHeld(new shootWithVelocity(m_shooter, 4000.0))
            .whenReleased(() -> this.m_shooter.setShooterPower(0))
            .whenReleased(() -> this.m_shooter.setFeedPower(0));

        new JoystickDPad(m_operatorController, Position.kLeft)
            .whenHeld(new shootWithVelocity(m_shooter, 4500.0))
            .whenReleased(() -> this.m_shooter.setShooterPower(0))
            .whenReleased(() -> this.m_shooter.setFeedPower(0));

        new JoystickDPad(m_operatorController, Position.kUp)
            .whenHeld(new shootWithVelocity(m_shooter, 5000.0))
            .whenReleased(() -> this.m_shooter.setShooterPower(0))
            .whenReleased(() -> this.m_shooter.setFeedPower(0));
            
        new JoystickDPad(m_operatorController, Position.kRight)
            .whenHeld(new shootWithVelocity(m_shooter, 5500.0))
            .whenReleased(() -> this.m_shooter.setShooterPower(0))
            .whenReleased(() -> this.m_shooter.setFeedPower(0));

     }

    public void teleopExecute() {
        // Checks if the jotstick drive is being locked out by a command
        if (!m_driveLocked.get()) {
            driveWithJoystick(m_swerve.IsFieldRelative());
        }
        new liftRotationControl(m_lift, m_operatorController, 3);
        
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

    private void configDefaultCommands(){
        CommandScheduler scheduler = CommandScheduler.getInstance();

		scheduler.setDefaultCommand(this.m_lift, liftCommand);
    }

    private void autoConfig() {

        this.autoChooser.addOption("shoot terminal center", new SequentialCommandGroup(
            // ball 1 and 2
            new ResetGyro(m_swerve),
            new AutoReset(m_swerve),
            new PixySearch(m_swerve, 1, 1),
            new PixyPickup(m_swerve, m_intake, 1.5),
            new ParallelCommandGroup(
                new TimedIntake(m_intake, 800),
                new AutoDrive(m_swerve, 0.5, -2)
            ),
            new AutoReset(m_swerve),
            new TurnWithGyro (m_swerve, 180.0, 3),
            new ParallelCommandGroup(
                new AutoLimelight(m_limelight, m_swerve, 1000),
                new shootWithVelocity(m_shooter, 4400.0, 3500)),
            //turn to terminal and run intake for 10 seconds
            new TurnWithGyro(m_swerve, 300.0, 3),
            new AutoReset(m_swerve),
            new AutoDrive(m_swerve, 3, -2),
            new PickupAndSearch(m_swerve, m_intake, 30)
        ));

        this.autoChooser.addOption("shoot terminal right", new SequentialCommandGroup(
                        // ball 1 and 2
            new ResetGyro(m_swerve),
            new AutoReset(m_swerve),
            new PixySearch(m_swerve, 1, 1),
            new PixyPickup(m_swerve, m_intake, 1.5),
            new ParallelCommandGroup(
                new TimedIntake(m_intake, 800),
                new AutoDrive(m_swerve, 0.5, -2)
            ),
            new AutoReset(m_swerve),
            new TurnWithGyro (m_swerve, 180.0, 3),
            new ParallelCommandGroup(
                new AutoLimelight(m_limelight, m_swerve, 1000),
                new shootWithVelocity(m_shooter, 4400.0, 3500)),
            //turn to terminal and run intake for 10 seconds
            new TurnWithGyro(m_swerve, 270.0, 3),
            new AutoReset(m_swerve),
            new AutoDrive(m_swerve, 3, -2),
            new PickupAndSearch(m_swerve, m_intake, 30)
        ));

        this.autoChooser.addOption("shoot terminal left", new SequentialCommandGroup(

                    // ball 1 and 2
                    new ResetGyro(m_swerve),
                    new AutoReset(m_swerve),
                    new PixySearch(m_swerve, 1, 1),
                    new PixyPickup(m_swerve, m_intake, 1.5),
                    new ParallelCommandGroup(
                        new TimedIntake(m_intake, 800),
                        new AutoDrive(m_swerve, 0.5, -2)
                    ),
                    new AutoReset(m_swerve),
                    new TurnWithGyro (m_swerve, 180.0, 3),
                    new ParallelCommandGroup(
                        new AutoLimelight(m_limelight, m_swerve, 1000),
                        new shootWithVelocity(m_shooter, 4400.0, 3500)),
                    //turn to terminal and run intake for 10 seconds
                    new TurnWithGyro(m_swerve, 30.0, 3),
                    new AutoReset(m_swerve),
                    new AutoDrive(m_swerve, 3, -2),
                    new PickupAndSearch(m_swerve, m_intake, 30)
                ));
                 
                //experimental: shoot 4 balls total (including shooting a ball from terminal)
        this.autoChooser.addOption("shoot 4 balls left (red alliance)", new SequentialCommandGroup(
        //balls 1 and 2
            new ResetGyro(m_swerve),
            new PixySearch(m_swerve, 1, 1),
            new PixyPickup(m_swerve, m_intake, 1),
            new ParallelCommandGroup(
                new TimedIntake(m_intake, 800),
                new AutoDrive(m_swerve, 0.5, -2)
            ),
            //reset encoders and turn to hub
            new AutoReset (m_swerve),
            new TurnWithGyro(m_swerve, 180.0, 3),
            // aim and shoot
            new AutoLimelight(m_limelight, m_swerve, 1000),
            new shootWithVelocity(m_shooter, 4400.0, 3500),
            // turn to terminal and get a ball from the human player
            //intake runs for 10 seconds
            //placeholder turn angle that may have to be changed cuz I didn't do the math lol 
            new AutoReset(m_swerve),
            new TurnWithGyro(m_swerve, 90.0, 3),
            new AutoReset(m_swerve),
            new AutoDrive(m_swerve, 3, -2),
            new PickupAndSearch(m_swerve, m_intake, 30),
            // turn and look for the target
            // placeholder angle
            // use turnWithGyro
            new TurnWithGyro(m_swerve, -90.0, 1),
            new AutoLimelight(m_limelight, m_swerve, 1000),
            new AutoShoot(m_shooter, m_limelight, 2)
        ));

        //CHANGE TURN ANGLES!!! (for going to terminal)
        this.autoChooser.addOption("shoot 4 balls center (red alliance)", new SequentialCommandGroup(
        //balls 1 and 2
            new ResetGyro(m_swerve),
            new PixySearch(m_swerve, 1, 1),
            new PixyPickup(m_swerve, m_intake, 1),
            new ParallelCommandGroup(
                new TimedIntake(m_intake, 800),
                new AutoDrive(m_swerve, 0.5, -2)
            ),
            //reset encoders and turn to hub
            new AutoReset (m_swerve),
            new TurnWithGyro(m_swerve, 180.0, 3),
            // aim and shoot
            new AutoLimelight(m_limelight, m_swerve, 1000),
            new shootWithVelocity(m_shooter, 4400.0, 3500),
            // turn to terminal and get a ball from the human player
            //intake runs for 10 seconds
            //placeholder turn angle that may have to be changed cuz I didn't do the math lol 
            new AutoReset(m_swerve),
            new TurnWithGyro(m_swerve, 20.0, 3),
            new AutoReset(m_swerve),
            new AutoDrive(m_swerve, 3, -2),
            new PickupAndSearch(m_swerve, m_intake, 30),
            // turn and look for the target
            // placeholder angle
            // use turnWithGyro
            new TurnWithGyro(m_swerve, -20.0, 1),
            new AutoLimelight(m_limelight, m_swerve, 1000),
            new AutoShoot(m_shooter, m_limelight, 2)
        ));
        //CHANGE TURN ANGLES!!! (for going to terminal)
        this.autoChooser.addOption("shoot 4 balls right (red alliance)", new SequentialCommandGroup(
        //balls 1 and 2
            new ResetGyro(m_swerve),
            new PixySearch(m_swerve, 1, 1),
            new PixyPickup(m_swerve, m_intake, 1),
            new ParallelCommandGroup(
                new TimedIntake(m_intake, 800),
                new AutoDrive(m_swerve, 0.5, -2)
            ),
            //reset encoders and turn to hub
            new AutoReset (m_swerve),
            new TurnWithGyro(m_swerve, 180.0, 3),
            // aim and shoot
            new AutoLimelight(m_limelight, m_swerve, 1000),
            new shootWithVelocity(m_shooter, 4400.0, 3500),
            // turn to terminal and get a ball from the human player
            //intake runs for 10 seconds
            //placeholder turn angle that may have to be changed cuz I didn't do the math lol 
            new AutoReset(m_swerve),
            new TurnWithGyro(m_swerve, 60.0, 3),
            new AutoReset(m_swerve),
            new AutoDrive(m_swerve, 3, -2),
            new PickupAndSearch(m_swerve, m_intake, 30),
            // turn and look for the target
            // placeholder angle
            // use turnWithGyro
            new TurnWithGyro(m_swerve, -60.0, 1),
            new AutoLimelight(m_limelight, m_swerve, 1000),
            new AutoShoot(m_shooter, m_limelight, 2)
        ));
      
        //CHANGE TURN ANGLES!!! (for going to terminal)
        this.autoChooser.addOption("shoot 4 balls left (blue alliance)", new SequentialCommandGroup(
        //balls 1 and 2
            new ResetGyro(m_swerve),
            new PixySearch(m_swerve, 1, 1),
            new PixyPickup(m_swerve, m_intake, 1),
            new ParallelCommandGroup(
                new TimedIntake(m_intake, 800),
                new AutoDrive(m_swerve, 0.5, -2)
            ),
            //reset encoders and turn to hub
            new AutoReset (m_swerve),
            new TurnWithGyro(m_swerve, 180.0, 3),
            // aim and shoot
            new AutoLimelight(m_limelight, m_swerve, 1000),
            new shootWithVelocity(m_shooter, 4400.0, 3500),
            // turn to terminal and get a ball from the human player
            //intake runs for 10 seconds
            //placeholder turn angle that may have to be changed cuz I didn't do the math lol 
            new AutoReset(m_swerve),
            new TurnWithGyro(m_swerve, 45.0, 3),
            new AutoReset(m_swerve),
            new AutoDrive(m_swerve, 3, -2),
            new PickupAndSearch(m_swerve, m_intake, 30),
            // turn and look for the target
            // placeholder angle
            // use turnWithGyro
            new TurnWithGyro(m_swerve, -45.0, 1),
            new AutoLimelight(m_limelight, m_swerve, 1000),
            new AutoShoot(m_shooter, m_limelight, 2)
        ));
        
        //CHANGE TURN ANGLES!!! (for going to terminal)
        this.autoChooser.addOption("shoot 4 balls center (blue alliance)", new SequentialCommandGroup(
        //balls 1 and 2
            new ResetGyro(m_swerve),
            new PixySearch(m_swerve, 1, 1),
            new PixyPickup(m_swerve, m_intake, 1),
            new ParallelCommandGroup(
                new TimedIntake(m_intake, 800),
                new AutoDrive(m_swerve, 0.5, -2)
            ),
            //reset encoders and turn to hub
            new AutoReset (m_swerve),
            new TurnWithGyro(m_swerve, 180.0, 3),
            // aim and shoot
            new AutoLimelight(m_limelight, m_swerve, 1000),
            new shootWithVelocity(m_shooter, 4400.0, 3500),
            // turn to terminal and get a ball from the human player
            //intake runs for 10 seconds
            //placeholder turn angle that may have to be changed cuz I didn't do the math lol 
            new AutoReset(m_swerve),
            new TurnWithGyro(m_swerve, 180.0, 3),
            new AutoReset(m_swerve),
            new AutoDrive(m_swerve, 3, -2),
            new PickupAndSearch(m_swerve, m_intake, 30),
            // turn and look for the target
            // placeholder angle
            // use turnWithGyro
            new TurnWithGyro(m_swerve, -180.0, 1),
            new AutoLimelight(m_limelight, m_swerve, 1000),
            new AutoShoot(m_shooter, m_limelight, 2)
        ));

        //CHANGE TURN ANGLES!!! (for going to terminal)
        this.autoChooser.addOption("shoot 4 balls right (blue alliance)", new SequentialCommandGroup(
        //balls 1 and 2
            new ResetGyro(m_swerve),
            new PixySearch(m_swerve, 1, 1),
            new PixyPickup(m_swerve, m_intake, 1),
            new ParallelCommandGroup(
                new TimedIntake(m_intake, 800),
                new AutoDrive(m_swerve, 0.5, -2)
            ),
            //reset encoders and turn to hub
            new AutoReset (m_swerve),
            new TurnWithGyro(m_swerve, 180.0, 3),
            // aim and shoot
            new AutoLimelight(m_limelight, m_swerve, 1000),
            new shootWithVelocity(m_shooter, 4400.0, 3500),
            // turn to terminal and get a ball from the human player
            //intake runs for 10 seconds
            //placeholder turn angle that may have to be changed cuz I didn't do the math lol 
            new AutoReset(m_swerve),
            new TurnWithGyro(m_swerve, 70.0, 3),
            new AutoReset(m_swerve),
            new AutoDrive(m_swerve, 3, -2),
            new PickupAndSearch(m_swerve, m_intake, 30),
            // turn and look for the target
            // placeholder angle
            // use turnWithGyro
            new TurnWithGyro(m_swerve, -70.0, 1),
            new AutoLimelight(m_limelight, m_swerve, 1000),
            new AutoShoot(m_shooter, m_limelight, 2)
        ));

                //updated for longer distance taxi
        this.autoChooser.addOption("pickup shoot", new SequentialCommandGroup(
            new ResetGyro(m_swerve),
            new AutoReset(m_swerve),
            new AutoDelay(500),
            new PixySearch(m_swerve, 1, 1),
            new PixyPickup(m_swerve, m_intake, 1.5),
            new ParallelCommandGroup(
                new IntakeUsingColorSensor(m_intake),
                new AutoDrive(m_swerve, 1, -2)
            ),
            new AutoReset(m_swerve),
            new TurnWithGyro (m_swerve, 180.0, 3),
            new AutoLimelight(m_limelight, m_swerve, 1000),
            new shootWithVelocity(m_shooter, 4600.0, 3500),
            new AutoDoNothing(m_swerve)
        ));

        this.autoChooser.addOption("other", new SequentialCommandGroup(
            new ResetGyro(m_swerve),
            new AutoDrive(m_swerve, 3, -1.4),
            new TurnWithGyro(m_swerve, 30.0, 2),
            new AutoReset(m_swerve),
            new AutoDrive(m_swerve, 8, -1.4),
            new PixySearch(m_swerve, 1, 1),
            new PixyPickup(m_swerve, m_intake, 1),
            new TimedIntake(m_intake, 2000), 
            new PickupAndSearch(m_swerve, m_intake, 30)

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
