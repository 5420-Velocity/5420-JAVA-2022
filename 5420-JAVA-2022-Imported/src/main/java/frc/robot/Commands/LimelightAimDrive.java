/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.*;

public class LimelightAimDrive extends CommandBase {
  private LimeLight m_limelight;
  private Drivetrain m_drivetrain;
  private Joystick m_joystick;
  private AtomicBoolean m_lock;
  private PIDController turnPidController = new PIDController(Constants.DriveTrainConstants.limeP, 
  Constants.DriveTrainConstants.limeI, Constants.DriveTrainConstants.limeD);
  private int x;
  private int y;
  private int r;

  public LimelightAimDrive(LimeLight limelight, Drivetrain drivetrain, Joystick joystick, int x, int y, int r , AtomicBoolean lock) {
    this.m_limelight = limelight;
    this.m_drivetrain = drivetrain;
    this.m_joystick = joystick;
    this.m_lock = lock;
    this.x = x;
    this.y = y;
    this.r = r;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    m_limelight.setLedMode(0);
    this.m_lock.set(true);
  }

  @Override
  public void execute() {
    // Calculates x-axis rotation.
    double output = (turnPidController.calculate(m_limelight.getTX(), 0))
          * Constants.DriveTrainConstants.kMaxAngularSpeed;
    // Determine rotation speed for x-axis.
    final var xSpeed = -m_joystick.getRawAxis(x) * Constants.DriveTrainConstants.kMaxSpeed;
    // Determine rotation speed for y-axis.
    final var ySpeed = -m_joystick.getRawAxis(y) * Constants.DriveTrainConstants.kMaxSpeed;
    final var rotSpeed = -m_joystick.getRawAxis(r) * Constants.DriveTrainConstants.kMaxSpeed;

    if(this.m_limelight.hasTarget()){
      m_drivetrain.CanDrive(true);
      m_drivetrain.drive(getCurve(xSpeed), getCurve(ySpeed), output, m_drivetrain.IsFieldRelative());
    }
    else{
      if ((Math.abs(xSpeed) < Constants.ControllerConstants.NoInputTolerance)
                    && (Math.abs(ySpeed) < Constants.ControllerConstants.NoInputTolerance)
                    && (Math.abs(rotSpeed) < Constants.ControllerConstants.NoInputTolerance)) {
          m_drivetrain.CanDrive(false);
      }
      else{
        m_drivetrain.CanDrive(true);
        m_drivetrain.drive(getCurve(xSpeed), getCurve(ySpeed), getCurve(rotSpeed), m_drivetrain.IsFieldRelative());
      }
    }
  }

  // Curves speed imput.
  public static double getCurve(double input) {
    double sign = Math.signum(input);

    double value = Math.abs(input);
    value = Math.pow(value, 2);
    value += 0.02;

    return sign * value;
}

  @Override
  public void end(boolean interrupted) {
    m_limelight.setLedMode(1);
    this.m_lock.set(false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
