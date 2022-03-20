/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.*;
import frc.robot.utils.StateList;

public class AutoLimelight extends CommandBase {
  private LimeLight m_limelight;
  private Drivetrain m_drivetrain;
  private PIDController turnPidController = new PIDController(Constants.DriveTrainConstants.limeP, 
  Constants.DriveTrainConstants.limeI, Constants.DriveTrainConstants.limeD);
  
  private StateList<Boolean> stateList = StateList.bool(5);


  public AutoLimelight(LimeLight limelight, Drivetrain drivetrain) {
    this.m_limelight = limelight;
    this.m_drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    m_limelight.setLedMode(3);
  }

  @Override
  public void execute() {
    // Calculates x-axis rotation.
    double output = (turnPidController.calculate(m_limelight.getTX(), 0))
          * Constants.DriveTrainConstants.kMaxAngularSpeed;

    m_drivetrain.CanDrive(true);
    m_drivetrain.drive(0, 0, output, false);
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
    m_drivetrain.CanDrive(false);
    m_drivetrain.drive(0, 0, 0, false);
    m_limelight.setLedMode(0);
  }

  @Override
  public boolean isFinished() {
    // If we are NOT in the tollerance then continue to run
    // TODO: add a condition for has target.

    // boolean isGoodGood = Math.abs(this.m_limelight.getTX()) < 0.5;
    // this.stateList.add(isGoodGood);
    // return this.stateList.get();
    return (Math.abs(this.m_limelight.getTX()) < 0.5) && this.m_limelight.hasTarget();
  }
}
