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

public class AutoLimelight extends CommandBase {
  private LimeLight m_limelight;
  private Drivetrain m_drivetrain;
  private PIDController turnPidController = new PIDController(Constants.DriveTrainConstants.limeP, 
  Constants.DriveTrainConstants.limeI, Constants.DriveTrainConstants.limeD);
  private boolean isFinished;


  public AutoLimelight(LimeLight limelight, Drivetrain drivetrain) {
    this.m_limelight = limelight;
    this.m_drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    this.isFinished = false;
    m_limelight.setLedMode(0);
  }

  @Override
  public void execute() {
    // Calculates x-axis rotation.
    double output = (turnPidController.calculate(m_limelight.getTX(), 0))
          * Constants.DriveTrainConstants.kMaxAngularSpeed;

    if(this.m_limelight.hasTarget()){
      if(Math.abs(this.m_limelight.getTX()) < 0.3){
        m_drivetrain.CanDrive(true);
        m_drivetrain.drive(0, 0, output, false);
        System.out.println("moving");
      }
      else{
        System.out.println("done");
        this.isFinished = true;
      }
    }
    else{
      System.out.println("no target");
      this.isFinished = true;
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
    m_drivetrain.CanDrive(false);
    m_limelight.setLedMode(1);
  }

  @Override
  public boolean isFinished() {
    return this.isFinished;
  }
}
