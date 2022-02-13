/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.LimeLight;

public class AutoLimelight extends CommandBase {
  private boolean isFinished;
  private LimeLight limelight;
  private Drivetrain drivetrain;
  private boolean finishable;
  private AtomicBoolean lock;
  private PIDController turnPidController = new PIDController(Constants.DriveTrainConstants.limeP, 
  Constants.DriveTrainConstants.limeI, Constants.DriveTrainConstants.limeD);

  public AutoLimelight(LimeLight limelight, Drivetrain drivetrain, boolean finishable) {
    this.limelight = limelight;
    this.drivetrain = drivetrain;
    this.finishable = finishable;
  }

  public AutoLimelight(LimeLight limelight, Drivetrain drivetrain, boolean finishable, AtomicBoolean lock) {
    this.limelight = limelight;
    this.drivetrain = drivetrain;
    this.finishable = finishable;
    this.lock = lock;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.setLedMode(0);
    this.isFinished = false;
    if (this.lock != null) {
      this.lock.set(true);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Check if the joysticks are locked out then turn the bot towards limelight
    // target with PID
    if (Math.abs(limelight.getTX()) > 0.1) {
      double output = (turnPidController.calculate(limelight.getTX(), 0))
          * Constants.DriveTrainConstants.kMaxAngularSpeed;
      drivetrain.drive(0, 0, output, false);
    } else {
      //drivetrain.keepWheelStates();
      this.isFinished = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limelight.setLedMode(1);
    if (this.lock != null) {
      this.lock.set(false);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (finishable) {
      return this.isFinished;
    } else {
      return false;
    }
  }
}
