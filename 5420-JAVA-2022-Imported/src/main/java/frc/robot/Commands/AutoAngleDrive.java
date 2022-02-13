/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Drivetrain.module;

public class AutoAngleDrive extends CommandBase {
  private Drivetrain drivetrain;
  private double target;
  private double angle;
  private double power;
  private boolean isFinished;

  public AutoAngleDrive(Drivetrain drivetrain, double targetDistance, double angle, double power) {
    this.drivetrain = drivetrain;
    this.target = targetDistance;
    this.angle = angle;
    this.power = power;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetAllDriveEncoders();
    this.isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if encoder value is less than the target keep driving
    // otherwise don't turn
    if (Math.abs(drivetrain.getWheelDriveEncoder(module.frontLeft) / 13460) < target) {
      Rotation2d rot = new Rotation2d(Math.toRadians(angle));
      drivetrain.drive(rot.getCos() * Constants.DriveTrainConstants.kMaxSpeed * power,
          rot.getSin() * Constants.DriveTrainConstants.kMaxSpeed * power, 0, false);
    } else {
      drivetrain.drive(0, 0, 0, false);
      this.isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.resetAllDriveEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.isFinished;
  }
}
