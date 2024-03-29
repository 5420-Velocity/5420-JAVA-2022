// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Drivetrain;

public class TurnWithGyro extends CommandBase {
  private Drivetrain drivetrain;
  private Boolean isFinished;
  private Double target;
  private double power;
  
  /** Creates a new TurnWithGyro. */
  public TurnWithGyro(Drivetrain drivetrain, Double target, double power) {
    this.drivetrain = drivetrain;
    this.target = target;
    this.power = power;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Math.abs(target - drivetrain.GetGyroDegrees()) < 7) {
      drivetrain.drive(0, 0, 0,false);
      drivetrain.CanDrive(false);
      this.isFinished = true;
    }
    else{
      drivetrain.drive(0, 0, power, false);
      drivetrain.CanDrive(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.isFinished;
  }
}
