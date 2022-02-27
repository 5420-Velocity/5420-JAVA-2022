// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.LimeLight;
import frc.robot.Subsystems.Drivetrain.module;

public class LimelightSearch extends CommandBase {
  private Drivetrain _drivetrain;
  private LimeLight _limelight;
  private double target;
  private boolean isFinished;
  private double power;
  public LimelightSearch(Drivetrain drivetrain, LimeLight limeLight, double target, double power) {
    this._drivetrain = drivetrain;
    this._limelight = limeLight;
    this.target = target;
    this.power = power;
  }

  @Override
  public void initialize() {
    this.isFinished = false;
  }

  @Override
  public void execute() {
    if(!_limelight.hasTarget()){
      //Keep turning if the encoder value is less than the target.
      if(Math.abs(_drivetrain.getWheelDriveEncoder(module.frontLeft)  / 13460)  < target){
        _drivetrain.CanDrive(true);
        _drivetrain.drive(0, 0, power, false);
      }
      else{
        // Stop turning if the encoder value is the same as the target
        _drivetrain.CanDrive(false);
        _drivetrain.drive(0, 0, 0, false);
      }
    }
    else{
      _drivetrain.CanDrive(false);
      _drivetrain.drive(0, 0, 0, false);
      this.isFinished = true;
    }
        
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _drivetrain.CanDrive(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.isFinished;
  }
}
