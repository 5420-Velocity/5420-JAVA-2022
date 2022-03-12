// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.*;

public class ShootWithRPM extends CommandBase {
  private Shooter _shooter;
  private double target;
  private PIDController shootPID = new PIDController(0.05, 0, 0);
  public ShootWithRPM(Shooter shooter, double target) {
    this._shooter = shooter;
    this.target = target;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(_shooter.GetShooterRPM());
    double output = shootPID.calculate(_shooter.GetShooterRPM(), target);
    _shooter.setShooterPower(output);
    if(Math.abs(_shooter.GetShooterRPM() - target) < 300){
      _shooter.setFeedPower(0.2);
    }
    else{
      _shooter.setFeedPower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _shooter.setShooterPower(0);
    _shooter.setFeedPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
