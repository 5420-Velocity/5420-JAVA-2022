// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Shooter;


public class shootWithVelocity extends CommandBase {
  private Shooter shooter;
  private Double speed;

  public shootWithVelocity(Shooter shooter, Double speed) {
  this.shooter = shooter;
  this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.shooter.setShooterVelocity(speed * 3.4);
    
    if(Math.abs(this.shooter.GetShooterRPM() - speed) < 150){
      this.shooter.setFeedPower(-0.6);
      System.out.println(this.shooter.GetShooterRPM() + " " + speed);
    }
    else {
      this.shooter.setFeedPower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.shooter.setShooterVelocity(0);
    this.shooter.setShooterPower(0);
    this.shooter.setFeedPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
