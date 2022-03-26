// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Shooter;

public class AutoShootWithVelocity extends CommandBase {
  private Shooter shooter;
  private Double speed;
  private Boolean isFinished;
  private Double timeout;
  private Double feedTimer;
  private long oldTimeSinceStart = 0;

  public AutoShootWithVelocity(Shooter shooter, Double speed, Double timeout) {
    this.shooter = shooter;
    this.speed = speed;
    this.timeout = timeout;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feedTimer = 0.0;
    this.isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.shooter.setShooterVelocity(speed);

    // saving the time since last scheduler run
    long timeSinceStart = System.nanoTime() / 1000;
    long deltaTime = timeSinceStart - oldTimeSinceStart;
    oldTimeSinceStart = timeSinceStart;
    
    if(Math.abs(this.shooter.GetShooterRPM() - speed) < 150){
      this.shooter.setFeedPower(-0.5);
      feedTimer += deltaTime;
    }
    else {
      this.shooter.setFeedPower(0);
    }

    if(feedTimer >= timeout){
      this.isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    oldTimeSinceStart = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.isFinished;
  }
}
