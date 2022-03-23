// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.Date;
import java.util.Calendar;
import java.util.GregorianCalendar;

import edu.wpi.first.wpilibj2.command.CommandBase;
import io.github.pseudoresonance.pixy2api.Pixy2;

public class AutoDelay extends CommandBase {
  private int duration;
  private Date endTime;
  private boolean isFinished;
  public AutoDelay(int duration) {
    this.duration = duration;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Calendar calculateDate = GregorianCalendar.getInstance();
    calculateDate.add(GregorianCalendar.MILLISECOND, duration);
		this.endTime = calculateDate.getTime();
    this.isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(new Date().after(endTime)){
      this.isFinished = true;
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
