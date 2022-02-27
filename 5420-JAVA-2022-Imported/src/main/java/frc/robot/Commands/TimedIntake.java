// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.*;

public class TimedIntake extends CommandBase {
  private Intake intake;
  private int duration;
  private Date endTime;
  private boolean isFinished;

  public TimedIntake(Intake intake, int duration) {
    this.intake = intake;
    this.duration = duration;
    // Use addRequirements() here to declare subsystem dependencies.
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
      intake.setIntakePower(0);
      this.isFinished = true;
    }
    else{
      intake.setIntakePower(-0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakePower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.isFinished;
  }
}
