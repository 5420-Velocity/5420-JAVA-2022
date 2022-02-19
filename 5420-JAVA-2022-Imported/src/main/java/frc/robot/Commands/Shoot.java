/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import java.util.Date;
import java.util.Calendar;
import java.util.GregorianCalendar;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.*;

public class Shoot extends CommandBase {
  private Shooter _shooter;
  private int feedDuration = 2000;
  private int rampDuration = 1000;
  private Date rampEndTime;
  private Date feedEndTime;
  private boolean isFinished;

  public Shoot(Shooter shooter) {
    this._shooter = shooter;
  }

  // Called when the command is initially scheduled.
  public void initialize() {
    Calendar calculateDate = GregorianCalendar.getInstance();
    calculateDate.add(GregorianCalendar.MILLISECOND, rampDuration);
		this.rampEndTime = calculateDate.getTime();
    calculateDate.add(GregorianCalendar.MILLISECOND, feedDuration);
    this.feedEndTime = calculateDate.getTime();
    this.isFinished = false;
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (new Date().after(this.rampEndTime)) {

      if(new Date().after(this.feedEndTime)){
        _shooter.setFeedPower(0);
        _shooter.setShooterPower(0);
        this.isFinished = true;
      }
      else{
        _shooter.setShooterPower(0.4);
        _shooter.setFeedPower(0.4);
      }
      
    }
    else {
      _shooter.setFeedPower(0);
      _shooter.setShooterPower(0.4);
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
    return this.isFinished;
  }
}
