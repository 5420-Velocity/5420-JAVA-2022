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
import frc.robot.Subsystems.Drivetrain;

public class AutoReset extends CommandBase {
  private Drivetrain drivetrain;
  private boolean isFinished;
  private Date finishTime;

  public AutoReset(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Calendar calculateDate = GregorianCalendar.getInstance();
    calculateDate.add(GregorianCalendar.MILLISECOND, 1000);
    this.finishTime = calculateDate.getTime();
    this.isFinished = false;
    System.out.println("reset start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.resetAllDriveEncoders();
    drivetrain.drive(0, 0, 0, false);
    if (new Date().after(finishTime)) {
      this.isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("reset end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.isFinished;
  }
}
