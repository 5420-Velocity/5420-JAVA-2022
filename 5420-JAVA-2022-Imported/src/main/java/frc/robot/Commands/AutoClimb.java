// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Lift;

public class AutoClimb extends CommandBase {
  private Lift lift;
  private Boolean isFinished;

  public AutoClimb(Lift lift) {
    this.lift = lift;
    //do i need this
    addRequirements(lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //do i need this
    this.isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // implement limit switch
    //implement current sensor
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //set lift power to 0
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
