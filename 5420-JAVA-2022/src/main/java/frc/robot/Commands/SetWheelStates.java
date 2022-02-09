/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.*;

public class SetWheelStates extends CommandBase {
  private Drivetrain drivetrain;
  private double[] angles;

  public SetWheelStates(Drivetrain drivetrain, double[] angles) {
    this.drivetrain = drivetrain;
    this.angles = angles;
  }

  @Override
  public void initialize() {    
  }

  @Override
  public void execute() {
    drivetrain.setWheelAngleStates(angles[0], angles[1], angles[2], angles[3]);
  }

  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
