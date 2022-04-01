// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Intake;

// .add(new InakeUsingColorSensor(intake))

public class IntakeUsingColorSensor extends CommandBase {
  
  private Intake intake;

  public IntakeUsingColorSensor(Intake intake) {
    this.intake = intake;
    this.addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (intake.getColor().color == Constants.ColorTargets.COLOR_BLUE || intake.getColor().color == Constants.ColorTargets.COLOR_RED) {
      intake.setIntakePower(0.5);
    }
    else {
      intake.setIntakePower (0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setIntakePower(-0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakePower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(intake.getColor().confidence);
    return intake.getColor().confidence > 0.9;
  }
}
