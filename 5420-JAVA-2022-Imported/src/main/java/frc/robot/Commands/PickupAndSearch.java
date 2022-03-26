// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.*;

public class PickupAndSearch extends CommandBase {
  private Drivetrain drivetrain;
  private Intake intake;
  private double gyroTarget;
  private PIDController turnPidController = new PIDController(0.02, 0, 0);
  private PIDController gyroPIDController = new PIDController(0.02, 0, 0);

  public PickupAndSearch(Drivetrain drivetrain, Intake intake, double gyroTarget) {
    this.drivetrain = drivetrain;
    this.intake = intake;
    this.gyroTarget = gyroTarget;
  }

  @Override
  public void initialize() {
    drivetrain.CanDrive(true);
  }

  @Override
  public void execute() {
    intake.setIntakePower(-0.8);
    if(drivetrain.pixyAlgo.getPixyBest() == null){
      double gyroOutput = gyroPIDController.calculate(drivetrain.GetGyroDegrees(), gyroTarget);
      if(Math.abs(drivetrain.GetGyroDegrees() - gyroTarget) < 10){
        drivetrain.drive(0, 0, gyroOutput, false);
      }
      else{
        drivetrain.drive(0, 0, 0, false);
      }
    }
    else{
      double output = turnPidController.calculate(drivetrain.pixyAlgo.getPixyBest().getX(), 150);
      drivetrain.drive(0, 0, output, false);
      System.out.println("turning");
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.CanDrive(false);
    drivetrain.drive(0, 0, 0, false);
    intake.setIntakePower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
