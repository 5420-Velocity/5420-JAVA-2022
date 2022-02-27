/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Drivetrain.module;


public class PixySearch extends CommandBase {
    private Drivetrain drivetrain;
    private double target;
    private double power;
    private boolean isFinished;

    public PixySearch(Drivetrain drivetrain, double distance, double power) {
    this.drivetrain = drivetrain;
    this.target = distance;
    this.power = power * Constants.DriveTrainConstants.kMaxSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetAllDriveEncoders();
    this.isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    //Keep turning if the encoder value is less than the target.
    if(Math.abs(drivetrain.getWheelDriveEncoder(module.frontLeft)  / 13460)  < target){
      drivetrain.CanDrive(true);
      drivetrain.drive(0, 0, power, false);
    }
    else{
      // Stop turning if the encoder value is the same as the target
      drivetrain.CanDrive(false);
      drivetrain.drive(0, 0, 0, false);
      this.isFinished = true;
    }

    if(drivetrain.pixyAlgo.getPixyBest() != null) {
      drivetrain.CanDrive(false);
        this.isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.resetAllDriveEncoders();
    drivetrain.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.isFinished;
  }
}
