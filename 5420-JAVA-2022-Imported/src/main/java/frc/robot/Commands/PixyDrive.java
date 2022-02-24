package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.*;

public class PixyDrive extends CommandBase {
    private Drivetrain drivetrain;
    private double power;
    private boolean isFinished;
    private Intake intake;

    public PixyDrive(Drivetrain drivetrain, double power, Intake intake) {
    this.drivetrain = drivetrain;
    this.power = power;
    this.intake = intake;
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
    
    // Drive forward when it gets a target.
    if(drivetrain.pixyAlgo.getPixyBest() != null){
      if (drivetrain.pixyAlgo.getPixyBest().getArea() <= Constants.DriveTrainConstants.pixyTargetArea){
        drivetrain.drive(power, 0, 0, false);
        intake.setIntakePower(0.4);
      }
      else{
        // Don't drive if it doesn't have a target.
        drivetrain.drive(0, 0, 0, false);
        intake.setIntakePower(0);
        this.isFinished = true;
      }
    }
    else{
      this.isFinished = true;
  
    }
    

}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.resetAllDriveEncoders();
    drivetrain.drive(0, 0, 0, false);
    intake.setIntakePower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.isFinished;
  }
}
