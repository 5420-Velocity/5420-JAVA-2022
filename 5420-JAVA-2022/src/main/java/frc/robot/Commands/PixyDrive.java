package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;

public class PixyDrive extends CommandBase {
    private Drivetrain drivetrain;
    private double power;
    private boolean isFinished;

    public PixyDrive(Drivetrain drivetrain, double power) {
    this.drivetrain = drivetrain;
    this.power = power;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetAllDriveEncoders();
    System.out.println("Start drive");
    this.isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // Drive forward when it gets a target.
    if(drivetrain.pixyAlgo.getPixyBest() != null){
      if (drivetrain.pixyAlgo.getPixyBest().getArea() <= Constants.DriveTrainConstants.pixyTargetArea){
        drivetrain.drive(power, 0, 0, false);
      }
      else{
        // Don't drive if it doesn't have a target.
        drivetrain.drive(0, 0, 0, false);
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
    System.out.println ("End drive");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.isFinished;
  }
}
