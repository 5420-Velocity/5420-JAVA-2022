// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Lift;
import frc.robot.utils.StateList;

public class SemiAutoClimb extends CommandBase {
  // extends limitSwitch
  private StateList<Boolean> stateList = StateList.bool(3);

private Lift lift;
private Joystick joystick;

// public class SemiAutoClimb {
//   private boolean isHooked;

// }

  public SemiAutoClimb(Lift lift, Double power, boolean isHooked, Joystick joystick) {
      // Use addRequirements() here to declare subsystem dependencies.
    
      //if the boolean for the limitSwitch is true, then set lift motor power ()
    // plugged into DIO 5
    
  //    if (isHooked = true) {
  //     this.lift.setRotationPower(1);
  //     this.lift.setMotorPower(1);
  //   }t
  //     else{
  //       this.lift.setRotationPower(0);
  //       this.lift.setMotorPower(0);
  //     }
  }

 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.lift.setRotationPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.isFinished();
  }
}
