// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.fasterxml.jackson.databind.jsonFormatVisitors.JsonObjectFormatVisitor;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Lift;
import frc.robot.utils.StateList;

public class SemiAutoClimb extends CommandBase {
  // extends limitSwitch
  private StateList<Boolean> stateList = StateList.bool(3);
private boolean isHooked;
private Lift lift;
private Joystick joystick;


// public class SemiAutoClimb {
//   private boolean isHooked;

// }

//can get rid of joystick because this will be added in robotContainer as (.whileHeld)?
  public SemiAutoClimb(Lift lift, Double power, boolean isHooked, Joystick joystick) {
    this.lift = lift;
    this.joystick = joystick;

  }

 

  public SemiAutoClimb(Lift m_lift, int i) {
}
//ur mom


// Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isHooked = true){
      this.lift.setRotationPower(1);
      this.lift.setRotationPower(1);
      System.out.println("SemiAutoClimb is active");
    }
    else {
      this.lift.setRotationPower(0);
      this.lift.setRotationPower(0);
      System.out.print("SemiAutoClimb is innactive");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.lift.setRotationPower(0);
    System.out.println("stopped climbing, boolean interrupted");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("stopped climbing");
    return this.isFinished();
  }
}
