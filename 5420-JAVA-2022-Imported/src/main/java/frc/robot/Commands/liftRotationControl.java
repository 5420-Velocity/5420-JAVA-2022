// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Axis;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.LiftRotationMechanism;

public class liftRotationControl extends CommandBase {
  /** Creates a new liftRotationMechanismRotationControl. */
  private LiftRotationMechanism liftRotationMechanism;
  private Joystick joystick;
  private int axis;
  public liftRotationControl(LiftRotationMechanism liftRotationMechanism, Joystick joystick, int axis) {
    this.liftRotationMechanism = liftRotationMechanism;
    this.joystick = joystick;
    this.axis = axis;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.liftRotationMechanism.setRotationPower(joystick.getRawAxis(axis) * 0.6);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.liftRotationMechanism.setRotationPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

public static void set(double power) {
}
}
