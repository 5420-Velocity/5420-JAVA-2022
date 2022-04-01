package frc.robot.Subsystems;

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.I2C;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ColorTargets;

public class Intake extends SubsystemBase {
  
  private WPI_TalonFX _intakeMotor = new WPI_TalonFX(11);
  private WPI_TalonSRX _releaseMotor = new WPI_TalonSRX(52);
  private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  private NetworkTableEntry hasBall = SmartDashboard.getEntry("has ball");
	private final ColorMatch colorMatch = new ColorMatch();


  public Intake() {
    this.colorMatch.addColorMatch(ColorTargets.COLOR_RED);
    this.colorMatch.addColorMatch(ColorTargets.COLOR_BLUE);
  }

  public void setIntakePower(double power){
    _intakeMotor.set(power);
  }


  public void setReleasePower(double power){
    _releaseMotor.set(power);
  }

  public ColorMatchResult getColor() {
    return this.colorMatch.matchClosestColor(colorSensor.getColor());
  }

  @Override
  public void periodic() {
    Color cvalue = colorSensor.getColor();

    //prints out color to dashboard
    hasBall.setBoolean(getColor().confidence > 0.9);
  }
}
