/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private WPI_TalonFX m_shootMotor = new WPI_TalonFX(Constants.ShooterConstants.shooterMotor1);
  private WPI_TalonFX m_shootMotor2 = new WPI_TalonFX(Constants.ShooterConstants.shooterMotor2);
  private WPI_TalonSRX m_feedMotor = new WPI_TalonSRX(Constants.ShooterConstants.feedMotor);
  private WPI_TalonSRX m_feedMotor2 = new WPI_TalonSRX(53);
  private AnalogInput m_rangeInput = new AnalogInput(0);
  private AnalogPotentiometer m_rangeSensor = new AnalogPotentiometer(m_rangeInput);
  private NetworkTableEntry RangeSensor = SmartDashboard.getEntry("RangeSensor");
  private NetworkTableEntry ShooterRPM = SmartDashboard.getEntry("Shooter RPM");

  public Shooter() {
    m_shootMotor.configFactoryDefault();
    m_feedMotor.configFactoryDefault();
    
    m_shootMotor.config_kP(0, 0.05);
    m_shootMotor2.config_kP(0, 0.05);

    
    m_shootMotor.config_kD(0, 0.002);
    m_shootMotor2.config_kD(0, 0.002);

    m_shootMotor.config_kF(0, 0);
    m_shootMotor2.config_kF(0, 0);
  }



  public void setShooterPower(double power){
    m_shootMotor.set(power);
    m_shootMotor2.set(-power);
  }

  public void setShooterVelocity(double value){
    m_shootMotor.set(ControlMode.Velocity, value);
    m_shootMotor2.set(ControlMode.Velocity, -value);
  }

  public void setFeedPower(double power){
    m_feedMotor.set(-power);
    m_feedMotor2.set(power);
  }

  public double GetShooterRPM(){
    //convert to rpm 
    return (m_shootMotor.getSensorCollection().getIntegratedSensorVelocity()/2048)*600;
  }

  @Override
  public void periodic() {
    ShooterRPM.setDouble(GetShooterRPM());
    RangeSensor.setDouble(m_rangeSensor.get());
  }
  public double getRange(){
    return m_rangeSensor.get();
  }
}
