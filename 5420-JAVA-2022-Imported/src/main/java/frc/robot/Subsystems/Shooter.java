/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private WPI_TalonFX m_shootMotor = new WPI_TalonFX(Constants.ShooterConstants.shooterMotor1);
  private WPI_TalonFX m_shootMotor2 = new WPI_TalonFX(Constants.ShooterConstants.shooterMotor2);
  private WPI_TalonSRX m_feedMotor = new WPI_TalonSRX(Constants.ShooterConstants.feedMotor);
  private WPI_TalonSRX m_feedMotor2 = new WPI_TalonSRX(53);

  public Shooter() {
    m_shootMotor.configFactoryDefault();
    m_feedMotor.configFactoryDefault();
  }

  public void setShooterPower(double power){
    m_shootMotor.set(power);
    m_shootMotor2.set(-power);
  }

  public void setFeedPower(double power){
    m_feedMotor.set(-power);
    m_feedMotor2.set(power);
  }

  public double GetShooterRPM(){
    return m_shootMotor.getSensorCollection().getIntegratedSensorVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
