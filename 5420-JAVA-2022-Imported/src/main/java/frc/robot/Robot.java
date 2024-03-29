/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.AutoReset;
import frc.robot.Commands.IntakeRelease;

public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_robotContainer.m_limelight.setLedMode(1);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    m_robotContainer.m_limelight.setLedMode(1);
  }

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();

    new IntakeRelease(m_robotContainer.m_intake).schedule();
    new AutoReset(m_robotContainer.m_swerve).schedule();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
        m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    m_robotContainer.m_lift.ZeroLiftEncoder();
    CommandScheduler.getInstance().cancelAll();

    m_robotContainer.m_swerve.initialize();
  }

  @Override
  public void teleopPeriodic() {
    m_robotContainer.teleopExecute();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.m_limelight.setLedMode(0);
  }

  @Override
  public void testPeriodic() {
  }

}
