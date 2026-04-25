// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public SparkMax FrontLeft, FrontRight, BackLeft, BackRight;
  public SparkMaxConfig FrontLeftConfig, FrontRightConfig, BackLeftConfig, BackRightConfig;
  public AHRS gyro;

  private final RobotContainer m_robotContainer;

  public Robot() {
    FrontLeft = new SparkMax(1, MotorType.kBrushless);
    FrontRight = new SparkMax(2, MotorType.kBrushless);
    BackLeft = new SparkMax(3, MotorType.kBrushless);
    BackRight = new SparkMax(4, MotorType.kBrushless);

    gyro = new AHRS(NavXComType.kMXP_SPI);

    FrontLeftConfig = new SparkMaxConfig();
    FrontRightConfig = new SparkMaxConfig();
    BackLeftConfig = new SparkMaxConfig();
    BackRightConfig = new SparkMaxConfig();

    FrontLeftConfig
      .idleMode(IdleMode.kBrake)
      .inverted(false);
    FrontRightConfig
      .idleMode(IdleMode.kBrake)
      .inverted(true);
    BackLeftConfig
      .follow(FrontLeft);
    BackRightConfig
      .follow(FrontRight);

    FrontLeft.configure(FrontLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
