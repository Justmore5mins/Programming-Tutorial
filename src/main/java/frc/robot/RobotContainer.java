// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Drivetrain.Constants;
import frc.robot.Drivetrain.DataLog;

public class RobotContainer {
  public Drivetrain drivetrain = Drivetrain.getInstance();
  public CommandXboxController controller = new CommandXboxController(0);
  

  public RobotContainer() {
    new DataLog();
    drivetrain.setDefaultCommand(drivetrain.drive(
      () -> Constants.MaxVelocity.times(controller.getLeftX()),
      () -> Constants.MaxVelocity.times(controller.getLeftY()),
      () -> Constants.MaxOmega.times(controller.getRightX())
    ));
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
