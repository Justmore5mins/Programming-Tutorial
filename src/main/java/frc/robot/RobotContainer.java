// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Drivetrain.Constants;

public class RobotContainer {
  public Drivetrain drivetrain = Drivetrain.getInstance();
  public CommandXboxController controller = new CommandXboxController(0);
  
  private SendableChooser<Command> autoChooser;

  public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    DogLog.setOptions(new DogLogOptions()
      .withCaptureConsole(true)
      .withCaptureDs(true)
      .withNtPublish(true)
      .withCaptureNt(true)
      .withLogExtras(true));
    DogLog.setPdh(drivetrain.PDP);

    drivetrain.setDefaultCommand(
      drivetrain.drive(
        () -> Constants.MaxVelocity.times(controller.getLeftX()),
        () -> Constants.MaxVelocity.times(controller.getLeftY()),
        () -> Constants.MaxOmega.times(controller.getRightX())
      )
    );

    configureBindings();
  }

	private void configureBindings() {}

	public Command getAutonomousCommand() {
		try {
      return autoChooser.getSelected();
		}
		catch(Exception e) {
			return Commands.print("No autonomous command configured");
		}
	}
}
