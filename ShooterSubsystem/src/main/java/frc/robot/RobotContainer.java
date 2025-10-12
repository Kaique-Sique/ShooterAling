// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants.JoystickDriverConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RobotContainer {
  public static final ShooterSubsystem shooterSubsys = new ShooterSubsystem();

  private final CommandXboxController m_driverController =
      new CommandXboxController(JoystickDriverConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
