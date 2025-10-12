// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants.JoystickDriverConstants;
import frc.robot.commands.ShootBall;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RobotContainer {
  // Subsystens instance 
  public static final ShooterSubsystem shooterSubsys = new ShooterSubsystem();

  // Sequential Commands 
  public static final ShootBall m_shootBall = new ShootBall();

  private final CommandXboxController m_driverController =
      new CommandXboxController(JoystickDriverConstants.kDriverControllerPort);

  public RobotContainer() {
    m_shootBall.addRequirements(shooterSubsys);

    configureBindings();
  }

  private void configureBindings() 
  {
    m_driverController.a().onTrue(m_shootBall);
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
