// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooterCmds;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.BallCatcherSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootCmd extends SequentialCommandGroup {
  //Subsystens used in this command instance
  private final ShooterSubsystem m_ShooterSubsystem = RobotContainer.shooterSubsys;
  private final BallCatcherSubsystem m_BallCatcherSubsystem = RobotContainer.ballCatcher;
  public ShootCmd() {
    addCommands(
      // Initialize Shooter Motors
      new InstantCommand(()-> m_ShooterSubsystem.setOutput(0.2), m_ShooterSubsystem),

      // Wait a time to shooter get his target speed
      new WaitCommand(1),

      // Initializa ball Catcher
      new InstantCommand(()-> m_BallCatcherSubsystem.setSpeedTarget(5200), m_BallCatcherSubsystem),

      // Wait a time to robot stop to shoot
      new WaitCommand(1),

      // Stop all motors
      new InstantCommand(()-> m_BallCatcherSubsystem.stopMotor(), m_BallCatcherSubsystem)
        .andThen(()-> m_ShooterSubsystem.stopMotors(), m_ShooterSubsystem)
    );
  }
}
