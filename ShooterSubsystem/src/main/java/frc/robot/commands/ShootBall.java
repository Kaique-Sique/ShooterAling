// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootBall extends SequentialCommandGroup {
  private final ShooterSubsystem shooterSubsys = RobotContainer.shooterSubsys;
  public ShootBall() {
    addRequirements(shooterSubsys);
    addCommands(
      new RunCommand((()-> shooterSubsys.setMPS(1)), shooterSubsys),
      new WaitCommand(3),
      new InstantCommand((()-> shooterSubsys.stopMotors()), shooterSubsys)
    );
  }
}
