// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooterCmds;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.BallCatcherSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootBall extends SequentialCommandGroup {
  private final ShooterSubsystem shooterSubsys = RobotContainer.shooterSubsys;
  private final BallCatcherSubsystem ballCatcher = RobotContainer.ballCatcher;
  public ShootBall() {
    addRequirements(shooterSubsys, ballCatcher);
    addRequirements(shooterSubsys);
    addCommands(
      new WaitCommand(0.3),
      new RunCommand(()-> shooterSubsys.setMPS(580), shooterSubsys),
      new WaitCommand(1),
      new RunCommand(()-> ballCatcher.setMPSTarget(5600), ballCatcher),
      new WaitCommand(2),
      new InstantCommand((()-> shooterSubsys.stopMotors()), shooterSubsys),
      new InstantCommand(()-> ballCatcher.stopMotor(), ballCatcher)
    );
  }
}
