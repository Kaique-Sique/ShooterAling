// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooterCmds;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.BallCatcherSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Shoot extends SequentialCommandGroup {
  public final BallCatcherSubsystem ballCatcher = RobotContainer.ballCatcher;
  public final ShooterSubsystem shooterSubsys = RobotContainer.shooterSubsys;
  public Shoot() {
    
    addCommands(
        new CapoAlingCmd(),
        new RunCommand(()-> shooterSubsys.setMPS(580), shooterSubsys).withTimeout(1),
        new RunCommand(()-> ballCatcher.setMPSTarget(5600), ballCatcher).withTimeout(0.8)
        .andThen(new InstantCommand(()-> ballCatcher.stopMotor(), ballCatcher)),

        new WaitCommand(1.2),

        new InstantCommand(()-> shooterSubsys.stopMotors(), shooterSubsys)
    );
    
  }
}
