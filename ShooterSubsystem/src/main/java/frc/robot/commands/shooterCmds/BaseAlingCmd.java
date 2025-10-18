// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooterCmds;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.ShooterBaseAlingSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BaseAlingCmd extends Command {
  private final ShooterBaseAlingSubsystem mBaseAling = RobotContainer.baseShooter;

  private double targetPose;
  public BaseAlingCmd() {
    addRequirements(mBaseAling);
  }

  @Override
  public void initialize() 
  {
    targetPose = SmartDashboard.getNumber("shooterMeasurements/AlingMotor/targetPose", 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    targetPose = SmartDashboard.getNumber("shooterMeasurements/AlingMotor/targetPose", 0.0);
    mBaseAling.setPIDPosition(targetPose);
  }

  @Override
  public void end(boolean interrupted) 
  {
    mBaseAling.setPIDPosition(targetPose);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
