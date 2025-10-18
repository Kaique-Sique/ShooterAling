// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooterCmds;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.CapoShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CapoAlingCmd extends Command {
  private final CapoShooterSubsystem capoSubsystem = RobotContainer.capoShooter;

  double targetPose;
  boolean finished = false;

  public CapoAlingCmd() {
    //get subsystem requirements
    addRequirements(capoSubsystem);

    //get target pose
    targetPose = SmartDashboard.getNumber("shooterMeasurements/CapoMotor/targetPose", 0.0);

    //set finish to zero
    finished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    finished = false;

    //capo subsystem pid pose
    capoSubsystem.setTargetPosePID(targetPose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    capoSubsystem.setTargetPosePID(targetPose);

    // if capo is in position, finish command
    if (capoSubsystem.getEncoderValue() + 0.1 >= targetPose && 
    capoSubsystem.getEncoderValue() - 0.1 <= targetPose) 
    {
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    //continue holding the motor on position
    capoSubsystem.setTargetPosePID(targetPose);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //call end when finished = true
    return finished;
  }
}
