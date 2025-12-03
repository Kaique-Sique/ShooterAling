// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooterCmds;

// FRC WPI Imports
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

// Robot Imports
import frc.robot.RobotContainer;
import frc.robot.Constants.SubsystemsConstants.FieldPoses;
import frc.robot.Constants.SubsystemsConstants.shooterConstants.CapoConstants;
import frc.robot.subsystems.shooter.CapoShooterSubsystem;
import frc.robot.subsystems.shooter.RobotPoseOnTheField;

public class CapoPositionAling extends Command {
  private final CapoShooterSubsystem m_CapoShooterSubsystem = RobotContainer.capoShooter;
  private final RobotPoseOnTheField robotPose = RobotContainer.robotPoseOnTheField;

  boolean finished = false;

  double targetPose;

  Pose2d robotPose2d;

  // CapoPositionAling contructor
  public CapoPositionAling() 
  {
    // Get subsystem requirements
    addRequirements(m_CapoShooterSubsystem);

    // Set finish to false
    finished = false;
  }

  @Override
  public void initialize() 
  {
    // Set finish to zero
    finished = false;

    // Multiplicate de dist for a constant 
    targetPose = getTargetDist();

    // Start capo to position with PID
    m_CapoShooterSubsystem.setTargetPosePID(targetPose);
  }

  @Override
  public void execute() 
  {
    // Capo to position with PID
    m_CapoShooterSubsystem.setTargetPosePID(getTargetDist());
  }

  @Override
  public void end(boolean interrupted) 
  {
    // When stop to use this command, hold motors on position
    m_CapoShooterSubsystem.setTargetPosePID(targetPose);

    for(int x = 0; x < 4; x++)
    {
      System.out.println("Dist: " + getTargetDist() 
      / CapoConstants.multiplicadorTargerDist);
    }
  }

  @Override
  public boolean isFinished() {
    // If's finish == true, call for void end
    return finished;
  }

  /**
   * Calc hip from robot dist to target
   * 
   * @return double targetPose
   */
  public double getTargetDist()
  {
    // Get swerve Positions
    robotPose2d = robotPose.getRobotPose();

    // its check if the robot already saw any tag
    if(robotPose2d == null) return 0.0;
    
    // Calc relativa hip from robot and target
    double hip = Math.sqrt(
           Math.pow(
                Math.abs(robotPose2d.getX() - FieldPoses.redPoses.tag07.getX()), 2)
      
                + Math.pow(
                    Math.abs(robotPose2d.getY() - FieldPoses.redPoses.tag07.getY()), 2));

    // Multiplicate de dist for a constant 
    targetPose = hip * CapoConstants.multiplicadorTargerDist;

    return targetPose;
  } 
}
