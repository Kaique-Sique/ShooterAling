// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooterCmds;

// FRC WPI Imports
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SubsystemsConstants.FieldPoses;
import frc.robot.RobotContainer;
// Robot Imports
import frc.robot.subsystems.shooter.RobotPoseOnTheField;
import frc.robot.subsystems.shooter.ShooterBaseAlingSubsystem;


public class BasePositionAling extends Command {
  // Base shooter subsystem instance 
  private final ShooterBaseAlingSubsystem baseShooter = RobotContainer.baseShooter;
  private final RobotPoseOnTheField robotPose = RobotContainer.robotPoseOnTheField;

  // Finish state
  boolean isFinished = false;

  double targetPose;
  //Encoder Position
  double baseEncoderPosition;

  Rotation2d baseRotation2d;
  Rotation2d rotRobot;

  Pose2d robotPose2d;

  public BasePositionAling() 
  {
    // Set is finished to false
    isFinished = false;

    // Get Subsystem requirements
    addRequirements(baseShooter);
  }

  @Override
  public void initialize() 
  {
    // Initialize isFinished to false
    isFinished = false;

    //Update TargetPose
    targetPose = getTargetPose();

    // Set PID to position targetPose
    baseShooter.setPIDPosition(targetPose);
  }

  @Override
  public void execute() 
  {
    //Update Encoder position
    baseEncoderPosition = baseShooter.getEncoderValue();

    // Set PID to position targetPose
    baseShooter.setPIDPosition(getTargetPose());
  }

  @Override
  public void end(boolean interrupted) 
  {
    // Hold Motors on position target if command stop
    baseShooter.setPIDPosition(targetPose);
  }

  @Override
  public boolean isFinished() 
  {
    //if isFinished == true, call for end function
    return isFinished;
  }

  /**
   * Calc arcTan from robot dist of tag
   */
  public double getTargetPose()
  {
    //GetRobotPose
    robotPose2d = robotPose.getRobotPose();
    if (robotPose2d == null) return 0.0;

    // Get Target Pose based on swerve 
    targetPose = Units.radiansToDegrees(
      Math.atan2((FieldPoses.redPoses.tag07.getY() - robotPose2d.getY()), 
            (FieldPoses.redPoses.tag07.getX() - robotPose2d.getX())));
  
    return targetPose;
  } 
}
