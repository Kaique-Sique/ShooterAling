// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemsConstants.DriveConstants;
import frc.robot.Limelight.LimelightHelpers;

public class RobotPoseOnTheField extends SubsystemBase {
  Pose2d robotPose2d;
  boolean isCameraPoseSetted = false;

  public RobotPoseOnTheField() {
  }

  @Override
  public void periodic() 
  {
    if(!isCameraPoseSetted)
    {
      setCameraPoseOnRobot();
      isCameraPoseSetted = true;
    }
    else 
    {
      Pose2d currentPose = limeLightMt1();

      if(currentPose != null)
      {
        robotPose2d = currentPose;
      }
    }
  }

  public Pose2d getRobotPose() {
    return robotPose2d;
  }

  public void setCameraPoseOnRobot() 
  {  
    LimelightHelpers.setCameraPose_RobotSpace(DriveConstants.limelightFrontLeft,0.110,0.259,0.410,0,0,0);
    LimelightHelpers.SetIMUMode(DriveConstants.limelightFrontLeft, 2);
  }

  private Pose2d limeLightMt1() {
    Pose2d robotPose2d = new Pose2d();

    boolean doRejectUpdate = false;
    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      
      if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
      {
        if(mt1.rawFiducials[0].ambiguity > .7)
        {
          doRejectUpdate = true;
        }
        if(mt1.rawFiducials[0].distToCamera > 3)
        {
          doRejectUpdate = true;
        }
      }
      if(mt1.tagCount == 0)
      {
        doRejectUpdate = true;
      }

      if(!doRejectUpdate)
      {
        robotPose2d = mt1.pose;
        return robotPose2d;
      }
      else
      {
        return null;
      }
  }
}
