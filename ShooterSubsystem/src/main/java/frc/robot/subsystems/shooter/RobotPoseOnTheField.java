// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemsConstants.DriveConstants;
import frc.robot.Limelight.LimelightHelpers;

public class RobotPoseOnTheField extends SubsystemBase {
  Pose2d robotPose2d;
  boolean isCameraPoseSetted = false;

  /**
   * NetworkTables publisher for Pose2d data.
   * Useful for debugging and visualization in tools like Shuffleboard or custom dashboards.
   * Advantage Scope: Can visualize robot pose in real-time.
   */
  private StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
                                                                  .getStructTopic("MyPose", Pose2d.struct)
                                                                  .publish();
  
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
      Pose2d currentPose = limeLightMt2();

      if(currentPose != null)
      {
        robotPose2d = currentPose;
      }
    }
    publisher.set(robotPose2d);
  }

  public Pose2d getRobotPose() {
    return robotPose2d;
  }

  public void setCameraPoseOnRobot() 
  {  
    LimelightHelpers.setCameraPose_RobotSpace(DriveConstants.limelightFrontLeft,0.155,-0.14,0.27,0,0,0);
    LimelightHelpers.SetIMUMode(DriveConstants.limelightFrontLeft, 2);
  }

  private Pose2d limeLightMt1() {
    Pose2d robotPose2d = new Pose2d();

    boolean doRejectUpdate = false;
    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(DriveConstants.limelightFrontLeft);
    if(mt1 == null) return null;

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

  private Pose2d limeLightMt2() {

    boolean doRejectUpdate = false;
      LimelightHelpers.SetRobotOrientation(DriveConstants.limelightFrontLeft, 0, 0, 0, 0, 0, 0);
  LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(DriveConstants.limelightFrontLeft);
  if(mt2 == null) return null;
  
  if(mt2.tagCount == 0)
  {
    doRejectUpdate = true;
  }
  if(!doRejectUpdate)
  {
    return mt2.pose;
  }
  else return null;
}
}

