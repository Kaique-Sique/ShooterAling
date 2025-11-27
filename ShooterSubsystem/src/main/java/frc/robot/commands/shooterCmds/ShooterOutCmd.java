// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooterCmds;

import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SubsystemsConstants.FieldPoses;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.RobotPoseOnTheField;
//import frc.robot.utils.FieldPoses;
//import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterOutCmd extends Command {
  private final ShooterSubsystem mShooterSubsystem = RobotContainer.shooterSubsys;
  private final RobotPoseOnTheField mRobotPose = RobotContainer.robotPoseOnTheField;

  double out;
  boolean finished = false;

  public ShooterOutCmd() 
  {
    finished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    finished = false;
    out = getShooterOut();
    mShooterSubsystem.setOutput(out);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    finished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }

  public double getShooterOut()
  {
    // Get swerve Positions
    Pose2d robotPose2d = mRobotPose.getRobotPose();

    // Calc relativa hip from robot and target
    double hip = Math.sqrt(
           Math.pow(
                Math.abs(robotPose2d.getX() - FieldPoses.redPoses.tag10.getX()), 2)
                      + Math.pow(
                    Math.abs(robotPose2d.getY() - FieldPoses.redPoses.tag10.getY()), 2));


    //***Gets shooter target dist from smartdashboard****
    //Important: make sure the value is in meters
    //double hip = SmartDashboard.getNumber("shooterMeasurements/hipDistance", 0.0);

    return 0.123 *  Math.log(hip) + 0.145;
  }
}