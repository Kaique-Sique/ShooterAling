// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.SubsystemConstants.shooterConstants;
import frc.robot.Utils.Conversions;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shooterSubsystem extends SubsystemBase {
  // MotorBig wheel motors configuration
  private final SparkMax MotorShooterBigWheel;
  private final SparkMax MotorFollowerBigWheel;

  private final RelativeEncoder ShooterEncoderBigWheel;
  private final SparkClosedLoopController shooterBigWheelPID;

  private final RelativeEncoder followerEncoderBigWheel;
  private final SparkClosedLoopController followerBigWheelPID;

  // Small wheel configuration
  private final SparkMax MotorShooterSmallWheel;

  private final RelativeEncoder ShooterEncoderSmallWheel;
  private final SparkClosedLoopController shooterSmallWheelPID;

  public shooterSubsystem() {
    //Motors Instance 
    MotorShooterBigWheel = new SparkMax(shooterConstants.kShooterBigWheelID, MotorType.kBrushless);
    ShooterEncoderBigWheel = MotorShooterBigWheel.getEncoder();
    shooterBigWheelPID = MotorShooterBigWheel.getClosedLoopController();

    MotorFollowerBigWheel = new SparkMax(0, MotorType.kBrushless);
    followerEncoderBigWheel = MotorFollowerBigWheel.getEncoder();
    followerBigWheelPID = MotorFollowerBigWheel.getClosedLoopController();

    MotorShooterSmallWheel = new SparkMax(0, MotorType.kBrushless);
    ShooterEncoderSmallWheel = MotorShooterSmallWheel.getEncoder();
    shooterSmallWheelPID = MotorShooterSmallWheel.getClosedLoopController();

    //initialize motors config
    initializeBigWheelMotors();
    initializeSmallWheelsMotors();
  }

  /**
   * @ initialize motor of small wheel
   */
  private void initializeSmallWheelsMotors() 
  {

  }

  /**
   * @ initialize motors of big wheel
   */
  private void initializeBigWheelMotors() 
  {

  }

  /**
   * Set a pid speed in RPM to both wheels
   * @param speed RPM 
   */
  public void setRPM(double speed)
  {
    shooterBigWheelPID.setReference(speed, ControlType.kVelocity);
    followerBigWheelPID.setReference(speed, ControlType.kVelocity);

    shooterSmallWheelPID.setReference(speed, ControlType.kVelocity);
  }

  /**
   * Set a pid controler speed to MPS of both wheels
   * @param mps setpoint in mps
   */
  public void setMPS(double mps)
  {
    double rpsSmallWheel;
    double rpsBigWheel;

    rpsSmallWheel = Conversions.MPSToRPS(mps, shooterConstants.kDiameterSmallWheel, shooterConstants.kGearRatioMotorSmallWheel);
    rpsBigWheel = Conversions.MPSToRPS(mps, shooterConstants.kDiameterBigWheel, shooterConstants.kGearRatioMotorBigWheel);

    shooterBigWheelPID.setReference(rpsBigWheel * 60, ControlType.kVelocity);
    followerBigWheelPID.setReference(rpsBigWheel * 60, ControlType.kVelocity);

    shooterSmallWheelPID.setReference(rpsSmallWheel * 60, ControlType.kVelocity); 
  }

  @Override
  public void periodic() 
  {
    ////////////////////////////////////////////////
    //***Measurements Shooter to smart dashboard**//
    ////////////////////////////////////////////////
     
    /// Big Wheel Motor
    SmartDashboard.putNumber("shooterMeasurements/BigWheel/kPosition", getBigWheelEncoderPose());
    SmartDashboard.putNumber("shooterMeasurements/BigWheel/kVelocity", getBigWheelEncoderSpeed());

    //Small Wheel Motor
    SmartDashboard.putNumber("shooterMeasurements/SmallWheel/kPosition", getBigWheelEncoderPose());
    SmartDashboard.putNumber("shooterMeasurements/SmallWheel/kVelocity", getBigWheelEncoderSpeed());

    //Follower Motor
    SmartDashboard.putNumber("shooterMeasurements/Follower/kPosition", getFollowerEncoderPose());
    SmartDashboard.putNumber("shooterMeasurements/Follower/kVelocity", getFollowerEncoderSpeed());
  }

  /**
   * get encoder position small wheel shooter
   * @return position degress
   */
  public double getSmallWheelEncoderPose()
  {
    return ShooterEncoderSmallWheel.getPosition();
  }
  /**
   * get encoder speed small wheel shooter
   * @return Speed from rpm
   */
  public double getSmallWheelEncoderSpeed()
  {
    return ShooterEncoderSmallWheel.getVelocity();
  }

  /**
   * get encoder position small wheel shooter
   * @return position degress
   */
  public double getBigWheelEncoderPose()
  {
    return ShooterEncoderBigWheel.getPosition();
  }
  /**
   * get encoder speed small wheel shooter
   * @return Speed from rpm
   */
  public double getBigWheelEncoderSpeed()
  {
    return ShooterEncoderBigWheel.getVelocity();
  }

  /**
   * get encoder position small wheel shooter
   * @return position degress
   */
  public double getFollowerEncoderPose()
  {
    return followerEncoderBigWheel.getPosition();
  }
  /**
   * get encoder speed small wheel shooter
   * @return Speed from rpm
   */
  public double getFollowerEncoderSpeed()
  {
    return followerEncoderBigWheel.getVelocity();
  }
}
