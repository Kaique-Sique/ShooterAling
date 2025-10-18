// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

// REV imports
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.SubsystemConstants.shooterConstants.ShooterBigWheelConstans;
import frc.robot.Constants.SubsystemConstants.shooterConstants.ShooterSmallWheelConstants;
import frc.robot.Utils.Conversions;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
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

  public ShooterSubsystem() {
    // Motors Instance
    MotorShooterBigWheel = new SparkMax(ShooterBigWheelConstans.kShooterMotorID, MotorType.kBrushless);
    ShooterEncoderBigWheel = MotorShooterBigWheel.getEncoder();
    shooterBigWheelPID = MotorShooterBigWheel.getClosedLoopController();

    MotorFollowerBigWheel = new SparkMax(ShooterBigWheelConstans.kFollowerMotorID, MotorType.kBrushless);
    followerEncoderBigWheel = MotorFollowerBigWheel.getEncoder();
    followerBigWheelPID = MotorFollowerBigWheel.getClosedLoopController();

    MotorShooterSmallWheel = new SparkMax(ShooterSmallWheelConstants.kMotorID, MotorType.kBrushless);
    ShooterEncoderSmallWheel = MotorShooterSmallWheel.getEncoder();
    shooterSmallWheelPID = MotorShooterSmallWheel.getClosedLoopController();

    // initialize motors config
    initializeBigWheelMotors();
    initializeSmallWheelsMotors();
  }

  /**
   * @ initialize motor of small wheel
   */
  private void initializeSmallWheelsMotors() {
    SparkMaxConfig globalConfig = new SparkMaxConfig();

    // Encoder setup
    globalConfig.encoder
        .velocityConversionFactor(ShooterSmallWheelConstants.kGearRatio)
        .positionConversionFactor(ShooterSmallWheelConstants.kGearRatio);

    // Motor config
    globalConfig
        .smartCurrentLimit(ShooterSmallWheelConstants.kCurrentLimitMotor)
        .idleMode(ShooterSmallWheelConstants.kIdleMode)
        .inverted(false);

    // PID setup
    globalConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(ShooterSmallWheelConstants.kp)
        .i(ShooterSmallWheelConstants.ki)
        .d(ShooterSmallWheelConstants.kd)
        .velocityFF(0.0)
        .maxOutput(1)
        .minOutput(-1);

    MotorShooterSmallWheel.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    ShooterEncoderSmallWheel.setPosition(0);
  }

  /**
   * @ initialize motors of big wheel
   */
  private void initializeBigWheelMotors() {
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig invertedConfig = new SparkMaxConfig();

    // Encoder setup
    globalConfig.encoder
        .velocityConversionFactor(ShooterBigWheelConstans.kGearRatioMotor)
        .positionConversionFactor(ShooterBigWheelConstans.kGearRatioMotor);

    // Motor config
    globalConfig
        .smartCurrentLimit(ShooterBigWheelConstans.kMotorCurrentLimit)
        .idleMode(ShooterBigWheelConstans.kIdleMode)
        .inverted(false);

    // PID setup
    globalConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(ShooterBigWheelConstans.kp)
        .i(ShooterBigWheelConstans.ki)
        .d(ShooterBigWheelConstans.kd)
        .velocityFF(0.0)
        .maxOutput(1)
        .minOutput(-1);

    invertedConfig
        .apply(globalConfig)
        .inverted(true);
    // Reset and persist parameters
    MotorShooterBigWheel.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    MotorFollowerBigWheel.configure(invertedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    ShooterEncoderBigWheel.setPosition(0);
    followerEncoderBigWheel.setPosition(0);
  }

  /**
   * Set a pid speed in RPM to both wheels
   * 
   * @param speed RPM
   */
  public void setRPM(double speed) {
    shooterBigWheelPID.setReference(speed, ControlType.kVelocity);
    followerBigWheelPID.setReference(speed, ControlType.kVelocity);

    shooterSmallWheelPID.setReference(speed, ControlType.kVelocity);
  }

  /**
   * Set a pid controler speed to MPS of both wheels
   * 
   * @param mps setpoint in mps
   */
  public void setMPS(double mps) {
    double rpsSmallWheel;
    double rpsBigWheel;

    rpsSmallWheel = Conversions.MPSToRPS(mps,
        ShooterSmallWheelConstants.kDiameterWheel,
        ShooterSmallWheelConstants.kGearRatio);

    rpsBigWheel = Conversions.MPSToRPS(mps,
        ShooterBigWheelConstans.kDiameterBigWheel,
        ShooterBigWheelConstans.kGearRatioMotor);

    shooterBigWheelPID.setReference(rpsBigWheel * 60, ControlType.kVelocity);
    followerBigWheelPID.setReference(rpsBigWheel * 60, ControlType.kVelocity);

    shooterSmallWheelPID.setReference(rpsSmallWheel * 60, ControlType.kVelocity);
  }

  /**
   * Stop all motors (brake)
   */
  public void stopMotors() {
    MotorFollowerBigWheel.set(0);
    MotorShooterBigWheel.set(0);

    MotorShooterSmallWheel.set(0);
  }

  @Override
  public void periodic() {
    ////////////////////////////////////////////////
    // ***Measurements Shooter to smart dashboard**//
    ////////////////////////////////////////////////

    /// Big Wheel Motor
    SmartDashboard.putNumber("shooterMeasurements/BigWheelMotor/kPosition", getBigWheelEncoderPose());
    SmartDashboard.putNumber("shooterMeasurements/BigWheelMotor/kVelocity", getBigWheelEncoderSpeed());
    SmartDashboard.putNumber("shooterMeasurements/BigWheelMotor/kCurrent", getBigWheelMotorCurrent());

    // Small Wheel Motor
    SmartDashboard.putNumber("shooterMeasurements/SmallWheelMotor/kPosition", getBigWheelEncoderPose());
    SmartDashboard.putNumber("shooterMeasurements/SmallWheelMotor/kVelocity", getBigWheelEncoderSpeed());
    SmartDashboard.putNumber("shooterMeasurements/SmallWheelMotor/kCurrent", getSmallWheelMotorCurrent());

    // Follower Motor
    SmartDashboard.putNumber("shooterMeasurements/FollowerMotor/kPosition", getFollowerEncoderPose());
    SmartDashboard.putNumber("shooterMeasurements/FollowerMotor/kVelocity", getFollowerEncoderSpeed());
    SmartDashboard.putNumber("shooterMeasurements/FollowerMotor/kCurrent", getSmallWheelMotorCurrent());
  }

  /**
   * get encoder position small wheel shooter
   * 
   * @return position degress
   */
  public double getSmallWheelEncoderPose() {
    return ShooterEncoderSmallWheel.getPosition();
  }

  /**
   * get encoder speed small wheel shooter
   * 
   * @return Speed from rpm
   */
  public double getSmallWheelEncoderSpeed() {
    return ShooterEncoderSmallWheel.getVelocity();
  }

  /**
   * Get Motor Current
   * 
   * @return
   */
  public double getSmallWheelMotorCurrent() {
    return MotorShooterSmallWheel.getOutputCurrent();
  }

  /**
   * get encoder position small wheel shooter
   * 
   * @return position degress
   */
  public double getBigWheelEncoderPose() {
    return ShooterEncoderBigWheel.getPosition();
  }

  /**
   * get encoder speed small wheel shooter
   * 
   * @return Speed from rpm
   */
  public double getBigWheelEncoderSpeed() {
    return ShooterEncoderBigWheel.getVelocity();
  }

  /**
   * Get Motor Current
   * 
   * @return
   */
  public double getBigWheelMotorCurrent() {
    return MotorShooterBigWheel.getOutputCurrent();
  }

  /**
   * get encoder position small wheel shooter
   * 
   * @return position degress
   */
  public double getFollowerEncoderPose() {
    return followerEncoderBigWheel.getPosition();
  }

  /**
   * get encoder speed small wheel shooter
   * 
   * @return Speed from rpm
   */
  public double getFollowerEncoderSpeed() {
    return followerEncoderBigWheel.getVelocity();
  }

  /**
   * Get Motor Current
   * 
   * @return
   */
  public double getFollowerMotorCurrent() {
    return MotorFollowerBigWheel.getOutputCurrent();
  }

  public void resetEncoders() {
    ShooterEncoderBigWheel.setPosition(0);
    followerEncoderBigWheel.setPosition(0);

    ShooterEncoderSmallWheel.setPosition(0);
  }
}
