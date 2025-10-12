// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants.shooterConstants.MotorShooterBaseConstants;

public class ShooterBaseAling extends SubsystemBase {
  private final SparkMax motorBase;

  private final RelativeEncoder baseEncoder;
  private final SparkClosedLoopController basePIDController;

  public ShooterBaseAling() {
    motorBase = new SparkMax(MotorShooterBaseConstants.kMotorId, MotorType.kBrushless);

    baseEncoder = motorBase.getAlternateEncoder();
    basePIDController = motorBase.getClosedLoopController();

    initializeBaseMotor();
    resetEncoder();
  }

  private void initializeBaseMotor() 
  {
    SparkMaxConfig glConfig = new SparkMaxConfig();

    glConfig.encoder
      .velocityConversionFactor(MotorShooterBaseConstants.kGearRatio)
      .positionConversionFactor(MotorShooterBaseConstants.kGearRatio);

    glConfig
      .smartCurrentLimit(MotorShooterBaseConstants.kCurrentLimitMotor)
      .idleMode(MotorShooterBaseConstants.kIdleMode)
      .inverted(false);

    // PID setup
    glConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(MotorShooterBaseConstants.kp)
      .i(MotorShooterBaseConstants.ki)
      .d(MotorShooterBaseConstants.kd)
      .velocityFF(0.0)
      .maxOutput(1)
      .minOutput(-1);

    motorBase.configure(glConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    ////////////////////////////////////////////////
    //***Measurements Shooter to smart dashboard**//
    ////////////////////////////////////////////////
    
    SmartDashboard.putNumber("shooterMeasurements/AlingMotor/kPosition", getEncoderValue());
    SmartDashboard.putNumber("shooterMeasurements/AlingMotor/kCurrent", getMotorCurrent());
  }

  /***
   * Get encoder value
   * @return from degress
   */
  public double getEncoderValue()
  {
    return baseEncoder.getPosition();
  }
  /** 
   * get Motor Current
   * @return
   */
  public double getMotorCurrent()
  {
    return motorBase.getOutputCurrent();
  }

  /**
   * Reset motor encoder 
   */
  public void resetEncoder()
  {
    baseEncoder.setPosition(0.0);
  }

  /**
   * Set a Pid position 
   * @param targetPose from degrees
   */
  public void setPIDPosition(double targetPose)
  {
    basePIDController.setReference(targetPose, ControlType.kPosition);
  }

  /** 
   * Stop Motor Base Shooter
   */
  public void stopMotor()
  {
    motorBase.set(0);
  }

  /** 
   * Set a output to motor base
   * 
   * @param output -1 up to 1 
   */
  public void setOutput(double output)
  {
    motorBase.set(output);
  }
}
