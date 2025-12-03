// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemsConstants.shooterConstants.ShooterBaseConstants;
public class ShooterBaseAlingSubsystem extends SubsystemBase {
  private final SparkMax motorBase;

  private final RelativeEncoder baseEncoder;
  private final SparkClosedLoopController basePIDController;

  public boolean autoAling = false;

  public ShooterBaseAlingSubsystem() {
    motorBase = new SparkMax(ShooterBaseConstants.kMotorId, MotorType.kBrushless);

    baseEncoder = motorBase.getEncoder();
    basePIDController = motorBase.getClosedLoopController();

    SmartDashboard.putNumber("shooterMeasurements/AlingMotor/targetPose", 0.0);

    initializeBaseMotor();
    resetEncoder();
  }

  private void initializeBaseMotor() 
  {
    SparkMaxConfig glConfig = new SparkMaxConfig();

    glConfig.encoder
      .velocityConversionFactor(1)
      .positionConversionFactor(360 / ShooterBaseConstants.kGearRatio);

    glConfig
      .smartCurrentLimit(ShooterBaseConstants.kCurrentLimitMotor)
      .idleMode(ShooterBaseConstants.kIdleMode)
      .inverted(false);

    // PID setup
    glConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(ShooterBaseConstants.kp)
      .i(ShooterBaseConstants.ki)
      .d(ShooterBaseConstants.kd)
      .velocityFF(0.0)
      .maxOutput(1)
      .minOutput(-1);

      MAXMotionConfig maxMotion = glConfig.closedLoop.maxMotion;

      maxMotion
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot0)
        .maxAcceleration(34200 * 2, ClosedLoopSlot.kSlot0)
        .maxVelocity(34200, ClosedLoopSlot.kSlot0)

        .allowedClosedLoopError(0.85, ClosedLoopSlot.kSlot0);

      glConfig.softLimit
        .forwardSoftLimit(ShooterBaseConstants.kSoftLimitMax)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(ShooterBaseConstants.kSoftLimitMin)
        .reverseSoftLimitEnabled(true);

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
    basePIDController.setReference(targetPose, ControlType.kMAXMotionPositionControl);
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

  /**
   * Disable Shooter Base Auto Aling
   */
  public void disabledAutoAling()
  {
    autoAling = false;
    for(int x = 0; x < 3; x++)
    {
      System.out.println("Manual aling enable!!");
    }
  }

  /**
   * Enable Shooter Base Auto Aling
   */
  public void enableAutoAling()
  {
    autoAling = true;
  }

  /**
   * 
   * @return Auto Aling isTrue
   */
  public Boolean isEnableAutoAling()
  {
    return autoAling;
  }
}
