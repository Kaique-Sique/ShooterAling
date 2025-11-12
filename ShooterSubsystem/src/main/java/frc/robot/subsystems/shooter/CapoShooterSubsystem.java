// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemsConstants.shooterConstants.CapoConstants;

public class CapoShooterSubsystem extends SubsystemBase {
  //Motor instance
  private final SparkMax capoMotor;

  //encoder instance
  private final RelativeEncoder mEncoder;
  //PID instance
  private final SparkClosedLoopController mController;

  public CapoShooterSubsystem() {
    capoMotor = new SparkMax(CapoConstants.kMotorId, MotorType.kBrushless);

    mEncoder = capoMotor.getEncoder();
    mController = capoMotor.getClosedLoopController();

    //initialize Motor config
    intializeMotor();
    //reset encoder
    resetEncoder();

    //create a NT to target position
    //it will be use on a position Command
    SmartDashboard.putNumber("shooterMeasurements/CapoMotor/targetPose", 0.0);
  }

  /**
   * set encoder to zero degress
   */
  private void resetEncoder() {
    mEncoder.setPosition(0);
  }

  /** 
   * initialize motor config
   */
  private void intializeMotor() {
    SparkMaxConfig glConfig = new SparkMaxConfig();

    glConfig.encoder
      .velocityConversionFactor(1)
      .positionConversionFactor(CapoConstants.kGearRatio);

    glConfig
      .smartCurrentLimit(CapoConstants.kCurrentLimitMotor)
      .idleMode(CapoConstants.kIdleMode)
      .inverted(true);

    // PID setup
    glConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(CapoConstants.kp)
      .i(CapoConstants.ki)
      .d(CapoConstants.kd)
      .velocityFF(0.0)
      .maxOutput(1)
      .minOutput(-1);

      glConfig.softLimit
        .forwardSoftLimit(CapoConstants.kSoftLimitMax)
        .forwardSoftLimitEnabled(CapoConstants.kEnableSoftLimitMax)
        .reverseSoftLimit(CapoConstants.kSoftLimitMin)
        .reverseSoftLimitEnabled(CapoConstants.kEnableSoftLimitMin);

    MAXMotionConfig maxMotion = glConfig.closedLoop.maxMotion;

    maxMotion
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot0)
        .maxAcceleration(34200 * 2, ClosedLoopSlot.kSlot0)
        .maxVelocity(34200, ClosedLoopSlot.kSlot0)

        .allowedClosedLoopError(0.85, ClosedLoopSlot.kSlot0);


    capoMotor.configure(glConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    /** Put Motor Data on smartDashboard */
    SmartDashboard.putNumber("shooterMeasurements/CapoMotor/kPosition", getEncoderValue());
    SmartDashboard.putNumber("shooterMeasurements/CapoMotor/kCurrent", getMotorCurrent());
  }

  /**
   * get Motor current
   * @return output Current
   */
  private double getMotorCurrent() {
    return capoMotor.getOutputCurrent();
  }

  /**
   * get encoder value
   * @return value from degrees
   */
  public double getEncoderValue() {
    return mEncoder.getPosition();
  }

  /**
   * set a pid pose target
   * @param pose pose target from degress
   */
  public void setTargetPosePID(double pose)
  {
    mController.setReference(pose, ControlType.kMAXMotionPositionControl);
  }

  /** 
   * stop Motors
   * set output to zero
   */
  public void stopMotors()
  {
    capoMotor.set(0);
  }
}
