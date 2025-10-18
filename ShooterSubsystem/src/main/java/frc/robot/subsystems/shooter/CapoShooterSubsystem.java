// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants.shooterConstants.CapoConstants;

public class CapoShooterSubsystem extends SubsystemBase {
  private final SparkMax capoMotor;

  private final RelativeEncoder mEncoder;
  private final SparkClosedLoopController mController;

  public CapoShooterSubsystem() {
    capoMotor = new SparkMax(0, null);

    mEncoder = capoMotor.getEncoder();
    mController = capoMotor.getClosedLoopController();

    intializeMotor();
    resetEncoder();
  }

  private void resetEncoder() {
    mEncoder.setPosition(0);
  }

  private void intializeMotor() {
    SparkMaxConfig glConfig = new SparkMaxConfig();

    glConfig.encoder
      .velocityConversionFactor(CapoConstants.kGearRatio)
      .positionConversionFactor(CapoConstants.kGearRatio);

    glConfig
      .smartCurrentLimit(CapoConstants.kCurrentLimitMotor)
      .idleMode(CapoConstants.kIdleMode)
      .inverted(false);

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

    capoMotor.configure(glConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    /** Put Motor Data on smartDashboard */
    SmartDashboard.putNumber("shooterMeasurements/CapoMotor/kPosition", getEncoderValue());
    SmartDashboard.putNumber("shooterMeasurements/CapoMotor/kCurrent", getMotorCurrent());
  }

  private double getMotorCurrent() {
    return capoMotor.getOutputCurrent();
  }

  public double getEncoderValue() {
    return mEncoder.getPosition();
  }

  public void setTargetPosePID(double pose)
  {
    mController.setReference(pose, ControlType.kPosition);
  }

  public void stopMotors()
  {
    capoMotor.set(0);
  }
}
