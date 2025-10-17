// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants.shooterConstants.motorCatcherConstants;
import frc.robot.Utils.Conversions;

public class BallCatcherSubsystem extends SubsystemBase {
  private final SparkMax motorCatcher;

  private final RelativeEncoder encoderMotor;
  private final SparkClosedLoopController PIDControler;

  public BallCatcherSubsystem() {
    motorCatcher = new SparkMax(motorCatcherConstants.kMotorId, MotorType.kBrushless);

    encoderMotor = motorCatcher.getEncoder();
    PIDControler = motorCatcher.getClosedLoopController();

    initializeMotor();
  }

  /**
   * @ initialize motor
   */
  private void initializeMotor() 
  {
    SparkMaxConfig gConfig = new SparkMaxConfig();

    // Encoder setup
    gConfig.encoder
        .velocityConversionFactor(motorCatcherConstants.kGearRatio)
        .positionConversionFactor(motorCatcherConstants.kGearRatio);

    // Motor config
    gConfig
        .smartCurrentLimit(motorCatcherConstants.kCurrentLimitMotor)
        .idleMode(motorCatcherConstants.kIdleMode)
        .inverted(false);

    // PID setup
    gConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(motorCatcherConstants.kp)
        .i(motorCatcherConstants.ki)
        .d(motorCatcherConstants.kd)
        .velocityFF(0.0)
        .maxOutput(1)
        .minOutput(-1);

    motorCatcher.configure(gConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoderMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * @ set encoder to zero
   */
  public void resetEncoder()
  {
    encoderMotor.setPosition(0);
  }
  /**
   * get encoder position
   * @return encoder position
   */
  public double getEncoderPosition()
  {
    return encoderMotor.getPosition();
  }
  /**
   * get encoder speed
   * @return RPM
   */
  public double getEncoderSpeed()
  {
    return encoderMotor.getVelocity();
  }
  /**
   * get Motor Current
   * @return output current
   */
  public double getMotorCurrent()
  {
    return motorCatcher.getOutputCurrent();
  }

  /**
   * position pid
   * @param targetPose angle from degress
   */
  public void setTargetPose(double targetPose)
  {
    PIDControler.setReference(targetPose, ControlType.kPosition);
  }

  /**
   * Set a pid speed in RPM to both wheels
   * 
   * @param speed RPM
   */
  public void setSpeedTarget(double speedRPM)
  {
    PIDControler.setReference(speedRPM, ControlType.kVelocity);
  }

  /**
   * Set a pid controler speed to MPS of both wheels
   * 
   * @param mps setpoint in mps
   */
  public void setMPSTarget(double MPSspeed)
  {
    double rps;
    rps = Conversions.MPSToRPS(MPSspeed,
        motorCatcherConstants.kDiameterWheel,
        motorCatcherConstants.kGearRatio);

    PIDControler.setReference(rps * 60, ControlType.kVelocity);
  }

  /**
   * stop Motor catcher
   */
  public void stopMotor()
  {
    motorCatcher.set(0);
  }
}
