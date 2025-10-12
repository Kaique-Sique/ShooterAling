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

// wpi imports
import frc.robot.Constants.SubsystemConstants.shooterConstants.MotorShooterBigWheelConstans;
import frc.robot.Constants.SubsystemConstants.shooterConstants.MotorShooterSmallWheelConstants;
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
    MotorShooterBigWheel = new SparkMax(MotorShooterBigWheelConstans.kShooterMotorID, MotorType.kBrushless);
    ShooterEncoderBigWheel = MotorShooterBigWheel.getEncoder();
    shooterBigWheelPID = MotorShooterBigWheel.getClosedLoopController();

    MotorFollowerBigWheel = new SparkMax(MotorShooterBigWheelConstans.kFollowerMotorID, MotorType.kBrushless);
    followerEncoderBigWheel = MotorFollowerBigWheel.getEncoder();
    followerBigWheelPID = MotorFollowerBigWheel.getClosedLoopController();

    MotorShooterSmallWheel = new SparkMax(MotorShooterSmallWheelConstants.kMotorID, MotorType.kBrushless);
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
    SparkMaxConfig globalConfig = new SparkMaxConfig();

    // Encoder setup
    globalConfig.encoder
        .velocityConversionFactor(MotorShooterSmallWheelConstants.kGearRatio)
        .positionConversionFactor(MotorShooterSmallWheelConstants.kGearRatio);

    // Motor config
    globalConfig
        .smartCurrentLimit(MotorShooterSmallWheelConstants.kCurrentLimitMotor)
        .idleMode(MotorShooterSmallWheelConstants.kIdleMode)
        .inverted(false);

    // PID setup
    globalConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.0005)
        .i(0.0)
        .d(0.0002)
        .velocityFF(0.0)
        .maxOutput(1)
        .minOutput(-1);

    MotorShooterSmallWheel.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * @ initialize motors of big wheel
   */
  private void initializeBigWheelMotors() 
  {
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig invertedConfig = new SparkMaxConfig();

    // Encoder setup
    globalConfig.encoder
        .velocityConversionFactor(MotorShooterBigWheelConstans.kGearRatioMotor)
        .positionConversionFactor(MotorShooterBigWheelConstans.kGearRatioMotor);

    // Motor config
    globalConfig
        .smartCurrentLimit(MotorShooterBigWheelConstans.kMotorCurrentLimit)
        .idleMode(MotorShooterBigWheelConstans.kIdleMode)
        .inverted(false);

    // PID setup
    globalConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.0005)
        .i(0.0)
        .d(0.0002)
        .velocityFF(0.0)
        .maxOutput(1)
        .minOutput(-1);

    invertedConfig
      .apply(globalConfig)
      .inverted(true);
    // Reset and persist parameters
    MotorShooterBigWheel.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    MotorFollowerBigWheel.configure(invertedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

    rpsSmallWheel = Conversions.MPSToRPS(mps, 
          MotorShooterSmallWheelConstants.kDiameterWheel, 
          MotorShooterSmallWheelConstants.kGearRatio);

    rpsBigWheel = Conversions.MPSToRPS(mps, 
          MotorShooterBigWheelConstans.kDiameterBigWheel, 
          MotorShooterBigWheelConstans.kGearRatioMotor);

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
    SmartDashboard.putNumber("shooterMeasurements/BigWheel/kCurrent", getBigWheelMotorCurrent());

    //Small Wheel Motor
    SmartDashboard.putNumber("shooterMeasurements/SmallWheel/kPosition", getBigWheelEncoderPose());
    SmartDashboard.putNumber("shooterMeasurements/SmallWheel/kVelocity", getBigWheelEncoderSpeed());
    SmartDashboard.putNumber("shooterMeasurements/SmallWheel/kCurrent", getSmallWheelMotorCurrent());

    //Follower Motor
    SmartDashboard.putNumber("shooterMeasurements/Follower/kPosition", getFollowerEncoderPose());
    SmartDashboard.putNumber("shooterMeasurements/Follower/kVelocity", getFollowerEncoderSpeed());
    SmartDashboard.putNumber("shooterMeasurements/Follower/kCurrent", getSmallWheelMotorCurrent());
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
   * Get Motor Current
   * @return
   */
  public double getSmallWheelMotorCurrent()
  {
    return MotorShooterSmallWheel.getOutputCurrent();
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
   * Get Motor Current
   * @return
   */
  public double getBigWheelMotorCurrent()
  {
    return MotorShooterBigWheel.getOutputCurrent();
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
  /**
   * Get Motor Current
   * @return
   */
  public double getFollowerMotorCurrent()
  {
    return MotorFollowerBigWheel.getOutputCurrent();
  }
}
