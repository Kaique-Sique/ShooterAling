// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.SubsystemConstants.shooterConstants;

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
    MotorShooterBigWheel = new SparkMax(shooterConstants.kShooterBigWheelID, MotorType.kBrushless);
    ShooterEncoderBigWheel = MotorShooterBigWheel.getEncoder();
    shooterBigWheelPID = MotorShooterBigWheel.getClosedLoopController();

    MotorFollowerBigWheel = new SparkMax(0, MotorType.kBrushless);
    followerEncoderBigWheel = MotorFollowerBigWheel.getEncoder();
    followerBigWheelPID = MotorFollowerBigWheel.getClosedLoopController();

    MotorShooterSmallWheel = new SparkMax(0, MotorType.kBrushless);
    ShooterEncoderSmallWheel = MotorShooterSmallWheel.getEncoder();
    shooterSmallWheelPID = MotorShooterSmallWheel.getClosedLoopController();

    initializeBigWheelMotors();
    initializeSmallWheelsMotors();
  }

  private void initializeSmallWheelsMotors() {
  }

  private void initializeBigWheelMotors() {
  }

  @Override
  public void periodic() {
  }
}
