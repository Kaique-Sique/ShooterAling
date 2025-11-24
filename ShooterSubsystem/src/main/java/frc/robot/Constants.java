// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//CTRE Imports
import com.ctre.phoenix6.signals.NeutralModeValue;
//REV Imports
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
//WPI Imports
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

 //Constants class
 public final class Constants 
 {

  //Subsystem Constants
  public static final class SubsystemsConstants
  {

    public static final class shooterConstants {

      public static final class ShooterBigWheelConstans {
        public static final double kGearRatioMotor = 1.7;
        public static final double kDiameterBigWheel = Units.inchesToMeters(4);

        public static final int kShooterMotorID = 34;
        public static final int kFollowerMotorID = 33;

        public static final IdleMode kIdleMode = IdleMode.kCoast;
        public static final int kMotorCurrentLimit = 80;

        public static final double kp = 0.0001;
        public static final double kd = 0.00001;
        public static final double ki = 0.0;
      }

      public static final class ShooterSmallWheelConstants {
        public static final double kGearRatio = 1.7;
        public static final double kDiameterWheel = Units.inchesToMeters(2);

        public static final int kMotorID = 32;

        public static final IdleMode kIdleMode = IdleMode.kCoast;
        public static final int kCurrentLimitMotor = 80;

        public static final double kp = 0.0005;
        public static final double kd = 0.0002;
        public static final double ki = 0.0;
      }

      public static final class ShooterBaseConstants
      {
        public static final int kMotorId = 35;

        public static final double kGearRatio = 103.125;

        public static final IdleMode kIdleMode = IdleMode.kCoast;
        public static final int kCurrentLimitMotor = 40;

        public static final double kp = 0.009;
        public static final double kd = 0.002;
        public static final double ki = 0.0;

        public static final double kSoftLimitMax = 50;
        public static final double kSoftLimitMin = -12;
      }

      public static final class CatcherConstants 
      {
        public static final int kMotorId = 31;

        public static final double kGearRatio = 3;

        public static final double kDiameterWheel = Units.inchesToMeters(4);

        public static final IdleMode kIdleMode = IdleMode.kCoast;
        public static final int kCurrentLimitMotor = 80;

        public static final double kp = 0.0003;
        public static final double kd = 0.000;
        public static final double ki = 0.0;
      }

      public static final class CapoConstants 
      {
        public static final int kMotorId = 36;

        public static final double kGearRatio = 15;

        public static final IdleMode kIdleMode = IdleMode.kBrake;
        public static final int kCurrentLimitMotor = 40;

        public static final double kp = 0.009;
        public static final double kd = 0.00;
        public static final double ki = 0.000001;

        public static final double kSoftLimitMax = 120;
        public static final boolean kEnableSoftLimitMax = true;

        public static final double kSoftLimitMin = 0;
        public static final boolean kEnableSoftLimitMin = true;

        public static final double multiplicadorTargerDist = 45;
      }
    }

    //Lift Constants
    public static final class LiftConstants
    {
      //Lift Motor ports
      public static final int kLiftLeftMotorId = 25;
      //public static final int kLiftRightMotorId = 23;

      //Lift Constants gear box
      public static final double kLiftPulleyRatio = 20.0/42.0;
     
      //public static final double kLiftPulleyRatio = 42.0/20.0;
      public static final double kLiftTuboDiam = 0.0381; //0.033 
      public static final double  kLiftGearRatio = 1.0/3.0*kLiftPulleyRatio*kLiftTuboDiam*Math.PI;

      public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake; // TalonFx

      //PID Constants
      /**
       * Ks	0.065599 
       * Kv	7.0412 ------ 6.8364
       * Ka	0 ------------0.53589
       * Kg	0.32101-------0.34714
       * Kp	69.326 ------53.773
       * Kd	19.184--------8.6157
       * 
       */
      public static final double kS = 0.065599;//0.45817;//0.25;//<-MNL 11/11/2024 0.1; //0.32;  // Add 0.1 V output to overcome static friction
      public static final double kV = 6.8364;//5.8956;//0.12; //1.51; // A velocity target of 1 rps results in 0.12 V output
      public static final double kA =0.53589; //0.0;//0.01;  //0.27;
      public static final double kG = 0.34714;//0.3834;

      //TO DO: This must be tuned to specific robot
      public static final double kP = 53.773;//48.429;//50//35// 0.0665;//ANTERIOR 0,0665 ; 0.05872615 // kP = 0.11 An error of 1 rps results in 0.11 V output
      public static final double kI =0.0;
      public static final double kD= 8.6157;//13.049;
      
      public static final double kFF = 1.0/6000.0;
      public static final double kMinOutput= -1;
      public static final double kMaxOutput = 1;

      //Motion Magic PID - TO DO
      public static final double KS1 =0.065599;//0.25;//<-MNL 11/11/2024 0.1; //0.32;  // Add 0.1 V output to overcome static friction
      public static final double KV1 = 6.8364;//0.12; //1.51; // A velocity target of 1 rps results in 0.12 V output
      public static final double KA1 = 0.53589;//0.01;  //0.27;
      public static final double kP1 = 53.773;
      public static final double kI1 =0.0;
      public static final double kD1= 8.6157;


      /* Current Limiting */
     //TalonFX
      public static final int kCurrentLimit = 70; //supply current
      public static final int kCurrentThreshold = 120; //stator current
      public static final double kCurrentThresholdTime = 0.1;
      public static final boolean kEnableCurrentLimit = true;
      public static final double kClosedLoopRamp=0.25;
      public static boolean kInverted = false;
    
      
      /* 
      //Lift SparkMax
      public static final IdleMode kLiftIdleMode = IdleMode.kBrake;
      public static final int kLiftCurrentLimit = 40;
      public static final ClosedLoopSlot pidSparkSlot = ClosedLoopSlot.kSlot0;
      public static final double kLiftEncoderPositionPIDMinInput =0 ;
      public static final double kLiftEncoderPositionPIDMaxInput= 2 * Math.PI;

      //Lift Encoder
      public static final double kLiftPositionConversionFactor = kLiftGearRatio* Math.PI * kLiftPulleyRatio * kLiftTuboDiam; // rotation to meters
      public static final double kLiftVelocityConversionFactor = kLiftPositionConversionFactor/60; //meters/sec (2 * Math.PI) / 60.0; // radians per second
      //Lift PID
      public static final double kPLiftUp = 1.0;//ANTERIOR 0,25 ; 0.2142875; 0.175; 0.125; 0.23
      public static final double kILiftUp = 0;
      public static final double kDLiftUp = 0;
      public static final double kLiftUpFF = 0; // = input = 0 to position control and 1.0/473.0 to velocity control
      public static final double kLiftMinOutput = -1;
      public static final double kLiftMaxOutput = 1;

      public static final double kPLiftDown = 0.5;//ANTERIOR 0,25 ; 0.2142875; 0.175; 0.125; 0.23
      public static final double kILiftDown = 0;
      public static final double kDLiftDown = 0;
      public static final double kLiftDownFF = 0; // = input = 0 to position control and 1.0/473.0 to velocity control
      
      //Lift Soft Limits
      public static final double kSoftLimitMin = -0.1;
      public static final double kSoftLimitMax = 1.0;*/
      
    } 

    public static final class JoystickDriverConstants 
    {
      public static final int kDriverControllerPort = 0;
    }
  } 
}
