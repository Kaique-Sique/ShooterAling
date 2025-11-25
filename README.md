# Shooter Subsystem 
## Hardware Overview 

> [!IMPORTANT]
>
> Neo Front Motor shooter          - CanID: 34
> 
> Neo Front Follower Motor shooter - CanID: 33
> 
> Neo Base Motor Shooter           - CanID: 35
>
> Neo Ball Catcher Motor           - CanID: 31
>
> Neo **550** Motor Capo Shooter   - CanID: 36

## Software Overview
### 1- Shooter Motor Follower Initialize

``` java
/**
   * @ initialize motors of big wheel
   */
  private void initializeBigWheelMotors() {
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig followerRightConfig = new SparkMaxConfig();

    // Encoder setup
    globalConfig.encoder
        .velocityConversionFactor(2* Math.PI / 60)
        .positionConversionFactor(1);

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
        .velocityFF(0.00211416490486257928118393234672)
        .maxOutput(1)
        .minOutput(-1);

    followerRightConfig
        .apply(globalConfig)
        .follow(MotorShooterBigWheel, true);  // ** True to inverted param
        //.inverted(true);
        
        
        
    // Reset and persist parameters
    MotorShooterBigWheel.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    MotorFollowerBigWheel.configure(followerRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //ShooterEncoderBigWheel.setPosition(0);
    //followerEncoderBigWheel.setPosition(0);
  }

```

### 2- BasePositionAling
This command uses the robot pose2d and target pose2d with trigonometry and arc tangent to calc target base angle.
> [!WARNING]
> 
> MaxSoft Limits: 50
> 
> Min Soft Limits: -12
<img width="555" height="331" alt="image" src="https://github.com/user-attachments/assets/0ae9cbec-dc81-425c-80d6-5ee043ebaa79" />

```java
public double getTargetPose()
  {
    //GetRobotPose
    robotPose2d = swerveDrive.getPoseEstimator();
    rotRobot = robotPose2d.getRotation();


    // Get Target Pose based on swerve 
    targetPose = Units.radiansToDegrees(
      Math.atan2((FieldPoses.redPoses.tag02.getY() - robotPose2d.getY()) * -1, 
            (FieldPoses.redPoses.tag02.getX()- robotPose2d.getX()) * -1));
          
    if(LimelightHelpers.getTV(DriveConstants.limelightBack)){
    // Implement robot angle to target pose
    targetPose =  swerveDrive.getBlueAlliance()
                                ? (targetPose - rotRobot.getDegrees())
                                : (targetPose - rotRobot.getDegrees());
    }
    else
    {
      targetPose = 0;
    }
    return targetPose;
  } 
```

### 3- Capo position Aling Cmd
this command calc robot dist with the target linking to capo pose with a proportional.

> [!WARNING]
> 
> MaxSoft Limits: 120
> 
> Min Soft Limits: 0

``` java
/**
   * Calc hip from robot dist to target
   * 
   * @return double targetPose
   */
  public double getTargetDist()
  {
    // Get swerve Positions
    robotPose2d = swerveDrive.getPoseEstimator();
    
    // Calc relativa hip from robot and target
    /************* Pitagoras *******************/
    double hip = Math.sqrt(
           Math.pow(
                Math.abs(robotPose2d.getX() - FieldPoses.redPoses.tag02.getX()), 2)
      
                + Math.pow(
                    Math.abs(robotPose2d.getY() - FieldPoses.redPoses.tag02.getY()), 2));

    // Multiplicate de dist for a constant 
    targetPose = hip * CapoConstants.multiplicadorTargerDist;

    return targetPose;
  } 
```
If the robot didn't see the tag any moment since start, the shooter will move wrong!!!

### 4- Shoot sequencial command group
* Active front motors of shooter.
* Wait a second
* Active ballcatcher
* wait ball be shooted
* Stop Motors

``` java
addCommands(
      // Initialize Shooter Motors
      new ShooterOutCmd(),

      // Wait a time to shooter get his target speed
      new WaitCommand(1),

      // Initializa ball Catcher
      new InstantCommand(()-> m_BallCatcherSubsystem.setSpeedTarget(5200), m_BallCatcherSubsystem),

      // Wait a time to robot stop to shoot
      new WaitCommand(1),

      // Stop all motors
      new InstantCommand(()-> m_BallCatcherSubsystem.stopMotor(), m_BallCatcherSubsystem)
        .andThen(()-> m_ShooterSubsystem.stopMotors(), m_ShooterSubsystem)
    );
```
