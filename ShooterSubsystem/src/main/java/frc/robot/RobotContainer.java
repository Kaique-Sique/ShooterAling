// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SubsystemsConstants.JoystickDriverConstants;
import frc.robot.commands.shooterCmds.BasePositionAling;
import frc.robot.commands.shooterCmds.CapoAlingCmd;
import frc.robot.commands.shooterCmds.CapoPositionAling;
import frc.robot.commands.shooterCmds.ShootCmd;
import frc.robot.subsystems.shooter.BallCatcherSubsystem;
import frc.robot.subsystems.shooter.CapoShooterSubsystem;
import frc.robot.subsystems.shooter.RobotPoseOnTheField;
import frc.robot.subsystems.shooter.ShooterBaseAlingSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RobotContainer {
  // Subsystens instance 
  public static final ShooterSubsystem shooterSubsys = new ShooterSubsystem();
  public static final ShooterBaseAlingSubsystem baseShooter = new ShooterBaseAlingSubsystem();
  public static final BallCatcherSubsystem ballCatcher = new BallCatcherSubsystem();
  public static final CapoShooterSubsystem capoShooter = new CapoShooterSubsystem();
  public static final RobotPoseOnTheField robotPoseOnTheField = new RobotPoseOnTheField();

  // Driver controller instance
  private final CommandXboxController m_driverController =
      new CommandXboxController(JoystickDriverConstants.kDriverControllerPort);

  // Robot container constructor
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() 
  {
    /******* Shooter Commands *******/
    // Capo Aling to dashboard target pose
    m_driverController.a().onTrue(new CapoAlingCmd());

    // Shoot Command
    m_driverController.x().onTrue(new ShootCmd());

    // Ball catcher control 
    m_driverController.y().whileTrue(new RunCommand(()-> ballCatcher.setSpeedTarget(5600), ballCatcher))
        .onFalse(new InstantCommand(()-> ballCatcher.stopMotor(), ballCatcher));
    
    // Shooter set output to 20%
    m_driverController.b().whileTrue(new RunCommand(()-> shooterSubsys.setOutput(0.20), shooterSubsys))
        .onFalse(new InstantCommand(()-> shooterSubsys.stopMotors(), shooterSubsys));

    // Manual Aling
    m_driverController.povUp()
      .onTrue(new BasePositionAling().alongWith(new CapoPositionAling()));

    // Base HomePosition
    m_driverController.povRight()
      .whileTrue(new RunCommand(()-> baseShooter.setPIDPosition(0), baseShooter))
      .onFalse(new InstantCommand(()-> baseShooter.stopMotor(), baseShooter));
  }

  public Command getAutonomousCommand() 
  {
    return null;
  }
}
