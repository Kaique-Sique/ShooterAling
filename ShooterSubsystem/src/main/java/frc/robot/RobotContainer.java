// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants.JoystickDriverConstants;
import frc.robot.commands.shooterCmds.BaseAlingCmd;
import frc.robot.commands.shooterCmds.CapoAlingCmd;
import frc.robot.commands.shooterCmds.Shoot;
import frc.robot.subsystems.shooter.BallCatcherSubsystem;
import frc.robot.subsystems.shooter.CapoShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterBaseAlingSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RobotContainer {
  // Subsystens instance 
  public static final ShooterSubsystem shooterSubsys = new ShooterSubsystem();
  public static final ShooterBaseAlingSubsystem baseShooter = new ShooterBaseAlingSubsystem();
  public static final BallCatcherSubsystem ballCatcher = new BallCatcherSubsystem();
  public static final CapoShooterSubsystem capoShooter = new CapoShooterSubsystem();

  // Commands Instanc
  public static final BaseAlingCmd m_baseAling = new BaseAlingCmd();

  private final CommandXboxController m_driverController =
      new CommandXboxController(JoystickDriverConstants.kDriverControllerPort);

  public RobotContainer() {
    m_baseAling.addRequirements(baseShooter);
    configureBindings();
  }

  private void configureBindings() 
  {
    //shootBall
    m_driverController.a().onTrue(new CapoAlingCmd());
    m_driverController.x().onTrue(new Shoot());

    //enable and disable auto aling
    m_driverController.leftBumper().onTrue(new InstantCommand(()-> baseShooter.disabledAutoAling()));
    m_driverController.rightBumper().onTrue(new InstantCommand(()-> baseShooter.enableAutoAling()));

    //Auto aling Enable
    Trigger autoAling = new Trigger(()-> (baseShooter.isEnableAutoAling()));
    autoAling.whileTrue(m_baseAling);
    autoAling.onFalse(new InstantCommand(()->baseShooter.stopMotor(), baseShooter));

    //Manual aling Enable, auto aling disable
    Trigger manualAling = new Trigger(()-> (!baseShooter.isEnableAutoAling()));
    manualAling.and(m_driverController.povRight())
        .whileTrue(new RunCommand(()->baseShooter.setOutput(0.1), baseShooter))
        .onFalse(new InstantCommand(()-> baseShooter.stopMotor(), baseShooter));
    manualAling.and(m_driverController.povLeft())
        .whileTrue(new RunCommand(()->baseShooter.setOutput(-0.1), baseShooter))
        .onFalse(new InstantCommand(()-> baseShooter.stopMotor(), baseShooter));

    manualAling.whileFalse(m_baseAling);

    // Home position
    manualAling.and(m_driverController.povDown())
      .whileTrue(new RunCommand(()-> baseShooter.setPIDPosition(0), baseShooter))
      .onFalse(new InstantCommand(()-> baseShooter.stopMotor(), baseShooter));
  }

  public Command getAutonomousCommand() 
  {
    return null;
  }
}
