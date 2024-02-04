// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AimCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();

  private final CommandXboxController armJoystick = new CommandXboxController(1);
  public static final CommandXboxController baseJoystick = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_swerveSubsystem.setDefaultCommand(new ManualDriveCommand(m_swerveSubsystem));
    m_climbSubsystem.setDefaultCommand(new ClimbCommand(m_climbSubsystem));
    configureBindings();
  }

  private void configureBindings() {
    baseJoystick.b().onTrue(Commands.runOnce(()->{
      m_swerveSubsystem.resetGyro();
    }));

    armJoystick.leftBumper().onTrue(Commands.runOnce(()->{
      m_climbSubsystem.climbComplete();
    }, m_climbSubsystem));

    armJoystick.rightTrigger(0.4).whileTrue(Commands.run(()->{
      m_shooterSubsystem.shooterMotorTurn();
    }, m_shooterSubsystem));

    armJoystick.x().onTrue(new IntakeCommand(m_intakeSubsystem, IntakeConstants.intakeInPosition, true));
    armJoystick.b().onTrue(new IntakeCommand(m_intakeSubsystem, IntakeConstants.intakePrimetivePosition, false));
    armJoystick.y().onTrue(new ShooterCommand(m_shooterSubsystem, ShooterConstants.shooterAMPSetpoint));
    armJoystick.a().onTrue(new ShooterCommand(m_shooterSubsystem, ShooterConstants.shooterPrimetivePosition));
    baseJoystick.leftTrigger(0.4).whileTrue(new AimCommand(m_visionSubsystem, m_shooterSubsystem, m_swerveSubsystem));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
