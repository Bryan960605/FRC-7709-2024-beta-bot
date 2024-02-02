// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Visionsubsystem;

import static frc.robot.Constants.JoystickButtonNumbers.*;
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
  private final Visionsubsystem m_visionSubsystem = new Visionsubsystem();

  private final ManualDriveCommand m_manualDriveCommand = new ManualDriveCommand(m_swerveSubsystem);
  private final ClimbCommand m_climbCommand = new ClimbCommand(m_climbSubsystem);

  private final Joystick armJoystick = new Joystick(1);
  public static final Joystick baseJoystick = new Joystick(0);

  private final JoystickButton inButon = new JoystickButton(armJoystick, inButtonNumber);
  private final JoystickButton shootButton = new JoystickButton(armJoystick, shootButtonNumber);
  private final JoystickButton ampButton = new JoystickButton(armJoystick, ampButtonNumber);
  private final JoystickButton primetiveButton = new JoystickButton(armJoystick, primetiveButtonNumber);
  private final JoystickButton aimButton = new JoystickButton(baseJoystick, aimButtonNumber);
  private final JoystickButton resetGyroButton = new JoystickButton(baseJoystick, resetGyroButtonNumber);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_swerveSubsystem.setDefaultCommand(m_manualDriveCommand);
    m_climbSubsystem.setDefaultCommand(m_climbCommand);
    configureBindings();
  }

  private void configureBindings() {
    resetGyroButton.onTrue(Commands.runOnce(()->{
      m_swerveSubsystem.resetGyro();
    }));
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