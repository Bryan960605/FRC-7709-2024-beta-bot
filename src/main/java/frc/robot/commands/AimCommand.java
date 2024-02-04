// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AimCommand extends Command {
  /** Creates a new AimCommand. */
  private final VisionSubsystem visionSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final SwerveSubsystem swerveSubsystem;
  public AimCommand(VisionSubsystem _visionSubsystem, ShooterSubsystem _shooterSubsystem, SwerveSubsystem __swerveSubsystem) {
    this.visionSubsystem = _visionSubsystem;
    this.shooterSubsystem = _shooterSubsystem;
    this.swerveSubsystem = __swerveSubsystem;
    addRequirements(visionSubsystem, shooterSubsystem, swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
