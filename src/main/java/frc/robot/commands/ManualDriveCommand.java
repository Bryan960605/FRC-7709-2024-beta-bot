// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import static frc.robot.RobotContainer.*;
public class ManualDriveCommand extends Command {
  /** Creates a new ManualDriveCommand. */
  private final SwerveSubsystem swerveSubsystem;
  private double xSpeed;
  private double ySpeed;
  private double zSpeed;
  public ManualDriveCommand(SwerveSubsystem _swerveSubsystem) {
    this.swerveSubsystem = _swerveSubsystem;
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(baseJoystick.rightBumper().getAsBoolean()){
      xSpeed = baseJoystick.getRawAxis(1)*0.4;
      ySpeed = baseJoystick.getRawAxis(0)*0.4;
      zSpeed = baseJoystick.getRawAxis(4)*0.4;
    }
    else{
      xSpeed = baseJoystick.getRawAxis(1)*0.8;
      ySpeed = baseJoystick.getRawAxis(0)*0.8;
      zSpeed = baseJoystick.getRawAxis(4)*0.8;
    }
    swerveSubsystem.drive(xSpeed, ySpeed, zSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
