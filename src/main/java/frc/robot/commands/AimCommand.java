// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import static frc.robot.Constants.ApriltagConstants.blueModeSelect;
import static frc.robot.Constants.ApriltagConstants.redModeSelect;

public class AimCommand extends Command {
  /** Creates a new AimCommand. */
  private final VisionSubsystem visionSubsystem;
  private final SwerveSubsystem swerveSubsystem;
  private double xSpeed;
  private double ySpeed;
  private double zSpeed;
  private double setpoint[];
  public AimCommand(VisionSubsystem _visionSubsystem, SwerveSubsystem __swerveSubsystem) {
    this.visionSubsystem = _visionSubsystem;
    this.swerveSubsystem = __swerveSubsystem;
    addRequirements(visionSubsystem, swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(visionSubsystem.alliance.get() == DriverStation.Alliance.Red){
      setpoint = redModeSelect(visionSubsystem.targetID);
    }
    else{
      setpoint = blueModeSelect(visionSubsystem.targetID);
    }
    visionSubsystem.getSetpoint(setpoint);
    xSpeed = visionSubsystem.xMovePIDOutput;
    ySpeed = visionSubsystem.yMovePIDOutput;
    zSpeed = visionSubsystem.turnPIDOutput;
    swerveSubsystem.drive(xSpeed, ySpeed, zSpeed, false);
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
