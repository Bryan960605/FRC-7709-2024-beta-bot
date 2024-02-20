// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  private final CANSparkMax climbRightMotor = new CANSparkMax(12, MotorType.kBrushless);
  private final CANSparkMax climbLeftMotor = new CANSparkMax(51, MotorType.kBrushless);

  private final RelativeEncoder climbRightEncoder = climbRightMotor.getEncoder();
  private final RelativeEncoder climbLeftEncoder = climbLeftMotor.getEncoder();

  private final DigitalInput climbRightLimitSwitch = new DigitalInput(1);
  private final DigitalInput climbLeftLimitSwitch = new DigitalInput(3);

  private double climbLeftPosition;
  private double climbRightPosition;
  public ClimbSubsystem() {
    climbRightMotor.restoreFactoryDefaults();
    climbLeftMotor.restoreFactoryDefaults();

    climbRightMotor.setInverted(true);
    climbLeftMotor.setInverted(false);

    climbRightMotor.setIdleMode(IdleMode.kBrake);
    climbLeftMotor.setIdleMode(IdleMode.kBrake);

    climbRightMotor.burnFlash();
    climbLeftMotor.burnFlash();

    climbLeftEncoder.setPosition(0);
    climbRightEncoder.setPosition(0);
  }

  public void climbMove(double leftSpeed, double rightSpeed){
    climbRightMotor.set(Constants.climbMotorSpeed(rightSpeed, climbRightLimitSwitch.get(), 152, climbRightPosition));
    climbLeftMotor.set(Constants.climbMotorSpeed(leftSpeed, climbLeftLimitSwitch.get(), 150, climbLeftPosition));
  }

  @Override
  public void periodic() {
    climbLeftPosition = climbLeftEncoder.getPosition();
    climbRightPosition = climbRightEncoder.getPosition();

    if(climbRightLimitSwitch.get()){
      climbRightEncoder.setPosition(0);
    }
    if(climbLeftLimitSwitch.get()){
      climbLeftEncoder.setPosition(0);
    }

    SmartDashboard.putNumber("climbRightPosition", climbRightPosition);
    SmartDashboard.putNumber("climbLeftPosition", climbLeftPosition);
    // This method will be called once per scheduler run
  }
}
