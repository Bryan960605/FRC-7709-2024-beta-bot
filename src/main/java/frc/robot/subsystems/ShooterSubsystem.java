// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final CANSparkMax shooterTurnMotor = new CANSparkMax(45, MotorType.kBrushless);
  private final CANSparkMax shooterShafMotor = new CANSparkMax(48, MotorType.kBrushless);
  private final CANSparkMax shooterTransportMotor = new CANSparkMax(2, MotorType.kBrushless);

  private final CANcoder shooterShaftCancoder = new CANcoder(54);
  private final CANcoderConfiguration shooterShaftCancoderCofig = new CANcoderConfiguration();

  private final PIDController shooterShaftPID = new PIDController(0, 0, 0);
  
  private final RelativeEncoder shooterTurnEncoder = shooterTurnMotor.getEncoder();

  private final DigitalInput redline = new DigitalInput(0);

  private double shooterTurnSpeed;
  private double shooterShaftPIDOutput;
  private double shooterShaftAngle;
  private double shooterShaftSetpoint;
  private double shooterShaftErrorvalue;
  private boolean haveNote;

  private double shooterTransportSpeed = 0;

  private final double shooterCancoderOffset = 0;

  public ShooterSubsystem() {
    shooterTurnMotor.restoreFactoryDefaults();
    shooterShafMotor.restoreFactoryDefaults();
    shooterTransportMotor.restoreFactoryDefaults();

    shooterTurnMotor.setInverted(false);
    shooterShafMotor.setInverted(false);
    shooterTransportMotor.setInverted(false);

    shooterTurnMotor.setIdleMode(IdleMode.kCoast);
    shooterShafMotor.setIdleMode(IdleMode.kBrake);
    shooterTransportMotor.setIdleMode(IdleMode.kCoast);

    shooterTurnMotor.burnFlash();
    shooterShafMotor.burnFlash();
    shooterTransportMotor.burnFlash();

    shooterShaftCancoderCofig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    shooterShaftCancoderCofig.MagnetSensor.MagnetOffset = shooterCancoderOffset;
    shooterShaftCancoderCofig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

    shooterShaftCancoder.getConfigurator().apply(shooterShaftCancoderCofig);
  }

  public void shooterMotorTurn(){
    if(shooterTurnSpeed >= (ShooterConstants.shooterSpeedSetpoint - 200)){
      shooterTransportMotor.setVoltage(9.6);
      shooterTransportSpeed = 6;
    }
    else{
      shooterTurnMotor.setVoltage(9.6);
      shooterTransportSpeed = 0;
    }
  }

  public void shooterTransportMotorSpeed(double speed){
    shooterTransportSpeed = speed;
  }

  public void getShooterShaftsetpoint(double angleSetpoint){
    shooterShaftSetpoint = angleSetpoint;
  }
  @Override
  public void periodic() {
    haveNote = redline.get();
    shooterShaftAngle = shooterShaftCancoder.getAbsolutePosition().getValueAsDouble();
    shooterTurnSpeed = shooterTurnEncoder.getVelocity();
    shooterShaftErrorvalue = shooterShaftSetpoint - shooterShaftAngle;

    shooterShaftPIDOutput = shooterShaftPID.calculate(shooterShaftAngle, shooterShaftSetpoint);

    if(shooterShaftErrorvalue > 2){
      shooterShafMotor.set(shooterShaftPIDOutput);
    }
    else{
      shooterShafMotor.set(0);
    }
    shooterTransportMotor.set(shooterTransportSpeed);
  }
}
