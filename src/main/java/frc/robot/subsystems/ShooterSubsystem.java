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

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
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
  private final PIDController shooterTurnPID = new PIDController(0, 0, 0);
  private final ArmFeedforward shooterShaftFeedforward = new ArmFeedforward(0, 0, 0, 0);
  
  private final RelativeEncoder shooterSheftEncoder = shooterShafMotor.getEncoder();
  private final RelativeEncoder shooterTurnEncoder = shooterTurnMotor.getEncoder();

  private double shooterTurnSpeed;
  private double shooterShaftFeedforwardOutput;
  private double shooterShaftPIDOutput;
  private double shooterTurnPIDOutput;
  private double shooterShaftAngle;
  private double shooterShaftRadians;
  private double shooterShaftAngularVelocity;
  private double shooterShaftSetpoint;
  private double shooterShaftErrorvalue;
  private double distance;

  private final double shooterCancoderOffset = 0;
  private final double change2AngularVelocity = 1*2*Math.PI/60;

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
    if(shooterTurnSpeed > (ShooterConstants.shooterSpeedSetpoint - 100)){
      shooterTransportMotor.setVoltage(6);
      shooterTransportMotor.setVoltage(6);
    }
    else{
      shooterTurnMotor.setVoltage(9.6 + shooterTurnPIDOutput);
      shooterTransportMotor.setVoltage(0);
    }
  }

  public void getShooterShaftsetpoint(double angleSetpoint){
    shooterShaftSetpoint = angleSetpoint;
  }
  @Override
  public void periodic() {
    shooterShaftAngle = shooterShaftCancoder.getAbsolutePosition().getValueAsDouble();
    shooterShaftRadians = Math.toRadians(shooterShaftAngle);
    shooterShaftAngularVelocity = shooterSheftEncoder.getVelocity()*change2AngularVelocity;
    shooterTurnSpeed = shooterTurnEncoder.getVelocity();
    shooterShaftErrorvalue = shooterShaftSetpoint - shooterShaftAngle;

    shooterTurnPIDOutput = shooterTurnPID.calculate(shooterTurnSpeed, ShooterConstants.shooterSpeedSetpoint);
    shooterShaftPIDOutput = shooterShaftPID.calculate(shooterShaftAngle, shooterShaftSetpoint);
    shooterShaftFeedforwardOutput = shooterShaftFeedforward.calculate(shooterShaftRadians, shooterShaftAngularVelocity)/12;

    if(shooterShaftErrorvalue > 2){
      shooterShafMotor.set(shooterShaftPIDOutput + shooterShaftFeedforwardOutput);
    }
    else{
      shooterShafMotor.set(shooterShaftFeedforwardOutput);
    }
  }
}
