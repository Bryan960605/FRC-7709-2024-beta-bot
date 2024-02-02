// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  private final CANSparkMax climbRightMotor = new CANSparkMax(12, MotorType.kBrushless);
  private final CANSparkMax climbLeftMotor = new CANSparkMax(51, MotorType.kBrushless);

  private final Servo rightServo = new Servo(0);
  private final Servo leftServo = new Servo(9);

  public boolean climbDone = false;
  public ClimbSubsystem() {
    climbRightMotor.restoreFactoryDefaults();
    climbLeftMotor.restoreFactoryDefaults();

    climbRightMotor.setInverted(true);
    climbLeftMotor.setInverted(false);

    climbRightMotor.setIdleMode(IdleMode.kBrake);
    climbLeftMotor.setIdleMode(IdleMode.kBrake);

    climbRightMotor.burnFlash();
    climbLeftMotor.burnFlash();
  }

  public void climbMove(double leftSpeed, double rightSpeed){
    if(climbDone == false){
      climbLeftMotor.set(leftSpeed);
      climbRightMotor.set(rightSpeed);
    }
    else{
      climbLeftMotor.set(0);
      climbRightMotor.set(0);
    }
  }

  public void climbComplete(){
    if(climbDone ==false){
      climbDone = true;
    leftServo.setAngle(90);
    rightServo.setAngle(-90);
    }
    else{
    climbDone = false;
    leftServo.setAngle(0);
    rightServo.setAngle(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
