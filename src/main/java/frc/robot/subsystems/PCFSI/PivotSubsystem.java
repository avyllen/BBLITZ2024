// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.PCFSI;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.Constants;
import frc.robot.generated.Constants.PivotConstants;

public class PivotSubsystem extends SubsystemBase {

private CANSparkMax m_pivot;
private SparkPIDController m_pidController;
private RelativeEncoder m_encoder;
public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem() {
    m_pivot = new CANSparkMax(frc.robot.generated.Constants.PivotConstants.pivot, MotorType.kBrushless);
    m_pivot.restoreFactoryDefaults();
    m_pidController = m_pivot.getPIDController();
    m_encoder = m_pivot.getEncoder();
  
     // PID coefficients
     kP = 0.11; 
     kI = 0;
     kD = 0; 
     kIz = 0; 
     kFF = 1/565; 
     kMaxOutput = 0.3; 
     kMinOutput = -0.3;
  
     // set PID coefficients
     m_pidController.setP(kP);
     m_pidController.setI(kI);
     m_pidController.setD(kD);
     m_pidController.setIZone(kIz);
     m_pidController.setFF(kFF);
     m_pidController.setOutputRange(kMinOutput, kMaxOutput);
  
     m_encoder.setPosition(0);
  }

public void setVelocity(double setPoint)
{
m_pivot.set(setPoint);
}

public void setPosition(double setPoint)
{
  m_pidController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
}

public void intakePosition()
{
  m_pidController.setReference(-8.95, CANSparkMax.ControlType.kPosition);
}

public void homePosition()
{
  m_pidController.setReference(Constants.PivotConstants.homePosition, CANSparkMax.ControlType.kPosition);
}


public void subwooferPosition()
{
  m_pidController.setReference(Constants.PivotConstants.subwooferShotPosition, CANSparkMax.ControlType.kPosition);
}

public void farShotPosition()
{
  m_pidController.setReference(Constants.PivotConstants.farShot, CANSparkMax.ControlType.kPosition);
}

public void extremeFarShotPosition() {
  m_pidController.setReference(Constants.PivotConstants.extremeFarShot, CANSparkMax.ControlType.kPosition);
}

public void otherPositions()
{
  m_pidController.setReference(-12.5, CANSparkMax.ControlType.kPosition);
}

public boolean LimitChecks()
{
return ((m_encoder.getPosition() > -0.4 && m_pivot.getAppliedOutput() > 0) || (m_encoder.getPosition() < PivotConstants.PIVOTMAX && m_pivot.getAppliedOutput() < 0));
}

public Command withPosition(double setPoint)
{
  return run(() -> this.setPosition(setPoint));
}

public Command intakePositionCommand()
{
  return run(() -> this.setPosition(-9.95));
}

public Command subwooferPositionCommand()
{
  return run(() -> this.subwooferPosition());
}

public Command ampPositionCommand()
{
  return run(() -> this.setPosition(-19.7)); //amp stuff
}

public Command setHomePositionCommand()
{
  return run(() -> this.homePosition());
}
public Command slowUp()
{
  return run(() -> this.setVelocity(.1));
}

public Command slowDown()
{
  return run(() -> this.setVelocity(-.1));
}

public Command stop()
{
  return run(() -> this.setVelocity(0));
}

public Command holdPosition()
{
  return run(() -> this.setPosition(this.m_encoder.getPosition()));
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  SmartDashboard.putNumber("Shooter Pivot Encoder", m_encoder.getPosition());
  SmartDashboard.putNumber("Shooter Pivot Appied Voltage", m_pivot.getAppliedOutput() );
}

}
