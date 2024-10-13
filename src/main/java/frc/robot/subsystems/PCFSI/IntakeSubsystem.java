package frc.robot.subsystems.PCFSI;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX m_intake = new TalonFX(frc.robot.generated.Constants.IntakeConstants.intakeID);
  private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

private final NeutralOut m_brake = new NeutralOut();

TalonFXConfiguration intakeconfigs = new TalonFXConfiguration();

private ShuffleboardTab tab = Shuffleboard.getTab("Intake");
private GenericEntry intakeSpeed =
      tab.add("Intake Speed", 0)
         .getEntry();
private GenericEntry intakeVoltage =
      tab.add("Intake Voltage", 0)
         .getEntry();



  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
           /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
           intakeconfigs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
           intakeconfigs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
           intakeconfigs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
           intakeconfigs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
           // Peak output of 8 volts
           intakeconfigs.Voltage.PeakForwardVoltage = 8;
           intakeconfigs.Voltage.PeakReverseVoltage = -8;
    
           m_intake.getConfigurator().apply(intakeconfigs);
  }

//  COMMANDS  //

  private void disable()
  {
    m_intake.setControl(m_brake);
  }

  public void setVelocity(double desiredRotationsPerSecond)
{
    m_intake.setControl(m_voltageVelocity.withVelocity(desiredRotationsPerSecond));
}

  public void intake()
{
    m_intake.setControl(m_voltageVelocity.withVelocity(IntakeConstants.intakeSpeed));
}

  public void outtake()
{
    m_intake.setControl(m_voltageVelocity.withVelocity(IntakeConstants.outtakeSpeed));
}


public Command intakeCommand()
{
  return run(() -> this.intake());
}

public Command outtakeCommand()
{
  return run(() -> this.outtake());
}

public Command withVelocity(double desiredRotationsPerSecond)
{
  return runOnce(() -> this.setVelocity(desiredRotationsPerSecond));
}

public Command withDisable()
{
    return run(() -> this.disable());
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeVoltage.setDouble(m_intake.getMotorVoltage().getValue());
    intakeSpeed.setDouble(m_intake.getRotorVelocity().getValue());
  }
}
