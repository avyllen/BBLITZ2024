// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.PCFSI;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.Constants.FeederConstants;
import frc.robot.generated.Constants.ShooterConstants;

public class FeederSubsystem extends SubsystemBase {

  private final TalonFX m_feeder = new TalonFX(frc.robot.generated.Constants.FeederConstants.feederID);
  private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
  private final NeutralOut m_brake = new NeutralOut();
  TalonFXConfiguration shooterConfigs = new TalonFXConfiguration();

private final DigitalInput intakeLine;
private final DigitalOutput outLine;

  private ShuffleboardTab tab = Shuffleboard.getTab("Feeder");
private GenericEntry FeederSpeed =
      tab.add("Feeder Speed", 0)
         .getEntry();
private GenericEntry FeederVoltage =
      tab.add("Feeder Voltage", 0)
         .getEntry();


  /** Creates a new ShooterSubsystem. */
  public FeederSubsystem() {
  /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
  shooterConfigs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
  shooterConfigs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
  shooterConfigs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
  shooterConfigs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
  // Peak output of 8 volts
  shooterConfigs.Voltage.PeakForwardVoltage = 8;
  shooterConfigs.Voltage.PeakReverseVoltage = -8;

  m_feeder.getConfigurator().apply(shooterConfigs);

  intakeLine = new DigitalInput(0);
  outLine = new DigitalOutput(1); 
  }

  private void disable()
  {
      m_feeder.setControl(m_brake);
  }

  public double getSpeed()
{
  return m_feeder.getRotorVelocity().getValue();
}
  public void setVelocity(double desiredRotationsPerSecond)
  {
      m_feeder.setControl(m_voltageVelocity.withVelocity(desiredRotationsPerSecond));
  }
  private void setVelocityDiff(double TOPdesiredRotationsPerSecond,double BOTdesiredRotationsPerSecond)
  {
     m_feeder.setControl(m_voltageVelocity.withVelocity(TOPdesiredRotationsPerSecond));
  }

//  COMMANDS

public Command intake()
{
  return run(() -> this.setVelocity(FeederConstants.IntakeSPEED));
}

public Command outtake()
{
  return run(() -> this.setVelocity(FeederConstants.OutakeSPEED));
}

public Command AMPouttake()
{
  return run(() -> this.setVelocity(FeederConstants.ampOutakeSPEED));
}

public boolean noteCheck()
{
  return intakeLine.get();
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
    FeederVoltage.setDouble(m_feeder.getMotorVoltage().getValue());
    FeederSpeed.setDouble(m_feeder.getRotorVelocity().getValue());
    SmartDashboard.putBoolean("FEEDER NOTE CHECK", intakeLine.get());
    outLine.set(true);
  }
}