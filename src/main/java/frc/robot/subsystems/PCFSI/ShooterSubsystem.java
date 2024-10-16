// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.PCFSI;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX m_topSh = new TalonFX(frc.robot.generated.Constants.ShooterConstants.topShooter);
  private final TalonFX m_botSh = new TalonFX(frc.robot.generated.Constants.ShooterConstants.botShooter);
  private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
  private final NeutralOut m_brake = new NeutralOut();
  TalonFXConfiguration shooterConfigs = new TalonFXConfiguration();

  private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
private GenericEntry topShooterSpeed =
      tab.add("Top Shooter Speed", 0)
         .getEntry();
private GenericEntry bottomShooterSpeed =
      tab.add("Botoom Shooter Speed", 0)
         .getEntry();
private GenericEntry topShooterVoltage =
      tab.add("Top Shooter Voltage", 0)
         .getEntry();
private GenericEntry bottomShooterVoltage =
      tab.add("Bottom Shooter Voltage", 0)
         .getEntry();

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
  /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
  shooterConfigs.Slot0.kP = 0.3; // An error of 1 rotation per second results in 2V output
  shooterConfigs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
  shooterConfigs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
  shooterConfigs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
  // Peak output of 8 volts
  shooterConfigs.Voltage.PeakForwardVoltage = 16;
  shooterConfigs.Voltage.PeakReverseVoltage = -16;

  m_topSh.getConfigurator().apply(shooterConfigs);
  m_botSh.getConfigurator().apply(shooterConfigs);
  }

  private void disable()
  {
      m_topSh.setControl(m_brake);
      m_botSh.setControl(m_brake);
  }

  public double getSpeed()
{
  return m_topSh.getRotorVelocity().getValue();
}
  public void setVelocity(double desiredRotationsPerSecond)
  {
      m_topSh.setControl(m_voltageVelocity.withVelocity(-desiredRotationsPerSecond));
      m_botSh.setControl(m_voltageVelocity.withVelocity(-desiredRotationsPerSecond));
  }
  private void setVelocityDiff(double TOPdesiredRotationsPerSecond,double BOTdesiredRotationsPerSecond)
  {
     m_topSh.setControl(m_voltageVelocity.withVelocity(TOPdesiredRotationsPerSecond));
     m_botSh.setControl(m_voltageVelocity.withVelocity(BOTdesiredRotationsPerSecond));
  }

//  COMMANDS

public Command shoot()
{
  return run(() -> this.setVelocity(ShooterConstants.shootSpeed));
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
    topShooterVoltage.setDouble(m_topSh.getMotorVoltage().getValue());
    topShooterSpeed.setDouble(m_topSh.getRotorVelocity().getValue());
    bottomShooterVoltage.setDouble(m_botSh.getMotorVoltage().getValue());
    bottomShooterSpeed.setDouble(m_botSh.getRotorVelocity().getValue());
  }
}
