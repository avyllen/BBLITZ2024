// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Constants;
import frc.robot.subsystems.PCFSI.PivotSubsystem;

public class PivotHome extends Command {
  private final PivotSubsystem pivot;

  public PivotHome(PivotSubsystem pivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pivot = pivot;
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivot.withPosition(Constants.PivotConstants.homePosition);
  }

  @Override
  public boolean isFinished() {
    return pivot.CheckPositionHome();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }


}
