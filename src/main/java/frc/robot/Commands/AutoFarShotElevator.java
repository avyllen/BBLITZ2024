package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PCFSI.ElevatorSubsystem;
import frc.robot.subsystems.PCFSI.PivotSubsystem;

public class AutoFarShotElevator extends Command{

    private final ElevatorSubsystem elevator;

    public AutoFarShotElevator(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
      }

      @Override
  public void initialize() {
    elevator.farShotPosition();
  }

      @Override
      public boolean isFinished() {
          return true;
      }

     @Override
     public void end(boolean interrupted) {
      elevator.farShotPosition();
    }
}