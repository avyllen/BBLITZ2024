package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PCFSI.PivotSubsystem;

public class AutoSubwooferPivot extends Command{

    private final PivotSubsystem pivot;

    public AutoSubwooferPivot(PivotSubsystem pivot) {
        this.pivot = pivot;
        addRequirements(pivot);
      }

      @Override
  public void initialize() {
    pivot.subwooferPosition();
  }

      @Override
      public boolean isFinished() {
          return true;
      }

     @Override
     public void end(boolean interrupted) {
      pivot.subwooferPosition();
    }
}