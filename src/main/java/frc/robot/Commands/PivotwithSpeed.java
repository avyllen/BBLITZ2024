package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PCFSI.PivotSubsystem;

public class PivotwithSpeed extends Command{

    private final PivotSubsystem pivot;
    private double speed;


    public PivotwithSpeed(PivotSubsystem pivot,double speed) {
        this.pivot = pivot;
        this.speed = speed;
        addRequirements(pivot);
      }

      @Override
  public void initialize() {
    //pivot.setVelocity(speed);
    
  }

    @Override
  public void execute() {
    pivot.setVelocity(speed);
    
  }

      @Override
      public boolean isFinished() {
       return pivot.LimitChecks();
      }

     @Override
     public void end(boolean interrupted) {
      pivot.setVelocity(0);
     }
}