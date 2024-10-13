package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PCFSI.ElevatorSubsystem;
import frc.robot.subsystems.PCFSI.FeederSubsystem;
import frc.robot.subsystems.PCFSI.IntakeSubsystem;
import frc.robot.subsystems.PCFSI.LEDSubsystem;
import frc.robot.subsystems.PCFSI.PivotSubsystem;


public class IntakeCommandAuto extends Command{
    private final IntakeSubsystem intake;
    private final FeederSubsystem feeder;
    private final PivotSubsystem pivot;
    private final ElevatorSubsystem elevator;
    private final LEDSubsystem led;

    public IntakeCommandAuto(IntakeSubsystem intake,FeederSubsystem feeder, LEDSubsystem led, PivotSubsystem pivot,ElevatorSubsystem elevator) {
        this.intake = intake;
        this.feeder = feeder;
        this.pivot = pivot;
        this.led = led;
        this.elevator = elevator;
        addRequirements(pivot,intake,feeder,led,elevator);
      }

      @Override
  public void initialize() {
    led.setRED();
    feeder.setVelocity(.32);
    intake.setVelocity(-50);
    pivot.intakePosition();
    led.setRED();
    
  }

  @Override
  public void execute()
  {
    feeder.setVelocity(.32);
    intake.setVelocity(-50);
  }

      @Override
      public boolean isFinished() {
       return feeder.noteCheck();
      }

     @Override
     public void end(boolean interrupted) {
      feeder.setVelocity(0);
      intake.setVelocity(0);
      pivot.setPosition(24.96);

    }
}