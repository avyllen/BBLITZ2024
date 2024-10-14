package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PCFSI.ElevatorSubsystem;
import frc.robot.subsystems.PCFSI.FeederSubsystem;
import frc.robot.subsystems.PCFSI.IntakeSubsystem;
import frc.robot.subsystems.PCFSI.LEDSubsystem;
import frc.robot.subsystems.PCFSI.PivotSubsystem;

public class IntakeCommand extends Command{

    private final IntakeSubsystem intake;
    private final FeederSubsystem feeder;
    private final PivotSubsystem pivot;
    private final ElevatorSubsystem elevator;
    private final LEDSubsystem led;
    
    private boolean firstcheck = true;

    public IntakeCommand(IntakeSubsystem intake,FeederSubsystem feeder,LEDSubsystem led, PivotSubsystem pivot, ElevatorSubsystem elevator) {
        this.intake = intake;
        this.feeder = feeder;
        this.pivot = pivot;
        this.led = led;
        this.elevator = elevator;
        addRequirements(intake,feeder,pivot,led,elevator);

      }

      @Override
  public void initialize() {
    led.setRED();
    feeder.intake();
    intake.intake();
    pivot.intakePosition();  
    elevator.homePosition();  
  }

  @Override
  public void execute()
  {
    feeder.setVelocity(-15);
    intake.setVelocity(-20);
  }

      @Override
      public boolean isFinished() {
          return !feeder.noteCheck();
      }

     @Override
     public void end(boolean interrupted) {
      feeder.withDisable();
      intake.withDisable();
      elevator.holdPosition();
      pivot.setVelocity(0);
      if(!interrupted)
      {
        led.setGREEN();
        //LimelightHelpers.setLEDMode_ForceBlink("limelight-backup");
      }

    }
}