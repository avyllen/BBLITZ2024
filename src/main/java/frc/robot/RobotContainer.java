// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.ElevatorWithSpeed;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Commands.PivotwithSpeed;
import frc.robot.Commands.TELEShootCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PCFSI.ClimberSubsystem;
import frc.robot.subsystems.PCFSI.ElevatorSubsystem;
import frc.robot.subsystems.PCFSI.FeederSubsystem;
import frc.robot.subsystems.PCFSI.IntakeSubsystem;
import frc.robot.subsystems.PCFSI.LEDSubsystem;
import frc.robot.subsystems.PCFSI.PivotSubsystem;
import frc.robot.subsystems.PCFSI.ShooterSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {

   private final SendableChooser<Command> autoChooser;

   // CONTROLLER
  private final CommandXboxController xbox = new CommandXboxController(0); 
  private final CommandJoystick joystick = new CommandJoystick(1);

   //intake Subsystem
 public final IntakeSubsystem intake = new IntakeSubsystem();

  //Shooter Subsystem
  public final ShooterSubsystem shooter = new ShooterSubsystem();

  //Feeder Subsystem
  public final FeederSubsystem feeder = new FeederSubsystem();
  //Elevator Subsystem
  public final ElevatorSubsystem elevator = new ElevatorSubsystem();
  public final ElevatorWithSpeed elevatorUp = new ElevatorWithSpeed(elevator, -1);
  public final ElevatorWithSpeed elevatorDown = new ElevatorWithSpeed(elevator, 1);

  //pivot Subsystem
  public final PivotSubsystem pivot = new PivotSubsystem();
  public final PivotwithSpeed pivotUp = new PivotwithSpeed(pivot,-.2);
  public final PivotwithSpeed pivotDown = new PivotwithSpeed(pivot,.2);

  //Climber SUBSYSTEM
  public final ClimberSubsystem climber = new ClimberSubsystem();

  //LED SUBSYSTEM
  private final LEDSubsystem led = new LEDSubsystem(1);

  //DRIVETRAIN / SWERVE
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  //COMPLEX COMMANDS
  TELEShootCommand  teleShootCommand = new TELEShootCommand(shooter,feeder,led);
  IntakeCommand intakeCommand = new IntakeCommand(intake,feeder,led,pivot,elevator);
  
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-xbox.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-xbox.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-xbox.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    xbox.a().whileTrue(drivetrain.applyRequest(() -> brake)); 
    xbox.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-xbox.getLeftY(), -xbox.getLeftX()))));

    // RESET THE FIELD-CENTRIC HEADING ON THE LEFT BUMPER
    xbox.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    //DEFAULT COMMANDS
    shooter.setDefaultCommand(shooter.withDisable());
    feeder.setDefaultCommand(feeder.withDisable());
    intake.setDefaultCommand(intake.withDisable());
    pivot.setDefaultCommand(pivot.holdPosition());
    elevator.setDefaultCommand(elevator.holdPosition());
    climber.setDefaultCommand(climber.stop());

/* DRIVE CONTROLS */
//Shoot
  xbox.rightTrigger().onTrue(teleShootCommand); 
//Intake
  xbox.rightBumper().onTrue(intakeCommand);
//Outtake
  xbox.leftBumper().whileTrue(new ParallelCommandGroup(intake.outtakeCommand(),feeder.outtakeCommand(),pivot.intakePositionCommand()));
//AmpOuttake
/* tba */
// Pivot Up & Down
  xbox.pov(0).whileTrue(pivotUp);
  xbox.pov(180).whileTrue(pivotDown);
//Positions
  xbox.a().onTrue(pivot.intakePositionCommand());
  xbox.b().onTrue(pivot.subwooferPositionCommand());
  xbox.x().onTrue(pivot.setHomePositionCommand());
  xbox.y().onTrue(pivot.ampPositionCommand());
  /* !!!!!!! SOME BUTTONS ARE THE SAME, WILL FIX :)!!!!! */

//////

/*Operator Controls */

//Climb Set Up
  joystick.button(5).onTrue(climber.setUpPosition());
//Climb All the way
//tba//     
//Elevator Position Up
 joystick.button(3).onTrue(elevator.setAMPPosition());
//Elevator Position Down
  joystick.button(4).onTrue(elevator.setHomePosition());
//Elevator AMP Position
  joystick.button(6).onTrue(new ParallelCommandGroup(elevator.setAMPPosition(),pivot.ampPositionCommand()));


  
  }

  public RobotContainer() {
    configureBindings();

    //SET-FIELD ORIENTATION
     NamedCommands.registerCommand("setFieldRelative",drivetrain.runOnce(() ->  drivetrain.seedFieldRelative()));
  //NamedCommands.registerCommand("startIntake", intakecommandAuto);
  //NamedCommands.registerCommand("STAGEIntake", intakecommandAutoSTAGE);
  //NamedCommands.registerCommand("SubwooferPivot",autoPivotSub);
  //NamedCommands.registerCommand("FeederShoot", feederShot);
  //NamedCommands.registerCommand("AutoShoot",autoshoot );
  //NamedCommands.registerCommand("PivotShot", autoshootintakepos);
  NamedCommands.registerCommand("PivotShot", teleShootCommand);
     autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
     SmartDashboard.putData("Auto Mode", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
