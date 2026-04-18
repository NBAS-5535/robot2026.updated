// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem.HoodSetpoints;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.HopperSubsystem.HopperSetpoints;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FuelIntakeSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Vision.LimelightHelpers;

import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.8; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();


    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController copilot = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    //private final LimelightSubsystem limelight = new LimelightSubsystem();

    /* IntakeSubsystem */
    //private final IntakeSubsystem m_intake = new IntakeSubsystem(0.0);
    // using Falcon/TalonFX
    private final FuelIntakeSubsystem m_intake = new FuelIntakeSubsystem();

    /** TurretSubsystem */
    //private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();
    //private final DynamicTurretSubsystem m_dynamicTurretSubsystem = new DynamicTurretSubsystem(drivetrain);

     /** RangeSensorSubsystem */
    //private final RangeSensorSubsystem m_sensorSubsystem = new RangeSensorSubsystem();

    /** Hood */
    private final HoodSubsystem m_hood = new HoodSubsystem();

    /** Shooter */
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();

    /** Feeder */
    private final FeederSubsystem m_feeder = new FeederSubsystem();

    /** Hopper */
    //private final HopperSubsystem m_hopper = new HopperSubsystem();

     /* Path follower */
    private final SendableChooser<Command> autoChooser;
    /*  autonomous dropdown menu */
    private SendableChooser<String> autonomousChooser;

    private final double feederDelay = 0.5;

    public RobotContainer() {
        // create some NamedCommands for PathPlanner
        configureNamedCommands();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("PathPlanner Scenario", autoChooser);
        /* autonomous position chooser */
        autonomousChooser = new SendableChooser<>();
        autonomousChooser.setDefaultOption("No Action", "None");
        autonomousChooser.addOption("Blue_1", "Start_Right");
        autonomousChooser.addOption("Blue_3", "Blue_Right_Scenario");
        SmartDashboard.putData("AutonomousMenu", autonomousChooser);
        /**/

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
            
        );
        
               

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        /*
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));*/

        boolean runSysIdCommands = false;
        if (runSysIdCommands) {
            // Run SysId routines when holding back/start and X/Y.
            // Note that each routine should be run exactly once in a single log.
            joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
            joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
            joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
            joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        }

        /* operate the intakemotor */
        boolean useIntake = true;
        if ( useIntake ) {
            //copilot.a().onTrue(new InstantCommand(() -> m_intake.fastIntakeCommand()));
            //copilot.b().onTrue(new InstantCommand(() -> m_intake.stopIntakeCommand()));
            joystick.a().onTrue(new InstantCommand(() -> m_intake.fastIntake()));
            //copilot.x().onTrue(new InstantCommand(() -> m_intake.slowIntake()));
            joystick.b().onTrue(new InstantCommand(() -> m_intake.stopIntake()));
            joystick.povDown().whileTrue(new InstantCommand(() -> m_intake.reverseIntake()));
        }

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // if direction is WRONG reset the field-centric by 180 degrees via rightBumper()
        joystick.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric(new Rotation2d(Math.PI))));

        boolean useHood = false;
        if (useHood){
            //copilot.povUp().whileTrue(new InstantCommand(() -> m_hood.runHoodInCommand()));
            //copilot.povDown().whileTrue(new InstantCommand(() -> m_hood.runHoodOutCommand()));
            /* 
            joystick.x().onTrue(
                m_hood.setSetpointCommand(HoodSetpoints.kBase)
            );*/
            copilot.a().onTrue(
                m_hood.setSetpointCommand(HoodSetpoints.k6ft)
            );
            copilot.b().onTrue(
                m_hood.setSetpointCommand(HoodSetpoints.k9ft)
            );
             copilot.x().onTrue(
                m_hood.setSetpointCommand(HoodSetpoints.k13ft)
            );/* 
            copilot.y().onTrue(
                m_hood.setSetpointCommand(HoodSetpoints.k15ft)
            );*/
            copilot.y().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> m_hood.setVariableHoodSetpoint(drivetrain.computeDistanceToTower())),
                m_hood.setSetpointCommand(HoodSetpoints.kvariable)
            ));
        }

        boolean useShooter = false;
        if (useShooter){
            copilot.leftBumper().onTrue(new InstantCommand(() -> m_shooter.fastMode()));
            copilot.rightBumper().onTrue(new InstantCommand(() -> m_shooter.stopShooter()));
        }

        boolean useHopper = false;
        if (useHopper){
            //joystick.povUp().onTrue(m_hopper.setSetpointCommand(HopperSetpoints.kBaseUpright));
            //joystick.povDown().onTrue(m_hopper.setSetpointCommand(HopperSetpoints.kDownSetpoint));
            //copilot.y().onTrue(m_hopper.setSetpointCommand(HopperSetpoints.ktiltSetpoint));
        }

        boolean useFeeder = true;
        if (useFeeder){
            //copilot.povRight().onTrue(new InstantCommand(() -> m_feeder.setPower("lead", 0.5)));
            //copilot.povLeft().onTrue(new InstantCommand(() -> m_feeder.setPower("follow", 0.5)));
            // may have to start both motors at the same time to prevent jamming
            copilot.povRight().onTrue(new InstantCommand(() -> m_feeder.setPower("both", 0.7)));
            // STOP command for feeder motors
            copilot.povLeft().onTrue(new InstantCommand(() -> m_feeder.setPower("both", 0.0)));
        }
        
         boolean comboControlsTesting = true;
        if (comboControlsTesting) {
            // close shooting position
            copilot.a().onTrue(new SequentialCommandGroup(
                m_hood.setSetpointCommand(HoodSetpoints.k6ft),
                new InstantCommand(() -> m_shooter.fastMode()),//.withTimeout(feederDelay),
                new WaitCommand(feederDelay),
                new InstantCommand(() -> m_feeder.setPower("both", 0.8))
            ));
            // medium shooting position
            copilot.b().onTrue(new SequentialCommandGroup(
                m_hood.setSetpointCommand(HoodSetpoints.k9ft),
                new InstantCommand(() -> m_shooter.fastMode()),//.withTimeout(feederDelay),
                new WaitCommand(feederDelay),
                new InstantCommand(() -> m_feeder.setPower("both", 0.8))
            ));
            // far shooting position
            copilot.x().onTrue(new SequentialCommandGroup(
                m_hood.setSetpointCommand(HoodSetpoints.k13ft),
                new InstantCommand(() -> m_shooter.fastMode()),//.withTimeout(feederDelay),
                new WaitCommand(feederDelay),
                new InstantCommand(() -> m_feeder.setPower("both", 0.8))
            ));
            copilot.y().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> m_hood.setVariableHoodSetpoint(drivetrain.computeDistanceToTower()))
            ));

            copilot.rightBumper().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> m_feeder.setPower("both", 0.0)),
                new InstantCommand(() -> m_shooter.stopShooter())
            ));

        }
        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureNamedCommands() {
        NamedCommands.registerCommand("ReInitializeComboPose", new InstantCommand(() -> drivetrain.reInitializePoseForBRCombo()));
        NamedCommands.registerCommand("StartIntakeMotor", new InstantCommand(() -> m_intake.fastIntake()));
        NamedCommands.registerCommand("StopIntakeMotor", new InstantCommand(() -> m_intake.stopIntake()));
        NamedCommands.registerCommand("StartFeederMotors", new InstantCommand(() -> m_feeder.setPower("both", 0.8)));
        NamedCommands.registerCommand("StopFeederMotors", new InstantCommand(() -> m_feeder.setPower("both", 0.0)));
        NamedCommands.registerCommand("StartShooterMotors", new InstantCommand(() -> m_shooter.fastMode()));
        NamedCommands.registerCommand("StopShooterMotors", new InstantCommand(() -> m_shooter.stopShooter()));
        //NamedCommands.registerCommand("HopperDown", m_hopper.setSetpointCommand(HopperSetpoints.kDownSetpoint));
        //NamedCommands.registerCommand("HopperUp", m_hopper.setSetpointCommand(HopperSetpoints.kBaseUpright));
        NamedCommands.registerCommand("MoveHoodTo6ftSetpoint", m_hood.setSetpointCommand(HoodSetpoints.k6ft));
        NamedCommands.registerCommand("MoveHoodToVariableSetpoint", new InstantCommand(() -> m_hood.setVariableHoodSetpoint(12.)));
        NamedCommands.registerCommand("SetupAndStartShooting", 
                                new SequentialCommandGroup(
                                    m_hood.setSetpointCommand(HoodSetpoints.k15ft),
                                    //new InstantCommand(() -> m_shooter.fastMode()),//.withTimeout(2),
                                    new InstantCommand(() -> m_feeder.setPower("both", 0.8)))
                                    );
        NamedCommands.registerCommand("SetupAndStartShootingOnTheRight", 
                                new SequentialCommandGroup(
                                    //new InstantCommand(() -> m_hood.setVariableHoodSetpoint(12.)),
                                    m_hood.setSetpointCommand(HoodSetpoints.kauto13ft),
                                    //new InstantCommand(() -> m_shooter.fastMode()),//.withTimeout(2),
                                    new InstantCommand(() -> m_feeder.setPower("both", 0.8)))
                                    );
        NamedCommands.registerCommand("StopShooting", 
                                new SequentialCommandGroup(                               
                                    new InstantCommand(() -> m_feeder.setPower("both", 0.0)),
                                    new InstantCommand(() -> m_shooter.stopShooter())));
        NamedCommands.registerCommand("SetHood", 
                                new SequentialCommandGroup(
                                    m_hood.setSetpointCommand(HoodSetpoints.kauto13ft))
                                    );
        //NamedCommands.registerCommand("LowerHopper", 
        //                       new SequentialCommandGroup(
        //                            m_hopper.setSetpointCommand(HopperSetpoints.kDownSetpoint))
        //                            );
        //NamedCommands.registerCommand("RaiseHopper", 
        //                        new SequentialCommandGroup(
        //                            m_hopper.setSetpointCommand(HopperSetpoints.kBaseUpright))
        //                            );
    }

    public Command getAutonomousCommand() {
        Command autoCommand = null;
        boolean simpleDriveForward = false;
        if (simpleDriveForward) 
        {
            // Simple drive forward auton
            final var idle = new SwerveRequest.Idle();
            autoCommand = Commands.sequence(
                // Reset our field centric heading to match the robot
                // facing away from our alliance station wall (0 deg).
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                // Then slowly drive forward (away from us) for 5 seconds.
                drivetrain.applyRequest(() ->
                    drive.withVelocityX(0.5)
                        .withVelocityY(0)
                        .withRotationalRate(0)
                )
                .withTimeout(1.0),
                // Finally idle for the rest of auton
                drivetrain.applyRequest(() -> idle)
            );
        } else {
            autoCommand = autoChooser.getSelected();
        }
        return autoCommand;
    }
}
