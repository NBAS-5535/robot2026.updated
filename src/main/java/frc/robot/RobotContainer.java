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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DynamicTurretSubsystem;
import frc.robot.subsystems.DynamicTurretSubsystem.DynamicTurretSetpoints;
import frc.robot.subsystems.HoodSubsystem.HoodSetpoints;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.HopperSubsystem.HopperSetpoints;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FuelIntakeSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.RangeSensorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.AlignToClosestAprilTag;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.FindAndAlignCommand;
import frc.robot.commands.FollowAprilTagCommand;
import frc.robot.commands.NewTurretCommand;
import frc.robot.commands.ResetPoseFromAprilTagCommand;
import frc.robot.commands.TurretAlignCommand;
import frc.robot.experimental.IntakeSubsystem;
import frc.robot.experimental.LimelightSubsystem;
import frc.robot.commands.RobotAlignCommand;
import frc.robot.experimental.RobotAlignCommandTest;
import frc.robot.experimental.RobotAlignCommandWithLimeLight;
import frc.robot.experimental.TwoMotorShooterSubsystem;

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
    private final VisionSubsystem m_vision = new VisionSubsystem();

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
    private final HopperSubsystem m_hopper = new HopperSubsystem();

     /* Path follower */
    private final SendableChooser<Command> autoChooser;
    /*  autonomous dropdown menu */
    private SendableChooser<String> autonomousChooser;

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
        }

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // if direction is WRONG reset the field-centric by 180 degrees via rightBumper()
        joystick.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric(new Rotation2d(Math.PI))));

        boolean useHood = true;
        if (useHood){
            //copilot.povUp().whileTrue(new InstantCommand(() -> m_hood.runHoodInCommand()));
            //copilot.povDown().whileTrue(new InstantCommand(() -> m_hood.runHoodOutCommand()));
            copilot.a().onTrue(
                m_hood.setSetpointCommand(HoodSetpoints.k6ft)
            );
            copilot.b().onTrue(
                m_hood.setSetpointCommand(HoodSetpoints.k9ft)
            );
             copilot.x().onTrue(
                m_hood.setSetpointCommand(HoodSetpoints.k13ft)
            );
            copilot.y().onTrue(
                m_hood.setSetpointCommand(HoodSetpoints.k15ft)
            );
        }

        boolean useShooter = true;
        if (useShooter){
            copilot.leftBumper().onTrue(new InstantCommand(() -> m_shooter.fastMode()));
            copilot.rightBumper().onTrue(new InstantCommand(() -> m_shooter.stopShooter()));
        }

        boolean useHopper = true;
        if (useHopper){
            copilot.povUp().onTrue(m_hopper.setSetpointCommand(HopperSetpoints.kBase));
            copilot.povDown().onTrue(m_hopper.setSetpointCommand(HopperSetpoints.ktiltSetpoint));
        }

        boolean useFeeder = true;
        if (useFeeder){
            //copilot.povRight().onTrue(new InstantCommand(() -> m_feeder.setPower("lead", 0.5)));
            //copilot.povLeft().onTrue(new InstantCommand(() -> m_feeder.setPower("follow", 0.5)));
            // may have to start both motors at the same time to prevent jamming
            copilot.povRight().onTrue(new InstantCommand(() -> m_feeder.setPower("both", 0.8)));
            // STOP command for feeder motors
            copilot.povLeft().onTrue(new InstantCommand(() -> m_feeder.setPower("both", 0.0)));
        }
        
        /* run turret motor in suck-in and push-out modes */
         
        boolean useTurretSubsystem = false;
        if ( useTurretSubsystem ) {
             //original TurretSubsustem
            //joystick.a().onTrue(new NewTurretCommand(m_dynamicTurretSubsystem, drivetrain));
            //joystick.a().whileTrue(m_dynamicTurretSubsystem.runDynamicTurretRightCommand());
            //joystick.b().whileTrue(m_dynamicTurretSubsystem.runDynamicTurretLeftCommand());
            

            /* to determine encoder setting versus deg of rotation */
            /*
            joystick.a().onTrue(m_dynamicTurretSubsystem.setSetpointCommand(DynamicTurretSetpoints.kPointAtTargetSetpoint));
            joystick.b().onTrue(m_dynamicTurretSubsystem.setSetpointCommand(DynamicTurretSetpoints.kMoveLeftSetpoint));
            //joystick.povUp().onTrue(m_dynamicTurretSubsystem.setSetpointCommand(DynamicTurretSetpoints.kMoveRightSetpoint));
            joystick.povRight().onTrue(m_dynamicTurretSubsystem.setSetpointCommand(DynamicTurretSetpoints.kBase));
            joystick.x().whileTrue(new AutoAlignCommand(m_dynamicTurretSubsystem, drivetrain));
            joystick.y().onTrue(new ResetPoseFromAprilTagCommand(drivetrain, m_dynamicTurretSubsystem));
            */
            joystick.povLeft().onTrue(
                new InstantCommand(() -> drivetrain.setInitializePose()));

                /* Experimental */
            joystick.povDown().onTrue(
                new AlignToClosestAprilTag(drivetrain)
            );

            joystick.povUp().onTrue(new InstantCommand(() -> drivetrain.updateVisionPose_MT_1and2()));
            //joystick.b().whileTrue(m_dynamicTurretSubsystem.runDynamicTurretLeftCommand());
        }
            

        /* get distance and move for that amount */
        /*
        boolean getRangeAndGoTest = false;
        if (getRangeAndGoTest) {
            joystick.y().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> drivetrain.setCurrentPose()),
                drivetrain.sysIdDynamic(Direction.kForward)
                    .until(() -> drivetrain.isDesiredPoseReached(m_sensorSubsystem.getDistance() - 1.))
            ));
        }
        */

        /* align robot */
        boolean alignTesting = false;
        if (alignTesting) {
            /* odd behavior when 3D tracking is ON!!!!!
            int testTagId = 0;
            joystick.x().onTrue(
                new AlignCommand(drivetrain, m_vision, testTagId)
            );
            
            joystick.back().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> AlignCommand.setTagId(1)),
                new AlignCommand(drivetrain, m_vision, 1)
            ));
            */
            joystick.povLeft().onTrue(
                new InstantCommand(() -> drivetrain.setInitializePose()));
            /* Experimental */
            joystick.povUp().onTrue(
                new AlignToClosestAprilTag(drivetrain)
            );  
            /*
            joystick.povDown().onTrue(new SequentialCommandGroup(
                new RobotAlignCommand(drivetrain, 10)
            )); 

            joystick.povRight().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> RobotAlignCommandWithLimeLight.setTagId(13)),
                new RobotAlignCommandWithLimeLight(drivetrain, 13)
            ));

            joystick.povLeft().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> RobotAlignCommand.setTagId(9)),
                new RobotAlignCommand(drivetrain, 9)
            ));

            // reset pipeline to generic and align to the closest AprilTag
            joystick.x().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> VisionSubsystem.setPipeline(0)),
                new AlignCommand(drivetrain, m_vision, 0)
            ));


            // Face the April Tag while moving
            joystick.back().whileTrue(
                new FollowAprilTagCommand(drivetrain, m_vision)
            ); 

            // Align with a specific AprilTag: 12
            joystick.y().onTrue(
                new AlignCommand(drivetrain, m_vision, 13)
            );
    
            // use a preset pipeline index = 5 
            joystick.rightBumper().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> VisionSubsystem.setPipeline(5)),
                new AlignCommand(drivetrain, m_vision, 0)
            )); 

            //use a preset pipeline index = 3 
            joystick.leftBumper().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> VisionSubsystem.setPipeline(3)),
                new AlignCommand(drivetrain, m_vision, 13)
            ));
             // use a preset pipeline index = 0 
            joystick.start().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> VisionSubsystem.setPipeline(0))
            ));
            /*
            joystick.back().whileTrue(
                drivetrain.applyRequest(() ->
                    drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-LimelightHelpers.getTA("") * MaxAngularRate) // Drive counterclockwise with negative X (left)
                )
            );
            */
        }

        /* align robot */
        boolean findAndAlignTesting = false;
        if (findAndAlignTesting) {
            int testTagId = 0;
            joystick.povRight().onTrue(
                new FindAndAlignCommand(drivetrain, m_vision, testTagId)
            );

            joystick.povLeft().onTrue(
                new FindAndAlignCommand(drivetrain, m_vision, 12)
            );
            
            /* use a preset pipeline index = 5 */
            joystick.povDown().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> VisionSubsystem.setPipeline(5)),
                new FindAndAlignCommand(drivetrain, m_vision, 1)
            )); 

            /* use a preset pipeline index = 3 */
            joystick.povUp().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> VisionSubsystem.setPipeline(3)),
                new FindAndAlignCommand(drivetrain, m_vision, 12)
            ));
        }
        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureNamedCommands() {
        NamedCommands.registerCommand("StartIntakeMotor", new InstantCommand(() -> m_intake.fastIntake()));
        NamedCommands.registerCommand("StopIntakeMotor", new InstantCommand(() -> m_intake.stopIntake()));
        NamedCommands.registerCommand("StartFeederMotors", new InstantCommand(() -> m_feeder.setPower("both", 0.8)));
        NamedCommands.registerCommand("StopFeederMotors", new InstantCommand(() -> m_feeder.setPower("both", 0.0)));
        NamedCommands.registerCommand("StartShooterMotors", new InstantCommand(() -> m_shooter.fastMode()));
        NamedCommands.registerCommand("StopShooterMotors", new InstantCommand(() -> m_shooter.stopShooter()));
        NamedCommands.registerCommand("MoveHoodTo6ftSetpoint", m_hood.setSetpointCommand(HoodSetpoints.k6ft));
        NamedCommands.registerCommand("MoveHoodToVariableSetpoint", new InstantCommand(() -> m_hood.setVariableHoodSetpoint(12.)));
        NamedCommands.registerCommand("SetupAndStartShooting", 
                                new SequentialCommandGroup(
                                    m_hood.setSetpointCommand(HoodSetpoints.k6ft),
                                    new InstantCommand(() -> m_shooter.fastMode()).withTimeout(1),
                                    new InstantCommand(() -> m_feeder.setPower("both", 0.8)))
                                    );
        NamedCommands.registerCommand("SetupAndStartShootingOnTheRight", 
                                new SequentialCommandGroup(
                                    new InstantCommand(() -> m_hood.setVariableHoodSetpoint(12.)),
                                    new InstantCommand(() -> m_shooter.fastMode()).withTimeout(1),
                                    new InstantCommand(() -> m_feeder.setPower("both", 0.8)))
                                    );
        NamedCommands.registerCommand("StopShooting", 
                                new SequentialCommandGroup(                               
                                    new InstantCommand(() -> m_feeder.setPower("both", 0.0)),
                                    new InstantCommand(() -> m_shooter.stopShooter())));
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
