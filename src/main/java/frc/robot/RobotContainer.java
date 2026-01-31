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
import frc.robot.subsystems.RangeSensorSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.FindAndAlignCommand;
import frc.robot.commands.FollowAprilTagCommand;
import frc.robot.commands.NewTurretCommand;
import frc.robot.commands.TurretAlignCommand;
import frc.robot.experimental.LimelightSubsystem;
import frc.robot.experimental.RobotAlignCommand;

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

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    //private final LimelightSubsystem limelight = new LimelightSubsystem();
    private final VisionSubsystem m_vision = new VisionSubsystem();

    /** TurretSubsystem */
    private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();

     /** RangeSensorSubsystem */
     private final RangeSensorSubsystem m_sensorSubsystem = new RangeSensorSubsystem();

     /* Path follower */
    private final SendableChooser<Command> autoChooser;
    /*  autonomous dropdown menu */
    private SendableChooser<String> autonomousChooser;

    public RobotContainer() {
        
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("PathPlanner Scenario", autoChooser);
        /* autonomous position chooser 
        autonomousChooser = new SendableChooser<>();
        autonomousChooser.setDefaultOption("No Action", "None");
        autonomousChooser.addOption("Blue_1", "Start_Right");
        SmartDashboard.putData("AutonomousMenu", autonomousChooser);
        */

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

        // reset the field-centric heading on left bumper press
        //joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        /* run turret motor in suck-in and push-out modes */
         
        boolean useTurretSubsystem = true;
        if ( useTurretSubsystem ) {
            // y() -> Turreting the robot UP
            //joystick.a().whileTrue(m_turretSubsystem.runTurretRightCommand());
           joystick.a().onTrue(new NewTurretCommand(m_turretSubsystem, drivetrain));




            // start() -> Lowering the robot DOWN
            joystick.b().whileTrue(m_turretSubsystem.runTurretLeftCommand());

            //joystick.rightBumper().onTrue(new TurretAlignCommand(m_turretSubsystem, limelight, 0));
        }
            

        /* get distance and move for that amount */
        boolean getRangeAndGoTest = false;
        if (getRangeAndGoTest) {
            joystick.y().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> drivetrain.setCurrentPose()),
                drivetrain.sysIdDynamic(Direction.kForward)
                    .until(() -> drivetrain.isDesiredPoseReached(m_sensorSubsystem.getDistance()))
            ));
        }

        /* align robot */
        boolean alignTesting = true;
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

            /* Experimental */
            joystick.povUp().onTrue(
                new RobotAlignCommand(drivetrain, m_vision, 13)
            );  

            /* reset pipeline to generic and align to the closest AprilTag*/
            joystick.x().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> VisionSubsystem.setPipeline(0)),
                new AlignCommand(drivetrain, m_vision, 0)
            ));

            /* Face the April Tag while moving*/
            joystick.back().whileTrue(
                new FollowAprilTagCommand(drivetrain, m_vision)
            ); /**/

            /* Align with a specific AprilTag: 12*/
            joystick.y().onTrue(
                new AlignCommand(drivetrain, m_vision, 13)
            );
    
            /* use a preset pipeline index = 5 */
            joystick.rightBumper().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> VisionSubsystem.setPipeline(5)),
                new AlignCommand(drivetrain, m_vision, 0)
            )); 

            /* use a preset pipeline index = 3 */
            joystick.leftBumper().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> VisionSubsystem.setPipeline(3)),
                new AlignCommand(drivetrain, m_vision, 13)
            ));
             /* use a preset pipeline index = 0 */
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
