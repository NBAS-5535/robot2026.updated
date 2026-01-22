// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FollowAprilTagCommand extends Command {
  private final CommandSwerveDrivetrain m_drivetrain;
  private final VisionSubsystem m_limelight;

  double kP_x = 0.01;
  double kP_y = 0.05;
  boolean hasTarget;

  private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
  
  /** Creates a new FollowAprilTagCommand. */
  public FollowAprilTagCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight) {
    m_drivetrain = drivetrain;
    m_limelight = limelight;
    
    addRequirements(m_drivetrain, m_limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tx = LimelightHelpers.getTX("limelight");
    double ty = LimelightHelpers.getTY("limelight");
    hasTarget = LimelightHelpers.getTV("limelight");

    if (hasTarget) {
        // Use a simple PID or proportional gain to determine speeds
        double forwardSpeed = ty * kP_y; // ty is distance-related
        double sidewaysSpeed = tx * kP_x;
        
        // Apply the request to the drivetrain
        m_drivetrain.setControl(
            m_drivetrain.visionFollowRequest
                .withVelocityX(forwardSpeed)
                .withVelocityY(sidewaysSpeed)
                .withTargetDirection(Rotation2d.fromDegrees(0)) // Face tag directly
        );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.applyRequest(() -> idleRequest);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return LimelightHelpers.getTV("limelight");
  }
}
