package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.Vision.Vision;
import frc.robot.Vision.LimelightHelpers.RawFiducial;


public class AlignToClosestAprilTag extends Command {
  private final CommandSwerveDrivetrain drivetrain;

  private final double kSetpoint = 0.0; // Desired horizontal offset in degrees
  private final double kSetpointTolerance = 1.0; // Desired horizontal offset tolerance in degrees
 
  private  final PIDControllerConfigurable rotationalPidController = 
                    new PIDControllerConfigurable(0.1, 0., 0., kSetpoint, kSetpointTolerance); // tol=1deg
  private  final PIDControllerConfigurable xPidController = 
                    new PIDControllerConfigurable(0.05, 0., 0., kSetpoint, kSetpointTolerance);

  private double rotationalRate = 0.0;
  private double velocityX = 0.1;
  
  private static final SwerveRequest.RobotCentric alignRequest = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

  RawFiducial[] fiducials;

  public AlignToClosestAprilTag(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }


  @Override
  public void initialize() {
    // Reset the controllers with the current robot pose
    //rotationalPidController.reset();
    LimelightHelpers.setLEDMode_PipelineControl("limelight");
    double t = LimelightHelpers.getFiducialID("");
    fiducials = LimelightHelpers.getRawFiducials("limelight");
    SmartDashboard.putNumber("AlignToClosestAprilTag/TagToFindIn", t);
  }

  @Override
  public void execute() {
        // Check if a target is valid
    if (LimelightHelpers.getTV("limelight")) {
      SmartDashboard.putNumber("AlignToClosestAprilTag/idInSight", LimelightHelpers.getFiducialID("limelight"));
      // Get the horizontal offset from the Limelight
      double tx = LimelightHelpers.getTX("limelight");
      SmartDashboard.putNumber("AlignToClosestAprilTag/tx", tx);

      

      rotationalRate = rotationalPidController.calculate(tx, kSetpoint);
      //velocityX = xPidController.calculate(fiducial.distToRobot, 0.1);

      SmartDashboard.putNumber("AlignToClosestAprilTag/rotationalPidController", rotationalRate);
      SmartDashboard.putNumber("AlignToClosestAprilTag/distToRobot", fiducials[0].distToRobot);
      drivetrain.setControl(alignRequest.withRotationalRate(rotationalRate).withVelocityX(velocityX));

      // check we reached the setpoint 
      if (rotationalPidController.atSetpoint()) {
              System.out.println("STOP alignment");
              // interruptCommand = true;
              this.end(true);
      }
    }
  }

  @Override
  public boolean isFinished() {
    boolean temp = rotationalPidController.atSetpoint();
    SmartDashboard.putBoolean("AlignToClosestAprilTag/AlignFinished", temp);
    return temp;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.applyRequest(() -> idleRequest);
    
  }

}