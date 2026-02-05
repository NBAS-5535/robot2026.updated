package frc.robot.experimental;

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
import frc.robot.commands.PIDControllerConfigurable;


public class RobotAlignCommandWithLimeLight extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private static int m_tagId;
  private final double kSetpoint = 0.0; // Desired horizontal offset in degrees
  private final double kSetpointTolerance = 1.0; // Desired horizontal offset tolerance in degrees
 
  private  final PIDControllerConfigurable rotationalPidController = 
                    new PIDControllerConfigurable(0.05000, 0.000000, 0.001000, kSetpoint, kSetpointTolerance); // tol=1deg
  private  final PIDControllerConfigurable xPidController = 
                    new PIDControllerConfigurable(0.05000, 0.000000, 0.001000, kSetpoint, kSetpointTolerance);

  private RawFiducial[] fiducials;

  private static final SwerveRequest.RobotCentric alignRequest = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

  public RobotAlignCommandWithLimeLight(CommandSwerveDrivetrain drivetrain, int tagId) {
    this.drivetrain = drivetrain;
    m_tagId = tagId;
    //SmartDashboard.putNumber("RobotAlignCommand/TagToFindIn", tagId);

    addRequirements(drivetrain);
  }


  @Override
  public void initialize() {
    // Reset the controllers with the current robot pose
    //rotationalPidController.reset();
    LimelightHelpers.setLEDMode_PipelineControl("limelight");
    double t = LimelightHelpers.getFiducialID("");
    SmartDashboard.putNumber("RobotAlignCommandWithLimeLight/TagToFindIn", t);
  }

  @Override
  public void execute() {
    
    // Get the horizontal offset from the Limelight
    double tx = LimelightHelpers.getTX("limelight");
    fiducials = LimelightHelpers.getRawFiducials("limelight");

    SmartDashboard.putNumber("RobotAlignCommandWithLimeLight/tx", tx);
    // Check if a target is valid
    if (LimelightHelpers.getTV("limelight")) {

      boolean justGo = false; // resetting tagId seems to work only with "false" 02/04/26 !!!!!

      for (RawFiducial fiducial : fiducials) {
        int tagId = fiducial.id; // This is the AprilTag ID
        if (tagId == m_tagId) {
          SmartDashboard.putBoolean("RobotAlignCommandWithLimeLight/TargetFound", true);
          SmartDashboard.putNumber("RobotAlignCommandWithLimeLight/DetectedTagID", tagId);
          if ( !justGo) {
            // Calculate movement based on PID
            double rotationalRate = rotationalPidController.calculate(tx, kSetpoint);
            //double velocityX = xPidController.calculate(fiducial.distToRobot, 0.1)
            SmartDashboard.putNumber("RobotAlignCommandWithLimeLight/rotationalPidController", rotationalRate);

            drivetrain.setControl(alignRequest.withRotationalRate(rotationalRate).withVelocityX(0.1));

            if (rotationalPidController.atSetpoint()) {
              System.out.println("STOP alignment");
              this.end(true);
            }
          }
        }
        
      }

      if ( justGo) {
        // Calculate movement based on PID
        double rotationalRate = rotationalPidController.calculate(tx, kSetpoint);
        //double velocityX = xPidController.calculate(fiducial.distToRobot, 0.1)
        SmartDashboard.putNumber("RobotAlignCommandWithLimeLight/rotationalPidController", rotationalRate);

        drivetrain.setControl(alignRequest.withRotationalRate(rotationalRate).withVelocityX(0.1));

        if (rotationalPidController.atSetpoint()) {
          System.out.println("STOP alignment");
          this.end(true);
        }
      }
    }

  }

  @Override
  public boolean isFinished() {
    boolean temp = rotationalPidController.atSetpoint();
    SmartDashboard.putBoolean("RobotAlignCommandWithLimeLight/AlignFinished", temp);
    return temp;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.applyRequest(() -> idleRequest);
    
  }

  public static void setTagId(int tag){
    SmartDashboard.putNumber("RobotAlignCommandWithLimeLight/SettingTagID", tag);
    m_tagId = tag;
  }
}