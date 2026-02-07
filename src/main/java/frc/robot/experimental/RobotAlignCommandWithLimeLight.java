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

  private double rotationalRate = 0.0;
  private double velocityX = 0.1;
  
  //private RawFiducial[] fiducials;
  private boolean interruptCommand = false;
  private static int m_testCase;// = 0; // 0: align to the closest tag, 1: align to a specific m_tagId, 2: align to any tag (probably the closest one)

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
        // Check if a target is valid
    if (LimelightHelpers.getTV("limelight")) {
      SmartDashboard.putNumber("RobotAlignCommandWithLimeLight/idInSight", LimelightHelpers.getFiducialID("limelight"));
      // Get the horizontal offset from the Limelight
      double tx = LimelightHelpers.getTX("limelight");
      RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("limelight");
      SmartDashboard.putNumber("RobotAlignCommandWithLimeLight/tx", tx);

      switch ( m_testCase ) {
        case 0: 
          // align to the closet tag - whichever is in sight - LimeLightHelpers
          rotationalRate = rotationalPidController.calculate(tx, kSetpoint);
          //velocityX = xPidController.calculate(fiducial.distToRobot, 0.1);
          break;

        case 1: 
          // align to the closest tag only if it is the same as the input m_tagId
          if (LimelightHelpers.getFiducialID("limelight") == m_tagId) {
            rotationalRate = rotationalPidController.calculate(tx, kSetpoint);
            //velocityX = xPidController.calculate(fiducial.distToRobot, 0.1);
          } else {
            System.out.println("Not our tag - NO alignment");
            interruptCommand = true;
          }
          break;

        case 2: 
          // align to m_tagId if it is in sight, otherwise end/interrupt command
          // get info on a specific m_tagId
          for (RawFiducial fiducial : fiducials) {
            if (fiducial.id == m_tagId) {
              SmartDashboard.putBoolean("RobotAlignCommandWithLimeLight/TargetFound", true);
              SmartDashboard.putNumber("RobotAlignCommandWithLimeLight/DetectedTagID", fiducial.id);
              // Calculate movement based on PID
              rotationalRate = rotationalPidController.calculate(fiducial.txnc, kSetpoint);
              //double velocityX = xPidController.calculate(fiducial.distToRobot, 0.1);
            } else {
              System.out.println("Not my tag - NO alignment");
              interruptCommand = true;
            }
          }
          break;

      }

      SmartDashboard.putNumber("RobotAlignCommandWithLimeLight/rotationalPidController", rotationalRate);
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
    SmartDashboard.putBoolean("RobotAlignCommandWithLimeLight/AlignFinished", temp);
    SmartDashboard.putBoolean("RobotAlignCommandWithLimeLight/InterruptCommand", interruptCommand);
    return temp || interruptCommand;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.applyRequest(() -> idleRequest);
    
  }

  public static void setTagId(int tag){
    SmartDashboard.putNumber("RobotAlignCommandWithLimeLight/SettingTagID", tag);
    m_tagId = tag;
  }

  public static void setScenario(int testScenario){
    SmartDashboard.putNumber("RobotAlignCommandWithLimeLight/SettingCase", testScenario);
    m_testCase = testScenario;
  }

  /*
  public RawFiducial getIdOfClosestFiducial() {
    if (fiducials == null || fiducials.length == 0) {
        return null;
    }

    RawFiducial closest = fiducials[0];
    double area = closest.ta;

    for (RawFiducial fiducial : fiducials) {
        if (fiducial.ta > area) {
            closest = fiducial;
        }
    }
    SmartDashboard.putNumber("RobotAlignCommandWithLimeLight/closetTag", closest.id);
    return closest;
  }
    */
}