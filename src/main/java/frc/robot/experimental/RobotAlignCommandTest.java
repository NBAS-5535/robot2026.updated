package frc.robot.experimental;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Vision.Vision;


public class RobotAlignCommandTest extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private int tagId;
  private final Pose3d targetPose; // The desired field pose

  private static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(5., 0.0);
  private final ProfiledPIDController rotationalPidController = new ProfiledPIDController(0.10, 0.0,0.0, constraints);
  private final ProfiledPIDController xPidController = new ProfiledPIDController(0.4, 0.0, 0.0006, constraints);
  private final ProfiledPIDController yPidController = new ProfiledPIDController(0.3, 0, 0, constraints);
  
  private static final SwerveRequest.RobotCentric alignRequest = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

  public RobotAlignCommandTest(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision, int tagId) {
    this.drivetrain = drivetrain;
    this.tagId = tagId;
    //SmartDashboard.putNumber("RobotAlignCommandTest/TagToFindIn", tagId);

    // retrieve the target pose from the FieldLayout based on the tag ID
    this.targetPose = Vision.fieldLayout.getTagPose(tagId).get();
    SmartDashboard.putString("RobotAlignCommandTest/targetPose", this.targetPose.toString());
    //SmartDashboard.putNumberArray("RobotAlignCommandTest/Tolerances", new double[] {rotationalPidController.getPositionTolerance(), 
    //                                      rotationalPidController.getVelocityTolerance()});

    addRequirements(drivetrain);
  }


  @Override
  public void initialize() {
    // Reset the controllers with the current robot pose
    xPidController.reset(drivetrain.getCurrentPose().getX());
    yPidController.reset(drivetrain.getCurrentPose().getY());
    rotationalPidController.reset(drivetrain.getCurrentPose().getRotation().getRadians());
    xPidController.setTolerance(0.01);
    yPidController.setTolerance(0.01);
    rotationalPidController.setTolerance(3.0 * Math.PI/180.0); // 3 degrees 
    SmartDashboard.putNumber("RobotAlignCommandTest/InitialPose_X", drivetrain.getCurrentPose().getX());
    SmartDashboard.putNumber("RobotAlignCommandTest/InitialPose_Y", drivetrain.getCurrentPose().getY());
    SmartDashboard.putNumber("RobotAlignCommandTest/InitialPose_Rot", drivetrain.getCurrentPose().getRotation().getRadians());
  }

  @Override
  public void execute() {
    
    drivetrain.updateVisionPose();

    Pose2d currentPose = drivetrain.getCurrentPose();

    SmartDashboard.putNumber("RobotAlignCommandTest/CurrentPose_X", currentPose.getX());
    SmartDashboard.putNumber("RobotAlignCommandTest/CurrentPose_Y", currentPose.getY());
    SmartDashboard.putNumber("RobotAlignCommandTest/CurrentPose_Rot", currentPose.getRotation().getRadians());

    // Calculate movement based on PID
    double velocityX = xPidController.calculate(currentPose.getX(), targetPose.getX());
    double velocityY = yPidController.calculate(currentPose.getY(), targetPose.getY());
    double rotationalRate = rotationalPidController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().toRotation2d().getRadians());

    SmartDashboard.putNumber("RobotAlignCommandTest/xPidController", velocityX);
    SmartDashboard.putNumber("RobotAlignCommandTest/yPidController", velocityY);
    SmartDashboard.putNumber("RobotAlignCommandTest/rotationalPidController", rotationalRate);
    
    SmartDashboard.putNumber("RobotAlignCommandTest/TagID", tagId);


    /* move the robot to correct position */
    /*
    drivetrain.setControl(alignRequest.withVelocityX(velocityX)
                                      .withVelocityY(velocityY)
                                      .withRotationalRate(rotationalRate));
    */
    drivetrain.setControl(alignRequest.withRotationalRate(rotationalRate));
  }

  @Override
  public boolean isFinished() {
    // return xPidController.atGoal() && yPidController.atGoal() && rotationalPidController.atGoal();
    return rotationalPidController.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.applyRequest(() -> idleRequest);
    
  }
}