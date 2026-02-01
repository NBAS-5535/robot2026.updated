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
import frc.robot.Vision.Vision;


public class RobotAlignCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private int tagId;
  private final Pose3d targetPose; // The desired field pose

 
  private  final PIDControllerConfigurable rotationalPidController = 
                    new PIDControllerConfigurable(0.05000, 0.000000, 0.001000, 0.0, 1.); // tol=1deg

  
  private static final SwerveRequest.RobotCentric alignRequest = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

  public RobotAlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision, int tagId) {
    this.drivetrain = drivetrain;
    this.tagId = tagId;
    //SmartDashboard.putNumber("RobotAlignCommand/TagToFindIn", tagId);

    // retrieve the target pose from the FieldLayout based on the tag ID
    this.targetPose = Vision.fieldLayout.getTagPose(tagId).get();
    SmartDashboard.putString("RobotAlignCommand/targetPose", this.targetPose.toString());
    //SmartDashboard.putNumberArray("RobotAlignCommand/Tolerances", new double[] {rotationalPidController.getPositionTolerance(), 
    //                                      rotationalPidController.getVelocityTolerance()});

    addRequirements(drivetrain);
  }


  @Override
  public void initialize() {
    // Reset the controllers with the current robot pose
    rotationalPidController.reset();
    SmartDashboard.putNumber("RobotAlignCommand/InitialPose_Rot", drivetrain.getCurrentPose().getRotation().getRadians());
  }

  @Override
  public void execute() {
    
    drivetrain.updateVisionPose();

    Pose2d currentPose = drivetrain.getCurrentPose();
    SmartDashboard.putNumber("RobotAlignCommand/CurrentPose_Rot", currentPose.getRotation().getRadians());

    // Calculate movement based on PID
    double rotationalRate = rotationalPidController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().toRotation2d().getRadians());

    SmartDashboard.putNumber("RobotAlignCommand/rotationalPidController", rotationalRate);
    SmartDashboard.putNumber("RobotAlignCommand/TagID", tagId);

    if (rotationalPidController.atSetpoint()) {
        System.out.println("STOP alignment");
        this.end(true);
    }

    drivetrain.setControl(alignRequest.withRotationalRate(rotationalRate));
  }

  @Override
  public boolean isFinished() {
    boolean temp = rotationalPidController.atSetpoint();
    SmartDashboard.putBoolean("RobotAlignCommand/AlignFinished", temp);
    return temp;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.applyRequest(() -> idleRequest);
    
  }
}