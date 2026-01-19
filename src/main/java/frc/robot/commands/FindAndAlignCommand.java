package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Vision.LimelightHelpers.RawFiducial;
//import frc.robot.Vision.LimelightHelpers.LimelightTarget_Fiducial;
//import edu.wpi.first.math.geometry.Pose3d;

public class FindAndAlignCommand extends Command {
  private final CommandSwerveDrivetrain m_drivetrain;
  private final VisionSubsystem m_limelight;
  private int m_tagId;

  private  final PIDControllerConfigurable rotationalPidController = new PIDControllerConfigurable(0.05000, 0.000000, 0.001000, 0.1);
  private  final PIDControllerConfigurable xPidController = new PIDControllerConfigurable(0.400000, 0.000000, 0.000600, 0.2);
  private  final PIDControllerConfigurable yPidController = new PIDControllerConfigurable(0.3, 0, 0, 0.3);
  private static final SwerveRequest.RobotCentric alignRequest = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
  private static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  public double rotationalRate = 0;
  public double velocityX = 0;
  private double rotationalRateToFindTag = -0.05;

  public FindAndAlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight, int tagId) {
    this.m_drivetrain = drivetrain;
    this.m_limelight = limelight;
    this.m_tagId = tagId;
    addRequirements(m_limelight);
  }


  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    
    RawFiducial fiducial;
    //LimelightTarget_Fiducial targetFiducial; 

    try {
      if (m_tagId == 0) {
        fiducial = m_limelight.getClosestFiducial();
        //targetFiducial = m_limelight.getTargetFiducialWithId(m_tagId);
      } else {
        fiducial = m_limelight.getFiducialWithId(m_tagId);
        //targetFiducial = m_limelight.getTargetFiducialWithId(m_tagId);
        //System.out.println("got it! " + String.valueOf(m_tagId));
      }

      rotationalRate = rotationalPidController.calculate(2*fiducial.txnc, 0.0) * 0.75* 0.9;
      
      //final double velocityX = xPidController.calculate(fiducial.distToRobot, 0.1) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.7;
      velocityX = xPidController.calculate(fiducial.distToRobot, 0.1) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.7;
        
      if (rotationalPidController.atSetpoint() && xPidController.atSetpoint()) {
        System.out.println("STOP alignment");
        this.end(true);
      }

      SmartDashboard.putNumber("FindAndAlignCommand/txnc", fiducial.txnc);
      SmartDashboard.putNumber("FindAndAlignCommand/ta", fiducial.ta);
      SmartDashboard.putNumber("FindAndAlignCommand/distToRobot", fiducial.distToRobot);
      SmartDashboard.putNumber("FindAndAlignCommand/distToCamera", fiducial.distToCamera);
      SmartDashboard.putNumber("FindAndAlignCommand/rotationalPidController", rotationalRate);
      SmartDashboard.putNumber("FindAndAlignCommand/xPidController", velocityX);
      SmartDashboard.putNumber("FindAndAlignCommand/TagID", m_tagId);


      /* move the robot to correct position */
      m_drivetrain.setControl(alignRequest.withRotationalRate(rotationalRate).withVelocityX(velocityX));
      
    } catch (VisionSubsystem.NoSuchTargetException nste) { 
      //System.out.println("No apriltag found");
      //System.out.println("No apriltag found");
      // rotate the bot until the desired apriltag is found
      m_drivetrain.setControl(alignRequest.withRotationalRate(rotationalRateToFindTag));//.withVelocityX(0.2));
    }
  }

  @Override
  public boolean isFinished() {
    boolean temp = rotationalPidController.atSetpoint() && xPidController.atSetpoint();
    SmartDashboard.putBoolean("FindAndAlignCommand/AlignFinished", temp);
    return temp;
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.applyRequest(() -> idleRequest);
    
  }

  public static void setTagId(int tagId) {
    //m_tagId = tagId;
  }
}