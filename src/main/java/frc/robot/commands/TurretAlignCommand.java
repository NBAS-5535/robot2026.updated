package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.Vision.LimelightHelpers.RawFiducial;
import frc.robot.experimental.LimelightSubsystem;
import frc.robot.experimental.LimelightSubsystem.NoSuchTargetException;

public class TurretAlignCommand extends Command {
  private final TurretSubsystem m_turret;
  private final LimelightSubsystem m_limelight;
  private int m_tagId;

  private RawFiducial[] fiducials;

  private static final PIDControllerConfigurable rotationalPidController = new PIDControllerConfigurable(0.05000, 0.000000, 0.001000, 0.1);
  private static final PIDControllerConfigurable xPidController = new PIDControllerConfigurable(0.400000, 0.000000, 0.000600, 0.2);
  private static final PIDControllerConfigurable yPidController = new PIDControllerConfigurable(0.3, 0, 0, 0.3);
  public double rotationalRate = 0;
  public double velocityX = 0;
  private double m_minDistance = 0.;

  public TurretAlignCommand(TurretSubsystem turret, LimelightSubsystem limelight, int tagId) {
    this.m_turret = turret;
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

      rotationalRate = rotationalPidController.calculate(2*fiducial.txnc, 0.0) * 0.1;
      
      if (rotationalPidController.atSetpoint()) {
        System.out.println("STOP alignment");
        this.end(true);
      }

      SmartDashboard.putNumber("AlignCommand/txnc", fiducial.txnc);
      SmartDashboard.putNumber("AlignCommand/ta", fiducial.ta);
      SmartDashboard.putNumber("AlignCommand/distToRobot", fiducial.distToRobot);
      SmartDashboard.putNumber("AlignCommand/distToCamera", fiducial.distToCamera);
      SmartDashboard.putNumber("AlignCommand/rotationalPidController", rotationalRate);
      SmartDashboard.putNumber("AlignCommand/xPidController", velocityX);
      SmartDashboard.putNumber("AlignCommand/TagID", m_tagId);


      /* move the robot to correct position */
      m_turret.setTurretPower(0.1);
      
    } catch (LimelightSubsystem.NoSuchTargetException nste) { 
      this.end(true);
    }
  }

  @Override
  public boolean isFinished() {
    boolean temp = rotationalPidController.atSetpoint();
    SmartDashboard.putBoolean("AlignCommand/AlignFinished", temp);
    return temp;
  }

  @Override
  public void end(boolean interrupted) {
    m_turret.setTurretPower(0);
    
  }

  public RawFiducial getClosestFiducial() {
    if (fiducials == null || fiducials.length == 0) {
        throw new NoSuchTargetException("No fiducials found.");
    }

    RawFiducial closest = fiducials[0];
    double minDistance = closest.ta;

    for (RawFiducial fiducial : fiducials) {
        if (fiducial.ta > minDistance) {
            closest = fiducial;
            minDistance = fiducial.ta;
        }
    }
    SmartDashboard.putNumber("minDistance", minDistance);

    /* persist closest distance value */
    setMinDistance(minDistance);

    return closest;
  }
  /* keep track of the minDistance found via linelight Apriltag search */
  public void setMinDistance(double distance) {
    m_minDistance = distance;
    SmartDashboard.putNumber("VisionClosetAprilTag", distance);
  }

  public double getMinDistance() {
    return m_minDistance;
  }

  public RawFiducial getFiducialWithId(int id) {
  
    for (RawFiducial fiducial : fiducials) {
        if (fiducial.id == id) {
            return fiducial;
        }
    }
    throw new NoSuchTargetException("Can't find ID: " + id);
  }

public RawFiducial getFiducialWithId(int id, boolean verbose) {
  StringBuilder availableIds = new StringBuilder();

  for (RawFiducial fiducial : fiducials) {
      if (availableIds.length() > 0) {
          availableIds.append(", ");
      } //Error reporting
      availableIds.append(fiducial.id);
      
      if (fiducial.id == id) {
          return fiducial;
      }
  }
  throw new NoSuchTargetException("Cannot find: " + id + ". IN view:: " + availableIds.toString());
  }
}