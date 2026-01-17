package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Vision.LimelightHelpers;
import frc.robot.Vision.LimelightHelpers.RawFiducial;
import frc.robot.Vision.LimelightHelpers.LimelightResults;
import frc.robot.Vision.LimelightHelpers.LimelightTarget_Fiducial;


public class VisionSubsystem extends SubsystemBase {
  private RawFiducial[] fiducials;
  private LimelightResults limelightResults;
  private final String limelightName = "";
  private double m_minDistance = 0.;
  
  //public final AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(jsonPath);
  //private LimelightDistanceEstimator estimator = new LimelightDistanceEstimator();

// Define constants for your robot and target (measure these physically)
  private static final double LIMELIGHT_MOUNT_ANGLE = 19.; //7.0; // Degrees
  private static final double TARGET_HEIGHT = 0.85; // Meters or inches, consistently
  private static final double LIMELIGHT_HEIGHT = 0.24; // Meters or inches, consistently

  public VisionSubsystem() {
    config();
  }

  public static class NoSuchTargetException extends RuntimeException {
    public NoSuchTargetException(String message) {
      super(message);
    }
  }

  public void config() {

    // LimelightHelpers.setCropWindow("", -0.5, 0.5, -0.5, 0.5);
    LimelightHelpers.setCameraPose_RobotSpace(
      this.limelightName,
        Meters.convertFrom(16.5, Inches), // forward location wrt robot center: half side + camera thickness
        0., // assume perfect alignment with robor center
        LIMELIGHT_HEIGHT, //m Meters.convertFrom(VisionConstants.limelightLensHeightInches, Inches), // height of camera from the floor
        LIMELIGHT_MOUNT_ANGLE,
        0,
        0);
    // Overrides the valid AprilTag IDs that will be used for localization. 
    // Tags not in this list will be ignored for robot pose estimation.
    /*
    Optional<Alliance> ally = DriverStation.getAlliance();
    int[] tagsToConsider = new int[] {};
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
          tagsToConsider = new int[] {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
        }
        if (ally.get() == Alliance.Blue) {
          tagsToConsider = new int[] {12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
        }
    }
    else {
      System.out.println("No Alliance info is available");
    }
    */

    int [] tagsToConsider = new int[] {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
    LimelightHelpers.SetFiducialIDFiltersOverride("", tagsToConsider);
  }

  @Override
  public void periodic() {
    fiducials = LimelightHelpers.getRawFiducials("");
    this.limelightResults = LimelightHelpers.getLatestResults(this.limelightName);
    SmartDashboard.putNumber("/VisionSubsystem/num" + this.limelightName, this.limelightResults.targets_Fiducials.length);

    /*
    for (RawFiducial fiducial : fiducials) {
      int id = fiducial.id; // Tag ID
      double txnc = fiducial.txnc; // X offset (no crosshair)
      double tync = fiducial.tync; // Y offset (no crosshair)
      double ta = fiducial.ta; // Target area
      double distToCamera = fiducial.distToCamera; // Distance to camera
      double distToRobot = fiducial.distToRobot; // Distance to robot
      double ambiguity = fiducial.ambiguity; // Tag pose ambiguity
      SmartDashboard.putNumber("/DebugVision/id", id);
      SmartDashboard.putNumber("/DebugVision/txnc", txnc);
      SmartDashboard.putNumber("/DebugVision/tync", tync);
      
      SmartDashboard.putNumber("/DebugVision/ty", tync);
      SmartDashboard.putNumber("/DebugVision/ta", ta);
      SmartDashboard.putNumber("/DebugVision/distToCamera", distToCamera);
      SmartDashboard.putNumber("/DebugVision/distToRobot", distToRobot);
      SmartDashboard.putNumber("/DebugVision/ambiguity", ambiguity);
    }
    */
    SmartDashboard.putNumber("/DebugVision/tx", LimelightHelpers.getTX(""));
    SmartDashboard.putNumber("/DebugVision/ty", LimelightHelpers.getTY(""));
    SmartDashboard.putNumber("/DebugVision/txnc", LimelightHelpers.getTXNC(""));
    SmartDashboard.putNumber("/DebugVision/tync", LimelightHelpers.getTYNC(""));
    SmartDashboard.putNumber("/DebugVision/ta", LimelightHelpers.getTA(""));

    getDistance();

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
    SmartDashboard.putNumber("/VisionSubsystem/minDistance", minDistance);

    /* persist closest distance value */
    setMinDistance(minDistance);

    return closest;
  }

  public RawFiducial getFiducialWithId(int id) {
  
    for (RawFiducial fiducial : fiducials) {
        //System.out.println("Look up " + Integer.toString(fiducial.id));
        if (fiducial.id == id) {
          //System.out.println("Found " + Integer.toString(fiducial.id));
            return fiducial;
        } //else {
          //System.out.println("Nothing yet " + Integer.toString(fiducial.id));
          //throw new NoSuchTargetException("Can't find ID: " + id);
        //}
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

  public LimelightTarget_Fiducial getTargetFiducialWithId(int id) {
    for (LimelightTarget_Fiducial fiducial : limelightResults.targets_Fiducials) {
      if (fiducial.fiducialID != (double) id) {
        continue;
      }

      return fiducial;
    }

    throw new NoSuchTargetException("No target with ID " + id + " is in view!");
  }
  
  /* keep track of the minDistance found via limelight Apriltag search */
  public void setMinDistance(double distance) {
    m_minDistance = distance;
    SmartDashboard.putNumber("/VisionSubsystem/VisionClosetAprilTag", distance);
  }

  public double getMinDistance() {
    return m_minDistance;
  }

  /* calculate the distance in meters */
  public double getDistanceToTargetInMeters(double distance) {
    /* use/update estimation formula via resgression*/
    double distInMeters = -70.08 * distance + 2.35; 
    System.out.print(distance + " - " + distInMeters);
    return distInMeters;
  }

  /* utility functions */
  public double getTX(){
    return LimelightHelpers.getTX("");
  }
  public double getTY(){
    return LimelightHelpers.getTY("");
  }
  public double getTA(){
    return LimelightHelpers.getTA("");
  }
  public boolean getTV(){
    return LimelightHelpers.getTV("");
  }

  public double getClosestTX(){
    return getClosestFiducial().txnc;
  }
  public double getClosestTY(){
    return getClosestFiducial().tync;
  }
  public double getClosestTA(){
    return getClosestFiducial().ta;
  }

  /* set Limelight preset pipeline */
  public static void setPipeline(int index){
    LimelightHelpers.setPipelineIndex("", index);
    SmartDashboard.putNumber("/VisionSubsystem/PipeLineInUse", index);
  }

  public double getDistance() {
        // Get the vertical offset angle (ty) from the Limelight
        double ty = LimelightHelpers.getTY("limelight"); // "limelight" is the default name, change if yours is different

        // Calculate the angle to the goal using the camera mount angle and ty
        Rotation2d angleToGoal = Rotation2d.fromDegrees(LIMELIGHT_MOUNT_ANGLE)
            .plus(Rotation2d.fromDegrees(ty));

        // Use trigonometry to calculate the distance: distance = (targetHeight - limelightHeight) / tan(angleToGoal)
        double distance = (TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(angleToGoal.getRadians());

        //SmartDashboard.putNumber("/VisionSubsystem/EstimatedDistance", distance);
        SmartDashboard.putNumber("/VisionSubsystem/ty_estimator", distance);

        return distance;
    } 
}