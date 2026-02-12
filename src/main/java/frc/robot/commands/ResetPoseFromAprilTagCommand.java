package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.subsystems.DynamicTurretSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ResetPoseFromAprilTagCommand extends Command {

    private final DynamicTurretSubsystem turret;
    private final CommandSwerveDrivetrain drivetrain;

    // Only reset pose from these tags
    private static final int[] ALLOWED_TAGS = {1, 2, 3, 4, 5, 6, 7, 8,10};

    public ResetPoseFromAprilTagCommand(DynamicTurretSubsystem turret, CommandSwerveDrivetrain drivetrain) {
        this.turret = turret;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    private boolean isAllowedTag(int tid) {
        for (int id : ALLOWED_TAGS) {
            if (id == tid) return true;
        }
        return false;
    }

    @Override
    public void initialize() {

        var table = NetworkTableInstance.getDefault().getTable("limelight");

        double tv = table.getEntry("tv").getDouble(0);

        if (tv == 1) {

            // Read tag ID
            int tid = (int) table.getEntry("tid").getDouble(-1);

            // Read distance to tag (meters)
            double[] targetpose_robotspace =
                table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);

            double distance = targetpose_robotspace.length >= 6 ? targetpose_robotspace[2] : 999;

            // Reject tags not in allowed list
            if (!isAllowedTag(tid)) return;

            // Reject tags too far away (optional)
            if (distance > 4.0) return; // only trust tags within 4 meters

            boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;

            double[] botpose = table
                .getEntry(isBlue ? "botpose_wpiblue" : "botpose_wpired")
                .getDoubleArray(new double[6]);

            if (botpose.length >= 6) {

                Pose2d llPose = new Pose2d(
                    botpose[0],
                    botpose[1],
                    Rotation2d.fromDegrees(botpose[5])
                );

                // Keep gyro heading
                Pose2d corrected = new Pose2d(
                    llPose.getX(),
                    llPose.getY(),
                    drivetrain.getCurrentPose().getRotation()
                );

                drivetrain.resetPose(corrected);

                // Enable turret tracking after pose reset
                turret.enableTracking();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
