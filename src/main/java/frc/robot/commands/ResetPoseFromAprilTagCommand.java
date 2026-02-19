package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DynamicTurretSubsystem;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.Vision.LimelightHelpers.RawFiducial;

public class ResetPoseFromAprilTagCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final DynamicTurretSubsystem turret;

    private final double kSetpoint = 0.0; // want tx = 0
    private final double kTolerance = 1.0; // degrees

    private final PIDControllerConfigurable rotationPID =
        new PIDControllerConfigurable(0.1, 0.0, 0.0, kSetpoint, kTolerance);

    private double rotationalRate = 0.0;
    private double forwardSpeed = 0.1;

    private static final SwerveRequest.RobotCentric alignRequest =
        new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    RawFiducial[] fiducials;

    public ResetPoseFromAprilTagCommand(CommandSwerveDrivetrain drivetrain, DynamicTurretSubsystem turret) {
        this.drivetrain = drivetrain;
        this.turret = turret;
        addRequirements(drivetrain); // DO NOT require turret
    }

    @Override
    public void initialize() {
        LimelightHelpers.setLEDMode_PipelineControl("limelight");

        // Enable turret tracking toward the tower
        turret.enableTracking();

        // Initial fiducial read
        fiducials = LimelightHelpers.getRawFiducials("limelight");

        SmartDashboard.putNumber("AutoAlign/TagSeenInit", LimelightHelpers.getFiducialID("limelight"));
    }

    @Override
    public void execute() {

        // Refresh fiducials every cycle
        fiducials = LimelightHelpers.getRawFiducials("limelight");

        // If no target or no fiducials, stop safely
        if (!LimelightHelpers.getTV("limelight") || fiducials == null || fiducials.length == 0) {
            SmartDashboard.putBoolean("AutoAlign/HasTarget", false);
            drivetrain.setControl(idleRequest);
            return;
        }

        SmartDashboard.putBoolean("AutoAlign/HasTarget", true);

        // Pick closest tag safely
        RawFiducial closest = fiducials[0];
        for (RawFiducial f : fiducials) {
            if (f.distToRobot < closest.distToRobot) {
                closest = f;
            }
        }

        SmartDashboard.putNumber("AutoAlign/ClosestTagID", closest.id);
        SmartDashboard.putNumber("AutoAlign/ClosestTagDist", closest.distToRobot);

        // Robot rotation using tx
        double tx = LimelightHelpers.getTX("limelight");
        SmartDashboard.putNumber("AutoAlign/tx", tx);

        rotationalRate = rotationPID.calculate(tx, kSetpoint);

        SmartDashboard.putNumber("AutoAlign/rotRate", rotationalRate);

        drivetrain.setControl(
            alignRequest
                .withRotationalRate(rotationalRate)
                .withVelocityX(forwardSpeed)
        );

        // ⭐ Turret is NOT controlled here — it updates itself in periodic()
        // turret.setPointAtTargetSetpointValue() runs automatically when trackingtarget = true
    }

    @Override
    public boolean isFinished() {
        boolean done = rotationPID.atSetpoint();
        SmartDashboard.putBoolean("AutoAlign/Finished", done);
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.applyRequest(() -> idleRequest);
    }
}
