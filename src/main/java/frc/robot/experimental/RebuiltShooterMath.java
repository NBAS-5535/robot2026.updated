package frc.robot.experimental;

public class RebuiltShooterMath {

    // -----------------------------
    // FRC REBUILT GAME CONSTANTS
    // -----------------------------
    private static final double g = 9.81;          // gravity (m/s^2)
    private static final double SHOOTER_HEIGHT = 1.0;   // meters above carpet
    private static final double TARGET_HEIGHT = 2.64;    // TOWER opening height (m)

    /**
     * Predicts horizontal distance (meters) at which the projectile reaches
     * the TOWER height (2.64 m).
     *
     * @param v0       initial velocity (m/s)
     * @param angleDeg launch angle (degrees)
     * @return horizontal distance (meters), or -1 if unreachable
     */
    public static double predictTrajectory(double v0, double angleDeg) {

        double angleRad = Math.toRadians(angleDeg);

        double v0x = v0 * Math.cos(angleRad);
        double v0y = v0 * Math.sin(angleRad);

        double a = -0.5 * g;
        double b = v0y;
        double c = SHOOTER_HEIGHT - TARGET_HEIGHT;

        double discriminant = b*b - 4*a*c;
        if (discriminant < 0) return -1;

        double t1 = (-b + Math.sqrt(discriminant)) / (2*a);
        double t2 = (-b - Math.sqrt(discriminant)) / (2*a);

        double timeToTarget = Math.max(t1, t2);

        return v0x * timeToTarget;
    }
    /**
     * Computes the optimal angle (degrees) to hit the TOWER at a given distance.
     *
     * @param v0       initial velocity (m/s)
     * @param distance horizontal distance to TOWER (m)
     * @return optimal angle (degrees), or -1 if no valid angle exists
     */
    public static double optimalAngle(double v0, double distance) {

        double h = TARGET_HEIGHT - SHOOTER_HEIGHT;

        double A = (g * distance * distance) / (2 * v0 * v0);
        double B = -distance;
        double C = A + h;

        double discriminant = B*B - 4*A*C;
        if (discriminant < 0) return -1;

        double tan1 = (-B + Math.sqrt(discriminant)) / (2 * A);
        double tan2 = (-B - Math.sqrt(discriminant)) / (2 * A);

        double angle1 = Math.toDegrees(Math.atan(tan1));
        double angle2 = Math.toDegrees(Math.atan(tan2));

        return Math.min(angle1, angle2);
    }

    // Example usage
    public static void main(String[] args) {
        double v0 = 12.0;      // m/s shooter exit velocity
        double distance = 4.0; // meters from TOWER

        double angle = optimalAngle(v0, distance);
        System.out.println("Optimal angle: " + angle + " degrees");

        double predicted = predictTrajectory(v0, angle);
        System.out.println("Predicted distance: " + predicted + " meters");
    }
}

