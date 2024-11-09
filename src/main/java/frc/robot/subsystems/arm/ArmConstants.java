package frc.robot.subsystems.arm;

public class ArmConstants {

    public static final ArmGains SimGains = new ArmGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    public static final ArmGains TalonFXGains = new ArmGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    
    record ArmGains(double kP, double kI, double kD, double kS, double kG, double kV, double kA) {
        public ArmGains {
            if (kP < 0 || kI < 0 || kD < 0 || kS < 0 || kV < 0 || kA < 0 || kG < 0) {
                throw new IllegalArgumentException("Gains must be non-negative");
            }
        }
    }
}
