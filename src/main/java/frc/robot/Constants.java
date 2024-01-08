package frc.robot;

public class Constants {
    
    public static class FlyWheelConstants {
        //
        // Hardaware
        //
        public static final int kLeaderID = 10;
        public static final int KFollowerID = 10;

        //
        // PID Constants
        //
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0; // Start out with 12 Volts/(Max speed RPM). 

        //
        // LQR State Space
        //

        // Volts per (radian per second)
        public static final double kFlywheelKv = 0.023;

        // Volts per (radian per second squared)
        public static final double kFlywheelKa = 0.001;
    }
}
