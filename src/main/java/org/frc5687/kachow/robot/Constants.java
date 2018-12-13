package org.frc5687.kachow.robot;

/**
 * Created by Ben Bernard on 10/23/2018.
 */
public class Constants {
    public static final int CYCLES_PER_SECOND = 50;

    public static class DriveTrain {
        public static final double DEADBAND = 0.01;
        public static final double SENSITIVITY_LOW_GEAR = 0.1;
        public static final double SENSITIVITY_HIGH_GEAR = 0.3;
        public static final double ROTATION_SENSITIVITY = 0.3;

        public static final double HIGH_POW = 1.0;
        public static final double LOW_POW = -HIGH_POW;

        public static final boolean LEFT_MOTORS_INVERTED = false;
        public static final boolean RIGHT_MOTORS_INVERTED = true;

        public static class Encoders {

            public static final boolean REVERSED = true; //TODO change to new robot specifications
            public static final int SAMPLES_TO_AVERAGE = 20;
            public static final int PULSES_PER_ROTATION = 4096; // 1024 in quad mode. talon is 4096.

            public class WheelDiameter {
                public static final double INCHES = 6; //TODO: confirm wheel diameters!
                public static final double METERS = 0.1524; // TODO: Confirm!
            }
            public static final double GEAR_RATIO = 3.0; // TODO: Confirm!
            public class DistancePerRotation {
                public static final double INCHES = Math.PI * WheelDiameter.INCHES * GEAR_RATIO;
                public static final double METERS = Math.PI * WheelDiameter.METERS;
            }

            public class DistancePerPulse {
                public static final double INCHES = DistancePerRotation.INCHES / PULSES_PER_ROTATION;
            }

            public static final double SCALAR_RATIO = 8; // TODO: Confirm!
            public static final double MAX_PERIOD = 5;

            public class Track {
                public static final double INCHES = 24; // TODO: Confirm!
                public static final double METERS = 0.6096; // TODO: Confirm!
            }

            public static final double INCHES_PER_PULSE = DistancePerPulse.INCHES;

        }
    }
    public static class Arm {
        public static final double DEADBAND = 0.1;
        public static final double SPEED_MAX = 1.0;
        public static final double SENSITIVITY = 0.75;

        public static final double FRONT_FLAT = 90.0;
        public static final double FRONT_SWITCH = 30.0;
        public static final double UP = 0;
        public static final double BACK_SWITCH = -30.0;
        public static final double BACK_FLAT = -90.0;

        //Currently using numbers from Gizmo, will be changed.
        public static final double ANGLE_MIN = -110;
        public static final double ANGLE_MAX = 110;
        public static final double POT_MIN = 0.065;
        public static final double POT_MAX = 0.695;
        public static final boolean MOTOR_INVERTED = false;
    }
    public class Shifter {
        public static final long STOP_MOTOR_TIME = 60;
        public static final long SHIFT_TIME = 60;

        public static final double SHIFT_UP_THRESHOLD = 50; // in inches per second TODO tune
        public static final double SHIFT_DOWN_THRESHOLD = 40; //in inches per second TODO tune

        public static final long AUTO_WAIT_PERIOD = 500;
        public static final long MANUAL_WAIT_PERIOD = 3000;
    }
}
