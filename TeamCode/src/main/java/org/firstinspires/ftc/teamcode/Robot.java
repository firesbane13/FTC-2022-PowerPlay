package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

public class Robot extends LinearOpMode {
    // DcMotors: These are the code representation of the motors
    private DcMotor frontLeftWheel = null;
    private DcMotor backLeftWheel = null;
    private DcMotor frontRightWheel = null;
    private DcMotor backRightWheel = null;

    private ArrayList<Object> mecanumParameters = new ArrayList<Object>();

    /*
     * IMU: This contains 3 different sensors in FTC on the Control Hub/Expansian Hub
     *
     * - Accelerameter: Used to determine the acceleration of the robot
     * - Gyroscope: Used to determine the robot's orientation from it's start position (0 deg is
     *              the initial start position when the robot is first turned on)
     * - Magnetometer: Used to measure magnetic forces, especially earth's magnetism.
     */
    BNO055IMU imu;

    // @Override is used to override the existing runOpMode() function in LinearOpMode
    @Override
    public void runOpMode() {
        /*
         * Used to connect the hardware wheel confirmation to the software wheel variables.
         *
         * hardwareMap is a variable defined in LinearOpMode which is why it can be used without
         * defining it in this class.    It is used to retrieve the configuration for the
         * requested value from the Control Hub/Expansion Hub.
         *
         * This code was written with the following ports in mind for each wheel
         * 0 = frontLeftWheel
         * 1 = backLeftWheel
         * 2 = frontRightWheel
         * 3 = backRightWheel
         */
        frontLeftWheel = hardwareMap.dcMotor.get("frontLeftWheel");
        backLeftWheel = hardwareMap.dcMotor.get("backLeftWheel");
        frontRightWheel = hardwareMap.dcMotor.get("frontRightWheel");
        backRightWheel = hardwareMap.dcMotor.get("backRightWheel");

        /*
         * IMU: This is retrieving the IC2 configuration from the Control Hub/Expansion Hub
         * for the three sensors (accelerometer, gyroscope, and magnetometer).
         */
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        /*
         * IMU: Used to configure Gyroscope and Accelerometer
         */
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // IMU: Initialize the IMU sensors with the desired settings.
        imu.initialize(parameters);

        // Waits until the user presses Play on the Driver Station.
        waitForStart();

        // While the TeleOp or other op mode is running.
        while (opModeIsActive()) {

        }
    }
}
