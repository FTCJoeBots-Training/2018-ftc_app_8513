package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This is NOT an opmode. This is a hardware class used to abstract the hardware config for the
 * 2018 JoeBots FTC Rover Ruckus challenge. This file has been generalized to work as a base for
 * all three JoeBots FTC teams (8513, 11855, and 13702). As the season progresses, this file may be
 * customized for each individual team in their own branch.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 *
 * motor0 (left front)
 * motor1 (right front)
 * motor2 (left rear)
 * motor3 (right rear)
 * imu - navigation features
 *
 * Note:  All names are lower case and some have single spaces between words.
 *
 */

public class HardwareJoeBot2018
{
    /* Public OpMode members. */

    // Declare Motors
    public DcMotor  motor0 = null; // Left Front
    public DcMotor  motor1 = null; // Right Front
    public DcMotor  motor2 = null; // Left Rear
    public DcMotor  motor3 = null; // Right Rear
    public DcMotor  liftMotor = null; // Lander Lift Motor
    public DcMotor  shoulderMotor = null;
    public DcMotor  elbowMotor =  null;
    public DcMotor  intakeMotor = null;

    public Servo    mineralServo = null;
    public Servo    markerServo = null;

    // Declare Sensors
    public BNO055IMU imu;                  // The IMU sensor object

    // Variables used for IMU tracking...
    public Orientation angles;
    public Acceleration gravity;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    // Private Members
    private LinearOpMode myOpMode;
    private ElapsedTime runtime = new ElapsedTime();
    private Orientation lastImuAngles = new Orientation();
    private double globalAngle;

    // Declare Static members for calculations
    static final double COUNTS_PER_MOTOR_REV    = 1120;
    static final double DRIVE_GEAR_REDUCTION    = 1;
    static final double WHEEL_DIAMETER_INCHES   = 4.0;
    static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159);
    static final double INTAKE_MOTOR_POWER = 0.4;

    static final int LIFT_DOWN_POSITION = 0;
    static final int LIFT_UP_POSITION = 2500;
    static final double LIFT_POWER = 0.3;


    static final int ELBOW_STOW_POS = 0;
    static final int ELBOW_SEARCH_POS = 365;
    static final int ELBOW_SCORE_POS = 351;

    static final int SHOULDER_STOW_POS = 0;
    static final int SHOULDER_SEARCH_POS = -1220;
    static final int SHOULDER_SCORE_POS = -267;

    static final double ELBOW_STD_POWER = 0.4;
    static final double SHOULDER_STD_POWER = 0.5;

    static final double MARKER_OPEN_POS = 0.8;
    static final double MARKER_CLOSE_POS = 0.3;

    static final double MINERAL_OPEN_POS = 0.3;
    static final double MINERAL_CLOSE_POS = 0.8;

    private boolean bMineralDoorOpen = false;


    /* Constructor */
    public HardwareJoeBot2018(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, LinearOpMode opMode) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        myOpMode = opMode;

        // Define and Initialize Motors
        motor0 = hwMap.dcMotor.get("motor0");
        motor1 = hwMap.dcMotor.get("motor1");
        motor2 = hwMap.dcMotor.get("motor2");
        motor3 = hwMap.dcMotor.get("motor3");

        liftMotor       = hwMap.dcMotor.get("liftMotor");
        shoulderMotor   = hwMap.dcMotor.get("shoulderMotor");
        elbowMotor      = hwMap.dcMotor.get("elbowMotor");
        intakeMotor     = hwMap.dcMotor.get("intakeMotor");

        // Set Default Motor Directions for Drive Motors
        motor0.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motor1.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors
        motor2.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motor3.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors

        // Set Default Motor Direction for other Motors
        // For now, assume all directions run forward
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        shoulderMotor.setDirection(DcMotor.Direction.FORWARD);
        elbowMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        motor0.setPower(0);
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);

        liftMotor.setPower(0);
        shoulderMotor.setPower(0);
        elbowMotor.setPower(0);
        intakeMotor.setPower(0);

        // Map Servos
        mineralServo = hwMap.get(Servo.class, "mineralServo");
        markerServo = hwMap.get(Servo.class, "markerServo");

        // Set intial Servo Positions
        mineralServo.setPosition(MINERAL_OPEN_POS);
        bMineralDoorOpen = true;
        markerServo.setPosition(MARKER_CLOSE_POS);


        // Set all drive motors to run without encoders.
        // May want to switch to  RUN_USING_ENCODERS during autonomous
        motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // TESTING
        // Set liftMotor to RUN_TO_POSITION and tell it to run to zero

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.3);

        //


        // IMU Initializaiton
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);



    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }


    /***
     * void setMode(DcMotor.RunMode mode ) Set all drive motors to same mode.
     * @param mode    Desired Motor mode.
     */
    public void setMode(DcMotor.RunMode mode ) {
        motor0.setMode(mode);
        motor1.setMode(mode);
        motor2.setMode(mode);
        motor3.setMode(mode);
    }

    /**
     *
     * void moveRobot(double forward, double rigclockwise)
     *ht, double
     * Calculates power settings for Mecanum drive for JoeBots
     *
     * @param forward
     * @param right
     * @param clockwise
     *
     */
    public void moveRobot(double forward, double right, double clockwise) {

        // Declare Variables to hold calculated power values for each motor
        double power1;
        double power2;
        double power3;
        double power4;
        double max;

        power1 = forward + clockwise + right;
        power2 = forward - clockwise - right;
        power3 = forward + clockwise - right;
        power4 = forward - clockwise + right;

        // Normalize Wheel speeds so that no speed exceeds 1.0
        max = Math.abs(power1);
        if (Math.abs(power2) > max) {
            max = Math.abs(power2);
        }
        if (Math.abs(power3) > max) {
            max = Math.abs(power3);
        }
        if (Math.abs(power4) > max) {
            max = Math.abs(power4);
        }

        if (max > 1) {
            power1 /= max;
            power2 /= max;
            power3 /= max;
            power4 /= max;
        }

        motor0.setPower(power1);
        motor1.setPower(power2);
        motor2.setPower(power3);
        motor3.setPower(power4);

    }

    /**
     *
     * stop()
     *
     * method to set all motor powers to zero
     *
     */

    public void stop() {

        motor0.setPower(0);
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);

    }

    /**
     *
     * moveInches(double inches, double power)
     *
     * method to drive forward (only) for a set # of inches at a set power
     *
     * @param inches
     * @param power
     * @param timeoutSec
     *
     */

    public void moveInches(double inches, double power, int timeoutSec) {

        // method will accept a value in inches and a power setting and calculate the number of
        // rotations required to drive the given distance.

        // Tell Telemetry what we're starting
        myOpMode.telemetry.log().add("Starting moveInches method");

        // Declare needed variables
        int newMotor0Target;
        int newMotor1Target;
        int newMotor2Target;
        int newMotor3Target;

        // Check to make sure the OpMode is still active; If it isn't don't run the method
        if(myOpMode.opModeIsActive()) {

            // Determine new target positions for each wheel
            newMotor0Target = motor0.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newMotor1Target = motor1.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newMotor2Target = motor2.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newMotor3Target = motor3.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

            // Send target Positions to motors
            motor0.setTargetPosition(newMotor0Target);
            motor1.setTargetPosition(newMotor1Target);
            motor2.setTargetPosition(newMotor2Target);
            motor3.setTargetPosition(newMotor3Target);

            // Set Robot to RUN_TO_POSITION mode
            setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Reset the runtime
            runtime.reset();

            // Start moving the robot
            moveRobot(power,0,0);

            // Keep looping (wait) until the motors are finished or timeout is reached.
            while (myOpMode.opModeIsActive() && (runtime.seconds() < timeoutSec) &&
                    (motor0.isBusy() && motor1.isBusy() && motor2.isBusy() && motor3.isBusy())) {


                //Compose Telemetry message
                myOpMode.telemetry.addLine("> Waiting for robot to reach target");
                myOpMode.telemetry.addLine("Curr. Pos. |")
                        .addData("1:",motor0.getCurrentPosition())
                        .addData("2:",motor1.getCurrentPosition())
                        .addData("3:",motor2.getCurrentPosition())
                        .addData("4:",motor3.getCurrentPosition());
                myOpMode.telemetry.addLine("Target | ")
                        .addData("1:",newMotor0Target)
                        .addData("2:",newMotor1Target)
                        .addData("3:",newMotor2Target)
                        .addData("4:",newMotor3Target);
                myOpMode.telemetry.addData("Power: ", power);
                myOpMode.telemetry.update();

                myOpMode.idle();
            }

            // Stop the motors
            stop();

            // Update telemetry log
            myOpMode.telemetry.log().add("Ending moveInches method");

            // Set the motors back to standard mode
            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }

    }

    /**
     *
     * resetImuAngle()
     *
     * Method to grab the current reading from the IMU and set the cumulative angle tracking
     * to 0
     *
     */

    private void resetAngle(){

        // Grab reading from IMU and store it in lastImuAngles
        lastImuAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Set globalAngle to zero
        globalAngle = 0;

    }

    /**
     *
     * getAngle()
     *
     * Gets the current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees (+ left, - right)
     *
     */

    private double getAngle(){

        // Grab the current IMU Angle reading
        Orientation currAngles = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);

        // Determine the difference between the current Angle reading and the last reset
        double deltaAngle = currAngles.firstAngle - lastImuAngles.firstAngle;
        deltaAngle = -deltaAngle; // Fixing sign on deltaAngle
        // Since the Rev IMU measures in Euler angles (-180 <-> +180), we need to detect this
        if (deltaAngle < -180) deltaAngle += 360;
        else if (deltaAngle > 180) deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastImuAngles = currAngles;

        return globalAngle;

    }

    /**
     *
     * rotate(int degrees, double power)
     *
     * Does not support turning more than 180 degrees.
     *
     * @param degrees
     * @param power
     *
     *
     */

    public void rotate(int degrees, double power){

        myOpMode.telemetry.log().add("Starting rotate method");

        // Restart IMU movement tracking
        resetAngle();

        // getAngle returns + when rotating clockwise and - when rotating counter clockwise
        // set power (speed) negative when turning left
        if (degrees < 0 ) power = -power;

        // start robot turning
        moveRobot(0,0,power);

        // stop turning when getAngle() returns a value greater or less than intended degrees
        if (degrees > 0) {
            // Expect this to be a right turn
            // On a right turn, since we start on zero, we have to get off zero first

            while (myOpMode.opModeIsActive() && getAngle() == 0) {
                myOpMode.telemetry.addLine(">getAngle() returned 0");
                myOpMode.telemetry.addLine(">>")
                        .addData("Cur: ", getAngle())
                        .addData("Tar: ", degrees);
                myOpMode.telemetry.update();
            }

            while (myOpMode.opModeIsActive() && getAngle() < degrees) {
                myOpMode.telemetry.addLine(">getAngle() returned >0");
                myOpMode.telemetry.addLine(">>")
                        .addData("Cur: ", getAngle())
                        .addData("Tar: ", degrees);
                myOpMode.telemetry.update();
            }
        } else {
            // left turn

            while (myOpMode.opModeIsActive() && getAngle() > degrees) {
                myOpMode.telemetry.addLine(">getAngle() returned <0");
                myOpMode.telemetry.addLine(">>")
                        .addData("Cur: ", getAngle())
                        .addData("Tar: ", degrees);
                myOpMode.telemetry.update();
            }

        }


        //Stop the motors
        stop();

        // reset IMU tracking
        resetAngle();



    }

    public void toggleIntake(String strIntakeDirection) {

        //This method should determine the current status of the intake motor. If the motor is
        // stopped, it should be started in the appropriate direction at the appropriate power.
        // if the motor is currently running in the same direction as supplied to the method,
        // then the motor should stop. Otherwise, we should switch the motor to the direction
        // supplied.

        if (intakeMotor.getPower() != 0) {
            // Motor must be running... Is it running in the correct direction?
            if (intakeMotor.getPower() < 0) {
                // Motor is running in reverse
                if (strIntakeDirection.equals("reverse")) {
                    // Motor is running and running the correct direction. Stop the motor.
                    intakeMotor.setPower(0);
                } else if (strIntakeDirection.equals("forward")) {
                    // Motor is running and running in the opposite direction. Invert the power.
                    intakeMotor.setPower(-intakeMotor.getPower());
                }
            } else {
                // intake motor is running forward
                if (strIntakeDirection.equals("forward")) {
                    //Motor is running in the correct direction. Stop the motor.
                    intakeMotor.setPower(0);
                } else if (strIntakeDirection.equals("reverse")) {
                    //Motor is running in the wrong direction
                    intakeMotor.setPower(-intakeMotor.getPower());
                }
            }
        } else {
            // Intake motor is currently stopped. Start it int he appropriate direction.
            if (strIntakeDirection.equals("forward")) {
                intakeMotor.setPower(INTAKE_MOTOR_POWER);
            } else if (strIntakeDirection.equals("reverse")) {
                intakeMotor.setPower(-INTAKE_MOTOR_POWER);
            }

        }

    }

    public void lowerLift() {
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setTargetPosition(LIFT_DOWN_POSITION);
        liftMotor.setPower(LIFT_POWER);
        while (myOpMode.opModeIsActive() && liftMotor.isBusy()){
            myOpMode.telemetry.addLine("Lowering Lift");
            myOpMode.telemetry.addData("Target Position: ", LIFT_DOWN_POSITION);
            myOpMode.telemetry.addData("Current Position: ", liftMotor.getCurrentPosition());
            myOpMode.telemetry.update();
            myOpMode.idle();
        }
    }

    public void raiseLift(){
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setTargetPosition(LIFT_UP_POSITION);
        liftMotor.setPower(LIFT_POWER);
        while (myOpMode.opModeIsActive() && liftMotor.isBusy()){
            myOpMode.telemetry.addLine("Raising Lift");
            myOpMode.telemetry.addData("Target Position: ", LIFT_UP_POSITION);
            myOpMode.telemetry.addData("Current Position: ", liftMotor.getCurrentPosition());
            myOpMode.telemetry.update();
            myOpMode.idle();
        }
    }

    public void stowArm() {
        // Move the arm to stowing position. In initial testing we determined that we should begin
        // to fold the elbow before we move the shoulder. This will (hopefully) prevent the shoulder
        // from throwing the elbow out of position.

        // Move the elbow first
        elbowMotor.setTargetPosition(ELBOW_STOW_POS);
        elbowMotor.setPower(ELBOW_STD_POWER);

        // sleep for 1/4 second to let the elbow get started.
        myOpMode.sleep(250);

        // now nove the shoulder
        shoulderMotor.setTargetPosition(SHOULDER_STOW_POS);
        shoulderMotor.setPower(SHOULDER_STD_POWER);

        // In "stow" position, we don't need to hold the arm in place, so set motor power to zero
        // after the shoulder is in place.
        while (myOpMode.opModeIsActive() && shoulderMotor.isBusy()) {
            myOpMode.idle();
        }
        shoulderMotor.setPower(0);

    }

    public void scoreArm() {
        // Move the arm to Scoring position. Initial testing has shown that the arm carries too
        // much momentum when moving to scoring position, and can thow it too far back. To control
        // this, we're going to try to fold a the elbow first, reducing the force at the end of the
        // arm.

        // fold the elbow in
        elbowMotor.setTargetPosition(ELBOW_STOW_POS);
        elbowMotor.setPower(ELBOW_STD_POWER);

        // Move the shoulder to scoring position
        shoulderMotor.setTargetPosition(SHOULDER_SCORE_POS);
        shoulderMotor.setPower(SHOULDER_STD_POWER);

        // Wait for the shoulder motor move to complete
        while (myOpMode.opModeIsActive() && shoulderMotor.isBusy()) {
            myOpMode.idle();
        }

        // move the elbow to scoring position
        elbowMotor.setTargetPosition(ELBOW_SCORE_POS);
        elbowMotor.setPower(ELBOW_STD_POWER);

    }

    public void searchArm() {
        // Move the arm to Searching position.

        // Start the elbow moving forward
        elbowMotor.setTargetPosition(ELBOW_SEARCH_POS);
        elbowMotor.setPower(ELBOW_STD_POWER);

        // Start the shoulder moving forward
        shoulderMotor.setTargetPosition(SHOULDER_SEARCH_POS);
        shoulderMotor.setPower(SHOULDER_STD_POWER);
    }

    public void toggleMineralDoor() {

        if(bMineralDoorOpen) {
            //Mineral door is open; close it.
            mineralServo.setPosition(MINERAL_CLOSE_POS);
            bMineralDoorOpen = false;
        } else {
            // Mineral Door is closed. Open it.
            mineralServo.setPosition(MINERAL_OPEN_POS);
            bMineralDoorOpen = true;
        }

    }

    public void dropMarker() {
        // Open and Close Servo
        markerServo.setPosition(MARKER_OPEN_POS);
        myOpMode.sleep(1000);
        markerServo.setPosition(MARKER_CLOSE_POS);

    }


}