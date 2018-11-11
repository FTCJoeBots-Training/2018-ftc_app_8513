package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 *import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 *import com.qualcomm.robotcore.hardware.DcMotor;
 *
 *
 */

/**
 *
 *Notes For this TeleOp Code. This code is for Comp and all proggramers should review over this
 *code and understand this code for the possibility that a question may be asked related to TeleOp and
 *you should be able to explain in good detail everything in this code.
 *11/16/17-> Changed all gamepad's in code to correct gamepad (i.e some gamepad1's to gamepad2)
 ***11/18/17-> Competition Notes below
 *Notes-> Autonomous is incorrect, Not much was wrong from a software sandpoint but hardware issues were fixed
 *Autonomous issues included: Incorrect spinning causing us to move out of destination,
 *To much time on the down motion of the clamp and arm.
 *These issues are still not resolved
 * Recomendation for autonomous issues(Not Offical):Fine tune the timer on the clamp
 * Fine tune the movements and LOWER the TIME OF MOVEMENT in autonomous.
 * List of issues at Comp(1)-> https://docs.google.com/a/stjoebears.com/spreadsheets/d/1r_liipKBU7GHfONdxq9E6d4f7zikcCuXwDL2bsQfwm0/edit?usp=sharing
 *G-Sheet of time VS Heading for autonomous -> https://docs.google.com/a/stjoebears.com/spreadsheets/d/1pqv0iN94fFd5KvX1YIWP7z39HgpURXsscn0zPujs1q4/edit?usp=sharing
*/
@TeleOp(name="Simple Mecanum Drive 2", group="TeleOp")

public class teleOpSimpleMecanum2018 extends LinearOpMode {

    HardwareJoeBot2018 robot = new HardwareJoeBot2018();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, this);


        double forward;
        double clockwise;
        double right;
        double shoulderPower;
        double elbowPower;
        double maxShoulderPower = 0.5;
        double maxElbowPower = 0.5;

        //boolean variables for ButtonStates
        boolean bCurrStateLB = false;
        boolean bPrevStateLB = false;
        boolean bCurrStateRB = false;
        boolean bPrevStateRB = false;
        boolean bCurrStateA = false;
        boolean bPrevStateA = false;
        boolean bCurrStateB = false;
        boolean bPrevStateB = false;
        boolean bCurrStateX = false;
        boolean bPrevStateX = false;
        boolean bCurrStateY = false;
        boolean bPrevStateY = false;
        boolean bCurrStateDPD = false;
        boolean bPrevStateDPD = false;
        boolean bCurrStateDPU = false;
        boolean bPrevStateDPU = false;


        waitForStart();

        //robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //start of loop
        while (opModeIsActive()) {


            //Drive Via "Analog Sticks" (Not Toggle)
            //Set initial motion parameters to Gamepad1 Inputs
            forward = -gamepad1.left_stick_y;
            //right = gamepad1.left_stick_x;
            right = -gamepad1.left_trigger + gamepad1.right_trigger;
            clockwise = gamepad1.right_stick_x;

            robot.moveRobot(forward, right, clockwise);


            bCurrStateLB = gamepad2.left_bumper;
            if ((bCurrStateLB == true) && (bCurrStateLB != bPrevStateLB)) {
                // Left bumper has been pressed. We should set intake to reverse
                // also, if Intake is currently running, and running in reverse, we should stop it

                robot.toggleIntake("reverse");

            }
            bPrevStateLB = bCurrStateLB;

            bCurrStateRB = gamepad2.right_bumper;
            if ((bCurrStateRB == true) && (bCurrStateRB != bPrevStateRB)) {
                // Left bumper has been pressed. We should set intake to reverse
                // also, if Intake is currently running, and running in reverse, we should stop it

                robot.toggleIntake("forward");

            }
            bPrevStateRB = bCurrStateRB;

            bCurrStateDPD = gamepad2.dpad_down;
            if ((bCurrStateDPD == true) && (bCurrStateDPD != bPrevStateDPD)) {
                // Left bumper has been pressed. We should set intake to reverse
                // also, if Intake is currently running, and running in reverse, we should stop it

                robot.lowerLift();

            }
            bPrevStateDPD = bCurrStateDPD;

            bCurrStateDPU = gamepad2.dpad_up;
            if ((bCurrStateDPU == true) && (bCurrStateDPU != bPrevStateDPU)) {
                // Left bumper has been pressed. We should set intake to reverse
                // also, if Intake is currently running, and running in reverse, we should stop it

                robot.raiseLift();

            }
            bPrevStateDPU = bCurrStateDPU;

            while (opModeIsActive() && gamepad2.dpad_left) {
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.liftMotor.setPower(0.3);
            }
            robot.liftMotor.setPower(0);
            while (opModeIsActive() && gamepad2.dpad_right) {
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.liftMotor.setPower(-0.3);
            }
            robot.liftMotor.setPower(0);
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            bCurrStateA = gamepad2.a;
            if ((bCurrStateA == true) && (bCurrStateA != bPrevStateA)) {

                // When the "A" button is pressed, we want to enable search mode

                robot.searchArm();

            }
            bPrevStateA = bCurrStateA;

            bCurrStateB = gamepad2.b;
            if ((bCurrStateB == true) && (bCurrStateB != bPrevStateB)) {

                // When the "B" button is pressed, we want to enable Scoring mode

                robot.scoreArm();

            }
            bPrevStateB = bCurrStateB;

            bCurrStateY = gamepad2.y;
            if ((bCurrStateY == true) && (bCurrStateY != bPrevStateY)) {

                // When the "B" button is pressed, we want to enable Scoring mode

                robot.stowArm();

            }
            bPrevStateY = bCurrStateY;

            // X Button should open/close Mineral Door
            bCurrStateX = gamepad2.x;
            if ((bCurrStateX) && (bCurrStateX != bPrevStateX)) {
                robot.toggleMineralDoor();
            }
            bPrevStateX = bCurrStateX;




            // Update Telemetry
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.addData("Lift Motor Position: ", robot.liftMotor.getCurrentPosition());
            telemetry.update();
            idle();


        }//end while
    }
}