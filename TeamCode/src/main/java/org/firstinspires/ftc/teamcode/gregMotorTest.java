/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Sample code to test mapping of one motor to the gamepad.
 */
@TeleOp(name = "READ Encoder Values", group = "Testing")
//@Disabled
public class gregMotorTest extends LinearOpMode {


    // Define class members
    DcMotor  liftMotor = null; // Lander Lift Motor
    DcMotor  shoulderMotor = null;
    DcMotor  elbowMotor =  null;
    DcMotor  intakeMotor = null;

    Servo    markerServo = null;
    Servo    mineralServo = null;


    @Override
    public void runOpMode() {

        liftMotor       = hardwareMap.dcMotor.get("liftMotor");
        shoulderMotor   = hardwareMap.dcMotor.get("shoulderMotor");
        elbowMotor      = hardwareMap.dcMotor.get("elbowMotor");
        intakeMotor     = hardwareMap.dcMotor.get("intakeMotor");

        markerServo     = hardwareMap.servo.get("markerServo");
        mineralServo    = hardwareMap.servo.get("mineralServo");

        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        shoulderMotor.setDirection(DcMotor.Direction.FORWARD);
        elbowMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        liftMotor.setPower(0);
        shoulderMotor.setPower(0);
        elbowMotor.setPower(0);
        intakeMotor.setPower(0);




        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors." );
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {

            // Display the current value
            telemetry.addData("liftMotor Pos: ", liftMotor.getCurrentPosition());
            telemetry.addData("shoulderMotor Pos: ", shoulderMotor.getCurrentPosition());
            telemetry.addData("elbowMotor Pos: ", elbowMotor.getCurrentPosition());
            telemetry.addData("Marker Servo: ", markerServo.getPosition());
            telemetry.addData("Mineral Servo: ", mineralServo.getPosition());
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            idle();
        }

        // Turn off motor and signal done;
        telemetry.addData(">", "Done");
        telemetry.update();

    }
}
















