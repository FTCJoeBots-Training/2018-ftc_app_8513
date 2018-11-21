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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Sample code to test mapping of one motor to the gamepad.
 */
@TeleOp(name = "READ Servo Values", group = "Testing")
//@Disabled
public class gregServoTest extends LinearOpMode {

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position


    Servo    markerServo = null;
    Servo    mineralServo = null;
    double   markerPos = 0.5;
    double   mineralPos = 0.5;

    boolean bCurrStateA = false;
    boolean bPrevStateA = false;
    boolean bCurrStateB = false;
    boolean bPrevStateB = false;

    boolean bMarkerRampUp = false;
    boolean bMineralRampUp = false;
    boolean bMarkerMove = false;
    boolean bMineralMove = false;


    @Override
    public void runOpMode() {

        markerServo     = hardwareMap.get(Servo.class, "markerServo");
        mineralServo    = hardwareMap.get(Servo.class, "mineralServo");



        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors." );
        telemetry.update();
        waitForStart();




        while(opModeIsActive()) {

            // Read button Data

            bCurrStateA = gamepad1.a;
            if (bCurrStateA && (bCurrStateA!=bPrevStateA)) {
                bMineralMove = !bMineralMove;
            }
            bPrevStateA = bCurrStateA;

            bCurrStateB = gamepad1.b;
            if (bCurrStateB && (bCurrStateB!=bPrevStateB)) {
                bMarkerMove = !bMarkerMove;
            }
            bPrevStateB = bCurrStateB;


            // Set Servo Positions

            if (bMarkerMove) {
                // We should be moving the marker
                if (bMarkerRampUp) {
                    // Keep stepping up until we hit the max value.
                    markerPos += INCREMENT;
                    if (markerPos >= MAX_POS) {
                        markerPos = MAX_POS;
                        bMarkerRampUp = !bMarkerRampUp;   // Switch ramp direction
                    }
                } else {
                    markerPos -= INCREMENT;
                    if (markerPos <= MIN_POS ) {
                        markerPos = MIN_POS;
                        bMarkerRampUp = !bMarkerRampUp;  // Switch ramp direction
                    }
                }
            }
            if (bMineralMove) {
                // We should be moving the marker
                if (bMineralRampUp) {
                    // Keep stepping up until we hit the max value.
                    mineralPos += INCREMENT;
                    if (mineralPos >= MAX_POS) {
                        mineralPos = MAX_POS;
                        bMineralRampUp = !bMineralRampUp;   // Switch ramp direction
                    }
                } else {
                    mineralPos -= INCREMENT;
                    if (mineralPos <= MIN_POS ) {
                        mineralPos = MIN_POS;
                        bMineralRampUp = !bMineralRampUp;  // Switch ramp direction
                    }
                }
            }


            // Display the current value
            telemetry.addData("Marker Position", "%5.2f", markerPos);
            telemetry.addData("Mineral Position", "%5.2f", mineralPos);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the servo to the new position and pause;
            mineralServo.setPosition(mineralPos);
            markerServo.setPosition(markerPos);
            sleep(CYCLE_MS);
            idle();


        }

        // Turn off motor and signal done;
        telemetry.addData(">", "Done");
        telemetry.update();

    }
}
















