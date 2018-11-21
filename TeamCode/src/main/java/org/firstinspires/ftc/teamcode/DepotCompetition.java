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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 *
 * This is a test Autonomous code to check the workings of the "moveInches" and "rotate" commands
 * in the 2018 HardwareJoeBots class.
 *
 */

@Autonomous(name="Depot Corner", group="8513")
//@Disabled
public class DepotCompetition extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareJoeBot2018      robot   = new HardwareJoeBot2018();

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, this);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //Detaches robot from lander
        robot.raiseLift();
        robot.StrafeRobot(9,'L', 10);
        //robot.lowerLift();

        //Gets the robot to the depot and drops the marker
        //robot.StrafeRobot(7,'R',10);
        robot.moveInches(43, 0.75, 15);
        robot.rotate(62,0.25);
        robot.moveInches(17, 0.75, 15);
        //robot.rotate(128,0.15);
        //robot.moveInches(50, 0.75, 15);
        robot.dropMarker();
        //robot.rotate(90,0.15);

        //Gets robot from depot to crater
        robot.moveInches(-64, 0.75, 15);
        //robot.rotate(90,0.15);

        //robot.moveInches(-10, .3, 15);
        //robot.rotate(90,0.15);

        //robot.moveInches(30, .75, 15);


    }

}