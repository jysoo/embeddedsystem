# embeddedsystem

1. Spin for a defined number of rotations and stop without overshooting. <br />
--->Follow syntax in specification <br />


2. Spin at a defined angular velocity, either continuously or as a maximum while carrying out spec 1. <br />
--->Follow syntax in specification <br />


3. The normal precision is: <br />
(i) The nearest one rotation for number of rotations <br />
(ii) The nearest one rotations per second for angular velocity <br />

4. Operate at high precision: <br />
(i) The nearest 0.01 rotations for number of rotations <br />
(ii) The nearest 0.01 rotations per second for angular velocity <br />

5. Play a melody while it is spinning by modulating the control voltage. <br />
--->Follow syntax in specification <br />
--->Interchange from R/V to T **works without problems** during our testing after we took some precautions in the code (e.g. using int8 instead of int for integer variables so that we do not encounter 'out of memory' error). However, in the **unlikely event that an "out of memory" problem occurs, reset the motor.** <br />


6. Automatically tune its control parameters to optimise for a change in the connected moment of inertia. <br />
--->Send "M" <br />
--->After the above command finished executing, send "R..." or "V..." <br />


7. Detect wrong syntax of the input. When a wrong syntax is detected, corresponding error value is returned. (Error message printed). <br />

8. If the motor does not work at high speed commands such as V20, then **change line 520 from Thread::wait(10) to Thread::wait(1)**. <br/>
