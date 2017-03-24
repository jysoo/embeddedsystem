# embeddedsystem

A. Spin for a defined number of rotations and stop without overshooting.
--->Follow syntax in specification


B. Spin at a defined angular velocity, either continuously or as a maximum while carrying out spec 1.
--->Follow syntax in specification


C. The normal precision is:
(i) The nearest one rotation for number of rotations
(ii) The nearest one rotations per second for angular velocity

D. Operate at high precision:
(i) The nearest 0.01 rotations for number of rotations
(ii) The nearest 0.01 rotations per second for angular velocity

E. Play a melody while it is spinning by modulating the control voltage.
--->Follow syntax in specification
--->Interchange from R/V to T works without problems during our testing after we took some precautions in the code (e.g. using int8 instead of int for integer variables so that we do not encounter 'out of memory' error). However, in the unlikely event that an "out of memory" problem occurs, reset the motor.


F. Automatically tune its control parameters to optimise for a change in the connected moment of inertia.
--->Send "A"
--->After the above command finished executing, send "R..." or "V..."


G. Detect wrong syntax of the input. When a wrong syntax is detected, corresponding error value is returned. (Error message printed).
