

This document explains how to use the TSDZ8_Assist_curve_generator.html tool.

# 1.Purpose
OSF has several assist modes (power, torque, hybrid, eMTB, cadence). Except in cadence mode, the torque sensor is essential to calculate a target current and so the motor assistance.

For each assist mode, the user can define different levels of assistance (Eco, tour, turbo,...). The number of levels depends on the type of display. It is at least 4 (e.g. VLCD5) and at max 9 (e.g. 860C).

In order to configure the TSDZ8 with the same user interface (javaconfigurator or 860C firmware,...) as for the TSDZ2, each level of assistance is defined by just one value (in range 0...255). Still, in this TSDZ8 OSF version, the value combines 3 different characteristics:
* The max current percentage (can be 0=20%, 1=40%, 2=60%, 3=80%, 4=100%) 
* The pedal weight where the max current is reached = saturation weight (can be 0=16, 1=20, 2=24, 3=28, 4=32, 5=36, 6=40, 7=48, 8=64, 9=88 kg)
* The curve to be applied to convert torque to current (can be 0=LOW, 1=Medium, 2=High, 3=Linear, 4=Decrease). Low means that the current is very very low for low weight and grow more and more up to reach max current for the saturation weight. Medium is similar to Low but already closier to a linear curve (Medium = quadratic: when the weight doubles, the current quadruples). High is even closier to linear curve; Linear means that current is proportional to weight. Decrease means that the current grows very fast for low weight but grows much less when weight becomes close to the saturation weight.

The value to fill is (max current * 50 ) + (saturation weight * 5) + (curve).
E.g. For Max current 60% reached at 28 kg with a linear curve, level would be 2*50 + 3*5 + 3 = 118

The htlm tool can be used to manage all those parameters. It can be used in 2 ways : Normal and Advanced

# 2. Normal

In Normal use, the user can select (in the top right frame) for each assist level (1...9):
- the max current (in % of the maximum current defined by 2 other general parameters : motor power + battery current)
- the saturation torque: this is weight at which the maximum current is already reached
- the profile : this is the curve that define how the weight(=torque) is converted to a target current (= assistance provided by the motor). The profile applies when the weight is less than the saturation torque.
The last column gives the code to be used as asssit level in the user interface (javaconfigurator, 860C, ...).

In the top left frame, you can see in a graphical way how the assistance is provided depending on the weight for each level. This can be very usefull.

Note: if you use less than 9 assist levels, best is to fill the unused levels (in the html tool) with the same options as those from one used level. So you see only used curves.

In Normal use, you can't change/edit other frames and so the options about saturation torque and profile are fixed.


# 3. Advanced

It is possible for a user to change saturation torque and profile options. Still pay attention that in this case you have to edit the C code of the firmware and recompile yourself. Changes have then to be put in file "ebike_app.c" in array variables assist_cmax_kg[] and/or assist_curve_lut[].

If you want change saturation torque and/or profile options, you have to enable the box "Enable LUT/profile editing" (in the frame below the graphic).

You can now edit directly the saturation torque table (=assist_cmax_kg).

In the frame curve generator, you can change the name of each profile.

When the radio button is set on "Manual", you can directly edit the values in the lookup table. A curve is defined by 17 values (=16 equidistant steps); the lowest must be 0 and the latest 255.

It is not easy to get a regular curve if you manually edit the values in llokup table. To help you, you can set the radio button on 5-param. In  this case, you can change the next five parameters (for each curve separately). The first parameter "X0" allows the curve to be shifted more or less to the right (avoid assistance for very low weight). Start and End slope let you setup the slope of the curve at start and end point. Start and end curve let you change the transition between start and end points. Changes are reported automatically in the lookup table and in the graphic frames.

