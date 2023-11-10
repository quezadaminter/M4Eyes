# M4Eyes
Implementation of the Adafruit Monster M4SK that controls the eye position via input from the Useful Sensors Person Sensor.

Use the eye[].eyeOffsetX and eye[].eyeOffsetY parameters to help compensate for misalignment when mounting the eyes on the prop.

Accepts a few commands via the serial interface:

| Command | Parameters | Function | Example |
|---------|------------|----------|---------|
| O | x1,y1,x2,y2 | Sets the eye position offset. Requires ALL 4 values. Must be numbers in the range [-1.0 and 1.0.] | O0.1,0.21,0.0,0.0 |
| P | x,y | Forces the eye focus point in both axes. Requires both values. Must be numbers in the range [-1.0, 1.0] | P0.5,-0.25 |
| D | | Toggles the Person Sensor debug mode on and off. It is not always clear if the Person Sensor handles this properly in a consistent manner... | D |

