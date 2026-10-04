/* Matlab Code used for curve fitting
height = [15 20 30 40 50 60 70 80]';
volts = [2.55 2.35 1.85 1.44 1.16 0.95 0.84 0.70]';
f = fit(volts,height,'power2')
plot(f,volts,height)


height = a*(volts^b) + c
-- fitting results --
a =    122.9  
b =  -0.4514 
c =   -64.28

volts = (analogread * 5 )/ 1024
*/

void read_IR_sensor()
{
  // From the datasheet, after doing a curve fitting at the sensor reading
  distance2ground = 122.9*((float)pow((float)(analogRead(A0)*5)/1024,-0.4514)) - 64.28; //gives the height in cm
  pos_I[2] = distance2ground / 100; //Z position in meters
  SATURATION(pos_I[2],3,0);
}
