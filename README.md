Welcome to the README for the Oregon Tech RoboSub Software

Written By: Theodor Giles

**Syntax, Naming, etc.**

Currently, I use my own custom system for variable naming, syntax, etc.

CamelCase is used for most Very Important Variables. Data for calculation, communication, etc. Also used for more complex class methods.

thisCase is for basic methods like setting, getting, printing, etc.

lowercase is used for variables that are unimportant, method arguments, or basic members. No methods.

UPPER_CASE_SPACED is mainly for constants and valuable arrays, no methods.

i hate using single digit variables, but when I do, it signifies an index or counter.

This_case1 signifies a piece of hardware, the type being *This* and the *case1* meaning the designation and sometimes which # hardpoint it is. Example below. 

```
self.Gyro_drone1 = imu_ard_data.WT61P(self.serial) 

self.Thruster_VentralLB = ThrusterDriver("LB")  # left back  
```
