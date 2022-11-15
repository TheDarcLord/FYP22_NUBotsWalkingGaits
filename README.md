## **2022 Final Year Project** - *NUBots Walking Gaits*
***
**Supervisor**
: Dr Joel Ferguson

**Student**
: Darcy Byrne

### **_Formative**
Formative contains a collection of MATLAB examples. Each of these run independently.

### **NUgus_Global_Quasi**
Simulation of NUbots' original robot using the ***Quasistatic*** approach to walking. Global Coordinates means the position of each **End Effector** (The *Left* and *Right* Foot) is tracked for all time. Thus, when plotted, 
the NUgus travels from the **zero** position to the end of an arbitrary trajectory. Global simulations run slowly as the aforementioned indexed **End Effector** and **Base** positions are relatively large variables.

### **NUgus_Local_Quasi**
Local Coordinates means the position of the **Base** is considered to be the origin.
Thus, after taking a step, the position of the forward foot is the new **Base** of the kinematic chain, stretching (presumably) backwards to the what was the old **Base** but is now the **End Effector**. The **Homogenous Transform** between the **Base** and the **End Effector** can then be applied to the entire trajectory, thus shifting it into the coordinate system associated with the new **Base**, before
taking a subsequent step. Thus, when plotted, the NUgus never travels from the **zero** position, while the arbitrary trajectory appears to travel towards the NUgus.

### **NUgus_Global_ZMP**
Simulation of NUbots' original robot using the ***Zero Moment Point*** approach to walking. This results in a **Dynamically Stable** walk. Imagine, if you will, a pendulum. When upright no Torque exists above the pivot point. However, as the pendulum drifts from this position, torque gradually increases about the pivot point. To counteract this torque, the approach is to accelerate the mass of the pendulum in the planar opposite direction, thus cancelling the torque at the pivot point. Thus, the pivot point becomes the **Zero Moment Point**. To walk, **Preview Control** is utilised to shift the **Zero Moment Point** to the position of the 
footstep just prior to foot fall.

### **NUgus_Local_ZMP**
Simulation of NUbots' original robot using the ***Zero Moment Point*** approach to walking, but in Local Coordinates (see above).