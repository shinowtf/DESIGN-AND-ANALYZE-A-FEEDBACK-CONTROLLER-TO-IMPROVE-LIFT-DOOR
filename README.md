DEPARTMENT OF ELECTRICAL AND ELECTRONCS ENGINEERING
FACULTY OF ENGINEERING
NATIONAL DEFENCE UNVIERSITY OF MALAYSIA
LAB REPORT 2
EEE 3351 – ENGINEERING LAB V
SEMESTER 1 ACADEMIC SESSION 2024/2025
PROGRAMME ZK23
DESIGN AND ANALYZE A FEEDBACK CONTROLLER TO IMPROVE LIFT DOOR
LECTERUR
1.
Prof. Madya Ts. Gs. Dr. Syed Mohd Fairuz Bin Syed Mohd Dardin
2.
Dr. Siti Noormiza Binti Makhtar
3.
Dr. Chew Sue Ping
4.
Dr. Elya Binti Mohd Nor
NAME
NO MATRIC
LEE KAH CHUN
2220847

INTRODUCTION
Managing lift door mechanisms is a crucial aspect of the design and functioning of contemporary elevator systems. A key element that powers the lift door system is the DC motor, responsible for regulating the door's speed and motion. The motor needs precise regulation to guarantee seamless functioning, safety, and effectiveness. A major difficulty in creating such a system is ensuring accurate regulation of the motor’s speed, especially under different load conditions (i.e., the door's weight) and attaining the intended opening and closing durations of the lift door.
In this lab, we will investigate the design and execution of a PID (Proportional-Integral-Derivative) controller to manage the speed of the DC motor driving the lift door. The aim is to manage the motor speed to effectively control the lift door's movement. In particular, we will model the dynamics of the lift door, taking into account the influence of the door's weight and the motor shaft's angular velocity. The controller's effectiveness will be evaluated in two different scenarios:
1.
Lacking PID Control (Open-loop): This describes how the system operates when feedback control is not utilized. The motor functions entirely on its dynamics, potentially causing delayed responses, overshoot, or errors at steady state.
2.
Using PID Control: A PID controller is utilized to modify the motor's speed instantaneously depending on the discrepancy between the intended and actual position of the door. The controller seeks to reduce the overshoot, shorten the settling time, and eradicate steady-state error.
Overview of the System
The system is made up of:
1.
DC Motor: The motor operates the lift door, and its velocity is controlled by the input voltage. The motor is represented as a second-order system, with the angular velocity of the motor as the output.
2.
Lift Door: The position of the lift door is directly connected to the angular movement of the motor's shaft. The door's weight contributes to the motor's load, influencing the dynamics of the system. The door's weight creates a resistive force that the motor must counter in order to open the door.
3.
Motor Speed Regulation: The angular velocity of the motor determines the position of the door, thereby affecting the speed at which the door opens and closes based on the motor's reaction to input control signals.
System Dynamics
1.
Angular Velocity of the Motor: The motor's pace, quantified in radians per second, controls how quickly the door opens or closes.
2.
Weight of the Lift Door: The door's heaviness serves as a burden on the motor. The motor needs to generate sufficient torque to raise or lower the door.
The transfer function of the DC motor defines the connection between the input voltage and the resulting angular velocity, influenced by the motor's physical traits (inertia, damping, and resistance) as well as the external load (the door's weight).
4
PROBLEM STATEMENT
We will model the lift door system utilizing two distinct control approaches:
1.
Without PID Control (Open-Loop System):
In this instance, the motor functions without any feedback regulation. The door's placement is determined exclusively by the motor's natural dynamics and the load from the weight of the door. The system operates in an open-loop manner, indicating that the motor's input is not modified according to the door's position.
2.
Utilizing PID Control (Closed-Loop System):
In this scenario, a PID controller is employed to regulate the speed of the motor. The controller employs feedback to modify the motor's input voltage, decreasing the discrepancy between the intended and actual door position. The goal of the PID controller is to reduce the error by modifying the motor speed, enhancing the system’s effectiveness by decreasing overshoot and guaranteeing that the door attains its target position with minimal lag.
OBJECTIVES
1.
Simulate the behavior of the DC motor and the elevator door mechanism.
2.
Test the system's performance both with and without PID control.
3.
Assess the impact of the PID controller on the performance of the system, particularly concentrating on:
•
Settling Time: The duration required for the door to attain a specific threshold of the target position and stay there.
•
Overshoot: The degree to which the door position goes beyond the intended position prior to stabilizing.
•
Response Time: The duration needed for the door to react to the setpoint (intended position).
METHODOLOGY
START
Define system requirements
Model the lift door
Design PID Controller
Develop control algorithm
Implement DC motor model
Simulate system in MATLAB
Evaluate system response
Adjust PID parameters
Final system validation
END
NO
YES
The approach starts by modeling the dynamics of the lift door, combining the behavior of the DC motor with the movement of the door, taking into account factors like inertia and friction. A PID control algorithm is subsequently created, adjusting the motor speed according to the difference between the desired and actual position of the door. The system is modeled in MATLAB to assess and analyze the performance of the controller, focusing on essential performance indicators like response time, overshoot, and settling time. The PID parameters (Kp, Ki, Kd) are adjusted through iterations to enhance system response and reduce steady-state error, guaranteeing that the lift door functions efficiently and adheres to time specifications.
MECHANICAL DESIGN OF LIFT DOOR GEAR SYSTEM
In the design shown in the image, we referenced open-source materials from GrabCAD to create our lift door system. The design incorporates a single gear with a radius of 0.2 meters, which is compatible with the DC motor used to open the door. This setup forms the basis of our lift door system design.
Source cited: https://grabcad.com/library/sliding-door-mechanism-2
Explanation about the door and the motor
Physical Elements of the Elevator Door System:
➢
Doors: The system includes a pair of doors, with each door weighing 50 kg. The weight of the door is evenly spread, and it is presumed to have a rectangular form.
➢
Weight of the Door: 50 kg each.
➢
Width of Door: The greatest extent the door swings is 1 meter (door width).
➢
Motor: The motor is tasked with powering the movement of the door. The motor produces torque that is essential to counteract the door's inertia and the frictional forces that could occur because of the door's weight.
➢
Torque Needed: The door needs a specific torque to counteract the gravitational force applied to it. This torque is determined by employing the equation:
𝑟𝑒𝑞𝑢𝑖𝑟𝑒𝑑 𝑡𝑜𝑟𝑞𝑢𝑒∶𝐹 𝑥 𝑅 𝐹=100𝑘𝑔 𝑥 9.81=981 𝑁 𝑅𝑒𝑞𝑢𝑖𝑟𝑒𝑑 𝑡𝑜𝑟𝑞𝑢𝑒=981 𝑥 0.2=196.2 𝑁𝑚
TYPE OF DC MOTOR
Typical Parameters for DC Motors (General Information)
For a DC motor like the C23-L50, the key parameters we use is:
1.
Resistance (R):
o
The resistance of the motor is 16.50 ohm.
2.
Inductance (L):
o
The inductance value is 27.00 miliHenry.
3.
Back Electromotive Force (Back EMF) Constant (K):
o
This constant relates the motor’s angular velocity to the voltage induced by the motor's rotation (back EMF). The value is 0.3056 volts/rad/sec.
4.
Inertia (J):
o
The inertia of the motor's rotor (in kg·m²) is necessary to model the rotational dynamics. The value is 459.0 g-cm2.
5.
Damping Coefficient (B):
o
This is the resistance to motion caused by friction and other forces.. The value is 0.000009549.
6.
Nominal Voltage (V):
o
The rated operating voltage of the motor, which is using 60V.
7.
Torque constant
o
This is the constant that relates the armature current to the output torque. The value is 0.3065 N m / amp.
MATLAB CODE
The first section of the code sets up the parameters for the door.
•
Door Weight: The total weight of the doors is 100 kg.
•
Door Width: The maximum distance the door can open is 1 meter.
•
Time to Open: The time it takes for the door to open fully (5 seconds).
•
Gravitational Force (g): The acceleration due to gravity (9.81 m/s²), used to calculate the weight force.
•
Torque Calculation: The torque required to lift the door is calculated by assuming a pulley mechanism. The torque is the force multiplied by the radius of the pulley (0.2 m).
MOTOR PARAMETERS
Rotor Inertia (J): Indicates how much the motor rotor resists variations in angular velocity. The value is 0.0000459 kg m-2.
Damping Coefficient (B): Represents the energy losses caused by friction in the motor. The value is 0.000009549 N m s / rad.
Motor Constant (K): Establishes the connection between the motor current and the torque produced. The value is 0.3065 N m / A.
Armature Resistance (R) and Inductance (L): These define the electrical properties of the motor. The value R is 16.5 Ω and the L is 0.027 H.
Combined System Inertia: The door's inertia is represented as a rectangular shape, with the overall system inertia being the sum of the motor's inertia plus the door's inertia, taking into account the pulley radius.
This section sets up the simulation parameters:
•
Simulation Time (sim_time): The total duration of the simulation is 10 seconds.
•
Time Step (ts): The time step for numerical integration is 1 ms (0.001 seconds), which ensures that the simulation is run with enough granularity to capture the dynamics.
•
Time Array (t): A time array is created using the specified time step.
Additionally, PID parameters (Kp, Ki, Kd) are set, which will be used for the closed-loop control later in the simulation.
Initial Conditions for Open-Loop and Closed-Loop Systems:
•
Initial Conditions (No PID): The door starts at rest (position and velocity are both zero). The motor’s armature current is also initially zero.
•
Initial Conditions (With PID): Similarly, the door starts at rest for the closed-loop system. The PID controller is initialized with an integral term of zero, and the previous error is also zero.
Simulating Motor Dynamics Without PID Control (Open-Loop):
In the open-loop simulation, the motor dynamics are modeled without any feedback from the door's position (i.e., no PID control is applied). The steps include:
•
Torque Generation: The torque generated by the motor depends on the armature current (i_a_no_pid) and the noise added to simulate disturbances.
•
Angular Velocity and Position: The angular acceleration (alpha_no_pid) is calculated using the torque and damping. The velocity and position are updated using numerical integration (Euler method).
o
Noise: Sensor noise is added to the measured angular velocity (omega_no_pid_measured), and random voltage noise is added to simulate voltage fluctuations.
o
Position Saturation: The door's position is limited between 0 and the maximum width (door_width = 1m).
•
Armature Current Dynamics: The armature current is updated using the input voltage, back EMF (proportional to the angular velocity), and the motor's electrical properties (resistance R and inductance L).

Simulating Motor Dynamics With PID Control (Closed-Loop):
In the closed-loop simulation, a PID controller is used to control the motor and make the door reach the target position (door_width = 1m). The steps include:
•
Error Calculation: The error is the difference between the target position and the current position of the door.
•
PID Control Law: The control signal is calculated using the proportional, integral, and derivative terms. The formula for the control signal is:
𝐶𝑜𝑛𝑡𝑟𝑜𝑙 𝑠𝑖𝑔𝑛𝑎𝑙=𝐾𝑝 𝑥 𝐸𝑟𝑟𝑜𝑟+𝐾𝑖 𝑥 ʃ 𝐸𝑟𝑟𝑜𝑟+𝐾𝑑 𝑥 𝑑𝐸𝑟𝑟𝑜𝑟𝑑𝑡
•
The integral term helps eliminate steady-state error, while the derivative term anticipates the system's future behavior.
•
Voltage Saturation: The control signal (voltage) is saturated within the motor's voltage range (0 to 24V).
•
Torque, Angular Velocity, and Position: The dynamics of the motor are simulated as in the open-loop case, but with feedback from the PID controller influencing the system's response.
•
Armature Current Update: Similar to the open-loop case, the armature current is updated based on the input voltage and the motor's electrical characteristics.
Time to Fully Open the Door:
After running the simulations, the time to fully open the door is calculated for both open-loop and closed-loop systems. The code checks the time at which the door's position reaches 1 meter (door_width).
•
Time to Open Without PID: This is the time when the door reaches the target position in the open-loop system.
•
Time to Open With PID: This is the time when the door reaches the target position in the closed-loop system.
Step Response of the Motor (Transfer Function):
A transfer function for the motor is created based on its electrical and mechanical properties. This transfer function is used to simulate the motor’s step response to a 12V input over 20 seconds, which helps visualize how the motor reacts to a step change in voltage.
The transfer function is derived from the motor's differential equations and relates the input voltage to the motor's angular velocity or position.
Decay of Angular Velocity After Opening:
Once the door reaches its maximum opening, the simulation introduces an exponential decay in the motor's angular velocity to simulate the effects of friction and other losses. Small random noise is also added to this decay to further simulate real-world conditions.
NOISES TO SIMULATE REAL WORLD APPLICATION
Real-world applications control systems are often subject to various types of noise that can affect performance and accuracy. To simulate this, we add sensor noise, torque noise, and voltage noise. Sensor noise, modeled as Gaussian noise, simulates inaccuracies in the measurement of angular velocity and position, reflecting imperfections in real sensors. Torque noise introduces random fluctuations in the applied torque, mimicking disturbances such as mechanical vibrations or frictional variations in the system. Voltage noise represents random variations in the control voltage, which can occur due to inconsistencies in the power supply, affecting the precision of motor control. These noises are
incorporated into the control system to more accurately model the challenges faced in practical, real-world environments.
Visualization:
Finally, the results of the simulation are visualized:
•
Position vs Time: A plot shows the position of the door over time, comparing the open-loop (no PID) and closed-loop (with PID) systems. The door’s position is plotted against time for both cases, illustrating how the PID controller improves the system's performance.
•
Motor Angular Velocity vs Time: Another plot shows the motor's angular velocity over time, showing how the velocity changes in both open-loop
and closed
-loop scenarios. After the door is fully open, an exponential decay is applied to the velocity to simulate friction and losses.
SIMULINK MODELING
Top Model: Open-Loop Control
1. Input Signal (Step Function):
The step input provides a predefined signal to the system. In this case, it acts as the desired speed for the DC motor.
2. Summation Block:
There is a summation block present, but it is not connected to a feedback signal. As a result, this block simply passes the step signal forward without any error correction.
3. DC Motor Block:
The step signal is converted into a voltage input for the DC motor. The motor block simulates the behavior of a real DC motor, producing an output speed based on the input voltage.
4. Scope:
The motor's output speed is sent to a scope, where its response can be visualized over time. The scope provides a way to observe the system's performance (e.g., rise time, steady-state speed).
Characteristics:
No Feedback: The system doesn't monitor or correct the actual motor speed.
Prone to Error: If there are external disturbances (e.g., load changes), the motor speed may deviate from the desired value.
Simplicity: This control system is straightforward to implement but lacks precision.
Bottom Model: Closed-Loop Control with PID
1. Input Signal (Step Function):
Similar to the top model, the step input provides the reference or desired speed for the motor.
2. Summation Block:
This block calculates the error by subtracting the actual motor speed (feedback signal) from the desired speed. The error indicates how far the actual speed is from the desired speed.
3. PID Controller:
The error is fed into a PID controller, which adjusts the input voltage to minimize the error.
Proportional (P): Corrects errors proportional to the magnitude of the error.
Integral (I): Addresses accumulated past errors to eliminate steady-state error.
Derivative (D): Predicts future error based on the rate of change of the error.
The PID controller ensures the motor speed converges to the desired value quickly and accurately.
4.DC Motor Block:
The controlled voltage from the PID drives the motor. The motor outputs its speed, which is fed back into the summation block to close the loop.
5. Scope:
Like the top model, the scope displays the motor's speed response. With a properly tuned PID, the response should show minimal overshoot, fast settling time, and zero steady-state error.
ADVANTAGES AND LIMITATIONS
1. Open-Loop System:
Advantages:
•
Simple to design and implement.
•
No additional hardware required for feedback.
Limitations:
•
Inability to adjust to disturbances or parameter changes.
•
Accuracy depends heavily on precise modeling and consistent operating conditions.
2. Closed-Loop System with PID:
Advantages:
•
Precise and robust control of motor speed.
•
Adapts to changes in load or external disturbances.
Limitations:
•
More complex to design and tune (requires determining Kp, Ki, Kd values).
•
Requires additional components for feedback (e.g., sensors).
RESULT & DISCUSSION MATLAB CODE
Time to Fully Open the Door:
The time to fully open the door is the point where the door reaches its maximum position of 1 meter (specified by door_width).Time is being range by ± sign as the random noise is not always the same everything simulating real world applications.
•
Without PID (7.8 ± 0.1 seconds): In this open-loop simulation, the motor is driven by a constant input, but no feedback is used to adjust its speed or position. As a result, the door opens in a less controlled manner. It might take longer to reach the target position because:
o
The motor doesn't account for any position errors.
o
Noise (such as torque noise, sensor noise, and voltage fluctuations) impacts the motor's performance, further delaying the door’s movement.
o
Since there is no feedback loop (as in the case with PID), the system cannot correct its motion if there are deviations from the desired position, causing a longer time to stabilize and reach the target.
Hence, the open-loop system (no PID) takes 7.8± 0.1 seconds to fully open the door.
•
With PID (5.8 ± 0.1 seconds): The PID controller works by adjusting the input voltage to the motor continuously based on the door’s position error. The controller tries to minimize the difference between the current position and the desired position (which is 1 meter in this case). The key benefits of using PID control include:
o
Proportional action helps reduce the error by directly correcting based on the current error.
o
Integral action helps eliminate steady-state errors by accumulating past errors.
o
Derivative action helps smooth out rapid changes and reduce overshoot.
The PID controller corrects the door's motion more effectively, minimizing the time it takes to reach the target position. As a result, the door opens in 5.8± 0.1 seconds, which is significantly faster than the open-loop system.
RESULT & DISCUSSION SIMULINK
THE RESULT
NOTES
This graph shows that there is steady-state error for without PID dc motor to achieve stable.
PID Controller is tuned by PID tuner with overshoot 0%, best PID value is obtained.
PID Tuner use for tuning the best PID value
PID Tuner show parameter will show all controller information, value of rise time, settling time and overshoot for tune. The red block highlight our final tuned PID value and controller information.
DISCUSSION
The system's performance with and without a PID controller is thoroughly contrasted in the graphs. The system with PID (red curve) responds far more rapidly and precisely, reaching the target position of about 1 metre in less time, according to the Door Position vs. Time figure. This shows that the PID controller efficiently reduces mistakes and guarantees that the system quickly approaches the desired location. The system without PID (blue curve), on the other hand, responds more slowly and gradually, taking a lot longer and achieving the required position with less accuracy. This demonstrates the limits of a system that lacks sophisticated feedback control since it finds it difficult to effectively rectify deviations. The variances are further shown by the Motor Angular Velocity vs. Time graphic. Although the angular velocity of the PID-controlled system increases quickly, it overshoots by around 5–6 seconds, most likely as a result of the controller's aggressive reaction. The quick stabilisation that follows this overshoot, however, shows how the PID can adapt and keep control even in the face of early transitory behaviour. The system without PID, on the other hand, exhibits a consistent but significantly slower increase in angular velocity and lacks the responsiveness required for dynamic control. Furthermore, both systems are impacted by noise and decay, but the PID controller can respond to these disruptions more rapidly. Although it causes temporary overshoot in return for longer-term stability and precision gains, the PID controller improves the system's performance overall by providing quicker responses, increased accuracy, and efficient disturbance rejection.
CONCLUSION
This lab concludes by demonstrating the many benefits of employing a PID controller in dynamic systems. The faster convergence to the goal position in the door control experiment unequivocally demonstrates that the PID controller enhances the system's reaction speed and accuracy. Furthermore, despite early overshooting—a trade-off for its rapid response to changes—the PID controller stabilises the motor's angular velocity and efficiently handles transient disturbances. The shortcomings of systems lacking feedback mechanisms are demonstrated by the system without PID, which has shorter reaction times and finds it difficult to attain exact control. Despite a brief overshoot in angular velocity, the PID-controlled system rapidly stabilises and outperforms the uncontrolled system in terms of overall performance. PID controllers are crucial for enhancing system performance in real-world situations with noise and disturbances, as demonstrated by this experiment, which highlights its significance in applications needing quick, precise, and reliable responses.
REFRENCES
➢
Bennett, S. (1993). Development of the PID controller. IEEE Control Systems Magazine, 13(6), 58-62.
➢
Xu, X., & Wang, Q. (2017, May). Speed control of hydraulic elevator by using PID controller and self-tuning fuzzy PID controller. In 2017 32nd Youth Academic Annual Conference of Chinese Association of Automation (YAC) (pp. 812-817). IEEE.
➢
https://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=ControlPID
➢
Free CAD designs, files & 3D models | The GrabCAD Community Library. (n.d.). https://grabcad.com/library/sliding-door-mechanism-2
