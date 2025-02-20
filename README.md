# Building an Efficient Dual-Axis Solar Tracker with MPPT

Harnessing solar energy efficiently is a growing challenge in renewable energy research. In my final-year project, I designed and implemented a dual-axis solar tracker with a maximum power point tracker (MPPT) to optimise solar energy collection. This project involved electrical, mechanical, and software design to create a fully functional, stand-alone system capable of tracking the sun and maximising power output.

![IMG_3358](https://github.com/user-attachments/assets/e7b55b9b-970f-4096-96ae-679bb9c9e900)

## Understanding the Problem



Photovoltaic (PV) panels generate power based on the sunlight they receive. However, static panels are not always positioned optimally, leading to inefficiencies. Additionally, the power-voltage relationship of PV panels is nonlinear, requiring MPPT algorithms to extract the maximum power available at any given moment.

## Design Approach

### 1. Mechanical Design – Dual-Axis Tracking

To ensure optimal energy capture, I designed a system that moves in both azimuth (horizontal) and elevation (vertical) axes. The frame was adapted from a previous project that only allowed single-axis movement. A second motor was introduced to control the vertical angle, ensuring the panel’s orientation was adjusted daily for seasonal changes.

![IMG_0643 2](https://github.com/user-attachments/assets/f8ea6e58-c98d-410f-ab6a-fbc3e8efef51)

### 2. Electrical System – MPPT and Power Conversion

The MPPT algorithm utilized a modified Perturb and Observe (P&O) method, allowing real-time optimization of power output based on irradiance and load conditions.

A two-stage DC-DC converter was implemented:

-  Boost Converter: Increases the voltage to match the system’s DC bus.
-  Buck Converter: Regulates voltage to charge the battery using a constant current/constant voltage (CC-CV) algorithm.
-  Motor drive circuit was also implemented to control the DC motors used for the tracking movements.


![circuits copy](https://github.com/user-attachments/assets/a0bbb6f5-f819-401a-92c5-f1a78a6f4096)

### 3. Control System – Microcontroller and Software Integration

The entire system was controlled using an STM32 microcontroller, which:

-  Calculated the sun’s position based on time and location.
-  Controlled the motors for precise panel movement.
-  Managed the MPPT and battery charging processes.
-  A real-time clock (RTC) ensured accurate solar positioning calculations, while encoders provided feedback on panel positioning.



https://github.com/user-attachments/assets/28bed303-d0ea-48e3-a241-1f171b2a18dc


### 4. Solar Tracking Algorithm – PSA

The most accurate solar tracking algorithm is the solar prediction algorithm (SPA), providing an accuracy of 0.0003° over a large range of years. The SPA algorithms require multiple iterations to converge to the solution and, as such, are not suitable for implementation on microprocessors. A suitable algorithm for implementation on a microprocessor is the PSA algorithm, which provides an average tracking error of 0.002° in both axes.

## Key Challenges and Solutions

Balancing Power Consumption vs. Accuracy: Motor usage was minimised using an optimised movement schedule, preventing excessive energy drain.
Ensuring Stable MPPT Operation: The algorithm was fine-tuned to reduce oscillations around the maximum power point.
Conclusion

This project demonstrated the feasibility of an efficient dual-axis solar tracker with MPPT for stand-alone solar systems. Future improvements could include implementing AI-based tracking and optimising battery management for enhanced performance.

If you’re interested in more technical details or source code, feel free to reach out!
