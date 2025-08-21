# Competition Car Design – WRO Future Engineer
**Team Name:** Bloody Hilux  

This document describes the design of our competition car, developed for the *WRO Future Engineer* challenge.  
The car was engineered to combine **speed, stability, precise turning, and efficiency**, using a balance between carefully selected components, optimized mechanics, and a lightweight chassis.

---

## 1. Motors & Selection
The car is powered by a **DC motor** with the following specifications:
- Maximum speed: **281 RPM**
- Maximum torque: **5 kgf·cm (~0.49 N·m)**

This motor was selected because it provides the best trade-off between torque and speed. It ensures:
- Strong performance in acceleration  
- Reliable top speed on straight tracks  
- Compatibility with the chosen gear ratio and wheel size  

By tuning the gear ratio (see below), we ensure that the motor provides both speed and enough torque to overcome track resistance.

---

## 2. Drivetrain & Gear Ratio
The drivetrain is the link between the motor and the wheels. It consists of:
- **Gear ratio:** 2.2:1  
  - This means for every 2.2 motor rotations, the wheel completes 1 rotation.  
  - This increases **wheel speed** but slightly reduces torque, a trade-off suitable for flat tracks.
- **Differential system:** A 1:1 bevel gear system with 4 internal gears  
  - Allows left and right wheels to rotate at different speeds while turning.  
  - Minimizes wheel slip and improves cornering stability.
- **Bearings:** 12 precision ball bearings  
  - Reduce friction  
  - Increase efficiency and smoothness of drivetrain  

This system ensures **power efficiency and stability** while turning.

---

## 3. Speed Calculation
Given the motor speed and wheel diameter, we calculate top speed:

- Motor RPM: 281  
- Wheel RPM = 281 × 2.2 ≈ 618  
- Wheel diameter = 59 mm → circumference ≈ 0.185 m  

 
---

## 4. Power Calculation
- Motor torque: \(T_\{motor} = 0.49~\{N·m}\)  
- Motor angular velocity: \(\omega_\{motor} = 29.41~\{rad/s}\)  

 
**At the wheels (after gear ratio):**
- Torque: \(T_\{wheel} = \frac{0.49}{2.2} \approx 0.223~\{N·m}\)  
- Angular velocity: \(\omega_\{wheel} = 29.41 \times 2.2 \approx 64.7~\{rad/s}\)  
- Power: \(P_\{wheel} = 0.223 \times 64.7 \approx 14.4~\{W}\)  

 
---

## 5. Torque & Linear Force
- Wheel radius: \(r = 0.0295~\{m}\)  
- Torque at wheels: \(T_\{wheel} = 0.223~\{N·m}\)  

---

## 6. Steering Mechanism (Ackermann)
The steering system follows **Ackermann geometry**:
- Inner wheel angle: **65°**  
- Outer wheel angle: **38°**  

This ensures that:
- Each wheel follows the correct turning radius.  
- Tire drag is minimized.  
- Cornering becomes precise and stable.  

The steering is controlled by a **single servo motor**, connected with a durable linkage system.

---

## 7. Chassis & Structure
- **Weight:** ~1.25 – 1.3 kg  
- **Dimensions:** 23 cm (length) × 20 cm (width)  
- **Design:** Double plexiglass plates with 3D-printed support components  
- **Balance:** Components are placed to keep the center of gravity low  

This combination provides both **durability and lightness**, key for competitions where stability and speed are critical.

---

## 8. Assembly & CAD Design
All custom parts were designed in **CAD software** and manufactured with **3D printing** for precision.  
Features:
- Modular design → quick part replacement  
- Optimized geometry → reduced friction  
- Bearings and inserts → increased durability  

---

## 9. Summary
To summarize, our car balances mechanics, power, and control:

- **Motor & Gear Ratio** → Provide high speed while maintaining torque  
- **Differential** → Smooth turning and power distribution  
- **Ackermann Steering** → Precision and minimal drag in corners  
- **Chassis** → Lightweight, low center of gravity, strong structure  
- **Bearings** → Smooth motion with reduced friction  
- **Calculated Performance**:  
  - Max speed ≈ 1.91 m/s (~6.9 km/h)  
  - Power ≈ 14.4 W  
  - Wheel torque ≈ 0.223 N·m  
  - Linear force ≈ 7.56 N  


 
