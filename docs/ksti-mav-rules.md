# KSTI 2026 – Mini Autonomous Vehicle: Competition Rules

Source: `GUIDEBOOK_PESERTA_MINI_AUTONOMOUS_VEHICLE_KSTI_2026.pdf` (Chapters I–IV).
This document is a structured transcription, not an official source. If it differs from the PDF or from organizer announcements, **the organizer is correct**. Items marked **[A#]** have notes in the "Ambiguities" section at the end. Do not guess those items; ask the user or the organizer contact.

## 1. Event information

- Name: Mini Autonomous Vehicle, part of Robotics Competition at KSTI 2026.
- Date/venue: 12 November 2026, on-site, Robotics Hall ITS, Surabaya.
- Technical Meeting: 8 November 2026 (announcement of registered teams and group assignments).
- Registration: https://its-robotics.com/ (choose KSTI 2026, category Mini Autonomous Vehicle).
- Fee: Rp150,000 (batch 1), Rp175,000 (batch 2). Non-refundable unless the competition is cancelled.
- Registration verification within 2x24 hours of submission.
- Official information is only shared through the official registration platform and official social media accounts of Robotics Competition at KSTI 2026.

## 2. Team requirements

- Participants must be students (proof: student ID card). No limit on number of teams per institution.
- Maximum 5 members per team, 1 team leader. Each participant and robot may only belong to one team.
- Team or robot names must not contain SARA (ethnic, religious, racial, group issues), pornography, insults, harassment. If two teams have the same name, the team that completed administration first gets the name. **[A4]**

## 3. Prizes

| Rank | Prize |
|---|---|
| 1st place | Gold medal + certificate + cash prize (written as `Rp4.00.000,00`) **[A3]** |
| 2nd place | Silver medal + certificate + cash prize (written as `Rp3.00.000,00`) |
| 3rd place | Bronze medal + certificate + cash prize (written as `Rp2.00.000,00`) |

## 4. Missions

- **Autonomous Race (AR):** navigate from start to finish on the prepared track as fast as possible.
- **Autonomous Race Challenge (ARC):** same as AR, but avoid obstacles along the way. Optional, only for teams that qualified for the final.

## 5. Robot requirements

### 5.1 General
- Electric car at 1:10 scale. Fuel-powered vehicles are strictly prohibited.
- At least one steering axle.
- Must have a **physical emergency stop button** within easy reach, and **another button to start navigation**.
- One car or one platform per team. Changing the platform is banned; modifying components for repair is allowed.

### 5.2 Power
- Must be electric. No limit on number or capacity of batteries.

### 5.3 Drive and steering
- 4WD and 2WD are allowed.
- Omni wheels, mecanum wheels, or other configurations that allow motion in all directions are banned.
- "The robot must implement a differential drive system." **[A1]**
- Steering system is free.
- Bonus for Ackermann steering (single-axle or two-axle): `STP = 0.25`. **[A1]**

### 5.4 Dimensions

| Dimension | Minimum | Maximum |
|---|---|---|
| Track width (distance between left and right wheel centers on the same axle) | 130 mm | 350 mm |
| Wheel base (distance between front and rear axle centers) | 160 mm | 550 mm |
| Height (ground to highest point) | – | 450 mm |
| Width (including all components and attached equipment) | – | 400 mm |
| Length (front to back, including all components) | – | 600 mm |

### 5.5 Sensors
- **Banned:** infrared sensors and/or other line follower sensors.
- Other sensors are allowed, except those that can directly follow the line. **[A5]**

### 5.6 Communication
- Any communication (wireless or wired) used to control or talk to the robot **during the competition is strictly prohibited**.
- The robot must be fully autonomous during runs.

## 6. Track

- Marking color is chosen to match the arena floor, so teams must work with both white and black markings.
- Checkpoints are shown in **green**; the start/finish line is shown in **red**. However, checkpoints do not need any marking, and the start/finish line uses a color different from the track markings, "not necessarily red". **[A11]**
- Track width: 50 cm ± 10%, **not necessarily constant** along the track.
- Minimum turning radius (outer radius): 1.5 m ± 10%.
- Markings: two straight continuous edge lines (white or black); one dashed center line in the same color as the edge lines.
- Marking width at least 1 cm. Gap between dashes at least 5 cm.
- Checkpoints are placed along the track. An example map is linked as "Lintasan" in the PDF (the link did not survive extraction). **[A9]**

## 7. Competition rules: Autonomous Race

Two stages: **Elimination** (must reach a minimum score to advance) and **Final** (pure score from speed and number of checkpoints passed). **[A9]**

1. Team must be ready at the start area 5 minutes before its run time.
2. Not ready when called = the attempt is forfeited.
3. 3 attempts, each with a 5-minute time limit. The **highest score** of the three attempts is used.
4. Goal: cross the finish line as fast as possible, passing all checkpoints within 5 minutes.
5. If 5 minutes end before finishing, the team must stop the car; points are counted from progress made.
6. 10 minutes preparation time between attempts. Allowed: small adjustments, sensor calibration, reviewing strategy, car repairs.
7. Checkpoints **must be passed in order**. If not: violation; the car is stopped, moved to the last checkpoint, and restarted with no time added by the organizer.
8. The car must stay on the track (left or right lane, as long as it does not cross the edge lines). **The car is considered off track if 30% of its body is outside the track.** Counted as a violation; the car is stopped, moved to the last checkpoint, and restarted with no time added. **[A2]**
9. Violation limit: 3 per attempt. More than 3 = the attempt ends. **[A6]**
10. Any part of the car detaching or falling off = violation, and the attempt ends immediately. **[A7]**

### Penalties (added to the total time)

General formula: `added_seconds = k * T_stage / N_cp`, where `N_cp` is the total number of checkpoints.

| Violation | k |
|---|---|
| Not passing a checkpoint | 0.5 |
| Car part detached | 0.2 |
| Leaving the track | 0.8 |

## 8. Competition rules: Autonomous Race Challenge (ARC)

Same as AR, with these differences:
- Only for teams that qualified for the final; it adds points in the final. Teams may choose to take it or not.
- **Checkpoints are obstacles to avoid.** Avoiding an obstacle counts as passing the checkpoint.
- Hitting or failing to avoid an obstacle = violation (car stopped, moved to last checkpoint, restarted with no time added).
- Off track if **50%** of the car is outside the track. **[A2]**
- Penalties: same as the AR table, but the first row becomes "hitting or not avoiding an obstacle" (k = 0.5); detached part k = 0.2; leaving the track k = 0.8.
- 3-violation limit per attempt, a detached part ends the attempt, 3 attempts, 10 minutes preparation: same as AR.

## 9. Match flow

### Test Day
- Open to all teams to test their cars; not scored. Schedule set by the organizer. **[A9]**
- Teams must follow all safety protocols, return equipment to its original position, and must not damage or modify competition equipment. Damage is reported to the organizer.

### Competition (elimination, final, challenge: same flow)
1. Teams are called in order and immediately get 10 minutes of preparation.
2. When preparation ends, the 5-minute attempt starts immediately: the team places the robot on the start line and **presses the start button**.
3. After the start button is pressed, **the team must not touch the robot at all**. Contact = attempt ends.
4. When the robot commits a violation, the referee announces it; the team must immediately **press the emergency button**, place the robot at the last checkpoint, and press start again.
5. When the referee announces the attempt is finished, the team must immediately press the emergency button.

## 10. Scoring

```
S_AR  = (1 + max((T_stage - T_total) / T_stage, 0)) * cp
S_ARC = ((1 + max((T_stage - T_total) / T_stage, 0)) * cp) / (2 * Tcp)
Total = S_AR * (1 + STP + S_ARC)
```

- `T_stage`: time limit given per attempt (5 minutes per attempt; assumed 300 s). **[A9]**
- `T_total`: total time taken, **including** penalty time added.
- `cp`: number of checkpoints successfully passed. `Tcp`: total checkpoints on the track.
- `STP = 0.25` if using Ackermann steering, otherwise 0.
- `S_ARC` is only computed if taking the challenge; if not taken, treat as 0. **[A8]**

Derived notes (interpretation, not guidebook text):
- Speed bonus: `(1 + max(...,0))` ranges from 1 to 2. If `T_total >= T_stage`, it equals 1, so the base score equals `cp`.
- Number of checkpoints matters more than speed; consistency matters more than going fast.

## 11. Other provisions

- Decisions of the jury and referees are final. Participants must follow instructions from jury, referees, and organizers.
- Protests: only by one team member, to the jury, politely, immediately after the match and before the next match begins. Not allowed during a match. Protests against referee decisions on violations and scoring are not accepted; protests are only for suspected violations of robot rules or procedure.
- Ethics: sportsmanship, cleanliness, order. The organizer is not responsible for loss or damage of participants' belongings.
- Matters not covered are decided by the organizer and jury (announced at the Technical Meeting or via official channels). The organizer may change the rules; changes are announced on the official registration platform and social media accounts.
- By registering, participants agree to the full guidebook.

## 12. Contacts

- Nindita: +62 896-4920-8124
- Yuke: +62 857-5577-3985

## 13. Ambiguities and possible typos

If a task depends on one of these points, **do not guess**. Ask the user or the organizer contact.

- **A1 – Differential vs Ackermann.** 5.3c requires a "differential drive system", but 5.3e gives a bonus for Ackermann steering. Hypothesis (unconfirmed): it means a differential in the drive system (wheels that can turn at different speeds), not differential steering. Confirm with the organizer before deciding the steering architecture.
- **A2 – Off-track threshold.** 30% in AR, 50% in ARC. May be a typo. For safety, design so the car never leaves the track at all (assume 30%).
- **A3 – Prize section.** The title mentions "Humanoid Penalty Kick" (copied from another category), and the amount is written as `Rp4.00.000,00` (possibly Rp4,000,000, etc.).
- **A4 – "KSTI 2025"** appears in participant requirement 7; likely left over from an older edition.
- **A5 – Sensor ban.** Infrared and line followers are clearly banned. The next sentence is confusing. The guidebook does not explicitly state whether cameras are allowed, even though Chapter I mentions computer vision. The status of LiDAR, ultrasonic, IMU, and encoders is also not stated. Confirm with the organizer before relying on any sensor other than a camera.
- **A6 – Penalties vs violation limit.** Penalties are added seconds, but more than 3 violations end the attempt. It is unclear whether a 4th violation is penalized, and how the score is calculated for an attempt that ends early.
- **A7 – Detached parts.** Counted as a violation with penalty (k = 0.2) and also ends the attempt immediately. Both are written.
- **A8 – Score if not finished / not taking ARC.** `T_total` for an unfinished attempt is not defined. Hypothesis: `T_total >= T_stage`, so the speed bonus is 0. If ARC is not taken, `S_ARC` is treated as 0.
- **A9 – Missing data.** Minimum elimination score, number of checkpoints `N_cp`/`Tcp`, track map (link only), test day schedule, batch 1/2 dates.
- **A11 – Start/finish color.** Written as red, then "not necessarily red". Do not hardcode the start/finish or checkpoint colors without confirmation; design color parameters to be configurable.
