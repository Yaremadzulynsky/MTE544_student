# Lab 1 submission audit

Submit one existing report PDF and `submission/lab1_code.zip` to the appropriate LEARN Dropbox; one submission per group.

## Verified in this checkout

- Fetched GitHub and ran `git pull --ff-only` on `alison-lab-1-plots`: already up to date (7b6de16). Local work preserved.
- ROS message imports, `/cmd_vel` publisher, sensor subscriptions, timestamped callbacks, CSV logging, quaternion-to-yaw conversion, and circle/spiral/accelerated-line implementations are present.
- BEST_EFFORT, VOLATILE, KEEP_LAST(10) subscriptions are configured. Confirm actual publisher QoS with `ros2 topic info /odom --verbose` on the target robot/simulator.
- Nine October 9 in-lab CSVs contain samples with nondecreasing timestamps. Circle: 412 odometry / 677 IMU / 150 scans (~21 s); spiral: 917 / 1502 / 336 (~46.5 s); line: 249 / 404 / 89 (~12.6 s).
- Successfully regenerated all 12 required sensor figures from these logs. Odometry includes XY trajectory and X/Y/yaw versus time; IMU includes ax/ay/wz versus time; each laser plot converts one scan to Cartesian coordinates, excluding invalid/nonpositive ranges while preserving beam indices. Plotting code supplies titles, units, legends, differentiated series, and grids.
- In-lab map PGM and YAML are present, along with a PNG preview. The YAML references the included `map.pgm`.
- Python syntax checked. Motion formula checks performed without ROS; hardware execution and QoS were not retested here.
- ZIP includes source, all available CSVs (including earlier runs), saved maps, and plots. Rebuild after further edits with `python3 build_submission.py`.

## Report PDF reviewed (October 18)

Reviewed the user-provided `Lab 1 Report.pdf` visually and using extracted text.
- PASS: exactly three pages: cover plus two content pages.
- PASS: three members' family/first names and IDs, Robot 4, all three motion composites, map image, captions and interpretation/quality discussion.
- PASS: required plot types, titles, axis labels/units, legends, differentiated series and grids are present. The upper composites have very small labels at final page size; enlarge if practical.
- CHECK: cover says Group 9, not station number. Add the actual station number explicitly.
- FIX: circular-motion text says radius 1 m and speed 0.1 m/s; current code and logged trajectory support radius 0.5 m and speed about 0.2 m/s (about 1 m diameter, median IMU yaw rate 0.395 rad/s).
- FIX: yaw wraps at -pi and +pi, not -2pi and +2pi.
- FIX: spiral text says initial radius 0.5 m and speed 0.1 m/s; current code uses 0.3 m and 0.2 m/s, with radius increasing 0.01 m/s. Logs show growing loops and declining turn rate consistent with those tuned values; CSVs do not independently record command settings.
- FIX: line acceleration is described as 0.02 m/s. Acceleration units must be m/s²; current code uses 0.04 m/s² capped at 0.2 m/s after 5 s. Describe an initial curved position-time segment followed by approximately linear position-time behavior, rather than implying acceleration throughout.
- FIX: constant wheel speeds in a turn do not imply zero acceleration: turning produces lateral centripetal acceleration. Circle IMU median lateral acceleration is about 0.045 m/s²; the ideal value for v=0.2 m/s and r=0.5 m is 0.08 m/s². Sensor bias/noise and actual motion can affect agreement.
- QUALIFY: low battery causing unstable IMU supply is an untested hypothesis, not established by these data. Sparse single-scan structure also reflects missing returns and scene geometry, not necessarily sensor noise alone. Map objects cannot be identified confidently as people or robots from occupancy cells alone.

The original PDF was not modified. Correct these statements in the source document and export a replacement PDF before submission.

## Requires group confirmation

- Pre-lab submission was made at least 24 hours before the lab.
- TA checked robot operation and all three motion demonstrations.
- Target-machine QoS was inspected; code and data removed from the shared lab PC as instructed.

These historical/in-lab requirements cannot be established from this checkout alone.
