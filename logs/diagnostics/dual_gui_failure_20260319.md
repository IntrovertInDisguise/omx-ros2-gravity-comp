Dual GUI Launch Failure — diagnostic summary
Timestamp: 2026-03-19

Summary
-------
Repeated attempts to run `dual_gazebo_variable_stiffness.launch.py` with `gui:=true` in this devcontainer resulted in an orderly shutdown triggered by a SIGINT/SIGTERM sent to the ROS process tree. The controller manager logs show a clean shutdown (controllers deactivated), not a crash or Python traceback.

Key evidence
------------
- `gzserver` log: normal controller messages and trajectory debug until shutdown.
- Final lines (controller manager):
  - `signal_handler(SIGINT/SIGTERM)`
  - `Shutdown request received` / `Shutting down all controllers`
  - Controller deactivation and hardware shutdown successful.

Likely causes
-------------
- The process received an external SIGINT/SIGTERM (environment, user, or container watchdog). The logs do not show an uncaught exception.
- In devcontainer environments GUI processes frequently fail due to graphics/Wayland/X11 limitations or resource constraints; the host or container tooling may terminate GUI-backed processes.

Recommended fixes / next steps
-----------------------------
1. Retry headless (`gui:=true`) inside the container — this is more reliable and recommended for automated tests.
2. Collect the full `/root/.ros/log/<most_recent>/` directory after any future failure and attach it for deeper analysis.

Actions taken here
------------------
- Added this diagnostic file.
- Attempted dual-GUI runs twice; both ended with an external SIGINT/SIGTERM. Single-GUI run succeeded earlier (CSV logs moved to workspace `logs/variable_stiffness/single_gazebo/...`).

