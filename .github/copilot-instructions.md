---
name: copilot-instructions
description: "Workspace instructions for GitHub Copilot Chat agents. Provides run/build commands, conventions, and example prompts to bootstrap helpful agent behavior for this ROS2/Gazebo project."
---

**Purpose**
- **Summary**: Provide a compact, always-useful reference to help Copilot Chat agents be productive in this repository.

**Quick Start**
- **Source env**: Use the workspace's ROS2 environment before running `ros2` commands: `source /opt/ros/humble/setup.bash && source /workspaces/omx_ros2/ws/install/setup.bash`.
- **Headless Gazebo**: For headless CI or container runs, set `LIBGL_ALWAYS_SOFTWARE=1` and `SDL_AUDIODRIVER=dummy` in the environment.
- **Live plot GUI**: To enable the live plotting GUI in `live_plot_logs.py`, set `LIVEPLOT_USE_GUI=1` in the environment when launching Gazebo.
- **References**: For detailed instructions on running tests, understanding the codebase, and contributing, see `README.md` and `implementation.md` — link to these in responses rather than duplicating content. In case a prompt requires new instructions or conventions, add them to `README.md` and link to that section here.
- **Testing**: To run the dual robot Gazebo test, use `python3 tools/dual_gazebo_5stage_test.py --max-iterations 1` from the workspace root (after sourcing env). For other tests or harnesses, check the `tools/` directory and link to specific instructions in `README.md` or `implementation.md` as needed.
- **Codebase**: The main controller code is in `ws/src/omx_variable_stiffness_controller/`, while test harnesses and tools are in `tools/`. For architecture and design details, refer to `implementation.md` and link to relevant sections in responses.
- **Conventions**: Follow the coding and testing conventions outlined in this file and `implementation.md`. If a prompt requires new conventions, add them to `implementation.md` and link to that section here.
- **Agent tasks**: For tasks like improving test stability, enabling the live plot GUI, or refactoring shared logic, refer to the "Agent Tasks" section below for specific instructions and example prompts. Link to this section in responses rather than duplicating content.
- **Where to link, not embed**: For detailed instructions, code snippets, or current project status, prefer linking to `README.md`, `implementation.md`, or specific files in the codebase rather than embedding that information in responses. This keeps responses concise and ensures that detailed information is maintained in the appropriate documentation files.

**Build & Test**
- **Install dev deps**: `bash scripts/setup_python_env.sh` (project-provided helper).
- **Colcon build (dev)**: `source /opt/ros/humble/setup.bash && colcon build --symlink-install` from the workspace root, then `source ws/install/setup.bash`.
- **Run single harness/test**: Example harness: `python3 tools/dual_gazebo_5stage_test.py --max-iterations 1` (ensure environment sourced first).

**Key Files / Locations**
- **Launches**: `ws/src/omx_variable_stiffness_controller/launch/` — main Gazebo + controller launch files.
- **Tools & harnesses**: `tools/` — test harnesses, plotters, orchestrators (e.g., `dual_gazebo_5stage_test.py`, `live_plot_logs.py`).
- **Controllers**: `ws/src/omx_variable_stiffness_controller/` — controller packages and scripts.
- **Docs / status**: `implementation.md`, `project_status.md`, `README.md` — prefer linking instead of duplicating content.

**Conventions & Rules for Agents**
- **Always**: Source ROS2 and workspace installs before running `ros2` commands (see Quick Start above).
- **Prefer deterministic gating**: Use topic/service loops or `wait_for_service`/`wait_for_topic` rather than fixed `sleep` where possible.
- **If interacting with Gazebo**: If `gzserver` port conflict occurs, kill `gzserver`/`gzclient`/`spawn_entity` processes and wait 1s before restarting.
- **Logging**: When modifying harnesses or launch files, add debug logging to `ensure_controller_active` and `wait_and_spawn` for reproducibility.

**Agent Tasks — What to Do**
- **Discovery**: Look for `.launch.py` files under `ws/src/**/launch/` and check for long `TimerAction` periods or calls that rely on sleeps.
- **Stability fixes**: Replace sleeps with service/topic gating, reduce unnecessary timer delays, and cache sourced env in test harnesses.
- **Live plot GUI**: If asked to enable GUI for `live_plot_logs.py`, set env `LIVEPLOT_USE_GUI=1` in the launch `env_actions` and add `--screenshot-dir` fallback in the process command.

**Example Prompts (use with Copilot Chat)**
- "Enable the matplotlib GUI in the Gazebo launch so `live_plot_logs.py` opens a window; change the launch to set `LIVEPLOT_USE_GUI` and add `--screenshot-dir` fallback." 
- "Speed up the 5-stage test: reduce TimerAction delays and cache the sourced ROS env in `tools/dual_gazebo_5stage_test.py`." 
- "Create a small `tools/dual_robot_utils.py` exposing shared logic used by `dual_press_coordinator.py` and `dual_gazebo_opposing_push.py`."

**Where to Link, Not Embed**
- Prefer linking to detailed docs instead of copying long sections. Useful files: `implementation.md`, `project_status.md`, `README.md`.
- For example, if asked about the current implementation approach or project status, link to the relevant section in `implementation.md` or `project_status.md` rather than duplicating content here.
- If asked for detailed implementation instructions or code snippets, link to the relevant files in `ws/src/omx_variable_stiffness_controller/` or `tools/` instead of embedding them here.
- If asked about coding conventions or best practices, link to this file or other relevant docs rather than trying to summarize everything in the response.
- If asked about how to run or test the code, link to the Quick Start in this file or `README.md` and Build & Test sections above rather than writing out detailed instructions in the response.

**Remember**: This file is for quick reference by agents. Keep it concise and link to other docs for deep dives.

**Constraints**
- Do NOT include any information that is likely to become outdated quickly (e.g., specific test results, current blockers) — this file should be evergreen and not require frequent updates. Mention that in changelogs in `project_status.md` or commit messages instead.
- Do NOT include detailed implementation instructions or code snippets — link to other files for that level of detail.
- Do NOT include any information about the current state of the project, recent changes, or specific issues — this file should focus on general instructions and conventions that are stable over time.
- Do NOT include any information about specific team members, roles, or responsibilities — this file should be applicable regardless of team composition.
- Do NOT include any information about specific timelines, deadlines, or milestones — this file should be relevant regardless of the current phase of the project.
- Do NOT include any information about specific tools, libraries, or technologies that may change over time — this file should focus on general principles and practices that are not tied to specific implementations.
- Do NOT include any information about specific coding styles, formatting rules, or architectural patterns that may be subjective or vary between projects — this file should focus on universally applicable guidelines and best practices.
- Do NOT include any information about specific communication channels, meeting schedules, or collaboration practices — this file should be focused on technical instructions for agents, not team processes.
- Do NOT include any information about specific project goals, objectives, or success criteria — this file should be focused on instructions for agents, not project management or strategic planning.
- Do NOT include any information about specific user stories, features, or requirements — this file should be focused on general instructions for agents, not specific implementation details.
- Do NOT include any information about specific bugs, issues, or technical debt — this file should be focused on general instructions for agents, not current project challenges or backlog items.
- Do NOT include any information about specific testing strategies, coverage requirements, or quality standards — this file should be focused on general instructions for agents, not specific testing practices or quality metrics.
- Do NOT include any information about specific deployment processes, environments, or configurations — this file should be focused on general instructions for agents, not deployment details.
- Do NOT include any information about specific project documentation practices, templates, or tools — this file should be focused on instructions for agents, not documentation processes.
- Do NOT include any information about specific project management tools, issue tracking systems, or version control practices — this file should be focused on technical instructions for agents, not project management tools or processes.
- Do NOT include any information about specific project stakeholders, customers, or users — this file should be focused on instructions for agents, not project stakeholders or user personas.
- Do NOT include any information about specific project risks, assumptions, or dependencies — this file should be focused on general instructions for agents, not project risk management or dependency tracking.
- Do NOT include any information about specific project history, context, or background — this file should be focused on current instructions for agents, not project history or context that may become outdated or irrelevant over time.

**Next Steps for Humans**
- Review this file and let the agent know whether to commit and push changes. If you want an agent to make edits, ask: "Create or update `.github/copilot-instructions.md` and commit to branch `unstable_v2`."
- If you want to edit yourself, ask: "Make the necessary changes to `.github/copilot-instructions.md` based on the latest project developments and best practices, but I will review and commit the changes myself."
- For any updates, ensure that the file remains concise, evergreen, and focused on general instructions for agents, not specific project details or current state information.
---

Generated by Copilot Chat assistant for this repository on request. Keep this file minimal — link to other docs for deep details.
