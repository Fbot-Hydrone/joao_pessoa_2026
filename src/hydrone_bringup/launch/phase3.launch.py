"""
hydrone_bringup/launch/phase3.launch.py

AUTONOMY layer for Phase 3 (human-swarm interaction): take off, fly
approach_forward_m ahead, turn approach_turn_deg to the right to face the
operator, then move ONLY on the operator's gestures — land on each base the
operator guides it to, and fly home by itself after target_bases landings (or
on a return_home gesture).

  ros2 launch hydrone_bringup phase3_sim.launch.py     # sim, everything
  ros2 launch hydrone_bringup phase3.launch.py         # autonomy only

Gestures come in on /hydrone/vision/human_gesture (hydrone_msgs/HumanGesture).
In the simulator nobody is there to film, so type them instead, in a second
terminal:

  ros2 run hydrone_mission gesture_terminal

Vocabulary, frames and step behaviour: hydrone_mission/phase3/core.py and
docs/Phase 3 HRI Mission.md.

Consumes only /mavros/* and the gesture topic, so it is the same on the real
drone. Like every mission launch it publishes position setpoints: do not run it
alongside another mission.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# (name, default, type, description)
ARGS = [
    ("auto_start", "true", bool,
     "Take off and approach as soon as the FCU is ready. false: wait for "
     "/hydrone/phase3/start (gesture_terminal: iniciar) — the 'command from the "
     "team computer' the rules describe."),
    ("takeoff_alt", "1.5", float,
     "Height above the base it takes off from, m. The camera has to frame a "
     "standing operator from here."),
    ("approach_forward_m", "2.0", float, "Autonomous approach: metres straight ahead."),
    ("approach_turn_deg", "90.0", float, "Autonomous approach: then turn this much to the RIGHT."),
    ("step_m", "0.5", float, "One horizontal gesture moves this far, m."),
    ("step_z_m", "0.3", float, "One up/down gesture moves this far, m."),
    ("yaw_step_deg", "45.0", float, "One turn gesture turns this much."),
    ("target_bases", "6", int,
     "Landings before flying home by itself. 6 with one drone, 3 each with two."),
    ("min_confidence", "0.5", float, "Gestures below this confidence are ignored."),
]


def generate_launch_description():
    args = [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, _, h in ARGS]
    mission = Node(
        package="hydrone_mission", executable="phase3_hri_node", output="screen",
        emulate_tty=True,
        parameters=[{n: ParameterValue(LaunchConfiguration(n), value_type=t)
                     for n, _, t, _ in ARGS}],
    )
    return LaunchDescription(args + [mission])
