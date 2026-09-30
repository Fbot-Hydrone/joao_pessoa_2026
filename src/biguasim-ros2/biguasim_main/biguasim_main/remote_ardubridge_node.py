#!/usr/bin/env python3
"""ArduPilot SITL against a BiguaSim world that lives in another process.

This is :mod:`ardubridge_node` with the simulator taken out from under it. The
local node owns a ``biguasim.make()`` environment, which means it owns the UE5
binary, which means exactly one such node can exist. A served world moves that
ownership to the world process, so several vehicles -- from several machines --
can fly in one simulation and see each other in it.

All of the flying lives in :class:`biguasim.ardubridge.RemoteArduRunner`, which
is also what ``tools/ardu_pilot.py`` drives with no ROS at all. This node is
the same runner with ROS publishers attached; keeping the two on one
implementation is the point, since a bug found flying by hand is then already
fixed here.

**Run this on the machine running the world.** ArduPilot's JSON backend is a
blocking lockstep handshake, so a round trip across a network would cap the
flight loop at ``1/RTT``. Co-located it costs nothing, and MAVLink -- which was
built for telemetry radios -- is what crosses the network instead. See the
runner's module docstring for the full argument.
"""

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from biguasim.ardubridge import VEHICLE_REGISTRY, RemoteArduRunner

from biguasim_main.ardubridge_node import SPINNING_PHASES
from biguasim_main.interface import BiguaSimInterface

GPS_ORIGIN = (33.810313, -118.393867)
DEFAULT_PACKAGE_NAME = "Competition"
DEFAULT_WORLD = "CompetionMap"


class _RemoteDynamics:
    """What :class:`BiguaSimInterface` needs to know about a dynamics model.

    ``create_sensor_list`` reads ``batch_size`` and ``control_abstraction`` off
    ``env._dynamics_dict`` to size the command vector. Here the model lives in
    the world process, so the two facts are supplied directly rather than
    building a second copy locally -- which would load torch and claim a GPU
    for no reason at all.
    """

    def __init__(self, control_abstraction, batch_size=1):
        self.control_abstraction = control_abstraction
        self.batch_size = batch_size


class _RemoteEnv:
    """Stands in for the environment the interface expects to be handed."""

    def __init__(self, dynamics_dict):
        self._dynamics_dict = dynamics_dict


class RemoteArduBridgeNode(Node):
    """Publishes a remotely-flown agent's sensors onto ROS topics."""

    def __init__(self):
        super().__init__('remote_ardubridge_node')

        self.declare_parameter('params_file', '')
        self.declare_parameter('world_address', '127.0.0.1')
        self.declare_parameter('world_port', 8770)
        self.declare_parameter('instance', 0)
        self.declare_parameter('stream_backlog', 256)
        self.declare_parameter('report_every', 0)
        self.declare_parameter('phase', 1)

        # Phases 3 and 4 simulate a Livox Mid-360 by spinning a depth camera
        # between steps (see ardubridge_node._spin_setup). That is impossible
        # from here: the sensor lives in the world process, RemoteArduRunner
        # holds only a socket to it, and the protocol has no command to rotate
        # anything. Saying so beats a lidar that quietly never sweeps -- the
        # mimic degrades to a single wedge, which looks like a broken sensor.
        phase = int(self.get_parameter('phase').value)
        if phase in SPINNING_PHASES:
            self.get_logger().warn(
                f"phase {phase} expects a spinning sensor, which a REMOTE "
                "world cannot provide: the world process owns the sensor and "
                "there is no rotate command in the protocol. The lidar mimic "
                "will publish a single fixed wedge. Run the simulator locally "
                "(drop --world) for a full sweep.")

        file_path = self.get_parameter('params_file').get_parameter_value().string_value
        if not file_path:
            raise RuntimeError("params_file nao encontrado!")

        self.interface = BiguaSimInterface(file_path, init=False, node=self)
        scenario_cfg = self.interface.scenario
        agent_cfg = scenario_cfg['agents'][0]
        agent_type = agent_cfg['agent_type']
        agent = agent_cfg['agent_name']

        match = next((k for k in VEHICLE_REGISTRY if k.upper() == agent_type.upper()), None)
        if match is None:
            raise RuntimeError("Veiculo desconhecido: {}".format(agent_type))
        profile = VEHICLE_REGISTRY[match]

        # The YAML's sensors go on top of the ones ArduPilot needs. The
        # ArduPilot set is built by the runner at connect time, from the tick
        # rate the world reports -- not from ticks_per_sec in this YAML, which
        # describes a simulator this node is not running and can disagree.
        self.runner = RemoteArduRunner(
            profile,
            package_name=scenario_cfg.get('package_name', DEFAULT_PACKAGE_NAME),
            world=scenario_cfg.get('world', DEFAULT_WORLD),
            agent=agent,
            address=self._param('world_address'),
            port=self._param('world_port'),
            instance=self._param('instance'),
            extra_sensors=agent_cfg.get('sensors', []),
            location=tuple(agent_cfg.get('location', [0, 0, 5])),
            rotation=tuple(agent_cfg.get('rotation', [0, 0, 0])),
            dynamics=agent_cfg.get('dynamics', {}),
            gps_origin=GPS_ORIGIN,
            stream_backlog=self._param('stream_backlog'),
            client_id="ardubridge-{}".format(agent),
        )

        info = self.runner.connect()
        self.get_logger().info(
            "conectado ao mundo | tick {} | {} Hz | input delay {}".format(
                info.get('tick'), self.runner.ticks_per_sec,
                info.get('input_delay')))

        # ros_publish is the YAML's call, per sensor, and can only be applied
        # once the runner has settled what the sensor list actually is. The
        # ones ArduPilot added default to off, so the EKF's inputs do not
        # silently become four extra topics nobody asked for.
        wanted = {spec.get('sensor_name', spec['sensor_type']): spec.get('ros_publish', False)
                  for spec in agent_cfg.get('sensors', [])}
        for spec in self.runner.sensors:
            spec['ros_publish'] = wanted.get(
                spec.get('sensor_name', spec['sensor_type']), False)

        self._wire_ros(agent, agent_type, self.runner.sensors, profile)

        self.get_logger().info(
            "aguardando SITL em udp/{} | MAVLink em tcp/{}".format(
                self.runner.fdm_port, self.runner.mavlink_port))
        self.runner.start()
        self.get_logger().info("RemoteArduBridge pronto: {} | {} sensores".format(
            agent_type, len(self.interface.sensors)))

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._fly, daemon=True)
        self._thread.start()

    def _param(self, name):
        return self.get_parameter(name).value

    # ------------------------------------------------------------ ROS wiring

    def _wire_ros(self, agent, agent_type, sensors, profile):
        """Build the publishers the interface will feed.

        ``create_sensor_list`` keys off the ``-id0`` the environment appends to
        every agent name, and splits it back apart to index the state dict. The
        world renames identically, so the published names line up.
        """
        self.interface.env = _RemoteEnv({
            agent: _RemoteDynamics(profile.control_abstraction),
        })
        self.interface.scenario = {
            'agents': [{
                'agent_name': agent + '-id0',
                'agent_type': agent_type,
                'publish_commands': False,
                'sensors': sensors,
            }],
        }
        self.interface.initialized = True
        self.interface.sensors = self.interface.create_sensor_list()

        for sensor in self.interface.sensors:
            topic = "{}/{}".format(sensor.agent_name, sensor.name)
            sensor.publisher = self.create_publisher(sensor.message_type, topic, 10)
            self.get_logger().info("Publisher: {}".format(topic))

        # Direct control, for anything not going through ArduPilot. Unlike the
        # local node -- where this topic writes into a dict nothing reads -- it
        # reaches the world here. Latest wins, so publishing on it while SITL
        # is flying will fight the flight controller; it is meant for driving
        # the vehicle with SITL stopped.
        self.create_subscription(
            Float64MultiArray,
            "{}/command_control".format(agent.replace('-', '_')),
            lambda msg: self.runner.world.stream_control(agent, list(msg.data)),
            10)

    # ----------------------------------------------------------------- loop

    def _fly(self):
        report = self._param('report_every')
        try:
            self.runner.run(on_frame=self._publish,
                            should_stop=self._stop.is_set,
                            report_every=report)
        except Exception as exc:                                  # noqa: BLE001
            self.get_logger().error("bridge parou: {}: {}".format(
                type(exc).__name__, exc))

    def _publish(self, frame, sim_time):
        try:
            self.interface.publish_sensor_data(
                {self.runner.agent: [frame], 't': sim_time})
        except Exception as exc:                                  # noqa: BLE001
            self.get_logger().warn("Erro publicando sensores: {}".format(exc))

    def destroy_node(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self.runner.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RemoteArduBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
