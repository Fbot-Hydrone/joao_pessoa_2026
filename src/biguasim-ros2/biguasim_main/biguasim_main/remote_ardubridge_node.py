#!/usr/bin/env python3
"""ArduPilot SITL against a BiguaSim world that lives in another process.

This is :mod:`ardubridge_node` with the simulator taken out from under it.

The local node owns a ``biguasim.make()`` environment, which means it owns the
UE5 binary, which means exactly one such node can exist. A served world moves
that ownership to the world process, so any number of clients can fly in the
same simulation at once -- and, unlike the local node, each one can *see* the
others, because there is only one set of physics.

Why this can work at all
------------------------

ArduPilot's JSON backend is a lockstep protocol: ``JSON::update()`` is
``output_servos()`` followed by a blocking ``recv_fdm()``, one frame in flight,
and the servos for frame N+1 are computed from the state at frame N. Nothing
about that can be pipelined -- it is a control dependency, not a protocol
choice -- so if the state ArduPilot needs had to be *fetched* per frame, the
loop rate would be pinned at ``1/RTT``.

It doesn't have to be fetched. The world publishes state every tick on a PUB
socket, streamed and pipelined, so the states are already arriving; ``recv_fdm``
never actually waits. The two directions decouple:

===============  ===========================  ==========================
path             shape                        what latency costs
===============  ===========================  ==========================
state            world -> client, streamed    a constant offset, no rate
control          client -> world, no reply    delay before it takes effect
===============  ===========================  ==========================

So ArduPilot's loop runs at *the world's tick rate*, not at ``1/RTT``, and
network latency turns into plant delay rather than a rate cap -- the same thing
a real ESC contributes, and a thing a detuned rate controller handles. At 200
ticks/sec a LAN adds well under a tick; a 45 ms link adds about nine, which is
worth measuring before trusting.

Two consequences shape the code below:

* Control is sent with :meth:`~biguasim.client.remote.RemoteWorld.stream_control`,
  never ``set_control``. The latter waits for an ack, which would put a full
  round trip back into the loop and undo the whole argument.
* The world's ``t`` is the clock. ArduPilot derives its timestep from the
  timestamps we send (``deltat = state.timestamp_s - last_timestamp_s``), so
  the number has to be the world's own elapsed time, not something this node
  counts. If we skip a tick, ArduPilot is told the truth and adjusts.
"""

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from biguasim.ardubridge.bridge import ArduPilotBridge
from biguasim.ardubridge import ArduBiguaSimRunner, VEHICLE_REGISTRY
from biguasim.client.remote import RemoteWorld
from biguasim.server import protocol as proto

from biguasim_main.interface import BiguaSimInterface

GPS_ORIGIN = (33.810313, -118.393867)
DEFAULT_PACKAGE_NAME = "Competition"
DEFAULT_WORLD = "CompetionMap"

#: Sensors ArduPilot's EKF is fed from. A tick missing any of them cannot be
#: turned into an FDM packet, so it is skipped rather than half-sent.
REQUIRED_SENSORS = ("IMUSensor", "LocationSensor",
                    "VelocitySensor", "DynamicsSensor")


class _RemoteDynamics:
    """What :class:`BiguaSimInterface` needs to know about a dynamics model.

    ``create_sensor_list`` reads ``batch_size`` and ``control_abstraction`` off
    ``env._dynamics_dict`` to size the command vector. Here the model lives in
    the world process, so the two facts are supplied directly rather than
    building a second copy of it locally -- which would load torch and pick a
    GPU for no reason.
    """

    def __init__(self, control_abstraction, batch_size=1):
        self.control_abstraction = control_abstraction
        self.batch_size = batch_size


class _RemoteEnv:
    """Stands in for the environment the interface expects to be handed."""

    def __init__(self, dynamics_dict):
        self._dynamics_dict = dynamics_dict


class RemoteArduBridgeNode(Node):
    """Bridges ArduPilot SITL to an agent in a world running somewhere else."""

    def __init__(self):
        super().__init__('remote_ardubridge_node')

        self.declare_parameter('params_file', '')
        self.declare_parameter('world_address', '127.0.0.1')
        self.declare_parameter('world_port', 8770)
        # Generous on purpose. The bridge blocks up to 10 ms in receive_pwm,
        # so a world ticking faster than SITL consumes builds a short backlog;
        # dropping it would show up as skipped ticks rather than as lag.
        self.declare_parameter('stream_backlog', 256)
        self.declare_parameter('report_every', 0)

        file_path = self.get_parameter('params_file').get_parameter_value().string_value
        if not file_path:
            raise RuntimeError("params_file nao encontrado!")

        self.interface = BiguaSimInterface(file_path, init=False, node=self)
        scenario_cfg = self.interface.scenario
        agent_cfg = scenario_cfg['agents'][0]
        agent_type = agent_cfg['agent_type']
        self._agent = agent_cfg['agent_name']

        match = next((k for k in VEHICLE_REGISTRY if k.upper() == agent_type.upper()), None)
        if match is None:
            raise RuntimeError("Veiculo desconhecido: {}".format(agent_type))
        self.profile = VEHICLE_REGISTRY[match]

        sensors = self._merge_sensors(scenario_cfg, agent_cfg)

        # ------------------------------------------------------------ connect
        address = self.get_parameter('world_address').get_parameter_value().string_value
        port = self.get_parameter('world_port').get_parameter_value().integer_value
        backlog = self.get_parameter('stream_backlog').get_parameter_value().integer_value
        self._report_every = self.get_parameter('report_every').get_parameter_value().integer_value

        # The build id is checked against the world's on connect. It is derived
        # from the world package, so it has to be the same package this agent
        # is being spawned into or the world refuses us -- which is the point.
        self.world = RemoteWorld(
            address=address, port=port, stream_backlog=backlog,
            client_id="ardubridge-{}".format(self._agent),
            scenario_cfg={
                "package_name": scenario_cfg.get('package_name', DEFAULT_PACKAGE_NAME),
                "world": scenario_cfg.get('world', DEFAULT_WORLD),
            },
        )
        info = self.world.connect()
        self.get_logger().info(
            "conectado a {}:{} | tick {} | input delay {}".format(
                address, port, info.get('tick'), info.get('input_delay')))

        self.world.spawn_agent(
            self._agent,
            agent_type,
            location=tuple(agent_cfg.get('location', [0, 0, 5])),
            rotation=tuple(agent_cfg.get('rotation', [0, 0, 0])),
            control_abstraction=self.profile.control_abstraction,
            sensors=sensors,
            dynamics=agent_cfg.get('dynamics', {}),
        )
        # If this node dies mid-flight the world would otherwise hold the last
        # throttle it was given forever. Zero is the only safe standing order.
        self.world.set_control_defaults(
            self._agent, [0.0] * self.profile.num_motors)

        self.world.watch_state()
        for spec in sensors:
            self.world.watch_sensor(
                self._agent, spec.get('sensor_name', spec['sensor_type']))

        self._await_spawn()

        # -------------------------------------------------------------- ROS
        # create_sensor_list keys off the '-id0' the environment appends to
        # every agent name, and splits it back apart to index the state dict.
        # The world does the same renaming, so the published names line up.
        self.interface.env = _RemoteEnv({
            self._agent: _RemoteDynamics(self.profile.control_abstraction),
        })
        self.interface.scenario = {
            'agents': [{
                'agent_name': self._agent + '-id0',
                'agent_type': agent_type,
                'publish_commands': agent_cfg.get('publish_commands', False),
                'sensors': sensors,
            }],
        }
        self.interface.initialized = True
        self.interface.sensors = self.interface.create_sensor_list()

        self._sensor_publisher_create()
        self._control_subscribers_create()

        self.bridge = ArduPilotBridge(self.profile, gps_origin=GPS_ORIGIN)

        self.get_logger().info("RemoteArduBridge pronto: {} | {} sensores".format(
            agent_type, len(self.interface.sensors)))

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run_bridge, daemon=True)
        self._thread.start()

    def _await_spawn(self, timeout=15.0):
        """Confirm the agent really appeared before flying anything at it.

        Whether a spawn works is not knowable when it is accepted -- it runs in
        the world several ticks later -- so refusals come back asynchronously
        on the request socket. Without this check a vehicle type the world
        cannot build looks exactly like a healthy node that never receives any
        sensor data, which is a miserable thing to debug at a competition.

        Raises:
            RuntimeError: If the world refused, or the agent never showed up.
        """
        state_topic = proto.TOPIC_STATE.decode()
        deadline = time.time() + timeout
        while time.time() < deadline:
            for failure in self.world.failures():
                raise RuntimeError("o mundo recusou o spawn: {}".format(
                    failure.get("error")))
            got = self.world.recv(timeout=0.5)
            if got is None:
                continue
            topic, message = got
            if topic == state_topic and self._agent in message.get("agents", {}):
                self.get_logger().info("agente {} presente no tick {}".format(
                    self._agent, message["tick"]))
                return
        raise RuntimeError(
            "agente {} nao apareceu no mundo em {}s".format(self._agent, timeout))

    # ------------------------------------------------------------- scenario

    def _merge_sensors(self, scenario_cfg, agent_cfg):
        """ArduPilot's required sensors, plus whatever the YAML adds.

        Same rule as the local node: ``build_scenario`` decides what the EKF
        needs, and the YAML may add anything else it wants on a ROS topic. A
        sensor named in both is taken from ``build_scenario``, because getting
        the EKF's inputs wrong is a far worse failure than a camera at the
        wrong rate.
        """
        ardu = ArduBiguaSimRunner.build_scenario(
            self.profile,
            package_name=scenario_cfg.get('package_name', DEFAULT_PACKAGE_NAME),
            world=scenario_cfg.get('world', DEFAULT_WORLD),
            agent_name=agent_cfg['agent_name'],
            ticks_per_sec=scenario_cfg.get('ticks_per_sec', 200),
        )
        sensors = list(ardu['agents'][0]['sensors'])
        have = {s['sensor_type'] for s in sensors}
        for spec in agent_cfg.get('sensors', []):
            if spec['sensor_type'] not in have:
                sensors.append(dict(spec))

        # ros_publish is the YAML's call for every sensor, including the ones
        # build_scenario added -- those default to off, so the EKF's inputs do
        # not silently become four extra ROS topics nobody asked for.
        wanted = {}
        for spec in agent_cfg.get('sensors', []):
            wanted[spec.get('sensor_name', spec['sensor_type'])] = spec.get('ros_publish', False)
        for spec in sensors:
            name = spec.get('sensor_name', spec['sensor_type'])
            spec['ros_publish'] = wanted.get(name, False)
        return sensors

    # ----------------------------------------------------------------- loop

    def _run_bridge(self):
        """Turn the world's tick stream into FDM packets, and PWM into control.

        The stream carries one state message per tick followed by that tick's
        sensor messages, in order, so the *next* state message is the marker
        that the previous tick is complete. That is why the frame is emitted on
        the boundary rather than after a timeout -- no guessing, and no waiting
        for sensors that were never coming.
        """
        self.bridge.bind()
        self.get_logger().info("Bridge UDP iniciada, aguardando ArduPilot...")

        state_topic = proto.TOPIC_STATE.decode()
        prefix = "{}/{}/".format(proto.TOPIC_SENSOR, self._agent)

        frame = {}
        tick = None
        sim_time = 0.0
        emitted = skipped = 0

        try:
            while not self._stop.is_set():
                got = self.world.recv(timeout=1.0)
                if got is None:
                    continue
                topic, message = got

                if topic == state_topic:
                    if tick is not None:
                        if all(s in frame for s in REQUIRED_SENSORS):
                            self._emit(frame, sim_time)
                            emitted += 1
                        else:
                            # A tick lost to the receive high-water mark. Told
                            # to nobody: ArduPilot sees a larger deltat next
                            # time and adjusts its own frame time to match.
                            skipped += 1
                        if self._report_every and \
                                (emitted + skipped) % self._report_every == 0:
                            self.get_logger().info(
                                "tick {}  frames {}  skipped {}".format(
                                    tick, emitted, skipped))
                    tick = message['tick']
                    sim_time = message['time']
                    frame = {}

                elif topic.startswith(prefix) and message.get('tick') == tick:
                    frame[message['sensor']] = message['data']

        except Exception as exc:                                  # noqa: BLE001
            self.get_logger().error("bridge parou: {}: {}".format(
                type(exc).__name__, exc))
        finally:
            self.bridge.close()

    def _emit(self, frame, sim_time):
        """One tick: tell SITL where it is, take its servos, publish to ROS.

        Order matters. ArduPilot is blocked in ``recv_fdm`` waiting for state,
        so sending it first is what lets SITL run at all; only then is there a
        PWM packet to collect. On the very first pass the bridge has not yet
        learned SITL's address and ``send_state`` is a no-op -- ``receive_pwm``
        learns it from the servo packet ArduPilot sends unprompted.
        """
        json_state = self.bridge.build_json_state(frame, sim_time)
        if json_state:
            self.bridge.send_state(json_state)

        pwm_frame, pwm = self.bridge.receive_pwm()
        if pwm_frame is not None:
            self.world.stream_control(
                self._agent, self.bridge.pwm_to_motor_cmds(pwm, pwm_frame))

        try:
            self.interface.publish_sensor_data(
                {self._agent: [frame], 't': sim_time})
        except Exception as exc:                                  # noqa: BLE001
            self.get_logger().warn("Erro publicando sensores: {}".format(exc))

    # ------------------------------------------------------------ ROS wiring

    def _sensor_publisher_create(self):
        for sensor in self.interface.sensors:
            topic = "{}/{}".format(sensor.agent_name, sensor.name)
            sensor.publisher = self.create_publisher(sensor.message_type, topic, 10)
            self.get_logger().info("Publisher: {}".format(topic))

    def _control_subscribers_create(self):
        """Direct control, for anything not going through ArduPilot.

        Unlike the local node -- where this topic writes into a command dict
        nothing reads -- here it reaches the world. Whatever arrives last wins,
        so publishing on it while SITL is flying will fight the flight
        controller. It is meant for driving the vehicle with SITL stopped.
        """
        for ag in self.interface.scenario['agents']:
            name = ag['agent_name'].replace('-', '_')
            self.create_subscription(
                Float64MultiArray,
                "{}/command_control".format(name),
                lambda msg, a=self._agent: self.world.stream_control(a, list(msg.data)),
                10,
            )

    def destroy_node(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        try:
            self.world.close()
        except Exception:                                          # noqa: BLE001
            pass
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
