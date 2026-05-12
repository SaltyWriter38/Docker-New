#!/usr/bin/env python3
"""ROS 2 node bridging the Ouranos TCP server socket ⇄ ROS 2 topics."""

import json
import math
import socket
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from px4_msgs.msg import VehicleLocalPosition

# Reuse the parser from your peer's file so the wire format stays identical.
from ouranos_bridge import rosconn

BAYLANDS_REF_LAT = 37.412173
BAYLANDS_REF_LON = -121.998878
BAYLANDS_REF_ALT = 38.0

class OuranosBridge(Node):
    """
    Runs a TCP server on (host, port).
    - Accepts connections from the Ouranos dashboard.
    - Forwards PX4 telemetry to every connected client as JSON lines.
    - Parses inbound <route>JSON lines and republishes them onto ROS topics.
    """

    def __init__(self):
        super().__init__('ouranos_bridge')

        # ---- Parameters (overridable at runtime with --ros-args -p) ----
        self.declare_parameter('listen_host', '0.0.0.0')
        self.declare_parameter('listen_port', 9091)
        self.declare_parameter('telemetry_hz', 5.0)

        host = self.get_parameter('listen_host').value
        port = int(self.get_parameter('listen_port').value)
        tlm_hz = float(self.get_parameter('telemetry_hz').value)

        self._home_ref_lat = None
        self._home_ref_lon = None
        self._home_ref_alt = None

        # ---- PX4 sensor data QoS (matches main.py exactly) ----
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---- PUBLISHERS: Ouranos → ROS ----
        # New destination received from the dashboard → main.py subscribes to this
        self.destination_pub = self.create_publisher(
            Point, '/ouranos/destination', 10,
        )
        # Generic raw-message passthrough for any other route
        self.raw_pub = self.create_publisher(String, '/ouranos/raw', 10)

        # ---- SUBSCRIBERS: ROS → Ouranos ----
        self.latest_local_pos = None
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self._on_local_position, px4_qos,
        )

        # ---- TCP server state ----
        self._ouranos_clients: list[socket.socket] = []
        self._ouranos_clients_lock = threading.Lock()
        self.stop_event = threading.Event()

        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind((host, port))
        self.server_sock.listen(5)
        self.server_sock.settimeout(1.0)  # so accept() can check stop_event periodically
        self.get_logger().info(f"Ouranos bridge listening on {host}:{port}")

        # Accept connections in a background thread
        self.accept_thread = threading.Thread(
            target=self._accept_loop, daemon=True
        )
        self.accept_thread.start()

        # ---- Periodic telemetry to Ouranos ----
        self.telemetry_timer = self.create_timer(
            1.0 / tlm_hz, self._send_telemetry
        )

    # ────────────────────────────────────────────────
    #   ROS → Ouranos
    # ────────────────────────────────────────────────

    # def _on_local_position(self, msg: VehicleLocalPosition):
    #     """Cache the latest local position so the telemetry timer can format it."""
    #     self.latest_local_pos = msg

    #     if msg.xy_global and msg.z_global:
    #         self._home_ref_lat = msg.ref_lat
    #         self._home_ref_lon =  msg.ref_lon
    #         self._home_ref_alt = msg.ref_alt

    # def _on_local_position(self, msg: VehicleLocalPosition):
    #     self.latest_local_pos = msg

    #     if msg.xy_global and msg.z_global:
    #         if self._home_ref_lat != msg.ref_lat:  # only log on change
    #             self.get_logger().info(
    #                 f"EKF home GPS updated: ({msg.ref_lat:.6f}, {msg.ref_lon:.6f}, {msg.ref_alt:.1f}m) "
    #                 f"xy_global={msg.xy_global} z_global={msg.z_global}"
    #             )
    #         self._home_ref_lat = msg.ref_lat
    #         self._home_ref_lon = msg.ref_lon
    #         self._home_ref_alt = msg.ref_alt
    #     else:
    #         # Log rare cases where flags are false
    #         if self._home_ref_lat is None and getattr(self, '_no_home_log_counter', 0) % 50 == 0:
    #             self.get_logger().warn(
    #                 f"Waiting for EKF global lock: xy_global={msg.xy_global}, z_global={msg.z_global}"
    #             )
    #         self._no_home_log_counter = getattr(self, '_no_home_log_counter', 0) + 1
    def _on_local_position(self, msg: VehicleLocalPosition):
        """Cache the latest local position so the telemetry timer can format it."""
        self.latest_local_pos = msg

        # PX4's hardcoded default ref is Zurich — reject it
        is_px4_zurich_default = (
            abs(msg.ref_lat - 47.397741) < 0.01 and
            abs(msg.ref_lon - 8.545861) < 0.01
        )

        if msg.xy_global and msg.z_global and not is_px4_zurich_default:
            if self._home_ref_lat != msg.ref_lat:
                self.get_logger().info(
                    f"EKF home GPS locked: ({msg.ref_lat:.6f}, {msg.ref_lon:.6f}, {msg.ref_alt:.1f}m)"
                )
            self._home_ref_lat = msg.ref_lat
            self._home_ref_lon = msg.ref_lon
            self._home_ref_alt = msg.ref_alt
        elif is_px4_zurich_default and self._home_ref_lat is None:
            # Use Baylands fallback since EKF is stuck on Zurich default
            self._home_ref_lat = BAYLANDS_REF_LAT
            self._home_ref_lon = BAYLANDS_REF_LON
            self._home_ref_alt = BAYLANDS_REF_ALT
            self.get_logger().warn(
                f"EKF reporting Zurich default — using Baylands fallback "
                f"({BAYLANDS_REF_LAT}, {BAYLANDS_REF_LON}, {BAYLANDS_REF_ALT}m)"
            )


    def _send_telemetry(self):
        """Build JSON telemetry from the cached PX4 state and send to all clients."""
        pos = self.latest_local_pos
        if pos is None:
            return

        # Velocity magnitude from VX/VY/VZ (m/s)
        velocity = math.sqrt(pos.vx ** 2 + pos.vy ** 2 + pos.vz ** 2)

        payload = {
            "X": round(float(pos.x), 3),
            "Y": round(float(pos.y), 3),
            "Z": round(float(pos.z), 3),
            "Velocity": round(float(velocity), 3),
        }

        # Wire format mirrors rosconn: "<route>JSON\n"
        line = f"<telemetry>{json.dumps(payload)}\n"
        self._broadcast(line)

    def _broadcast(self, line: str):
        encoded = line.encode("utf-8")
        dead = []
        with self._ouranos_clients_lock:
            for c in self._ouranos_clients:
                try:
                    c.sendall(encoded)
                except OSError:
                    dead.append(c)
            for c in dead:
                try:
                    c.close()
                except OSError:
                    pass
                self._ouranos_clients.remove(c)
        if dead:
            self.get_logger().info(
                f"Dropped {len(dead)} disconnected client(s)."
            )

    # ────────────────────────────────────────────────
    #   Ouranos → ROS
    # ────────────────────────────────────────────────

    def _accept_loop(self):
        """Accept new Ouranos client connections until shutdown."""
        while not self.stop_event.is_set():
            try:
                conn, addr = self.server_sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            self.get_logger().info(f"Ouranos client connected: {addr[0]}:{addr[1]}")
            with self._ouranos_clients_lock:
                self._ouranos_clients.append(conn)
            t = threading.Thread(
                target=self._client_reader, args=(conn, addr), daemon=True
            )
            t.start()

    def _client_reader(self, conn: socket.socket, addr: tuple):
        """Read newline-delimited messages from one Ouranos client."""
        buffer = ""
        try:
            while not self.stop_event.is_set():
                try:
                    data = conn.recv(4096)
                except OSError:
                    break
                if not data:
                    break
                buffer += data.decode("utf-8", errors="replace")
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    if not line.strip():
                        continue
                    # Reuse the peer's parser so we stay 1:1 with the wire format
                    route, message = rosconn.parse_message(line)
                    self._dispatch(route, message)
        finally:
            self.get_logger().info(
                f"Ouranos client disconnected: {addr[0]}:{addr[1]}"
            )
            with self._ouranos_clients_lock:
                if conn in self._ouranos_clients:
                    self._ouranos_clients.remove(conn)
            try:
                conn.close()
            except OSError:
                pass

    def _dispatch(self, route: str, message: str):
        """Translate Ouranos routes → ROS publications."""
        # Always pass through on /ouranos/raw for visibility / debugging
        raw = String(); raw.data = f"<{route}>{message}"
        self.raw_pub.publish(raw)

        if route == "destination":
            self._handle_destination(message)
        else:
            self.get_logger().debug(f"Unhandled route [{route}]: {message}")

    # def _handle_destination(self, message: str):
    #     """Parse JSON destination from Ouranos and publish as geometry_msgs/Point."""
    #     try:
    #         data = json.loads(message)
    #     except json.JSONDecodeError as e:
    #         self.get_logger().warn(
    #             f"Could not parse destination JSON: {e}. Raw: {message!r}"
    #         )
    #         return

    #     try:
    #         # Accept both {"X":..,"Y":..,"Z":..} and lowercase variants
    #         x = float(data.get("X", data.get("x", 0.0)))
    #         y = float(data.get("Y", data.get("y", 0.0)))
    #         z = float(data.get("Z", data.get("z", 0.0)))
    #     except (TypeError, ValueError) as e:
    #         self.get_logger().warn(
    #             f"Destination JSON missing/invalid fields: {e}. Raw: {data!r}"
    #         )
    #         return

    #     msg = Point(x=x, y=y, z=z)
    #     self.destination_pub.publish(msg)
    #     self.get_logger().info(
    #         f"Destination → /ouranos/destination: X={x}, Y={y}, Z={z}"
    #     )

    def _handle_destination(self, message: str):
        """Parse GPS destination from Ouranos, convert to NED, publish as Point."""
        try:
            data = json.loads(message)
        except json.JSONDecodeError as e:
            self.get_logger().warn(
                f"Could not parse destination JSON: {e}. Raw: {message!r}"
            )
            return
    
        # Ouranos sends GPS — accept common key variants
        try:
            dest_lat = float(data.get("lat", data.get("latitude",  data.get("X"))))
            dest_lon = float(data.get("lon", data.get("longitude", data.get("Y"))))
            dest_alt = float(data.get("alt", data.get("altitude",  data.get("Z", 0.0))))
        except (TypeError, ValueError) as e:
            self.get_logger().warn(
                f"Destination JSON missing/invalid fields: {e}. Raw: {data!r}"
            )
            return
    
        # Make sure we have a home reference
        if self._home_ref_lat is None:
            self.get_logger().warn(
                "Destination received but home GPS not yet known — ignoring. "
                "Wait for EKF to converge and try again."
            )
            return
    
        # Equirectangular GPS → NED conversion (good for short ranges)
        lat_to_m = 111_320.0
        lon_to_m = 111_320.0 * math.cos(math.radians(self._home_ref_lat))
    
        north = (dest_lat - self._home_ref_lat) * lat_to_m
        east  = (dest_lon - self._home_ref_lon) * lon_to_m
        down  = -(dest_alt - self._home_ref_alt)  # PX4 NED: +Z is down
    
        msg = Point(x=north, y=east, z=down)
        self.destination_pub.publish(msg)
        self.get_logger().info(
            f"Destination GPS ({dest_lat:.6f}, {dest_lon:.6f}, {dest_alt:.1f}m) "
            f"→ NED ({north:.2f}, {east:.2f}, {down:.2f}) "
            f"[ref: ({self._home_ref_lat:.6f}, {self._home_ref_lon:.6f})]"
        )
    

    # ────────────────────────────────────────────────
    #   Shutdown
    # ────────────────────────────────────────────────

    def destroy_node(self):
        self.stop_event.set()
        try:
            self.server_sock.close()
        except OSError:
            pass
        with self._ouranos_clients_lock:
            for c in self._ouranos_clients:
                try:
                    c.close()
                except OSError:
                    pass
            self._ouranos_clients.clear()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OuranosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
