#!/usr/bin/env python3
"""ROS 2 node bridging the Ouranos TCP server socket ⇄ ROS 2 topics.

The bridge is a TCP *client* of the Go central server (Ouranos):
it dials out to (server_host, server_port) and reconnects with backoff
if the connection drops or the server isn't up yet.
"""

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

# BAYLANDS_REF_LAT = 37.412173
# BAYLANDS_REF_LON = -121.998878
# BAYLANDS_REF_ALT = 38.0


class OuranosBridge(Node):
    """
    TCP client of the Ouranos Go central server.
    - Dials (server_host, server_port) with reconnect-on-failure.
    - Forwards PX4 telemetry to the server as <telemetry>JSON\\n lines.
    - Parses inbound <route>JSON\\n lines and republishes them onto ROS topics.
    """

    def __init__(self):
        super().__init__('ouranos_bridge')

        # ---- Parameters (overridable with --ros-args -p) ----
        self.declare_parameter('server_host', 'host.docker.internal')
        self.declare_parameter('server_port', 9091)        # RegionalClient Port
        self.declare_parameter('reconnect_min_s', 1.0)
        self.declare_parameter('reconnect_max_s', 15.0)
        self.declare_parameter('telemetry_hz', 5.0)
        self.declare_parameter('notam_default_alt_min', 0.0)
        self.declare_parameter('notam_default_alt_max', 1000.0)
        self.declare_parameter('listen_host', '0.0.0.0')
        self.declare_parameter('listen_port', 9091)

        self._server_host           = self.get_parameter('server_host').value
        self._server_port           = int(self.get_parameter('server_port').value)
        self._reconnect_min         = float(self.get_parameter('reconnect_min_s').value)
        self._reconnect_max         = float(self.get_parameter('reconnect_max_s').value)
        tlm_hz                      = float(self.get_parameter('telemetry_hz').value)
        self._notam_default_alt_min = float(self.get_parameter('notam_default_alt_min').value)
        self._notam_default_alt_max = float(self.get_parameter('notam_default_alt_max').value)
        self._listen_host           = self.get_parameter('listen_host').value
        self._listen_port           = int(self.get_parameter('listen_port').value)
        
        self._has_published_destination = False

        # ---- QoS profiles (must match main.py exactly) ----
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        notam_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )

        # ---- Publishers: Ouranos → ROS ----
        self.destination_pub = self.create_publisher(Point, '/ouranos/destination', 10)
        self.notam_pub       = self.create_publisher(String, '/ouranos/notam', notam_qos)
        self.raw_pub         = self.create_publisher(String, '/ouranos/raw', 10)
        self._notam_counter  = 0
        self.telemetry_echo_pub = self.create_publisher(String, '/ouranos/telemetry_echo', 10)  # <-- NEW

        # ---- Subscribers: ROS → Ouranos ----
        self.latest_local_pos = None
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self._on_local_position, px4_qos,
        )
        self.create_subscription(String, '/gpig/landing/status', self._on_landing_status, 10)

        # ---- TCP client state ----
        self._ouranos_conn: socket.socket | None = None
        self._conn_lock = threading.Lock()
        self.stop_event = threading.Event()

        self.conn_thread = threading.Thread(target=self._connection_loop, daemon=True)
        self.conn_thread.start()

        self.telemetry_timer = self.create_timer(1.0 / tlm_hz, self._send_telemetry)

        self.get_logger().info(
            f"Ouranos bridge will dial {self._server_host}:{self._server_port} "
            f"(backoff {self._reconnect_min}s → {self._reconnect_max}s)"
        )

        # ---- TCP server state (inbound: server-initiated commands on :9091) ----
        self._inbound_clients: list[socket.socket] = []
        self._inbound_clients_lock = threading.Lock()
        self._listen_sock: socket.socket | None = None
        
        self.accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self.accept_thread.start()
        

    # ────────────────────────────────────────────────
    #   Connection management
    # ────────────────────────────────────────────────

    def _connection_loop(self):
        """Dial the Go server; on connect, run the reader inline; on drop, back off and retry."""
        backoff = self._reconnect_min
        while not self.stop_event.is_set():
            try:
                sock = socket.create_connection(
                    (self._server_host, self._server_port), timeout=5.0
                )
                sock.settimeout(None)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            except (OSError, socket.timeout) as e:
                self.get_logger().warn(
                    f"Dial {self._server_host}:{self._server_port} failed: {e}. "
                    f"Retrying in {backoff:.1f}s."
                )
                self.stop_event.wait(backoff)
                backoff = min(backoff * 2.0, self._reconnect_max)
                continue

            self.get_logger().info(
                f"Connected to Ouranos at {self._server_host}:{self._server_port}"
            )
            backoff = self._reconnect_min  # reset on success

            with self._conn_lock:
                self._ouranos_conn = sock

            # Block here reading until the connection dies
            self._reader_loop(sock)

            # Reader returned → connection is dead; clear state and loop to redial
            with self._conn_lock:
                if self._ouranos_conn is sock:
                    self._ouranos_conn = None
            try:
                sock.close()
            except OSError:
                pass

            if not self.stop_event.is_set():
                self.get_logger().warn("Ouranos connection lost; will reconnect.")
                self.stop_event.wait(self._reconnect_min)

    def _reader_loop(self, sock: socket.socket):
        """Read newline-delimited <route>JSON messages from the server."""
        buffer = ""
        while not self.stop_event.is_set():
            try:
                data = sock.recv(4096)
            except OSError:
                return
            if not data:
                return
            buffer += data.decode("utf-8", errors="replace")
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                if not line.strip():
                    continue
                route, message = rosconn.parse_message(line)
                self._dispatch(route, message)

    # ────────────────────────────────────────────────
    #   Inbound listener (server-initiated commands)
    # ────────────────────────────────────────────────
    
    def _accept_loop(self):
        """Listen on (listen_host, listen_port) and spawn a reader per accepted client."""
        try:
            self._listen_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._listen_sock.bind((self._listen_host, self._listen_port))
            self._listen_sock.listen(5)
        except OSError as e:
            self.get_logger().error(
                f"Could not bind inbound listener on "
                f"{self._listen_host}:{self._listen_port}: {e}"
            )
            return
    
        self.get_logger().info(
            f"Inbound listener ready on {self._listen_host}:{self._listen_port}"
        )
    
        while not self.stop_event.is_set():
            try:
                client, addr = self._listen_sock.accept()
            except OSError:
                return  # listener closed during shutdown
    
            self.get_logger().info(f"Inbound client connected: {addr[0]}:{addr[1]}")
            with self._inbound_clients_lock:
                self._inbound_clients.append(client)
    
            t = threading.Thread(
                target=self._inbound_reader_loop,
                args=(client, addr),
                daemon=True,
            )
            t.start()

    def _inbound_reader_loop(self, sock: socket.socket, addr):
        """Read <route>{json} frames from one inbound client until it closes.

        flight.go sends frames *without* a trailing newline, so we can't split
        on '\\n' like we do upstream. Instead we look for '<route>' followed by
        a balanced JSON object, and treat that as one frame. Tolerates the
        presence or absence of a newline either way.
        """
        buffer = ""
        try:
            while not self.stop_event.is_set():
                try:
                    data = sock.recv(4096)
                except OSError:
                    break
                if not data:
                    break
                buffer += data.decode("utf-8", errors="replace")

                # Drain as many complete frames as we can find in the buffer.
                while True:
                    frame, buffer = self._extract_frame(buffer)
                    if frame is None:
                        break
                    route, message = rosconn.parse_message(frame)
                    self.get_logger().debug(
                        f"[inbound←{addr[0]}:{addr[1]}] route=<{route}> "
                        f"len={len(message)}"
                    )
                    self._dispatch(route, message)
        finally:
            with self._inbound_clients_lock:
                if sock in self._inbound_clients:
                    self._inbound_clients.remove(sock)
            try:
                sock.close()
            except OSError:
                pass
            self.get_logger().info(f"Inbound client disconnected: {addr[0]}:{addr[1]}")


    @staticmethod
    def _extract_frame(buffer: str) -> tuple[str | None, str]:
        """Pull one '<route>{json}' frame out of buffer.

        Returns (frame, remaining_buffer). frame is None if no complete frame
        is present yet (caller should recv more bytes and retry).

        Also tolerates trailing whitespace / newlines and skips them.
        """
        # Skip leading whitespace / stray newlines between frames.
        i = 0
        while i < len(buffer) and buffer[i] in " \t\r\n":
            i += 1
        if i:
            buffer = buffer[i:]

        if not buffer:
            return None, buffer

        if not buffer.startswith("<"):
            # Junk before the next '<' — drop up to the next '<' to resync.
            nxt = buffer.find("<")
            if nxt == -1:
                return None, ""  # all junk, discard
            buffer = buffer[nxt:]

        end_route = buffer.find(">")
        if end_route == -1:
            return None, buffer  # need more bytes to finish the route token

        # Find the start of the JSON object.
        j = end_route + 1
        # Skip whitespace between '>' and '{'
        while j < len(buffer) and buffer[j] in " \t\r\n":
            j += 1

        if j >= len(buffer):
            return None, buffer  # need more bytes

        # If the payload doesn't start with '{' or '[', fall back to newline
        # framing (handles route-only messages like '<Ping>' or simple strings).
        if buffer[j] not in "{[":
            nl = buffer.find("\n", j)
            if nl == -1:
                # No JSON, no newline — wait for more or for connection close.
                return None, buffer
            frame = buffer[:nl]
            return frame, buffer[nl + 1:]

        # Brace-balance scan, respecting strings and escapes.
        open_ch = buffer[j]
        close_ch = "}" if open_ch == "{" else "]"
        depth = 0
        in_str = False
        escape = False
        k = j
        while k < len(buffer):
            c = buffer[k]
            if escape:
                escape = False
            elif c == "\\":
                escape = True
            elif in_str:
                if c == '"':
                    in_str = False
            else:
                if c == '"':
                    in_str = True
                elif c == open_ch:
                    depth += 1
                elif c == close_ch:
                    depth -= 1
                    if depth == 0:
                        frame = buffer[: k + 1]
                        rest = buffer[k + 1:]
                        return frame, rest
            k += 1

        # Unbalanced — need more bytes.
        return None, buffer



    # ────────────────────────────────────────────────
    #   ROS → Ouranos
    # ────────────────────────────────────────────────

    def _on_local_position(self, msg: VehicleLocalPosition):
        """Cache the latest local position so the telemetry timer can format it."""
        self.latest_local_pos = msg

    # def _on_landing_status(self, msg: String):
    #     """Forward landing completion back to the drone client.

    #     flight.go's fly() loop exits when it receives <Data> (it calls
    #     SendOdometryUpdate then sets DeliveryInProgress=false). We piggyback
    #     on that path because <Delivered> and <Docked> in flight.go currently
    #     keep DeliveryInProgress=true (looks like a bug on the Go side).
    #     """
    #     if not msg.data or not msg.data.startswith("landing_ready"):
    #         return
    #     payload = json.dumps({"status": msg.data})
    #     line = f"<Delivered>{payload}\n"  
    #     encoded = line.encode("utf-8")
    #     with self._inbound_clients_lock:
    #         clients = list(self._inbound_clients)
    #     sent = 0
    #     for c in clients:
    #         try:
    #             c.sendall(encoded)
    #             sent += 1
    #         except OSError:
    #             pass
    #     self.get_logger().info(
    #         f"Landing complete → sent <Data> ack to {sent} inbound client(s)"
    #     )

    # def _on_landing_status(self, msg: String):
    #     if not msg.data:
    #         return

    #     # Pick the right upstream route token based on the status string.
    #     if msg.data.startswith("docked"):
    #         route_token = "<Docked>"
    #         log_label = "Docked at home"
    #     elif msg.data.startswith("landing_ready"):
    #         route_token = "<Delivered>"
    #         log_label = "Delivery landing complete"
    #     else:
    #         self.get_logger().debug(f"Ignoring unrecognised landing status: {msg.data!r}")
    #         return

    #     payload = json.dumps({"status": msg.data})
    #     line = f"{route_token}{payload}\n"
    #     encoded = line.encode("utf-8")
    #     with self._inbound_clients_lock:
    #         clients = list(self._inbound_clients)
    #     sent = 0
    #     for c in clients:
    #         try:
    #             c.sendall(encoded)
    #             sent += 1
    #         except OSError:
    #             pass
    #     self.get_logger().info(
    #         f"{log_label} → sent {route_token} to {sent} inbound client(s)"
    #     )
    
    def _on_landing_status(self, msg: String):
        """Forward landing completion back to the drone client.

        Routes 'docked_*' status strings as <Docked> (home-return / mission complete),
        and 'landing_ready_*' as <Delivered> (delivery landing). flight.go's fly() loop
        exits on either token, but the Go server treats them differently downstream:
        <Delivered> → SignalDeliveryComplete; <Docked> → CurrentStatus=Docked + RestockDrone.
        """
        if not msg.data:
            return

        if msg.data.startswith("docked"):
            route_token = "<Docked>"
            log_label = "Docked at home"
        elif msg.data.startswith("landing_ready"):
            route_token = "<Delivered>"
            log_label = "Delivery landing complete"
        else:
            self.get_logger().debug(f"Ignoring unrecognised landing status: {msg.data!r}")
            return

        payload = json.dumps({"status": msg.data})
        line = f"{route_token}{payload}\n"
        encoded = line.encode("utf-8")
        with self._inbound_clients_lock:
            clients = list(self._inbound_clients)
        sent = 0
        for c in clients:
            try:
                c.sendall(encoded)
                sent += 1
            except OSError:
                pass
        self.get_logger().info(
            f"{log_label} → sent {route_token} to {sent} inbound client(s)"
        )



    def _send_telemetry(self):
        pos = self.latest_local_pos
        if pos is None:
            return
        velocity = math.sqrt(pos.vx ** 2 + pos.vy ** 2 + pos.vz ** 2)
        payload = {
            "X": round(float(pos.x), 3),
            "Y": round(float(pos.y), 3),
            "Z": round(float(pos.z), 3),
            "Velocity": round(float(velocity), 3),
        }
        # NOTE: route token is a placeholder — confirm with peer whether the
        # central server expects <UpdateOdometry>, <Data>, or something else.
        line = f"<telemetry>{json.dumps(payload)}\n"
        self._send_line(line)

    def _send_line(self, line: str):
        """Send one framed line upstream; drop the connection on write error so the manager redials."""
        encoded = line.encode("utf-8")
        with self._conn_lock:
            sock = self._ouranos_conn
            if sock is None:
                return  # not connected yet — drop telemetry silently
            try:
                sock.sendall(encoded)
            except OSError as e:
                self.get_logger().warn(f"Upstream write failed: {e}; dropping connection.")
                try:
                    sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                try:
                    sock.close()
                except OSError:
                    pass
                self._ouranos_conn = None

    # ────────────────────────────────────────────────
    #   Ouranos → ROS
    # ────────────────────────────────────────────────

    def _dispatch(self, route: str, message: str):
        route_lc = route.lower()

        if route_lc == "telemetry":
            self._handle_telemetry_echo(message)
            return

        raw = String(); raw.data = f"<{route}>{message}"
        self.raw_pub.publish(raw)
    
        
        if route_lc == "destination":
            self._handle_destination(message)
        # elif route_lc == "data":
        #     self._handle_data_envelope(message)         # <-- NEW
        elif route_lc == "notam":
            self._handle_notam(message)
        elif route_lc in ("ping", "reply", "correct", "data"):
            self.get_logger().debug(f"Control route [{route}]: {message[:200]}")
        else:
            self.get_logger().debug(f"Unhandled route [{route}]: {message[:200]}")  # promote to info while debugging
    
    def _handle_telemetry_echo(self, message: str):
        """Republish inbound <telemetry> frames on /ouranos/telemetry for the dashboard."""
        out = String(); out.data = message
        self.telemetry_echo_pub.publish(out)


    def _handle_data_envelope(self, message: str):
        """Unwrap the Go server's <Data>|{ticket}|{ticket}... envelope.

        Each record is a ticket object whose Flight.Location is the destination.
        """
        # Strip the leading '|' separator and split on '|' in case multiple
        # tickets are concatenated (matches the pattern in regionclient.go).
        payload = message.lstrip("|")
        if not payload:
            self.get_logger().warn("Empty <Data> payload")
            return

        for chunk in payload.split("|"):
            chunk = chunk.strip()
            if not chunk:
                continue
            try:
                ticket = json.loads(chunk)
            except json.JSONDecodeError as e:
                self.get_logger().warn(f"Bad ticket JSON in <Data>: {e}. Raw: {chunk!r}")
                continue

            flight = ticket.get("Flight") or {}
            loc = flight.get("Location") or {}
            lat = loc.get("Lat")
            lon = loc.get("Lon")

            if lat is None or lon is None:
                self.get_logger().info(
                    f"<Data> ticket without Flight.Location, ignoring "
                    f"(TicketID={ticket.get('Flight', {}).get('TicketID')})"
                )
                continue

            # Re-emit as a normalised destination dict and reuse the existing handler.
            normalised = json.dumps({
                "lat": lat,
                "lon": lon,
                "alt": loc.get("Alt", self._home_ref_alt or 0.0),
            })
            self.get_logger().info(
                f"Translating <Data> ticket {flight.get('TicketID')} → destination "
                f"({lat}, {lon})"
            )
            self._handle_destination(normalised)

    def _handle_destination(self, message: str):
        """Parse an X/Y destination in the sim's local frame and publish as Point.
    
        Two payload shapes are accepted:
          1. Drone-client status dump from flight.go:
             {"Flight": {"Location": {"X": ..., "Y": ...}, "TicketID": ..., ...}, ...}
          2. Flat shape for manual rosconn.py / netcat testing:
             {"x": ..., "y": ...}     (also accepts "X"/"Y")
        """
        try:
            data = json.loads(message)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Could not parse destination JSON: {e}. Raw: {message!r}")
            return
    
        dest_x = dest_y = None
        ticket_id = flight_id = None
    
        # Shape 1: nested under Flight.Location (flight.go's actual output)
        if isinstance(data, dict) and isinstance(data.get("Flight"), dict):
            flight = data["Flight"]
            loc = flight.get("Location") or {}
            x_raw = loc.get("X", loc.get("x", loc.get("Lat", loc.get("lat"))))  # tolerate misnamed 'Lat' key
            y_raw = loc.get("Y", loc.get("y", loc.get("Lon", loc.get("lon"))))  # tolerate misnamed 'Lon' key
            if x_raw is not None and y_raw is not None:
                try:
                    dest_x = float(x_raw)
                    dest_y = float(y_raw)
                    ticket_id = flight.get("TicketID")
                    flight_id = flight.get("FlightID")
                except (TypeError, ValueError) as e:
                    self.get_logger().warn(f"Flight.Location invalid: {e}. Raw: {loc!r}")
                    return
    
        # Shape 2: flat keys (manual testing)
        if dest_x is None and isinstance(data, dict):
            x_raw = data.get("x", data.get("X", data.get("Lat", data.get("lat"))))
            y_raw = data.get("y", data.get("Y", data.get("Lon", data.get("lon"))))
            if x_raw is not None and y_raw is not None:
                try:
                    dest_x = float(x_raw)
                    dest_y = float(y_raw)
                except (TypeError, ValueError) as e:
                    self.get_logger().warn(f"Destination JSON invalid numeric fields: {e}. Raw: {data!r}")
                    return
    
        if dest_x is None or dest_y is None:
            self.get_logger().warn(f"Destination JSON has no usable X/Y. Raw: {data!r}")
            return
    
        # flight.go initialises Flight.Location to client.Home (often {0,0}) before
        # any flight is dispatched. Treat that as 'no active flight'.
        if (abs(dest_x) < 1e-6 and abs(dest_y) < 1e-6 and ticket_id is None and not self._has_published_destination):
            self.get_logger().info("Destination is (0,0) — treating as 'no active flight', ignoring.")
            return
    
        msg = Point(x=dest_x, y=dest_y, z=0.0)
        self.destination_pub.publish(msg)
        self._has_published_destination = True
        self.get_logger().info(
            f"Destination → /ouranos/destination: ({dest_x:.2f}, {dest_y:.2f}) "
            f"[ticket={ticket_id} flight={flight_id}]"
        )
    

    def _handle_notam(self, message: str):
        """Parse a NOTAM from Ouranos and republish as a JSON cylinder spec."""
        try:
            data = json.loads(message)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Could not parse NOTAM JSON: {e}. Raw: {message!r}")
            return

        if isinstance(data, list):
            if len(data) != 3:
                self.get_logger().warn(f"NOTAM list payload must be [x, y, radius]; got {data!r}")
                return
            data = {"x": data[0], "y": data[1], "radius": data[2]}

        if not isinstance(data, dict):
            self.get_logger().warn(f"NOTAM payload must be JSON object or list; got {type(data).__name__}")
            return

        missing = [k for k in ("x", "y", "radius") if k not in data]
        if missing:
            self.get_logger().warn(f"NOTAM missing required field(s) {missing}. Raw: {data!r}")
            return

        try:
            cx = float(data["x"]); cy = float(data["y"]); radius = float(data["radius"])
        except (TypeError, ValueError) as e:
            self.get_logger().warn(f"NOTAM x/y/radius must be numeric: {e}. Raw: {data!r}")
            return

        if not math.isfinite(radius) or radius <= 0.0:
            self.get_logger().warn(f"NOTAM radius must be positive and finite, got {radius}. Ignoring.")
            return
        if not (math.isfinite(cx) and math.isfinite(cy)):
            self.get_logger().warn(f"NOTAM centre must be finite, got ({cx}, {cy}). Ignoring.")
            return

        try:
            alt_min = float(data.get("alt_min", self._notam_default_alt_min))
            alt_max = float(data.get("alt_max", self._notam_default_alt_max))
        except (TypeError, ValueError):
            alt_min = self._notam_default_alt_min
            alt_max = self._notam_default_alt_max

        if alt_max <= alt_min:
            alt_min = self._notam_default_alt_min
            alt_max = self._notam_default_alt_max

        notam_id = data.get("id")
        if notam_id is None or notam_id == "":
            notam_id = f"notam-{self._notam_counter}"
            self._notam_counter += 1
        notam_id = str(notam_id)

        cylinder = {
            "id": notam_id, "center_x": cx, "center_y": cy,
            "radius": radius, "alt_min": alt_min, "alt_max": alt_max,
        }
        out = String(); out.data = json.dumps(cylinder)
        self.notam_pub.publish(out)
        self.get_logger().info(
            f"NOTAM [{notam_id}] → /ouranos/notam: "
            f"centre=({cx:.2f}, {cy:.2f}) radius={radius:.2f}m "
            f"alt=[{alt_min:.1f}, {alt_max:.1f}]m"
        )

    # ────────────────────────────────────────────────
    #   Shutdown
    # ────────────────────────────────────────────────

    def destroy_node(self):
        self.stop_event.set()
    
        # Close outbound (dialer) socket
        with self._conn_lock:
            sock = self._ouranos_conn
            self._ouranos_conn = None
        if sock is not None:
            try: sock.shutdown(socket.SHUT_RDWR)
            except OSError: pass
            try: sock.close()
            except OSError: pass
    
        # Close inbound listener so accept() returns
        if self._listen_sock is not None:
            try: self._listen_sock.close()
            except OSError: pass
    
        # Close any accepted inbound clients
        with self._inbound_clients_lock:
            clients = list(self._inbound_clients)
            self._inbound_clients.clear()
        for c in clients:
            try: c.shutdown(socket.SHUT_RDWR)
            except OSError: pass
            try: c.close()
            except OSError: pass
    
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
