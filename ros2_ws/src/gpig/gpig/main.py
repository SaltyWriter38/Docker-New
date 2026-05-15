import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleOdometry, VehicleStatus, VehicleControlMode
from geometry_msgs.msg import Point
from std_msgs.msg import String

import ast
import math

class OffboardControl(Node):

    START_X = -20.0
    START_Y = 0.0
    #target x,y coordinates (these will come from the server and we need some method of recieveing / setting them)
    TARGET_X = 0.0 - START_Y
    TARGET_Y = 0.0 - START_X
    #THESE ARE NOT BACKWARDS - THE X AND Y IS FLIPPED !!!!


    TARGET_ALTITUDE = 10.0 #the altitude we want to be flying around at
    MOVEMENT_SPEED = 5.0 #normal movement speed in m/s
    ARR_RAD = 5 #how many metres away from the target we have to be, to consider ourselves to be there

    #landing guidance tuning
    #image vector is in pixels, so we need a rough scale to convert to metres.
    #bullshit factors :D
    PIXEL_TO_METER_GAIN = 0.02
    CLOSE_ENOUGH_THRESHOLD = 0.3
    MAX_LANDING_STEP_METERS = 0.2
    SMALL_VECTOR_THRESHOLD_PX = 100  # below this pixel magnitude, stop adjusting and hover

    #which way do we need to flip x and y for it to be correct in the image compared to the drone rotation
    #one or both might need minusing ??
    IMAGE_X_TO_BODY_RIGHT_SIGN = 1.0
    IMAGE_Y_TO_BODY_FORWARD_SIGN = 1.0

    #-------------------------- CLASS INIT ----------------------------
    def __init__(self):
        super().__init__('controller')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        #PUBLISHERS
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        # status publisher for landing outcome
        self.status_pub = self.create_publisher(String, '/gpig/landing/status', qos_profile)

        #SUBSCRIBERS
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_position_callback, qos_profile)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.vehicle_control_mode_callback, qos_profile)
        self.create_subscription(Point, '/ouranos/destination', self._on_destination, 10)
        self.create_subscription(String, '/gpig/object_detection/summary', self.image_summary_callback, 10)

        #CALLBACKS AND OTHER VARIABLES NOT KNOWN AT COMPILE TIME
        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz
        self.counter = 0

        # diagnostics
        self.position_received = False

        # arming / offboard gating
        self.ARM_WAIT_TICKS = 50  # wait this many ticks (~5s) before sending arm/mode
        self.armed_sent = False
        self.vehicle_status = None

        self.arrived = False
        #position unknown until we hear from VehicleLocalPosition
        self.currentX = None
        self.currentY = None
        self.currentZ = None
        #info about landing safe spot
        self.safe_spot_found = False
        self.vector_to_safe_spot = (None, None)
        self.landing_command_sent = False
        #info about current rotation
        self.current_yaw_rad = None
        self.current_pose_frame = VehicleOdometry.POSE_FRAME_UNKNOWN
        #info about takeoff phase
        self.takeoff_complete = False
        # landing/approach state
        self.landing_spot_locked = False
        self.landing_spot_world = (None, None)
        self.landing_approach_started = False
        self.landing_descend_started = False
        self.landing_success_sent = False
        self.landing_hold_counter = 0
        self.LANDING_HOLD_TICKS = 10  # require 1s hold before descend
        self.FINAL_LANDING_ALTITUDE = 2.5


        #-- notam test stuff --
        self.testNotam_x = 0.0
        self.testNotam_y = 10.0
        self.testNotam_radius = 5.0
        self.notam_avoidance_margin = 2.0

    #-------------------------- PUBLISHERS ----------------------------

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position, msg.velocity, msg.acceleration = True, False, False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        #float('nan') means ignore this axis
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.position = [x, y, z]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

    def publish_hover_setpoint(self, x=None, y=None, z=None):
        """Publish a setpoint that hovers at given x/y (or current pos) and z.
        Use None for x/y to use current position if available, otherwise NaN.
        """
        if x is None:
            x = self.currentX if self.currentX is not None else float('nan')
        if y is None:
            y = self.currentY if self.currentY is not None else float('nan')
        if z is None:
            z = self.target_z()
        self.publish_trajectory_setpoint(x, y, z)

    def target_z(self, altitude=None):
        alt = altitude if altitude is not None else self.TARGET_ALTITUDE
        return -abs(alt)

    def publish_vehicle_command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1, msg.param2 = p1, p2
        msg.target_system, msg.target_component = 1, 1
        msg.source_system, msg.source_component = 1, 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)

    #-------------------------- LOCATION ----------------------------

    def local_position_callback(self, msg:VehicleLocalPosition):
        was_none = self.currentX is None
        self.currentX = msg.x
        self.currentY = msg.y
        self.currentZ = msg.z
        if was_none and not self.position_received:
            self.position_received = True
            self.get_logger().info(f"Local position available: x={self.currentX:.2f}, y={self.currentY:.2f}, z={self.currentZ:.2f}", once=True)

    def vehicle_odometry_callback(self, msg: VehicleOdometry):
        self.current_pose_frame = msg.pose_frame

        #rotation data comes through as quaternion
        q0, q1, q2, q3 = msg.q
        if any(math.isnan(v) for v in (q0, q1, q2, q3)):
            #reject invalid data
            return

        #find yaw in radians from quaternion
        self.current_yaw_rad = math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3))

    def vehicle_status_callback(self, msg: VehicleStatus):
        # store latest vehicle status and log key fields when they change
        prev = self.vehicle_status
        self.vehicle_status = msg
        try:
            arming = getattr(msg, 'arming_state', None)
            nav = getattr(msg, 'nav_state', None)
        except Exception:
            arming = None
            nav = None

        if prev is None:
            self.get_logger().info(f"VehicleStatus: arming={arming} nav={nav}")
        else:
            prev_arming = getattr(prev, 'arming_state', None)
            prev_nav = getattr(prev, 'nav_state', None)
            if arming != prev_arming or nav != prev_nav:
                self.get_logger().info(f"VehicleStatus changed: arming={arming} nav={nav}")

    def vehicle_control_mode_callback(self, msg: VehicleControlMode):
        # log whether PX4 reports offboard control accepted
        try:
            offboard = getattr(msg, 'flag_control_offboard_enabled', None)
        except Exception:
            offboard = None
        self.get_logger().info(f"VehicleControlMode: offboard_enabled={offboard}")

    def _on_destination(self, msg: Point):
        """Callback fired when the Ouranos dashboard sends a new destination."""
        self.TARGET_X = float(msg.x) - self.START_Y
        self.TARGET_Y = float(msg.y) - self.START_X
        # Reset arrival flag so the drone re-plans toward the new target
        self.arrived = False
        self.landing_command_sent = False
        # Don't reset takeoff_complete - we only need to take off once
        self.get_logger().info(
            f"New destination received from Ouranos: "
            f"TARGET_X={self.TARGET_X}, TARGET_Y={self.TARGET_Y}"
        )

    #-------------------------- LANDING IMAGE PROCESSING ----------------------------

    def image_summary_callback(self, msg:String):
        summary = msg.data
        # summary just comes through as a big string; detect flag then parse
        self.safe_spot_found = "safe_spot_found=True" in summary
        self.vector_to_safe_spot = (None, None)

        if not self.safe_spot_found:
            return

        parsed = self._parse_vector_from_summary(summary)
        if parsed is None:
            self.get_logger().error("Failed to parse vector_to_safe_spot from summary")
            self.safe_spot_found = False
            return

        self.vector_to_safe_spot = parsed
        #self.get_logger().info(f"Image parser: safe_spot_found={self.safe_spot_found} vector={self.vector_to_safe_spot}")

    def _parse_vector_from_summary(self, summary: str):
        start_idx = summary.find("vector_to_safe_spot=")
        if start_idx == -1:
            return None
        start_idx += len("vector_to_safe_spot=")
        s = summary[start_idx:].strip()

        end_idx = -1
        paren_count = 0
        for i, ch in enumerate(s):
            if ch in '([{':
                paren_count += 1
            elif ch in ')]}':
                paren_count -= 1
                if paren_count == 0:
                    end_idx = i + 1
                    break

        if end_idx <= 0:
            return None

        try:
            parsed = ast.literal_eval(s[:end_idx])
        except Exception:
            return None

        if isinstance(parsed, (tuple, list)) and len(parsed) == 2:
            try:
                return (float(parsed[0]), float(parsed[1]))
            except Exception:
                return None
        return None


    #-------------------------- TIMER CALLBACK -> MAIN LOOP ----------------------------

    def timer_callback(self):
        if self.landing_command_sent:
            return

        #every time the timer ticks, reaffirm to PX4 that we are actually flying the drone so it doesnt auto-land
        self.publish_offboard_control_mode()

        #logic below assumes timer ticks every tenth of a second

        # Warmup: publish steady non-NaN setpoints for a short period before arming
        if self.counter < self.ARM_WAIT_TICKS:
            self.publish_hover_setpoint(x=None, y=None, z=None)
            if self.counter % 10 == 0:
                self.get_logger().info(f"Warming up offboard setpoints ({self.counter}/{self.ARM_WAIT_TICKS})")
        else:
            # attempt to arm and switch to offboard once warmed up and position known
            if not self.armed_sent:
                if self.position_received:
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0) # 6 = Offboard
                    self.armed_sent = True
                    self.get_logger().info("DRONE SWITCHING TO OFFBOARD MODE", once=True)
                else:
                    self.get_logger().warn("Delaying arming: waiting for local position", once=True)

            # after arming proceed with mission control
            if self.armed_sent:
                if not self.takeoff_complete:
                    self.perform_takeoff()
                else:
                    if self.arrived:
                        self.get_logger().info("DESTINATION REACHED, ATTEMPTING LANDING !!", once=True)
                        self.land_the_drone()
                    else:
                        self.fly_towards_coord()
                        if self.counter % 50 == 0:
                            self.get_logger().info("DRONE FLYING")

        #increment the timer counter
        self.counter += 1

    #-------------------------- CLASS HELPER FUNCTIONS ----------------------------
    #i.e. rely on info from self, rather than static args

    def perform_takeoff(self):
        """Climb straight up to TARGET_ALTITUDE without moving horizontally."""
        target_z = self.target_z()

        if self.currentX is None or self.currentZ is None:
            self.get_logger().warn("Waiting for position data during takeoff...")
            self.publish_trajectory_setpoint(0.0, 0.0, target_z)
            return

        if self.counter % 10 == 0:
            self.get_logger().info(f"Takeoff: currentZ={self.currentZ:.2f} target_z={target_z:.2f}")

        # stay at current X,Y while climbing
        self.publish_hover_setpoint(x=self.currentX, y=self.currentY, z=target_z)

        # use absolute tolerance to be robust to sign conventions
        if abs(self.currentZ - target_z) <= 0.5:
            self.takeoff_complete = True
            self.get_logger().info("TAKEOFF COMPLETE, BEGINNING HORIZONTAL FLIGHT", once=True)

    def fly_towards_coord(self):
        #at the moment this takes the target coord from the class attribute
        #can be easily modified to take it as an argument

        target_z = self.target_z()

        if self.currentX is None:
            #have not yet heard from VehicleLocalPosition
            #really want to do nothing, but if we idle then PX4 hates us and times out
            #so hover at origin (or just stay put since we don't know where we are)

            self.get_logger().warn("I don't know where my local position is !!!")

            # publish a hover setpoint (NaN = ignore axis) to maintain altitude
            self.publish_hover_setpoint(x=None, y=None, z=target_z)

            return

        xy_error = math.hypot(self.TARGET_X - self.currentX, self.TARGET_Y - self.currentY)
        z_error = abs(target_z - self.currentZ) if self.currentZ is not None else float('inf')
        if not self.arrived:
            self.arrived = (xy_error < self.ARR_RAD) and (z_error < 1.0)
        # periodic debug logging
        if self.counter % 20 == 0:
            self.get_logger().info(f"Fly-> target=({self.TARGET_X:.2f},{self.TARGET_Y:.2f}) pos=({self.currentX:.2f},{self.currentY:.2f}) xy_err={xy_error:.2f} z_err={z_error:.2f}")

        #do the notam avoidance pizzazz
        if self.is_inside_notam():
            avoid_x, avoid_y = self.compute_notam_avoidance_point()
            if avoid_x is not None and avoid_y is not None:
                self.get_logger().info(f"Inside NOTAM! Avoidance point: ({avoid_x:.2f}, {avoid_y:.2f})")
                self.publish_trajectory_setpoint(avoid_x, avoid_y, target_z)
                return
            else:
                self.get_logger().error("Failed to compute NOTAM avoidance point")

        else:
            #normal flying towards target
            self.publish_trajectory_setpoint(self.TARGET_X, self.TARGET_Y, target_z)

    def land_the_drone(self):
        # New landing procedure:
        # 1) Once `arrived` is True we hover and look for a safe spot
        # 2) When a safe spot is detected, lock its world coords
        # 3) Approach the locked spot at the current mission altitude
        # 4) When within ARR_RAD and held for several ticks, descend to FINAL_LANDING_ALTITUDE (2.5m)
        # 5) Publish a success message and hover

        target_z = self.target_z()

        # we need yaw and NED frame to convert camera vector to world coordinates
        if self.current_yaw_rad is None or self.current_pose_frame != VehicleOdometry.POSE_FRAME_NED:
            hover_x = self.currentX if self.currentX is not None else float('nan')
            hover_y = self.currentY if self.currentY is not None else float('nan')
            self.publish_hover_setpoint(x=hover_x, y=hover_y, z=target_z)
            self.get_logger().warn("Waiting for valid odometry/rotation before landing", once=True)
            return

        # If we've already declared success, just hover at final altitude
        if self.landing_success_sent:
            final_z = self.target_z(self.FINAL_LANDING_ALTITUDE)
            hover_x = self.currentX if self.currentX is not None else float('nan')
            hover_y = self.currentY if self.currentY is not None else float('nan')
            self.publish_hover_setpoint(x=hover_x, y=hover_y, z=final_z)
            return

        # 1) If we don't yet have a locked landing spot, hover and attempt to lock one
        if not self.landing_spot_locked:
            hover_x = self.currentX if self.currentX is not None else float('nan')
            hover_y = self.currentY if self.currentY is not None else float('nan')
            self.publish_trajectory_setpoint(hover_x, hover_y, target_z)

            if self.safe_spot_found and None not in self.vector_to_safe_spot:
                dx_px = float(self.vector_to_safe_spot[0])
                dy_px = float(self.vector_to_safe_spot[1])

                pixel_distance = math.hypot(dx_px, dy_px)

                # If the vector is already very small, lock to current hover position and stop adjusting
                if pixel_distance <= self.SMALL_VECTOR_THRESHOLD_PX:
                    self.landing_spot_world = (self.currentX, self.currentY)
                    self.landing_spot_locked = True
                    self.get_logger().info(f"Small image vector ({pixel_distance:.1f}px) -> locking to current hover position X={self.currentX:.2f}, Y={self.currentY:.2f}", once=True)
                    return

                # convert image pixels to metres in body frame
                body_right_m = dx_px * self.PIXEL_TO_METER_GAIN * self.IMAGE_X_TO_BODY_RIGHT_SIGN
                body_forward_m = dy_px * self.PIXEL_TO_METER_GAIN * self.IMAGE_Y_TO_BODY_FORWARD_SIGN

                # world-frame delta using current yaw
                yaw = self.current_yaw_rad
                north_delta = math.cos(yaw) * body_forward_m - math.sin(yaw) * body_right_m
                east_delta = math.sin(yaw) * body_forward_m + math.cos(yaw) * body_right_m

                landing_x = self.currentX + north_delta
                landing_y = self.currentY + east_delta

                self.landing_spot_world = (landing_x, landing_y)
                self.landing_spot_locked = True
                self.get_logger().info(f"Locked landing spot X={landing_x:.2f}, Y={landing_y:.2f}", once=True)
                self.get_logger().info(f"Vector-> body (fwd,right)=({body_forward_m:.2f},{body_right_m:.2f}) north_delta={north_delta:.2f} east_delta={east_delta:.2f}")

            else:
                self.get_logger().info("Hovering and awaiting safe spot detection", once=True)
            return

        # 2) We have a locked landing spot -> approach it at mission altitude
        landing_x, landing_y = self.landing_spot_world
        self.publish_trajectory_setpoint(landing_x, landing_y, target_z)

        # compute horizontal distance to locked spot
        if self.currentX is None or self.currentY is None:
            horiz_dist = float('inf')
        else:
            horiz_dist = math.hypot(landing_x - self.currentX, landing_y - self.currentY)

        # if within ARR_RAD, start hold counter
        if horiz_dist <= self.ARR_RAD:
            self.landing_hold_counter += 1
            if self.counter % 5 == 0:
                self.get_logger().info(f"Holding over spot: horiz_dist={horiz_dist:.2f} hold_counter={self.landing_hold_counter}")
        else:
            if self.landing_hold_counter != 0 and self.counter % 5 == 0:
                self.get_logger().info(f"Left hold region: horiz_dist={horiz_dist:.2f} resetting hold")
            self.landing_hold_counter = 0

        # when held long enough, descend to final altitude
        if self.landing_hold_counter >= self.LANDING_HOLD_TICKS:
            self.landing_descend_started = True

        if self.landing_descend_started:
            final_z = self.target_z(self.FINAL_LANDING_ALTITUDE)
            # log descent start
            if self.counter % 5 == 0:
                self.get_logger().info(f"Descending to final_z={final_z:.2f} from currentZ={self.currentZ}")
            self.publish_hover_setpoint(x=landing_x, y=landing_y, z=final_z)

            # when reached final altitude (allow small tolerance) declare success
            if self.currentZ is not None and abs(self.currentZ - final_z) <= 0.3:
                if not self.landing_success_sent:
                    msg = String()
                    msg.data = "landing_ready_at_2.5m"
                    self.status_pub.publish(msg)
                    self.landing_success_sent = True
                    self.get_logger().info("Reached 2.5m above spot — publishing success and hovering", once=True)
            return
        
    #TODO ADAPT THESE METHODS FOR HAVING A STRUCTURE OF SEVERAL NOTAMS, EACH OF WHICH IS SOME OBJECT WITH X Y R
    def distance_to_notam_center(self):
        if self.currentX is None or self.currentY is None:
            return float('inf')
        return math.hypot(self.currentX - self.testNotam_x, self.currentY - self.testNotam_y)   
    
    def is_inside_notam(self):
        return self.distance_to_notam_center() < (self.testNotam_radius + self.notam_avoidance_margin)
    
    def compute_notam_avoidance_point(self):
        if self.currentX is None or self.currentY is None:
            return None, None

        dx = self.currentX - self.testNotam_x
        dy = self.currentY - self.testNotam_y
        d = math.hypot(dx, dy)
        if d < 1e-6:
            ux, uy = 1.0, 0.0
        else:
            ux, uy = dx / d, dy / d
        avoid_x = self.testNotam_x + ux * (self.testNotam_radius + self.notam_avoidance_margin)
        avoid_y = self.testNotam_y + uy * (self.testNotam_radius + self.notam_avoidance_margin)
        return avoid_x, avoid_y
    
def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# NOTAM: gets a central point (x,y) and a radius in metres
# model this as an infinite height cylinder 
# pathfind around this:
#   if my current x/y coord in inside the cylinder, then what? move sideways by one radius + a bit
#   does the drone know the radius of this notam cylinder ?