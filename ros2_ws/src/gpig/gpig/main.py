import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleOdometry
from geometry_msgs.msg import Point
from std_msgs.msg import String

import ast
import math

class OffboardControl(Node):

    #target x,y coordinates (these will come from the server and we need some method of recieveing / setting them)
    TARGET_X = 0.0
    TARGET_Y = 0.0 - 20

    TARGET_ALTITUDE = 10.0 #the altitude we want to be flying around at
    MOVEMENT_SPEED = 5.0 #normal movement speed in m/s
    ARR_RAD = 5 #how many metres away from the target we have to be, to consider ourselves to be there

    #landing guidance tuning
    #image vector is in pixels, so we need a rough scale to convert to metres.
    #bullshit factors :D
    PIXEL_TO_METER_GAIN = 0.02
    CLOSE_ENOUGH_THRESHOLD = 0.3
    MAX_LANDING_STEP_METERS = 0.2

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

        #SUBSCRIBERS
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_position_callback, qos_profile)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        self.create_subscription(Point, '/ouranos/destination', self._on_destination, 10)
        self.create_subscription(String, '/gpig/object_detection/summary', self.image_summary_callback, 10)

        #CALLBACKS AND OTHER VARIABLES NOT KNOWN AT COMPILE TIME
        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz
        self.counter = 0

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
        self.currentX = msg.x
        self.currentY = msg.y
        self.currentZ = msg.z

    def vehicle_odometry_callback(self, msg: VehicleOdometry):
        self.current_pose_frame = msg.pose_frame

        #rotation data comes through as quaternion
        q0, q1, q2, q3 = msg.q
        if any(math.isnan(v) for v in (q0, q1, q2, q3)):
            #reject invalid data
            return

        #find yaw in radians from quaternion
        self.current_yaw_rad = math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3))

    def _on_destination(self, msg: Point):
        """Callback fired when the Ouranos dashboard sends a new destination."""
        self.TARGET_X = float(msg.x)
        self.TARGET_Y = float(msg.y)
        # Reset arrival flag so the drone re-plans toward the new target
        self.arrived = False
        self.landing_command_sent = False
        self.get_logger().info(
            f"New destination received from Ouranos: "
            f"TARGET_X={self.TARGET_X}, TARGET_Y={self.TARGET_Y}"
        )

    #-------------------------- LANDING IMAGE PROCESSING ----------------------------

    def image_summary_callback(self, msg:String):
        summary = msg.data
        #summary just comes through as a big old string
        #so parse that string for the data we're interested in
        self.safe_spot_found = "safe_spot_found=True" in summary
        self.vector_to_safe_spot = (None, None)

        if self.safe_spot_found:
            try:
                #string shenanigans
                #find where in the message the vector info starts (is this always in the same place?)
                start_idx = summary.find("vector_to_safe_spot=")
                if start_idx != -1:
                    #now look at after the = sign
                    start_idx += len("vector_to_safe_spot=")
                    #there is nothing else in the message after the data we need
                    vector_str = summary[start_idx:]
                    self.vector_to_safe_spot = ast.literal_eval(vector_str)
        
            except Exception as e:
                self.get_logger().error(f"Failed to parse vector_to_safe_spot: {e}")


    #-------------------------- TIMER CALLBACK -> MAIN LOOP ----------------------------

    def timer_callback(self):
        if self.landing_command_sent:
            return

        #every time the timer ticks, reaffirm to PX4 that we are actually flying the drone so it doesnt auto-land
        self.publish_offboard_control_mode()

        #logic below assumes timer ticks every tenth of a second

        #px4 absolutely hates it if there is no setpoint at any point
        #so have to publish one here even though i don't really want to
        #could replace this with a function that just makes it hover in place?
        self.fly_towards_coord()
        #todo does this fight with the landing code ?

        #after one second of being active, arm the drone and switch it to offboard mode
        if self.counter == 10:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0) # 6 = Offboard
            self.get_logger().info("DRONE SWITCHING TO OFFBOARD MODE", once=True)

        #after that, tell the drone what to do (control loop)
        if self.counter > 10:
            if self.arrived:
                #if we have arrived at the destination, land the drone
                self.get_logger().info("DESTINATION REACHED, ATTEMPTING LANDING !!", once=True)
                self.land_the_drone()
            else:
                #if we have not arrived yet
                self.fly_towards_coord()
                if self.counter % 50 == 0:
                    self.get_logger().info("DRONE FLYING")

        #increment the timer counter
        self.counter += 1

    #-------------------------- CLASS HELPER FUNCTIONS ----------------------------
    #i.e. rely on info from self, rather than static args

    def fly_towards_coord(self):
        #at the moment this takes the target coord from the class attribute
        #can be easily modified to take it as an argument

        target_z = -self.TARGET_ALTITUDE

        if self.currentX is None:
            #have not yet heard from VehicleLocalPosition
            #really want to do nothing, but if we idle then PX4 hates us and times out
            #so tell the drone to go nowhere instead

            self.get_logger().warn("I don't know where my local position is !!!")

            #must negate the TARGET_ALTITUDE because PX4 uses 'altitude = 10m' to mean 10m underground (NED-Z coord system uses 'positive down')
            self.publish_trajectory_setpoint(0.0, 0.0, target_z)

            return

        xy_error = math.hypot(self.TARGET_X - self.currentX, self.TARGET_Y - self.currentY)
        z_error = abs(target_z - self.currentZ) if self.currentZ is not None else float('inf')
        if not self.arrived:
            self.arrived = (xy_error < self.ARR_RAD) and (z_error < 1.0)

        self.publish_trajectory_setpoint(self.TARGET_X, self.TARGET_Y, target_z)

    def land_the_drone(self):
        #read summary topic
        #need safe spot to be found
        #if safe spot found, move in direction of vector to safe spot
        #if length of that vector is very short, we are directly above the safe spot, so land

        target_z = -self.TARGET_ALTITUDE

        #we don't know our rotation
        if self.current_yaw_rad is None or self.current_pose_frame != VehicleOdometry.POSE_FRAME_NED:
            #hover in place
            self.publish_trajectory_setpoint(self.currentX, self.currentY, target_z)
            self.get_logger().warn("WARNING ! I DON'T KNOW MY CURRENT ROTATION YET", once=True)
            return

        #we have a safe spot and the vector we want to move towards
        if self.safe_spot_found and None not in self.vector_to_safe_spot:

            #vector to safe spot is in camera image pixels
            #convert to distance in metres
            dx_px = float(self.vector_to_safe_spot[0])
            dy_px = float(self.vector_to_safe_spot[1])
            pixel_distance = math.hypot(dx_px, dy_px)
            distance_m = pixel_distance * self.PIXEL_TO_METER_GAIN #bullshit factor :D

            #if we are close enough to actually fully just land straight down
            if distance_m <= self.CLOSE_ENOUGH_THRESHOLD:
                self.get_logger().info("OKAY NOW WE'RE ACTUALLY LANDING !", once=True)
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.landing_command_sent = True
            #we are not yet close enough, move in the direction of the vector
            else:
                #pixel vector -> metres relative to the drone
                body_right_m = dx_px * self.PIXEL_TO_METER_GAIN * self.IMAGE_X_TO_BODY_RIGHT_SIGN
                body_forward_m = dy_px * self.PIXEL_TO_METER_GAIN * self.IMAGE_Y_TO_BODY_FORWARD_SIGN

                body_distance = math.hypot(body_forward_m, body_right_m) #sophisticated pythagoras
                #only move at most MAX_LANDING_STEP_METRES so we don't fly around all over the place
                if body_distance > self.MAX_LANDING_STEP_METERS and body_distance > 0.0:
                    scale = self.MAX_LANDING_STEP_METERS / body_distance
                    body_forward_m *= scale
                    body_right_m *= scale

                #using that information, convert to world-space coordinate system using our current rotation
                yaw = self.current_yaw_rad
                north_delta = math.cos(yaw) * body_forward_m - math.sin(yaw) * body_right_m
                east_delta = math.sin(yaw) * body_forward_m + math.cos(yaw) * body_right_m

                landingTargetX = self.currentX + north_delta
                landingTargetY = self.currentY + east_delta

                #breath a sigh of relief and actually fucking publish this stupid fucking message
                self.publish_trajectory_setpoint(landingTargetX, landingTargetY, target_z)
                self.get_logger().info(
                    f"LANDING ALIGN: px=({dx_px:.1f},{dy_px:.1f}) dist~{distance_m:.2f}m "
                    f"-> target X={landingTargetX:.2f}, Y={landingTargetY:.2f}"
                )
        else:
            #we don't have a safe spot to land
            #hover in place and hope it will work (??)
            #todo make it move around a little bit
            self.publish_trajectory_setpoint(self.currentX, self.currentY, target_z)
            self.get_logger().warn("Safe landing spot not available yet; holding altitude.")


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