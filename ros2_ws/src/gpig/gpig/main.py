import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition
from geometry_msgs.msg import Point

import math

#-------------------------- NON-CLASS HELPER FUNCTIONS ----------------------------
#i.e. don't need any information about the drone itself

def pos_to_velocities(currentX, currentY, targetX, targetY, speed, arrivalRadius):
    #given the drone's current X and Y coordinates, and a target X and Y, finds the velocity vector from us to the target
    #takes movement speed as an arg
    #and arrivalRadius, so that we can slow down when nearing the destination and know when we have reached it
    #returns vx: float, vy: float, arrived: bool
    #assuming vx is north component and vy is east component

    dx = targetX - currentX
    dy = targetY - currentY
    dist = math.sqrt(dx * dx + dy * dy) #cheeky pythagoras

    if dist < arrivalRadius:
        #we have reached the destination
        return 0.0, 0.0, True

    slowdownRadiuses = 5 #begin to slow down when we are within this many arrivalRadiuses of the destination
    slowdown = min(1.0, (dist / (arrivalRadius * slowdownRadiuses)))
    vx = (dx / dist) * speed * slowdown
    vy = (dy / dist) * speed * slowdown
    return vx, vy, False

class OffboardControl(Node):

    #target x,y coordinates (these will come from the server and we need some method of recieveing / setting them)
    TARGET_X = 0.0
    TARGET_Y = 0.0 - 20

    TARGET_ALTITUDE = 10.0 #the altitude we want to be flying around at
    MOVEMENT_SPEED = 5.0 #normal movement speed in m/s
    ARR_RAD = 0.5 #how many metres away from the target we have to be, to consider ourselves to be there

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
        self.create_subscription(Point, '/ouranos/destination', self._on_destination, 10)


        #CALLBACKS AND OTHER VARIABLES NOT KNOWN AT COMPILE TIME
        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz
        self.counter = 0

        self.arrived = False
        #position unknown until we hear from VehicleLocalPosition
        self.currentX = None
        self.currentY = None
        self.currentZ = None

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

    def _on_destination(self, msg: Point):
        """Callback fired when the Ouranos dashboard sends a new destination."""
        self.TARGET_X = float(msg.x)
        self.TARGET_Y = float(msg.y)
        # Reset arrival flag so the drone re-plans toward the new target
        self.arrived = False
        self.get_logger().info(
            f"New destination received from Ouranos: "
            f"TARGET_X={self.TARGET_X}, TARGET_Y={self.TARGET_Y}"
        )


    #-------------------------- TIMER CALLBACK -> MAIN LOOP ----------------------------

    def timer_callback(self):
        #every time the timer ticks, reaffirm to PX4 that we are actually flying the drone so it doesnt auto-land
        self.publish_offboard_control_mode()

        #always stream a valid setpoint before and during offboard mode
        self.fly_towards_coord()

        #logic below assumes timer ticks every tenth of a second

        #after one second of being active, arm the drone and switch it to offboard mode
        if self.counter == 10:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0) # 6 = Offboard
            self.get_logger().info("DRONE SWITCHING TO OFFBOARD MODE", once=True)

        #after that, tell the drone what to do (control loop)
        if self.counter > 10:
            if self.arrived:
                #if we have arrived at the destination, land the drone
                self.get_logger().info("DESTINATION REACHED, DRONE LANDING !!", once=True)
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                raise SystemExit #stop this entire node running

            #if we have not landed:
            if self.counter % 10 == 0:
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
        self.arrived = (xy_error < self.ARR_RAD) and (z_error < 1.0)

        self.publish_trajectory_setpoint(self.TARGET_X, self.TARGET_Y, target_z)

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