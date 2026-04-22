import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition

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
    TARGET_X = 10.0
    TARGET_Y = 20.0

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
        msg.position, msg.velocity, msg.acceleration = True, True, False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, vx, vy, z):
        msg = TrajectorySetpoint()
        #float('nan') means ignore this axis
        msg.velocity = [vx, vy, float('nan')] #we are only flying in the X/Y plane
        msg.position = [float('nan'), float('nan'), z] #and we are maintaining our Z position (altitude)
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

    #-------------------------- TIMER CALLBACK -> MAIN LOOP ----------------------------

    def timer_callback(self):
        #every time the timer ticks, reaffirm to PX4 that we are actually flying the drone so it doesnt auto-land
        self.publish_offboard_control_mode()

        #logic below assumes timer ticks every tenth of a second

        #after one second of being active, arm the drone and switch it to offboard mode
        if self.counter == 10:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0) # 6 = Offboard

        #after that, tell the drone what to do (control loop)
        if self.counter > 10:
            if self.arrived:
                #if we have arrived at the destination, land the drone
                self.get_logger().info("DESTINATION REACHED, DRONE LANDING !!", once=True)
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                raise SystemExit #stop this entire node running
            
            #if we have not landed:
            self.fly_towards_coord() #doesnt take any arguments AS OF YET

        #increment the timer counter
        self.counter += 1

    #-------------------------- CLASS HELPER FUNCTIONS ----------------------------
    #i.e. rely on info from self, rather than static args

    def fly_towards_coord(self):
        #at the moment this takes the target coord from the class attribute
        #can be easily modified to take it as an argument

        if self.currentX is None:
            #have not yet heard from VehicleLocalPosition
            #really want to do nothing, but if we idle then PX4 hates us and times out
            #so tell the drone to go nowhere instead

            self.get_logger().warn("I don't know where my local position is !!!")

            #must negate the TARGET_ALTITUDE because PX4 uses 'altitude = 10m' to mean 10m underground (NED-Z coord system uses 'positive down')
            self.publish_trajectory_setpoint(0.0, 0.0, -self.TARGET_ALTITUDE)

            return
        
        vx, vy, self.arrived = pos_to_velocities(self.currentX, self.currentY, self.TARGET_X, self.TARGET_Y, self.MOVEMENT_SPEED, self.ARR_RAD)

        #must negate the TARGET_ALTITUDE because PX4 uses 'altitude = 10m' to mean 10m underground (NED-Z coord system uses 'positive down')
        self.publish_trajectory_setpoint(vx, vy, -self.TARGET_ALTITUDE)

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