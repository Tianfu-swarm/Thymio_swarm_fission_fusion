import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import time
import math

from allocation_fsm import AllocationFSM
from extrema_radio import ExtremaRadio

class ThymioAllocationDriverNode(Node):
    def __init__(self):
        super().__init__('thymio_allocation_driver')
        self.L = 0.095

        self.get_logger().info("Connecting to physical Thymio for Allocation Task...")
        try:
            from thymiodirect import Connection, Thymio
            port = Connection.serial_default_port()
            self.th = Thymio(serial_port=port)
            self.th.connect()
            time.sleep(1.5)
            self.node_id = self.th.first_node()
        except Exception as e:
            self.get_logger().error(f"Hardware Connection Failed: {e}")
            return

        self.robot_id = "bot7"
        self.latest_virtual_prox = [0.0] * 24 
        self.tracked_neighbors = {}
        self.prev_target_left = 0.0
        self.prev_target_right = 0.0
        
        self.incoming_radio_buffer = []
        self.fsm = AllocationFSM()
        self.radio_logic = ExtremaRadio(k_dim=50)
        self.current_network_size = 1.0 
        
        self.last_stay_heard_time = 0.0 

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, f'/{self.robot_id}/cmd_vel', 10)
        self.radio_tx_pub = self.create_publisher(Float64MultiArray, f'/{self.robot_id}/radio_tx', 10) 
        
        # Subscribers
        self.create_subscription(Float64MultiArray, f'/{self.robot_id}/virtual_prox', self.virtual_prox_callback, 10) 
        self.create_subscription(Float64MultiArray, f'/{self.robot_id}/neighbors', self.neighbor_callback, 10)
        self.create_subscription(Float64MultiArray, f'/{self.robot_id}/radio_rx', self.radio_rx_callback, 10) 

        # Timers
        self.control_loop_timer = self.create_timer(0.05, self.execute_control_loop)
        self.radio_timer = self.create_timer(0.5, self.radio_and_propagation_step) 

    def virtual_prox_callback(self, msg):
        if len(msg.data) >= 24: self.latest_virtual_prox = msg.data

    def neighbor_callback(self, msg):
        self.tracked_neighbors.clear()
        idx = 1
        for i in range(0, len(msg.data), 2):
            self.tracked_neighbors[idx] = {"distance_cm": msg.data[i], "bearing_rad": msg.data[i + 1]}
            idx += 1

    def radio_rx_callback(self, msg):
        if len(msg.data) > 0: 
            self.incoming_radio_buffer.extend(msg.data)

    def radio_and_propagation_step(self):
        self.current_network_size, tx_data = self.radio_logic.step(
            self.incoming_radio_buffer, 
            self.fsm.current_state, 
            self.current_network_size, 
            len(self.tracked_neighbors),
            self.get_logger()
        )
        self.incoming_radio_buffer.clear()
        
        if tx_data:
            msg = Float64MultiArray()
            msg.data = tx_data
            self.radio_tx_pub.publish(msg)

    def execute_control_loop(self):
        try:
            current_time = time.time()
            prox_horizontal = self.th[self.node_id]["prox.horizontal"]
            ground_delta = self.th[self.node_id]["prox.ground.delta"]

            gray_value = self.fsm.process_ground_sensor(ground_delta)

            closest_neighbor_dist = min([d["distance_cm"] for d in self.tracked_neighbors.values()] or [999.0])
            
            stride = self.radio_logic.K_DIMENSION + 3
            num_msgs = len(self.incoming_radio_buffer) // stride
            for i in range(num_msgs):
                is_vec_flag = self.incoming_radio_buffer[i * stride + 2]
                if is_vec_flag == 1.0:
                    self.last_stay_heard_time = current_time
                    break
            
            stay_robot_heard = (current_time - self.last_stay_heard_time) < 1.0

            target_data = {'detected': False, 'is_follower': False, 'distance_m': 99.0, 'bearing_rad': 0.0}
            if self.tracked_neighbors:
                closest_idx = min(self.tracked_neighbors, key=lambda k: self.tracked_neighbors[k]['distance_cm'])
                closest = self.tracked_neighbors[closest_idx]
                
                target_data['detected'] = True
                target_data['distance_m'] = closest['distance_cm'] / 100.0
                target_data['bearing_rad'] = closest['bearing_rad']
                target_data['is_follower'] = (target_data['distance_m'] < 0.6) and stay_robot_heard

            state = self.fsm.evaluate_transitions(
                gray_value, 
                self.current_network_size, 
                current_time, 
                closest_neighbor_dist,
                target_data,
                self.get_logger()
            )

            val_left = min(max(ground_delta[0] / 1000.0, 0.0), 1.0) if len(ground_delta) >= 2 else 0.0
            val_right = min(max(ground_delta[1] / 1000.0, 0.0), 1.0) if len(ground_delta) >= 2 else 0.0
            is_boundary_recovery = (state == 'STAY' and (val_left < 0.40 or val_right < 0.40))

            if is_boundary_recovery:
                if val_left < 0.40 and val_right >= 0.40:
                    raw_left = 50.0   
                    raw_right = 10.0
                elif val_right < 0.40 and val_left >= 0.40:
                    raw_left = 10.0   
                    raw_right = 50.0
                else:
                    raw_left = 40.0   
                    raw_right = -40.0
            else:
                # Calculate normal forces (Now includes scaled-down smooth avoidance for FUSION/STAY)
                net_x, net_y = self.fsm.get_avoidance_vector(prox_horizontal, self.latest_virtual_prox, state)
                
                is_near_physical_wall = any(v > 3800 for v in prox_horizontal[:5])

                if state == 'RANDOM_WALK':
                    w_x, w_y = self.fsm.get_wander_vector()
                    net_x += w_x * 1.5; net_y += w_y * 1.5
                    target_speed = self.fsm.CRUISE_SPEED * 2.0  
                    
                elif state == 'FUSION':
                    t_x, t_y = self.fsm.get_target_tracking_vector(target_data)
                    net_x += t_x * 12.0; net_y += t_y * 12.0
                    
                    if target_data['detected'] and target_data['distance_m'] < 0.4:
                        target_speed = self.fsm.CRUISE_SPEED * 1.0  
                    else:
                        target_speed = self.fsm.CRUISE_SPEED * 2.0  

                elif state == 'STAY':
                    s_x, s_y = self.fsm.get_spring_vector(self.tracked_neighbors)
                    g_x, g_y = self.fsm.get_gray_retention_vector(ground_delta)
                    
                    net_x += s_x + g_x
                    net_y += s_y + g_y
                    target_speed = self.fsm.CRUISE_SPEED * 0.30  
                    
                elif state == 'FISSION':
                    w_x, w_y = self.fsm.get_wander_vector()
                    net_x += w_x * 2.0; net_y += w_y * 2.0
                    target_speed = self.fsm.CRUISE_SPEED * 1.5

                magnitude = math.hypot(net_x, net_y)
                if magnitude > 0.1:
                    h_err = math.atan2(net_y, net_x)
                    effective_speed = target_speed if target_speed > 0 else (20.0 if magnitude > 2.0 else 0.0)

                    if abs(h_err) < self.fsm.HEADING_DEADZONE_RAD:
                        rotation_offset = 0.0
                        forward_speed = effective_speed
                    else:
                        # Determine if we need an AGGRESSIVE emergency turn
                        if state in ['FUSION', 'STAY']:
                            # Only physical walls cause emergency turns; virtual peers use smooth avoidance.
                            emergency = is_near_physical_wall
                        else:
                            is_near_virtual_wall = any(v > self.fsm.VIRTUAL_PROX_THRESHOLD for v in self.latest_virtual_prox)
                            emergency = is_near_virtual_wall or is_near_physical_wall

                        if emergency:
                            rotation_offset = max(-130.0, min(130.0, h_err * 40.0))
                            forward_speed = effective_speed * max(0.1, math.cos(h_err))
                        else:
                            # Smooth, gentle avoidance rotation limit
                            rotation_offset = max(-60.0, min(60.0, h_err * 35.0))
                            forward_speed = effective_speed * max(0.1, math.cos(h_err))
                        
                        min_clamp = 0.0 if state == 'STAY' else 40.0
                        forward_speed = max(min_clamp if effective_speed > 0 else 0.0, forward_speed)

                    raw_left  = forward_speed - rotation_offset
                    raw_right = forward_speed + rotation_offset
                else:
                    raw_left = target_speed
                    raw_right = target_speed

            smoothing = 0.25
            self.target_left = int((smoothing * raw_left) + ((1.0 - smoothing) * self.prev_target_left))
            self.target_right = int((smoothing * raw_right) + ((1.0 - smoothing) * self.prev_target_right))
            self.prev_target_left, self.prev_target_right = self.target_left, self.target_right
            
            self.th[self.node_id]["motor.left.target"] = max(-300, min(300, self.target_left))
            self.th[self.node_id]["motor.right.target"] = max(-300, min(300, self.target_right))
            
            twist = Twist()
            twist.linear.x = ((self.target_right + self.target_left) / 2.0) * 0.0004
            twist.angular.z = ((self.target_right - self.target_left) / self.L) * 0.0004
            self.cmd_vel_publisher.publish(twist)

        except Exception as e:
            self.get_logger().error(f"Loop Error: {e}")

    def stop_robot(self):
        try:
            self.th[self.node_id]["motor.left.target"] = 0
            self.th[self.node_id]["motor.right.target"] = 0
            self.th.disconnect()
        except: pass

def main(args=None):
    rclpy.init(args=args)
    node = ThymioAllocationDriverNode()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: 
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': 
    main()