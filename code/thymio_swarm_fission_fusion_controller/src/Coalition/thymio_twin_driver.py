import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import time
import math

from extrema_radio import ExtremaRadio
from swarm_fsm import SwarmFSM

class ThymioTwinDriverNode(Node):
    def __init__(self):
        super().__init__('thymio_twin_driver')
        self.L = 0.095

        self.get_logger().info("Connecting to physical Thymio...")
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

        self.robot_id = "bot1"
        self.latest_virtual_prox = [0.0] * 24
        self.tracked_neighbors   = {}
        self.prev_target_left  = 0.0
        self.prev_target_right = 0.0
        self.incoming_radio_buffer = []
        
        self.radio_logic = ExtremaRadio(k_dim=50)
        self.fsm = SwarmFSM()
        self.G_E = 1.0 

        # ── ROS Setup ────────────────────────────────────────────────────────
        self.cmd_vel_publisher = self.create_publisher(Twist, f'/{self.robot_id}/cmd_vel', 10)
        self.radio_tx_pub = self.create_publisher(Float64MultiArray, f'/{self.robot_id}/radio_tx', 10)
        self.create_subscription(Float64MultiArray, f'/{self.robot_id}/virtual_prox', self.virtual_prox_callback, 10)
        self.create_subscription(Float64MultiArray, f'/{self.robot_id}/neighbors', self.neighbor_callback, 10)
        self.create_subscription(Float64MultiArray, f'/{self.robot_id}/radio_rx', self.radio_rx_callback, 10)

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
        if len(msg.data) > 0: self.incoming_radio_buffer.extend(msg.data)

    def radio_and_propagation_step(self):
        # NEW: Provide the number of tracked neighbors to lock vector broadcasts safely
        self.G_E, tx_data = self.radio_logic.step(
            self.incoming_radio_buffer, 
            self.fsm.current_state, 
            self.G_E, 
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
            prox_horizontal = self.th[self.node_id]["prox.horizontal"]
            closest_neighbor_dist = min([d["distance_cm"] for d in self.tracked_neighbors.values()] or [999.0])
            
            self.G_E = self.fsm.evaluate_fsm_transitions(
                self.G_E, closest_neighbor_dist, len(self.tracked_neighbors), self.get_logger()
            )

            a_x, a_y = self.fsm.get_continuous_avoidance_vector(prox_horizontal, self.latest_virtual_prox)
            net_x, net_y = a_x, a_y 
            is_near_virtual_wall = any(v > self.fsm.VIRTUAL_PROX_THRESHOLD for v in self.latest_virtual_prox)

            if self.fsm.current_state == 'RANDOM_WALK':
                w_x, w_y = self.fsm.get_wander_vector()
                net_x += w_x * 1.5; net_y += w_y * 1.5
                target_speed = self.fsm.CRUISE_SPEED * 2.0  
            elif self.fsm.current_state == 'FUSION':
                c_x, c_y = self.fsm.get_cohesion_vector(self.tracked_neighbors)
                net_x += c_x * 12.0; net_y += c_y * 12.0 
                target_speed = self.fsm.CRUISE_SPEED * 0.7  
            elif self.fsm.current_state == 'STAY':
                # NEW L-J BEHAVIOR: No longer strictly 0.0. The L-J potential gently adjusts positions
                lj_x, lj_y = self.fsm.get_lennard_jones_vector(self.tracked_neighbors)
                net_x += lj_x; net_y += lj_y
                target_speed = self.fsm.CRUISE_SPEED * 0.4  # Micro-adjustments 
            elif self.fsm.current_state == 'FISSION':
                s_x, s_y = self.fsm.get_dispersion_vector(self.tracked_neighbors)
                net_x += s_x * 5.0; net_y += s_y * 5.0
                target_speed = self.fsm.CRUISE_SPEED * 1.5

            magnitude = math.hypot(net_x, net_y)
            
            if magnitude > 0.1:
                h_err = math.atan2(net_y, net_x)
                effective_speed = target_speed if target_speed > 0 else (20.0 if magnitude > 2.0 else 0.0)

                if abs(h_err) < self.fsm.HEADING_DEADZONE_RAD:
                    rotation_offset = 0.0
                    forward_speed = effective_speed
                else:
                    if is_near_virtual_wall:
                        rotation_offset = max(-130.0, min(130.0, h_err * 40.0))
                        forward_speed = effective_speed * max(0.1, math.cos(h_err))
                    else:
                        rotation_offset = max(-30.0, min(30.0, h_err * 14.0))
                        forward_speed = effective_speed * max(0.7, math.cos(h_err))
                    
                    forward_speed = max(40.0 if effective_speed > 0 else 0.0, forward_speed)  

                raw_left  = forward_speed - rotation_offset
                raw_right = forward_speed + rotation_offset
            else:
                raw_left = target_speed; raw_right = target_speed

            self.target_left = int((self.fsm.SMOOTHING_FACTOR * raw_left) + ((1.0 - self.fsm.SMOOTHING_FACTOR) * self.prev_target_left))
            self.target_right = int((self.fsm.SMOOTHING_FACTOR * raw_right) + ((1.0 - self.fsm.SMOOTHING_FACTOR) * self.prev_target_right))
            self.prev_target_left, self.prev_target_right = self.target_left, self.target_right
            
            self.th[self.node_id]["motor.left.target"] = max(-300, min(300, self.target_left))
            self.th[self.node_id]["motor.right.target"] = max(-300, min(300, self.target_right))
            
            twist = Twist()
            twist.linear.x = ((self.target_right + self.target_left) / 2.0) * 0.0004
            twist.angular.z = ((self.target_right - self.target_left) / self.L) * 0.0004
            self.cmd_vel_publisher.publish(twist)

        except Exception as e: self.get_logger().error(f"Loop Error: {e}")

    def stop_robot(self):
        try:
            self.th[self.node_id]["motor.left.target"] = 0
            self.th[self.node_id]["motor.right.target"] = 0
            self.th.disconnect()
        except: pass

def main(args=None):
    rclpy.init(args=args)
    node = ThymioTwinDriverNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.stop_robot(); node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
