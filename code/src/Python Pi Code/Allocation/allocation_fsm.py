import math
import random
import time

class AllocationFSM:
    def __init__(self):
        self.current_state = 'RANDOM_WALK'
        self.state_timer = 0.0
        self.stay_start_time = 0.0
        self.fission_start_time = 0.0  
        self.last_letter_seen_time = 0.0
        self.wait_time = 0.0
        
        # Configuration
        self.CRUISE_SPEED = 120.0    
        self.AVOIDANCE_MULTIPLIER = 450.0
        self.PHYSICAL_PROX_THRESHOLD = 3200 
        self.WAITING_TIME_SCALE_FACTOR = 60.0
        
        # Virtual Wall Configuration
        self.VIRTUAL_PROX_THRESHOLD = 0.02
        self.HEADING_DEADZONE_RAD = 0.12
        
        # Allocation variables
        self.desired_subgroup_size = 100
        self.spring_distance = 50.0  
        self.estimated_group_size = 1.0
        self.initial_group_size = 1.0
        
        self.wander_angle = random.uniform(-math.pi, math.pi)

    def process_ground_sensor(self, ground_delta):
        if not ground_delta or len(ground_delta) < 2:
            return 0.0  

        val_left = min(max(ground_delta[0] / 1000.0, 0.0), 1.0)
        val_right = min(max(ground_delta[1] / 1000.0, 0.0), 1.0)

        if val_left >= 0.15 and val_right >= 0.15:
            avg_gray = (val_left + val_right) / 2.0
            
            if self.current_state != 'STAY':
                if 0.15 <= avg_gray <= 0.40:
                    self.desired_subgroup_size = 2   
                    self.spring_distance = 42.0
                elif 0.40 < avg_gray <= 0.95:      
                    self.desired_subgroup_size = 3    
                    self.spring_distance = 55.0
                else:
                    self.desired_subgroup_size = 4    
                    self.spring_distance = 60.0
                    
            return avg_gray
        else:
            if self.current_state != 'STAY':
                self.desired_subgroup_size = 100  
                
            return min(val_left, val_right)

    def evaluate_transitions(self, gray_value, current_group_size, current_time, 
                             closest_neighbor_dist, target_data, logger):
        self.estimated_group_size = current_group_size
        actual_group_size = round(self.estimated_group_size)

        if self.current_state == 'RANDOM_WALK':
            if gray_value >= 0.15:  
                logger.info(f"Letter detected (Gray: {gray_value:.2f}). Transitioning to STAY.")
                self.initial_group_size = 1
                self.stay_start_time = current_time
                self.last_letter_seen_time = current_time  # <-- Initialize timer
                self.wait_time = self.WAITING_TIME_SCALE_FACTOR * self.initial_group_size
                self.current_state = 'STAY'
            elif target_data['detected'] and target_data['is_follower']:
                logger.info("Follower target detected. Transitioning to FUSION.")
                self.current_state = 'FUSION'

        elif self.current_state == 'FUSION':
            if gray_value >= 0.15:
                logger.info("Reached letter while following. Transitioning to STAY.")
                self.initial_group_size = actual_group_size
                self.stay_start_time = current_time
                self.last_letter_seen_time = current_time  # <-- Initialize timer
                self.wait_time = self.WAITING_TIME_SCALE_FACTOR * self.initial_group_size
                self.current_state = 'STAY'
            elif not target_data['detected'] or not target_data['is_follower'] or actual_group_size > self.desired_subgroup_size:
                logger.info("Target lost, no STAY broadcast heard, or group oversized. Transitioning to RANDOM_WALK.")
                self.current_state = 'RANDOM_WALK'

        elif self.current_state == 'STAY':
            # ── NEW TIMEOUT LOGIC ──
            if gray_value >= 0.15:
                self.last_letter_seen_time = current_time
            elif (current_time - self.last_letter_seen_time) > 20.0:
                logger.info("Letter lost for 10 seconds. Transitioning to RANDOM_WALK.")
                self.current_state = 'RANDOM_WALK'
                return self.current_state

            # ── EXISTING GROUP SIZE LOGIC ──
            if actual_group_size <= self.desired_subgroup_size:
                if actual_group_size != self.initial_group_size:
                    if actual_group_size > self.initial_group_size:
                        self.initial_group_size = actual_group_size
                        self.wait_time = self.WAITING_TIME_SCALE_FACTOR * self.initial_group_size
                        self.stay_start_time = current_time
                    else:
                        elapsed = current_time - self.stay_start_time
                        remaining = self.wait_time - elapsed
                        new_wait = self.WAITING_TIME_SCALE_FACTOR * actual_group_size
                        if new_wait < remaining:
                            self.wait_time = new_wait
                            self.stay_start_time = current_time
                        self.initial_group_size = actual_group_size

                if (current_time - self.stay_start_time) > self.wait_time:
                    logger.info(f"Wait time ran out. Transitioning to FISSION.  ({self.wait_time}s).")
                    self.fission_start_time = current_time
                    self.current_state = 'FISSION'
                    
            else:
                logger.info(f"Overcrowded. Transitioning to FISSION. : {actual_group_size} > {self.desired_subgroup_size}")
                self.fission_start_time = current_time
                self.current_state = 'FISSION'

        elif self.current_state == 'FISSION':
            if (current_time - self.fission_start_time) < 5.0:
                pass
            elif closest_neighbor_dist > 50.0 and gray_value < 0.15:  
                logger.info("Separation complete or isolated. Transitioning to RANDOM_WALK.")
                self.current_state = 'RANDOM_WALK'

        return self.current_state

    def get_wander_vector(self):
        if random.random() < 0.02: 
            self.wander_angle = random.uniform(-math.pi, math.pi)
        self.wander_angle += random.uniform(-0.05, 0.05)
        return math.cos(self.wander_angle), math.sin(self.wander_angle)

    def get_target_tracking_vector(self, target_data):
        if not target_data['detected']:
            return 0.0, 0.0
            
        bearing = target_data['bearing_rad']
        dist = target_data['distance_m']
        
        orbit_radius = 0.07  
        
        if dist > 0.4:
            return math.cos(bearing), math.sin(bearing)
            
        radial_force = (dist - orbit_radius) * 3.0 
        tangent_force = 2.0  
        
        tangent_bearing = bearing - (math.pi / 2.0)
        
        f_x = (math.cos(bearing) * radial_force) + (math.cos(tangent_bearing) * tangent_force)
        f_y = (math.sin(bearing) * radial_force) + (math.sin(tangent_bearing) * tangent_force)
        
        mag = math.hypot(f_x, f_y)
        if mag > 0:
            f_x /= mag
            f_y /= mag
            
        return f_x, f_y

    def get_spring_vector(self, tracked_neighbors):
        if not tracked_neighbors: return 0.0, 0.0
        fx, fy = 0.0, 0.0
        k = 1.0
        
        for d in tracked_neighbors.values():
            dist = max(d["distance_cm"] * 10.0, 1.0) 
            force = k * (dist - self.spring_distance)
            fx += force * math.cos(d["bearing_rad"])
            fy += force * math.sin(d["bearing_rad"])
            
        return fx * 0.05, fy * 0.05  

    def get_gray_retention_vector(self, ground_delta):
        if not ground_delta or len(ground_delta) < 2:
            return 4.0, 0.0

        w_left = min(max(ground_delta[0] / 1000.0, 0.0), 1.0)
        w_right = min(max(ground_delta[1] / 1000.0, 0.0), 1.0)

        if w_left >= 0.15 and w_right >= 0.15:
            return 4.0, 0.0

        return 0.0, 0.0

    def get_avoidance_vector(self, prox_horizontal, latest_virtual_prox, current_state):
        accum_x, accum_y = 0.0, 0.0
        
        # ── DEDICATED VIRTUAL COLLISION AVOIDANCE ──
        if current_state == 'STAY':
            v_thresh = 0.30 
            v_mult = 700.0   
        elif current_state == 'FUSION':
            # Weaker avoidance while traveling towards the group
            v_thresh = 0.05 
            v_mult = 80.0
        else:
            # Aggressive default avoidance for RANDOM_WALK and FISSION
            v_thresh = self.VIRTUAL_PROX_THRESHOLD
            v_mult = self.AVOIDANCE_MULTIPLIER
        
        # 1. Evaluate Virtual Sensors (Radio/RAB)
        for i in range(24):
            val = latest_virtual_prox[i]
            if val > v_thresh:
                # Calculate angle for 24 virtual sensors
                angle_deg = (7.5 + (i * 15.0)) if i < 12 else (-172.5 + ((i - 12) * 15.0))
                rad = math.radians(angle_deg) + math.pi
                
                # Apply the state-specific multiplier
                weight = ((val - v_thresh) ** 2) * v_mult
                accum_x += weight * math.cos(rad)
                accum_y += weight * math.sin(rad)
                
        # 2. Evaluate Physical Sensors (IR Proximity)
        phys_angles = {0: 40, 1: 20, 2: 0, 3: -20, 4: -40}
        for idx, angle_deg in phys_angles.items():
            val = prox_horizontal[idx]
            if val > self.PHYSICAL_PROX_THRESHOLD:
                rad = math.radians(angle_deg) + math.pi
                weight = (val - self.PHYSICAL_PROX_THRESHOLD) * 0.25
                accum_x += weight * math.cos(rad)
                accum_y += weight * math.sin(rad)
                
        return accum_x, accum_y