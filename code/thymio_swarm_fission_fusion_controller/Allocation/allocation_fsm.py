import math
import random
import time

class AllocationFSM:
    """
    Finite State Machine managing task allocation, state transitions,
    and vector-based navigation forces (Random Walk, Fusion, Stay, Fission).
    """
    def __init__(self):
        # State tracking and timers
        self.current_state = 'RANDOM_WALK'
        self.stay_start_time = 0.0
        self.fission_start_time = 0.0  
        self.last_letter_seen_time = 0.0
        self.wait_time = 0.0
        
        # Navigation & Avoidance Parameters
        self.CRUISE_SPEED = 120.0    
        self.AVOIDANCE_MULTIPLIER = 450.0
        self.PHYSICAL_PROX_THRESHOLD = 3200 
        self.WAITING_TIME_SCALE_FACTOR = 60.0
        
        # Virtual Proximity & Turning Deadzone
        self.VIRTUAL_PROX_THRESHOLD = 0.02
        self.HEADING_DEADZONE_RAD = 0.12
        
        # Subgroup Allocation Configuration
        self.desired_subgroup_size = 100
        self.spring_distance = 50.0  
        self.estimated_group_size = 1.0
        self.initial_group_size = 1.0
        
        # Initial wander heading
        self.wander_angle = random.uniform(-math.pi, math.pi)

    def process_ground_sensor(self, ground_delta):
        """
        Processes left/right ground IR sensors to map grayscale brightness
        to target subgroup capacity and inter-robot spacing.
        """
        if not ground_delta or len(ground_delta) < 2:
            return 0.0  

        # Normalize ground sensor values to [0.0, 1.0]
        val_left = min(max(ground_delta[0] / 1000.0, 0.0), 1.0)
        val_right = min(max(ground_delta[1] / 1000.0, 0.0), 1.0)

        # Detect valid floor target (grayscale threshold)
        if val_left >= 0.15 and val_right >= 0.15:
            avg_gray = (val_left + val_right) / 2.0
            
            # Map floor brightness to required subgroup sizes
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
            # Reset desired subgroup size when off target
            if self.current_state != 'STAY':
                self.desired_subgroup_size = 100  
                
            return min(val_left, val_right)

    def evaluate_transitions(self, gray_value, current_group_size, current_time, 
                             closest_neighbor_dist, target_data, logger):
        """
        Evaluates state transition triggers based on floor perception,
        estimated collective group size, timers, and proximity.
        """
        self.estimated_group_size = current_group_size
        actual_group_size = round(self.estimated_group_size)

        # ── STATE: RANDOM_WALK ──
        if self.current_state == 'RANDOM_WALK':
            if gray_value >= 0.15:  
                logger.info(f"Letter detected (Gray: {gray_value:.2f}). Transitioning to STAY.")
                self.initial_group_size = 1
                self.stay_start_time = current_time
                self.last_letter_seen_time = current_time  
                self.wait_time = self.WAITING_TIME_SCALE_FACTOR * self.initial_group_size
                self.current_state = 'STAY'
            elif target_data['detected'] and target_data['is_follower']:
                logger.info("Follower target detected. Transitioning to FUSION.")
                self.current_state = 'FUSION'

        # ── STATE: FUSION ──
        elif self.current_state == 'FUSION':
            if gray_value >= 0.15:
                logger.info("Reached letter while following. Transitioning to STAY.")
                self.initial_group_size = actual_group_size
                self.stay_start_time = current_time
                self.last_letter_seen_time = current_time  
                self.wait_time = self.WAITING_TIME_SCALE_FACTOR * self.initial_group_size
                self.current_state = 'STAY'
            elif not target_data['detected'] or not target_data['is_follower'] or actual_group_size > self.desired_subgroup_size:
                logger.info("Target lost, no STAY broadcast heard, or group oversized. Transitioning to RANDOM_WALK.")
                self.current_state = 'RANDOM_WALK'

        # ── STATE: STAY ──
        elif self.current_state == 'STAY':
            # Timeout check: return to search if letter visual lost for > 20s
            if gray_value >= 0.15:
                self.last_letter_seen_time = current_time
            elif (current_time - self.last_letter_seen_time) > 20.0:
                logger.info("Letter lost for 20 seconds. Transitioning to RANDOM_WALK.")
                self.current_state = 'RANDOM_WALK'
                return self.current_state

            # Capacity evaluation & timer recalculation
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

                # Transition to FISSION once retention wait time expires
                if (current_time - self.stay_start_time) > self.wait_time:
                    logger.info(f"Wait time ran out. Transitioning to FISSION.  ({self.wait_time}s).")
                    self.fission_start_time = current_time
                    self.current_state = 'FISSION'
                    
            else:
                logger.info(f"Overcrowded. Transitioning to FISSION. : {actual_group_size} > {self.desired_subgroup_size}")
                self.fission_start_time = current_time
                self.current_state = 'FISSION'

        # ── STATE: FISSION ──
        elif self.current_state == 'FISSION':
            if (current_time - self.fission_start_time) < 5.0:
                pass
            elif closest_neighbor_dist > 50.0 and gray_value < 0.15:  
                logger.info("Separation complete or isolated. Transitioning to RANDOM_WALK.")
                self.current_state = 'RANDOM_WALK'

        return self.current_state

    def get_wander_vector(self):
        """Generates smooth, correlated random walk direction vectors."""
        if random.random() < 0.02: 
            self.wander_angle = random.uniform(-math.pi, math.pi)
        self.wander_angle += random.uniform(-0.05, 0.05)
        return math.cos(self.wander_angle), math.sin(self.wander_angle)

    def get_target_tracking_vector(self, target_data):
        """Calculates orbital attraction and tangent approach forces toward target peers."""
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
        """Generates virtual spring attraction/repulsion forces for spacing within group."""
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
        """Generates forward attraction vector to remain centered on gray target zones."""
        if not ground_delta or len(ground_delta) < 2:
            return 4.0, 0.0

        w_left = min(max(ground_delta[0] / 1000.0, 0.0), 1.0)
        w_right = min(max(ground_delta[1] / 1000.0, 0.0), 1.0)

        if w_left >= 0.15 and w_right >= 0.15:
            return 4.0, 0.0

        return 0.0, 0.0

    def get_avoidance_vector(self, prox_horizontal, latest_virtual_prox, current_state):
        """Computes collision avoidance vectors using physical IR and virtual proximity sensors."""
        accum_x, accum_y = 0.0, 0.0
        
        # Configure state-dependent virtual sensor thresholds and gain weights
        if current_state == 'STAY':
            v_thresh = 0.30 
            v_mult = 700.0   
        elif current_state == 'FUSION':
            v_thresh = 0.05 
            v_mult = 80.0
        else:
            v_thresh = self.VIRTUAL_PROX_THRESHOLD
            v_mult = self.AVOIDANCE_MULTIPLIER
        
        # 1. Accumulate virtual proximity forces (from radio/RAB medium)
        for i in range(24):
            val = latest_virtual_prox[i]
            if val > v_thresh:
                angle_deg = (7.5 + (i * 15.0)) if i < 12 else (-172.5 + ((i - 12) * 15.0))
                rad = math.radians(angle_deg) + math.pi
                weight = ((val - v_thresh) ** 2) * v_mult
                accum_x += weight * math.cos(rad)
                accum_y += weight * math.sin(rad)
                
        # 2. Accumulate physical IR proximity forces
        phys_angles = {0: 40, 1: 20, 2: 0, 3: -20, 4: -40}
        for idx, angle_deg in phys_angles.items():
            val = prox_horizontal[idx]
            if val > self.PHYSICAL_PROX_THRESHOLD:
                rad = math.radians(angle_deg) + math.pi
                weight = (val - self.PHYSICAL_PROX_THRESHOLD) * 0.25
                accum_x += weight * math.cos(rad)
                accum_y += weight * math.sin(rad)
                
        return accum_x, accum_y