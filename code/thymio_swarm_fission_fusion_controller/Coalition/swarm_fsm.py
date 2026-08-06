import math
import random

class SwarmFSM:
    def __init__(self):
        self.current_state = 'RANDOM_WALK'
        self.state_timer = 0.0
        self.OPTIMAL_GROUP_SIZE = 2.5  
        self.ALPHA = 5.0  
        self.T_0 = 20.0    
        self.wander_angle = random.uniform(-math.pi, math.pi)
        
        self.CRUISE_SPEED         = 120.0    
        self.AVOIDANCE_MULTIPLIER = 450.0    
        self.SMOOTHING_FACTOR     = 0.25     
        self.HEADING_DEADZONE_RAD = 0.12     
        self.PHYSICAL_PROX_THRESHOLD = 3200  
        self.VIRTUAL_PROX_THRESHOLD  = 0.02

    def get_continuous_avoidance_vector(self, prox_horizontal, latest_virtual_prox):
        accum_x, accum_y = 0.0, 0.0
        for i in range(24):
            val = latest_virtual_prox[i]
            if val > self.VIRTUAL_PROX_THRESHOLD:
                angle_deg = (7.5 + (i * 15.0)) if i < 12 else (-172.5 + ((i - 12) * 15.0))
                rad = math.radians(angle_deg) + math.pi
                weight = ((val - self.VIRTUAL_PROX_THRESHOLD) ** 2) * self.AVOIDANCE_MULTIPLIER
                accum_x += weight * math.cos(rad); accum_y += weight * math.sin(rad)
                
        if self.current_state in ['RANDOM_WALK', 'FISSION']:
            phys_angles = {0: 40, 1: 20, 2: 0, 3: -20, 4: -40}
            for idx, angle_deg in phys_angles.items():
                val = prox_horizontal[idx]
                if val > self.PHYSICAL_PROX_THRESHOLD:
                    rad = math.radians(angle_deg) + math.pi
                    weight = (val - self.PHYSICAL_PROX_THRESHOLD) * 0.25
                    accum_x += weight * math.cos(rad)
                    accum_y += weight * math.sin(rad)
                    
        return (accum_x, accum_y)

    def get_wander_vector(self):
        if random.random() < 0.005: 
            self.wander_angle = random.uniform(-math.pi, math.pi)
        self.wander_angle += random.uniform(-0.02, 0.02)
        return (math.cos(self.wander_angle), math.sin(self.wander_angle))

    def get_cohesion_vector(self, tracked_neighbors):
        if not tracked_neighbors: return (0.0, 0.0)
        ax, ay = 0.0, 0.0
        for d in tracked_neighbors.values():
            ax += math.cos(d["bearing_rad"]); ay += math.sin(d["bearing_rad"])
        return (ax, ay)

    def get_dispersion_vector(self, tracked_neighbors):
        if not tracked_neighbors: return (0.0, 0.0)
        ax, ay = 0.0, 0.0
        for d in tracked_neighbors.values():
            dist = max(d["distance_cm"], 1.0)
            force = 150.0 / dist
            ax += force * math.cos(d["bearing_rad"] + math.pi); ay += force * math.sin(d["bearing_rad"] + math.pi)
        return (ax, ay)

    def get_lennard_jones_vector(self, tracked_neighbors):
        """Calculates a simplified Lennard-Jones force to maintain a comfortable cluster distance."""
        if not tracked_neighbors: return (0.0, 0.0)
        ax, ay = 0.0, 0.0
        equilibrium_dist = 24.0  
        
        for d in tracked_neighbors.values():
            dist = max(d["distance_cm"], 1.0)
            angle = d["bearing_rad"]
            
            ratio = equilibrium_dist / dist
            force = 250.0 * ((ratio ** 2) - (ratio ** 4))
            
            ax += force * math.cos(angle)
            ay += force * math.sin(angle)
        return (ax, ay)

    # ── STRICT FSM LOGIC (With Core Group Preservation) ──

    def evaluate_fsm_transitions(self, G_E, closest_neighbor_dist, num_neighbors, logger):
        self.state_timer += 0.05
        
        if num_neighbors == 0:
            G_E = 1.0

        if self.current_state == 'RANDOM_WALK':
            # Table: Random_Walk -> Fusion (find target)
            if num_neighbors > 0:
                self.current_state = 'FUSION'
                self.state_timer = 0.0

        elif self.current_state == 'FUSION':
            # LAYER 1 PROTECTION: Predictive check before entry
            # If target cluster size + myself exceeds capacity, abort and run away immediately!
            if (G_E + 1.0) > self.OPTIMAL_GROUP_SIZE: 
                self.current_state = 'FISSION'
                self.state_timer = 0.0
                logger.info(f"[FSM] Fusion Aborted (Predictive Overcrowding): Target G_E ({G_E:.2f}) + 1 > Optimal ({self.OPTIMAL_GROUP_SIZE})")
            
            # Table: Fusion -> Stay (close to target)
            elif closest_neighbor_dist < 22.0:  
                self.current_state = 'STAY'
                self.state_timer = 0.0
                
            # Failsafe: Lost neighbors
            elif num_neighbors == 0:
                self.current_state = 'RANDOM_WALK'
                self.state_timer = 0.0

        elif self.current_state == 'STAY':
            # Calculate dynamic countdown timer
            waiting_time = (self.ALPHA * G_E) + self.T_0
            
            # Table: Stay -> Fission (G^E > G*)
            if G_E > self.OPTIMAL_GROUP_SIZE:
                # LAYER 2 PROTECTION: Failsafe filter
                # Only the newcomer (lowest timer) peels off. Core pair handles it seamlessly.
                if self.state_timer < 3.0: 
                    self.current_state = 'FISSION'
                    self.state_timer = 0.0
                    logger.info(f"*** NEWCOMER FISSION TRIGGERED! G_E ({G_E:.2f}) > Optimal ({self.OPTIMAL_GROUP_SIZE}) ***")
                else:
                    # Core group units have timers > 3.0s, so they remain safely in STAY together
                    pass
                
            # Table: Stay -> Fission (waiting time exceeded)
            elif self.state_timer > waiting_time:
                self.current_state = 'FISSION'
                self.state_timer = 0.0
                logger.info(f"[FSM] Stay Timer Exceeded ({waiting_time:.1f}s). Leaving group via FISSION.")
                
            # Failsafe: Lost neighbors
            elif num_neighbors == 0:
                self.current_state = 'RANDOM_WALK'
                self.state_timer = 0.0

        elif self.current_state == 'FISSION':
            # Table: Fission -> Random_Walk (far from origin)
            if num_neighbors == 0:
                self.current_state = 'RANDOM_WALK'
                G_E = 1.0
                self.state_timer = 0.0
                
        return G_E
