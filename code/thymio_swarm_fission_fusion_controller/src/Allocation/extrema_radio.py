import math
import random
from collections import deque

class ExtremaRadio:
    """
    Implements decentralized swarm size estimation using Min-Vector consensus 
    (Extrema Propagation Algorithm) over simulated radio media.
    """
    def __init__(self, k_dim=50):
        self.K_DIMENSION = k_dim
        self.current_round_id = 1.0
        self.x = []
        self.N_history = deque(maxlen=5)
        self.smooth_history = []
        self.stability_window = 5
        self.early_converge_window = 3
        
        self.required_propagation_hops = 3
        self.propagation_hops = 0
        self.has_started_convergence = False

    def initialize_vector(self):
        """Initializes local minimum vector with exponentially distributed random values."""
        self.x = [random.expovariate(1.0) for _ in range(self.K_DIMENSION)]

    def estimate_group_size_extrema(self):
        """Computes network/group size estimate G_E from the sum of vector elements."""
        if not self.x or sum(self.x) == 0.0: return 0.0
        return (self.K_DIMENSION - 1) / sum(self.x)

    def apply_smoothed_estimate(self, new_estimate):
        """Applies exponential moving average filter over historical size estimates."""
        window_size = 5
        size_decay = 0.8
        
        if len(self.smooth_history) >= window_size:
            self.smooth_history.pop(0)
        self.smooth_history.append(new_estimate)
        
        weighted_sum = 0.0
        weight_total = 0.0
        for i in range(len(self.smooth_history)):
            weight = math.pow(size_decay, len(self.smooth_history) - i - 1)
            weighted_sum += weight * self.smooth_history[i]
            weight_total += weight
            
        return weighted_sum / weight_total

    def step(self, buffered_data, current_state, current_G_E, num_neighbors_physical, logger):
        """
        Executes one radio iteration: unpacks incoming packets, updates min-vectors,
        checks round synchronization, and returns updated estimate + outgoing payload.
        """
        epsilon = 0.05
        stride = self.K_DIMENSION + 3  # Header (round, size, is_vec_flag) + K vectors
        
        num_neighbors_radio = len(buffered_data) // stride
        heard_sizes = []
        valid_vectors = []

        # Parse raw floating-point buffer
        for i in range(num_neighbors_radio):
            base = i * stride
            rx_round = buffered_data[base]
            rx_size = buffered_data[base + 1]
            is_vec = buffered_data[base + 2]
            rx_vec = buffered_data[base + 3 : base + stride]

            if rx_size > 0:
                heard_sizes.append(rx_size)
            if rx_round > 0 and is_vec == 1.0: 
                valid_vectors.append((rx_round, rx_vec))

        # ── MODULE 1: FISSION STATE ──
        if current_state == 'FISSION':
            return current_G_E, [0.0, 1.0, 0.0] + [0.0] * self.K_DIMENSION

        # ── MODULE 2: RANDOM WALK & FUSION STATES ──
        if current_state in ['RANDOM_WALK', 'FUSION']:
            if heard_sizes: current_G_E = max(heard_sizes)
            else: current_G_E = 1.0
            
            self.x.clear()
            self.N_history.clear()
            self.propagation_hops = 0
            self.has_started_convergence = False
            
            return current_G_E, [0.0, 1.0, 0.0] + [0.0] * self.K_DIMENSION

        # ── MODULE 3: STAY STATE (Consensus Engine) ──
        if current_state == 'STAY':
            if num_neighbors_physical == 0:
                return current_G_E, [0.0, current_G_E, 0.0] + [0.0] * self.K_DIMENSION

            if not self.x: self.initialize_vector()
            x_changed = False
            should_sync = False
            sync_to = -1.0

            # Element-wise Min-Vector consensus computation
            for rx_round, rx_vec in valid_vectors:
                if rx_round > self.current_round_id:
                    if not should_sync or rx_round > sync_to:
                        should_sync = True; sync_to = rx_round
                    continue
                if rx_round != self.current_round_id: continue
                if not all(math.isfinite(v) and v > 0.0 for v in rx_vec): continue
                
                for k in range(self.K_DIMENSION):
                    if rx_vec[k] < self.x[k]:
                        self.x[k] = rx_vec[k]
                        x_changed = True

            N = self.estimate_group_size_extrema()
            self.N_history.append(N)

            # Check for early local vector convergence
            if not self.has_started_convergence and len(self.N_history) >= self.early_converge_window:
                early_converged = True
                hist_list = list(self.N_history)
                start_idx = len(hist_list) - self.early_converge_window
                for i in range(start_idx + 1, len(hist_list)):
                    if abs(hist_list[i] - hist_list[i - 1]) > epsilon:
                        early_converged = False
                        break
                if early_converged:
                    self.has_started_convergence = True

            # Sync round ID if higher round received from neighboring peers
            if should_sync and sync_to > self.current_round_id and self.has_started_convergence:
                self.current_round_id = float(sync_to)
                self.initialize_vector()
                self.has_started_convergence = False
                self.propagation_hops = 0
                self.N_history.clear()
                current_G_E = self.apply_smoothed_estimate(N)
                
                return current_G_E, [float(self.current_round_id), float(current_G_E), 1.0] + self.x

            if valid_vectors:
                if x_changed: self.propagation_hops = 0
                else: self.propagation_hops += 1

            # Check network stability and increment consensus round when stable
            stable = False
            if len(self.N_history) == self.stability_window:
                stable = all(abs(self.N_history[i] - self.N_history[i-1]) < epsilon for i in range(1, len(self.N_history)))

            if stable and self.propagation_hops >= self.required_propagation_hops:
                current_G_E = self.apply_smoothed_estimate(N)
                self.has_started_convergence = False
                self.current_round_id += 1.0
                self.x.clear()

            if not self.x: self.initialize_vector()
            return current_G_E, [float(self.current_round_id), float(current_G_E), 1.0] + self.x
        
        return current_G_E, []