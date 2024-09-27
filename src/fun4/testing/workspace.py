import numpy as np
def generate_random_target(min_val=0.03, max_val=0.53):
        x = np.random.choice([np.random.uniform(min_val, max_val), np.random.uniform(-min_val, -max_val)])
        y = np.random.choice([np.random.uniform(min_val, max_val), np.random.uniform(-min_val, -max_val)])
        z = np.random.choice([np.random.uniform(min_val, max_val), np.random.uniform(-min_val, -max_val)])
        return x, y, z
    
print(generate_random_target())