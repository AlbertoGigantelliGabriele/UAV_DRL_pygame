import os
from stable_baselines3 import SAC
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import CheckpointCallback
import wandb
from wandb.integration.sb3 import WandbCallback
from environment import DEnv

run = wandb.init(
    project="drlquadro",
    sync_tensorboard=True,
    monitor_gym=True,
)

# Ottieni il percorso della directory corrente
current_dir = os.path.dirname(os.path.abspath(__file__))

# Crea la directory di log nel percorso corrente
log_dir = os.path.join(current_dir, "log_directory/")
os.makedirs(log_dir, exist_ok=True)

# Crea e avvolge l'ambiente
env = DEnv(True, False)
env = Monitor(env, log_dir)

model = SAC("MlpPolicy", env, verbose=1, tensorboard_log=log_dir)

# Percorso per salvare i modelli
model_save_dir = os.path.join(current_dir, f"models/{run.id}")
os.makedirs(model_save_dir, exist_ok=True)

# Crea il callback di checkpoint
checkpoint_callback = CheckpointCallback(
    save_freq=100000, save_path=log_dir, name_prefix="model_")

# Addestra
model.learn(
    total_timesteps=10000000,
    callback=[
        checkpoint_callback,
        WandbCallback(
            gradient_save_freq=100000,
            model_save_path=model_save_dir,
            model_save_freq=100000,
            verbose=2,
        ),
    ],
)
