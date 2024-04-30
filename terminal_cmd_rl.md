# Terminal Commands of RL

## Zero/Random Agent
``` bash
# zero agent
python scripts/zero_agent.py --task Isaac-Reach-FR5-v0 --num_envs 1
# random agent
python scripts/random_agent.py --task Isaac-Reach-FR5-v0 --num_envs 1
```

## Train
``` bash
cd ~/orbit.fr
# from scratch
python scripts/rsl_rl/train.py --task Isaac-Reach-FR5-v0 --num_envs 64 --headless
# from checkpoint
python scripts/rsl_rl/train.py --task Isaac-Reach-FR5-v0 --resume True --load_run /path/to/run --checkpoint xxx.pt --num_envs 64 --headless
```

## Evaluate
``` bash
cd ~/orbit.fr
python scripts/rsl_rl/play.py --task Isaac-Reach-FR5-Play-v0 --load_run /path/to/run --checkpoint xxx.pt
```