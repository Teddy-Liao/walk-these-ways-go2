import torch
import pickle
import glob

label = "gait-conditioned-agility/pretrain-go2/train"
dirs = glob.glob(f"../../runs/{label}/*")
logdir = sorted(dirs)[0]

# transfer
with open(logdir+"/parameters.pkl", 'rb') as file:
    pkl_cfg = pickle.load(file)
    # 我们通过 torch.is_tensor(v) 检查字典中的每个值 v 是否为 PyTorch 张量。
    # 如果是，我们应用 .cpu() 方法将其转移到 CPU；如果不是，我们保留原值。
    # 这样，只有真正的张量会被转移，其他类型的值（如整数、字符串等）会保持不变。
    pkl_cfg_cpu = {k: v.cpu() if torch.is_tensor(v) else v for k, v in pkl_cfg.items()}
    print("Transfer Succeed ! !")

# save transferred .pkl file
with open(logdir+"/parameters_cpu.pkl", 'wb') as file:
    pickle.dump(pkl_cfg_cpu, file)
    print("Transferred Pickle File has been saved as parameters_cpu.pkl")