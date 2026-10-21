"""Fine-tune SmolVLA on JetArm demonstrations (custom recorder format) on Modal.

The host recorder writes:
  <custom>/meta.json
  <custom>/episode_XXXX/{states.npy, actions.npy, times.npy, images/*.jpg}

This app converts that to a LeRobotDataset on the GPU host, then fine-tunes
``lerobot/smolvla_base`` (expert-only, vision frozen: fits a T4).

Upload real data (from the host):
  modal volume put jetarm-data ./datasets/<run> custom/<name> --profile=tobiwarwager
Train:
  modal run --profile=tobiwarwager modal/train_smolvla.py --name <name> --steps 20000 --batch-size 4
Smoke-test the whole pipeline with synthetic data:
  modal run --profile=tobiwarwager modal/train_smolvla.py --smoke
"""

import json
from pathlib import Path

import modal

APP_NAME = "jetarm-smolvla"
DATA = modal.Volume.from_name("jetarm-data", create_if_missing=True)
HF_CACHE = modal.Volume.from_name("jetarm-hf-cache", create_if_missing=True)

image = (
    modal.Image.debian_slim(python_version="3.12")
    .apt_install("ffmpeg", "libgl1", "libglib2.0-0", "git")
    .pip_install("lerobot[smolvla,training]==0.6.1", "pillow")
)

app = modal.App(APP_NAME)


def _make_smoke_dataset(custom: Path) -> None:
    import numpy as np
    from PIL import Image

    custom.mkdir(parents=True, exist_ok=True)
    n = 20
    rng = np.random.default_rng(0)
    for e in range(2):
        ep = custom / f"episode_{e:04d}"
        (ep / "images").mkdir(parents=True, exist_ok=True)
        start = np.array([500, 500, 500, 500, 500, 700], dtype=np.float32)
        end = start + rng.integers(-40, 40, size=6).astype(np.float32)
        states = np.linspace(start, end, n, dtype=np.float32)
        actions = np.roll(states, -1, axis=0)
        np.save(ep / "states.npy", states)
        np.save(ep / "actions.npy", actions)
        np.save(ep / "times.npy", np.arange(n, dtype=np.float64) / 20.0)
        for i in range(n):
            img = np.zeros((240, 320, 3), dtype=np.uint8)
            img[..., 0] = (i * 10 + e * 40) % 256
            img[..., 1] = 90
            Image.fromarray(img).save(ep / "images" / f"{i:06d}.jpg", quality=85)
    (custom / "meta.json").write_text(
        json.dumps({"task": "smoke", "fps": 20, "image": {"size": [320, 240]}, "episodes": []})
    )


def _build_lerobot_dataset(custom_root: Path, repo_id: str, root: Path) -> dict:
    import numpy as np
    from PIL import Image
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    meta = json.loads((custom_root / "meta.json").read_text())
    task = meta.get("task", "jetarm task")
    fps = int(meta.get("fps", 20))
    episodes = sorted(custom_root.glob("episode_*"))
    if not episodes:
        raise SystemExit(f"no episodes under {custom_root}")

    first = next(iter(sorted((episodes[0] / "images").glob("*.jpg"))))
    with Image.open(first) as im:
        width, height = im.size

    features = {
        "observation.images.top": {
            "dtype": "image",
            "shape": (height, width, 3),
            "names": ["height", "width", "channels"],
        },
        "observation.state": {"dtype": "float32", "shape": (6,), "names": None},
        "action": {"dtype": "float32", "shape": (6,), "names": None},
    }

    if root.exists():
        import shutil

        shutil.rmtree(root)

    ds = LeRobotDataset.create(
        repo_id=repo_id,
        fps=fps,
        features=features,
        root=root,
        robot_type="jetarm",
        use_videos=False,
    )

    frames = 0
    for ep in episodes:
        states = np.atleast_2d(np.load(ep / "states.npy")).astype(np.float32)
        actions = np.atleast_2d(np.load(ep / "actions.npy")).astype(np.float32)
        images = sorted((ep / "images").glob("*.jpg"))
        n = min(len(states), len(actions), len(images))
        if n < min(len(states), len(actions), len(images)):
            print(f"warning: {ep.name}: truncating to {n} aligned frames")
        for i in range(n):
            with Image.open(images[i]) as im:
                img = np.asarray(im.convert("RGB"))
            ds.add_frame(
                {
                    "observation.images.top": img,
                    "observation.state": states[i],
                    "action": actions[i],
                    "task": task,
                }
            )
        ds.save_episode()
        frames += n
        print(f"converted {ep.name}: {n} frames")

    ds.finalize()
    print(f"dataset ready: {repo_id} at {root} ({len(episodes)} episodes, {frames} frames, {fps} fps)")
    return {"repo_id": repo_id, "root": str(root), "episodes": len(episodes), "frames": frames, "fps": fps}


@app.function(
    image=image,
    gpu="A10G",
    timeout=86400,
    volumes={"/data": DATA, "/root/.cache/huggingface": HF_CACHE},
    env={"HF_HOME": "/root/.cache/huggingface", "TOKENIZERS_PARALLELISM": "false"},
)
def run(name: str = "", smoke: bool = False, resume: bool = False, steps: int = 20000, batch_size: int = 4) -> dict:
    import subprocess

    if smoke:
        name = name or "smoke"
        custom = Path("/data/custom") / name
        _make_smoke_dataset(custom)
        steps, batch_size = 2, 2

    custom = Path("/data/custom") / name
    if not (custom / "meta.json").exists():
        raise SystemExit(
            f"{custom} not found. Upload it first:\n"
            f"  modal volume put jetarm-data <host_dataset_dir> custom/{name} --profile=tobiwarwager"
        )

    repo_id = f"local/jetarm_{name}"
    lroot = Path("/data/lerobot") / name
    info = _build_lerobot_dataset(custom, repo_id, lroot)
    DATA.commit()

    out = Path("/data/outputs") / name
    if smoke and out.exists():
        import shutil

        shutil.rmtree(out)
    rename_map = json.dumps({"observation.images.top": "observation.images.camera1"})

    cmd = [
        "lerobot-train",
        f"--dataset.repo_id={repo_id}",
        f"--dataset.root={lroot}",
        "--policy.path=lerobot/smolvla_base",
        "--policy.device=cuda",
        "--policy.use_amp=false",
        "--policy.empty_cameras=2",
        f"--rename_map={rename_map}",
        f"--batch_size={batch_size}",
        f"--steps={steps}",
        "--save_freq=2000",
        "--save_checkpoint=true",
        "--wandb.enable=false",
        "--policy.push_to_hub=false",
        f"--output_dir={out}",
        f"--job_name=smolvla_{name}",
    ]
    if resume:
        cmd.append("--resume=true")
    print("RUN:", " ".join(cmd), flush=True)
    subprocess.run(cmd, check=True)

    DATA.commit()
    checkpoints = sorted(out.glob("checkpoints/*/pretrained_model"))
    return {
        "name": name,
        "dataset": info,
        "output_dir": str(out),
        "checkpoints": [str(c) for c in checkpoints],
        "steps": steps,
        "batch_size": batch_size,
    }


@app.function(
    image=image,
    gpu="A10G",
    timeout=900,
    volumes={"/data": DATA, "/root/.cache/huggingface": HF_CACHE},
    env={"HF_HOME": "/root/.cache/huggingface", "TOKENIZERS_PARALLELISM": "false"},
)
def checkpoint_check(name: str = "smoke") -> dict:
    import torch
    from torch.utils.data import default_collate
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.policies.factory import make_pre_post_processors
    from lerobot.policies.smolvla import SmolVLAPolicy

    ckpt = Path("/data/outputs") / name / "checkpoints/last/pretrained_model"
    if not ckpt.exists():
        raise SystemExit(f"no checkpoint at {ckpt}")

    policy = SmolVLAPolicy.from_pretrained(str(ckpt)).to("cuda").eval()
    ds = LeRobotDataset(repo_id=f"local/jetarm_{name}", root=Path("/data/lerobot") / name)
    item = dict(ds[0])
    if "observation.images.top" in item:
        item["observation.images.camera1"] = item.pop("observation.images.top")

    batch = default_collate([item])
    pre, post = make_pre_post_processors(policy.config, str(ckpt))
    img = item["observation.images.camera1"]
    batch = pre(batch)
    with torch.no_grad():
        action = policy.select_action(batch)
    pulses = post(action)
    return {
        "checkpoint": str(ckpt),
        "image_shape": list(img.shape),
        "image_dtype": str(img.dtype),
        "image_range": [float(img.min()), float(img.max())],
        "action_shape": list(action.shape),
        "action_norm": action.squeeze(0).tolist(),
        "action_pulses": pulses.squeeze(0).tolist(),
    }


@app.local_entrypoint()
def main(name: str = "", smoke: bool = False, check_only: bool = False, resume: bool = False, steps: int = 20000, batch_size: int = 4):
    if check_only:
        print(checkpoint_check.remote(name=name or "smoke"))
        return
    print(run.remote(name=name, smoke=smoke, resume=resume, steps=steps, batch_size=batch_size))
