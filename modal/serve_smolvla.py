"""Serve a fine-tuned JetArm SmolVLA checkpoint over HTTP from Modal.

Deploy:
  modal deploy --profile=tobiwarwager modal/serve_smolvla.py

Then:
  GET  <url>/health
  POST <url>/act  {"run": "<name>", "task": "...", "state": [6 floats],
                   "image": "<base64 jpeg>"}  ->  {"action": [6 pulses]}

The container loads a run lazily from /data/outputs/<run>/checkpoints/last and
keeps it in GPU memory while requests keep arriving.
"""

import base64
import io
from pathlib import Path

import modal

APP_NAME = "jetarm-smolvla-serve"
DATA = modal.Volume.from_name("jetarm-data", create_if_missing=True)
HF_CACHE = modal.Volume.from_name("jetarm-hf-cache", create_if_missing=True)

image = (
    modal.Image.debian_slim(python_version="3.12")
    .apt_install("ffmpeg", "libgl1", "libglib2.0-0")
    .pip_install("lerobot[smolvla,training]==0.6.1", "pillow", "fastapi")
)

app = modal.App(APP_NAME)


@app.function(
    image=image,
    gpu="A10G",
    timeout=3600,
    scaledown_window=600,
    volumes={"/data": DATA, "/root/.cache/huggingface": HF_CACHE},
    env={"HF_HOME": "/root/.cache/huggingface", "TOKENIZERS_PARALLELISM": "false"},
)
@modal.asgi_app()
def web():
    import numpy as np
    import torch
    from fastapi import FastAPI, Request
    from fastapi.responses import JSONResponse
    from lerobot.policies.factory import make_pre_post_processors
    from lerobot.policies.smolvla import SmolVLAPolicy
    from PIL import Image
    from torch.utils.data import default_collate

    cache: dict[str, tuple] = {}
    outputs = Path("/data/outputs")

    def load(run: str):
        if run not in cache:
            try:
                DATA.reload()
            except Exception:
                pass
            ckpt = outputs / run / "checkpoints/last/pretrained_model"
            if not ckpt.exists():
                raise FileNotFoundError(f"no checkpoint for run '{run}' at {ckpt}")
            policy = SmolVLAPolicy.from_pretrained(str(ckpt)).to("cuda").eval()
            pre, post = make_pre_post_processors(policy.config, str(ckpt))
            cache[run] = (policy, pre, post)
            print(f"loaded policy '{run}'", flush=True)
        return cache[run]

    api = FastAPI(title="jetarm-smolvla")

    @api.get("/health")
    def health():
        runs = sorted(
            p.name for p in outputs.glob("*") if (p / "checkpoints/last/pretrained_model").exists()
        )
        return {"ok": True, "runs": runs, "loaded": sorted(cache)}

    @api.post("/act")
    async def act(req: Request):
        try:
            d = await req.json()
            run = d.get("run", "smoke")
            policy, pre, post = load(run)

            raw = base64.b64decode(d["image"])
            arr = np.asarray(Image.open(io.BytesIO(raw)).convert("RGB"))
            item = {
                "observation.images.camera1": torch.from_numpy(arr).permute(2, 0, 1).float() / 255.0,
                "observation.state": torch.tensor(d["state"], dtype=torch.float32),
                "task": d.get("task", ""),
            }
            batch = pre(default_collate([item]))
            with torch.no_grad():
                pulses = post(policy.select_action(batch))
            return {"action": pulses.squeeze(0).tolist()}
        except Exception as exc:
            return JSONResponse(status_code=400, content={"error": repr(exc)})

    return api
