import modal

app = modal.App("gpu-smoke")
image = modal.Image.debian_slim(python_version="3.11").pip_install("pynvml")


@app.function(gpu="T4", image=image, timeout=300)
def check():
    from pynvml import (
        nvmlInit,
        nvmlDeviceGetCount,
        nvmlDeviceGetHandleByIndex,
        nvmlDeviceGetMemoryInfo,
        nvmlDeviceGetName,
        nvmlSystemGetDriverVersion,
    )

    nvmlInit()
    gpus = []
    for i in range(nvmlDeviceGetCount()):
        h = nvmlDeviceGetHandleByIndex(i)
        m = nvmlDeviceGetMemoryInfo(h)
        name = nvmlDeviceGetName(h)
        if isinstance(name, bytes):
            name = name.decode()
        gpus.append(f"{name} ({m.total // 1024**2} MiB)")
    return {"driver": nvmlSystemGetDriverVersion(), "gpus": gpus}


@app.local_entrypoint()
def main():
    print(check.remote())
